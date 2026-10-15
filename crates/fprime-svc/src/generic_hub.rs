//! # GenericHub — port of `Svc::GenericHub` (passive hub)
//!
//! C++ sources: `Svc/GenericHub/GenericHub.{fpp,hpp,cpp}`,
//! `default/config/GenericHubCfg.fpp`, `Svc/GenericHub/docs/sdd.md`; the
//! upstream unit tests are ported in the `tests` module below. Analysis:
//! `docs/cpp-analysis/fpp-autocoder.md` ("GenericHub serialized-port
//! tunneling") and `docs/cpp-analysis/fw-comp.md` (serialize ports).
//!
//! A hub makes logical port connections span two deployments. A pair of
//! hubs multiplexes events, telemetry, any serializable typed port call
//! (`serialIn` → `serialOut`), buffers, and remote command dispatch /
//! response through one buffer driver:
//!
//! ```text
//! FSW --> GenericHub --> Driver ~~> Driver --> GenericHub --> FSW
//! ```
//!
//! ## Wire format (byte-exact)
//!
//! Every message given to `toBufferDriver` is
//!
//! ```text
//! [hub type: U32 BE][port index: U32 BE][payload size: FwBuffSizeType (U16) BE][payload]
//! ```
//!
//! with the payload per [`HubType`]:
//!
//! | type | payload |
//! |------|---------|
//! | `Port` (0) | the serialized call (the `serialIn` buffer's bytes) |
//! | `Buffer` (1) | the `bufferIn` buffer's bytes (copied) |
//! | `Event` (2) | `[id U32][Time 11B][severity U8][LogBuffer: U16 size + bytes]` |
//! | `Channel` (3) | `[id U32][Time 11B][TlmBuffer: U16 size + bytes]` |
//! | `CmdDisp` (4) | `[ComBuffer bytes, NO length][context U32]` |
//! | `CmdResp` (5) | `[opCode U32][cmdSeq U32][CmdResponse U8]` |
//!
//! On receive the hub validates the header (size ≥ 10, type < `Max`,
//! declared size == received payload size), then dispatches; malformed
//! messages, invalid or unconnected destination ports and payload decode
//! failures are dropped silently, and the transport buffer is always
//! returned on `fromBufferDriverReturn` — except for a `Buffer` message
//! that was delivered, which travels on to the consumer and comes back
//! through `bufferOutReturn`.
//!
//! ## Buffer ownership
//!
//! C++ passes `Fw::Buffer&` views; here every [`Buffer`] MOVES:
//!
//! - send: `allocate` → serialize → `toBufferDriver` (the driver owns it) →
//!   `toBufferDriverReturn` → `deallocate`;
//! - `bufferIn`: the data is copied into a freshly allocated transport
//!   buffer and the original moves straight back on `bufferInReturn`;
//! - receive, `Buffer` type: the transport buffer's window is advanced past
//!   the 10-byte header (C++ `fwBuffer.set(rawData, rawSize, context)`) and
//!   the buffer itself moves out on `bufferOut`; the consumer returns it on
//!   `bufferOutReturn`, and it moves back to the driver on
//!   `fromBufferDriverReturn` with the window still advanced (as C++ returns
//!   the shifted pointer — the allocator keys on the context, not the
//!   window).
//!
//! ## Deviations from C++
//!
//! - `eventIn`/`tlmIn` scratch buffers are sized for the true worst case.
//!   Upstream sizes them as `sizeof(id) + Time + [severity +] MAX_SIZE` and
//!   forgets the `FwSizeStoreType` length the `LogBuffer`/`TlmBuffer`
//!   serializes with, so an event or channel whose argument buffer holds
//!   505 or 506 bytes trips `FW_ASSERT` in C++. The bytes on the wire are
//!   unchanged; here such calls are forwarded. (`cmdDispIn` was already
//!   fixed upstream the same way.)
//! - `serialIn`/`serialOut` are [`SerializePort`]s from
//!   `fprime_comp::serial_port`; a typed output connects to `serial_in(n)`
//!   with `OutputPort::connect_serial`, and `serial_out[n]` connects to a
//!   typed input with `OutputPort::connect_typed`. Ports that cannot be
//!   serialized (return values, `Fw::Buffer`) are rejected at compile time.
//! - The C++ `FW_ASSERT(data != nullptr)` in `send_data` becomes an assert
//!   that a `bufferIn` buffer has storage (the owned-model null).

use fprime_comp::{
    BufferGetPort, BufferSendPort, CmdResponsePort, ComPort, LogPort, OutputPort, PassiveBase,
    PortRef, SerializePort, TlmPort, input_port_adapter,
};
use fprime_config::generic_hub::{
    NUM_BUFFER_INPUT_PORTS, NUM_BUFFER_OUTPUT_PORTS, NUM_SERIAL_INPUT_PORTS,
    NUM_SERIAL_OUTPUT_PORTS,
};
use fprime_config::{
    CMD_DISPATCHER_SEQUENCE_PORTS, FW_COM_BUFFER_MAX_SIZE, FW_LOG_BUFFER_MAX_SIZE,
    FW_TLM_BUFFER_MAX_SIZE, FwBuffSizeType, FwChanIdType, FwEventIdType, FwIndexType, FwOpcodeType,
    FwSignedSizeType, FwSizeStoreType, FwSizeType,
};
use fprime_fw::fpp::FppSized;
use fprime_fw::{
    Buffer, CmdResponse, ComBuffer, Endianness, ExtBuf, LengthMode, LinearBuffer, LogBuffer,
    LogSeverity, SerBuf, SerBufAny, SerializeStatus, Time, TlmBuffer, fw_assert,
};
use std::sync::Arc;

/// Size of the message header: `[type U32][port U32][size FwBuffSizeType]`
/// (C++ `sizeof(U32) + sizeof(U32) + sizeof(FwBuffSizeType)`).
pub const HEADER_SIZE: usize = size_of::<u32>() + size_of::<u32>() + size_of::<FwBuffSizeType>();

/// Type of the serialized data on the wire (C++ `GenericHub::HubType`, a
/// plain enum serialized as `U32`).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HubType {
    /// Serialized port call (`serialIn` → `serialOut`).
    Port = 0,
    /// Buffer data (`bufferIn` → `bufferOut`).
    Buffer = 1,
    /// Event (`eventIn` → `eventOut`).
    Event = 2,
    /// Telemetry channel (`tlmIn` → `tlmOut`).
    Channel = 3,
    /// Command dispatch (`cmdDispIn` → `cmdDispOut`).
    CmdDisp = 4,
    /// Command response (`cmdRespIn` → `cmdRespOut`).
    CmdResp = 5,
    /// One past the last valid type (`HUB_TYPE_MAX`); never sent.
    Max = 6,
}

impl HubType {
    /// The type for a wire value, `None` for `>= Max` (C++ `type_in >=
    /// HUB_TYPE_MAX` → `FW_DESERIALIZE_INVALID_DATA`).
    pub fn from_wire(value: u32) -> Option<Self> {
        match value {
            0 => Some(Self::Port),
            1 => Some(Self::Buffer),
            2 => Some(Self::Event),
            3 => Some(Self::Channel),
            4 => Some(Self::CmdDisp),
            5 => Some(Self::CmdResp),
            _ => None,
        }
    }
}

/// Worst-case `Event` payload: id + `Time` + severity + `LogBuffer` with its
/// length prefix (see the module docs for the upstream undersizing).
const EVENT_PAYLOAD_MAX: usize = size_of::<FwEventIdType>()
    + Time::SERIALIZED_SIZE
    + <LogSeverity as FppSized>::SERIALIZED_SIZE
    + size_of::<FwSizeStoreType>()
    + FW_LOG_BUFFER_MAX_SIZE;

/// Worst-case `Channel` payload: id + `Time` + `TlmBuffer` with its length
/// prefix.
const CHANNEL_PAYLOAD_MAX: usize = size_of::<FwChanIdType>()
    + Time::SERIALIZED_SIZE
    + size_of::<FwSizeStoreType>()
    + FW_TLM_BUFFER_MAX_SIZE;

/// `CmdDisp` payload: a maximum-size command (no length token) + context
/// (C++ `FW_COM_BUFFER_MAX_SIZE + sizeof(U32)`).
const CMD_DISP_PAYLOAD_MAX: usize = FW_COM_BUFFER_MAX_SIZE + size_of::<u32>();

/// `CmdResp` payload: opcode + sequence + response (C++ `sizeof(opCode) +
/// sizeof(cmdSeq) + Fw::CmdResponse::SERIALIZED_SIZE`).
const CMD_RESP_PAYLOAD_SIZE: usize =
    size_of::<FwOpcodeType>() + size_of::<u32>() + <CmdResponse as FppSized>::SERIALIZED_SIZE;

/// `Svc::GenericHub` — passive; no commands, events, telemetry or state.
pub struct GenericHub {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,

    // -- FSW → hub (send side) ----------------------------------------------
    /// `bufferInReturn` — returns each `bufferIn` buffer to its sender
    /// (matched with `bufferIn`).
    pub buffer_in_return: [OutputPort<dyn BufferSendPort>; NUM_BUFFER_INPUT_PORTS],
    /// `cmdRespOut` — command responses received from the remote hub.
    pub cmd_resp_out: [OutputPort<dyn CmdResponsePort>; CMD_DISPATCHER_SEQUENCE_PORTS],

    // -- hub → buffer driver (`Svc.BufferAllocation`,
    //    `Drv.PassiveBufferDriverClientSend`) -----------------------------------
    /// `allocate` — transport-buffer allocation.
    pub allocate: OutputPort<dyn BufferGetPort>,
    /// `deallocate` — transport-buffer deallocation.
    pub deallocate: OutputPort<dyn BufferSendPort>,
    /// `toBufferDriver` — a serialized message to the driver (the buffer
    /// moves; it comes back on `toBufferDriverReturn`).
    pub to_buffer_driver: OutputPort<dyn BufferSendPort>,

    // -- buffer driver → hub (`Drv.PassiveBufferDriverClientRecv`) -----------
    /// `fromBufferDriverReturn` — received transport buffers back to the
    /// driver.
    pub from_buffer_driver_return: OutputPort<dyn BufferSendPort>,

    // -- hub → FSW (receive side) --------------------------------------------
    /// `eventOut` — received events.
    pub event_out: OutputPort<dyn LogPort>,
    /// `tlmOut` — received telemetry.
    pub tlm_out: OutputPort<dyn TlmPort>,
    /// `serialOut` — received serialized port calls; connect each to any
    /// serializable typed input with `connect_typed`.
    pub serial_out: [OutputPort<dyn SerializePort>; NUM_SERIAL_OUTPUT_PORTS],
    /// `bufferOut` — received buffers (window = payload); matched with
    /// `bufferOutReturn`.
    pub buffer_out: [OutputPort<dyn BufferSendPort>; NUM_BUFFER_OUTPUT_PORTS],
    /// `cmdDispOut` — received commands, to the local command dispatcher.
    pub cmd_disp_out: [OutputPort<dyn ComPort>; CMD_DISPATCHER_SEQUENCE_PORTS],
}

impl GenericHub {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            buffer_in_return: std::array::from_fn(|_| OutputPort::new()),
            cmd_resp_out: std::array::from_fn(|_| OutputPort::new()),
            allocate: OutputPort::new(),
            deallocate: OutputPort::new(),
            to_buffer_driver: OutputPort::new(),
            from_buffer_driver_return: OutputPort::new(),
            event_out: OutputPort::new(),
            tlm_out: OutputPort::new(),
            serial_out: std::array::from_fn(|_| OutputPort::new()),
            buffer_out: std::array::from_fn(|_| OutputPort::new()),
            cmd_disp_out: std::array::from_fn(|_| OutputPort::new()),
        })
    }

    /// `serialIn` — SYNC `serial` input: a serialized port call to send.
    /// Connect a typed output here with `OutputPort::connect_serial`.
    pub fn serial_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SerializePort> {
        assert_port(port_num, NUM_SERIAL_INPUT_PORTS);
        PortRef::new(Arc::new(SerialInAdapter { comp: self.clone() }), port_num)
    }

    // -- Helpers --------------------------------------------------------------

    /// C++ `send_data`: allocate a transport buffer, write the header and
    /// the length-prefixed payload, and hand it to the driver.
    fn send_data(&self, hub_type: HubType, port: FwIndexType, data: &[u8]) {
        let size = data.len();
        let alloc = self.allocate.get();
        // C++ parity: static_cast<U32>(size + sizeof(U32) + sizeof(U32) +
        // sizeof(FwBuffSizeType)).
        let request = (size + HEADER_SIZE) as u32;
        let mut outgoing = alloc
            .target
            .invoke(alloc.port_num, FwSizeType::from(request));
        let written = {
            let mut serializer = outgoing.get_serializer();
            let status = serializer.serialize_u32_be(hub_type as u32);
            fw_assert!(status.is_ok(), status as i32);
            // C++ parity: static_cast<U32>(port).
            let status = serializer.serialize_u32_be(port as u32);
            fw_assert!(status.is_ok(), status as i32);
            // C++ parity: the size is cast to FwBuffSizeType before the call,
            // so both the length token and the copied byte count truncate.
            let length = usize::from(size as FwBuffSizeType);
            let status = serializer.serialize_bytes(
                &data[..length],
                LengthMode::IncludeLength,
                Endianness::Big,
            );
            fw_assert!(status.is_ok(), status as i32);
            serializer.get_size()
        };
        outgoing.set_size(written);
        let driver = self.to_buffer_driver.get();
        driver.target.invoke(driver.port_num, outgoing);
    }

    /// Return a received transport buffer to the driver.
    fn return_to_driver(&self, buffer: Buffer) {
        let driver = self.from_buffer_driver_return.get();
        driver.target.invoke(driver.port_num, buffer);
    }

    // -- Handlers (FSW → hub) -------------------------------------------------

    /// `eventIn_handler`.
    fn event_in_handler(
        &self,
        port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &mut Time,
        severity: LogSeverity,
        args: &mut LogBuffer,
    ) {
        assert_port(port_num, 1);
        let mut serializer = LinearBuffer::<EVENT_PAYLOAD_MAX>::new();
        let status = serializer.serialize_u32_be(id);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize(&*time_tag, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize(&severity, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        self.send_data(HubType::Event, port_num, serializer.as_slice());
    }

    /// `tlmIn_handler`.
    fn tlm_in_handler(
        &self,
        port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) {
        assert_port(port_num, 1);
        let mut serializer = LinearBuffer::<CHANNEL_PAYLOAD_MAX>::new();
        let status = serializer.serialize_u32_be(id);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize(&*time_tag, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize_buffer(val, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        self.send_data(HubType::Channel, port_num, serializer.as_slice());
    }

    /// `serialIn_handler`: the whole serialized call (C++ `getBuffAddr()`,
    /// `getSize()` — independent of the read cursor).
    fn serial_in_handler(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) {
        assert_port(port_num, NUM_SERIAL_INPUT_PORTS);
        self.send_data(HubType::Port, port_num, buffer.as_slice());
    }

    /// `bufferIn_handler`: copy the data out, return the buffer at once.
    fn buffer_in_handler(&self, port_num: FwIndexType, buffer: Buffer) {
        assert_port(port_num, NUM_BUFFER_INPUT_PORTS);
        // C++ parity: send_data asserts data != nullptr.
        fw_assert!(buffer.capacity() != 0);
        self.send_data(HubType::Buffer, port_num, buffer.data());
        let back = self.buffer_in_return[port_num as usize].get();
        back.target.invoke(back.port_num, buffer);
    }

    /// `cmdDispIn_handler`: the command bytes WITHOUT a length token, then
    /// the context.
    fn cmd_disp_in_handler(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        assert_port(port_num, CMD_DISPATCHER_SEQUENCE_PORTS);
        let mut serializer = LinearBuffer::<CMD_DISP_PAYLOAD_MAX>::new();
        let status =
            serializer.serialize_bytes(data.as_slice(), LengthMode::OmitLength, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        self.send_data(HubType::CmdDisp, port_num, serializer.as_slice());
    }

    /// `cmdRespIn_handler`.
    fn cmd_resp_in_handler(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        assert_port(port_num, CMD_DISPATCHER_SEQUENCE_PORTS);
        let mut serializer = LinearBuffer::<CMD_RESP_PAYLOAD_SIZE>::new();
        let status = serializer.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = serializer.serialize(&response, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        self.send_data(HubType::CmdResp, port_num, serializer.as_slice());
    }

    // -- Handlers (driver ↔ hub) ----------------------------------------------

    /// `toBufferDriverReturn_handler`: the driver is done with a sent
    /// buffer; deallocate it.
    fn to_buffer_driver_return_handler(&self, port_num: FwIndexType, buffer: Buffer) {
        assert_port(port_num, 1);
        let dealloc = self.deallocate.get();
        dealloc.target.invoke(dealloc.port_num, buffer);
    }

    /// `bufferOutReturn_handler`: a consumer is done with a received
    /// buffer; give it back to the driver.
    fn buffer_out_return_handler(&self, port_num: FwIndexType, buffer: Buffer) {
        assert_port(port_num, NUM_BUFFER_OUTPUT_PORTS);
        self.return_to_driver(buffer);
    }

    /// `fromBufferDriver_handler`: validate, unpack and dispatch one
    /// message.
    // The name is the FPP port's (`fromBufferDriver`), not a conversion.
    #[allow(clippy::wrong_self_convention)]
    fn from_buffer_driver_handler(&self, port_num: FwIndexType, mut buffer: Buffer) {
        assert_port(port_num, 1);
        let mut hub_type = HubType::Max;
        let mut type_in: u32 = 0;
        let mut port: u32 = 0;
        let mut size: FwBuffSizeType = 0;
        let mut status;
        let too_short = buffer.size() < HEADER_SIZE;
        {
            let mut incoming = buffer.get_deserializer();
            // Check the size of the data first.
            if too_short {
                status = SerializeStatus::DeserSizeMismatch;
            } else {
                status = incoming.deserialize_u32_be(&mut type_in);
            }
            if status.is_ok() {
                match HubType::from_wire(type_in) {
                    None => status = SerializeStatus::DeserInvalidData,
                    Some(t) => {
                        hub_type = t;
                        status = incoming.deserialize_u32_be(&mut port);
                    }
                }
            }
            if status.is_ok() {
                status = incoming.deserialize_u16_be(&mut size);
            }
        }
        // The declared payload size must match what arrived.
        if !status.is_ok() || usize::from(size) != buffer.size() - HEADER_SIZE {
            // On deserialization failure, return the buffer to avoid leaks.
            self.return_to_driver(buffer);
            return;
        }
        let raw_size = buffer.size() - HEADER_SIZE;
        match hub_type {
            HubType::Port => {
                {
                    // Serial calls are deserialized (copied) before the
                    // call returns, so the transport storage is wrapped in
                    // place.
                    let payload = &mut buffer.data_mut()[HEADER_SIZE..];
                    let mut wrapper = ExtBuf::with_len(payload, raw_size);
                    if let Some(out) = connected(&self.serial_out, port) {
                        // C++ parity: the serialOut_out status is discarded.
                        let _ = out.target.invoke_serial(out.port_num, &mut wrapper);
                    }
                }
                self.return_to_driver(buffer);
            }
            HubType::Buffer => {
                // The transport buffer IS the delivered buffer: point its
                // window at the payload (C++ fwBuffer.set(rawData, rawSize,
                // context)).
                buffer.advance(HEADER_SIZE as FwSignedSizeType);
                match connected(&self.buffer_out, port) {
                    Some(out) => out.target.invoke(out.port_num, buffer),
                    // Invalid or unconnected port: return it to avoid leaks.
                    None => self.return_to_driver(buffer),
                }
            }
            HubType::Event => {
                {
                    let mut id: FwEventIdType = 0;
                    let mut time_tag = Time::default();
                    let mut severity = LogSeverity::default();
                    let mut args = LogBuffer::new();
                    let mut payload =
                        ExtBuf::with_len(&mut buffer.data_mut()[HEADER_SIZE..], raw_size);
                    let mut status = payload.deserialize_u32_be(&mut id);
                    if status.is_ok() {
                        status = payload.deserialize(&mut time_tag, Endianness::Big);
                    }
                    if status.is_ok() {
                        status = payload.deserialize(&mut severity, Endianness::Big);
                    }
                    if status.is_ok() {
                        status = payload.deserialize_buffer(&mut args, Endianness::Big);
                    }
                    if status.is_ok() && port == 0 {
                        if let Some(out) = self.event_out.try_get() {
                            out.target
                                .invoke(out.port_num, id, &mut time_tag, severity, &mut args);
                        }
                    }
                }
                self.return_to_driver(buffer);
            }
            HubType::Channel => {
                {
                    let mut id: FwChanIdType = 0;
                    let mut time_tag = Time::default();
                    let mut val = TlmBuffer::new();
                    let mut payload =
                        ExtBuf::with_len(&mut buffer.data_mut()[HEADER_SIZE..], raw_size);
                    let mut status = payload.deserialize_u32_be(&mut id);
                    if status.is_ok() {
                        status = payload.deserialize(&mut time_tag, Endianness::Big);
                    }
                    if status.is_ok() {
                        status = payload.deserialize_buffer(&mut val, Endianness::Big);
                    }
                    if status.is_ok() && port == 0 {
                        if let Some(out) = self.tlm_out.try_get() {
                            out.target.invoke(out.port_num, id, &mut time_tag, &mut val);
                        }
                    }
                }
                self.return_to_driver(buffer);
            }
            HubType::CmdDisp => {
                // The wrapped command must fit the ComBuffer capacity
                // (upstream compares against the capacity, not
                // ComBuffer::SERIALIZED_SIZE, which includes the length).
                let context_size = size_of::<u32>();
                if raw_size >= context_size && raw_size - context_size <= FW_COM_BUFFER_MAX_SIZE {
                    let cmd_size = raw_size - context_size;
                    let mut payload =
                        ExtBuf::with_len(&mut buffer.data_mut()[HEADER_SIZE..], raw_size);
                    let mut wrapper = ComBuffer::new();
                    let status = wrapper.set_buff(&payload.remaining_slice()[..cmd_size]);
                    fw_assert!(status.is_ok(), status as i32);
                    // Skip the command bytes already wrapped.
                    let status = payload.deserialize_skip(cmd_size);
                    fw_assert!(status.is_ok(), status as i32);
                    let mut context: u32 = 0;
                    let status = payload.deserialize_u32_be(&mut context);
                    if status.is_ok() {
                        if let Some(out) = connected(&self.cmd_disp_out, port) {
                            out.target.invoke(out.port_num, &mut wrapper, context);
                        }
                    }
                }
                self.return_to_driver(buffer);
            }
            HubType::CmdResp => {
                {
                    let mut op_code: FwOpcodeType = 0;
                    let mut cmd_seq: u32 = 0;
                    let mut response = CmdResponse::default();
                    let mut payload =
                        ExtBuf::with_len(&mut buffer.data_mut()[HEADER_SIZE..], raw_size);
                    let mut status = payload.deserialize_u32_be(&mut op_code);
                    if status.is_ok() {
                        status = payload.deserialize_u32_be(&mut cmd_seq);
                    }
                    if status.is_ok() {
                        status = payload.deserialize(&mut response, Endianness::Big);
                    }
                    if status.is_ok() {
                        if let Some(out) = connected(&self.cmd_resp_out, port) {
                            out.target.invoke(out.port_num, op_code, cmd_seq, response);
                        }
                    }
                }
                self.return_to_driver(buffer);
            }
            // Excluded by HubType::from_wire.
            HubType::Max => self.return_to_driver(buffer),
        }
    }
}

/// C++ handlerBase parity: `FW_ASSERT(portNum < getNum_<port>_InputPorts())`.
fn assert_port(port_num: FwIndexType, count: usize) {
    fw_assert!(
        port_num >= 0 && (port_num as usize) < count,
        port_num,
        count as i32
    );
}

/// The connection of `ports[port]` when the wire port index is in range and
/// connected (C++ `port < getNum_X_OutputPorts() && isConnected_X_OutputPort`).
fn connected<P: ?Sized>(ports: &[OutputPort<P>], port: u32) -> Option<&PortRef<P>> {
    ports.get(port as usize).and_then(OutputPort::try_get)
}

/// `serialIn` adapter: hand-written because the serial port's method is
/// `invoke_serial`. Always reports `Ok`, as C++
/// `InputSerializePort::invokeSerial` does.
struct SerialInAdapter {
    comp: Arc<GenericHub>,
}

impl SerializePort for SerialInAdapter {
    fn invoke_serial(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) -> SerializeStatus {
        self.comp.serial_in_handler(port_num, buffer);
        SerializeStatus::Ok
    }
}

input_port_adapter! {
    /// `eventIn` — SYNC `Fw.Log` input: an event to send.
    component: GenericHub;
    adapter: EventInAdapter;
    port: LogPort;
    input: pub event_in;
    handler: event_in_handler;
    args {
        val id: FwEventIdType,
        mut time_tag: Time,
        val severity: LogSeverity,
        buf args: LogBuffer,
    }
}

input_port_adapter! {
    /// `tlmIn` — SYNC `Fw.Tlm` input: a telemetry value to send.
    component: GenericHub;
    adapter: TlmInAdapter;
    port: TlmPort;
    input: pub tlm_in;
    handler: tlm_in_handler;
    args { val id: FwChanIdType, mut time_tag: Time, buf val: TlmBuffer }
}

input_port_adapter! {
    /// `bufferIn` — SYNC `Fw.BufferSend` input: buffer data to send; the
    /// buffer moves back on `bufferInReturn` at once.
    component: GenericHub;
    adapter: BufferInAdapter;
    port: BufferSendPort;
    input: pub buffer_in;
    handler: buffer_in_handler;
    args { val buffer: Buffer }
}

input_port_adapter! {
    /// `cmdDispIn` — SYNC `Fw.Com` input: a command for the remote
    /// dispatcher (e.g. from a command splitter).
    component: GenericHub;
    adapter: CmdDispInAdapter;
    port: ComPort;
    input: pub cmd_disp_in;
    handler: cmd_disp_in_handler;
    args { buf data: ComBuffer, val context: u32 }
}

input_port_adapter! {
    /// `cmdRespIn` — SYNC `Fw.CmdResponse` input: a local dispatcher's
    /// response to a remote command.
    component: GenericHub;
    adapter: CmdRespInAdapter;
    port: CmdResponsePort;
    input: pub cmd_resp_in;
    handler: cmd_resp_in_handler;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, val response: CmdResponse }
}

input_port_adapter! {
    /// `fromBufferDriver` — SYNC `Fw.BufferSend` input: a received message.
    component: GenericHub;
    adapter: FromBufferDriverAdapter;
    port: BufferSendPort;
    input: pub from_buffer_driver;
    handler: from_buffer_driver_handler;
    args { val buffer: Buffer }
}

input_port_adapter! {
    /// `toBufferDriverReturn` — SYNC `Fw.BufferSend` input: a sent buffer
    /// back from the driver.
    component: GenericHub;
    adapter: ToBufferDriverReturnAdapter;
    port: BufferSendPort;
    input: pub to_buffer_driver_return;
    handler: to_buffer_driver_return_handler;
    args { val buffer: Buffer }
}

input_port_adapter! {
    /// `bufferOutReturn` — SYNC `Fw.BufferSend` input: a `bufferOut` buffer
    /// back from its consumer (matched with `bufferOut`).
    component: GenericHub;
    adapter: BufferOutReturnAdapter;
    port: BufferSendPort;
    input: pub buffer_out_return;
    handler: buffer_out_return_handler;
    args { val buffer: Buffer }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/GenericHub/test/ut` (GenericHubTester + TestMain): two
    //! hubs back to back, `componentIn` sending and `componentOut`
    //! receiving, with the tester standing in for the buffer driver (it
    //! forwards `toBufferDriver` straight into `fromBufferDriver`) and the
    //! buffer manager (one transport storage of `DATA_SIZE` bytes, so a
    //! buffer that is not returned fails the next allocation). Followed by
    //! literal-byte wire tests and the Rust-only checks.

    use super::*;
    use fprime_comp::{PingPort, SerializablePort};
    use fprime_fw::{BufferStorage, TimeBase};
    use std::sync::Mutex;

    /// Larger than a com buffer (upstream `DATA_SIZE`).
    const DATA_SIZE: usize = FW_COM_BUFFER_MAX_SIZE * 10 + HEADER_SIZE;

    /// xorshift64: deterministic stand-in for `STest::Pick`.
    struct Pick(u64);

    impl Pick {
        fn any(&mut self) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 32) as u32
        }

        /// Inclusive on both ends, like `STest::Pick::lowerUpper`.
        fn lower_upper(&mut self, lower: u32, upper: u32) -> u32 {
            lower + self.any() % (upper - lower + 1)
        }
    }

    #[derive(Default)]
    struct State {
        rng_seed: u64,
        // Counters (upstream m_comm_in / m_buffer_in / m_comm_out / m_buffer_out).
        comm_in: usize,
        buffer_in: usize,
        comm_out: usize,
        buffer_out: usize,
        current_port: FwIndexType,
        /// The serial call last sent (upstream m_comm).
        comm: ComBuffer,
        /// Window bytes of the buffer last sent on bufferIn (upstream m_buffer).
        expected_buffer: Vec<u8>,
        // Port histories.
        to_buffer_driver: Vec<Vec<u8>>,
        from_buffer_driver_return: usize,
        buffer_in_return: Vec<(FwIndexType, usize)>,
        event_out: Vec<(FwEventIdType, Time, LogSeverity, LogBuffer)>,
        tlm_out: Vec<(FwChanIdType, Time, TlmBuffer)>,
        cmd_disp_out: Vec<(FwIndexType, ComBuffer, u32)>,
        cmd_resp_out: Vec<(FwIndexType, FwOpcodeType, u32, CmdResponse)>,
        allocations: Vec<FwSizeType>,
    }

    struct Tester {
        component_in: Arc<GenericHub>,
        component_out: Arc<GenericHub>,
        state: Mutex<State>,
        /// The single transport storage (upstream m_data_for_allocation);
        /// `None` while a transport buffer is out.
        allocation: Mutex<Option<BufferStorage>>,
        /// The bufferIn storage (upstream m_data_store); `None` while the
        /// hub holds it.
        data_store: Mutex<Option<BufferStorage>>,
    }

    // The tester keeps the C++ tester's `from_<port>` handler names.
    #[allow(clippy::wrong_self_convention)]
    impl Tester {
        fn new() -> Arc<Self> {
            let tester = Arc::new(Self {
                component_in: GenericHub::new("GenericHubIn"),
                component_out: GenericHub::new("GenericHubOut"),
                state: Mutex::new(State {
                    rng_seed: 0x9E37_79B9_7F4A_7C15,
                    ..State::default()
                }),
                allocation: Mutex::new(Some(vec![0u8; DATA_SIZE].into_boxed_slice())),
                data_store: Mutex::new(Some(vec![0u8; DATA_SIZE].into_boxed_slice())),
            });
            tester.connect_ports();
            tester
        }

        /// Upstream `connectPorts`.
        fn connect_ports(self: &Arc<Self>) {
            let hub_in = &self.component_in;
            let hub_out = &self.component_out;
            for i in 0..NUM_BUFFER_OUTPUT_PORTS.min(NUM_BUFFER_INPUT_PORTS) {
                hub_out.buffer_out[i].connect_to(self.from_buffer_out(i as FwIndexType));
            }
            hub_out.event_out.connect_to(self.from_event_out(0));
            hub_out.tlm_out.connect_to(self.from_tlm_out(0));
            for i in 0..CMD_DISPATCHER_SEQUENCE_PORTS {
                hub_out.cmd_disp_out[i].connect_to(self.from_cmd_disp_out(i as FwIndexType));
                hub_out.cmd_resp_out[i].connect_to(self.from_cmd_resp_out(i as FwIndexType));
            }
            hub_in
                .to_buffer_driver
                .connect_to(self.from_to_buffer_driver(0));
            hub_in.allocate.connect_to(self.from_allocate(0));
            hub_out
                .from_buffer_driver_return
                .connect_to(self.from_from_buffer_driver_return(0));
            hub_in.deallocate.connect_to(self.from_deallocate(0));
            for i in 0..NUM_BUFFER_INPUT_PORTS {
                hub_in.buffer_in_return[i].connect_to(self.from_buffer_in_return(i as FwIndexType));
            }
            for i in 0..NUM_SERIAL_INPUT_PORTS.min(NUM_SERIAL_OUTPUT_PORTS) {
                hub_out.serial_out[i].connect(
                    Arc::new(SerialOutSink {
                        tester: self.clone(),
                    }),
                    i as FwIndexType,
                );
            }
        }

        fn rng(&self) -> Pick {
            Pick(self.state.lock().unwrap().rng_seed)
        }

        fn save_rng(&self, pick: &Pick) {
            self.state.lock().unwrap().rng_seed = pick.0;
        }

        /// Upstream `random_fill`: reset, then a random number (0..=max) of
        /// random bytes.
        fn random_fill(&self, buffer: &mut dyn SerBufAny, max_size: usize) {
            let mut pick = self.rng();
            let random_size = pick.lower_upper(0, max_size as u32);
            buffer.reset_ser();
            for _ in 0..random_size {
                let _ = buffer.serialize_u8_be(pick.any() as u8);
            }
            self.save_rng(&pick);
        }

        fn clear_history(&self) {
            let mut s = self.state.lock().unwrap();
            s.from_buffer_driver_return = 0;
            s.buffer_in_return.clear();
            s.event_out.clear();
            s.tlm_out.clear();
            s.cmd_disp_out.clear();
            s.cmd_resp_out.clear();
            s.allocations.clear();
            s.to_buffer_driver.clear();
        }

        fn returns(&self) -> usize {
            self.state.lock().unwrap().from_buffer_driver_return
        }

        // -- Upstream helpers -------------------------------------------------

        fn send_random_comm(&self, port: FwIndexType) {
            let mut comm = ComBuffer::new();
            self.random_fill(&mut comm, FW_COM_BUFFER_MAX_SIZE);
            {
                let mut s = self.state.lock().unwrap();
                s.comm = comm.clone();
                s.current_port = port;
            }
            let p = self.component_in.serial_in(port);
            assert_eq!(
                p.target.invoke_serial(p.port_num, &mut comm),
                SerializeStatus::Ok
            );
            // The data went out, and serialOut unwrapped it.
            let mut s = self.state.lock().unwrap();
            assert_eq!(s.to_buffer_driver.len(), s.comm_in + s.buffer_out + 1);
            assert_eq!(s.comm_in + 1, s.comm_out);
            s.comm_in += 1;
        }

        fn send_random_buffer(&self, port: FwIndexType) {
            let mut pick = self.rng();
            let max_random_size = pick.lower_upper(0, (DATA_SIZE - HEADER_SIZE) as u32) as usize;
            self.save_rng(&pick);
            let storage = self.data_store.lock().unwrap().take();
            let mut buffer = Buffer::from_storage(storage.expect("data store in use"), 0);
            assert!(buffer.size() >= max_random_size);
            {
                let mut serializer = buffer.get_serializer();
                self.random_fill(&mut serializer, max_random_size);
            }
            buffer.set_size(max_random_size);
            {
                let mut s = self.state.lock().unwrap();
                s.expected_buffer = buffer.data().to_vec();
                s.current_port = port;
            }
            let p = self.component_in.buffer_in(port);
            p.target.invoke(p.port_num, buffer);
            let mut s = self.state.lock().unwrap();
            // The very buffer came back, on the same port.
            assert_eq!(s.buffer_in_return, vec![(port, max_random_size)]);
            s.buffer_in_return.clear();
            assert_eq!(s.to_buffer_driver.len(), s.buffer_in + s.comm_out + 1);
            assert_eq!(s.buffer_in + 1, s.buffer_out);
            s.buffer_in += 1;
        }

        /// Upstream `send_from_driver_packet`: hand-build a message in the
        /// transport storage and inject it on `fromBufferDriver`.
        fn send_from_driver_packet(
            &self,
            hub_type: u32,
            port: u32,
            declared_size: FwBuffSizeType,
            payload: &[u8],
        ) {
            let storage = self.allocation.lock().unwrap().take();
            let mut buffer = Buffer::from_storage(storage.expect("allocation in use"), 0);
            let written = {
                let mut serializer = buffer.get_serializer();
                assert!(serializer.serialize_u32_be(hub_type).is_ok());
                assert!(serializer.serialize_u32_be(port).is_ok());
                assert!(serializer.serialize_u16_be(declared_size).is_ok());
                assert!(
                    serializer
                        .serialize_bytes(payload, LengthMode::OmitLength, Endianness::Big)
                        .is_ok()
                );
                serializer.get_size()
            };
            buffer.set_size(written);
            let p = self.component_out.from_buffer_driver(0);
            p.target.invoke(p.port_num, buffer);
        }

        fn assert_no_outputs_sent(&self) {
            let s = self.state.lock().unwrap();
            assert_eq!(s.comm_out, 0);
            assert_eq!(s.buffer_out, 0);
            assert!(s.event_out.is_empty());
            assert!(s.tlm_out.is_empty());
            assert!(s.cmd_disp_out.is_empty());
            assert!(s.cmd_resp_out.is_empty());
        }

        /// Both storages are home: nothing leaked, nothing still in flight.
        fn assert_all_returned(&self) {
            assert!(self.allocation.lock().unwrap().is_some());
            assert!(self.data_store.lock().unwrap().is_some());
        }

        // -- "from" port handlers ---------------------------------------------

        fn from_to_buffer_driver_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
            // The transport buffer is the one the allocator handed out.
            assert_eq!(buffer.capacity(), DATA_SIZE, "incorrect transport buffer");
            self.state
                .lock()
                .unwrap()
                .to_buffer_driver
                .push(buffer.data().to_vec());
            // Back to back: the driver delivers straight to the other hub.
            let p = self.component_out.from_buffer_driver(0);
            p.target.invoke(p.port_num, buffer);
        }

        fn from_from_buffer_driver_return_handler(&self, port_num: FwIndexType, buffer: Buffer) {
            assert_eq!(buffer.capacity(), DATA_SIZE, "incorrect buffer returned");
            self.state.lock().unwrap().from_buffer_driver_return += 1;
            let p = self.component_in.to_buffer_driver_return(port_num);
            p.target.invoke(p.port_num, buffer);
        }

        fn from_allocate_handler(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
            let storage = self.allocation.lock().unwrap().take();
            let storage = storage.expect("allocation buffer is still in use");
            assert!(
                size as usize <= DATA_SIZE,
                "allocation buffer size mismatch"
            );
            self.state.lock().unwrap().allocations.push(size);
            let mut buffer = Buffer::from_storage(storage, 0x1234);
            buffer.set_size(size as usize);
            buffer
        }

        fn from_deallocate_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
            let mut slot = self.allocation.lock().unwrap();
            assert!(slot.is_none(), "deallocated a buffer that was not out");
            *slot = Some(buffer.into_storage());
        }

        fn from_buffer_out_handler(&self, port_num: FwIndexType, buffer: Buffer) {
            {
                let mut s = self.state.lock().unwrap();
                s.buffer_out += 1;
                assert_eq!(port_num, s.current_port);
                assert_eq!(buffer.data(), &s.expected_buffer[..]);
            }
            // Delivered in place: the transport storage itself, window
            // advanced past the header.
            assert_eq!(buffer.capacity(), DATA_SIZE);
            assert_eq!(buffer.offset(), HEADER_SIZE);
            let p = self.component_out.buffer_out_return(port_num);
            p.target.invoke(p.port_num, buffer);
        }

        fn from_buffer_in_return_handler(&self, port_num: FwIndexType, buffer: Buffer) {
            // The original buffer, unchanged.
            assert_eq!(buffer.capacity(), DATA_SIZE);
            {
                let mut s = self.state.lock().unwrap();
                assert_eq!(buffer.data(), &s.expected_buffer[..]);
                s.buffer_in_return.push((port_num, buffer.size()));
            }
            *self.data_store.lock().unwrap() = Some(buffer.into_storage());
        }

        fn from_event_out_handler(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.state
                .lock()
                .unwrap()
                .event_out
                .push((id, *time_tag, severity, args.clone()));
        }

        fn from_tlm_out_handler(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.state
                .lock()
                .unwrap()
                .tlm_out
                .push((id, *time_tag, val.clone()));
        }

        fn from_cmd_disp_out_handler(
            &self,
            port_num: FwIndexType,
            data: &mut ComBuffer,
            context: u32,
        ) {
            self.state
                .lock()
                .unwrap()
                .cmd_disp_out
                .push((port_num, data.clone(), context));
        }

        fn from_cmd_resp_out_handler(
            &self,
            port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.state
                .lock()
                .unwrap()
                .cmd_resp_out
                .push((port_num, op_code, cmd_seq, response));
        }
    }

    /// `from_serialOut` (a serial input on the tester).
    struct SerialOutSink {
        tester: Arc<Tester>,
    }

    impl SerializePort for SerialOutSink {
        fn invoke_serial(
            &self,
            port_num: FwIndexType,
            buffer: &mut dyn SerBufAny,
        ) -> SerializeStatus {
            let mut s = self.tester.state.lock().unwrap();
            s.comm_out += 1;
            assert_eq!(port_num, s.current_port);
            assert_eq!(buffer.as_slice(), s.comm.as_slice());
            SerializeStatus::Ok
        }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromToBufferDriver;
        port: BufferSendPort;
        input: from_to_buffer_driver;
        handler: from_to_buffer_driver_handler;
        args { val buffer: Buffer }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromFromBufferDriverReturn;
        port: BufferSendPort;
        input: from_from_buffer_driver_return;
        handler: from_from_buffer_driver_return_handler;
        args { val buffer: Buffer }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromAllocate;
        port: BufferGetPort;
        input: from_allocate;
        handler: from_allocate_handler;
        returns: Buffer;
        args { val size: FwSizeType }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromDeallocate;
        port: BufferSendPort;
        input: from_deallocate;
        handler: from_deallocate_handler;
        args { val buffer: Buffer }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromBufferOut;
        port: BufferSendPort;
        input: from_buffer_out;
        handler: from_buffer_out_handler;
        args { val buffer: Buffer }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromBufferInReturn;
        port: BufferSendPort;
        input: from_buffer_in_return;
        handler: from_buffer_in_return_handler;
        args { val buffer: Buffer }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromEventOut;
        port: LogPort;
        input: from_event_out;
        handler: from_event_out_handler;
        args {
            val id: FwEventIdType,
            mut time_tag: Time,
            val severity: LogSeverity,
            buf args: LogBuffer,
        }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromTlmOut;
        port: TlmPort;
        input: from_tlm_out;
        handler: from_tlm_out_handler;
        args { val id: FwChanIdType, mut time_tag: Time, buf val: TlmBuffer }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromCmdDispOut;
        port: ComPort;
        input: from_cmd_disp_out;
        handler: from_cmd_disp_out_handler;
        args { buf data: ComBuffer, val context: u32 }
    }

    input_port_adapter! {
        component: Tester;
        adapter: FromCmdRespOut;
        port: CmdResponsePort;
        input: from_cmd_resp_out;
        handler: from_cmd_resp_out_handler;
        args { val op_code: FwOpcodeType, val cmd_seq: u32, val response: CmdResponse }
    }

    // -----------------------------------------------------------------------
    // Upstream tests (GenericHubTestMain.cpp)
    // -----------------------------------------------------------------------

    /// `Nominal.TestIo` — `test_in_out`.
    #[test]
    fn nominal_test_io() {
        let t = Tester::new();
        for i in 0..NUM_SERIAL_INPUT_PORTS.min(NUM_SERIAL_OUTPUT_PORTS) {
            t.send_random_comm(i as FwIndexType);
            assert_eq!(t.returns(), 1);
            t.state.lock().unwrap().from_buffer_driver_return = 0;
        }
        t.assert_all_returned();
    }

    /// `Nominal.TestBufferIo` — `test_buffer_io`.
    #[test]
    fn nominal_test_buffer_io() {
        let t = Tester::new();
        for i in 0..NUM_BUFFER_INPUT_PORTS.min(NUM_BUFFER_OUTPUT_PORTS) {
            t.send_random_buffer(i as FwIndexType);
            assert_eq!(t.returns(), 1);
            t.state.lock().unwrap().from_buffer_driver_return = 0;
        }
        t.assert_all_returned();
    }

    /// `Nominal.TestRandomIo` — `test_random_io`.
    #[test]
    fn nominal_test_random_io() {
        let t = Tester::new();
        let mut pick = Pick(0x0123_4567_89AB_CDEF);
        for _ in 0..10_000 {
            if pick.lower_upper(0, 1) == 1 {
                let max = NUM_SERIAL_INPUT_PORTS.min(NUM_SERIAL_OUTPUT_PORTS) as u32;
                t.send_random_comm(pick.lower_upper(0, max - 1) as FwIndexType);
            } else {
                let max = NUM_BUFFER_INPUT_PORTS.min(NUM_BUFFER_OUTPUT_PORTS) as u32;
                t.send_random_buffer(pick.lower_upper(0, max - 1) as FwIndexType);
            }
            assert_eq!(t.returns(), 1);
            t.state.lock().unwrap().from_buffer_driver_return = 0;
        }
        t.assert_all_returned();
    }

    /// `Nominal.TestEvents` — `test_events`.
    #[test]
    fn nominal_test_events() {
        let t = Tester::new();
        let severity = LogSeverity::WarningHi;
        let mut buffer = LogBuffer::new();
        t.random_fill(&mut buffer, FW_LOG_BUFFER_MAX_SIZE);
        let mut time = Time::from_seconds_useconds(100, 200);
        let p = t.component_in.event_in(0);
        p.target
            .invoke(p.port_num, 123, &mut time, severity, &mut buffer.clone());
        // **must** return the transport buffer
        assert_eq!(t.returns(), 1);
        let s = t.state.lock().unwrap();
        assert_eq!(s.event_out, vec![(123, time, severity, buffer)]);
        drop(s);
        t.assert_all_returned();
    }

    /// `Nominal.TestTelemetry` — `test_telemetry`.
    #[test]
    fn nominal_test_telemetry() {
        let t = Tester::new();
        let mut buffer = TlmBuffer::new();
        t.random_fill(&mut buffer, FW_TLM_BUFFER_MAX_SIZE);
        let mut time = Time::from_seconds_useconds(100, 200);
        let p = t.component_in.tlm_in(0);
        p.target
            .invoke(p.port_num, 123, &mut time, &mut buffer.clone());
        assert_eq!(t.returns(), 1);
        let s = t.state.lock().unwrap();
        assert_eq!(s.tlm_out, vec![(123, time, buffer)]);
        drop(s);
        t.assert_all_returned();
    }

    /// `Nominal.TestCommands` — `test_command_dispatch` +
    /// `test_command_response`.
    #[test]
    fn nominal_test_commands() {
        let t = Tester::new();
        // test_command_dispatch
        let mut buffer = ComBuffer::new();
        t.random_fill(&mut buffer, FW_TLM_BUFFER_MAX_SIZE);
        let p = t.component_in.cmd_disp_in(0);
        p.target.invoke(p.port_num, &mut buffer.clone(), 279);
        assert_eq!(t.returns(), 1);
        assert_eq!(
            t.state.lock().unwrap().cmd_disp_out,
            vec![(0, buffer.clone(), 279)]
        );
        t.clear_history();

        // A maximum-size command payload must round-trip without overflow.
        let mut pick = Pick(7);
        buffer.reset_ser();
        for _ in 0..FW_COM_BUFFER_MAX_SIZE {
            assert_eq!(
                buffer.serialize_u8_be(pick.any() as u8),
                SerializeStatus::Ok
            );
        }
        p.target.invoke(p.port_num, &mut buffer.clone(), 280);
        assert_eq!(t.returns(), 1);
        assert_eq!(t.state.lock().unwrap().cmd_disp_out, vec![(0, buffer, 280)]);
        t.clear_history();

        // test_command_response
        let p = t.component_in.cmd_resp_in(0);
        p.target
            .invoke(p.port_num, 0x3A56_BF8C, 825, CmdResponse::ValidationError);
        assert_eq!(
            t.state.lock().unwrap().cmd_resp_out,
            vec![(0, 0x3A56_BF8C, 825, CmdResponse::ValidationError)]
        );
        t.assert_all_returned();
    }

    /// `Nominal.TestCommandsNonZeroPort` — `test_commands_nonzero_port`:
    /// the port index travels with the message.
    #[test]
    fn nominal_test_commands_nonzero_port() {
        let t = Tester::new();
        let port: FwIndexType = 1;
        let mut buffer = ComBuffer::new();
        t.random_fill(&mut buffer, FW_TLM_BUFFER_MAX_SIZE);
        let p = t.component_in.cmd_disp_in(port);
        p.target.invoke(p.port_num, &mut buffer.clone(), 279);
        assert_eq!(t.returns(), 1);
        assert_eq!(
            t.state.lock().unwrap().cmd_disp_out,
            vec![(port, buffer, 279)]
        );
        t.clear_history();

        let p = t.component_in.cmd_resp_in(port);
        p.target
            .invoke(p.port_num, 0x3A56_BF8C, 825, CmdResponse::ValidationError);
        assert_eq!(
            t.state.lock().unwrap().cmd_resp_out,
            vec![(port, 0x3A56_BF8C, 825, CmdResponse::ValidationError)]
        );
        t.assert_all_returned();
    }

    /// `Invalid.TestDeserializationGuards` —
    /// `test_invalid_deserialization_paths`: every malformed message is
    /// dropped and its buffer returned.
    #[test]
    fn invalid_test_deserialization_guards() {
        let t = Tester::new();

        // Header too short to deserialize.
        let storage = t.allocation.lock().unwrap().take().unwrap();
        let mut short = Buffer::from_storage(storage, 0);
        short.set_size(HEADER_SIZE - 1);
        let p = t.component_out.from_buffer_driver(0);
        p.target.invoke(p.port_num, short);
        assert_eq!(t.returns(), 1);
        t.assert_no_outputs_sent();

        // Invalid type is dropped.
        t.clear_history();
        t.send_from_driver_packet(HubType::Max as u32, 0, 0, &[]);
        assert_eq!(t.returns(), 1);
        t.assert_no_outputs_sent();

        // Declared payload size mismatch is dropped.
        t.clear_history();
        t.send_from_driver_packet(HubType::Port as u32, 0, 1, &[]);
        assert_eq!(t.returns(), 1);
        t.assert_no_outputs_sent();

        // Invalid serialOut destination port does not invoke output.
        t.clear_history();
        t.send_from_driver_packet(HubType::Port as u32, NUM_SERIAL_OUTPUT_PORTS as u32, 0, &[]);
        assert_eq!(t.returns(), 1);
        t.assert_no_outputs_sent();

        // Invalid bufferOut destination port returns the buffer.
        t.clear_history();
        t.send_from_driver_packet(
            HubType::Buffer as u32,
            NUM_BUFFER_OUTPUT_PORTS as u32,
            0,
            &[],
        );
        assert_eq!(t.returns(), 1);
        t.assert_no_outputs_sent();

        // Command dispatch: raw payload too small to include the context.
        t.clear_history();
        t.send_from_driver_packet(HubType::CmdDisp as u32, 0, 0, &[]);
        assert_eq!(t.returns(), 1);
        t.assert_no_outputs_sent();

        // Command dispatch: raw payload larger than a ComBuffer supports.
        t.clear_history();
        let too_large = ComBuffer::SERIALIZED_SIZE + size_of::<u32>() + 1;
        let payload = vec![0xA5; too_large];
        t.send_from_driver_packet(
            HubType::CmdDisp as u32,
            0,
            too_large as FwBuffSizeType,
            &payload,
        );
        assert_eq!(t.returns(), 1);
        t.assert_no_outputs_sent();

        t.assert_all_returned();
    }

    // -----------------------------------------------------------------------
    // Wire format: literal bytes for every message type
    // -----------------------------------------------------------------------

    fn last_sent(t: &Tester) -> Vec<u8> {
        t.state
            .lock()
            .unwrap()
            .to_buffer_driver
            .last()
            .unwrap()
            .clone()
    }

    #[test]
    fn event_message_bytes() {
        let t = Tester::new();
        let mut time = Time::new(TimeBase::TbWorkstationTime, 7, 0x0001_0203, 0x0000_0405);
        let mut args = LogBuffer::new();
        assert!(args.set_buff(&[0xAA, 0xBB, 0xCC]).is_ok());
        let p = t.component_in.event_in(0);
        p.target.invoke(
            p.port_num,
            0x0102_0304,
            &mut time,
            LogSeverity::ActivityLo,
            &mut args,
        );
        let payload_len = 4 + 11 + 1 + 2 + 3;
        assert_eq!(
            last_sent(&t),
            vec![
                0x00,
                0x00,
                0x00,
                0x02, // HUB_TYPE_EVENT
                0x00,
                0x00,
                0x00,
                0x00, // port 0
                0x00,
                payload_len as u8, // payload size
                0x01,
                0x02,
                0x03,
                0x04, // id
                0x00,
                0x02,
                0x07,
                0x00,
                0x01,
                0x02,
                0x03,
                0x00,
                0x00,
                0x04,
                0x05, // time
                0x06, // ACTIVITY_LO
                0x00,
                0x03,
                0xAA,
                0xBB,
                0xCC, // args with length
            ]
        );
        // Allocation request = payload + header (C++ send_data).
        assert_eq!(
            t.state.lock().unwrap().allocations,
            vec![(payload_len + HEADER_SIZE) as FwSizeType]
        );
    }

    #[test]
    fn channel_message_bytes() {
        let t = Tester::new();
        let mut time = Time::new(TimeBase::TbProcTime, 0, 1, 2);
        let mut val = TlmBuffer::new();
        assert!(val.set_buff(&[0x11, 0x22]).is_ok());
        let p = t.component_in.tlm_in(0);
        p.target
            .invoke(p.port_num, 0x0000_00FF, &mut time, &mut val);
        assert_eq!(
            last_sent(&t),
            vec![
                0x00, 0x00, 0x00, 0x03, // HUB_TYPE_CHANNEL
                0x00, 0x00, 0x00, 0x00, // port 0
                0x00, 19, // payload size
                0x00, 0x00, 0x00, 0xFF, // id
                0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, // time
                0x00, 0x02, 0x11, 0x22, // value with length
            ]
        );
    }

    #[test]
    fn serial_buffer_and_command_message_bytes() {
        let t = Tester::new();

        // Serial: the call's bytes, verbatim.
        let mut comm = ComBuffer::new();
        assert!(comm.set_buff(&[0xDE, 0xAD]).is_ok());
        t.state.lock().unwrap().comm = comm.clone();
        t.state.lock().unwrap().current_port = 3;
        let p = t.component_in.serial_in(3);
        assert!(p.target.invoke_serial(p.port_num, &mut comm).is_ok());
        assert_eq!(
            last_sent(&t),
            vec![0, 0, 0, 0, 0, 0, 0, 3, 0, 2, 0xDE, 0xAD]
        );

        // Buffer: the window bytes, verbatim.
        let storage = t.data_store.lock().unwrap().take().unwrap();
        let mut buffer = Buffer::from_storage(storage, 0);
        buffer.data_mut()[..3].copy_from_slice(&[1, 2, 3]);
        buffer.set_size(3);
        t.state.lock().unwrap().expected_buffer = vec![1, 2, 3];
        t.state.lock().unwrap().current_port = 9;
        let p = t.component_in.buffer_in(9);
        p.target.invoke(p.port_num, buffer);
        assert_eq!(last_sent(&t), vec![0, 0, 0, 1, 0, 0, 0, 9, 0, 3, 1, 2, 3]);

        // Command dispatch: command bytes WITHOUT a length, then the context.
        let mut cmd = ComBuffer::new();
        assert!(cmd.set_buff(&[0x00, 0x00, 0x12, 0x34]).is_ok());
        let p = t.component_in.cmd_disp_in(4);
        p.target.invoke(p.port_num, &mut cmd, 0x0A0B_0C0D);
        assert_eq!(
            last_sent(&t),
            vec![
                0, 0, 0, 4, 0, 0, 0, 4, 0, 8, 0x00, 0x00, 0x12, 0x34, 0x0A, 0x0B, 0x0C, 0x0D
            ]
        );

        // Command response: opcode, sequence, response (U8).
        let p = t.component_in.cmd_resp_in(2);
        p.target
            .invoke(p.port_num, 0x0102_0304, 5, CmdResponse::Busy);
        assert_eq!(
            last_sent(&t),
            vec![0, 0, 0, 5, 0, 0, 0, 2, 0, 9, 1, 2, 3, 4, 0, 0, 0, 5, 5]
        );
        t.assert_all_returned();
    }

    // -----------------------------------------------------------------------
    // Rust-only checks
    // -----------------------------------------------------------------------

    /// Deviation: upstream undersizes the eventIn/tlmIn scratch buffers by
    /// the 2-byte length token and asserts on 505/506-byte argument
    /// buffers; here they are forwarded.
    #[test]
    fn full_event_and_channel_buffers_are_forwarded() {
        let t = Tester::new();
        let mut args = LogBuffer::new();
        assert!(args.set_buff(&[0x5A; FW_LOG_BUFFER_MAX_SIZE]).is_ok());
        let mut time = Time::from_seconds_useconds(1, 2);
        let p = t.component_in.event_in(0);
        p.target.invoke(
            p.port_num,
            1,
            &mut time,
            LogSeverity::Fatal,
            &mut args.clone(),
        );
        assert_eq!(t.state.lock().unwrap().event_out[0].3, args);

        let mut val = TlmBuffer::new();
        assert!(val.set_buff(&[0xA5; FW_TLM_BUFFER_MAX_SIZE]).is_ok());
        let p = t.component_in.tlm_in(0);
        p.target.invoke(p.port_num, 2, &mut time, &mut val.clone());
        assert_eq!(t.state.lock().unwrap().tlm_out[0].2, val);
        t.assert_all_returned();
    }

    /// A malformed event / response payload (bad enum value, truncated
    /// buffer) is dropped after a valid header, and the buffer returned.
    #[test]
    fn malformed_payloads_are_dropped() {
        let t = Tester::new();
        // Event with severity 0 (not a LogSeverity).
        let mut event = vec![0, 0, 0, 1];
        event.extend_from_slice(&[0; 11]);
        event.push(0x00);
        event.extend_from_slice(&[0, 0]);
        t.send_from_driver_packet(HubType::Event as u32, 0, event.len() as u16, &event);
        // Channel whose value length runs past the payload.
        let mut chan = vec![0, 0, 0, 1];
        chan.extend_from_slice(&[0; 11]);
        chan.extend_from_slice(&[0, 9, 1]);
        t.send_from_driver_packet(HubType::Channel as u32, 0, chan.len() as u16, &chan);
        // Command response with an undeclared CmdResponse.
        let resp = [0, 0, 0, 1, 0, 0, 0, 2, 0x77];
        t.send_from_driver_packet(HubType::CmdResp as u32, 0, resp.len() as u16, &resp);
        // Events/telemetry only exist on port 0.
        let mut good = vec![0, 0, 0, 1];
        good.extend_from_slice(&[0; 11]);
        good.extend_from_slice(&[0x01, 0, 0]);
        t.send_from_driver_packet(HubType::Event as u32, 1, good.len() as u16, &good);
        assert_eq!(t.returns(), 4);
        t.assert_no_outputs_sent();
        t.assert_all_returned();
    }

    /// An unconnected destination drops the message but still returns the
    /// transport buffer (the `isConnected` guard).
    #[test]
    fn unconnected_destinations_drop_and_return() {
        let hub = GenericHub::new("hub");
        let returned = Arc::new(Mutex::new(0usize));
        struct Count(Arc<Mutex<usize>>);
        impl BufferSendPort for Count {
            fn invoke(&self, _port_num: FwIndexType, _buffer: Buffer) {
                *self.0.lock().unwrap() += 1;
            }
        }
        hub.from_buffer_driver_return
            .connect(Arc::new(Count(returned.clone())), 0);
        for (hub_type, payload) in [
            (HubType::Port, vec![1u8]),
            (HubType::Buffer, vec![1u8]),
            (HubType::CmdDisp, vec![1, 0, 0, 0, 0]),
            (HubType::CmdResp, vec![0, 0, 0, 1, 0, 0, 0, 2, 0]),
        ] {
            let mut buffer = Buffer::allocate(HEADER_SIZE + payload.len());
            {
                let mut ser = buffer.get_serializer();
                assert!(ser.serialize_u32_be(hub_type as u32).is_ok());
                assert!(ser.serialize_u32_be(0).is_ok());
                assert!(
                    ser.serialize_bytes(&payload, LengthMode::IncludeLength, Endianness::Big)
                        .is_ok()
                );
            }
            let p = hub.from_buffer_driver(0);
            p.target.invoke(p.port_num, buffer);
        }
        assert_eq!(*returned.lock().unwrap(), 4);
    }

    #[derive(Default)]
    struct PingSink(Mutex<Vec<(FwIndexType, u32)>>);

    impl PingPort for PingSink {
        fn invoke(&self, port_num: FwIndexType, key: u32) {
            self.0.lock().unwrap().push((port_num, key));
        }
    }

    /// Typed ports through the hub pair: a typed output on the sending side
    /// (`connect_serial` to `serialIn`) is replayed on a typed input on the
    /// receiving side (`serialOut` `connect_typed`); a typed `Fw.Log` call
    /// through `serialIn` carries exactly the `eventIn` payload.
    #[test]
    fn typed_calls_replay_across_the_hub_pair() {
        let t = Tester::new();
        // A second pair, wired directly (driver = a plain connection), with
        // a typed consumer on the receiving side.
        let hub_a = GenericHub::new("hubA");
        let hub_b = GenericHub::new("hubB");
        hub_a.allocate.connect_to(t.from_allocate(0));
        hub_a.deallocate.connect_to(t.from_deallocate(0));
        hub_a
            .to_buffer_driver
            .connect_to(hub_b.from_buffer_driver(0));
        hub_b
            .from_buffer_driver_return
            .connect_to(hub_a.to_buffer_driver_return(0));
        let sink = Arc::new(PingSink::default());
        hub_b.serial_out[5].connect_typed(PortRef::new(sink.clone() as Arc<dyn PingPort>, 2));

        let ping_out: OutputPort<dyn PingPort> = OutputPort::new();
        ping_out.connect_serial(hub_a.serial_in(5));
        let p = ping_out.get();
        p.target.invoke(p.port_num, 0xFEED_BEEF);
        assert_eq!(*sink.0.lock().unwrap(), vec![(2, 0xFEED_BEEF)]);

        // A typed Fw.Log call serialized through serialIn: its payload is
        // byte-identical to the eventIn payload.
        let log_out: OutputPort<dyn LogPort> = OutputPort::new();
        log_out.connect_serial(t.component_in.serial_in(0));
        let mut time = Time::from_seconds_useconds(3, 4);
        let mut args = LogBuffer::new();
        assert!(args.set_buff(&[9, 8, 7]).is_ok());
        let mut expected = LinearBuffer::<{ <dyn LogPort as SerializablePort>::CAPACITY }>::new();
        assert!(expected.serialize_u32_be(42).is_ok());
        assert!(expected.serialize(&time, Endianness::Big).is_ok());
        assert!(
            expected
                .serialize(&LogSeverity::Command, Endianness::Big)
                .is_ok()
        );
        assert!(expected.serialize_buffer(&args, Endianness::Big).is_ok());
        {
            let mut s = t.state.lock().unwrap();
            s.comm = ComBuffer::new();
            assert!(s.comm.set_buff(expected.as_slice()).is_ok());
            s.current_port = 0;
        }
        let p = log_out.get();
        p.target.invoke(
            p.port_num,
            42,
            &mut time,
            LogSeverity::Command,
            &mut args.clone(),
        );
        let serial_payload = last_sent(&t)[HEADER_SIZE..].to_vec();
        let ev = t.component_in.event_in(0);
        ev.target
            .invoke(ev.port_num, 42, &mut time, LogSeverity::Command, &mut args);
        let event_payload = last_sent(&t)[HEADER_SIZE..].to_vec();
        assert_eq!(serial_payload, event_payload);
        assert_eq!(serial_payload, expected.as_slice());
        t.assert_all_returned();
    }

    #[test]
    fn hub_type_discriminants_match_cpp() {
        assert_eq!(HubType::Port as u32, 0);
        assert_eq!(HubType::Buffer as u32, 1);
        assert_eq!(HubType::Event as u32, 2);
        assert_eq!(HubType::Channel as u32, 3);
        assert_eq!(HubType::CmdDisp as u32, 4);
        assert_eq!(HubType::CmdResp as u32, 5);
        assert_eq!(HubType::Max as u32, 6);
        assert_eq!(HubType::from_wire(5), Some(HubType::CmdResp));
        assert_eq!(HubType::from_wire(6), None);
        assert_eq!(HEADER_SIZE, 10);
    }

    #[test]
    #[should_panic]
    fn out_of_range_input_port_asserts() {
        let t = Tester::new();
        let mut comm = ComBuffer::new();
        let p = t
            .component_in
            .serial_in(NUM_SERIAL_INPUT_PORTS as FwIndexType);
        let _ = p.target.invoke_serial(p.port_num, &mut comm);
    }
}
