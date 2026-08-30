//! `Svc::DpWriter` — writes data-product packets to `.fdp` files (ACTIVE).
//!
//! Port of `Svc/DpWriter/DpWriter.{fpp,hpp,cpp}` plus `Svc/DpPorts`
//! (`Svc.DpWritten`); analysis: `docs/cpp-analysis/data-products.md`
//! (section "Svc::DpWriter component").
//!
//! `bufferSendIn` runs a strictly ordered validation pipeline, each step
//! gated on the previous one succeeding: valid buffer -> big enough for a
//! packet -> header hash -> header deserialization -> big enough for the
//! declared data -> processing fan-out -> file name -> data hash -> write ->
//! `DpWritten` notification. The buffer is ALWAYS deallocated afterwards if
//! it is still valid, and any failure bumps `NumErrors`.
//!
//! # Deviations from C++
//!
//! * `procBufferSendOut` is declared `Fw.BufferSend` in FPP, but C++ keeps
//!   using the buffer after the synchronous call (the port passes
//!   `ref Fw::Buffer`). The framework's [`fprime_comp::BufferSendPort`]
//!   moves the buffer, so the processing fan-out uses [`DpProcPort`], which
//!   borrows the buffer mutably — same call graph, same in-place mutation.
//! * The zlib processing components (`Svc::DpZLibCompressor`,
//!   `Svc::DpCompressProc`) are OUT OF SCOPE for this port (they call libz;
//!   the workspace has zero third-party dependencies). The fan-out itself is
//!   ported verbatim, so any processing component can be connected;
//!   `ProcType::ZlibDeflate` (bit 0) keeps its wire value.
//! * The C++ file is never explicitly closed (it relies on the `Os::File`
//!   destructor); the Rust [`File`] closes on drop, which is the same thing.

use crate::file_manager::StringFormatStatus;
use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferSendPort, CmdGlue, CmdPort, ComponentDispatch, EventGlue,
    EventThrottle, MsgDispatchStatus, OutputPort, PortRef, QueueFullPolicy, SchedPort, TlmGlue,
    msg,
};
use fprime_config::{
    FILE_NAME_STRING_SIZE, FW_CMD_ARG_BUFFER_MAX_SIZE, FW_LOG_STRING_MAX_SIZE, FwChanIdType,
    FwDpIdType, FwDpPriorityType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType,
    FwOpcodeType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::dp::DpContainer;
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Endianness, FileNameString, LinearBuffer, LogSeverity,
    SerBuf, SerBufAny, Success, fw_assert, fw_try,
};
use fprime_os::File;
use fprime_os::file::{Mode, Status as FileStatus, WaitType};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Svc/DpPorts and DpCfg
// ---------------------------------------------------------------------------

/// `Svc.DpWritten` — the notification `DpWriter` emits after a successful
/// write and `Svc::DpCatalog` consumes on `addToCat`. Declared here, with
/// its emitter, because `Svc/DpPorts` contains nothing else.
pub trait DpWrittenPort: Send + Sync {
    /// Invoke the port.
    fn invoke(
        &self,
        port_num: FwIndexType,
        file_name: &FileNameString,
        priority: FwDpPriorityType,
        size: FwSizeType,
    );
}

/// `procBufferSendOut` — the data-product processing fan-out (see the
/// module header for why this is not `Fw.BufferSend`).
pub trait DpProcPort: Send + Sync {
    /// Process the packet in place; the header (and the `Fw::Buffer` size)
    /// may be rewritten, after which `DpWriter` re-reads the header.
    fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer);
}

/// `DpWriterNumProcPorts` (`default/config/AcConstants.fpp`).
pub const DP_WRITER_NUM_PROC_PORTS: usize = 5;

/// `DP_EXT` (`default/config/DpCfg.hpp`).
pub const DP_EXT: &str = ".fdp";

/// `DP_FILENAME_FORMAT` = `"%s/Dp_%08<id>_%08<secs>_%08<usecs>.fdp"`, each
/// field zero-padded to a MINIMUM of 8 digits (larger values print wider).
///
/// The result is bounded by `FileNameString` (240): an overflow returns
/// [`StringFormatStatus::Overflowed`] and an empty name, as the C++
/// `Fw::StringBase::format` does. `Svc::DpCatalog` recomputes this name from
/// a file's own header and rejects the file unless the path matches
/// byte for byte, so the two producers must stay identical.
pub fn format_dp_file_name(
    base_dir: &str,
    id: FwDpIdType,
    seconds: u32,
    useconds: u32,
) -> (FileNameString, StringFormatStatus) {
    let formatted = format!("{base_dir}/Dp_{id:08}_{seconds:08}_{useconds:08}{DP_EXT}");
    let mut out = FileNameString::new();
    if formatted.len() > FILE_NAME_STRING_SIZE {
        return (out, StringFormatStatus::Overflowed);
    }
    out.set(&formatted);
    (out, StringFormatStatus::Success)
}

// ---------------------------------------------------------------------------
// Dictionary constants
// ---------------------------------------------------------------------------

/// Queue message types, numbered from 1 in FPP declaration order
/// (`schedIn`, `bufferSendIn`, `cmdIn`).
const MSG_TYPE_SCHED_IN: FwEnumStoreType = 1;
const MSG_TYPE_BUFFER_SEND_IN: FwEnumStoreType = 2;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 3;

/// Queue message size = the largest async invocation, the command envelope:
/// 6 + 4 + 4 + 2 + 506 = 522 (`bufferSendIn` needs 6 + 8 = 14).
pub const QUEUE_MSG_SIZE: FwSizeType =
    (msg::ENVELOPE_HEADER_SIZE + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE) as FwSizeType;

/// FPP declares no `priority` qualifier on any async input.
const PORT_PRIORITY: FwQueuePriorityType = 1;

/// Event string arguments are `string size FileNameStringSize` (240), but
/// the generated `log_*` methods serialize through `Fw::LogStringArg`
/// (`StringTemplate<FW_LOG_STRING_MAX_SIZE>`), so the effective on-wire cap
/// is `min(declared size, FW_LOG_STRING_MAX_SIZE)` = 200.
const EVENT_STRING_SIZE: usize = if FILE_NAME_STRING_SIZE < FW_LOG_STRING_MAX_SIZE {
    FILE_NAME_STRING_SIZE
} else {
    FW_LOG_STRING_MAX_SIZE
};

type MsgBuffer = LinearBuffer<{ QUEUE_MSG_SIZE as usize }>;

// ---------------------------------------------------------------------------
// Component state
// ---------------------------------------------------------------------------

/// Component-thread state (all C++ `m_num*` members are plain fields).
#[derive(Default)]
struct DpWriterState {
    /// `m_dpFileNamePrefix` — the base directory from `configure`.
    dp_file_name_prefix: FileNameString,
    num_buffers_received: u32,
    num_bytes_written: u64,
    num_successful_writes: u32,
    num_failed_writes: u32,
    num_errors: u32,
    // `update on change` caches.
    last_buffers_received: Option<u32>,
    last_bytes_written: Option<u64>,
    last_successful_writes: Option<u32>,
    last_failed_writes: Option<u32>,
    last_errors: Option<u32>,
}

/// `Svc::DpWriter` — active data-product file writer.
pub struct DpWriter {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response glue.
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `procBufferSendOut`: \[5\] processing fan-out.
    pub proc_buffer_send_out: [OutputPort<dyn DpProcPort>; DP_WRITER_NUM_PROC_PORTS],
    /// `dpWrittenOut`: `Svc.DpWritten` out (optional — invoked only when
    /// connected).
    pub dp_written_out: OutputPort<dyn DpWrittenPort>,
    /// `deallocBufferSendOut`: `Fw.BufferSend` out (back to the pool).
    pub dealloc_buffer_send_out: OutputPort<dyn BufferSendPort>,
    /// Escrow for the async `bufferSendIn` buffer.
    escrow: BufferEscrow,
    invalid_buffer_throttle: EventThrottle,
    buffer_too_small_for_packet_throttle: EventThrottle,
    invalid_header_hash_throttle: EventThrottle,
    invalid_header_throttle: EventThrottle,
    buffer_too_small_for_data_throttle: EventThrottle,
    /// `throttle 10`, but `CLEAR_EVENT_THROTTLE` never clears it (C++ bug,
    /// reproduced).
    file_name_format_error_throttle: EventThrottle,
    file_open_error_throttle: EventThrottle,
    file_write_error_throttle: EventThrottle,
    state: Mutex<DpWriterState>,
}

impl DpWriter {
    // -- Commands (FPP-relative opcodes) -----------------------------------

    /// `CLEAR_EVENT_THROTTLE` — no explicit opcode in FPP, so 0x00.
    pub const OPCODE_CLEAR_EVENT_THROTTLE: FwOpcodeType = 0x00;

    // -- Events (FPP-relative ids, declaration order) ----------------------

    /// `InvalidBuffer()` — WARNING_HI, id 0, `throttle 10`.
    pub const EVENTID_INVALID_BUFFER: FwEventIdType = 0;
    /// `BufferTooSmallForPacket(bufferSize, minSize)` — WARNING_HI, id 1.
    pub const EVENTID_BUFFER_TOO_SMALL_FOR_PACKET: FwEventIdType = 1;
    /// `InvalidHeaderHash(bufferSize, storedHash, computedHash)` — id 2.
    pub const EVENTID_INVALID_HEADER_HASH: FwEventIdType = 2;
    /// `InvalidHeader(bufferSize, errorCode)` — WARNING_HI, id 3.
    pub const EVENTID_INVALID_HEADER: FwEventIdType = 3;
    /// `BufferTooSmallForData(bufferSize, minSize)` — WARNING_HI, id 4.
    pub const EVENTID_BUFFER_TOO_SMALL_FOR_DATA: FwEventIdType = 4;
    /// `FileNameFormatError(status)` — WARNING_HI, id 5.
    pub const EVENTID_FILE_NAME_FORMAT_ERROR: FwEventIdType = 5;
    /// `FileOpenError(status, file)` — WARNING_HI, id 6.
    pub const EVENTID_FILE_OPEN_ERROR: FwEventIdType = 6;
    /// `FileWriteError(status, bytesWritten, bytesToWrite, file)` — id 7.
    pub const EVENTID_FILE_WRITE_ERROR: FwEventIdType = 7;
    /// `FileWritten(bytes, file)` — ACTIVITY_LO, id 8, NOT throttled.
    pub const EVENTID_FILE_WRITTEN: FwEventIdType = 8;
    /// The `throttle 10` limit shared by events 0..=7.
    const THROTTLE_10: u32 = 10;

    // -- Telemetry (FPP-relative ids) --------------------------------------

    /// `NumBuffersReceived: U32 update on change` — id 0.
    pub const CHANID_NUM_BUFFERS_RECEIVED: FwChanIdType = 0;
    /// `NumBytesWritten: U64 update on change` — id 1.
    pub const CHANID_NUM_BYTES_WRITTEN: FwChanIdType = 1;
    /// `NumSuccessfulWrites: U32 update on change` — id 2.
    pub const CHANID_NUM_SUCCESSFUL_WRITES: FwChanIdType = 2;
    /// `NumFailedWrites: U32 update on change` — id 3.
    pub const CHANID_NUM_FAILED_WRITES: FwChanIdType = 3;
    /// `NumErrors: U32 update on change` — id 4.
    pub const CHANID_NUM_ERRORS: FwChanIdType = 4;

    /// Construct (topology phase 1).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            proc_buffer_send_out: std::array::from_fn(|_| OutputPort::new()),
            dp_written_out: OutputPort::new(),
            dealloc_buffer_send_out: OutputPort::new(),
            escrow: BufferEscrow::new(),
            invalid_buffer_throttle: EventThrottle::new(Self::THROTTLE_10),
            buffer_too_small_for_packet_throttle: EventThrottle::new(Self::THROTTLE_10),
            invalid_header_hash_throttle: EventThrottle::new(Self::THROTTLE_10),
            invalid_header_throttle: EventThrottle::new(Self::THROTTLE_10),
            buffer_too_small_for_data_throttle: EventThrottle::new(Self::THROTTLE_10),
            file_name_format_error_throttle: EventThrottle::new(Self::THROTTLE_10),
            file_open_error_throttle: EventThrottle::new(Self::THROTTLE_10),
            file_write_error_throttle: EventThrottle::new(Self::THROTTLE_10),
            state: Mutex::new(DpWriterState::default()),
        })
    }

    /// Create the message queue (topology "configure" step).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active.queued.create_queue(queue_depth, QUEUE_MSG_SIZE);
    }

    /// C++ `configure(dpFileNamePrefix)`: the base directory `.fdp` files
    /// are written to.
    pub fn configure(&self, dp_file_name_prefix: &str) {
        self.state.lock().unwrap().dp_file_name_prefix = FileNameString::from(dp_file_name_prefix);
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_CLEAR_EVENT_THROTTLE]);
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    // -- Input-port factories ---------------------------------------------

    /// `bufferSendIn` — ASYNC `Fw.BufferSend` input; the owned buffer rides
    /// through the escrow.
    pub fn buffer_send_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(
            Arc::new(BufferSendInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `schedIn` — ASYNC `Svc.Sched` input.
    pub fn sched_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(SchedInAdapter { comp: self.clone() }), port_num)
    }

    /// `cmdIn` — ASYNC `Fw.Cmd` input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers (component thread) ---------------------------------------

    /// `bufferSendIn_handler`: run the pipeline, then ALWAYS deallocate a
    /// still-valid buffer and count an error on any failure.
    fn buffer_send_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        self.state.lock().unwrap().num_buffers_received += 1;
        let (status, buffer) = self.run_pipeline(buffer);
        if buffer.is_valid() {
            let port = self.dealloc_buffer_send_out.get();
            port.target.invoke(port.port_num, buffer);
        }
        if status != Success::Success {
            self.state.lock().unwrap().num_errors += 1;
        }
    }

    /// The ordered validation/write pipeline. Returns the buffer so the
    /// caller can deallocate it whatever happened.
    fn run_pipeline(&self, buffer: Buffer) -> (Success, Buffer) {
        let buffer_size = buffer.size() as FwSizeType;
        if !buffer.is_valid() {
            self.log_invalid_buffer();
            return (Success::Failure, buffer);
        }
        if (buffer.size()) < DpContainer::MIN_PACKET_SIZE {
            self.log_buffer_too_small_for_packet(buffer_size, DpContainer::MIN_PACKET_SIZE as u32);
            return (Success::Failure, buffer);
        }

        let mut container = DpContainer::new();
        container.set_buffer(buffer);
        let (hash_status, stored, computed) = container.check_header_hash();
        if hash_status != Success::Success {
            self.log_invalid_header_hash(buffer_size, stored, computed);
            return (Success::Failure, container.take_buffer());
        }

        // C++ parity: `deserializePacketHeader` calls setBuffer AGAIN before
        // deserializeHeader. The second call resets the data size to 0,
        // which is required before the header is read back — do not
        // "optimize" it away.
        let buffer = container.take_buffer();
        container.set_buffer(buffer);
        let serial_status = container.deserialize_header();
        if !serial_status.is_ok() {
            self.log_invalid_header(buffer_size, serial_status as i32 as u32);
            return (Success::Failure, container.take_buffer());
        }

        let packet_size = container.packet_size();
        if buffer_size < packet_size {
            self.log_buffer_too_small_for_data(buffer_size, packet_size as u32);
            return (Success::Failure, container.take_buffer());
        }

        self.perform_processing(&mut container);

        let prefix = self.state.lock().unwrap().dp_file_name_prefix.clone();
        let time_tag = container.time_tag();
        let (file_name, format_status) = format_dp_file_name(
            prefix.as_str().unwrap_or_default(),
            container.id(),
            time_tag.get_seconds(),
            time_tag.get_useconds(),
        );
        if format_status != StringFormatStatus::Success {
            self.log_file_name_format_error(format_status);
            return (Success::Failure, container.take_buffer());
        }

        container.update_data_hash();

        let (write_status, file_size) = self.write_file(&container, &file_name);
        if write_status != Success::Success {
            return (Success::Failure, container.take_buffer());
        }

        self.send_notification(&container, &file_name, file_size);
        (Success::Success, container.take_buffer())
    }

    /// C++ `performProcessing`: fan out on the `procTypes` BIT MASK — bit
    /// index n selects `procBufferSendOut` port n — then re-read, re-hash,
    /// re-serialize the header and shrink the buffer if anything ran.
    fn perform_processing(&self, container: &mut DpContainer) {
        let proc_types = container.proc_types();
        let mut did_process = false;
        for port_num in 0..DP_WRITER_NUM_PROC_PORTS {
            if (proc_types & (1u8 << port_num)) != 0 {
                let port = self.proc_buffer_send_out[port_num].get();
                port.target.invoke(port.port_num, container.buffer_mut());
                did_process = true;
            }
        }
        if did_process {
            let status = container.deserialize_header();
            fw_assert!(status.is_ok(), status as i32);
            fw_assert!(
                container.packet_size() <= container.buffer().size() as FwSizeType,
                container.packet_size() as i32,
                container.buffer().size() as i32
            );
            // C++ parity: updateHeaderHash() then serializeHeader() (which
            // updates the hash again) — redundant, reproduced.
            container.update_header_hash();
            container.serialize_header();
            container.shrink_buffer_size();
        }
    }

    /// C++ `writeFile`: open `OPEN_CREATE`, write exactly `packetSize`
    /// bytes, count the outcome. Returns `(status, fileSize)`.
    fn write_file(&self, container: &DpContainer, file_name: &FileNameString) -> (Success, u64) {
        let file_size = container.packet_size();
        let mut status = Success::Success;
        let mut file = File::new();
        let path = file_name.as_str().unwrap_or_default();
        let open_status = file.open(path, Mode::OpenCreate);
        if open_status != FileStatus::OpOk {
            self.log_file_open_error(open_status as i32 as u32, file_name);
            status = Success::Failure;
        }
        if status == Success::Success {
            let mut write_size: FwSizeType = file_size;
            let data = &container.buffer().data()[..file_size as usize];
            let file_status = file.write(data, &mut write_size, WaitType::Wait);
            if file_status == FileStatus::OpOk {
                self.state.lock().unwrap().num_bytes_written += write_size;
            }
            if file_status == FileStatus::OpOk && write_size == file_size {
                self.log_file_written(write_size as u32, file_name);
            } else {
                self.log_file_write_error(
                    file_status as i32 as u32,
                    write_size as u32,
                    file_size as u32,
                    file_name,
                );
                status = Success::Failure;
            }
        }
        {
            let mut state = self.state.lock().unwrap();
            if status == Success::Success {
                state.num_successful_writes += 1;
            } else {
                state.num_failed_writes += 1;
            }
        }
        (status, file_size)
    }

    /// C++ `sendNotification`: only when `dpWrittenOut` is connected.
    fn send_notification(
        &self,
        container: &DpContainer,
        file_name: &FileNameString,
        file_size: FwSizeType,
    ) {
        if let Some(port) = self.dp_written_out.try_get() {
            port.target
                .invoke(port.port_num, file_name, container.priority(), file_size);
        }
    }

    /// `schedIn_handler`: the five `update on change` channels, C++ order.
    fn sched_handler(&self, _port_num: FwIndexType, _context: u32) {
        let (received, written, successful, failed, errors) = {
            let state = &mut *self.state.lock().unwrap();
            (
                changed_u32(&mut state.last_buffers_received, state.num_buffers_received),
                changed_u64(&mut state.last_bytes_written, state.num_bytes_written),
                changed_u32(
                    &mut state.last_successful_writes,
                    state.num_successful_writes,
                ),
                changed_u32(&mut state.last_failed_writes, state.num_failed_writes),
                changed_u32(&mut state.last_errors, state.num_errors),
            )
        };
        let id_base = self.id_base();
        if let Some(v) = received {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_NUM_BUFFERS_RECEIVED,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = written {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_NUM_BYTES_WRITTEN,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = successful {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_NUM_SUCCESSFUL_WRITES,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = failed {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_NUM_FAILED_WRITES,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = errors {
            self.tlm
                .tlm_write(id_base, Self::CHANID_NUM_ERRORS, &v, self.evt.time_get());
        }
    }

    /// `CLEAR_EVENT_THROTTLE` handler: clears 7 of the 8 throttled events —
    /// `FileNameFormatError` is `throttle 10` but its ThrottleClear is never
    /// called in C++ (reproduced).
    fn clear_event_throttle_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.buffer_too_small_for_data_throttle.clear();
        self.buffer_too_small_for_packet_throttle.clear();
        self.file_open_error_throttle.clear();
        self.file_write_error_throttle.clear();
        self.invalid_buffer_throttle.clear();
        self.invalid_header_hash_throttle.clear();
        self.invalid_header_throttle.clear();
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// Command dispatch on the local opcode.
    fn handle_command(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_CLEAR_EVENT_THROTTLE => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.clear_event_throttle_cmd_handler(op_code, cmd_seq);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    // -- Events ------------------------------------------------------------

    fn log_invalid_buffer(&self) {
        if !self.invalid_buffer_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_INVALID_BUFFER,
            LogSeverity::WarningHi,
            "Received buffer is invalid",
            |_buf| fprime_fw::SerializeStatus::Ok,
        );
    }

    fn log_buffer_too_small_for_packet(&self, buffer_size: FwSizeType, min_size: u32) {
        if !self.buffer_too_small_for_packet_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BUFFER_TOO_SMALL_FOR_PACKET,
            LogSeverity::WarningHi,
            &format!("Received buffer has size {buffer_size}; minimum required size is {min_size}"),
            |buf| {
                fw_try!(buf.serialize_u64_be(buffer_size));
                buf.serialize_u32_be(min_size)
            },
        );
    }

    fn log_invalid_header_hash(&self, buffer_size: FwSizeType, stored: u32, computed: u32) {
        if !self.invalid_header_hash_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_INVALID_HEADER_HASH,
            LogSeverity::WarningHi,
            &format!(
                "Received a buffer of size {buffer_size} with an invalid header hash \
                 (stored {stored:x}, computed {computed:x})"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(buffer_size));
                fw_try!(buf.serialize_u32_be(stored));
                buf.serialize_u32_be(computed)
            },
        );
    }

    fn log_invalid_header(&self, buffer_size: FwSizeType, error_code: u32) {
        if !self.invalid_header_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_INVALID_HEADER,
            LogSeverity::WarningHi,
            &format!(
                "Received buffer of size {buffer_size}; deserialization of packet header \
                 failed with error code {error_code}"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(buffer_size));
                buf.serialize_u32_be(error_code)
            },
        );
    }

    fn log_buffer_too_small_for_data(&self, buffer_size: FwSizeType, min_size: u32) {
        if !self.buffer_too_small_for_data_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BUFFER_TOO_SMALL_FOR_DATA,
            LogSeverity::WarningHi,
            &format!("Received buffer has size {buffer_size}; minimum required size is {min_size}"),
            |buf| {
                fw_try!(buf.serialize_u64_be(buffer_size));
                buf.serialize_u32_be(min_size)
            },
        );
    }

    fn log_file_name_format_error(&self, status: StringFormatStatus) {
        if !self.file_name_format_error_throttle.ok_to_emit() {
            return;
        }
        let code = status.as_repr();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_NAME_FORMAT_ERROR,
            LogSeverity::WarningHi,
            &format!("Error {code} formatting DP file name"),
            |buf| buf.serialize_u8_be(code),
        );
    }

    fn log_file_open_error(&self, status: u32, file_name: &FileNameString) {
        if !self.file_open_error_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_OPEN_ERROR,
            LogSeverity::WarningHi,
            &format!("Error {status} opening file {file_name}"),
            |buf| {
                fw_try!(buf.serialize_u32_be(status));
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_file_write_error(
        &self,
        status: u32,
        bytes_written: u32,
        bytes_to_write: u32,
        file_name: &FileNameString,
    ) {
        if !self.file_write_error_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_WRITE_ERROR,
            LogSeverity::WarningHi,
            &format!(
                "Error {status} while writing {bytes_written} of {bytes_to_write} bytes \
                 to {file_name}"
            ),
            |buf| {
                fw_try!(buf.serialize_u32_be(status));
                fw_try!(buf.serialize_u32_be(bytes_written));
                fw_try!(buf.serialize_u32_be(bytes_to_write));
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    /// `FileWritten` is ACTIVITY_LO and NOT throttled.
    fn log_file_written(&self, bytes: u32, file_name: &FileNameString) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_WRITTEN,
            LogSeverity::ActivityLo,
            &format!("Wrote {bytes} bytes to file {file_name}"),
            |buf| {
                fw_try!(buf.serialize_u32_be(bytes));
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }
}

/// `update on change` helper for the `u32` channels.
fn changed_u32(last: &mut Option<u32>, value: u32) -> Option<u32> {
    if *last == Some(value) {
        None
    } else {
        *last = Some(value);
        Some(value)
    }
}

/// `update on change` helper for `NumBytesWritten` (U64).
fn changed_u64(last: &mut Option<u64>, value: u64) -> Option<u64> {
    if *last == Some(value) {
        None
    } else {
        *last = Some(value);
        Some(value)
    }
}

// -- Async input adapters ---------------------------------------------------

/// `bufferSendIn` adapter: `[msg_type][port_num][escrow token u64]`.
struct BufferSendInAdapter {
    comp: Arc<DpWriter>,
}

impl BufferSendPort for BufferSendInAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_BUFFER_SEND_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `schedIn` adapter.
struct SchedInAdapter {
    comp: Arc<DpWriter>,
}

impl SchedPort for SchedInAdapter {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_SCHED_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `cmdIn` adapter.
struct CmdInAdapter {
    comp: Arc<DpWriter>,
}

impl CmdPort for CmdInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_CMD_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

// -- Dispatch ---------------------------------------------------------------

impl ComponentDispatch for DpWriter {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            MSG_TYPE_SCHED_IN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.sched_handler(port_num, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_BUFFER_SEND_IN => {
                let mut token = 0u64;
                if !buf.deserialize_u64_be(&mut token).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.buffer_send_in_handler(port_num, buffer);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_CMD_IN => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok()
                    || !buf.deserialize_u32_be(&mut cmd_seq).is_ok()
                    || !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.handle_command(op_code, cmd_seq, &mut args);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for DpWriter {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::dp::{DpState, ProcType};
    use fprime_fw::{LogBuffer, TextLogString, Time, TimeBase, TlmBuffer};
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    const ID_BASE: FwIdType = 0x3000;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    fn temp_dir(tag: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fprime_rust_dpwriter_{tag}_{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    #[derive(Default)]
    struct GroundStub {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        written: Mutex<Vec<(String, FwDpPriorityType, FwSizeType)>>,
        deallocated: Mutex<Vec<Vec<u8>>>,
    }

    impl CmdRegPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for GroundStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.responses
                .lock()
                .unwrap()
                .push((op_code, cmd_seq, response));
        }
    }

    impl LogPort for GroundStub {
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

    impl LogTextPort for GroundStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _text: &mut TextLogString,
        ) {
        }
    }

    impl TlmPort for GroundStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm.lock().unwrap().push((id, val.as_slice().to_vec()));
        }
    }

    impl DpWrittenPort for GroundStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            file_name: &FileNameString,
            priority: FwDpPriorityType,
            size: FwSizeType,
        ) {
            self.written.lock().unwrap().push((
                file_name.as_str().unwrap_or_default().to_string(),
                priority,
                size,
            ));
        }
    }

    impl BufferSendPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.deallocated
                .lock()
                .unwrap()
                .push(buffer.data().to_vec());
        }
    }

    struct TimeStub;

    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 100, 42);
        }
    }

    /// A processing component: records the invocation and optionally
    /// rewrites the packet's data (shrinking it) exactly as a compressor
    /// would, header hash included.
    #[derive(Default)]
    struct ProcStub {
        calls: Mutex<Vec<(FwIndexType, usize)>>,
        shrink_to: Mutex<Option<Vec<u8>>>,
    }

    impl DpProcPort for ProcStub {
        fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer) {
            self.calls.lock().unwrap().push((port_num, buffer.size()));
            let replacement = self.shrink_to.lock().unwrap().clone();
            if let Some(data) = replacement {
                let mut container = DpContainer::new();
                container.set_buffer(std::mem::take(buffer));
                let status = container.deserialize_header();
                assert!(status.is_ok());
                container.data_region_mut()[..data.len()].copy_from_slice(&data);
                container.set_data_size(data.len() as FwSizeType);
                container.serialize_header();
                *buffer = container.take_buffer();
            }
        }
    }

    struct Harness {
        comp: Arc<DpWriter>,
        ground: Arc<GroundStub>,
        dir: PathBuf,
    }

    impl Harness {
        fn new(tag: &str) -> Self {
            let dir = temp_dir(tag);
            let h = Self::with_prefix(dir.to_str().unwrap());
            Self { dir, ..h }
        }

        fn with_prefix(prefix: &str) -> Self {
            let comp = DpWriter::new("dpWriter");
            let ground = Arc::new(GroundStub::default());
            comp.active.queued.base.set_id_base(ID_BASE);
            comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
            comp.cmd.cmd_response_out.connect(ground.clone(), 0);
            comp.evt.log_out.connect(ground.clone(), 0);
            comp.evt.text_log_out.connect(ground.clone(), 0);
            comp.evt.time_out.connect(Arc::new(TimeStub), 0);
            comp.tlm.tlm_out.connect(ground.clone(), 0);
            comp.dp_written_out.connect(ground.clone(), 0);
            comp.dealloc_buffer_send_out.connect(ground.clone(), 0);
            comp.init(16);
            comp.configure(prefix);
            Self {
                comp,
                ground,
                dir: PathBuf::new(),
            }
        }

        fn drain(&self) {
            let _ = self
                .comp
                .active
                .queued
                .dispatch_available_messages(self.comp.as_ref());
        }

        fn send(&self, buffer: Buffer) {
            let port = self.comp.buffer_send_in(0);
            port.target.invoke(port.port_num, buffer);
            self.drain();
        }

        fn events(&self) -> Vec<(FwEventIdType, LogSeverity, Vec<u8>)> {
            self.ground.events.lock().unwrap().clone()
        }

        fn send_cmd(&self, op_code: FwOpcodeType, cmd_seq: u32, arg_bytes: &[u8]) {
            let mut args = CmdArgBuffer::new();
            assert!(args.set_buff(arg_bytes).is_ok());
            let port = self.comp.cmd_in(0);
            port.target
                .invoke(port.port_num, op_code, cmd_seq, &mut args);
            self.drain();
        }
    }

    /// Build a valid data-product packet; `slack` extra bytes are left in
    /// the buffer beyond the packet.
    fn make_packet(
        id: FwDpIdType,
        priority: FwDpPriorityType,
        seconds: u32,
        useconds: u32,
        proc_types: u8,
        data: &[u8],
        slack: usize,
    ) -> Buffer {
        let mut container = DpContainer::with_buffer(
            id,
            Buffer::allocate(DpContainer::MIN_PACKET_SIZE + data.len() + slack),
        );
        container.set_priority(priority);
        container.set_time_tag(Time::new(TimeBase::TbWorkstationTime, 0, seconds, useconds));
        container.set_proc_types(proc_types);
        container.set_dp_state(DpState::Untransmitted);
        container.data_region_mut()[..data.len()].copy_from_slice(data);
        container.set_data_size(data.len() as FwSizeType);
        container.serialize_header();
        container.take_buffer()
    }

    #[test]
    fn dictionary_ids_match_the_fpp_model() {
        assert_eq!(DpWriter::OPCODE_CLEAR_EVENT_THROTTLE, 0);
        assert_eq!(DpWriter::EVENTID_INVALID_BUFFER, 0);
        assert_eq!(DpWriter::EVENTID_BUFFER_TOO_SMALL_FOR_PACKET, 1);
        assert_eq!(DpWriter::EVENTID_INVALID_HEADER_HASH, 2);
        assert_eq!(DpWriter::EVENTID_INVALID_HEADER, 3);
        assert_eq!(DpWriter::EVENTID_BUFFER_TOO_SMALL_FOR_DATA, 4);
        assert_eq!(DpWriter::EVENTID_FILE_NAME_FORMAT_ERROR, 5);
        assert_eq!(DpWriter::EVENTID_FILE_OPEN_ERROR, 6);
        assert_eq!(DpWriter::EVENTID_FILE_WRITE_ERROR, 7);
        assert_eq!(DpWriter::EVENTID_FILE_WRITTEN, 8);
        assert_eq!(DpWriter::CHANID_NUM_BUFFERS_RECEIVED, 0);
        assert_eq!(DpWriter::CHANID_NUM_ERRORS, 4);
        assert_eq!(QUEUE_MSG_SIZE, 522);
    }

    #[test]
    fn file_name_follows_the_dp_cfg_format() {
        let (name, status) = format_dp_file_name("/dp", 1, 2, 3);
        assert_eq!(status, StringFormatStatus::Success);
        assert_eq!(
            name.as_str().unwrap(),
            "/dp/Dp_00000001_00000002_00000003.fdp"
        );
        // Values wider than 8 digits are not truncated.
        let (name, status) = format_dp_file_name("d", 123_456_789, 4_000_000_000, 999_999);
        assert_eq!(status, StringFormatStatus::Success);
        assert_eq!(
            name.as_str().unwrap(),
            "d/Dp_123456789_4000000000_00999999.fdp"
        );
    }

    #[test]
    fn an_over_long_file_name_overflows() {
        let prefix = "x".repeat(220);
        let (name, status) = format_dp_file_name(&prefix, 1, 2, 3);
        assert_eq!(status, StringFormatStatus::Overflowed);
        assert!(name.is_empty());
    }

    #[test]
    fn writes_a_round_trippable_fdp_file() {
        let h = Harness::new("roundtrip");
        let data = [0xDE, 0xAD, 0xBE, 0xEF];
        h.send(make_packet(7, 3, 2, 3, 0x00, &data, 0));

        let path = h.dir.join("Dp_00000007_00000002_00000003.fdp");
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.len(), 69);

        // The file IS the packet: re-read it through a container.
        let mut buffer = Buffer::allocate(bytes.len());
        buffer.data_mut().copy_from_slice(&bytes);
        let mut container = DpContainer::new();
        container.set_buffer(buffer);
        assert_eq!(container.check_header_hash().0, Success::Success);
        assert!(container.deserialize_header().is_ok());
        assert_eq!(container.id(), 7);
        assert_eq!(container.priority(), 3);
        assert_eq!(container.data_size(), 4);
        assert_eq!(container.data(), &data);
        assert_eq!(container.check_data_hash().0, Success::Success);

        // The written file equals the deallocated buffer byte for byte.
        let deallocated = h.ground.deallocated.lock().unwrap();
        assert_eq!(deallocated.len(), 1);
        assert_eq!(deallocated[0], bytes);

        // FileWritten (id 8, ACTIVITY_LO) and the DpWritten notification.
        let events = h.events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, ID_BASE + DpWriter::EVENTID_FILE_WRITTEN);
        assert_eq!(events[0].1, LogSeverity::ActivityLo);
        assert_eq!(&events[0].2[..4], &69u32.to_be_bytes());
        assert_eq!(
            *h.ground.written.lock().unwrap(),
            vec![(path.to_str().unwrap().to_string(), 3, 69)]
        );
    }

    #[test]
    fn telemetry_counts_the_successful_write() {
        let h = Harness::new("tlm");
        h.send(make_packet(1, 0, 0, 0, 0x00, &[1, 2, 3], 0));
        let sched = h.comp.sched_in(0);
        sched.target.invoke(sched.port_num, 0);
        h.drain();
        assert_eq!(
            *h.ground.tlm.lock().unwrap(),
            vec![
                (ID_BASE, vec![0, 0, 0, 1]),                  // NumBuffersReceived
                (ID_BASE + 1, vec![0, 0, 0, 0, 0, 0, 0, 68]), // NumBytesWritten
                (ID_BASE + 2, vec![0, 0, 0, 1]),              // NumSuccessfulWrites
                (ID_BASE + 3, vec![0, 0, 0, 0]),              // NumFailedWrites
                (ID_BASE + 4, vec![0, 0, 0, 0]),              // NumErrors
            ]
        );
    }

    #[test]
    fn an_invalid_buffer_is_reported_and_not_deallocated() {
        let h = Harness::new("invalid");
        h.send(Buffer::empty());
        let events = h.events();
        assert_eq!(events[0].0, ID_BASE + DpWriter::EVENTID_INVALID_BUFFER);
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        assert!(events[0].2.is_empty());
        assert!(h.ground.deallocated.lock().unwrap().is_empty());
        assert_eq!(h.comp.state.lock().unwrap().num_errors, 1);
    }

    #[test]
    fn a_buffer_below_the_minimum_packet_size_is_rejected() {
        let h = Harness::new("small");
        h.send(Buffer::allocate(64));
        let events = h.events();
        assert_eq!(
            events[0].0,
            ID_BASE + DpWriter::EVENTID_BUFFER_TOO_SMALL_FOR_PACKET
        );
        let mut expected = 64u64.to_be_bytes().to_vec();
        expected.extend_from_slice(&65u32.to_be_bytes());
        assert_eq!(events[0].2, expected);
        // Still deallocated (it is a valid buffer).
        assert_eq!(h.ground.deallocated.lock().unwrap().len(), 1);
    }

    #[test]
    fn a_corrupted_header_hash_is_reported_with_both_hashes() {
        let h = Harness::new("hash");
        let mut buffer = make_packet(1, 0, 0, 0, 0x00, &[], 0);
        let stored = u32::from_be_bytes([
            buffer.data()[57],
            buffer.data()[58],
            buffer.data()[59],
            buffer.data()[60],
        ]);
        buffer.data_mut()[2] ^= 0xFF; // corrupt the id
        h.send(buffer);
        let events = h.events();
        assert_eq!(events[0].0, ID_BASE + DpWriter::EVENTID_INVALID_HEADER_HASH);
        assert_eq!(&events[0].2[..8], &65u64.to_be_bytes());
        assert_eq!(&events[0].2[8..12], &stored.to_be_bytes());
        assert_ne!(&events[0].2[12..16], &stored.to_be_bytes());
        assert_eq!(h.comp.state.lock().unwrap().num_errors, 1);
    }

    #[test]
    fn a_bad_packet_descriptor_is_an_invalid_header() {
        let h = Harness::new("header");
        let mut container =
            DpContainer::with_buffer(1, Buffer::allocate(DpContainer::MIN_PACKET_SIZE));
        container.serialize_header();
        // Break the descriptor AFTER serialization, then re-hash so the
        // hash check passes and the deserialization gate is the one that
        // fires.
        container.buffer_mut().data_mut()[1] = 0x06;
        container.update_header_hash();
        h.send(container.take_buffer());
        let events = h.events();
        assert_eq!(events[0].0, ID_BASE + DpWriter::EVENTID_INVALID_HEADER);
        assert_eq!(&events[0].2[..8], &65u64.to_be_bytes());
        // FW_SERIALIZE_FORMAT_ERROR == 1
        assert_eq!(&events[0].2[8..12], &1u32.to_be_bytes());
    }

    #[test]
    fn a_declared_data_size_beyond_the_buffer_is_rejected() {
        let h = Harness::new("data");
        let mut container =
            DpContainer::with_buffer(1, Buffer::allocate(DpContainer::MIN_PACKET_SIZE));
        container.set_data_size(10); // packet size 75 > buffer size 65
        container.serialize_header();
        h.send(container.take_buffer());
        let events = h.events();
        assert_eq!(
            events[0].0,
            ID_BASE + DpWriter::EVENTID_BUFFER_TOO_SMALL_FOR_DATA
        );
        let mut expected = 65u64.to_be_bytes().to_vec();
        expected.extend_from_slice(&75u32.to_be_bytes());
        assert_eq!(events[0].2, expected);
    }

    #[test]
    fn an_unwritable_directory_reports_a_file_open_error() {
        let h = Harness::with_prefix("/nonexistent-dp-directory");
        h.send(make_packet(1, 0, 0, 0, 0x00, &[], 0));
        let events = h.events();
        assert_eq!(events[0].0, ID_BASE + DpWriter::EVENTID_FILE_OPEN_ERROR);
        assert_eq!(h.comp.state.lock().unwrap().num_failed_writes, 1);
        assert_eq!(h.comp.state.lock().unwrap().num_errors, 1);
        // The buffer still comes back.
        assert_eq!(h.ground.deallocated.lock().unwrap().len(), 1);
    }

    #[test]
    fn an_event_file_name_is_clipped_to_the_log_string_cap() {
        // C++ parity: the event arg is declared `string size
        // FileNameStringSize` (240) but the generated `log_*` method carries
        // it as `Fw::LogStringArg` = `StringTemplate<FW_LOG_STRING_MAX_SIZE>`,
        // so anything past 200 bytes never reaches the wire.
        let prefix = format!("/nonexistent-dp-dir{}", "x".repeat(171));
        assert_eq!(prefix.len(), 190);
        let h = Harness::with_prefix(&prefix);
        h.send(make_packet(1, 0, 2, 3, 0x00, &[], 0));

        // 190 + "/" + "Dp_00000001_00000002_00000003.fdp" (33) = 224 bytes:
        // under FileNameString's 240, over FW_LOG_STRING_MAX_SIZE.
        let full = format!("{prefix}/Dp_00000001_00000002_00000003.fdp");
        assert_eq!(full.len(), 224);

        let events = h.events();
        assert_eq!(events[0].0, ID_BASE + DpWriter::EVENTID_FILE_OPEN_ERROR);
        // status (u32) then the string: 2-byte length then the bytes.
        let args = &events[0].2;
        assert_eq!(&args[4..6], &(FW_LOG_STRING_MAX_SIZE as u16).to_be_bytes());
        assert_eq!(args.len(), 4 + 2 + FW_LOG_STRING_MAX_SIZE);
        assert_eq!(&args[6..], &full.as_bytes()[..FW_LOG_STRING_MAX_SIZE]);
        // The un-clipped name still reaches the file-name port in full.
        assert_eq!(EVENT_STRING_SIZE, FW_LOG_STRING_MAX_SIZE);
    }

    #[test]
    fn writing_the_same_product_twice_fails_the_second_open() {
        let h = Harness::new("exists");
        h.send(make_packet(1, 0, 5, 6, 0x00, &[], 0));
        h.send(make_packet(1, 0, 5, 6, 0x00, &[], 0));
        let events = h.events();
        assert_eq!(events[0].0, ID_BASE + DpWriter::EVENTID_FILE_WRITTEN);
        // OPEN_CREATE defaults to NO_OVERWRITE: FILE_EXISTS (6).
        assert_eq!(events[1].0, ID_BASE + DpWriter::EVENTID_FILE_OPEN_ERROR);
        assert_eq!(
            &events[1].2[..4],
            &(FileStatus::FileExists as u32).to_be_bytes()
        );
    }

    #[test]
    fn processing_fans_out_on_the_bit_mask_and_shrinks_the_buffer() {
        let h = Harness::new("proc");
        let proc0 = Arc::new(ProcStub::default());
        let proc1 = Arc::new(ProcStub::default());
        h.comp.proc_buffer_send_out[0].connect(proc0.clone(), 0);
        h.comp.proc_buffer_send_out[1].connect(proc1.clone(), 1);
        // ProcType::One (0x02) selects port 1 only.
        *proc1.shrink_to.lock().unwrap() = Some(vec![0xAA, 0xBB]);
        h.send(make_packet(
            9,
            1,
            1,
            1,
            ProcType::One.as_repr(),
            &[1, 2, 3, 4, 5, 6],
            10,
        ));
        assert!(proc0.calls.lock().unwrap().is_empty());
        assert_eq!(proc1.calls.lock().unwrap().len(), 1);

        // The buffer was shrunk to the new packet size (65 + 2) and that is
        // exactly what was written and deallocated.
        let path = h.dir.join("Dp_00000009_00000001_00000001.fdp");
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.len(), 67);
        assert_eq!(&bytes[61..63], &[0xAA, 0xBB]);
        assert_eq!(h.ground.deallocated.lock().unwrap()[0].len(), 67);
    }

    #[test]
    fn clear_event_throttle_clears_seven_events_but_not_the_file_name_error() {
        // A 230-character prefix always overflows the 240-byte file name.
        let h = Harness::with_prefix(&"p".repeat(230));
        for _ in 0..12 {
            h.send(make_packet(1, 0, 0, 0, 0x00, &[], 0));
        }
        assert_eq!(h.events().len(), 10); // throttle 10
        h.ground.events.lock().unwrap().clear();

        h.send_cmd(ID_BASE + DpWriter::OPCODE_CLEAR_EVENT_THROTTLE, 4, &[]);
        assert_eq!(
            *h.ground.responses.lock().unwrap(),
            vec![(
                ID_BASE + DpWriter::OPCODE_CLEAR_EVENT_THROTTLE,
                4,
                CmdResponse::Ok
            )]
        );
        // FileNameFormatError stays throttled (C++ omits its ThrottleClear).
        h.send(make_packet(1, 0, 0, 0, 0x00, &[], 0));
        assert!(h.events().is_empty());

        // ... while InvalidBuffer was cleared and fires again.
        for _ in 0..11 {
            h.send(Buffer::empty());
        }
        assert_eq!(h.events().len(), 10);
    }

    #[test]
    fn unknown_opcodes_and_residual_bytes_are_rejected() {
        let h = Harness::new("cmd");
        h.send_cmd(ID_BASE + 0x42, 1, &[]);
        h.send_cmd(ID_BASE + DpWriter::OPCODE_CLEAR_EVENT_THROTTLE, 2, &[0x00]);
        assert_eq!(
            *h.ground.responses.lock().unwrap(),
            vec![
                (ID_BASE + 0x42, 1, CmdResponse::InvalidOpcode),
                (
                    ID_BASE + DpWriter::OPCODE_CLEAR_EVENT_THROTTLE,
                    2,
                    CmdResponse::FormatError
                ),
            ]
        );
    }

    #[test]
    fn the_buffer_send_envelope_is_byte_exact() {
        let h = Harness::new("envelope");
        let port = h.comp.buffer_send_in(2);
        port.target.invoke(port.port_num, Buffer::allocate(65));
        let mut dest = [0u8; QUEUE_MSG_SIZE as usize];
        let mut size: FwSizeType = 0;
        let mut priority: FwQueuePriorityType = 0;
        let status = h.comp.active.queued.queue().receive(
            &mut dest,
            fprime_os::queue::BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        assert_eq!(size, 14);
        assert_eq!(&dest[..6], &[0x00, 0x00, 0x00, 0x02, 0x00, 0x02]);
    }
}
