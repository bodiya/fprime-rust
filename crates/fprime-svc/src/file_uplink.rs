//! # Svc::FileUplink — file receive component (ACTIVE)
//!
//! Port of `Svc/FileUplink/{FileUplink,File,Warnings}.cpp` and
//! `FileUplink.fpp` per `docs/cpp-analysis/file-services.md`
//! (§ "Svc::FileUplink" + the FileUplink gotcha list).
//!
//! One `Fw::Buffer` in on the async `bufferSendIn` port carries exactly one
//! com packet: `[u16 descriptor = FW_PACKET_FILE (0x0003)]` followed by an
//! [`fprime_fw::file_packet::FilePacket`]. A two-state receive machine
//! (`START` → `DATA` → back to `START`) writes DATA payloads into a
//! sandboxed OS file at their absolute offsets, accumulates the
//! [`Checksum`] as it writes, and verifies it against the END packet.
//!
//! **The input buffer is returned on `bufferSendOut` in EVERY path** —
//! too-small buffer, wrong descriptor, decode error, and every packet type.
//!
//! Ported quirks (all from the analysis gotcha list, all covered by tests):
//!
//! - a DATA packet in `START` mode logs `InvalidReceiveMode` and RETURNS
//!   with the mode unchanged (the sdd says "go to START"; the code does not);
//! - duplicate suppression is a single-slot `seq == last_sequence_index`
//!   check that is bypassed entirely when the previous write did NOT
//!   succeed — that is the deliberate retry path;
//! - `check_sequence_index` ALWAYS stores the new index, even when it logs
//!   `PacketOutOfOrder`, so a gap resynchronizes instead of aborting;
//! - `PacketOutOfBounds` returns WITHOUT updating the last write status, so
//!   the previous packet's status still governs duplicate suppression;
//! - a START packet clears FOUR throttles (`FileWriteError`,
//!   `InvalidReceiveMode`, `PacketOutOfBounds`, `PacketOutOfOrder`) but NOT
//!   `PacketDuplicate`, whose throttle is never cleared;
//! - `go_to_data_mode` does not close the file; only `go_to_start_mode`
//!   does, and a second START while in DATA mode closes it explicitly first;
//! - the `Warnings` counter is bumped by every warning helper, including
//!   ones whose event was throttled away.
//!
//! Locking: the C++ component has no mutex at all — `bufferSendIn` and
//! `pingIn` are both async, so all state lives on the component thread. The
//! Rust port needs interior mutability for `&self` handlers, so the state
//! sits in a `Mutex<UplinkState>` that the handler holds for its whole
//! duration (including event/telemetry emission). Since no other thread ever
//! takes it, that is observably identical to the C++ and keeps the
//! event/telemetry interleaving exactly as C++ emits it.
//!
//! The destination file is an [`fprime_os::SandboxedFile`] (the
//! `Os::SandboxedFile` / `Os::FilePathUtils` port now lives in `fprime-os`
//! and is shared with `Svc::FileDownlink` and `Svc::PrmDb`).

use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferSendPort, ComponentDispatch, EventGlue, EventThrottle,
    MsgDispatchStatus, PingPort, PortRef, QueueFullPolicy, TlmGlue, msg,
};
use fprime_config::{
    FwChanIdType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType, FwPacketDescriptorType,
    FwQueuePriorityType, FwSizeType,
};
use fprime_fw::file_packet::{
    DataPacket, EndPacket, FilePacket, FilePacketType, PATH_NAME_MAX_LENGTH, StartPacket,
};
use fprime_fw::{
    Buffer, ComPacketType, Endianness, FileNameString, LinearBuffer, LogSeverity, LogStringArg,
    SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try,
};
use fprime_os::SandboxedFile;
use fprime_os::file::{Mode, SeekType, Status as FileStatus, WaitType};
use fprime_utils::cfdp::Checksum;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Svc.FileAnnounce port
// ---------------------------------------------------------------------------

/// `Svc.FileAnnounce` — `port FileAnnounce(ref file_name: string size 240)`.
///
/// Declared here because `fprime-comp` carries only the framework port
/// traits; this is a `Svc/Ports/FilePorts` port.
pub trait FileAnnouncePort: Send + Sync {
    /// Announce a successfully uplinked file.
    fn invoke(&self, port_num: FwIndexType, file_name: &mut FileNameString);
}

// ---------------------------------------------------------------------------
// Dictionary constants
// ---------------------------------------------------------------------------

/// `bufferSendIn` async message type.
const MSG_TYPE_BUFFER_SEND_IN: FwEnumStoreType = 1;
/// `pingIn` async message type.
const MSG_TYPE_PING_IN: FwEnumStoreType = 2;

/// Queue message size = max over the async invocations:
/// `bufferSendIn` = 6 (envelope) + 8 (buffer escrow token, the slot C++ uses
/// for the `Fw::Buffer` pointer) = 14; `pingIn` = 6 + 4 = 10.
pub const QUEUE_MSG_SIZE: FwSizeType = 14;

/// Queue priority for both async ports (FPP declares no `priority`).
const PORT_PRIORITY: FwQueuePriorityType = 1;

/// Event string arguments are `string size 40`.
const EVENT_STRING_SIZE: usize = 40;

type MsgBuffer = LinearBuffer<{ QUEUE_MSG_SIZE as usize }>;

/// `FileUplink::ReceiveMode`.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReceiveMode {
    /// Waiting for a START packet.
    Start = 0,
    /// Receiving DATA packets.
    Data = 1,
}

/// `FileUplink::File` — the file currently being received.
struct UplinkFile {
    /// The declared file size from the START packet.
    size: u32,
    /// The destination file name (event arg source).
    name: LogStringArg,
    /// The sandboxed OS file.
    os_file: SandboxedFile,
    /// The CFDP checksum accumulated over the written bytes.
    checksum: Checksum,
}

impl UplinkFile {
    fn new() -> Self {
        Self {
            size: 0,
            name: LogStringArg::new(),
            os_file: SandboxedFile::new(),
            checksum: Checksum::new(),
        }
    }

    /// C++ `FileUplink::File::open`. The BAD_SIZE branch mirrors the C++
    /// `length >= sizeof(path)` guard; it is unreachable because a
    /// `PathName` length is a `U8` capped at 255 and the C++ buffer is 256
    /// bytes — kept for parity.
    fn open(&mut self, start: &StartPacket<'_>) -> FileStatus {
        let dest = start.destination_path;
        if dest.len() > PATH_NAME_MAX_LENGTH {
            return FileStatus::BadSize;
        }
        // C++ builds a NUL-terminated char[] and constructs the LogStringArg
        // from it; set_bytes has the same C-string semantics.
        self.name.set_bytes(dest);
        self.size = start.file_size;
        self.checksum = Checksum::new();
        // Rust file APIs take &str; a non-UTF-8 destination path cannot be
        // opened (C++ passes the raw bytes to open(2)).
        match core::str::from_utf8(dest) {
            Ok(path) => self.os_file.open(path, Mode::OpenWrite),
            Err(_) => FileStatus::BadSize,
        }
    }

    /// C++ `FileUplink::File::write`: absolute seek, non-blocking write,
    /// short write is `NO_SPACE`, then the checksum update.
    fn write(&mut self, data: &[u8], byte_offset: u32) -> FileStatus {
        let status = self
            .os_file
            .seek(i64::from(byte_offset), SeekType::Absolute);
        if status != FileStatus::OpOk {
            return status;
        }
        let mut written: FwSizeType = 0;
        // Note: not waiting for the file write to finish (C++ NO_WAIT).
        let status = self.os_file.write(data, &mut written, WaitType::NoWait);
        if status != FileStatus::OpOk {
            return status;
        }
        if written as usize != data.len() {
            return FileStatus::NoSpace;
        }
        self.checksum.update(data, byte_offset);
        FileStatus::OpOk
    }
}

/// Mutable component state (see the module header on locking).
struct UplinkState {
    receive_mode: ReceiveMode,
    last_sequence_index: u32,
    /// C++ `m_lastPacketWriteStatus`, whose sentinel is
    /// `Os::File::MAX_STATUS`; `None` is that sentinel here.
    last_packet_write_status: Option<FileStatus>,
    file: UplinkFile,
    files_received: u32,
    files_received_failed: u32,
    packets_received: u32,
    warnings: u32,
}

impl UplinkState {
    fn new() -> Self {
        Self {
            receive_mode: ReceiveMode::Start,
            last_sequence_index: 0,
            last_packet_write_status: None,
            file: UplinkFile::new(),
            files_received: 0,
            files_received_failed: 0,
            packets_received: 0,
            warnings: 0,
        }
    }
}

/// `Svc::FileUplink` — the active file-receive component.
pub struct FileUplink {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `bufferSendOut` — returns every received buffer.
    pub buffer_send_out: fprime_comp::OutputPort<dyn BufferSendPort>,
    /// `pingOut` — echoes the ping key.
    pub ping_out: fprime_comp::OutputPort<dyn PingPort>,
    /// `fileAnnounce` — optional; invoked only when connected.
    pub file_announce: fprime_comp::OutputPort<dyn FileAnnouncePort>,
    /// Escrow for the async buffer-carrying port.
    escrow: BufferEscrow,
    file_write_error_throttle: EventThrottle,
    invalid_receive_mode_throttle: EventThrottle,
    packet_out_of_bounds_throttle: EventThrottle,
    packet_out_of_order_throttle: EventThrottle,
    packet_duplicate_throttle: EventThrottle,
    state: Mutex<UplinkState>,
}

impl FileUplink {
    // -- Events (FPP-relative ids) -----------------------------------------

    /// `BadChecksum(fileName: string 40, computed: U32, read: U32)` —
    /// WARNING_HI, id 0.
    pub const EVENTID_BAD_CHECKSUM: FwEventIdType = 0;
    /// `FileOpenError(fileName: string 40)` — WARNING_HI, id 1.
    pub const EVENTID_FILE_OPEN_ERROR: FwEventIdType = 1;
    /// `FileReceived(fileName: string 40)` — ACTIVITY_HI, id 2.
    pub const EVENTID_FILE_RECEIVED: FwEventIdType = 2;
    /// `FileWriteError(fileName: string 40)` — WARNING_HI, id 3,
    /// `throttle 5`.
    pub const EVENTID_FILE_WRITE_ERROR: FwEventIdType = 3;
    /// `InvalidReceiveMode(packetType: FwPacketDescriptorType, mode: U32)` —
    /// WARNING_HI, id 4, `throttle 5`.
    pub const EVENTID_INVALID_RECEIVE_MODE: FwEventIdType = 4;
    /// `PacketOutOfBounds(packetIndex: U32, fileName: string 40)` —
    /// WARNING_HI, id 5, `throttle 5`.
    pub const EVENTID_PACKET_OUT_OF_BOUNDS: FwEventIdType = 5;
    /// `PacketOutOfOrder(packetIndex: U32, lastPacketIndex: U32)` —
    /// WARNING_HI, id 6, `throttle 20`.
    pub const EVENTID_PACKET_OUT_OF_ORDER: FwEventIdType = 6;
    /// `PacketDuplicate(packetIndex: U32)` — WARNING_HI, id 7,
    /// `throttle 20`.
    pub const EVENTID_PACKET_DUPLICATE: FwEventIdType = 7;
    /// `UplinkCanceled` — ACTIVITY_HI, id 8.
    pub const EVENTID_UPLINK_CANCELED: FwEventIdType = 8;
    /// `DecodeError(status: I32)` — WARNING_HI, id 9.
    pub const EVENTID_DECODE_ERROR: FwEventIdType = 9;
    /// `InvalidPacketReceived(packetType: FwPacketDescriptorType)` —
    /// WARNING_HI, id 10.
    pub const EVENTID_INVALID_PACKET_RECEIVED: FwEventIdType = 10;

    /// FPP `throttle 5` (FileWriteError, InvalidReceiveMode,
    /// PacketOutOfBounds).
    pub const THROTTLE_5: u32 = 5;
    /// FPP `throttle 20` (PacketOutOfOrder, PacketDuplicate).
    pub const THROTTLE_20: u32 = 20;

    // -- Telemetry ---------------------------------------------------------

    /// `FilesReceived: U32` — id 0.
    pub const CHANID_FILES_RECEIVED: FwChanIdType = 0;
    /// `PacketsReceived: U32` — id 1.
    pub const CHANID_PACKETS_RECEIVED: FwChanIdType = 1;
    /// `Warnings: U32` — id 2.
    pub const CHANID_WARNINGS: FwChanIdType = 2;
    /// `FilesReceivedFailed: U32` — id 3.
    pub const CHANID_FILES_RECEIVED_FAILED: FwChanIdType = 3;

    /// Construct the component (topology step 1).
    #[must_use]
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            buffer_send_out: fprime_comp::OutputPort::new(),
            ping_out: fprime_comp::OutputPort::new(),
            file_announce: fprime_comp::OutputPort::new(),
            escrow: BufferEscrow::new(),
            file_write_error_throttle: EventThrottle::new(Self::THROTTLE_5),
            invalid_receive_mode_throttle: EventThrottle::new(Self::THROTTLE_5),
            packet_out_of_bounds_throttle: EventThrottle::new(Self::THROTTLE_5),
            packet_out_of_order_throttle: EventThrottle::new(Self::THROTTLE_20),
            packet_duplicate_throttle: EventThrottle::new(Self::THROTTLE_20),
            state: Mutex::new(UplinkState::new()),
        })
    }

    /// Create the message queue (topology step "configure").
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active.queued.create_queue(queue_depth, QUEUE_MSG_SIZE);
    }

    /// C++ `FileUplink::configure(directory)`: restrict uplink writes to
    /// `directory`. Optional — the default sandbox is `/` (fail open).
    pub fn configure(&self, directory: &str) {
        self.state.lock().unwrap().file.os_file.configure(directory);
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    // -- Input-port factories ---------------------------------------------

    /// `bufferSendIn` — ASYNC `Fw.BufferSend` input (one com packet per
    /// buffer). The owned `Buffer` rides through the escrow, occupying the
    /// same 8 message bytes C++ uses for the pointer.
    pub fn buffer_send_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(
            Arc::new(BufferSendInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `pingIn` — ASYNC `Svc.Ping` input.
    pub fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers (component thread) ---------------------------------------

    /// `pingIn_handler`: echo the key.
    fn ping_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `bufferSendIn_handler`. The buffer is returned on `bufferSendOut` in
    /// every path (C++ parity — never leak it).
    fn buffer_send_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        const DESCRIPTOR_SIZE: usize = size_of::<FwPacketDescriptorType>();
        if buffer.size() < DESCRIPTOR_SIZE {
            self.log_invalid_packet_received(ComPacketType::FwPacketUnknown.as_repr());
            self.return_buffer(buffer);
            return;
        }
        let data = buffer.data();
        let packet_type = u16::from_be_bytes([data[0], data[1]]);
        if packet_type != ComPacketType::FwPacketFile.as_repr() {
            self.log_invalid_packet_received(packet_type);
            self.return_buffer(buffer);
            return;
        }
        match FilePacket::from_buffer(&data[DESCRIPTOR_SIZE..]) {
            Err(status) => {
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_DECODE_ERROR,
                    LogSeverity::WarningHi,
                    &format!("Unable to decode file packet. Status: {}", status as i32),
                    |buf| buf.serialize_i32_be(status as i32),
                );
            }
            Ok(packet) => {
                let mut state = self.state.lock().unwrap();
                match packet {
                    FilePacket::Start(p) => self.handle_start_packet(&mut state, &p),
                    FilePacket::Data(p) => self.handle_data_packet(&mut state, &p),
                    FilePacket::End(p) => self.handle_end_packet(&mut state, &p),
                    FilePacket::Cancel(_) => self.handle_cancel_packet(&mut state),
                }
            }
        }
        self.return_buffer(buffer);
    }

    fn return_buffer(&self, buffer: Buffer) {
        let p = self.buffer_send_out.get();
        p.target.invoke(p.port_num, buffer);
    }

    /// C++ `handleStartPacket`.
    fn handle_start_packet(&self, state: &mut UplinkState, start: &StartPacket<'_>) {
        // Clear all event throttles in preparation for a new start packet —
        // FOUR of the five; PacketDuplicate is deliberately NOT cleared.
        self.file_write_error_throttle.clear();
        self.invalid_receive_mode_throttle.clear();
        self.packet_out_of_bounds_throttle.clear();
        self.packet_out_of_order_throttle.clear();
        self.packet_received(state);
        if state.receive_mode != ReceiveMode::Start {
            state.file.os_file.close();
            self.warning_invalid_receive_mode(state, FilePacketType::Start);
        }
        let status = state.file.open(start);
        if status == FileStatus::OpOk {
            self.go_to_data_mode(state);
        } else {
            self.warning_file_open(state);
            self.go_to_start_mode(state);
        }
    }

    /// C++ `handleDataPacket`.
    fn handle_data_packet(&self, state: &mut UplinkState, packet: &DataPacket<'_>) {
        self.packet_received(state);
        // GOTCHA: the mode is NOT changed here (the sdd says it is).
        if state.receive_mode != ReceiveMode::Data {
            self.warning_invalid_receive_mode(state, FilePacketType::Data);
            return;
        }

        let sequence_index = packet.header.sequence_index;

        // Skip a duplicate ONLY when the previous packet was written
        // successfully; a failed write leaves the retry path open. The C++
        // `&&` short-circuits, so no PacketDuplicate warning is issued when
        // the previous write failed.
        if state.last_packet_write_status == Some(FileStatus::OpOk)
            && self.check_duplicated_packet(state, sequence_index)
        {
            return;
        }

        self.check_sequence_index(state, sequence_index);
        let byte_offset = packet.byte_offset;
        let data_size = u32::from(packet.data_size);
        if (u32::MAX - byte_offset < data_size) || (byte_offset + data_size > state.file.size) {
            self.warning_packet_out_of_bounds(state, sequence_index);
            // GOTCHA: last_packet_write_status is NOT updated here.
            return;
        }
        let status = state.file.write(packet.data, byte_offset);
        if status != FileStatus::OpOk {
            self.warning_file_write(state);
        }
        state.last_packet_write_status = Some(status);
    }

    /// C++ `handleEndPacket`.
    fn handle_end_packet(&self, state: &mut UplinkState, end: &EndPacket) {
        self.packet_received(state);
        if state.receive_mode == ReceiveMode::Data {
            self.check_sequence_index(state, end.header.sequence_index);
            if self.compare_checksums(state, end) {
                state.files_received += 1;
                let count = state.files_received;
                self.tlm_write(Self::CHANID_FILES_RECEIVED, count);
                let name = state.file.name.clone();
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_FILE_RECEIVED,
                    LogSeverity::ActivityHi,
                    &format!("Received file {name}"),
                    |buf| name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
                );
                if let Some(p) = self.file_announce.try_get() {
                    let mut announced = FileNameString::new();
                    announced.set_bytes(state.file.name.as_bytes());
                    p.target.invoke(p.port_num, &mut announced);
                }
            } else {
                // compareChecksums already issued BadChecksum: the file is
                // neither announced nor counted as received.
                state.files_received_failed += 1;
                let count = state.files_received_failed;
                self.tlm_write(Self::CHANID_FILES_RECEIVED_FAILED, count);
            }
        } else {
            self.warning_invalid_receive_mode(state, FilePacketType::End);
        }
        self.go_to_start_mode(state);
    }

    /// C++ `handleCancelPacket`: no mode check — always closes the file.
    fn handle_cancel_packet(&self, state: &mut UplinkState) {
        self.packet_received(state);
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_UPLINK_CANCELED,
            LogSeverity::ActivityHi,
            "Received CANCEL packet",
            |_buf| SerializeStatus::Ok,
        );
        self.go_to_start_mode(state);
    }

    /// C++ `checkSequenceIndex`: warn on a gap but ALWAYS store the index.
    fn check_sequence_index(&self, state: &mut UplinkState, sequence_index: u32) {
        if sequence_index != state.last_sequence_index.wrapping_add(1) {
            self.warning_packet_out_of_order(state, sequence_index);
        }
        state.last_sequence_index = sequence_index;
    }

    /// C++ `checkDuplicatedPacket`: warns and returns true on a match.
    fn check_duplicated_packet(&self, state: &mut UplinkState, sequence_index: u32) -> bool {
        if sequence_index == state.last_sequence_index {
            self.warning_packet_duplicate(state, sequence_index);
            return true;
        }
        false
    }

    /// C++ `compareChecksums`: issues BadChecksum itself on a mismatch.
    fn compare_checksums(&self, state: &mut UplinkState, end: &EndPacket) -> bool {
        let computed = state.file.checksum.get_value();
        let stored = end.checksum_value;
        if computed != stored {
            self.warning_bad_checksum(state, computed, stored);
            return false;
        }
        true
    }

    fn go_to_start_mode(&self, state: &mut UplinkState) {
        state.file.os_file.close();
        state.receive_mode = ReceiveMode::Start;
        state.last_sequence_index = 0;
        state.last_packet_write_status = None;
    }

    /// C++ `goToDataMode`: does NOT close the file.
    fn go_to_data_mode(&self, state: &mut UplinkState) {
        state.receive_mode = ReceiveMode::Data;
        state.last_sequence_index = 0;
        state.last_packet_write_status = None;
    }

    // -- Counters, warnings, events ---------------------------------------

    fn tlm_write(&self, chan: FwChanIdType, value: u32) {
        self.tlm
            .tlm_write(self.id_base(), chan, &value, self.evt.time_get());
    }

    /// C++ `PacketsReceived::packetReceived()`: increment then write.
    fn packet_received(&self, state: &mut UplinkState) {
        state.packets_received += 1;
        let count = state.packets_received;
        self.tlm_write(Self::CHANID_PACKETS_RECEIVED, count);
    }

    /// C++ `Warnings::warning()`: bumped by EVERY warning helper, even when
    /// the event itself was throttled away.
    fn warning(&self, state: &mut UplinkState) {
        state.warnings += 1;
        let count = state.warnings;
        self.tlm_write(Self::CHANID_WARNINGS, count);
    }

    fn log_invalid_packet_received(&self, packet_type: u16) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_INVALID_PACKET_RECEIVED,
            LogSeverity::WarningHi,
            &format!("Invalid packet received. Wrong packet type: {packet_type}"),
            |buf| buf.serialize_u16_be(packet_type),
        );
    }

    fn warning_invalid_receive_mode(&self, state: &mut UplinkState, packet_type: FilePacketType) {
        let mode = state.receive_mode as u32;
        let type_value = FwPacketDescriptorType::from(packet_type.as_repr());
        if self.invalid_receive_mode_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_INVALID_RECEIVE_MODE,
                LogSeverity::WarningHi,
                &format!("Packet type {type_value} received in mode {mode}"),
                |buf| {
                    fw_try!(buf.serialize_u16_be(type_value));
                    buf.serialize_u32_be(mode)
                },
            );
        }
        self.warning(state);
    }

    fn warning_file_open(&self, state: &mut UplinkState) {
        let name = state.file.name.clone();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_OPEN_ERROR,
            LogSeverity::WarningHi,
            &format!("Could not open file {name}"),
            |buf| name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
        );
        self.warning(state);
    }

    fn warning_file_write(&self, state: &mut UplinkState) {
        if self.file_write_error_throttle.ok_to_emit() {
            let name = state.file.name.clone();
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_FILE_WRITE_ERROR,
                LogSeverity::WarningHi,
                &format!("Could not write to file {name}"),
                |buf| name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
            );
        }
        self.warning(state);
    }

    fn warning_packet_out_of_bounds(&self, state: &mut UplinkState, sequence_index: u32) {
        if self.packet_out_of_bounds_throttle.ok_to_emit() {
            let name = state.file.name.clone();
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_PACKET_OUT_OF_BOUNDS,
                LogSeverity::WarningHi,
                &format!("Packet {sequence_index} out of bounds for file {name}"),
                |buf| {
                    fw_try!(buf.serialize_u32_be(sequence_index));
                    name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
                },
            );
        }
        self.warning(state);
    }

    fn warning_packet_out_of_order(&self, state: &mut UplinkState, sequence_index: u32) {
        let last = state.last_sequence_index;
        if self.packet_out_of_order_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_PACKET_OUT_OF_ORDER,
                LogSeverity::WarningHi,
                &format!("Received packet {sequence_index} after packet {last}"),
                |buf| {
                    fw_try!(buf.serialize_u32_be(sequence_index));
                    buf.serialize_u32_be(last)
                },
            );
        }
        self.warning(state);
    }

    fn warning_packet_duplicate(&self, state: &mut UplinkState, sequence_index: u32) {
        if self.packet_duplicate_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_PACKET_DUPLICATE,
                LogSeverity::WarningHi,
                &format!("Received a duplicate of packet {sequence_index}"),
                |buf| buf.serialize_u32_be(sequence_index),
            );
        }
        self.warning(state);
    }

    fn warning_bad_checksum(&self, state: &mut UplinkState, computed: u32, read: u32) {
        let name = state.file.name.clone();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BAD_CHECKSUM,
            LogSeverity::WarningHi,
            &format!(
                "Bad checksum value during receipt of file {name}: computed 0x{computed:x}, read 0x{read:x}"
            ),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(computed));
                buf.serialize_u32_be(read)
            },
        );
        self.warning(state);
    }
}

// -- Async input adapters ---------------------------------------------------

/// `bufferSendIn` adapter. The buffer is deposited in the escrow and the
/// token serialized where C++ serializes the `Fw::Buffer` pointer.
struct BufferSendInAdapter {
    comp: Arc<FileUplink>,
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

/// `pingIn` adapter.
struct PingInAdapter {
    comp: Arc<FileUplink>,
}

impl PingPort for PingInAdapter {
    fn invoke(&self, port_num: FwIndexType, key: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_PING_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(key);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

// -- Dispatch ---------------------------------------------------------------

impl ComponentDispatch for FileUplink {
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
            MSG_TYPE_BUFFER_SEND_IN => {
                let mut token = 0u64;
                if !buf.deserialize_u64_be(&mut token).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.buffer_send_in_handler(port_num, buffer);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_PING_IN => {
                let mut key = 0u32;
                if !buf.deserialize_u32_be(&mut key).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.ping_handler(port_num, key);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for FileUplink {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::queued::MsgDispatchStatus as DispatchStatus;
    use fprime_comp::{LogPort, TlmPort};
    use fprime_fw::file_packet::CancelPacket;
    use fprime_fw::{LogBuffer, Time, TlmBuffer};
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    const ID_BASE: FwIdType = 0x0500_0000;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    fn temp_dir(tag: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fprime_rust_uplink_{tag}_{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        returned: Mutex<Vec<Vec<u8>>>,
        pings: Mutex<Vec<u32>>,
        announced: Mutex<Vec<String>>,
    }

    impl Ground {
        fn event_ids(&self) -> Vec<FwEventIdType> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .map(|e| e.0 - ID_BASE)
                .collect()
        }
        fn events_of(&self, id: FwEventIdType) -> Vec<(LogSeverity, Vec<u8>)> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == ID_BASE + id)
                .map(|e| (e.1, e.2.clone()))
                .collect()
        }
        fn last_tlm(&self, chan: FwChanIdType) -> Option<u32> {
            self.tlm
                .lock()
                .unwrap()
                .iter()
                .rfind(|e| e.0 == ID_BASE + chan)
                .map(|e| u32::from_be_bytes([e.1[0], e.1[1], e.1[2], e.1[3]]))
        }
    }

    impl LogPort for Ground {
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

    impl TlmPort for Ground {
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

    impl BufferSendPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.returned.lock().unwrap().push(buffer.data().to_vec());
        }
    }

    impl PingPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push(key);
        }
    }

    impl FileAnnouncePort for Ground {
        fn invoke(&self, _port_num: FwIndexType, file_name: &mut FileNameString) {
            self.announced
                .lock()
                .unwrap()
                .push(file_name.as_str().unwrap_or("").to_string());
        }
    }

    fn setup(announce: bool) -> (Arc<FileUplink>, Arc<Ground>) {
        let comp = FileUplink::new("fileUplink");
        let ground = Arc::new(Ground::default());
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.buffer_send_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        if announce {
            comp.file_announce.connect(ground.clone(), 0);
        }
        comp.init(64);
        (comp, ground)
    }

    /// `[u16 descriptor][file packet]` in an owned buffer.
    fn com_buffer(descriptor: u16, packet: &FilePacket<'_>) -> Buffer {
        let mut buffer = Buffer::from_storage(vec![0u8; 1024].into_boxed_slice(), 0);
        let size = packet.buffer_size() + 2;
        {
            let mut ser = buffer.get_serializer();
            assert_eq!(ser.serialize_u16_be(descriptor), SerializeStatus::Ok);
            assert_eq!(packet.serialize_to(&mut ser), SerializeStatus::Ok);
        }
        buffer.set_size(size);
        buffer
    }

    fn raw_buffer(bytes: &[u8]) -> Buffer {
        let mut storage = vec![0u8; bytes.len().max(1)].into_boxed_slice();
        storage[..bytes.len()].copy_from_slice(bytes);
        let mut buffer = Buffer::from_storage(storage, 0);
        buffer.set_size(bytes.len());
        buffer
    }

    fn deliver(comp: &Arc<FileUplink>, buffer: Buffer) {
        let p = comp.buffer_send_in(0);
        p.target.invoke(p.port_num, buffer);
        let status = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        assert!(status == DispatchStatus::Ok || status == DispatchStatus::Empty);
    }

    fn send_packet(comp: &Arc<FileUplink>, packet: &FilePacket<'_>) {
        deliver(comp, com_buffer(0x0003, packet));
    }

    fn start(dest: &str, size: u32) -> Vec<u8> {
        let packet = FilePacket::Start(StartPacket::initialize(size, b"src", dest.as_bytes()));
        let mut out = vec![0u8; packet.buffer_size()];
        let mut ext = fprime_fw::ExtBuf::new(&mut out);
        assert_eq!(packet.serialize_to(&mut ext), SerializeStatus::Ok);
        out
    }

    // ---- happy path -------------------------------------------------------

    #[test]
    fn uplink_receives_a_file_and_announces_it() {
        let (comp, ground) = setup(true);
        let dir = temp_dir("happy");
        let dest = dir.join("received.bin");
        let dest_str = dest.to_str().unwrap();
        let content: Vec<u8> = (0u8..200).collect();
        let mut checksum = Checksum::new();
        checksum.update(&content, 0);

        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                content.len() as u32,
                b"ground/src.bin",
                dest_str.as_bytes(),
            )),
        );
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 200, &content)),
        );
        send_packet(
            &comp,
            &FilePacket::End(EndPacket::initialize(2, checksum.get_value())),
        );

        assert_eq!(std::fs::read(&dest).unwrap(), content);
        assert_eq!(ground.last_tlm(FileUplink::CHANID_FILES_RECEIVED), Some(1));
        assert_eq!(
            ground.last_tlm(FileUplink::CHANID_PACKETS_RECEIVED),
            Some(3)
        );
        assert_eq!(ground.last_tlm(FileUplink::CHANID_WARNINGS), None);
        assert_eq!(ground.event_ids(), vec![FileUplink::EVENTID_FILE_RECEIVED]);
        assert_eq!(
            ground.events_of(FileUplink::EVENTID_FILE_RECEIVED)[0].0,
            LogSeverity::ActivityHi
        );
        assert_eq!(ground.announced.lock().unwrap().as_slice(), &[dest_str]);
        // Every buffer came back.
        assert_eq!(ground.returned.lock().unwrap().len(), 3);
    }

    #[test]
    fn multi_packet_file_is_reassembled_from_absolute_offsets() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("multi");
        let dest = dir.join("multi.bin");
        let content: Vec<u8> = (0u8..=255).cycle().take(1000).collect();
        let mut checksum = Checksum::new();
        checksum.update(&content, 0);

        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                1000,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
        );
        let mut seq = 1u32;
        let mut offset = 0usize;
        while offset < content.len() {
            let end = (offset + 499).min(content.len());
            send_packet(
                &comp,
                &FilePacket::Data(DataPacket::initialize(
                    seq,
                    offset as u32,
                    (end - offset) as u16,
                    &content[offset..end],
                )),
            );
            seq += 1;
            offset = end;
        }
        send_packet(
            &comp,
            &FilePacket::End(EndPacket::initialize(seq, checksum.get_value())),
        );

        assert_eq!(std::fs::read(&dest).unwrap(), content);
        assert_eq!(ground.last_tlm(FileUplink::CHANID_FILES_RECEIVED), Some(1));
        assert_eq!(ground.event_ids(), vec![FileUplink::EVENTID_FILE_RECEIVED]);
    }

    // ---- buffer discipline / bad packets ---------------------------------

    #[test]
    fn too_small_buffer_is_returned_with_unknown_packet_type() {
        let (comp, ground) = setup(false);
        deliver(&comp, raw_buffer(&[0x00]));
        let events = ground.events_of(FileUplink::EVENTID_INVALID_PACKET_RECEIVED);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, LogSeverity::WarningHi);
        // FW_PACKET_UNKNOWN = 0x00FF
        assert_eq!(events[0].1, vec![0x00, 0xFF]);
        assert_eq!(ground.returned.lock().unwrap().len(), 1);
        // InvalidPacketReceived is NOT a Warnings-counter warning.
        assert_eq!(ground.last_tlm(FileUplink::CHANID_WARNINGS), None);
    }

    #[test]
    fn wrong_descriptor_is_returned_with_that_descriptor() {
        let (comp, ground) = setup(false);
        deliver(&comp, raw_buffer(&[0x00, 0x01, 0xAA]));
        let events = ground.events_of(FileUplink::EVENTID_INVALID_PACKET_RECEIVED);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].1, vec![0x00, 0x01]);
        assert_eq!(ground.returned.lock().unwrap().len(), 1);
    }

    #[test]
    fn decode_error_logs_the_status_and_still_returns_the_buffer() {
        let (comp, ground) = setup(false);
        // A START packet with one residual byte: DeserSizeMismatch (5).
        let mut bytes = vec![0x00, 0x03];
        bytes.extend_from_slice(&start("/tmp/x", 4));
        bytes.push(0xFF);
        deliver(&comp, raw_buffer(&bytes));
        let events = ground.events_of(FileUplink::EVENTID_DECODE_ERROR);
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].1,
            (SerializeStatus::DeserSizeMismatch as i32)
                .to_be_bytes()
                .to_vec()
        );
        assert_eq!(ground.returned.lock().unwrap().len(), 1);
        // No packet was dispatched, so PacketsReceived never moved.
        assert_eq!(ground.last_tlm(FileUplink::CHANID_PACKETS_RECEIVED), None);
    }

    #[test]
    fn malformed_packet_type_reports_the_cpp_decode_status() {
        // C++ FilePacket::fromSerialBuffer: `default:` -> INVALID_DATA (8),
        // `case T_NONE:` -> TYPE_MISMATCH (6). Both are ground-visible in the
        // DecodeError event argument.
        for (raw_type, expected) in [
            (0x04u8, SerializeStatus::DeserInvalidData),
            (0xFF, SerializeStatus::DeserTypeMismatch),
        ] {
            let (comp, ground) = setup(false);
            let bytes = vec![0x00, 0x03, raw_type, 0x00, 0x00, 0x00, 0x00];
            deliver(&comp, raw_buffer(&bytes));
            let events = ground.events_of(FileUplink::EVENTID_DECODE_ERROR);
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].1, (expected as i32).to_be_bytes().to_vec());
            // The buffer still comes back on bufferSendOut (C++ parity).
            assert_eq!(ground.returned.lock().unwrap().len(), 1);
        }
    }

    // ---- receive-mode quirks ---------------------------------------------

    #[test]
    fn data_in_start_mode_warns_and_leaves_the_mode_unchanged() {
        let (comp, ground) = setup(false);
        let payload = [1u8, 2, 3];
        for _ in 0..2 {
            send_packet(
                &comp,
                &FilePacket::Data(DataPacket::initialize(1, 0, 3, &payload)),
            );
        }
        let events = ground.events_of(FileUplink::EVENTID_INVALID_RECEIVE_MODE);
        // GOTCHA: the mode is unchanged, so the SECOND packet warns too.
        assert_eq!(events.len(), 2);
        // [packetType u16 = T_DATA (1)][mode u32 = START (0)]
        assert_eq!(events[0].1, vec![0x00, 0x01, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(ground.last_tlm(FileUplink::CHANID_WARNINGS), Some(2));
    }

    #[test]
    fn end_in_start_mode_warns_with_the_end_type() {
        let (comp, ground) = setup(false);
        send_packet(&comp, &FilePacket::End(EndPacket::initialize(1, 0)));
        let events = ground.events_of(FileUplink::EVENTID_INVALID_RECEIVE_MODE);
        assert_eq!(events.len(), 1);
        // T_END = 2, mode START = 0
        assert_eq!(events[0].1, vec![0x00, 0x02, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn second_start_in_data_mode_warns_with_data_mode() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("restart");
        let dest = dir.join("f.bin");
        let dest_bytes = dest.to_str().unwrap().as_bytes().to_vec();
        for _ in 0..2 {
            send_packet(
                &comp,
                &FilePacket::Start(StartPacket::initialize(4, b"src", &dest_bytes)),
            );
        }
        let events = ground.events_of(FileUplink::EVENTID_INVALID_RECEIVE_MODE);
        assert_eq!(events.len(), 1);
        // T_START = 0, mode DATA = 1
        assert_eq!(events[0].1, vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x01]);
    }

    #[test]
    fn cancel_returns_to_start_mode_from_any_state() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("cancel");
        let dest = dir.join("f.bin");
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                4,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
        );
        send_packet(&comp, &FilePacket::Cancel(CancelPacket::initialize(1)));
        assert_eq!(
            ground.events_of(FileUplink::EVENTID_UPLINK_CANCELED).len(),
            1
        );
        // Back in START mode: a DATA packet now warns.
        let payload = [0u8; 4];
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &payload)),
        );
        assert_eq!(
            ground
                .events_of(FileUplink::EVENTID_INVALID_RECEIVE_MODE)
                .len(),
            1
        );
        assert_eq!(
            ground.last_tlm(FileUplink::CHANID_PACKETS_RECEIVED),
            Some(3)
        );
    }

    // ---- sequencing quirks ------------------------------------------------

    #[test]
    fn duplicate_after_a_successful_write_is_skipped() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("dup");
        let dest = dir.join("dup.bin");
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                4,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
        );
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &[1u8, 2, 3, 4])),
        );
        // Same sequence index, different payload: it must be dropped.
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &[9u8, 9, 9, 9])),
        );
        assert_eq!(std::fs::read(&dest).unwrap(), vec![1, 2, 3, 4]);
        assert_eq!(
            ground.events_of(FileUplink::EVENTID_PACKET_DUPLICATE).len(),
            1
        );
        assert_eq!(ground.last_tlm(FileUplink::CHANID_WARNINGS), Some(1));
    }

    #[test]
    fn duplicate_check_is_bypassed_when_the_previous_write_did_not_succeed() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("retry");
        let dest = dir.join("retry.bin");
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                4,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
        );
        // Out of bounds: warns, and does NOT update last_packet_write_status.
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 2, 4, &[7u8, 7, 7, 7])),
        );
        assert_eq!(
            ground
                .events_of(FileUplink::EVENTID_PACKET_OUT_OF_BOUNDS)
                .len(),
            1
        );
        // Same sequence index again: the duplicate check is bypassed because
        // the previous write never succeeded, so this one IS written.
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &[1u8, 2, 3, 4])),
        );
        assert_eq!(std::fs::read(&dest).unwrap(), vec![1, 2, 3, 4]);
        assert!(
            ground
                .events_of(FileUplink::EVENTID_PACKET_DUPLICATE)
                .is_empty()
        );
    }

    #[test]
    fn out_of_order_packet_warns_then_resynchronizes() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("order");
        let dest = dir.join("order.bin");
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                8,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
        );
        // Jump straight to sequence 5.
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(5, 0, 4, &[1u8, 2, 3, 4])),
        );
        let events = ground.events_of(FileUplink::EVENTID_PACKET_OUT_OF_ORDER);
        assert_eq!(events.len(), 1);
        // [packetIndex 5][lastPacketIndex 0]
        assert_eq!(events[0].1, vec![0, 0, 0, 5, 0, 0, 0, 0]);
        // The index resynchronized: 6 follows 5 with no further warning.
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(6, 4, 4, &[5u8, 6, 7, 8])),
        );
        assert_eq!(
            ground
                .events_of(FileUplink::EVENTID_PACKET_OUT_OF_ORDER)
                .len(),
            1
        );
        assert_eq!(std::fs::read(&dest).unwrap(), vec![1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn out_of_bounds_packet_is_rejected_both_ways() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("bounds");
        let dest = dir.join("b.bin");
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                4,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
        );
        // offset + size > declared file size
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 3, 4, &[1u8, 2, 3, 4])),
        );
        // offset + size wraps U32
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(2, u32::MAX - 1, 4, &[1u8, 2, 3, 4])),
        );
        let events = ground.events_of(FileUplink::EVENTID_PACKET_OUT_OF_BOUNDS);
        assert_eq!(events.len(), 2);
        assert_eq!(&events[0].1[..4], &[0, 0, 0, 1]);
        assert_eq!(std::fs::read(&dest).unwrap(), Vec::<u8>::new());
    }

    // ---- checksum ---------------------------------------------------------

    #[test]
    fn bad_checksum_reports_a_failed_file_and_does_not_announce() {
        let (comp, ground) = setup(true);
        let dir = temp_dir("badsum");
        let dest = dir.join("bad.bin");
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                4,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
        );
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &[1u8, 2, 3, 4])),
        );
        send_packet(&comp, &FilePacket::End(EndPacket::initialize(2, 0xDEAD)));
        let events = ground.events_of(FileUplink::EVENTID_BAD_CHECKSUM);
        assert_eq!(events.len(), 1);
        // [name string][computed u32][read u32]; computed = 0x01020304.
        let args = &events[0].1;
        assert_eq!(
            &args[args.len() - 8..],
            &[0x01, 0x02, 0x03, 0x04, 0, 0, 0xDE, 0xAD]
        );
        assert_eq!(
            ground.last_tlm(FileUplink::CHANID_FILES_RECEIVED_FAILED),
            Some(1)
        );
        assert_eq!(ground.last_tlm(FileUplink::CHANID_FILES_RECEIVED), None);
        assert!(
            ground
                .events_of(FileUplink::EVENTID_FILE_RECEIVED)
                .is_empty()
        );
        assert!(ground.announced.lock().unwrap().is_empty());
    }

    // ---- open errors ------------------------------------------------------

    #[test]
    fn file_open_error_warns_and_returns_to_start_mode() {
        let (comp, ground) = setup(false);
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                4,
                b"src",
                b"/nonexistent-directory-fprime/x.bin",
            )),
        );
        assert_eq!(
            ground.events_of(FileUplink::EVENTID_FILE_OPEN_ERROR).len(),
            1
        );
        assert_eq!(ground.last_tlm(FileUplink::CHANID_WARNINGS), Some(1));
        // Back in START mode.
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 1, &[0u8])),
        );
        assert_eq!(
            ground
                .events_of(FileUplink::EVENTID_INVALID_RECEIVE_MODE)
                .len(),
            1
        );
    }

    #[test]
    fn sandbox_rejects_a_destination_outside_the_configured_directory() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("sandbox");
        comp.configure(dir.to_str().unwrap());
        // Inside the sandbox: opens fine.
        let inside = dir.join("in.bin");
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(
                0,
                b"src",
                inside.to_str().unwrap().as_bytes(),
            )),
        );
        assert!(
            ground
                .events_of(FileUplink::EVENTID_FILE_OPEN_ERROR)
                .is_empty()
        );
        // Escaping with ".." is rejected textually.
        let outside = format!("{}/../escape.bin", dir.to_str().unwrap());
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(0, b"src", outside.as_bytes())),
        );
        assert_eq!(
            ground.events_of(FileUplink::EVENTID_FILE_OPEN_ERROR).len(),
            1
        );
    }

    // ---- throttles --------------------------------------------------------

    #[test]
    fn start_clears_four_throttles_but_never_the_duplicate_throttle() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("throttle");
        let dest = dir.join("t.bin");
        let dest_bytes = dest.to_str().unwrap().as_bytes().to_vec();
        let payload = [1u8, 2, 3, 4];

        // Six DATA packets in START mode: throttle 5 caps the events at 5
        // while the Warnings counter still counts all six.
        for _ in 0..6 {
            send_packet(
                &comp,
                &FilePacket::Data(DataPacket::initialize(1, 0, 4, &payload)),
            );
        }
        assert_eq!(
            ground
                .events_of(FileUplink::EVENTID_INVALID_RECEIVE_MODE)
                .len(),
            5
        );
        assert_eq!(ground.last_tlm(FileUplink::CHANID_WARNINGS), Some(6));

        // A START clears the InvalidReceiveMode throttle; END returns to
        // START mode so the next DATA warns again.
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(4, b"src", &dest_bytes)),
        );
        send_packet(&comp, &FilePacket::End(EndPacket::initialize(1, 0)));
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &payload)),
        );
        assert_eq!(
            ground
                .events_of(FileUplink::EVENTID_INVALID_RECEIVE_MODE)
                .len(),
            6
        );
    }

    #[test]
    fn duplicate_throttle_survives_a_start_packet() {
        let (comp, ground) = setup(false);
        let dir = temp_dir("duptl");
        let dest = dir.join("d.bin");
        let dest_bytes = dest.to_str().unwrap().as_bytes().to_vec();
        let payload = [1u8, 2, 3, 4];

        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(4, b"src", &dest_bytes)),
        );
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &payload)),
        );
        // 21 duplicates: throttle 20 emits 20 events.
        for _ in 0..21 {
            send_packet(
                &comp,
                &FilePacket::Data(DataPacket::initialize(1, 0, 4, &payload)),
            );
        }
        assert_eq!(
            ground.events_of(FileUplink::EVENTID_PACKET_DUPLICATE).len(),
            20
        );
        // A new START does NOT clear this throttle.
        send_packet(
            &comp,
            &FilePacket::Start(StartPacket::initialize(4, b"src", &dest_bytes)),
        );
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &payload)),
        );
        send_packet(
            &comp,
            &FilePacket::Data(DataPacket::initialize(1, 0, 4, &payload)),
        );
        assert_eq!(
            ground.events_of(FileUplink::EVENTID_PACKET_DUPLICATE).len(),
            20
        );
    }

    // ---- ping -------------------------------------------------------------

    #[test]
    fn ping_is_echoed_on_the_component_thread() {
        let (comp, ground) = setup(false);
        let p = comp.ping_in(0);
        p.target.invoke(p.port_num, 0xABCD_1234);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        assert_eq!(ground.pings.lock().unwrap().as_slice(), &[0xABCD_1234]);
    }

    // ---- active lifecycle -------------------------------------------------

    #[test]
    fn the_component_task_processes_a_transfer_end_to_end() {
        let (comp, ground) = setup(true);
        let dir = temp_dir("task");
        let dest = dir.join("task.bin");
        let content: Vec<u8> = (0u8..64).collect();
        let mut checksum = Checksum::new();
        checksum.update(&content, 0);

        comp.active.start(
            &comp,
            100,
            fprime_os::task::TASK_DEFAULT,
            fprime_os::task::TASK_DEFAULT,
        );

        for packet in [
            FilePacket::Start(StartPacket::initialize(
                content.len() as u32,
                b"src",
                dest.to_str().unwrap().as_bytes(),
            )),
            FilePacket::Data(DataPacket::initialize(1, 0, 64, &content)),
            FilePacket::End(EndPacket::initialize(2, checksum.get_value())),
        ] {
            let p = comp.buffer_send_in(0);
            p.target.invoke(p.port_num, com_buffer(0x0003, &packet));
        }

        // EXIT is priority 0 while the port messages are priority 1, so the
        // queued traffic drains before the task exits: join is a
        // deterministic sync point.
        comp.active.exit();
        assert_eq!(comp.active.join(), fprime_os::task::Status::OpOk);

        assert_eq!(std::fs::read(&dest).unwrap(), content);
        assert_eq!(ground.event_ids(), vec![FileUplink::EVENTID_FILE_RECEIVED]);
        assert_eq!(ground.returned.lock().unwrap().len(), 3);
    }

    #[test]
    fn queue_message_size_covers_the_largest_async_invocation() {
        // bufferSendIn: 6-byte envelope + 8-byte escrow token.
        assert_eq!(QUEUE_MSG_SIZE as usize, msg::ENVELOPE_HEADER_SIZE + 8);
        // pingIn is smaller.
        assert!(QUEUE_MSG_SIZE as usize >= msg::ENVELOPE_HEADER_SIZE + 4);
    }
}
