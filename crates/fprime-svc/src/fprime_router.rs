//! # FprimeRouter — port of `Svc::FprimeRouter` (passive, guarded)
//!
//! C++ sources: `Svc/FprimeRouter/FprimeRouter.{cpp,hpp,fpp}` and
//! `default/config/FprimeRouterCfg.fpp`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (FprimeRouter section).
//!
//! Routes deframed packets by `context.apid`: commands are COPIED into a
//! stack `ComBuffer` and sent on `commandOut` (buffer returned immediately);
//! file packets are handed off on `fileOut` (`Fw.BufferSend` — no context on
//! the wire, so the context is remembered in a bounded table and restored
//! when the buffer returns on `fileBufferReturnIn`); everything else goes to
//! `unknownDataOut` when connected, else straight back.
//!
//! ## Context-table divergence (documented)
//!
//! The C++ table keys on the buffer's data POINTER (`getData()`). Rust
//! buffers are owned values that move through ports, so pointer identity is
//! not observable. Instead the router stamps a generated token into the
//! buffer's `context` word before forwarding and keys the table on that
//! token; on return it restores the original context word and the saved
//! [`FrameContext`]. Observable behavior (table-full events, context
//! restoration, `BufferContextNotFound` + default context on a miss) is
//! identical to C++.

use fprime_comp::{
    BufferSendPort, CmdResponsePort, ComDataWithContextPort, ComPort, EventGlue, OutputPort,
    PassiveBase, PortRef,
};
use fprime_config::{FwEventIdType, FwIndexType, FwOpcodeType};
use fprime_fw::{Apid, Buffer, CmdResponse, ComBuffer, FrameContext, LogSeverity, SerBuf};
use std::sync::{Arc, Mutex};

/// `Svc::FprimeRouterCfg::BufferContextTableSize`.
pub const BUFFER_CONTEXT_TABLE_SIZE: usize = 50;

/// Relative event ids (FPP declaration order).
pub const EVENTID_SERIALIZATION_ERROR: FwEventIdType = 0;
pub const EVENTID_DESERIALIZATION_ERROR: FwEventIdType = 1;
pub const EVENTID_FILE_OUT_CONTEXT_TABLE_FULL: FwEventIdType = 2;
pub const EVENTID_UNKNOWN_DATA_OUT_CONTEXT_TABLE_FULL: FwEventIdType = 3;
pub const EVENTID_BUFFER_CONTEXT_NOT_FOUND: FwEventIdType = 4;

/// One buffer-to-context association.
#[derive(Debug, Clone, Copy)]
struct TableEntry {
    /// Router-generated token stamped into the forwarded buffer's context
    /// word (the Rust stand-in for the C++ data-pointer key).
    token: u32,
    /// The buffer's context word before stamping, restored on return.
    original_context_word: u32,
    /// The frame context to restore on return.
    frame_context: FrameContext,
}

/// Guarded state: the context table (C++ `m_bufferContextTable`).
struct RouterState {
    table: [Option<TableEntry>; BUFFER_CONTEXT_TABLE_SIZE],
    next_token: u32,
}

/// `Svc::FprimeRouter` — passive APID router.
pub struct FprimeRouter {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `commandOut` — command packets as `Fw.Com` (to CmdDispatcher).
    /// Critical: must be connected (invoked unconditionally, C++ parity).
    pub command_out: OutputPort<dyn ComPort>,
    /// `fileOut` — file packets as `Fw.BufferSend` (to FileUplink).
    pub file_out: OutputPort<dyn BufferSendPort>,
    /// `unknownDataOut` — unrecognized packet types with context.
    pub unknown_data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — buffer ownership back upstream (to the deframer).
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// Guarded state (the C++ guarded-port component mutex).
    state: Mutex<RouterState>,
}

impl FprimeRouter {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            command_out: OutputPort::new(),
            file_out: OutputPort::new(),
            unknown_data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            state: Mutex::new(RouterState {
                table: [None; BUFFER_CONTEXT_TABLE_SIZE],
                next_token: 1,
            }),
        })
    }

    // -- Input-port factories -----------------------------------------------

    /// `dataIn` — GUARDED `Svc.ComDataWithContext` input: deframed packets.
    pub fn data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(Arc::new(DataInAdapter { comp: self.clone() }), port_num)
    }

    /// `fileBufferReturnIn` — GUARDED `Fw.BufferSend` input: buffers coming
    /// back from `fileOut`/`unknownDataOut` receivers.
    pub fn file_buffer_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn BufferSendPort> {
        PortRef::new(
            Arc::new(FileBufferReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `cmdResponseIn` — SYNC `Fw.CmdResponse` input: no-op (C++ parity).
    pub fn cmd_response_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn CmdResponsePort> {
        PortRef::new(Arc::new(CmdResponseInAdapter), port_num)
    }

    // -- Handlers ------------------------------------------------------------

    fn log(&self, id: FwEventIdType, text: &str) {
        self.evt.log_event(
            self.base.get_id_base(),
            id,
            LogSeverity::WarningHi,
            text,
            |_buf| fprime_fw::SerializeStatus::Ok,
        );
    }

    fn data_return_out(&self, data: Buffer, context: &FrameContext) {
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// C++ `insertContext`: stamp a token and remember (token -> context).
    /// Returns false when the table is full (the buffer is left unstamped —
    /// its return will miss the table, exactly like the C++ pointer miss).
    fn insert_context(
        &self,
        state: &mut RouterState,
        buffer: &mut Buffer,
        context: &FrameContext,
    ) -> bool {
        for slot in state.table.iter_mut() {
            if slot.is_none() {
                let token = state.next_token;
                state.next_token = state.next_token.wrapping_add(1).max(1);
                *slot = Some(TableEntry {
                    token,
                    original_context_word: buffer.context(),
                    frame_context: *context,
                });
                buffer.set_context(token);
                return true;
            }
        }
        false
    }

    /// C++ `takeContext`: look the token up and remove the entry.
    fn take_context(&self, state: &mut RouterState, buffer: &mut Buffer) -> Option<FrameContext> {
        let token = buffer.context();
        for slot in state.table.iter_mut() {
            if let Some(entry) = slot {
                if entry.token == token {
                    let entry = *entry;
                    *slot = None;
                    buffer.set_context(entry.original_context_word);
                    return Some(entry.frame_context);
                }
            }
        }
        None
    }

    /// `dataIn` handler (guarded).
    fn data_in_handler(
        &self,
        _port_num: FwIndexType,
        mut packet_buffer: Buffer,
        context: &FrameContext,
    ) {
        let mut state = self.state.lock().unwrap();
        match context.apid {
            // Command packet: copy into a stack ComBuffer and forward.
            Apid::FwPacketCommand => {
                let mut com = ComBuffer::new();
                let status = com.set_buff(packet_buffer.data());
                if status.is_ok() {
                    // Critical path: commandOut must be connected (no
                    // is_connected check, C++ parity — get() asserts).
                    let p = self.command_out.get();
                    p.target.invoke(p.port_num, &mut com, 0);
                } else {
                    self.evt.log_event(
                        self.base.get_id_base(),
                        EVENTID_SERIALIZATION_ERROR,
                        LogSeverity::WarningHi,
                        &format!(
                            "Serializing com buffer failed with status {}",
                            status as u32
                        ),
                        |buf| buf.serialize_u32_be(status as u32),
                    );
                }
                // The bytes were copied: return the packet buffer now, with
                // the context it came with.
                self.data_return_out(packet_buffer, context);
            }
            // File packet: hand off on fileOut when connected.
            Apid::FwPacketFile => {
                if self.file_out.is_connected() {
                    if !self.insert_context(&mut state, &mut packet_buffer, context) {
                        self.log(
                            EVENTID_FILE_OUT_CONTEXT_TABLE_FULL,
                            "Buffer-to-context table full on fileOut; context will be lost for this buffer",
                        );
                    }
                    let p = self.file_out.get();
                    p.target.invoke(p.port_num, packet_buffer);
                } else {
                    self.data_return_out(packet_buffer, context);
                }
            }
            // Unknown packet type: forward with context when connected.
            _ => {
                if self.unknown_data_out.is_connected() {
                    if !self.insert_context(&mut state, &mut packet_buffer, context) {
                        self.log(
                            EVENTID_UNKNOWN_DATA_OUT_CONTEXT_TABLE_FULL,
                            "Buffer-to-context table full on unknownDataOut; context will be lost for this buffer",
                        );
                    }
                    let p = self.unknown_data_out.get();
                    p.target.invoke(p.port_num, packet_buffer, context);
                } else {
                    self.data_return_out(packet_buffer, context);
                }
            }
        }
    }

    /// `fileBufferReturnIn` handler (guarded): restore the saved context and
    /// return the buffer upstream.
    fn file_buffer_return_in_handler(&self, _port_num: FwIndexType, mut buffer: Buffer) {
        let mut state = self.state.lock().unwrap();
        let context = match self.take_context(&mut state, &mut buffer) {
            Some(context) => context,
            None => {
                self.log(
                    EVENTID_BUFFER_CONTEXT_NOT_FOUND,
                    "Returned buffer not found in context table; returning with empty context",
                );
                FrameContext::default()
            }
        };
        self.data_return_out(buffer, &context);
    }
}

/// Adapter for the guarded `dataIn` port.
struct DataInAdapter {
    comp: Arc<FprimeRouter>,
}

impl ComDataWithContextPort for DataInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_in_handler(port_num, data, context);
    }
}

/// Adapter for the guarded `fileBufferReturnIn` port.
struct FileBufferReturnInAdapter {
    comp: Arc<FprimeRouter>,
}

impl BufferSendPort for FileBufferReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        self.comp.file_buffer_return_in_handler(port_num, buffer);
    }
}

/// Adapter for the sync `cmdResponseIn` port (no-op, C++ parity).
struct CmdResponseInAdapter;

impl CmdResponsePort for CmdResponseInAdapter {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        _op_code: FwOpcodeType,
        _cmd_seq: u32,
        _response: CmdResponse,
    ) {
        // Nothing to do (C++ parity).
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::LogPort;
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    #[derive(Default)]
    struct Recorder {
        /// commandOut records: (bytes, context word).
        commands: StdMutex<Vec<(Vec<u8>, u32)>>,
        /// fileOut records: (bytes, buffer context word).
        files: StdMutex<Vec<(Vec<u8>, u32)>>,
        /// unknownDataOut records.
        unknown: StdMutex<Vec<(Vec<u8>, FrameContext, u32)>>,
        /// dataReturnOut records: (bytes, frame context, buffer context word).
        returned: StdMutex<Vec<(Vec<u8>, FrameContext, u32)>>,
        events: StdMutex<Vec<FwEventIdType>>,
    }

    impl ComPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
            self.commands
                .lock()
                .unwrap()
                .push((data.as_slice().to_vec(), context));
        }
    }

    impl BufferSendPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.files
                .lock()
                .unwrap()
                .push((buffer.data().to_vec(), buffer.context()));
        }
    }

    impl ComDataWithContextPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            if port_num == 0 {
                self.returned.lock().unwrap().push((
                    data.data().to_vec(),
                    *context,
                    data.context(),
                ));
            } else {
                self.unknown
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context, data.context()));
            }
        }
    }

    impl LogPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _args: &mut LogBuffer,
        ) {
            self.events.lock().unwrap().push(id);
        }
    }

    fn build(connect_file: bool, connect_unknown: bool) -> (Arc<FprimeRouter>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let router = FprimeRouter::new("router");
        router.command_out.connect(rec.clone(), 0);
        if connect_file {
            router.file_out.connect(rec.clone(), 0);
        }
        if connect_unknown {
            router.unknown_data_out.connect(rec.clone(), 1);
        }
        router.data_return_out.connect(rec.clone(), 0);
        router.evt.log_out.connect(rec.clone(), 0);
        (router, rec)
    }

    fn packet(bytes: &[u8], context_word: u32) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b.set_context(context_word);
        b
    }

    fn ctx(apid: Apid) -> FrameContext {
        FrameContext {
            apid,
            ..FrameContext::default()
        }
    }

    fn feed(router: &Arc<FprimeRouter>, buffer: Buffer, context: &FrameContext) {
        let din = router.data_in(0);
        din.target.invoke(din.port_num, buffer, context);
    }

    /// Command APID: bytes copied to commandOut (context 0), buffer returned
    /// immediately with its incoming context.
    #[test]
    fn command_packet_copied_and_returned() {
        let (router, rec) = build(true, true);
        let context = ctx(Apid::FwPacketCommand);
        let bytes = [0x00u8, 0x00, 0x00, 0x00, 0x01, 0x10];
        feed(&router, packet(&bytes, 7), &context);
        assert_eq!(*rec.commands.lock().unwrap(), vec![(bytes.to_vec(), 0)]);
        let ret = rec.returned.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].0, bytes);
        assert_eq!(ret[0].1.apid, Apid::FwPacketCommand);
        assert_eq!(ret[0].2, 7); // buffer context word untouched
        assert!(rec.events.lock().unwrap().is_empty());
    }

    /// A command payload too large for a ComBuffer: SerializationError,
    /// still returned, nothing sent.
    #[test]
    fn oversized_command_packet_serialization_error() {
        let (router, rec) = build(false, false);
        let big = vec![0u8; 600]; // > FW_COM_BUFFER_MAX_SIZE
        feed(&router, packet(&big, 0), &ctx(Apid::FwPacketCommand));
        assert!(rec.commands.lock().unwrap().is_empty());
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![EVENTID_SERIALIZATION_ERROR]
        );
        assert_eq!(rec.returned.lock().unwrap().len(), 1);
    }

    /// File APID with fileOut connected: forwarded (no immediate return);
    /// the return restores the frame context AND the buffer context word.
    #[test]
    fn file_packet_round_trip_restores_context() {
        let (router, rec) = build(true, true);
        let mut context = ctx(Apid::FwPacketFile);
        context.sequence_count = 42;
        feed(&router, packet(b"filedata", 0xAABB_CCDD), &context);
        assert!(rec.returned.lock().unwrap().is_empty());
        let forwarded = {
            let files = rec.files.lock().unwrap();
            assert_eq!(files.len(), 1);
            assert_eq!(files[0].0, b"filedata");
            files[0].clone()
        };
        // Return the buffer (as the file uplink would).
        let frin = router.file_buffer_return_in(0);
        frin.target
            .invoke(frin.port_num, packet(&forwarded.0, forwarded.1));
        let ret = rec.returned.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].1.apid, Apid::FwPacketFile);
        assert_eq!(ret[0].1.sequence_count, 42); // restored context
        assert_eq!(ret[0].2, 0xAABB_CCDD); // original context word restored
        assert!(rec.events.lock().unwrap().is_empty());
    }

    /// File APID with fileOut NOT connected: returned immediately.
    #[test]
    fn file_packet_without_file_out_returned() {
        let (router, rec) = build(false, false);
        feed(&router, packet(b"f", 0), &ctx(Apid::FwPacketFile));
        assert!(rec.files.lock().unwrap().is_empty());
        assert_eq!(rec.returned.lock().unwrap().len(), 1);
    }

    /// Unknown APID with unknownDataOut connected: forwarded with context
    /// and tracked in the table.
    #[test]
    fn unknown_packet_forwarded_with_context() {
        let (router, rec) = build(true, true);
        let context = ctx(Apid::InvalidUninitialized);
        feed(&router, packet(b"??", 5), &context);
        let unknown = rec.unknown.lock().unwrap();
        assert_eq!(unknown.len(), 1);
        assert_eq!(unknown[0].0, b"??");
        assert_eq!(unknown[0].1.apid, Apid::InvalidUninitialized);
        let token = unknown[0].2;
        drop(unknown);
        // Round trip restores.
        let frin = router.file_buffer_return_in(0);
        frin.target.invoke(frin.port_num, packet(b"??", token));
        let ret = rec.returned.lock().unwrap();
        assert_eq!(ret[0].1.apid, Apid::InvalidUninitialized);
        assert_eq!(ret[0].2, 5);
    }

    /// Unknown APID with unknownDataOut NOT connected: returned immediately.
    #[test]
    fn unknown_packet_without_port_returned() {
        let (router, rec) = build(true, false);
        feed(&router, packet(b"?", 0), &ctx(Apid::FwPacketTelem));
        assert!(rec.unknown.lock().unwrap().is_empty());
        assert_eq!(rec.returned.lock().unwrap().len(), 1);
    }

    /// Table full: event emitted, buffer STILL forwarded; its return misses
    /// the table (BufferContextNotFound + default context).
    #[test]
    fn full_table_still_forwards_and_return_misses() {
        let (router, rec) = build(true, true);
        let context = ctx(Apid::FwPacketFile);
        for i in 0..BUFFER_CONTEXT_TABLE_SIZE {
            feed(&router, packet(b"x", i as u32), &context);
        }
        assert!(rec.events.lock().unwrap().is_empty());
        // 51st entry overflows the table.
        feed(&router, packet(b"overflow", 0x51), &context);
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![EVENTID_FILE_OUT_CONTEXT_TABLE_FULL]
        );
        let files = rec.files.lock().unwrap();
        assert_eq!(files.len(), BUFFER_CONTEXT_TABLE_SIZE + 1);
        // The overflow buffer kept its original context word (unstamped).
        let (bytes, word) = files[BUFFER_CONTEXT_TABLE_SIZE].clone();
        drop(files);
        assert_eq!(word, 0x51);
        // Returning it misses the table.
        let frin = router.file_buffer_return_in(0);
        frin.target.invoke(frin.port_num, packet(&bytes, word));
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![
                EVENTID_FILE_OUT_CONTEXT_TABLE_FULL,
                EVENTID_BUFFER_CONTEXT_NOT_FOUND
            ]
        );
        let ret = rec.returned.lock().unwrap();
        assert_eq!(ret[0].1, FrameContext::default());
    }

    /// UnknownDataOut table-full uses its own event id.
    #[test]
    fn unknown_table_full_event() {
        let (router, rec) = build(true, true);
        let context = ctx(Apid::FwPacketDp);
        for i in 0..=BUFFER_CONTEXT_TABLE_SIZE {
            feed(&router, packet(b"u", i as u32), &context);
        }
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![EVENTID_UNKNOWN_DATA_OUT_CONTEXT_TABLE_FULL]
        );
    }

    /// A return that was never forwarded: BufferContextNotFound + default
    /// context.
    #[test]
    fn unexpected_return_gets_default_context() {
        let (router, rec) = build(true, true);
        let frin = router.file_buffer_return_in(0);
        frin.target.invoke(frin.port_num, packet(b"stray", 999));
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![EVENTID_BUFFER_CONTEXT_NOT_FOUND]
        );
        let ret = rec.returned.lock().unwrap();
        assert_eq!(ret[0].1, FrameContext::default());
        assert_eq!(ret[0].2, 999); // context word untouched on a miss
    }

    /// cmdResponseIn is a no-op.
    #[test]
    fn cmd_response_in_is_noop() {
        let (router, rec) = build(true, true);
        let crin = router.cmd_response_in(0);
        crin.target.invoke(crin.port_num, 0x100, 1, CmdResponse::Ok);
        assert!(rec.returned.lock().unwrap().is_empty());
        assert!(rec.events.lock().unwrap().is_empty());
    }
}
