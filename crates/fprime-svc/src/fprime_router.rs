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
//! ## Context-table keying (C++ parity)
//!
//! The C++ table keys on the buffer's data POINTER (`getData()`). The Rust
//! [`Buffer`] owns its storage (`Box<[u8]>`), whose heap address is stable
//! while the buffer moves through ports (and through the async-port escrow)
//! and unique among outstanding allocations, so the table keys on
//! `data().as_ptr()` exactly like C++. The buffer's `context` word is never
//! touched, a buffer forwarded while the table was full can never match a
//! different entry on return (its live data address cannot equal another
//! outstanding allocation's), and a miss produces `BufferContextNotFound`
//! + default context — all identical to C++.

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
    /// The forwarded buffer's data address (the C++ `getData()` key). The
    /// buffer owns its storage, so this address is stable until the buffer
    /// returns and cannot equal the address of any other outstanding
    /// allocation — an un-tracked buffer can never match this entry.
    key: usize,
    /// The frame context to restore on return.
    frame_context: FrameContext,
}

/// Guarded state: the context table (C++ `m_bufferContextTable`).
struct RouterState {
    table: [Option<TableEntry>; BUFFER_CONTEXT_TABLE_SIZE],
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

    /// The C++ pointer key (`buffer.getData()`): the address of the
    /// buffer's data window. `Buffer` owns its storage, so the address is
    /// stable across port moves (and the async escrow) and unique among
    /// outstanding buffers — a live allocation's address can never equal
    /// another live allocation's, exactly as in C++.
    fn buffer_key(buffer: &Buffer) -> usize {
        buffer.data().as_ptr() as usize
    }

    /// C++ `insertContext`: remember (data address -> context). Returns
    /// false when the table is full (the buffer is forwarded un-tracked —
    /// its return will miss the table, exactly like the C++ pointer miss).
    fn insert_context(
        &self,
        state: &mut RouterState,
        buffer: &Buffer,
        context: &FrameContext,
    ) -> bool {
        for slot in state.table.iter_mut() {
            if slot.is_none() {
                *slot = Some(TableEntry {
                    key: Self::buffer_key(buffer),
                    frame_context: *context,
                });
                return true;
            }
        }
        false
    }

    /// C++ `takeContext`: look the data address up and remove the entry.
    fn take_context(&self, state: &mut RouterState, buffer: &Buffer) -> Option<FrameContext> {
        let key = Self::buffer_key(buffer);
        for slot in state.table.iter_mut() {
            if let Some(entry) = slot {
                if entry.key == key {
                    let entry = *entry;
                    *slot = None;
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
        packet_buffer: Buffer,
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
                    if !self.insert_context(&mut state, &packet_buffer, context) {
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
                    if !self.insert_context(&mut state, &packet_buffer, context) {
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
    fn file_buffer_return_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let mut state = self.state.lock().unwrap();
        let context = match self.take_context(&mut state, &buffer) {
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
        /// fileOut records: the actual forwarded buffers (kept so tests can
        /// return the SAME object, as the real receiver does).
        files: StdMutex<Vec<Buffer>>,
        /// unknownDataOut records: the actual forwarded buffers + context.
        unknown: StdMutex<Vec<(Buffer, FrameContext)>>,
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
            self.files.lock().unwrap().push(buffer);
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
                self.unknown.lock().unwrap().push((data, *context));
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

    /// File APID with fileOut connected: forwarded (no immediate return)
    /// with its context word UNTOUCHED (C++ parity — no stamping); the
    /// return restores the saved frame context.
    #[test]
    fn file_packet_round_trip_restores_context() {
        let (router, rec) = build(true, true);
        let mut context = ctx(Apid::FwPacketFile);
        context.sequence_count = 42;
        feed(&router, packet(b"filedata", 0xAABB_CCDD), &context);
        assert!(rec.returned.lock().unwrap().is_empty());
        let forwarded = {
            let mut files = rec.files.lock().unwrap();
            assert_eq!(files.len(), 1);
            assert_eq!(files[0].data(), b"filedata");
            assert_eq!(files[0].context(), 0xAABB_CCDD); // word untouched
            files.remove(0)
        };
        // Return the SAME buffer (as the file uplink would).
        let frin = router.file_buffer_return_in(0);
        frin.target.invoke(frin.port_num, forwarded);
        let ret = rec.returned.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].1.apid, Apid::FwPacketFile);
        assert_eq!(ret[0].1.sequence_count, 42); // restored context
        assert_eq!(ret[0].2, 0xAABB_CCDD); // context word never changed
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
    /// (word untouched) and tracked in the table.
    #[test]
    fn unknown_packet_forwarded_with_context() {
        let (router, rec) = build(true, true);
        let context = ctx(Apid::InvalidUninitialized);
        feed(&router, packet(b"??", 5), &context);
        let forwarded = {
            let mut unknown = rec.unknown.lock().unwrap();
            assert_eq!(unknown.len(), 1);
            assert_eq!(unknown[0].0.data(), b"??");
            assert_eq!(unknown[0].0.context(), 5); // word untouched
            assert_eq!(unknown[0].1.apid, Apid::InvalidUninitialized);
            unknown.remove(0).0
        };
        // Round trip restores.
        let frin = router.file_buffer_return_in(0);
        frin.target.invoke(frin.port_num, forwarded);
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
        let overflow = {
            let mut files = rec.files.lock().unwrap();
            assert_eq!(files.len(), BUFFER_CONTEXT_TABLE_SIZE + 1);
            files.pop().unwrap()
        };
        // The overflow buffer kept its original context word (un-tracked).
        assert_eq!(overflow.context(), 0x51);
        // Returning it misses the table.
        let frin = router.file_buffer_return_in(0);
        frin.target.invoke(frin.port_num, overflow);
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![
                EVENTID_FILE_OUT_CONTEXT_TABLE_FULL,
                EVENTID_BUFFER_CONTEXT_NOT_FOUND
            ]
        );
        let ret = rec.returned.lock().unwrap();
        assert_eq!(ret[0].1, FrameContext::default());
        assert_eq!(ret[0].2, 0x51); // context word untouched on a miss
    }

    /// Regression (C++ parity): a buffer forwarded while the table was full
    /// keeps whatever context word it arrived with, and that word can
    /// numerically equal data belonging to a tracked entry. Because the
    /// table keys on the buffer's live data address — which can never equal
    /// a different outstanding allocation's — the un-tracked return must
    /// MISS (default context, word untouched) and every tracked buffer must
    /// still get its OWN context back afterwards.
    #[test]
    fn untracked_return_cannot_steal_a_tracked_entry() {
        let (router, rec) = build(true, true);
        // Fill the table; each entry's frame context is distinguished by
        // sequence_count == its BufferManager-style context word (mgr 0).
        for i in 0..BUFFER_CONTEXT_TABLE_SIZE {
            let mut context = ctx(Apid::FwPacketFile);
            context.sequence_count = i as u16;
            feed(&router, packet(b"tracked", i as u32), &context);
        }
        // Overflow buffer with context word 7 — a small integer that a
        // token-keyed table would have confused with an active entry.
        feed(&router, packet(b"overflow", 7), &ctx(Apid::FwPacketFile));
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![EVENTID_FILE_OUT_CONTEXT_TABLE_FULL]
        );
        let overflow = rec.files.lock().unwrap().pop().unwrap();
        assert_eq!(overflow.context(), 7);
        let frin = router.file_buffer_return_in(0);
        frin.target.invoke(frin.port_num, overflow);
        {
            let ret = rec.returned.lock().unwrap();
            assert_eq!(ret.len(), 1);
            assert_eq!(ret[0].1, FrameContext::default()); // guaranteed miss
            assert_eq!(ret[0].2, 7); // context word untouched
        }
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![
                EVENTID_FILE_OUT_CONTEXT_TABLE_FULL,
                EVENTID_BUFFER_CONTEXT_NOT_FOUND
            ]
        );
        // Every tracked buffer still restores its OWN context — no entry
        // was stolen or context word rewritten by the un-tracked return.
        let tracked: Vec<Buffer> = rec.files.lock().unwrap().drain(..).collect();
        for buffer in tracked {
            let word = buffer.context();
            frin.target.invoke(frin.port_num, buffer);
            let ret = rec.returned.lock().unwrap();
            let last = ret.last().unwrap();
            assert_eq!(last.1.apid, Apid::FwPacketFile);
            assert_eq!(u32::from(last.1.sequence_count), word); // its own frame context
            assert_eq!(last.2, word); // its own context word
        }
        // No further BufferContextNotFound events.
        assert_eq!(rec.events.lock().unwrap().len(), 2);
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
