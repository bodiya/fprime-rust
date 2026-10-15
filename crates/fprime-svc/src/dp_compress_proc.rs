//! `Svc::DpCompressProc` — in-place data-product compression driver (PASSIVE).
//!
//! Port of `Svc/DpCompressProc/DpCompressProc.{fpp,hpp,cpp}` with its
//! `Types/CompressionTypes.fpp` (`Svc.CompressionAlgorithm`,
//! `Svc.CompressionMetadata`) and `Ports/CompressChunk.fpp`
//! (`Svc.CompressChunk`). Upstream documentation:
//! `Svc/DpCompressProc/docs/sdd.md`; the data-product context is in
//! `docs/cpp-analysis/data-products.md`.
//!
//! `DpCompressProc` sits on one of `Svc::DpWriter`'s `procBufferSendOut`
//! ports (the bit of the container's `procTypes` mask that selects it —
//! `Fw.DpCfg.ProcType.PROC_TYPE_ZLIB_DEFLATE`, i.e. [`ProcType::ZlibDeflate`] on
//! port 0 in the upstream `DpCompression` subtopology). It does no
//! compression itself: it cuts the container's data region into
//! `CHUNK_SIZE` chunks, offers each one to a compressor through
//! `compressChunk` (`Svc::DpZLibCompressor` in upstream), and rewrites the
//! data region IN PLACE, without any extra buffer, as a sequence of
//! `CompressionRecord` records:
//!
//! ```text
//! [FwDpIdType  idBase + CompressionRecord]   4 B, big-endian
//! [FwSizeStoreType  1 + payload size]        2 B, the U8-array record count
//! [Svc.CompressionMetadata  algorithm]       1 B (UNCOMPRESSED / ZLIB_DEFLATE)
//! [payload]                                  a zlib stream, or raw bytes
//! ```
//!
//! Every compressed chunk gets its own record; runs of consecutive
//! incompressible chunks share one `UNCOMPRESSED` record. If no chunk
//! compresses, the container is left untouched. Otherwise the header's
//! data size is rewritten (and the header re-serialized, which updates the
//! header hash); `DpWriter` then re-reads the header, shrinks the buffer
//! and computes the data hash over the compressed region before writing
//! the `.fdp` file. The ground decoder (`fprime-dp`, fprime-gds
//! `common/dp/decoder.py`) recognizes the records by the dictionary name
//! `<instance>.CompressionRecord` with an instance named `dpCompressProc`,
//! concatenates the payloads (`zlib.decompress` for `ZLIB_DEFLATE`) and
//! decodes the result as the original records.
//!
//! [`ProcType::ZlibDeflate`]: fprime_fw::dp::ProcType::ZlibDeflate
//!
//! # Deviations from C++
//!
//! * `procRequest` is declared `Fw.BufferSend`, but it is driven by
//!   `DpWriter`'s processing fan-out, which borrows the buffer
//!   ([`DpProcPort`]); see the `dp_writer` module notes.
//! * `Svc.CompressChunk` passes `ref buffer: Fw.Buffer`, a non-owning view of
//!   one chunk inside the container. The Rust [`Buffer`] owns its storage,
//!   so the port passes a [`ChunkBuffer`] instead: a mutable borrow of the
//!   chunk bytes plus the size the compressor may shrink — the same data
//!   and the same in-place contract.
//! * The C++ serializer/deserializer pair over the data region becomes two
//!   byte cursors; `memmove`/`serializeFrom` become `copy_within`. The C++
//!   copy of a pending uncompressed run uses `serializeFrom` (`memcpy`) on
//!   a source that can overlap the destination; the destination always
//!   lies at or before the source, so the result is the same as the
//!   `memmove` semantics of `copy_within`.
//! * Parameter set/save commands follow the workspace's convention for
//!   generated parameters (`fprime-fpp` codegen): `FORMAT_ERROR` for a
//!   short or over-long argument, `VALIDATION_ERROR` for an undeclared
//!   `Fw.Enabled` value, `EXECUTION_ERROR` for a save with `prmSetOut`
//!   unconnected.
//! * `dpGet`/`dpSend` and the `Dummy` container exist upstream only because
//!   FPP requires product ports on a component that declares a record; they
//!   are declared here for wiring parity and never invoked.

use crate::dp_writer::DpProcPort;
use fprime_comp::{
    CmdGlue, CmdPort, EventGlue, EventThrottle, OutputPort, PassiveBase, PortRef, PrmGlue,
};
use fprime_config::{
    FwDpIdType, FwEventIdType, FwIdType, FwIndexType, FwOpcodeType, FwPrmIdType, FwSizeStoreType,
    FwSizeType,
};
use fprime_fw::dp::{DpContainer, DpGetPort, DpSendPort};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Enabled, Endianness, ExtBuf, LogSeverity, ParamBuffer,
    ParamValid, SerBuf, Serialize, fpp_enum, fpp_struct, fw_assert, fw_try,
};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Svc/DpCompressProc/Types and Ports
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `Svc.CompressionAlgorithm` — how a `CompressionRecord` payload is
    /// encoded; also the return value of [`CompressChunkPort`].
    pub enum CompressionAlgorithm : u8 {
        /// The payload is the original bytes.
        Uncompressed = 0,
        /// The payload is a zlib (RFC 1950) stream of DEFLATE data.
        ZlibDeflate = 1,
    }
    default Uncompressed
}

fpp_struct! {
    /// `Svc.CompressionMetadata` (a `dictionary struct`): the first bytes of
    /// every `CompressionRecord` payload.
    #[derive(Clone, Copy, Eq)]
    pub struct CompressionMetadata {
        /// The algorithm the rest of the record is encoded with.
        algorithm: CompressionAlgorithm { get_algorithm, set_algorithm },
    }
}

/// The `ref buffer: Fw.Buffer` argument of `Svc.CompressChunk`: a borrowed
/// window onto one chunk of the container's data region (see the module
/// notes). The compressor may overwrite the bytes and shrink the size; it
/// can never grow past the chunk.
#[derive(Debug)]
pub struct ChunkBuffer<'a> {
    data: &'a mut [u8],
    size: usize,
}

impl<'a> ChunkBuffer<'a> {
    /// A window over all of `data`.
    pub fn new(data: &'a mut [u8]) -> Self {
        let size = data.len();
        Self { data, size }
    }

    /// The current size (C++ `getSize`).
    pub fn size(&self) -> usize {
        self.size
    }

    /// The chunk length the window was created with.
    pub fn capacity(&self) -> usize {
        self.data.len()
    }

    /// Set the size (C++ `setSize`); `fw_assert`s that it stays within the
    /// chunk, since the bytes past it belong to the next chunk.
    pub fn set_size(&mut self, size: usize) {
        fw_assert!(size <= self.data.len(), size as i32, self.data.len() as i32);
        self.size = size;
    }

    /// The bytes in the window (C++ `getData`, `getSize` bytes).
    pub fn data(&self) -> &[u8] {
        &self.data[..self.size]
    }

    /// The bytes in the window, mutable.
    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.data[..self.size]
    }
}

/// `Svc.CompressChunk`:
/// `port CompressChunk(ref buffer: Fw.Buffer, min_compression: FwSizeType,
/// write_offset: FwSizeType) -> CompressionAlgorithm`.
///
/// Contract (upstream `docs/sdd.md`): on success the compressor writes at
/// most `min_compression` bytes of compressed data starting `write_offset`
/// bytes into `buffer`, sets the size to `write_offset + compressed size`
/// and returns the algorithm; otherwise it leaves `buffer` unmodified and
/// returns [`CompressionAlgorithm::Uncompressed`].
pub trait CompressChunkPort: Send + Sync {
    /// Invoke the port.
    fn invoke(
        &self,
        port_num: FwIndexType,
        buffer: &mut ChunkBuffer<'_>,
        min_compression: FwSizeType,
        write_offset: FwSizeType,
    ) -> CompressionAlgorithm;
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

/// The serialized size of one `CompressionRecord` header: record id, array
/// size and metadata (C++ `compression_header_size`).
pub const COMPRESSION_HEADER_SIZE: usize =
    size_of::<FwDpIdType>() + size_of::<FwSizeStoreType>() + CompressionMetadata::SERIALIZED_SIZE;

/// The C++ processing state machine (`DpCompressProcSM.drawio.png`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProcState {
    /// Before the first chunk.
    Init,
    /// No chunk compressed yet; the original bytes are still in place.
    PreCommit,
    /// The last chunk was compressed.
    LastCompressed,
    /// The last chunk did not compress; a run of raw bytes is pending.
    LastUncompressed,
}

/// The parameter store (the autocoded `m_<param>` / `m_param_<param>_valid`
/// members).
#[derive(Debug)]
struct DpCompressProcParams {
    chunk_size: FwSizeStoreType,
    chunk_size_valid: ParamValid,
    enable: Enabled,
    enable_valid: ParamValid,
}

impl Default for DpCompressProcParams {
    fn default() -> Self {
        Self {
            chunk_size: DpCompressProc::CHUNK_SIZE_DEFAULT,
            chunk_size_valid: ParamValid::Uninit,
            enable: DpCompressProc::ENABLE_DEFAULT,
            enable_valid: ParamValid::Uninit,
        }
    }
}

/// `FW_PARAM_OK(valid)`: the value is the loaded one or the FPP default.
fn param_ok(valid: ParamValid) -> bool {
    matches!(valid, ParamValid::Valid | ParamValid::Default)
}

/// `Svc::DpCompressProc` — passive data-product compression processor.
pub struct DpCompressProc {
    /// Passive core (name, id base).
    pub base: PassiveBase,
    /// Command registration/response ports (parameter commands only).
    pub cmd: CmdGlue,
    /// Event ports and the `timeCaller` port.
    pub evt: EventGlue,
    /// Parameter ports (`prmGetOut`, `prmSetOut`).
    pub prm: PrmGlue,
    /// `compressChunk`: the compressor (optional — an unconnected port
    /// bypasses compression, as in C++).
    pub compress_chunk: OutputPort<dyn CompressChunkPort>,
    /// `dpGet` (`Fw.DpGet`) — declared for FPP, never invoked.
    pub dp_get: OutputPort<dyn DpGetPort>,
    /// `dpSend` (`Fw.DpSend`) — declared for FPP, never invoked.
    pub dp_send: OutputPort<dyn DpSendPort>,
    did_not_compress_throttle: EventThrottle,
    invalid_header_throttle: EventThrottle,
    buffer_too_small_for_packet_throttle: EventThrottle,
    container_too_large_throttle: EventThrottle,
    params: Mutex<DpCompressProcParams>,
}

impl DpCompressProc {
    // -- Commands (FPP-relative opcodes: the implicit parameter commands) --

    /// `CHUNK_SIZE_PRM_SET(val: FwSizeStoreType)`.
    pub const OPCODE_CHUNK_SIZE_PRM_SET: FwOpcodeType = 0x00;
    /// `CHUNK_SIZE_PRM_SAVE`.
    pub const OPCODE_CHUNK_SIZE_PRM_SAVE: FwOpcodeType = 0x01;
    /// `ENABLE_PRM_SET(val: Fw.Enabled)`.
    pub const OPCODE_ENABLE_PRM_SET: FwOpcodeType = 0x02;
    /// `ENABLE_PRM_SAVE`.
    pub const OPCODE_ENABLE_PRM_SAVE: FwOpcodeType = 0x03;

    // -- Parameters ---------------------------------------------------------

    /// `CHUNK_SIZE: FwSizeStoreType default 32*1024` — the chunk size used
    /// to cut the container for the compressor.
    pub const PARAMID_CHUNK_SIZE: FwPrmIdType = 0x00;
    /// `ENABLE: Fw.Enabled default ENABLED` — DISABLED bypasses
    /// compression.
    pub const PARAMID_ENABLE: FwPrmIdType = 0x01;
    /// FPP default of `CHUNK_SIZE`.
    pub const CHUNK_SIZE_DEFAULT: FwSizeStoreType = 32 * 1024;
    /// FPP default of `ENABLE`.
    pub const ENABLE_DEFAULT: Enabled = Enabled::Enabled;

    // -- Events (FPP-relative ids) -----------------------------------------

    /// `CompressionComplete(dp_id, initial_size, final_size)` — DIAGNOSTIC,
    /// id 0.
    pub const EVENTID_COMPRESSION_COMPLETE: FwEventIdType = 0x00;
    /// `DidNotCompress(dp_id, data_size)` — ACTIVITY_LO, `throttle 10`.
    pub const EVENTID_DID_NOT_COMPRESS: FwEventIdType = 0x01;
    /// `InvalidHeader(buffer_size, error_code)` — WARNING_HI, `throttle 10`.
    pub const EVENTID_INVALID_HEADER: FwEventIdType = 0x02;
    /// `BufferTooSmallForPacket(buffer_size, min_size)` — WARNING_HI,
    /// `throttle 10`.
    pub const EVENTID_BUFFER_TOO_SMALL_FOR_PACKET: FwEventIdType = 0x03;
    /// `ContainerTooLarge(dp_id, data_size)` — WARNING_HI, `throttle 10`.
    pub const EVENTID_CONTAINER_TOO_LARGE: FwEventIdType = 0x04;
    /// The `throttle 10` limit shared by events 1..=4.
    pub const EVENT_THROTTLE: u32 = 10;

    // -- Data products ------------------------------------------------------

    /// `product record CompressionRecord: U8 array` — id 0. The on-disk
    /// record id is `idBase + RECORDID_COMPRESSION_RECORD`.
    pub const RECORDID_COMPRESSION_RECORD: FwDpIdType = 0x00;
    /// `product container Dummy default priority 0` — id 0 (never produced).
    pub const CONTAINERID_DUMMY: FwDpIdType = 0x00;

    /// Construct (topology phase 1). Follow with `base.set_id_base`,
    /// wiring, [`reg_commands`](Self::reg_commands) and
    /// [`load_parameters`](Self::load_parameters).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            prm: PrmGlue::new(),
            compress_chunk: OutputPort::new(),
            dp_get: OutputPort::new(),
            dp_send: OutputPort::new(),
            did_not_compress_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            invalid_header_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            buffer_too_small_for_packet_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            container_too_large_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            params: Mutex::new(DpCompressProcParams::default()),
        })
    }

    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_CHUNK_SIZE_PRM_SET,
                Self::OPCODE_CHUNK_SIZE_PRM_SAVE,
                Self::OPCODE_ENABLE_PRM_SET,
                Self::OPCODE_ENABLE_PRM_SAVE,
            ],
        );
    }

    /// The autocoded `loadParameters()`: fetch both parameters through
    /// `prmGetOut` (which must be connected). A `VALID` answer that
    /// deserializes is taken; anything else installs the FPP default with
    /// validity `DEFAULT`.
    ///
    /// C++ parity: until this runs both parameters are `UNINIT`, and
    /// `procRequest` `FW_ASSERT`s on `FW_PARAM_OK` — call it in the
    /// topology's `loadParameters` phase.
    pub fn load_parameters(&self) {
        let id_base = self.id_base();
        let mut buf = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(id_base, Self::PARAMID_CHUNK_SIZE, &mut buf);
        let mut chunk_size: FwSizeStoreType = 0;
        let chunk_ok =
            valid == ParamValid::Valid && buf.deserialize_u16_be(&mut chunk_size).is_ok();

        let mut buf = ParamBuffer::new();
        let valid = self.prm.get_param(id_base, Self::PARAMID_ENABLE, &mut buf);
        let mut enable = Self::ENABLE_DEFAULT;
        let enable_ok =
            valid == ParamValid::Valid && buf.deserialize(&mut enable, Endianness::Big).is_ok();

        let mut params = self.params.lock().unwrap();
        if chunk_ok {
            params.chunk_size = chunk_size;
            params.chunk_size_valid = ParamValid::Valid;
        } else {
            params.chunk_size = Self::CHUNK_SIZE_DEFAULT;
            params.chunk_size_valid = ParamValid::Default;
        }
        if enable_ok {
            params.enable = enable;
            params.enable_valid = ParamValid::Valid;
        } else {
            params.enable = Self::ENABLE_DEFAULT;
            params.enable_valid = ParamValid::Default;
        }
    }

    /// `paramGet_CHUNK_SIZE`.
    #[must_use]
    pub fn param_get_chunk_size(&self) -> (FwSizeStoreType, ParamValid) {
        let params = self.params.lock().unwrap();
        (params.chunk_size, params.chunk_size_valid)
    }

    /// `paramGet_ENABLE`.
    #[must_use]
    pub fn param_get_enable(&self) -> (Enabled, ParamValid) {
        let params = self.params.lock().unwrap();
        (params.enable, params.enable_valid)
    }

    // -- Input-port factories ----------------------------------------------

    /// `procRequest` — SYNC processing input, wired from
    /// `DpWriter.procBufferSendOut[n]`.
    pub fn proc_request(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn DpProcPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `cmdIn` — SYNC `Fw.Cmd` input (the parameter commands).
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- procRequest --------------------------------------------------------

    /// `procRequest_handler`: compress the container in place, or leave it
    /// untouched.
    fn proc_request_handler(&self, _port_num: FwIndexType, buffer: &mut Buffer) {
        let (enable, valid) = self.param_get_enable();
        fw_assert!(param_ok(valid), valid.as_repr() as i32);
        if enable == Enabled::Disabled {
            // Bypass compression.
            return;
        }
        if !self.compress_chunk.is_connected() {
            return;
        }
        let buffer_size = buffer.size() as FwSizeType;
        if buffer.size() < DpContainer::MIN_PACKET_SIZE {
            self.log_buffer_too_small_for_packet(
                buffer_size,
                DpContainer::MIN_PACKET_SIZE as FwSizeType,
            );
            return;
        }
        // The Rust container owns its buffer: lend it for the duration of
        // the call and always hand it back (C++ `DpContainer container(0,
        // fwBuffer)` is a view).
        let mut container = DpContainer::with_buffer(0, std::mem::take(buffer));
        self.process_container(&mut container, buffer_size);
        *buffer = container.take_buffer();
    }

    /// The body of `procRequest_handler` after the container is built.
    fn process_container(&self, container: &mut DpContainer, buffer_size: FwSizeType) {
        let des_stat = container.deserialize_header();
        if !des_stat.is_ok() {
            // Cannot process a container with an invalid header.
            self.log_invalid_header(buffer_size, des_stat as i32 as u32);
            return;
        }
        // Record sizes are FwSizeStoreType: give up on any buffer whose
        // contents could overflow a record size field.
        if buffer_size >= FwSizeStoreType::MAX as FwSizeType {
            self.log_container_too_large(container.id(), container.data_size());
            return;
        }
        // Consistency check: the header's data size must fit the buffer.
        let min_packet = DpContainer::MIN_PACKET_SIZE as FwSizeType;
        if container.data_size() > buffer_size - min_packet {
            self.log_buffer_too_small_for_packet(buffer_size, min_packet + container.data_size());
            return;
        }

        let (chunk_param, valid) = self.param_get_chunk_size();
        fw_assert!(param_ok(valid), valid.as_repr() as i32);
        let mut prm_chunk_size = FwSizeType::from(chunk_param);
        if prm_chunk_size > container.data_size() {
            prm_chunk_size = container.data_size();
        }
        let max_chunk_size = prm_chunk_size as usize;
        fw_assert!(
            max_chunk_size <= FwSizeStoreType::MAX as usize,
            max_chunk_size as i32
        );
        if max_chunk_size == 0 {
            // Chunk size is invalid (zero parameter or empty container).
            return;
        }

        let dp_id = container.id();
        let initial_size = container.data_size();
        let record_id = self.id_base() + Self::RECORDID_COMPRESSION_RECORD;
        let port = self.compress_chunk.get();

        // C++ parity: the data buffer is the whole data REGION
        // (`bufferSize - MIN_PACKET_SIZE` bytes from DATA_OFFSET), not just
        // the header's data size. The two are equal for a packet that
        // reaches DpWriter through the generated `dpSend`, which trims the
        // buffer to the packet size.
        let data = container.data_region_mut();
        fw_assert!(!data.is_empty(), data.len() as i32);
        let comp_data_size =
            compress_in_place(data, max_chunk_size, record_id, |chunk, min, off| {
                port.target.invoke(port.port_num, chunk, min, off)
            });

        match comp_data_size {
            None => {
                // Never compressed a chunk: the buffer is unchanged.
                self.log_did_not_compress(dp_id, initial_size);
            }
            Some(comp_data_size) => {
                let comp_data_size = comp_data_size as FwSizeType;
                self.log_compression_complete(dp_id, initial_size, comp_data_size);
                container.set_data_size(comp_data_size);
                container.serialize_header();
            }
        }
    }

    // -- Parameter commands --------------------------------------------------

    fn chunk_size_prm_set(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut val: FwSizeStoreType = 0;
        if !args.deserialize_u16_be(&mut val).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        {
            let mut params = self.params.lock().unwrap();
            params.chunk_size = val;
            params.chunk_size_valid = ParamValid::Valid;
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn enable_prm_set(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut raw = 0u8;
        if !args.deserialize_u8_be(&mut raw).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(val) = Enabled::try_from(raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        {
            let mut params = self.params.lock().unwrap();
            params.enable = val;
            params.enable_valid = ParamValid::Valid;
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn prm_save(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
        id: FwPrmIdType,
    ) {
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        if !self.prm.prm_set_out.is_connected() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        let mut buf = ParamBuffer::new();
        let status = {
            let params = self.params.lock().unwrap();
            if id == Self::PARAMID_CHUNK_SIZE {
                buf.serialize_u16_be(params.chunk_size)
            } else {
                params.enable.serialize_to(&mut buf, Endianness::Big)
            }
        };
        fw_assert!(status.is_ok(), status as i32);
        self.prm.set_param(self.id_base(), id, &mut buf);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    // -- Events ---------------------------------------------------------------

    fn log_compression_complete(&self, dp_id: FwDpIdType, initial: FwSizeType, final_: FwSizeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_COMPRESSION_COMPLETE,
            LogSeverity::Diagnostic,
            &format!("Compressed Data Product {dp_id} from {initial} to {final_} bytes"),
            |buf| {
                fw_try!(buf.serialize_u32_be(dp_id));
                fw_try!(buf.serialize_u64_be(initial));
                buf.serialize_u64_be(final_)
            },
        );
    }

    fn log_did_not_compress(&self, dp_id: FwDpIdType, data_size: FwSizeType) {
        if !self.did_not_compress_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_DID_NOT_COMPRESS,
            LogSeverity::ActivityLo,
            &format!(
                "Unable to reduce size of Data Product {dp_id}. Uncompressed size {data_size}"
            ),
            |buf| {
                fw_try!(buf.serialize_u32_be(dp_id));
                buf.serialize_u64_be(data_size)
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
                "Received buffer of size {buffer_size}; deserialization of container header \
                 failed with error code {error_code}"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(buffer_size));
                buf.serialize_u32_be(error_code)
            },
        );
    }

    fn log_buffer_too_small_for_packet(&self, buffer_size: FwSizeType, min_size: FwSizeType) {
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
                buf.serialize_u64_be(min_size)
            },
        );
    }

    fn log_container_too_large(&self, dp_id: FwDpIdType, data_size: FwSizeType) {
        if !self.container_too_large_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CONTAINER_TOO_LARGE,
            LogSeverity::WarningHi,
            &format!(
                "Data Product {dp_id} has data size {data_size}, too large for a record size field"
            ),
            |buf| {
                fw_try!(buf.serialize_u32_be(dp_id));
                buf.serialize_u64_be(data_size)
            },
        );
    }

    /// C++ `log_ACTIVITY_LO_DidNotCompress_ThrottleClear` (and the other
    /// throttle clears). Upstream has no command that calls them; exposed
    /// for tests and deployments.
    pub fn clear_event_throttles(&self) {
        self.did_not_compress_throttle.clear();
        self.invalid_header_throttle.clear();
        self.buffer_too_small_for_packet_throttle.clear();
        self.container_too_large_throttle.clear();
    }
}

/// `serializeCompressionHeader`: write one `CompressionRecord` header at
/// `*ser_loc` and advance it.
fn serialize_compression_header(
    data: &mut [u8],
    ser_loc: &mut usize,
    record_id: FwDpIdType,
    payload_size: usize,
    metadata: CompressionMetadata,
) {
    let record_size = payload_size + CompressionMetadata::SERIALIZED_SIZE;
    fw_assert!(
        record_size <= FwSizeStoreType::MAX as usize,
        record_size as i32
    );
    let mut ser = ExtBuf::new(&mut data[*ser_loc..]);
    let status = ser.serialize_u32_be(record_id);
    fw_assert!(status.is_ok(), status as i32);
    let status = ser.serialize_u16_be(record_size as FwSizeStoreType);
    fw_assert!(status.is_ok(), status as i32);
    let status = metadata.serialize_to(&mut ser, Endianness::Big);
    fw_assert!(status.is_ok(), status as i32);
    *ser_loc += ser.get_size();
}

/// Append `len` bytes starting at `src` to the output at `*ser_loc`
/// (C++ `serializeFrom(ptr, len, OMIT_LENGTH)` on the data serializer).
fn serialize_from(data: &mut [u8], ser_loc: &mut usize, src: usize, len: usize) {
    fw_assert!(*ser_loc + len <= data.len(), *ser_loc as i32, len as i32);
    data.copy_within(src..src + len, *ser_loc);
    *ser_loc += len;
}

/// The C++ chunking loop and state machine over the data region `data`.
///
/// Returns `None` when no chunk compressed (the region is untouched), else
/// the size of the rewritten region. `compress(chunk, min_compression,
/// write_offset)` is one `compressChunk` invocation.
fn compress_in_place<F>(
    data: &mut [u8],
    max_chunk_size: usize,
    record_id: FwDpIdType,
    mut compress: F,
) -> Option<usize>
where
    F: FnMut(&mut ChunkBuffer<'_>, FwSizeType, FwSizeType) -> CompressionAlgorithm,
{
    const H: usize = COMPRESSION_HEADER_SIZE;
    let total = data.len();
    // `data_deser` / `data_reser` positions.
    let mut deser_loc = 0usize;
    let mut ser_loc = 0usize;
    let mut state = ProcState::Init;
    let mut uncompressed_size = 0usize;
    let mut uncompressed_head: Option<usize> = None;

    while deser_loc < total {
        let chunk_size = (total - deser_loc).min(max_chunk_size);
        fw_assert!(chunk_size > 0, chunk_size as i32);
        fw_assert!(
            chunk_size <= FwSizeStoreType::MAX as usize,
            chunk_size as i32
        );
        let chunk_start = deser_loc;
        // Jump the deserializer to mark the chunk as read.
        deser_loc += chunk_size;

        let mut alg = CompressionAlgorithm::Uncompressed;
        let mut compression_offset = 0usize;
        let mut chunk_out_size = chunk_size;

        // Only offer chunks larger than 4 headers: below 3 the
        // min_compression arithmetic would go negative.
        if chunk_size > 4 * H {
            let mut min_compression = match state {
                // Room for this compressed chunk's header and a possible
                // uncompressed one after it.
                ProcState::Init => {
                    compression_offset = H;
                    chunk_size - 2 * H
                }
                // Room for the header of the initial uncompressed run, this
                // chunk's header and a possible uncompressed one after it.
                ProcState::PreCommit => {
                    compression_offset = 2 * H;
                    chunk_size - 3 * H
                }
                // Room for this chunk's header and a possible uncompressed
                // one (spare bytes from earlier compression are not tracked).
                ProcState::LastUncompressed | ProcState::LastCompressed => {
                    compression_offset = H;
                    chunk_size - 2 * H
                }
            };
            if min_compression > chunk_size - compression_offset {
                min_compression = chunk_size - compression_offset;
            }
            fw_assert!(
                min_compression <= chunk_size,
                min_compression as i32,
                chunk_size as i32
            );
            fw_assert!(
                compression_offset <= chunk_size,
                compression_offset as i32,
                chunk_size as i32
            );

            let mut chunk = ChunkBuffer::new(&mut data[chunk_start..chunk_start + chunk_size]);
            alg = compress(
                &mut chunk,
                min_compression as FwSizeType,
                compression_offset as FwSizeType,
            );
            chunk_out_size = chunk.size();
            if alg != CompressionAlgorithm::Uncompressed {
                // At most min_compression bytes after write_offset.
                fw_assert!(
                    chunk_out_size <= min_compression + compression_offset,
                    chunk_out_size as i32,
                    min_compression as i32,
                    compression_offset as i32
                );
            }
        }

        if alg != CompressionAlgorithm::Uncompressed {
            let comp_data_start = chunk_start + compression_offset;
            // C++ computes this in FwSizeType; a compressor that shrank the
            // buffer below write_offset wraps and trips the assert below.
            let compressed_size = chunk_out_size.wrapping_sub(compression_offset);
            fw_assert!(
                compressed_size <= FwSizeStoreType::MAX as usize,
                compressed_size as i32
            );
            fw_assert!(
                compressed_size < chunk_size,
                compressed_size as i32,
                chunk_size as i32
            );

            // A compressible first chunk behaves as LAST_COMPRESSED: no
            // move or extra header needed.
            if state == ProcState::Init {
                state = ProcState::LastCompressed;
            }
            let compressed = CompressionMetadata::new(alg);
            let raw = CompressionMetadata::new(CompressionAlgorithm::Uncompressed);
            match state {
                ProcState::PreCommit => {
                    // Case A: move the uncompressed prefix up by one header,
                    // prepend its header, then this chunk's header and data.
                    fw_assert!(
                        deser_loc >= uncompressed_size + 2 * H + compressed_size,
                        deser_loc as i32,
                        uncompressed_size as i32,
                        compressed_size as i32
                    );
                    data.copy_within(0..uncompressed_size, H);
                    serialize_compression_header(
                        data,
                        &mut ser_loc,
                        record_id,
                        uncompressed_size,
                        raw,
                    );
                    // Skip past the moved uncompressed run.
                    fw_assert!(ser_loc + uncompressed_size <= total);
                    ser_loc += uncompressed_size;
                    serialize_compression_header(
                        data,
                        &mut ser_loc,
                        record_id,
                        compressed_size,
                        compressed,
                    );
                    serialize_from(data, &mut ser_loc, comp_data_start, compressed_size);
                }
                ProcState::LastCompressed => {
                    // Case B: header, then the compressed data.
                    serialize_compression_header(
                        data,
                        &mut ser_loc,
                        record_id,
                        compressed_size,
                        compressed,
                    );
                    serialize_from(data, &mut ser_loc, comp_data_start, compressed_size);
                }
                ProcState::LastUncompressed => {
                    // Case E: close the pending uncompressed run, then this
                    // chunk's header and data.
                    fw_assert!(
                        deser_loc >= ser_loc + uncompressed_size + 2 * H + compressed_size,
                        deser_loc as i32,
                        ser_loc as i32,
                        uncompressed_size as i32,
                        compressed_size as i32
                    );
                    serialize_compression_header(
                        data,
                        &mut ser_loc,
                        record_id,
                        uncompressed_size,
                        raw,
                    );
                    let Some(head) = uncompressed_head else {
                        fw_assert!(false);
                        return None;
                    };
                    serialize_from(data, &mut ser_loc, head, uncompressed_size);
                    serialize_compression_header(
                        data,
                        &mut ser_loc,
                        record_id,
                        compressed_size,
                        compressed,
                    );
                    serialize_from(data, &mut ser_loc, comp_data_start, compressed_size);
                }
                ProcState::Init => fw_assert!(false),
            }
            // The last chunk compressed: forget the uncompressed history.
            uncompressed_size = 0;
            uncompressed_head = None;
            state = ProcState::LastCompressed;
        } else {
            // An incompressible first chunk behaves as PRE_COMMIT.
            if state == ProcState::Init {
                state = ProcState::PreCommit;
            }
            match state {
                // Keep looking for a compressible chunk to commit to.
                ProcState::PreCommit => {}
                // Case C: the first incompressible chunk after a compressed
                // one — remember where the run starts.
                ProcState::LastCompressed => {
                    uncompressed_head = Some(chunk_start);
                    state = ProcState::LastUncompressed;
                }
                // Case D: the run continues.
                ProcState::LastUncompressed => {}
                ProcState::Init => fw_assert!(false),
            }
            uncompressed_size += chunk_size;
        }

        // The write position never overtakes the read position.
        fw_assert!(ser_loc <= deser_loc, ser_loc as i32, deser_loc as i32);
    }

    match state {
        ProcState::PreCommit => None,
        ProcState::LastUncompressed => {
            // Flush the trailing uncompressed run.
            serialize_compression_header(
                data,
                &mut ser_loc,
                record_id,
                uncompressed_size,
                CompressionMetadata::new(CompressionAlgorithm::Uncompressed),
            );
            let Some(head) = uncompressed_head else {
                fw_assert!(false);
                return None;
            };
            serialize_from(data, &mut ser_loc, head, uncompressed_size);
            Some(ser_loc)
        }
        ProcState::LastCompressed => Some(ser_loc),
        ProcState::Init => {
            fw_assert!(false);
            None
        }
    }
}

impl DpProcPort for DpCompressProc {
    fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer) {
        self.proc_request_handler(port_num, buffer);
    }
}

impl CmdPort for DpCompressProc {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_CHUNK_SIZE_PRM_SET => self.chunk_size_prm_set(op_code, cmd_seq, args),
            Self::OPCODE_CHUNK_SIZE_PRM_SAVE => {
                self.prm_save(op_code, cmd_seq, args, Self::PARAMID_CHUNK_SIZE)
            }
            Self::OPCODE_ENABLE_PRM_SET => self.enable_prm_set(op_code, cmd_seq, args),
            Self::OPCODE_ENABLE_PRM_SAVE => {
                self.prm_save(op_code, cmd_seq, args, Self::PARAMID_ENABLE)
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of `Svc/DpCompressProc/test/ut` (`DpCompressProcTestMain.cpp`,
    //! `DpCompressProcTester.cpp`, `AbstractState.cpp`): a scripted
    //! compressor keyed on each chunk's first byte (`U`ncompressible,
    //! `C` = half size, `M` = exactly `min_compression`, `X` = 3 bytes),
    //! plus the error branches and the parameter commands.
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, PrmGetPort, PrmSetPort};
    use fprime_fw::{LogBuffer, Time};

    const ID_BASE: FwIdType = 0x0410_0000;
    const DP_ID: FwDpIdType = 0x0000_1234;
    const H: usize = COMPRESSION_HEADER_SIZE;

    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<EventRecord>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        prm_sets: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
        /// Canned `prmGetOut` answers: (absolute id, bytes, validity).
        prm_get: Mutex<Vec<(FwPrmIdType, Vec<u8>, ParamValid)>>,
    }

    impl Ground {
        fn events_with(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == ID_BASE + id)
                .map(|e| e.2.clone())
                .collect()
        }
        fn clear(&self) {
            self.events.lock().unwrap().clear();
            self.responses.lock().unwrap().clear();
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

    impl CmdRegPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for Ground {
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

    impl PrmGetPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwPrmIdType,
            val: &mut ParamBuffer,
        ) -> ParamValid {
            for (pid, bytes, valid) in self.prm_get.lock().unwrap().iter() {
                if *pid == id {
                    val.reset_ser();
                    let status = val.serialize_bytes(
                        bytes,
                        fprime_fw::LengthMode::OmitLength,
                        Endianness::Big,
                    );
                    assert!(status.is_ok());
                    return *valid;
                }
            }
            ParamValid::Invalid
        }
    }

    impl PrmSetPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
            self.prm_sets
                .lock()
                .unwrap()
                .push((id, val.as_slice().to_vec()));
        }
    }

    /// The upstream `AbstractState::Compressible` chunk kinds.
    const U: u8 = b'U';
    const C: u8 = b'C';
    const M: u8 = b'M';
    const X: u8 = b'X';

    /// The scripted compressor (`DpCompressProcTester::from_compressChunk_handler`)
    /// and the expected-size bookkeeping of `AbstractState`.
    #[derive(Default)]
    struct FakeCompressor {
        calls: Mutex<Vec<(usize, FwSizeType, FwSizeType)>>,
        exp_compressed_size: Mutex<usize>,
        last_uncompressed: Mutex<bool>,
    }

    impl FakeCompressor {
        fn reset(&self) {
            self.calls.lock().unwrap().clear();
            *self.exp_compressed_size.lock().unwrap() = 0;
            *self.last_uncompressed.lock().unwrap() = false;
        }

        fn update(&self, compressed_size: usize, kind: u8) {
            let mut exp = self.exp_compressed_size.lock().unwrap();
            let mut last = self.last_uncompressed.lock().unwrap();
            *exp += compressed_size;
            if kind != U || !*last {
                // A header for this chunk.
                *exp += H;
            }
            *last = kind == U;
        }
    }

    impl CompressChunkPort for FakeCompressor {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            buffer: &mut ChunkBuffer<'_>,
            min_compression: FwSizeType,
            write_offset: FwSizeType,
        ) -> CompressionAlgorithm {
            self.calls
                .lock()
                .unwrap()
                .push((buffer.size(), min_compression, write_offset));
            let kind = buffer.data()[0];
            let sentinel = buffer.data()[1];
            let size = buffer.size();
            let off = write_offset as usize;
            let out_size = match kind {
                U => {
                    self.update(size, kind);
                    return CompressionAlgorithm::Uncompressed;
                }
                C => size / 2 + off,
                M => min_compression as usize + off,
                X => 3 + off,
                _ => panic!("unexpected chunk kind {kind:#x}"),
            };
            assert!(out_size <= size);
            let data = buffer.data_mut();
            data[off] = kind;
            data[off + 1..out_size].fill(sentinel);
            buffer.set_size(out_size);
            self.update(out_size - off, kind);
            CompressionAlgorithm::ZlibDeflate
        }
    }

    struct Fixture {
        comp: Arc<DpCompressProc>,
        ground: Arc<Ground>,
        compressor: Arc<FakeCompressor>,
    }

    fn fixture() -> Fixture {
        let comp = DpCompressProc::new("dpCompressProc");
        comp.base.set_id_base(ID_BASE);
        let ground = Arc::new(Ground::default());
        let compressor = Arc::new(FakeCompressor::default());
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.prm.prm_get_out.connect(ground.clone(), 0);
        comp.prm.prm_set_out.connect(ground.clone(), 0);
        comp.compress_chunk.connect(compressor.clone(), 0);
        comp.load_parameters();
        Fixture {
            comp,
            ground,
            compressor,
        }
    }

    /// `AbstractState::build_compress_buffer`.
    fn build_compress_buffer(chunk_size: usize, chunks: &[(u8, u8)]) -> Buffer {
        let mut container = DpContainer::with_buffer(
            DP_ID,
            Buffer::allocate(chunk_size * chunks.len() + DpContainer::MIN_PACKET_SIZE),
        );
        {
            let data = container.data_region_mut();
            for (idx, &(kind, sentinel)) in chunks.iter().enumerate() {
                let start = idx * chunk_size;
                data[start] = kind;
                data[start + 1..start + chunk_size].fill(sentinel);
            }
        }
        container.set_data_size((chunk_size * chunks.len()) as FwSizeType);
        container.serialize_header();
        container.take_buffer()
    }

    fn read_header(buffer: Buffer) -> (DpContainer, SerializeStatusAlias) {
        let mut container = DpContainer::new();
        container.set_buffer(buffer);
        let status = container.deserialize_header();
        (container, status)
    }

    type SerializeStatusAlias = fprime_fw::SerializeStatus;

    /// `DpCompressProcTester::uncompress_data`: walk the records, check
    /// their framing, and rebuild the original data.
    fn uncompress_data(container: &DpContainer, chunk_size: usize) -> Vec<u8> {
        let data = container.data();
        let mut out = Vec::new();
        let mut pos = 0;
        while pos < data.len() {
            let id = u32::from_be_bytes(data[pos..pos + 4].try_into().unwrap());
            assert_eq!(id, ID_BASE + DpCompressProc::RECORDID_COMPRESSION_RECORD);
            let record_size = u16::from_be_bytes([data[pos + 4], data[pos + 5]]) as usize;
            pos += 6;
            assert!(record_size > CompressionMetadata::SERIALIZED_SIZE);
            assert!(record_size <= data.len() - pos);
            let algorithm = data[pos];
            pos += 1;
            let payload = &data[pos..pos + record_size - 1];
            match CompressionAlgorithm::try_from(algorithm) {
                Ok(CompressionAlgorithm::Uncompressed) => {
                    assert_eq!(payload.len() % chunk_size, 0);
                    for raw in payload.chunks(chunk_size) {
                        assert_eq!(raw[0], U);
                        assert_ne!(raw[1], 0);
                        assert!(raw[2..].iter().all(|&b| b == raw[1]));
                        out.extend_from_slice(raw);
                    }
                }
                Ok(CompressionAlgorithm::ZlibDeflate) => {
                    assert!([C, M, X].contains(&payload[0]), "{:#x}", payload[0]);
                    assert_ne!(payload[1], 0);
                    assert!(payload[2..].iter().all(|&b| b == payload[1]));
                    out.push(payload[0]);
                    out.extend(std::iter::repeat_n(payload[1], chunk_size - 1));
                }
                Err(raw) => panic!("bad algorithm {raw}"),
            }
            pos += payload.len();
        }
        out
    }

    /// `DpCompressProcTester::test_chunks_helper` with `ENABLE = ENABLED`.
    fn test_chunks(f: &Fixture, chunk_size: usize, chunks: &[(u8, u8)]) {
        f.ground.clear();
        f.comp.clear_event_throttles();
        f.compressor.reset();
        set_param_chunk_size(f, chunk_size as FwSizeStoreType);

        let mut buffer = build_compress_buffer(chunk_size, chunks);
        let original_size = buffer.size();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        // DpCompressProc never resizes the buffer (DpWriter shrinks it).
        assert_eq!(buffer.size(), original_size);

        let should_compress = chunks.iter().any(|c| c.0 != U);
        let did_not = f
            .ground
            .events_with(DpCompressProc::EVENTID_DID_NOT_COMPRESS);
        let complete = f
            .ground
            .events_with(DpCompressProc::EVENTID_COMPRESSION_COMPLETE);
        let (container, status) = read_header(buffer);
        assert!(status.is_ok());
        assert_eq!(container.check_header_hash().0, fprime_fw::Success::Success);
        if should_compress {
            assert!(did_not.is_empty());
            let exp_size = *f.compressor.exp_compressed_size.lock().unwrap();
            assert_eq!(container.data_size() as usize, exp_size);
            let rebuilt = uncompress_data(&container, chunk_size);
            assert_eq!(rebuilt.len(), chunk_size * chunks.len());
            // AbstractState::check_uncompressed_data
            for (idx, &(kind, sentinel)) in chunks.iter().enumerate() {
                assert_eq!(rebuilt[idx * chunk_size], kind);
                assert_eq!(rebuilt[idx * chunk_size + 1], sentinel);
            }
            assert_eq!(complete.len(), 1);
            let mut expected = Vec::new();
            expected.extend_from_slice(&DP_ID.to_be_bytes());
            expected.extend_from_slice(&((chunk_size * chunks.len()) as u64).to_be_bytes());
            expected.extend_from_slice(&(exp_size as u64).to_be_bytes());
            assert_eq!(complete[0], expected);
        } else {
            assert_eq!(did_not.len(), 1);
            assert_eq!(container.data_size() as usize, chunk_size * chunks.len());
            let expected = build_compress_buffer(chunk_size, chunks);
            assert_eq!(container.buffer().data(), expected.data());
            assert!(complete.is_empty());
        }
    }

    fn send_cmd(f: &Fixture, opcode: FwOpcodeType, args: &[u8]) -> CmdResponse {
        let mut buf = CmdArgBuffer::new();
        let status = buf.serialize_bytes(args, fprime_fw::LengthMode::OmitLength, Endianness::Big);
        assert!(status.is_ok());
        f.ground.responses.lock().unwrap().clear();
        CmdPort::invoke(&*f.comp, 0, ID_BASE + opcode, 7, &mut buf);
        let responses = f.ground.responses.lock().unwrap();
        assert_eq!(responses.len(), 1);
        assert_eq!(responses[0].0, ID_BASE + opcode);
        assert_eq!(responses[0].1, 7);
        responses[0].2
    }

    fn set_param_chunk_size(f: &Fixture, chunk_size: FwSizeStoreType) {
        assert_eq!(
            send_cmd(
                f,
                DpCompressProc::OPCODE_CHUNK_SIZE_PRM_SET,
                &chunk_size.to_be_bytes()
            ),
            CmdResponse::Ok
        );
    }

    fn set_param_enable(f: &Fixture, enable: Enabled) {
        assert_eq!(
            send_cmd(
                f,
                DpCompressProc::OPCODE_ENABLE_PRM_SET,
                &[enable.as_repr()]
            ),
            CmdResponse::Ok
        );
    }

    fn chunks_of(kinds: &[u8], sentinels: &[u8]) -> Vec<(u8, u8)> {
        kinds
            .iter()
            .copied()
            .zip(sentinels.iter().copied())
            .collect()
    }

    // -- Upstream TEST cases ---------------------------------------------------

    #[test]
    fn nominal_compressible() {
        let f = fixture();
        test_chunks(&f, 4096, &chunks_of(&[C, C, C], &[0xA5, 0x55, 0x23]));
    }

    #[test]
    fn nominal_disabled_leaves_the_container_untouched() {
        // Upstream `Nominal.Disabled`: compress once while enabled, then
        // run the (now compressed) container through with ENABLE=DISABLED.
        let f = fixture();
        let chunks = chunks_of(&[C, C, C], &[0xA5, 0x55, 0x23]);
        test_chunks(&f, 4096, &chunks);

        set_param_enable(&f, Enabled::Disabled);
        f.compressor.reset();
        f.ground.clear();
        let mut buffer = build_compress_buffer(4096, &chunks);
        let before = buffer.data().to_vec();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        assert_eq!(buffer.data(), &before[..]);
        assert!(f.compressor.calls.lock().unwrap().is_empty());
        assert!(f.ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn nominal_minimal_compressible() {
        let f = fixture();
        test_chunks(&f, 4096, &chunks_of(&[M, M, M], &[0xA5, 0x55, 0x23]));
    }

    #[test]
    fn nominal_minimal_compressible_mixed() {
        let f = fixture();
        test_chunks(
            &f,
            4096,
            &chunks_of(&[U, M, U, M], &[0xA5, 0x55, 0x24, 0x23]),
        );
    }

    #[test]
    fn nominal_maximal_compressible() {
        let f = fixture();
        test_chunks(&f, 4096, &chunks_of(&[X, X, X], &[0xA5, 0x55, 0x23]));
    }

    #[test]
    fn nominal_uncompressible() {
        let f = fixture();
        test_chunks(&f, 4096, &chunks_of(&[U, U, U], &[0xA5, 0x55, 0x23]));
    }

    #[test]
    fn nominal_mixed_first_compressible() {
        let f = fixture();
        test_chunks(&f, 4096, &chunks_of(&[C, U, U], &[0xA5, 0x55, 0x23]));
    }

    #[test]
    fn nominal_mixed_first_uncompressible() {
        let f = fixture();
        test_chunks(
            &f,
            4096,
            &chunks_of(&[U, U, C, C], &[0xA5, 0xA4, 0x55, 0x23]),
        );
    }

    #[test]
    fn nominal_failure_helper() {
        // The upstream regression string.
        let f = fixture();
        let kinds = b"MUUMMMUMMUMMCUUXUUUXXCCCCCCCCCXCMCMMUCUMMXXXUXMCUCCCUUXCXXXUXC";
        let chunks: Vec<(u8, u8)> = kinds
            .iter()
            .enumerate()
            .map(|(idx, &k)| (k, (idx as u8) | 0x80))
            .collect();
        test_chunks(&f, 512, &chunks);
    }

    #[test]
    fn nominal_random() {
        // Upstream runs 10000 time-seeded iterations; a fixed xorshift seed
        // keeps this reproducible (and 2000 keeps it quick in debug).
        let f = fixture();
        let mut state: u32 = 0x2545_F491;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        for _ in 0..2000 {
            let num_chunks = 1 + (next() % 100) as usize;
            let chunks: Vec<(u8, u8)> = (0..num_chunks)
                .map(|_| {
                    let kind = [C, M, X, U][(next() % 4) as usize];
                    let sentinel = 1 + (next() % 255) as u8;
                    (kind, sentinel)
                })
                .collect();
            test_chunks(&f, 512, &chunks);
        }
    }

    #[test]
    fn buffer_smaller_than_min_packet_size() {
        // A view one byte short of MIN_PACKET_SIZE over a well-formed header.
        let f = fixture();
        set_param_chunk_size(&f, 4096);
        let mut backing = build_compress_buffer(1, &[(U, 1)]);
        backing.set_size(DpContainer::MIN_PACKET_SIZE - 1);
        let before = backing.data().to_vec();
        DpProcPort::invoke(&*f.comp, 0, &mut backing);

        let events = f
            .ground
            .events_with(DpCompressProc::EVENTID_BUFFER_TOO_SMALL_FOR_PACKET);
        assert_eq!(events.len(), 1);
        let mut expected = Vec::new();
        expected.extend_from_slice(&((DpContainer::MIN_PACKET_SIZE - 1) as u64).to_be_bytes());
        expected.extend_from_slice(&(DpContainer::MIN_PACKET_SIZE as u64).to_be_bytes());
        assert_eq!(events[0], expected);
        assert!(
            f.ground
                .events_with(DpCompressProc::EVENTID_INVALID_HEADER)
                .is_empty()
        );
        assert!(f.compressor.calls.lock().unwrap().is_empty());
        assert_eq!(backing.size(), DpContainer::MIN_PACKET_SIZE - 1);
        assert_eq!(backing.data(), &before[..]);
    }

    #[test]
    fn buffer_too_large_for_record_size() {
        let f = fixture();
        set_param_chunk_size(&f, 4096);
        let size = FwSizeStoreType::MAX as usize;
        let mut container = DpContainer::with_buffer(DP_ID, Buffer::allocate(size));
        container.set_data_size((size - DpContainer::MIN_PACKET_SIZE) as FwSizeType);
        container.serialize_header();
        let mut buffer = container.take_buffer();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);

        assert!(
            f.ground
                .events_with(DpCompressProc::EVENTID_INVALID_HEADER)
                .is_empty()
        );
        let events = f
            .ground
            .events_with(DpCompressProc::EVENTID_CONTAINER_TOO_LARGE);
        assert_eq!(events.len(), 1);
        let mut expected = DP_ID.to_be_bytes().to_vec();
        expected.extend_from_slice(&((size - DpContainer::MIN_PACKET_SIZE) as u64).to_be_bytes());
        assert_eq!(events[0], expected);
        assert!(f.compressor.calls.lock().unwrap().is_empty());
        assert_eq!(buffer.size(), size);
    }

    // -- Further branches -------------------------------------------------------

    #[test]
    fn invalid_header_is_reported_with_the_serialize_status() {
        let f = fixture();
        let mut buffer = build_compress_buffer(512, &[(C, 1)]);
        // Corrupt the packet descriptor (must be FW_PACKET_DP).
        buffer.data_mut()[1] = 0x00;
        let size = buffer.size();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        let events = f.ground.events_with(DpCompressProc::EVENTID_INVALID_HEADER);
        assert_eq!(events.len(), 1);
        let mut expected = (size as u64).to_be_bytes().to_vec();
        expected.extend_from_slice(
            &(fprime_fw::SerializeStatus::FormatError as i32 as u32).to_be_bytes(),
        );
        assert_eq!(events[0], expected);
        assert!(f.compressor.calls.lock().unwrap().is_empty());
    }

    #[test]
    fn data_size_beyond_the_buffer_is_rejected() {
        let f = fixture();
        let mut container =
            DpContainer::with_buffer(DP_ID, Buffer::allocate(DpContainer::MIN_PACKET_SIZE + 100));
        container.set_data_size(101);
        container.serialize_header();
        let mut buffer = container.take_buffer();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        let events = f
            .ground
            .events_with(DpCompressProc::EVENTID_BUFFER_TOO_SMALL_FOR_PACKET);
        assert_eq!(events.len(), 1);
        let mut expected = ((DpContainer::MIN_PACKET_SIZE + 100) as u64)
            .to_be_bytes()
            .to_vec();
        expected.extend_from_slice(&((DpContainer::MIN_PACKET_SIZE + 101) as u64).to_be_bytes());
        assert_eq!(events[0], expected);
    }

    #[test]
    fn unconnected_compressor_bypasses_compression() {
        let comp = DpCompressProc::new("dpCompressProc");
        let ground = Arc::new(Ground::default());
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.prm.prm_get_out.connect(ground.clone(), 0);
        comp.load_parameters();
        let mut buffer = build_compress_buffer(512, &[(C, 1), (C, 2)]);
        let before = buffer.data().to_vec();
        DpProcPort::invoke(&*comp, 0, &mut buffer);
        assert_eq!(buffer.data(), &before[..]);
        assert!(ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn zero_chunk_size_or_empty_container_gives_up_silently() {
        let f = fixture();
        set_param_chunk_size(&f, 0);
        let mut buffer = build_compress_buffer(512, &[(C, 1)]);
        let before = buffer.data().to_vec();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        assert_eq!(buffer.data(), &before[..]);
        assert!(f.ground.events.lock().unwrap().is_empty());

        set_param_chunk_size(&f, 512);
        f.ground.clear();
        let mut buffer = build_compress_buffer(512, &[]);
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        assert!(f.ground.events.lock().unwrap().is_empty());
        assert!(f.compressor.calls.lock().unwrap().is_empty());
    }

    #[test]
    fn chunks_of_four_headers_or_less_are_not_offered() {
        let f = fixture();
        // 4 * H bytes: below the compression threshold, so even compressible
        // chunks stay raw and the container is reported as not compressed.
        set_param_chunk_size(&f, (4 * H) as FwSizeStoreType);
        let mut buffer = build_compress_buffer(4 * H, &chunks_of(&[C, C], &[1, 2]));
        let before = buffer.data().to_vec();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        assert_eq!(buffer.data(), &before[..]);
        assert!(f.compressor.calls.lock().unwrap().is_empty());
        assert_eq!(
            f.ground
                .events_with(DpCompressProc::EVENTID_DID_NOT_COMPRESS)
                .len(),
            1
        );
        // One more byte and each chunk is offered.
        test_chunks(&f, 4 * H + 1, &chunks_of(&[X, X], &[1, 2]));
        assert_eq!(f.compressor.calls.lock().unwrap().len(), 2);
    }

    #[test]
    fn min_compression_and_write_offset_follow_the_state_machine() {
        let f = fixture();
        test_chunks(&f, 512, &chunks_of(&[U, X, U, U, X], &[1, 2, 3, 4, 5]));
        let calls = f.compressor.calls.lock().unwrap().clone();
        let h = H as FwSizeType;
        assert_eq!(
            calls,
            vec![
                (512, 512 - 2 * h, h),     // INIT
                (512, 512 - 3 * h, 2 * h), // PRE_COMMIT
                (512, 512 - 2 * h, h),     // LAST_COMPRESSED
                (512, 512 - 2 * h, h),     // LAST_UNCOMPRESSED
                (512, 512 - 2 * h, h),     // LAST_UNCOMPRESSED
            ]
        );
    }

    #[test]
    fn last_chunk_shorter_than_chunk_size() {
        // 3 full chunks + a 100-byte tail; the tail is its own chunk.
        let f = fixture();
        set_param_chunk_size(&f, 512);
        let mut container = DpContainer::with_buffer(
            DP_ID,
            Buffer::allocate(3 * 512 + 100 + DpContainer::MIN_PACKET_SIZE),
        );
        {
            let data = container.data_region_mut();
            for (idx, chunk) in data.chunks_mut(512).enumerate() {
                chunk[0] = X;
                chunk[1..].fill(idx as u8 + 1);
            }
        }
        container.set_data_size(3 * 512 + 100);
        container.serialize_header();
        let mut buffer = container.take_buffer();
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        let calls = f.compressor.calls.lock().unwrap().clone();
        assert_eq!(
            calls.iter().map(|c| c.0).collect::<Vec<_>>(),
            vec![512, 512, 512, 100]
        );
        let (container, _) = read_header(buffer);
        assert_eq!(container.data_size() as usize, 4 * (H + 3));
    }

    #[test]
    fn compressed_record_wire_format() {
        // One maximally compressible chunk: [id][size = 1 + 3][alg = 1][payload].
        let f = fixture();
        set_param_chunk_size(&f, 64);
        let mut buffer = build_compress_buffer(64, &[(X, 0x42)]);
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        let (container, _) = read_header(buffer);
        let mut expected = (ID_BASE + DpCompressProc::RECORDID_COMPRESSION_RECORD)
            .to_be_bytes()
            .to_vec();
        expected.extend_from_slice(&[0x00, 0x04, 0x01, X, 0x42, 0x42]);
        assert_eq!(container.data(), &expected[..]);
        assert_eq!(COMPRESSION_HEADER_SIZE, 7);
        assert_eq!(CompressionMetadata::SERIALIZED_SIZE, 1);
        assert_eq!(CompressionAlgorithm::Uncompressed.as_repr(), 0);
        assert_eq!(CompressionAlgorithm::ZlibDeflate.as_repr(), 1);
    }

    #[test]
    fn did_not_compress_is_throttled_at_ten() {
        let f = fixture();
        set_param_chunk_size(&f, 64);
        for _ in 0..12 {
            let mut buffer = build_compress_buffer(64, &[(U, 1)]);
            DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        }
        assert_eq!(
            f.ground
                .events_with(DpCompressProc::EVENTID_DID_NOT_COMPRESS)
                .len(),
            10
        );
        f.comp.clear_event_throttles();
        let mut buffer = build_compress_buffer(64, &[(U, 1)]);
        DpProcPort::invoke(&*f.comp, 0, &mut buffer);
        assert_eq!(
            f.ground
                .events_with(DpCompressProc::EVENTID_DID_NOT_COMPRESS)
                .len(),
            11
        );
        assert!(
            f.ground
                .events
                .lock()
                .unwrap()
                .iter()
                .all(|e| e.1 == LogSeverity::ActivityLo)
        );
    }

    #[test]
    #[should_panic]
    fn proc_request_before_load_parameters_asserts() {
        // C++ parity: paramGet_ENABLE is UNINIT -> FW_ASSERT(FW_PARAM_OK).
        let comp = DpCompressProc::new("dpCompressProc");
        let mut buffer = build_compress_buffer(64, &[(C, 1)]);
        DpProcPort::invoke(&*comp, 0, &mut buffer);
    }

    // -- Parameters and commands ------------------------------------------------

    #[test]
    fn reg_commands_registers_the_parameter_opcodes() {
        let f = fixture();
        f.comp.reg_commands();
        assert_eq!(
            *f.ground.regs.lock().unwrap(),
            vec![ID_BASE, ID_BASE + 1, ID_BASE + 2, ID_BASE + 3]
        );
    }

    #[test]
    fn load_parameters_takes_valid_values_and_defaults_the_rest() {
        let f = fixture();
        assert_eq!(
            f.comp.param_get_chunk_size(),
            (32 * 1024, ParamValid::Default)
        );
        assert_eq!(
            f.comp.param_get_enable(),
            (Enabled::Enabled, ParamValid::Default)
        );

        *f.ground.prm_get.lock().unwrap() = vec![
            (
                ID_BASE + DpCompressProc::PARAMID_CHUNK_SIZE,
                vec![0x01, 0x00],
                ParamValid::Valid,
            ),
            (
                ID_BASE + DpCompressProc::PARAMID_ENABLE,
                vec![0x00],
                ParamValid::Valid,
            ),
        ];
        f.comp.load_parameters();
        assert_eq!(f.comp.param_get_chunk_size(), (256, ParamValid::Valid));
        assert_eq!(
            f.comp.param_get_enable(),
            (Enabled::Disabled, ParamValid::Valid)
        );

        // A corrupt stored enum (and a short chunk size) fall back to defaults.
        *f.ground.prm_get.lock().unwrap() = vec![
            (
                ID_BASE + DpCompressProc::PARAMID_CHUNK_SIZE,
                vec![0x01],
                ParamValid::Valid,
            ),
            (
                ID_BASE + DpCompressProc::PARAMID_ENABLE,
                vec![0x07],
                ParamValid::Valid,
            ),
        ];
        f.comp.load_parameters();
        assert_eq!(
            f.comp.param_get_chunk_size(),
            (32 * 1024, ParamValid::Default)
        );
        assert_eq!(
            f.comp.param_get_enable(),
            (Enabled::Enabled, ParamValid::Default)
        );
    }

    #[test]
    fn parameter_commands() {
        let f = fixture();
        assert_eq!(
            send_cmd(&f, DpCompressProc::OPCODE_CHUNK_SIZE_PRM_SET, &[0x02]),
            CmdResponse::FormatError
        );
        assert_eq!(
            send_cmd(
                &f,
                DpCompressProc::OPCODE_CHUNK_SIZE_PRM_SET,
                &[0x02, 0x00, 0x00]
            ),
            CmdResponse::FormatError
        );
        assert_eq!(
            send_cmd(&f, DpCompressProc::OPCODE_CHUNK_SIZE_PRM_SET, &[0x02, 0x00]),
            CmdResponse::Ok
        );
        assert_eq!(f.comp.param_get_chunk_size(), (512, ParamValid::Valid));
        assert_eq!(
            send_cmd(&f, DpCompressProc::OPCODE_ENABLE_PRM_SET, &[0x02]),
            CmdResponse::ValidationError
        );
        assert_eq!(
            f.comp.param_get_enable(),
            (Enabled::Enabled, ParamValid::Default)
        );
        assert_eq!(
            send_cmd(&f, DpCompressProc::OPCODE_ENABLE_PRM_SET, &[0x00]),
            CmdResponse::Ok
        );
        assert_eq!(
            f.comp.param_get_enable(),
            (Enabled::Disabled, ParamValid::Valid)
        );

        assert_eq!(
            send_cmd(&f, DpCompressProc::OPCODE_CHUNK_SIZE_PRM_SAVE, &[]),
            CmdResponse::Ok
        );
        assert_eq!(
            send_cmd(&f, DpCompressProc::OPCODE_ENABLE_PRM_SAVE, &[]),
            CmdResponse::Ok
        );
        assert_eq!(
            send_cmd(&f, DpCompressProc::OPCODE_ENABLE_PRM_SAVE, &[0]),
            CmdResponse::FormatError
        );
        assert_eq!(
            *f.ground.prm_sets.lock().unwrap(),
            vec![
                (
                    ID_BASE + DpCompressProc::PARAMID_CHUNK_SIZE,
                    vec![0x02, 0x00]
                ),
                (ID_BASE + DpCompressProc::PARAMID_ENABLE, vec![0x00]),
            ]
        );
        assert_eq!(send_cmd(&f, 0x04, &[]), CmdResponse::InvalidOpcode);
    }

    #[test]
    fn parameter_save_without_prm_set_out_is_an_execution_error() {
        let comp = DpCompressProc::new("dpCompressProc");
        let ground = Arc::new(Ground::default());
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        let mut args = CmdArgBuffer::new();
        CmdPort::invoke(
            &*comp,
            0,
            DpCompressProc::OPCODE_ENABLE_PRM_SAVE,
            1,
            &mut args,
        );
        assert_eq!(
            ground.responses.lock().unwrap()[0].2,
            CmdResponse::ExecutionError
        );
    }
}
