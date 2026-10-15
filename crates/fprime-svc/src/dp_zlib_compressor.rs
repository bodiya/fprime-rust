//! `Svc::DpZLibCompressor` — zlib DEFLATE chunk compressor (PASSIVE).
//!
//! Port of `Svc/DpZLibCompressor/DpZLibCompressor.{fpp,hpp,cpp}` and
//! `Types/ZLib.fpp`. Upstream documentation:
//! `Svc/DpZLibCompressor/docs/sdd.md`. It serves
//! [`DpCompressProc`](crate::dp_compress_proc::DpCompressProc)'s
//! `compressChunk` port: each call deflates one chunk into a scratch
//! buffer from `bufferCompressionGet` and, if the zlib stream fits in
//! `min_compression` bytes, copies it into the chunk at `write_offset`.
//!
//! **Only compiled with the `zlib` cargo feature** (off by default; it pulls
//! in the pure-Rust `miniz_oxide` crate, the workspace's only third-party
//! dependency). Upstream behaves the same way: `Svc/DpZLibCompressor`'s
//! CMake registers the module only when libz is found, and the
//! `DpCompression` subtopology needs it. Without the feature
//! `DpCompressProc` still builds; with its `compressChunk` port unconnected
//! it leaves every data product uncompressed.
//!
//! # Compression
//!
//! The output is a zlib (RFC 1950) stream — a 2-byte header, DEFLATE data
//! and an Adler-32 trailer — exactly what C++ `deflateInit(level)` +
//! `deflate(Z_FINISH)` produce and what the ground decoder hands to Python's
//! `zlib.decompress`. `CompressionLevel` is the zlib level: −1 means the
//! default (6), 0–9 select the speed/size trade-off, anything else fails
//! like `deflateInit` does (`ZLibInitError`, `Z_STREAM_ERROR`).
//!
//! # Deviations from C++
//!
//! * **Different encoder, same format.** The DEFLATE encoder is
//!   `miniz_oxide`, not libz, so the compressed bytes (and therefore the
//!   compressed size of a given chunk) differ from upstream's; both are
//!   valid zlib streams that any inflater — including the ground's — reads
//!   back to the same data. A chunk upstream compresses to N bytes may come
//!   out a little larger or smaller here, so the `ZLibNoCompression`
//!   threshold is crossed at slightly different `min_compression` values.
//! * **Working memory.** C++ points zlib's `zalloc` at a bump allocator over
//!   the `bufferZLibGet` buffer (`ZLibBufferSize` bytes). `miniz_oxide`
//!   cannot allocate from a caller's arena, so its encoder state is
//!   allocated ONCE, at construction, and reset for each chunk — the steady
//!   state allocates nothing. The arena is still requested and returned
//!   exactly as upstream does (so `ZLibAllocBadBuffer`, the BufferManager
//!   sizing of the `DpCompression` subtopology and the `ZLibBufferSize`
//!   parameter keep their meaning), and an arena smaller than what
//!   `deflateInit` needs ([`ZLIB_DEFLATE_MEMORY`]) fails as `deflateInit`
//!   does: `ZLibInitError(Z_MEM_ERROR, "insufficient memory")`.
//!   `ZLibMemoryUsage` reports that same figure as `memory_used`.
//! * **Stack.** Resetting the encoder builds its 64 KiB LZ code buffer in
//!   place, which may use that much stack transiently in unoptimized
//!   builds; give the calling thread (`DpWriter`'s) a stack well above
//!   64 KiB (the Rust Ref uses the platform default).
//! * **Serialized calls.** The single preallocated encoder sits behind a
//!   mutex, so concurrent `compressChunk` calls (several `DpWriter`s on one
//!   compressor) run one at a time; C++ keeps all state on the stack and in
//!   per-call buffers.
//! * `ZLibDeflateError` carries `Z_STREAM_ERROR`/`Z_BUF_ERROR` mapped from
//!   the encoder's status and an empty message (zlib's `msg` text has no
//!   `miniz_oxide` equivalent); a too-small output buffer is not an error
//!   in either library (zlib returns `Z_OK`), only `ZLibNoCompression`.

use crate::dp_compress_proc::{ChunkBuffer, CompressChunkPort, CompressionAlgorithm};
use fprime_comp::{
    BufferGetPort, BufferSendPort, CmdGlue, CmdPort, EventGlue, EventThrottle, OutputPort,
    PassiveBase, PortRef, PrmGlue,
};
use fprime_config::{
    FW_LOG_STRING_MAX_SIZE, FwEventIdType, FwIdType, FwIndexType, FwOpcodeType, FwPrmIdType,
    FwSizeType,
};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Endianness, FwDefaultString, LogSeverity, ParamBuffer,
    ParamValid, SerBuf, fpp_enum, fw_assert, fw_try,
};
use miniz_oxide::DataFormat;
use miniz_oxide::deflate::core::{CompressorOxide, TDEFLFlush, TDEFLStatus, compress};
use std::sync::{Arc, Mutex};

fpp_enum! {
    /// `Svc.ZLibData` (`Types/ZLib.fpp`) — declared upstream, unused by the
    /// component.
    pub enum ZLibData : i32 {
        /// Binary data.
        Binary = 0,
        /// Text data.
        Text = 1,
    }
    default Binary
}

// -- zlib return codes used in events (zlib.h) --------------------------------

/// `Z_STREAM_ERROR`.
pub const Z_STREAM_ERROR: i32 = -2;
/// `Z_MEM_ERROR`.
pub const Z_MEM_ERROR: i32 = -4;
/// `Z_BUF_ERROR`.
pub const Z_BUF_ERROR: i32 = -5;
/// zlib's `Z_DEFAULT_COMPRESSION` (−1, meaning level 6).
pub const Z_DEFAULT_COMPRESSION: i8 = -1;

/// The bytes C++ `deflateInit(level)` takes from the bump allocator:
/// `windowBits` 15 and `memLevel` 8 give a 64 KiB window, 64 KiB `prev`,
/// 64 KiB `head` and a 64 KiB `pending_buf` (262 144 bytes) plus the
/// `deflate_state` struct, independent of the level. Upstream's
/// `InsufficientZLibMem` test pins the total at 268 096 (one byte less
/// fails `deflateInit`); see the module notes for how the Rust port uses it.
pub const ZLIB_DEFLATE_MEMORY: FwSizeType = 268_096;

/// zlib's `uInt` limit on `avail_in`/`avail_out` (`BufferTooBigForZLib`).
pub const ZLIB_UINT_MAX: FwSizeType = u32::MAX as FwSizeType;

/// zlib's message for `Z_MEM_ERROR` (`z_errmsg`).
const ZLIB_MEM_ERROR_MSG: &str = "insufficient memory";

/// The parameter store (autocoded `m_<param>` / `m_param_<param>_valid`).
#[derive(Debug)]
struct DpZLibCompressorParams {
    compression_level: i8,
    compression_level_valid: ParamValid,
    zlib_buffer_size: FwSizeType,
    zlib_buffer_size_valid: ParamValid,
}

impl Default for DpZLibCompressorParams {
    fn default() -> Self {
        Self {
            compression_level: DpZLibCompressor::COMPRESSION_LEVEL_DEFAULT,
            compression_level_valid: ParamValid::Uninit,
            zlib_buffer_size: DpZLibCompressor::ZLIB_BUFFER_SIZE_DEFAULT,
            zlib_buffer_size_valid: ParamValid::Uninit,
        }
    }
}

/// `FW_PARAM_OK(valid)`.
fn param_ok(valid: ParamValid) -> bool {
    matches!(valid, ParamValid::Valid | ParamValid::Default)
}

/// The per-call context (C++ `ZLibCtx`): the two borrowed buffers.
struct ZLibCtx {
    compression_buffer: Buffer,
    zlib_alloc_buffer: Buffer,
}

/// `Svc::DpZLibCompressor` — passive zlib chunk compressor.
pub struct DpZLibCompressor {
    /// Passive core (name, id base).
    pub base: PassiveBase,
    /// Command registration/response ports (parameter commands only).
    pub cmd: CmdGlue,
    /// Event ports and the `timeCaller` port.
    pub evt: EventGlue,
    /// Parameter ports (`prmGetOut`, `prmSetOut`).
    pub prm: PrmGlue,
    /// `bufferCompressionGet` (`Fw.BufferGet`): the scratch output buffer.
    pub buffer_compression_get: OutputPort<dyn BufferGetPort>,
    /// `bufferCompressionReturn` (`Fw.BufferSend`).
    pub buffer_compression_return: OutputPort<dyn BufferSendPort>,
    /// `bufferZLibGet` (`Fw.BufferGet`): the zlib working-memory arena.
    pub buffer_zlib_get: OutputPort<dyn BufferGetPort>,
    /// `bufferZLibReturn` (`Fw.BufferSend`).
    pub buffer_zlib_return: OutputPort<dyn BufferSendPort>,
    compression_bad_buffer_throttle: EventThrottle,
    alloc_bad_buffer_throttle: EventThrottle,
    init_error_throttle: EventThrottle,
    deflate_error_throttle: EventThrottle,
    buffer_too_big_throttle: EventThrottle,
    params: Mutex<DpZLibCompressorParams>,
    /// The encoder, allocated once (see the module notes).
    compressor: Mutex<Box<CompressorOxide>>,
}

impl DpZLibCompressor {
    // -- Commands (FPP-relative opcodes: the implicit parameter commands) --

    /// `COMPRESSIONLEVEL_PRM_SET(val: I8)`.
    pub const OPCODE_COMPRESSIONLEVEL_PRM_SET: FwOpcodeType = 0x00;
    /// `COMPRESSIONLEVEL_PRM_SAVE`.
    pub const OPCODE_COMPRESSIONLEVEL_PRM_SAVE: FwOpcodeType = 0x01;
    /// `ZLIBBUFFERSIZE_PRM_SET(val: FwSizeType)`.
    pub const OPCODE_ZLIBBUFFERSIZE_PRM_SET: FwOpcodeType = 0x02;
    /// `ZLIBBUFFERSIZE_PRM_SAVE`.
    pub const OPCODE_ZLIBBUFFERSIZE_PRM_SAVE: FwOpcodeType = 0x03;

    // -- Parameters ---------------------------------------------------------

    /// `CompressionLevel: I8 default 6` — the zlib level (0–9, −1 = 6).
    pub const PARAMID_COMPRESSION_LEVEL: FwPrmIdType = 0x00;
    /// `ZLibBufferSize: FwSizeType default 269000` — the size requested on
    /// `bufferZLibGet`.
    pub const PARAMID_ZLIB_BUFFER_SIZE: FwPrmIdType = 0x01;
    /// FPP default of `CompressionLevel`.
    pub const COMPRESSION_LEVEL_DEFAULT: i8 = 6;
    /// FPP default of `ZLibBufferSize`.
    pub const ZLIB_BUFFER_SIZE_DEFAULT: FwSizeType = 269_000;

    // -- Events (FPP-relative ids, declaration order) ----------------------

    /// `ZLibCompressionBadBuffer(size)` — WARNING_LO, `throttle 5`.
    pub const EVENTID_ZLIB_COMPRESSION_BAD_BUFFER: FwEventIdType = 0x00;
    /// `ZLibAllocBadBuffer(size)` — WARNING_LO, `throttle 5`.
    pub const EVENTID_ZLIB_ALLOC_BAD_BUFFER: FwEventIdType = 0x01;
    /// `ZLibInitError(err: I32, msg: string)` — WARNING_LO, `throttle 5`.
    pub const EVENTID_ZLIB_INIT_ERROR: FwEventIdType = 0x02;
    /// `ZLibDeflateError(err: I32, msg: string)` — WARNING_LO, `throttle 5`.
    pub const EVENTID_ZLIB_DEFLATE_ERROR: FwEventIdType = 0x03;
    /// `BufferTooBigForZLib(in_buffer_size, comp_buffer_size, max_size)` —
    /// WARNING_LO, `throttle 5`.
    pub const EVENTID_BUFFER_TOO_BIG_FOR_ZLIB: FwEventIdType = 0x04;
    /// `ZLibNoCompression(buffer_size, min_compression)` — DIAGNOSTIC.
    pub const EVENTID_ZLIB_NO_COMPRESSION: FwEventIdType = 0x05;
    /// `ZLibCompression(buffer_size, compressed_size)` — DIAGNOSTIC.
    pub const EVENTID_ZLIB_COMPRESSION: FwEventIdType = 0x06;
    /// `ZLibMemoryUsage(memory_used, total_memory)` — DIAGNOSTIC.
    pub const EVENTID_ZLIB_MEMORY_USAGE: FwEventIdType = 0x07;
    /// The `throttle 5` limit shared by events 0..=4.
    pub const EVENT_THROTTLE: u32 = 5;

    /// Construct (topology phase 1); allocates the encoder state. Follow
    /// with `base.set_id_base`, wiring, [`reg_commands`](Self::reg_commands)
    /// and [`load_parameters`](Self::load_parameters).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            prm: PrmGlue::new(),
            buffer_compression_get: OutputPort::new(),
            buffer_compression_return: OutputPort::new(),
            buffer_zlib_get: OutputPort::new(),
            buffer_zlib_return: OutputPort::new(),
            compression_bad_buffer_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            alloc_bad_buffer_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            init_error_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            deflate_error_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            buffer_too_big_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            params: Mutex::new(DpZLibCompressorParams::default()),
            compressor: Mutex::new(Box::default()),
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
                Self::OPCODE_COMPRESSIONLEVEL_PRM_SET,
                Self::OPCODE_COMPRESSIONLEVEL_PRM_SAVE,
                Self::OPCODE_ZLIBBUFFERSIZE_PRM_SET,
                Self::OPCODE_ZLIBBUFFERSIZE_PRM_SAVE,
            ],
        );
    }

    /// The autocoded `loadParameters()` (see
    /// [`DpCompressProc::load_parameters`](crate::dp_compress_proc::DpCompressProc::load_parameters)
    /// for the rules). C++ parity: `compressChunk` `FW_ASSERT`s on
    /// `FW_PARAM_OK` until this has run.
    pub fn load_parameters(&self) {
        let id_base = self.id_base();
        let mut buf = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(id_base, Self::PARAMID_COMPRESSION_LEVEL, &mut buf);
        let mut level = 0i8;
        let level_ok = valid == ParamValid::Valid && buf.deserialize_i8_be(&mut level).is_ok();

        let mut buf = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(id_base, Self::PARAMID_ZLIB_BUFFER_SIZE, &mut buf);
        let mut size: FwSizeType = 0;
        let size_ok = valid == ParamValid::Valid && buf.deserialize_u64_be(&mut size).is_ok();

        let mut params = self.params.lock().unwrap();
        if level_ok {
            params.compression_level = level;
            params.compression_level_valid = ParamValid::Valid;
        } else {
            params.compression_level = Self::COMPRESSION_LEVEL_DEFAULT;
            params.compression_level_valid = ParamValid::Default;
        }
        if size_ok {
            params.zlib_buffer_size = size;
            params.zlib_buffer_size_valid = ParamValid::Valid;
        } else {
            params.zlib_buffer_size = Self::ZLIB_BUFFER_SIZE_DEFAULT;
            params.zlib_buffer_size_valid = ParamValid::Default;
        }
    }

    /// `paramGet_CompressionLevel`.
    #[must_use]
    pub fn param_get_compression_level(&self) -> (i8, ParamValid) {
        let params = self.params.lock().unwrap();
        (params.compression_level, params.compression_level_valid)
    }

    /// `paramGet_ZLibBufferSize`.
    #[must_use]
    pub fn param_get_zlib_buffer_size(&self) -> (FwSizeType, ParamValid) {
        let params = self.params.lock().unwrap();
        (params.zlib_buffer_size, params.zlib_buffer_size_valid)
    }

    // -- Input-port factories ----------------------------------------------

    /// `compressChunk` — SYNC input, wired from
    /// `DpCompressProc.compressChunk`.
    pub fn compress_chunk(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn CompressChunkPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `cmdIn` — SYNC `Fw.Cmd` input (the parameter commands).
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- compressChunk --------------------------------------------------------

    /// C++ `cleanupContext`: return whichever buffers were obtained.
    fn cleanup_context(&self, ctx: ZLibCtx) {
        if ctx.compression_buffer.size() != 0 {
            let port = self.buffer_compression_return.get();
            port.target.invoke(port.port_num, ctx.compression_buffer);
        }
        if ctx.zlib_alloc_buffer.size() != 0 {
            let port = self.buffer_zlib_return.get();
            port.target.invoke(port.port_num, ctx.zlib_alloc_buffer);
        }
    }

    /// `compressChunk_handler`.
    fn compress_chunk_handler(
        &self,
        _port_num: FwIndexType,
        buffer: &mut ChunkBuffer<'_>,
        min_compression: FwSizeType,
        write_offset: FwSizeType,
    ) -> CompressionAlgorithm {
        let mut ctx = ZLibCtx {
            compression_buffer: Buffer::empty(),
            zlib_alloc_buffer: Buffer::empty(),
        };

        let port = self.buffer_compression_get.get();
        ctx.compression_buffer = port.target.invoke(port.port_num, min_compression);
        if ctx.compression_buffer.size() == 0 {
            self.log_compression_bad_buffer(min_compression);
            self.cleanup_context(ctx);
            return CompressionAlgorithm::Uncompressed;
        }

        let (zlib_alloc_size, valid) = self.param_get_zlib_buffer_size();
        fw_assert!(param_ok(valid), valid.as_repr() as i32);
        let port = self.buffer_zlib_get.get();
        ctx.zlib_alloc_buffer = port.target.invoke(port.port_num, zlib_alloc_size);
        if ctx.zlib_alloc_buffer.size() == 0 {
            self.log_alloc_bad_buffer(zlib_alloc_size);
            self.cleanup_context(ctx);
            return CompressionAlgorithm::Uncompressed;
        }

        let alg = self.zlib_compression_helper(&mut ctx, buffer.data());

        if alg != CompressionAlgorithm::Uncompressed {
            // The compressed stream fits in min_compression bytes: copy it
            // into the chunk after the reserved write_offset bytes. The
            // caller guarantees the room; double-checked as in C++.
            let offset = write_offset as usize;
            let comp_size = ctx.compression_buffer.size();
            fw_assert!(
                buffer.size() >= offset && buffer.size() - offset >= comp_size,
                buffer.size() as i32,
                offset as i32,
                comp_size as i32
            );
            buffer.data_mut()[offset..offset + comp_size]
                .copy_from_slice(ctx.compression_buffer.data());
            buffer.set_size(comp_size + offset);
        }

        self.cleanup_context(ctx);
        alg
    }

    /// C++ `zlibCompressionHelper`: `deflateInit`, the size check, one
    /// `deflate(Z_FINISH)` into the scratch buffer.
    fn zlib_compression_helper(&self, ctx: &mut ZLibCtx, input: &[u8]) -> CompressionAlgorithm {
        let (level, valid) = self.param_get_compression_level();
        fw_assert!(param_ok(valid), valid.as_repr() as i32);

        // deflateInit: the level is checked before any allocation, then
        // the state must fit the arena (see the module notes).
        let level = if level == Z_DEFAULT_COMPRESSION {
            6
        } else {
            level
        };
        if !(0..=9).contains(&level) {
            self.log_init_error(Z_STREAM_ERROR, "");
            return CompressionAlgorithm::Uncompressed;
        }
        let arena_size = ctx.zlib_alloc_buffer.size() as FwSizeType;
        if arena_size < ZLIB_DEFLATE_MEMORY {
            self.log_init_error(Z_MEM_ERROR, ZLIB_MEM_ERROR_MSG);
            return CompressionAlgorithm::Uncompressed;
        }

        let in_size = input.len() as FwSizeType;
        let comp_buffer_size = ctx.compression_buffer.size() as FwSizeType;
        if in_size > ZLIB_UINT_MAX || comp_buffer_size > ZLIB_UINT_MAX {
            self.log_buffer_too_big_for_zlib(in_size, comp_buffer_size, ZLIB_UINT_MAX);
            return CompressionAlgorithm::Uncompressed;
        }

        let (status, total_out) = {
            let mut compressor = self.compressor.lock().unwrap();
            compressor.reset();
            compressor.set_format_and_level(DataFormat::Zlib, level as u8);
            let (status, _consumed, total_out) = compress(
                &mut compressor,
                input,
                ctx.compression_buffer.data_mut(),
                TDEFLFlush::Finish,
            );
            (status, total_out)
        };

        if status == TDEFLStatus::Done && total_out <= ctx.compression_buffer.size() {
            ctx.compression_buffer.set_size(total_out);
            self.log_compression(in_size, total_out as FwSizeType);
            self.log_memory_usage(ZLIB_DEFLATE_MEMORY, arena_size);
            CompressionAlgorithm::ZlibDeflate
        } else {
            // `Okay` is zlib's Z_OK: the stream did not fit the scratch
            // buffer, which is not an error.
            let err = match status {
                TDEFLStatus::Okay | TDEFLStatus::Done => None,
                TDEFLStatus::BadParam => Some(Z_STREAM_ERROR),
                TDEFLStatus::PutBufFailed => Some(Z_BUF_ERROR),
            };
            if let Some(err) = err {
                self.log_deflate_error(err, "");
            }
            self.log_no_compression(in_size, comp_buffer_size);
            CompressionAlgorithm::Uncompressed
        }
    }

    // -- Parameter commands --------------------------------------------------

    fn compression_level_prm_set(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut val = 0i8;
        if !args.deserialize_i8_be(&mut val).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        {
            let mut params = self.params.lock().unwrap();
            params.compression_level = val;
            params.compression_level_valid = ParamValid::Valid;
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn zlib_buffer_size_prm_set(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut val: FwSizeType = 0;
        if !args.deserialize_u64_be(&mut val).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        {
            let mut params = self.params.lock().unwrap();
            params.zlib_buffer_size = val;
            params.zlib_buffer_size_valid = ParamValid::Valid;
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
            if id == Self::PARAMID_COMPRESSION_LEVEL {
                buf.serialize_i8_be(params.compression_level)
            } else {
                buf.serialize_u64_be(params.zlib_buffer_size)
            }
        };
        fw_assert!(status.is_ok(), status as i32);
        self.prm.set_param(self.id_base(), id, &mut buf);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    // -- Events ---------------------------------------------------------------

    fn log_compression_bad_buffer(&self, size: FwSizeType) {
        if !self.compression_bad_buffer_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_ZLIB_COMPRESSION_BAD_BUFFER,
            LogSeverity::WarningLo,
            &format!("Unable to allocate a buffer of size {size} for compression"),
            |buf| buf.serialize_u64_be(size),
        );
    }

    fn log_alloc_bad_buffer(&self, size: FwSizeType) {
        if !self.alloc_bad_buffer_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_ZLIB_ALLOC_BAD_BUFFER,
            LogSeverity::WarningLo,
            &format!("Unable to allocate a buffer of size {size} for ZLib alloc"),
            |buf| buf.serialize_u64_be(size),
        );
    }

    fn log_init_error(&self, err: i32, msg: &str) {
        if !self.init_error_throttle.ok_to_emit() {
            return;
        }
        let msg = FwDefaultString::from(msg);
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_ZLIB_INIT_ERROR,
            LogSeverity::WarningLo,
            &format!("ZLib Error during initialization {err} {msg}"),
            |buf| {
                fw_try!(buf.serialize_i32_be(err));
                msg.serialize_to_truncated(buf, FW_LOG_STRING_MAX_SIZE, Endianness::Big)
            },
        );
    }

    fn log_deflate_error(&self, err: i32, msg: &str) {
        if !self.deflate_error_throttle.ok_to_emit() {
            return;
        }
        let msg = FwDefaultString::from(msg);
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_ZLIB_DEFLATE_ERROR,
            LogSeverity::WarningLo,
            &format!("ZLib Error during compression {err} {msg}"),
            |buf| {
                fw_try!(buf.serialize_i32_be(err));
                msg.serialize_to_truncated(buf, FW_LOG_STRING_MAX_SIZE, Endianness::Big)
            },
        );
    }

    fn log_buffer_too_big_for_zlib(
        &self,
        in_buffer_size: FwSizeType,
        comp_buffer_size: FwSizeType,
        max_size: FwSizeType,
    ) {
        if !self.buffer_too_big_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BUFFER_TOO_BIG_FOR_ZLIB,
            LogSeverity::WarningLo,
            &format!(
                "Compression buffer too large to compress with zlib. {in_buffer_size} or \
                 {comp_buffer_size} > {max_size}"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(in_buffer_size));
                fw_try!(buf.serialize_u64_be(comp_buffer_size));
                buf.serialize_u64_be(max_size)
            },
        );
    }

    fn log_no_compression(&self, buffer_size: FwSizeType, min_compression: FwSizeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_ZLIB_NO_COMPRESSION,
            LogSeverity::Diagnostic,
            &format!(
                "Unable to compress buffer of size {buffer_size} to at most {min_compression} \
                 bytes with zlib"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(buffer_size));
                buf.serialize_u64_be(min_compression)
            },
        );
    }

    fn log_compression(&self, buffer_size: FwSizeType, compressed_size: FwSizeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_ZLIB_COMPRESSION,
            LogSeverity::Diagnostic,
            &format!("Compressed buffer of size {buffer_size} to {compressed_size} bytes"),
            |buf| {
                fw_try!(buf.serialize_u64_be(buffer_size));
                buf.serialize_u64_be(compressed_size)
            },
        );
    }

    fn log_memory_usage(&self, memory_used: FwSizeType, total_memory: FwSizeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_ZLIB_MEMORY_USAGE,
            LogSeverity::Diagnostic,
            &format!("Compression used {memory_used} of {total_memory} available bytes"),
            |buf| {
                fw_try!(buf.serialize_u64_be(memory_used));
                buf.serialize_u64_be(total_memory)
            },
        );
    }

    /// The `log_WARNING_LO_*_ThrottleClear` calls for the five throttled
    /// events (upstream has no command that calls them).
    pub fn clear_event_throttles(&self) {
        self.compression_bad_buffer_throttle.clear();
        self.alloc_bad_buffer_throttle.clear();
        self.init_error_throttle.clear();
        self.deflate_error_throttle.clear();
        self.buffer_too_big_throttle.clear();
    }
}

impl CompressChunkPort for DpZLibCompressor {
    fn invoke(
        &self,
        port_num: FwIndexType,
        buffer: &mut ChunkBuffer<'_>,
        min_compression: FwSizeType,
        write_offset: FwSizeType,
    ) -> CompressionAlgorithm {
        self.compress_chunk_handler(port_num, buffer, min_compression, write_offset)
    }
}

impl CmdPort for DpZLibCompressor {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_COMPRESSIONLEVEL_PRM_SET => {
                self.compression_level_prm_set(op_code, cmd_seq, args)
            }
            Self::OPCODE_COMPRESSIONLEVEL_PRM_SAVE => {
                self.prm_save(op_code, cmd_seq, args, Self::PARAMID_COMPRESSION_LEVEL)
            }
            Self::OPCODE_ZLIBBUFFERSIZE_PRM_SET => {
                self.zlib_buffer_size_prm_set(op_code, cmd_seq, args)
            }
            Self::OPCODE_ZLIBBUFFERSIZE_PRM_SAVE => {
                self.prm_save(op_code, cmd_seq, args, Self::PARAMID_ZLIB_BUFFER_SIZE)
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of `Svc/DpZLibCompressor/test/ut` (`DpZLibCompressorTestMain.cpp`,
    //! `DpZLibCompressorTester.cpp`): every compressed chunk is inflated
    //! back with `miniz_oxide` and compared with the input.
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, PrmSetPort};
    use fprime_fw::{LogBuffer, Time};
    use miniz_oxide::inflate::decompress_to_vec_zlib;

    const ID_BASE: FwIdType = 0x0410_1000;
    /// Upstream's `header_size` (`sizeof(FwDpIdType) + CompressionMetadata::SERIALIZED_SIZE`).
    const HEADER_SIZE: FwSizeType = 5;

    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<EventRecord>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        prm_sets: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
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

    impl fprime_comp::PrmGetPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwPrmIdType,
            _val: &mut ParamBuffer,
        ) -> ParamValid {
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

    /// One `bufferXGet`/`bufferXReturn` pair (the tester's
    /// `from_buffer*_handler`s): at most one buffer out at a time.
    #[derive(Default)]
    struct Allocator {
        fail: Mutex<bool>,
        outstanding: Mutex<bool>,
        requests: Mutex<Vec<FwSizeType>>,
        returns: Mutex<usize>,
    }

    impl BufferGetPort for Allocator {
        fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
            self.requests.lock().unwrap().push(size);
            let mut outstanding = self.outstanding.lock().unwrap();
            if *self.fail.lock().unwrap() || *outstanding {
                return Buffer::empty();
            }
            *outstanding = true;
            Buffer::from_storage(vec![0u8; size as usize].into_boxed_slice(), 0x1234)
        }
    }

    impl BufferSendPort for Allocator {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            assert_eq!(buffer.context(), 0x1234);
            let mut outstanding = self.outstanding.lock().unwrap();
            assert!(*outstanding);
            *outstanding = false;
            *self.returns.lock().unwrap() += 1;
        }
    }

    struct Fixture {
        comp: Arc<DpZLibCompressor>,
        ground: Arc<Ground>,
        compression: Arc<Allocator>,
        zlib: Arc<Allocator>,
    }

    fn fixture() -> Fixture {
        let comp = DpZLibCompressor::new("dpZLibCompressor");
        comp.base.set_id_base(ID_BASE);
        let ground = Arc::new(Ground::default());
        let compression = Arc::new(Allocator::default());
        let zlib = Arc::new(Allocator::default());
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.prm.prm_get_out.connect(ground.clone(), 0);
        comp.prm.prm_set_out.connect(ground.clone(), 0);
        comp.buffer_compression_get.connect(compression.clone(), 0);
        comp.buffer_compression_return
            .connect(compression.clone(), 0);
        comp.buffer_zlib_get.connect(zlib.clone(), 0);
        comp.buffer_zlib_return.connect(zlib.clone(), 0);
        comp.load_parameters();
        Fixture {
            comp,
            ground,
            compression,
            zlib,
        }
    }

    fn send_cmd(f: &Fixture, opcode: FwOpcodeType, args: &[u8]) -> CmdResponse {
        let mut buf = CmdArgBuffer::new();
        let status = buf.serialize_bytes(args, fprime_fw::LengthMode::OmitLength, Endianness::Big);
        assert!(status.is_ok());
        f.ground.responses.lock().unwrap().clear();
        CmdPort::invoke(&*f.comp, 0, ID_BASE + opcode, 3, &mut buf);
        let responses = f.ground.responses.lock().unwrap();
        assert_eq!(responses.len(), 1);
        responses[0].2
    }

    fn set_params(f: &Fixture, zlib_alloc_size: FwSizeType, level: i8) {
        assert_eq!(
            send_cmd(
                f,
                DpZLibCompressor::OPCODE_ZLIBBUFFERSIZE_PRM_SET,
                &zlib_alloc_size.to_be_bytes()
            ),
            CmdResponse::Ok
        );
        assert_eq!(
            send_cmd(
                f,
                DpZLibCompressor::OPCODE_COMPRESSIONLEVEL_PRM_SET,
                &level.to_be_bytes()
            ),
            CmdResponse::Ok
        );
    }

    fn u64s(values: &[u64]) -> Vec<u8> {
        values.iter().flat_map(|v| v.to_be_bytes()).collect()
    }

    /// `DpZLibCompressorTester::compress`.
    fn compress(
        f: &Fixture,
        data: &[u8],
        exp_alg: CompressionAlgorithm,
        min_compression: FwSizeType,
        write_offset: FwSizeType,
        zlib_alloc_size: FwSizeType,
        level: i8,
    ) -> Vec<u8> {
        set_params(f, zlib_alloc_size, level);
        let mut storage = data.to_vec();
        let mut chunk = ChunkBuffer::new(&mut storage);
        let alg = CompressChunkPort::invoke(&*f.comp, 0, &mut chunk, min_compression, write_offset);
        assert_eq!(alg, exp_alg);
        let out_size = chunk.size();
        let off = write_offset as usize;
        if alg != CompressionAlgorithm::Uncompressed {
            assert!((out_size as FwSizeType) < min_compression + write_offset);
            let inflated =
                decompress_to_vec_zlib(&storage[off..out_size]).expect("valid zlib stream");
            assert_eq!(inflated, data);
            assert_eq!(
                f.ground
                    .events_with(DpZLibCompressor::EVENTID_ZLIB_MEMORY_USAGE)
                    .len(),
                1
            );
            let events = f
                .ground
                .events_with(DpZLibCompressor::EVENTID_ZLIB_COMPRESSION);
            assert_eq!(events.len(), 1);
            assert_eq!(
                events[0],
                u64s(&[data.len() as u64, (out_size - off) as u64])
            );
        } else {
            assert_eq!(out_size, data.len());
            assert_eq!(storage, data);
            if *f.compression.fail.lock().unwrap() {
                let events = f
                    .ground
                    .events_with(DpZLibCompressor::EVENTID_ZLIB_COMPRESSION_BAD_BUFFER);
                assert_eq!(events, vec![u64s(&[min_compression])]);
            }
            if *f.zlib.fail.lock().unwrap() {
                let events = f
                    .ground
                    .events_with(DpZLibCompressor::EVENTID_ZLIB_ALLOC_BAD_BUFFER);
                assert_eq!(events, vec![u64s(&[zlib_alloc_size])]);
            }
            if !*f.compression.fail.lock().unwrap() && !*f.zlib.fail.lock().unwrap() {
                let events = f
                    .ground
                    .events_with(DpZLibCompressor::EVENTID_ZLIB_NO_COMPRESSION);
                assert_eq!(events, vec![u64s(&[data.len() as u64, min_compression])]);
            }
        }
        // Every buffer obtained went back.
        assert!(!*f.compression.outstanding.lock().unwrap());
        assert!(!*f.zlib.outstanding.lock().unwrap());
        storage[..out_size].to_vec()
    }

    // -- Upstream TEST cases ---------------------------------------------------

    #[test]
    fn nominal_zeros() {
        let f = fixture();
        let zeros = vec![0u8; 32 * 1024];
        compress(
            &f,
            &zeros,
            CompressionAlgorithm::ZlibDeflate,
            zeros.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
        // The scratch buffer is min_compression bytes and the arena
        // ZLibBufferSize bytes.
        assert_eq!(*f.compression.requests.lock().unwrap(), vec![32 * 1024]);
        assert_eq!(*f.zlib.requests.lock().unwrap(), vec![1024 * 1024]);
        let usage = f
            .ground
            .events_with(DpZLibCompressor::EVENTID_ZLIB_MEMORY_USAGE);
        assert_eq!(usage[0], u64s(&[ZLIB_DEFLATE_MEMORY, 1024 * 1024]));
    }

    #[test]
    fn nominal_zeros_level_9() {
        let f = fixture();
        let zeros = vec![0u8; 32 * 1024];
        compress(
            &f,
            &zeros,
            CompressionAlgorithm::ZlibDeflate,
            zeros.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            9,
        );
    }

    #[test]
    fn nominal_regular_data() {
        let f = fixture();
        let data: Vec<u8> = (0..32 * 1024).map(|i| i as u8).collect();
        compress(
            &f,
            &data,
            CompressionAlgorithm::ZlibDeflate,
            data.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
    }

    #[test]
    fn uncompressed_random_data() {
        let f = fixture();
        let mut state: u32 = 42;
        let random: Vec<u8> = (0..32 * 1024)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                (state >> 24) as u8
            })
            .collect();
        compress(
            &f,
            &random,
            CompressionAlgorithm::Uncompressed,
            random.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
    }

    #[test]
    fn uncompressed_above_min_size() {
        // Upstream: 32 KiB of zeros deflates to 52 bytes with libz, so a
        // 51-byte limit fails. miniz_oxide's stream length differs; find it
        // and ask for one byte less.
        let f = fixture();
        let zeros = vec![0u8; 32 * 1024];
        let out = compress(
            &f,
            &zeros,
            CompressionAlgorithm::ZlibDeflate,
            zeros.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
        let compressed = (out.len() as u64) - HEADER_SIZE;
        let f = fixture();
        compress(
            &f,
            &zeros,
            CompressionAlgorithm::Uncompressed,
            compressed - 1,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
        // A limit of exactly the stream length fits.
        let f = fixture();
        compress(
            &f,
            &zeros,
            CompressionAlgorithm::ZlibDeflate,
            compressed + 1,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
    }

    #[test]
    fn off_nominal_bad_compression_alloc() {
        let f = fixture();
        *f.compression.fail.lock().unwrap() = true;
        let zeros = vec![0u8; 32 * 1024];
        compress(
            &f,
            &zeros,
            CompressionAlgorithm::Uncompressed,
            zeros.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
        // Gave up before asking for the arena.
        assert!(f.zlib.requests.lock().unwrap().is_empty());
    }

    #[test]
    fn off_nominal_bad_zlib_alloc() {
        let f = fixture();
        *f.zlib.fail.lock().unwrap() = true;
        let zeros = vec![0u8; 32 * 1024];
        compress(
            &f,
            &zeros,
            CompressionAlgorithm::Uncompressed,
            zeros.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            6,
        );
        // The compression buffer was returned.
        assert_eq!(*f.compression.returns.lock().unwrap(), 1);
    }

    #[test]
    fn off_nominal_insufficient_zlib_mem() {
        let f = fixture();
        set_params(&f, ZLIB_DEFLATE_MEMORY - 1, 6);
        let mut zeros = vec![0u8; 32 * 1024];
        let mut chunk = ChunkBuffer::new(&mut zeros);
        let alg = CompressChunkPort::invoke(&*f.comp, 0, &mut chunk, 32 * 1024, HEADER_SIZE);
        assert_eq!(alg, CompressionAlgorithm::Uncompressed);
        let events = f
            .ground
            .events_with(DpZLibCompressor::EVENTID_ZLIB_INIT_ERROR);
        assert_eq!(events.len(), 1);
        let mut expected = Z_MEM_ERROR.to_be_bytes().to_vec();
        expected.extend_from_slice(&(ZLIB_MEM_ERROR_MSG.len() as u16).to_be_bytes());
        expected.extend_from_slice(ZLIB_MEM_ERROR_MSG.as_bytes());
        assert_eq!(events[0], expected);
        assert_eq!(*f.compression.returns.lock().unwrap(), 1);
        assert_eq!(*f.zlib.returns.lock().unwrap(), 1);

        // Exactly the deflateInit requirement is enough.
        let f = fixture();
        let zeros = vec![0u8; 32 * 1024];
        compress(
            &f,
            &zeros,
            CompressionAlgorithm::ZlibDeflate,
            zeros.len() as u64,
            HEADER_SIZE,
            ZLIB_DEFLATE_MEMORY,
            6,
        );
    }

    // -- Further branches -------------------------------------------------------

    #[test]
    fn compression_levels_follow_deflate_init() {
        let data: Vec<u8> = (0..4096).map(|i| (i % 7) as u8).collect();
        // -1 is zlib's default level and every level 1..=9 compresses.
        for level in [-1i8, 1, 2, 3, 4, 5, 6, 7, 8, 9] {
            let f = fixture();
            compress(
                &f,
                &data,
                CompressionAlgorithm::ZlibDeflate,
                data.len() as u64,
                HEADER_SIZE,
                1024 * 1024,
                level,
            );
        }
        // Level 0 stores the data: the stream is larger than the input.
        let f = fixture();
        compress(
            &f,
            &data,
            CompressionAlgorithm::Uncompressed,
            data.len() as u64,
            HEADER_SIZE,
            1024 * 1024,
            0,
        );
        // Out-of-range levels fail deflateInit with Z_STREAM_ERROR.
        for level in [10i8, -2, i8::MAX, i8::MIN] {
            let f = fixture();
            set_params(&f, 1024 * 1024, level);
            let mut storage = data.clone();
            let mut chunk = ChunkBuffer::new(&mut storage);
            let alg = CompressChunkPort::invoke(&*f.comp, 0, &mut chunk, 4096, HEADER_SIZE);
            assert_eq!(alg, CompressionAlgorithm::Uncompressed);
            let events = f
                .ground
                .events_with(DpZLibCompressor::EVENTID_ZLIB_INIT_ERROR);
            let mut expected = Z_STREAM_ERROR.to_be_bytes().to_vec();
            expected.extend_from_slice(&[0, 0]);
            assert_eq!(events, vec![expected]);
            assert!(
                f.ground
                    .events_with(DpZLibCompressor::EVENTID_ZLIB_NO_COMPRESSION)
                    .is_empty()
            );
            assert_eq!(storage, data);
        }
    }

    #[test]
    fn the_encoder_is_reused_across_calls() {
        // The one preallocated encoder must reset cleanly between chunks of
        // different content and levels.
        let f = fixture();
        let a: Vec<u8> = (0..8192).map(|i| (i / 3) as u8).collect();
        let b = vec![0x5Au8; 1000];
        for (data, level) in [(&a, 6i8), (&b, 1), (&a, 9), (&b, 6)] {
            f.ground.events.lock().unwrap().clear();
            compress(
                &f,
                data,
                CompressionAlgorithm::ZlibDeflate,
                data.len() as u64,
                2 * HEADER_SIZE,
                1024 * 1024,
                level,
            );
        }
    }

    #[test]
    fn write_offset_bytes_are_left_alone() {
        let f = fixture();
        set_params(&f, 1024 * 1024, 6);
        let mut storage = vec![0xEEu8; 4096];
        storage[..14].copy_from_slice(b"reserved-bytes");
        let mut chunk = ChunkBuffer::new(&mut storage);
        let alg = CompressChunkPort::invoke(&*f.comp, 0, &mut chunk, 4096 - 14, 14);
        assert_eq!(alg, CompressionAlgorithm::ZlibDeflate);
        let size = chunk.size();
        assert_eq!(&storage[..14], b"reserved-bytes");
        let mut expected = vec![0xEEu8; 4096];
        expected[..14].copy_from_slice(b"reserved-bytes");
        assert_eq!(
            decompress_to_vec_zlib(&storage[14..size]).unwrap(),
            expected
        );
    }

    #[test]
    fn warning_events_are_throttled_at_five() {
        let f = fixture();
        *f.compression.fail.lock().unwrap() = true;
        set_params(&f, 1024 * 1024, 6);
        for _ in 0..7 {
            let mut storage = vec![0u8; 256];
            let mut chunk = ChunkBuffer::new(&mut storage);
            let _ = CompressChunkPort::invoke(&*f.comp, 0, &mut chunk, 200, 7);
        }
        assert_eq!(
            f.ground
                .events_with(DpZLibCompressor::EVENTID_ZLIB_COMPRESSION_BAD_BUFFER)
                .len(),
            5
        );
        f.comp.clear_event_throttles();
        let mut storage = vec![0u8; 256];
        let mut chunk = ChunkBuffer::new(&mut storage);
        let _ = CompressChunkPort::invoke(&*f.comp, 0, &mut chunk, 200, 7);
        assert_eq!(
            f.ground
                .events_with(DpZLibCompressor::EVENTID_ZLIB_COMPRESSION_BAD_BUFFER)
                .len(),
            6
        );
    }

    #[test]
    #[should_panic]
    fn compress_chunk_before_load_parameters_asserts() {
        let comp = DpZLibCompressor::new("dpZLibCompressor");
        let alloc = Arc::new(Allocator::default());
        comp.buffer_compression_get.connect(alloc.clone(), 0);
        comp.buffer_compression_return.connect(alloc.clone(), 0);
        let mut storage = vec![0u8; 256];
        let mut chunk = ChunkBuffer::new(&mut storage);
        let _ = CompressChunkPort::invoke(&*comp, 0, &mut chunk, 200, 7);
    }

    #[test]
    fn parameters_and_commands() {
        let f = fixture();
        f.comp.reg_commands();
        assert_eq!(
            *f.ground.regs.lock().unwrap(),
            vec![ID_BASE, ID_BASE + 1, ID_BASE + 2, ID_BASE + 3]
        );
        assert_eq!(
            f.comp.param_get_compression_level(),
            (6, ParamValid::Default)
        );
        assert_eq!(
            f.comp.param_get_zlib_buffer_size(),
            (269_000, ParamValid::Default)
        );
        assert_eq!(
            send_cmd(
                &f,
                DpZLibCompressor::OPCODE_COMPRESSIONLEVEL_PRM_SET,
                &[0xFF]
            ),
            CmdResponse::Ok
        );
        assert_eq!(
            f.comp.param_get_compression_level(),
            (-1, ParamValid::Valid)
        );
        assert_eq!(
            send_cmd(
                &f,
                DpZLibCompressor::OPCODE_ZLIBBUFFERSIZE_PRM_SET,
                &[1, 2, 3]
            ),
            CmdResponse::FormatError
        );
        assert_eq!(
            send_cmd(
                &f,
                DpZLibCompressor::OPCODE_ZLIBBUFFERSIZE_PRM_SET,
                &300_000u64.to_be_bytes()
            ),
            CmdResponse::Ok
        );
        assert_eq!(
            send_cmd(&f, DpZLibCompressor::OPCODE_COMPRESSIONLEVEL_PRM_SAVE, &[]),
            CmdResponse::Ok
        );
        assert_eq!(
            send_cmd(&f, DpZLibCompressor::OPCODE_ZLIBBUFFERSIZE_PRM_SAVE, &[]),
            CmdResponse::Ok
        );
        assert_eq!(
            *f.ground.prm_sets.lock().unwrap(),
            vec![
                (ID_BASE, vec![0xFF]),
                (ID_BASE + 1, 300_000u64.to_be_bytes().to_vec()),
            ]
        );
        assert_eq!(send_cmd(&f, 4, &[]), CmdResponse::InvalidOpcode);
        assert_eq!(ZLibData::Binary as i32, 0);
        assert_eq!(ZLibData::Text as i32, 1);
    }
}
