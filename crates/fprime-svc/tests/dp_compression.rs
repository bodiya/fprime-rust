//! End-to-end data-product compression (ROADMAP item 14): `DpWriter` →
//! `procBufferSendOut[0]` → `DpCompressProc` → `DpZLibCompressor` →
//! `BufferManager`, wired as upstream's `DpCompression` subtopology. The
//! `.fdp` file on disk is decoded the way the ground does it (fprime-gds
//! `common/dp/decoder.py`): check both CRCs, walk the `CompressionRecord`s,
//! inflate the `ZLIB_DEFLATE` payloads and concatenate — the result must be
//! the original record bytes.
//!
//! Only built with `--features zlib` (and `std`: `DpWriter` writes files).
#![cfg(all(feature = "zlib", feature = "std"))]

use fprime_comp::BufferSendPort;
use fprime_config::{FwDpIdType, FwSizeType, dp_compression};
use fprime_fw::dp::{DpContainer, ProcType};
use fprime_fw::{Buffer, Endianness, LengthMode, SerBuf, Time, TimeBase};
use fprime_svc::buffer_manager::{BufferBin, BufferManager};
use fprime_svc::dp_compress_proc::{COMPRESSION_HEADER_SIZE, CompressionAlgorithm, DpCompressProc};
use fprime_svc::dp_writer::DpWriter;
use fprime_svc::dp_zlib_compressor::DpZLibCompressor;
use miniz_oxide::inflate::decompress_to_vec_zlib;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

static COUNTER: AtomicU32 = AtomicU32::new(0);

fn temp_dir() -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "fprime_rust_dp_compression_{}_{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::create_dir_all(&path).unwrap();
    path
}

/// Where `DpWriter` returns the packet buffer.
#[derive(Default)]
struct Dealloc {
    returned: Mutex<Vec<usize>>,
}

impl BufferSendPort for Dealloc {
    fn invoke(&self, _port_num: fprime_config::FwIndexType, buffer: Buffer) {
        self.returned.lock().unwrap().push(buffer.size());
    }
}

struct Deployment {
    dp_writer: Arc<DpWriter>,
    compress_proc: Arc<DpCompressProc>,
    dir: PathBuf,
}

fn deployment() -> Deployment {
    let dir = temp_dir();
    let dp_writer = DpWriter::new("dpWriter");
    dp_writer.init(4);
    dp_writer.configure(dir.to_str().unwrap());
    let dealloc = Arc::new(Dealloc::default());
    dp_writer.dealloc_buffer_send_out.connect(dealloc, 0);

    // The DpCompression subtopology.
    let compress_proc = DpCompressProc::new("dpCompressProc");
    compress_proc.base.set_id_base(dp_compression::BASE_ID);
    let zlib = DpZLibCompressor::new("dpZLibCompressor");
    zlib.base.set_id_base(dp_compression::BASE_ID + 0x1000);
    let buffer_manager = BufferManager::new("dpZLibCompressorBufferManager");
    buffer_manager.setup(
        dp_compression::BUFFER_MANAGER_ID,
        &[
            BufferBin {
                buffer_size: dp_compression::COMPRESSION_BUFFER_STORE_SIZE,
                num_buffers: dp_compression::COMPRESSION_BUFFER_STORE_COUNT,
            },
            BufferBin {
                buffer_size: dp_compression::ALLOC_BUFFER_STORE_SIZE,
                num_buffers: dp_compression::ALLOC_BUFFER_STORE_COUNT,
            },
        ],
    );
    compress_proc
        .compress_chunk
        .connect_to(zlib.compress_chunk(0));
    zlib.buffer_compression_get
        .connect_to(buffer_manager.buffer_get_callee_in(0));
    zlib.buffer_compression_return
        .connect_to(buffer_manager.buffer_send_in(0));
    zlib.buffer_zlib_get
        .connect_to(buffer_manager.buffer_get_callee_in(0));
    zlib.buffer_zlib_return
        .connect_to(buffer_manager.buffer_send_in(0));

    // No parameter database in this test: a prmGetOut that answers INVALID
    // makes load_parameters install the FPP defaults.
    struct NoParams;
    impl fprime_comp::PrmGetPort for NoParams {
        fn invoke(
            &self,
            _port_num: fprime_config::FwIndexType,
            _id: fprime_config::FwPrmIdType,
            _val: &mut fprime_fw::ParamBuffer,
        ) -> fprime_fw::ParamValid {
            fprime_fw::ParamValid::Invalid
        }
    }
    let no_params = Arc::new(NoParams);
    compress_proc.prm.prm_get_out.connect(no_params.clone(), 0);
    zlib.prm.prm_get_out.connect(no_params, 0);
    compress_proc.load_parameters();
    zlib.load_parameters();

    // DataProducts.dpWriterProcOut[0] -> DpCompression.dpCompressProcIn
    dp_writer.proc_buffer_send_out[0].connect_to(compress_proc.proc_request(0));
    Deployment {
        dp_writer,
        compress_proc,
        dir,
    }
}

/// A packet holding `records` as the data, with the given proc-type mask,
/// sized exactly to the packet (as the generated `dpSend` leaves it).
fn packet(id: FwDpIdType, proc_types: u8, records: &[u8]) -> Buffer {
    let mut container = DpContainer::with_buffer(
        id,
        Buffer::allocate(DpContainer::MIN_PACKET_SIZE + records.len()),
    );
    container.set_priority(7);
    container.set_time_tag(Time::new(TimeBase::TbWorkstationTime, 0, 1000, 2000));
    container.set_proc_types(proc_types);
    container.data_region_mut().copy_from_slice(records);
    container.set_data_size(records.len() as FwSizeType);
    container.serialize_header();
    container.update_data_hash();
    container.take_buffer()
}

/// `count` SignalGen-style records: `[record id][U32 sample]`.
fn sample_records(count: u32) -> Vec<u8> {
    let mut out = Vec::new();
    for i in 0..count {
        out.extend_from_slice(&0x1001_1000u32.to_be_bytes());
        out.extend_from_slice(&(i / 16).to_be_bytes());
    }
    out
}

fn write_and_read(d: &Deployment, buffer: Buffer) -> Vec<u8> {
    let port = d.dp_writer.buffer_send_in(0);
    port.target.invoke(port.port_num, buffer);
    let _ = d
        .dp_writer
        .active
        .queued
        .dispatch_available_messages(d.dp_writer.as_ref());
    let mut files: Vec<_> = std::fs::read_dir(&d.dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(files.len(), 1, "{files:?}");
    let path = files.pop().unwrap();
    let bytes = std::fs::read(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    bytes
}

/// The ground's view: validate the file, return `(container, records)`
/// where `records` is the decompressed record stream.
fn decode(file: Vec<u8>, record_id: FwDpIdType) -> (DpContainer, Vec<u8>) {
    let mut container = DpContainer::new();
    container.set_buffer(Buffer::from_storage(file.into_boxed_slice(), 0));
    assert_eq!(container.check_header_hash().0, fprime_fw::Success::Success);
    assert!(container.deserialize_header().is_ok());
    assert_eq!(
        container.packet_size(),
        container.buffer().size() as FwSizeType
    );
    assert_eq!(container.check_data_hash().0, fprime_fw::Success::Success);

    let data = container.data().to_vec();
    let mut out = Vec::new();
    let mut de = fprime_fw::LinearBuffer::<65536>::new();
    assert!(
        de.serialize_bytes(&data, LengthMode::OmitLength, Endianness::Big)
            .is_ok()
    );
    while de.deserialize_size_left() > 0 {
        let mut id = 0u32;
        let mut size = 0u16;
        let mut algorithm = 0u8;
        assert!(de.deserialize_u32_be(&mut id).is_ok());
        assert_eq!(id, record_id);
        assert!(de.deserialize_u16_be(&mut size).is_ok());
        assert!(de.deserialize_u8_be(&mut algorithm).is_ok());
        let mut payload = vec![0u8; size as usize - 1];
        let mut received = payload.len();
        assert!(
            de.deserialize_bytes(
                &mut payload,
                &mut received,
                LengthMode::OmitLength,
                Endianness::Big
            )
            .is_ok()
        );
        match CompressionAlgorithm::try_from(algorithm).unwrap() {
            CompressionAlgorithm::Uncompressed => out.extend_from_slice(&payload),
            CompressionAlgorithm::ZlibDeflate => {
                out.extend_from_slice(&decompress_to_vec_zlib(&payload).unwrap())
            }
        }
    }
    (container, out)
}

#[test]
fn a_compressible_product_is_written_compressed_and_decodes_back() {
    let d = deployment();
    let records = sample_records(2000); // 16000 bytes, one 32 KiB chunk
    let file = write_and_read(
        &d,
        packet(0x1001_1000, ProcType::ZlibDeflate.as_repr(), &records),
    );
    let record_id = dp_compression::BASE_ID + DpCompressProc::RECORDID_COMPRESSION_RECORD;
    let (container, decoded) = decode(file, record_id);
    assert_eq!(decoded, records);
    // One ZLIB_DEFLATE record, much smaller than the original.
    assert!(container.data_size() < records.len() as FwSizeType / 10);
    assert_eq!(
        container.data()[6],
        CompressionAlgorithm::ZlibDeflate.as_repr()
    );
    // The header keeps everything else, including the proc-type bit.
    assert_eq!(container.id(), 0x1001_1000);
    assert_eq!(container.priority(), 7);
    assert_eq!(container.proc_types(), ProcType::ZlibDeflate.as_repr());
    assert_eq!(container.time_tag().get_seconds(), 1000);
    let _ = std::fs::remove_dir_all(&d.dir);
}

#[test]
fn mixed_chunks_produce_raw_and_deflate_records() {
    let d = deployment();
    // 1 KiB chunks: noise, then compressible records, then noise.
    assert!(
        d.compress_proc
            .param_get_chunk_size()
            .1
            .eq(&fprime_fw::ParamValid::Default)
    );
    let mut args = fprime_fw::CmdArgBuffer::new();
    assert!(args.serialize_u16_be(1024).is_ok());
    struct Responses;
    impl fprime_comp::CmdResponsePort for Responses {
        fn invoke(
            &self,
            _port_num: fprime_config::FwIndexType,
            _op_code: fprime_config::FwOpcodeType,
            _cmd_seq: u32,
            response: fprime_fw::CmdResponse,
        ) {
            assert_eq!(response, fprime_fw::CmdResponse::Ok);
        }
    }
    d.compress_proc
        .cmd
        .cmd_response_out
        .connect(Arc::new(Responses), 0);
    let port = d.compress_proc.cmd_in(0);
    port.target.invoke(
        port.port_num,
        dp_compression::BASE_ID + DpCompressProc::OPCODE_CHUNK_SIZE_PRM_SET,
        1,
        &mut args,
    );
    assert_eq!(d.compress_proc.param_get_chunk_size().0, 1024);

    let mut state = 0x1234_5678u32;
    let mut noise = |n: usize| -> Vec<u8> {
        (0..n)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                (state >> 24) as u8
            })
            .collect()
    };
    let mut records = noise(2048);
    records.extend(sample_records(512)); // 4096 bytes
    records.extend(noise(1500));
    let file = write_and_read(&d, packet(42, ProcType::ZlibDeflate.as_repr(), &records));
    let record_id = dp_compression::BASE_ID + DpCompressProc::RECORDID_COMPRESSION_RECORD;
    let (container, decoded) = decode(file, record_id);
    assert_eq!(decoded, records);
    assert!(container.data_size() < records.len() as FwSizeType);
    // First record: the leading 2 KiB of noise stays raw.
    let data = container.data();
    assert_eq!(data[6], CompressionAlgorithm::Uncompressed.as_repr());
    assert_eq!(u16::from_be_bytes([data[4], data[5]]) as usize, 2048 + 1);
    assert_eq!(
        data[COMPRESSION_HEADER_SIZE + 2048 + 6],
        CompressionAlgorithm::ZlibDeflate.as_repr()
    );
    let _ = std::fs::remove_dir_all(&d.dir);
}

#[test]
fn incompressible_or_unflagged_products_are_written_unchanged() {
    let d = deployment();
    let mut state = 99u32;
    let noise: Vec<u8> = (0..8000)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state >> 24) as u8
        })
        .collect();
    let original = packet(5, ProcType::ZlibDeflate.as_repr(), &noise);
    let expected = original.data().to_vec();
    assert_eq!(write_and_read(&d, original), expected);

    // Without the ZLIB_DEFLATE bit DpWriter never calls the processor.
    let records = sample_records(1000);
    let original = packet(6, ProcType::None.as_repr(), &records);
    let expected = original.data().to_vec();
    assert_eq!(write_and_read(&d, original), expected);
    let _ = std::fs::remove_dir_all(&d.dir);
}
