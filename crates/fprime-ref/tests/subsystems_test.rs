//! End-to-end integration tests for the subsystems wired into the Ref
//! deployment on top of the command/event/telemetry core: file services
//! (`fileManager`, `fileUplink`), the command sequencer, the parameter
//! database and the data-product chain.
//!
//! Every test drives the REAL port graph — bytes are injected on the
//! driver-facing side of `comStub` and travel
//! `frameAccumulator -> deframer -> fprimeRouter -> ...`, exactly as they
//! would from the GDS — and observes only what the deployment downlinks
//! (framed event packets) plus what it leaves on disk. Nothing calls a
//! component handler directly, and nothing waits on wall-clock time: rate
//! groups are ticked by hand and every wait is a bounded poll.

mod common;

use common::{
    DEADLINE, Harness, build_command_frame, build_file_frame, cmd_string_arg, wait_until,
};

use fprime_fw::file_packet::{DataPacket, EndPacket, FilePacket, StartPacket};
use fprime_fw::{ComBuffer, SerBuf, TimeBase};
use fprime_ref::signal_gen;
use fprime_ref::topology::{
    CMD_DISPATCHER_BASE_ID, CMD_SEQUENCER_BASE_ID, DP_CATALOG_BASE_ID, DP_WRITER_BASE_ID,
    FILE_MANAGER_BASE_ID, FILE_UPLINK_BASE_ID, PRM_DB_BASE_ID, SIGNAL_GEN_BASE_ID,
};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_svc::cmd_sequencer::CmdSequencer;
use fprime_svc::dp_catalog::DpCatalog;
use fprime_svc::dp_writer::DpWriter;
use fprime_svc::file_manager::{self, FileManager};
use fprime_svc::file_uplink::FileUplink;
use fprime_svc::prm_db::PrmDb;
use fprime_utils::Hash;
use fprime_utils::cfdp::Checksum;

/// Absolute opcode helper.
fn opcode(base: u32, local: u32) -> u32 {
    base + local
}

/// `OpCodeCompleted(opcode)` for an absolute opcode — the dispatcher's
/// "this command finished OK" event, the observable command response.
fn completed(harness: &Harness, absolute_opcode: u32) -> bool {
    let id = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    harness
        .log_events()
        .iter()
        .any(|(seen, args)| *seen == id && args[..] == absolute_opcode.to_be_bytes())
}

/// Uplink one framed command and tick until the dispatcher reports it
/// completed. Panics with the downlinked events on timeout.
fn run_command(harness: &Harness, absolute_opcode: u32, args: &[u8]) {
    harness.inject(&build_command_frame(absolute_opcode, args));
    assert!(
        harness.tick_until(|| completed(harness, absolute_opcode)),
        "command 0x{absolute_opcode:x} never completed; events {:?}",
        harness.log_events()
    );
}

// ---------------------------------------------------------------------------
// FileManager: a framed CreateDirectory command really creates a directory
// ---------------------------------------------------------------------------

/// Uplink a framed `FileManager.CreateDirectory` through the whole uplink
/// chain and assert (a) the directory exists on disk, (b) the
/// `CreateDirectoryStarted` / `CreateDirectorySucceeded` events came back
/// as downlink frames, and (c) the dispatcher answered `OpCodeCompleted`.
#[test]
fn file_manager_create_directory_end_to_end() {
    let harness = Harness::up();
    let target = format!("{}/made", harness.data_dir());
    assert!(!std::path::Path::new(&target).exists());

    let create = opcode(FILE_MANAGER_BASE_ID, FileManager::OPCODE_CREATE_DIRECTORY);
    run_command(&harness, create, &cmd_string_arg(&target));

    assert!(
        std::path::Path::new(&target).is_dir(),
        "CreateDirectory did not create {target}"
    );
    assert!(
        harness.saw_event(FILE_MANAGER_BASE_ID + FileManager::EVENTID_CREATE_DIRECTORY_STARTED)
    );
    assert!(
        harness.saw_event(FILE_MANAGER_BASE_ID + FileManager::EVENTID_CREATE_DIRECTORY_SUCCEEDED)
    );
    // The success event names the directory: `[u16 len][bytes]`.
    let args = harness
        .event_args(FILE_MANAGER_BASE_ID + FileManager::EVENTID_CREATE_DIRECTORY_SUCCEEDED)
        .expect("CreateDirectorySucceeded args");
    let len = usize::from(u16::from_be_bytes([args[0], args[1]]));
    assert_eq!(&args[2..2 + len], target.as_bytes());

    harness.topology.teardown();
}

/// A `RemoveDirectory` of a directory that is not there answers
/// `OpCodeError`, not `OpCodeCompleted` — the failure path travels the same
/// graph.
#[test]
fn file_manager_reports_command_errors_on_the_downlink() {
    let harness = Harness::up();
    let missing = format!("{}/nope", harness.data_dir());
    let remove = opcode(FILE_MANAGER_BASE_ID, FileManager::OPCODE_REMOVE_DIRECTORY);
    harness.inject(&build_command_frame(remove, &cmd_string_arg(&missing)));

    let error_id = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_ERROR;
    assert!(
        harness.tick_until(|| harness
            .log_events()
            .iter()
            .any(|(id, args)| *id == error_id && args[0..4] == remove.to_be_bytes())),
        "expected OpCodeError; events {:?}",
        harness.log_events()
    );
    assert!(harness.saw_event(FILE_MANAGER_BASE_ID + FileManager::EVENTID_DIRECTORY_REMOVE_ERROR));
    assert!(!completed(&harness, remove));

    harness.topology.teardown();
}

/// Uplink a framed `FileManager.GenerateDp` for a file in the data directory
/// and assert the whole data-product chain ran for it: three chunk
/// containers through `fileManager.productGetOut -> dpMgr[1] ->
/// dpBufferManager`, three `.fdp` files written by `dpWriter`, each carrying
/// the byte-exact `FileChunkHeaderRecord` + `FileChunkDataRecord` pair, and
/// `GenerateDpComplete(3)` on the downlink.
#[test]
fn file_manager_generate_dp_writes_chunk_containers_end_to_end() {
    let harness = Harness::up();
    let build = opcode(DP_CATALOG_BASE_ID, DpCatalog::OPCODE_BUILD_CATALOG);
    run_command(&harness, build, &[]);

    let path = format!("{}/dp.bin", harness.data_dir());
    let content: Vec<u8> = b"0123456789".repeat(10);
    std::fs::write(&path, &content).expect("source file");

    // GenerateDp(fileName, chunkSize=40, beginOffset=0, endOffset=0,
    // priority=0 (default), mode=IMMEDIATE).
    let mut args = cmd_string_arg(&path);
    args.extend_from_slice(&40u32.to_be_bytes());
    args.extend_from_slice(&0u64.to_be_bytes());
    args.extend_from_slice(&0u64.to_be_bytes());
    args.extend_from_slice(&0u32.to_be_bytes());
    args.extend_from_slice(
        &file_manager::GenerateDpMode::Immediate
            .as_repr()
            .to_be_bytes(),
    );
    let generate = opcode(FILE_MANAGER_BASE_ID, FileManager::OPCODE_GENERATE_DP);
    run_command(&harness, generate, &args);

    let complete = harness
        .event_args(FILE_MANAGER_BASE_ID + FileManager::EVENTID_GENERATE_DP_COMPLETE)
        .expect("GenerateDpComplete args");
    assert_eq!(&complete[complete.len() - 4..], &3u32.to_be_bytes());
    assert!(!harness.saw_event(FILE_MANAGER_BASE_ID + FileManager::EVENTID_GENERATE_DP_FAILED));
    assert!(
        !harness.saw_event(FILE_MANAGER_BASE_ID + FileManager::EVENTID_GENERATE_DP_BUFFER_FAILED)
    );

    assert!(
        harness.tick_until(|| dp_files(&harness).len() == 3),
        "expected three .fdp files, got {:?}; events {:?}",
        dp_files(&harness),
        harness.log_events()
    );

    // Each packet: [descriptor][id][priority][time]... header, header hash,
    // records, data hash. Collect the record bytes of every file and match
    // them against the three expected chunks regardless of file order.
    let mut seen: Vec<Vec<u8>> = dp_files(&harness)
        .iter()
        .map(|name| {
            let packet = std::fs::read(format!("{}/DpCat/{name}", harness.data_dir()))
                .expect("data product file");
            assert_eq!(&packet[0..2], &0x0005_u16.to_be_bytes());
            assert_eq!(
                &packet[2..6],
                &(FILE_MANAGER_BASE_ID + file_manager::CONTAINER_ID_FILE_DP).to_be_bytes()
            );
            assert_eq!(
                &packet[6..10],
                &file_manager::DEFAULT_DP_PRIORITY.to_be_bytes()
            );
            let size_at = fprime_fw::dp::Header::DATA_SIZE_OFFSET;
            let data_size = usize::from(u16::from_be_bytes([packet[size_at], packet[size_at + 1]]));
            let start = fprime_fw::dp::DpContainer::DATA_OFFSET;
            packet[start..start + data_size].to_vec()
        })
        .collect();
    seen.sort();
    let mut expected: Vec<Vec<u8>> = (0..3)
        .map(|i| {
            let offset = i * 40;
            let end = (offset + 40).min(content.len());
            let mut v = Vec::new();
            v.extend_from_slice(
                &(FILE_MANAGER_BASE_ID + file_manager::RECORD_ID_FILE_CHUNK_HEADER).to_be_bytes(),
            );
            v.extend_from_slice(&(path.len() as u16).to_be_bytes());
            v.extend_from_slice(path.as_bytes());
            v.extend_from_slice(&(offset as u64).to_be_bytes());
            v.extend_from_slice(&((end - offset) as u32).to_be_bytes());
            v.extend_from_slice(
                &(FILE_MANAGER_BASE_ID + file_manager::RECORD_ID_FILE_CHUNK_DATA).to_be_bytes(),
            );
            v.extend_from_slice(&((end - offset) as u16).to_be_bytes());
            v.extend_from_slice(&content[offset..end]);
            v
        })
        .collect();
    expected.sort();
    assert_eq!(seen, expected);

    harness.topology.teardown();
}

// ---------------------------------------------------------------------------
// FileUplink: a framed file transfer lands in the uplink sandbox
// ---------------------------------------------------------------------------

/// Serialize one `Fw::FilePacket` into its wire bytes.
fn file_packet_bytes(packet: &FilePacket<'_>) -> Vec<u8> {
    let mut buf = ComBuffer::new();
    assert!(packet.serialize_to(&mut buf).is_ok());
    buf.as_slice().to_vec()
}

/// Uplink START + DATA + END through the router's file path and assert the
/// reconstructed file is byte-identical in the uplink sandbox, with
/// `FileReceived` on the downlink. This also exercises the buffer-ownership
/// return path `fileUplink.bufferSendOut -> fprimeRouter.fileBufferReturnIn`
/// (the injected receive buffer must come all the way back to the driver).
#[test]
fn file_uplink_writes_the_uplinked_file() {
    let harness = Harness::up();
    let contents: Vec<u8> = (0..=200u8).collect();

    // The destination is ABSOLUTE: `Os::SandboxedFile::open` resolves the
    // ground-supplied path against the process CWD and then checks it
    // against the sandbox, so a relative name would land outside it (C++
    // parity — the GDS uplinks absolute destinations).
    let destination = format!("{}/uplink/up.bin", harness.data_dir());
    let start =
        StartPacket::initialize(contents.len() as u32, b"ground.bin", destination.as_bytes());
    harness.inject(&build_file_frame(&file_packet_bytes(&FilePacket::Start(
        start,
    ))));
    let data = DataPacket::initialize(1, 0, contents.len() as u16, &contents);
    harness.inject(&build_file_frame(&file_packet_bytes(&FilePacket::Data(
        data,
    ))));
    let mut checksum = Checksum::new();
    checksum.update(&contents, 0);
    let end = EndPacket::initialize(2, checksum.get_value());
    harness.inject(&build_file_frame(&file_packet_bytes(&FilePacket::End(end))));

    assert!(
        wait_until(DEADLINE, || harness
            .saw_event(FILE_UPLINK_BASE_ID + FileUplink::EVENTID_FILE_RECEIVED)),
        "expected FileReceived; events {:?}",
        harness.log_events()
    );
    assert_eq!(
        std::fs::read(&destination).expect("uplinked file"),
        contents
    );
    // Every injected buffer travelled the full ownership cycle back to the
    // driver: frameAccumulator -> deframer -> router -> fileUplink ->
    // router -> deframer -> frameAccumulator -> comStub -> driver.
    assert!(
        wait_until(DEADLINE, || harness
            .loopback
            .returned_buffers
            .lock()
            .unwrap()
            .len()
            == 3),
        "expected 3 returned receive buffers"
    );

    harness.topology.teardown();
}

// ---------------------------------------------------------------------------
// CmdSequencer: a sequence built in-test really dispatches its commands
// ---------------------------------------------------------------------------

/// A command com packet: `[0x0000 u16][opcode u32][args]`.
fn command_packet(absolute_opcode: u32, args: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(6 + args.len());
    out.extend_from_slice(&0u16.to_be_bytes());
    out.extend_from_slice(&absolute_opcode.to_be_bytes());
    out.extend_from_slice(args);
    out
}

/// One sequence record: `[descriptor u8][sec u32][usec u32][size u32][cmd]`.
/// Descriptor 1 = RELATIVE (fires `sec.usec` after the sequence starts).
fn relative_record(command: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(13 + command.len());
    out.push(1);
    out.extend_from_slice(&0u32.to_be_bytes());
    out.extend_from_slice(&0u32.to_be_bytes());
    out.extend_from_slice(&(command.len() as u32).to_be_bytes());
    out.extend_from_slice(command);
    out
}

/// An F Prime binary sequence file:
/// `[fileSize u32][numRecords u32][timeBase u16][timeContext u8][records]
///  [crc32 u32]`, where `fileSize` counts the records plus the trailing CRC
/// and the CRC covers the header and the records.
fn sequence_file(num_records: u32, records: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(15 + records.len());
    out.extend_from_slice(&((records.len() + 4) as u32).to_be_bytes());
    out.extend_from_slice(&num_records.to_be_bytes());
    // TB_DONT_CARE / FW_CONTEXT_DONT_CARE: run under any live time base.
    out.extend_from_slice(&(TimeBase::TbDontCare as u16).to_be_bytes());
    out.push(fprime_config::FW_CONTEXT_DONT_CARE);
    out.extend_from_slice(records);
    let crc = Hash::hash_u32(&out);
    out.extend_from_slice(&crc.to_be_bytes());
    out
}

/// Build a two-command sequence in a temp file, uplink `CS_RUN`, and assert
/// the sequencer really dispatched both commands through the REAL command
/// dispatcher (`cmdSeq.comCmdOut -> cmdDisp.seqCmdBuff[1]`, responses back
/// through `cmdDisp.seqCmdStatus[1]`): the dispatcher's own `NoOpReceived`
/// and the SignalGen `Toggled` event prove execution, and
/// `CS_CommandComplete` x2 + `CS_SequenceComplete` prove the sequencer saw
/// every response.
#[test]
fn cmd_sequencer_runs_a_sequence_through_the_dispatcher() {
    let harness = Harness::up();

    let no_op = opcode(CMD_DISPATCHER_BASE_ID, CmdDispatcher::OPCODE_CMD_NO_OP);
    let toggle = opcode(SIGNAL_GEN_BASE_ID, signal_gen::OPCODE_TOGGLE);
    let mut records = relative_record(&command_packet(no_op, &[]));
    records.extend_from_slice(&relative_record(&command_packet(toggle, &[])));
    records.push(2); // END_OF_SEQUENCE record
    let file = sequence_file(3, &records);

    let path = format!("{}/s.bin", harness.data_dir());
    std::fs::write(&path, &file).expect("write sequence");

    // CS_RUN(fileName: CmdStringArg, block: BlockState) — NO_BLOCK (1) so
    // the command answers immediately and the sequence runs on cmdSeq's
    // own thread, driven by the dispatcher's command responses.
    let mut args = cmd_string_arg(&path);
    args.push(1);
    let cs_run = opcode(CMD_SEQUENCER_BASE_ID, CmdSequencer::OPCODE_CS_RUN);
    run_command(&harness, cs_run, &args);

    let command_complete = CMD_SEQUENCER_BASE_ID + CmdSequencer::EVENTID_CS_COMMAND_COMPLETE;
    let sequence_complete = CMD_SEQUENCER_BASE_ID + CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE;
    assert!(
        harness.tick_until(|| harness.saw_event(sequence_complete)),
        "sequence never completed; events {:?}",
        harness.log_events()
    );

    // Both sequenced commands really executed, in order.
    assert!(harness.saw_event(CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED));
    assert!(harness.saw_event(SIGNAL_GEN_BASE_ID + signal_gen::EVENTID_TOGGLED));
    assert!(completed(&harness, no_op));
    assert!(completed(&harness, toggle));
    // The sequencer accounted for both of them.
    let completions = harness
        .log_events()
        .iter()
        .filter(|(id, _)| *id == command_complete)
        .count();
    assert_eq!(completions, 2, "expected one CS_CommandComplete per record");
    assert!(harness.saw_event(CMD_SEQUENCER_BASE_ID + CmdSequencer::EVENTID_CS_SEQUENCE_LOADED));

    harness.topology.teardown();
}

/// A sequence file whose CRC does not match is rejected with
/// `CS_FileCrcFailure` and no command is ever dispatched.
#[test]
fn cmd_sequencer_rejects_a_corrupt_sequence() {
    let harness = Harness::up();
    let no_op = opcode(CMD_DISPATCHER_BASE_ID, CmdDispatcher::OPCODE_CMD_NO_OP);
    let mut records = relative_record(&command_packet(no_op, &[]));
    records.push(2);
    let mut file = sequence_file(2, &records);
    let last = file.len() - 1;
    file[last] ^= 0xFF; // corrupt the stored CRC

    let path = format!("{}/bad.bin", harness.data_dir());
    std::fs::write(&path, &file).expect("write sequence");
    let mut args = cmd_string_arg(&path);
    args.push(1);
    let cs_run = opcode(CMD_SEQUENCER_BASE_ID, CmdSequencer::OPCODE_CS_RUN);
    harness.inject(&build_command_frame(cs_run, &args));

    let crc_failure = CMD_SEQUENCER_BASE_ID + CmdSequencer::EVENTID_CS_FILE_CRC_FAILURE;
    assert!(
        harness.tick_until(|| harness.saw_event(crc_failure)),
        "expected CS_FileCrcFailure; events {:?}",
        harness.log_events()
    );
    assert!(!harness.saw_event(CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED));

    harness.topology.teardown();
}

// ---------------------------------------------------------------------------
// PrmDb: a parameter set, saved to the database file, survives a restart
// ---------------------------------------------------------------------------

/// `AMPLITUDE_PARAM_SET` stages a value in SignalGen, `AMPLITUDE_PARAM_SAVE`
/// pushes it to `prmDb` over `prmSetOut`, `PRM_SAVE_FILE` writes the
/// database, and a SECOND topology booted on the same data directory reads
/// it back through `readParamFile` + `loadParameters`.
#[test]
fn parameter_saved_to_prm_db_survives_a_restart() {
    let data_dir = common::scratch_data_dir();
    let amplitude: f32 = 2.5;

    {
        let harness = Harness::up_in(&data_dir);
        // Nothing in the database yet: the FPP default is in force.
        assert_eq!(
            harness.topology.signal_gen.amplitude_param().0,
            signal_gen::AMPLITUDE_DEFAULT
        );

        let set = opcode(SIGNAL_GEN_BASE_ID, signal_gen::OPCODE_AMPLITUDE_PARAM_SET);
        run_command(&harness, set, &amplitude.to_be_bytes());
        assert!(harness.saw_event(SIGNAL_GEN_BASE_ID + signal_gen::EVENTID_AMPLITUDE_UPDATED));

        let save = opcode(SIGNAL_GEN_BASE_ID, signal_gen::OPCODE_AMPLITUDE_PARAM_SAVE);
        run_command(&harness, save, &[]);

        // The database write is a prmDb command; it is queued behind the
        // setPrm message the SAVE command just sent (same queue, same
        // priority, FIFO), so the file always contains the new value.
        let save_file = opcode(PRM_DB_BASE_ID, PrmDb::OPCODE_PRM_SAVE_FILE);
        run_command(&harness, save_file, &[]);
        assert!(harness.saw_event(PRM_DB_BASE_ID + PrmDb::EVENTID_PRM_FILE_SAVE_COMPLETE));

        let stored = std::fs::read(format!("{data_dir}/PrmDb.dat")).expect("PrmDb.dat");
        // Record: [0xA5][size u32][id u32][value]; the id is absolute.
        assert!(
            stored
                .windows(4)
                .any(|w| w == (SIGNAL_GEN_BASE_ID + signal_gen::PARAMID_AMPLITUDE).to_be_bytes()),
            "parameter id missing from the database file"
        );
        assert!(
            stored.windows(4).any(|w| w == amplitude.to_be_bytes()),
            "parameter value missing from the database file"
        );
        harness.topology.teardown();
    }

    // Reboot on the same data directory: readParamFile -> loadParameters.
    let restarted = Harness::up_in(&data_dir);
    assert_eq!(
        restarted.topology.signal_gen.amplitude_param(),
        (amplitude, fprime_fw::ParamValid::Valid)
    );
    restarted.topology.teardown();
}

// ---------------------------------------------------------------------------
// Data products: SignalGen produces one, DpWriter writes it, DpCatalog
// catalogs it
// ---------------------------------------------------------------------------

/// `.fdp` files in the deployment's data-product directory.
fn dp_files(harness: &Harness) -> Vec<String> {
    let dir = format!("{}/DpCat", harness.data_dir());
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .filter(|name| name.ends_with(".fdp"))
                .collect()
        })
        .unwrap_or_default();
    names.sort();
    names
}

/// Build the catalog, command SignalGen to produce a data product, and
/// assert the whole chain ran: `signalGen.productGetOut -> dpMgr ->
/// dpBufferManager` for the buffer, `dpMgr.productSendOut -> dpWriter` for
/// the packet, a `.fdp` file on disk, and `dpWriter.dpWrittenOut ->
/// dpCat.addToCat` putting it in the catalog.
#[test]
fn data_product_is_written_and_cataloged() {
    let harness = Harness::up();

    // The catalog must be built before addToCat will accept a product
    // (C++ parity: an unbuilt catalog answers DpFileNotLoaded).
    let build = opcode(DP_CATALOG_BASE_ID, DpCatalog::OPCODE_BUILD_CATALOG);
    run_command(&harness, build, &[]);
    assert!(dp_files(&harness).is_empty());

    let records: u32 = 4;
    let dp = opcode(SIGNAL_GEN_BASE_ID, signal_gen::OPCODE_DP);
    run_command(&harness, dp, &records.to_be_bytes());
    assert!(harness.saw_event(SIGNAL_GEN_BASE_ID + signal_gen::EVENTID_DP_SENT));

    assert!(
        harness
            .tick_until(|| harness.saw_event(DP_WRITER_BASE_ID + DpWriter::EVENTID_FILE_WRITTEN)),
        "expected DpWriter FileWritten; events {:?}",
        harness.log_events()
    );
    let files = dp_files(&harness);
    assert_eq!(
        files.len(),
        1,
        "expected exactly one .fdp file, got {files:?}"
    );
    let packet = std::fs::read(format!("{}/DpCat/{}", harness.data_dir(), files[0]))
        .expect("data product file");
    // 57-byte header + 4-byte header hash + records + 4-byte data hash.
    assert_eq!(packet.len(), 57 + 4 + (records as usize * 8) + 4);
    // Header: [descriptor 0x0005][id u32][priority u32][time 11B]...
    assert_eq!(&packet[0..2], &0x0005_u16.to_be_bytes());
    assert_eq!(
        &packet[2..6],
        &(SIGNAL_GEN_BASE_ID + signal_gen::CONTAINER_ID_DATA).to_be_bytes()
    );
    assert_eq!(
        &packet[6..10],
        &signal_gen::CONTAINER_PRIORITY_DATA.to_be_bytes()
    );

    assert!(
        harness.tick_until(|| harness.saw_event(DP_CATALOG_BASE_ID + DpCatalog::EVENTID_DP_FILE_ADDED)),
        "expected DpCatalog DpFileAdded; events {:?}",
        harness.log_events()
    );
    assert_eq!(harness.topology.dp_catalog.catalog_size(), 1);

    harness.topology.teardown();
}
