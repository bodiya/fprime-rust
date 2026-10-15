//! The sequence file (upstream `readHeader`, `readBody`, `readFooter`,
//! `readBytes`, `validate`, `CrcHardcodedValue_*`, `allocateBuffer`), plus
//! the decode failures upstream could not reach.

use super::*;
use fprime_config::fpy_sequencer::{MAX_SEQUENCE_ARG_COUNT, MAX_SEQUENCE_STATEMENT_COUNT};
use fprime_fw::Success;
use fprime_os::file::{File, Mode as FileMode};

/// Put `bytes` in the sequence buffer, as if read from a file.
fn load_buffer(t: &Tester, capacity: usize, bytes: &[u8]) {
    let mut st = t.st();
    st.sequence_buffer.set_ext_buffer(capacity);
    assert!(st.sequence_buffer.set_buff(bytes).is_ok());
}

fn header_bytes(header: &Header) -> Vec<u8> {
    let mut buf = LinearBuffer::<11>::new();
    assert!(buf.serialize(header, Endianness::Big).is_ok());
    buf.as_slice().to_vec()
}

#[test]
fn read_header() {
    let t = Tester::new();
    let mut header = Header {
        argument_count: MAX_SEQUENCE_ARG_COUNT as u8,
        body_size: 50,
        schema_version: SCHEMA_VERSION,
        statement_count: MAX_SEQUENCE_STATEMENT_COUNT as u16,
        ..Header::default()
    };
    load_buffer(&t, 11, &header_bytes(&header));
    assert_eq!(t.with(|c, st| c.read_header(st)), Success::Success);
    assert_eq!(t.st().sequence_obj.header, header);

    // one byte short
    {
        let mut st = t.st();
        st.sequence_buffer.reset_deser();
        assert!(st.sequence_buffer.set_buff_len(10).is_ok());
    }
    assert_eq!(t.with(|c, st| c.read_header(st)), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_FILE_READ_DESERIALIZE_ERROR);
    assert_eq!(events.len(), 1);
    // HEADER, empty path, DeserSizeMismatch, 3 left of 10 (the U32 body size does not fit)
    assert_eq!(
        events[0],
        [
            0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 10
        ]
    );

    header.schema_version = SCHEMA_VERSION + 1;
    load_buffer(&t, 11, &header_bytes(&header));
    assert_eq!(t.with(|c, st| c.read_header(st)), Success::Failure);
    assert_eq!(
        t.events(FpySequencer::EVENTID_WRONG_SCHEMA_VERSION),
        vec![vec![SCHEMA_VERSION, SCHEMA_VERSION + 1]]
    );
    header.schema_version = SCHEMA_VERSION;
    t.clear_history();

    header.argument_count = MAX_SEQUENCE_ARG_COUNT as u8 + 1;
    load_buffer(&t, 11, &header_bytes(&header));
    assert_eq!(t.with(|c, st| c.read_header(st)), Success::Failure);
    assert_eq!(
        t.events(FpySequencer::EVENTID_TOO_MANY_SEQUENCE_ARGS),
        vec![vec![17, 16]]
    );
    header.argument_count = MAX_SEQUENCE_ARG_COUNT as u8;

    header.statement_count = MAX_SEQUENCE_STATEMENT_COUNT as u16 + 1;
    load_buffer(&t, 11, &header_bytes(&header));
    assert_eq!(t.with(|c, st| c.read_header(st)), Success::Failure);
    assert_eq!(
        t.events(FpySequencer::EVENTID_TOO_MANY_SEQUENCE_DIRECTIVES),
        vec![vec![0x08, 0x01, 0x08, 0x00]]
    );
}

/// `[u16 len][bytes]`.
fn string(value: &str) -> Vec<u8> {
    let mut out = (value.len() as u16).to_be_bytes().to_vec();
    out.extend_from_slice(value.as_bytes());
    out
}

#[test]
fn read_body() {
    let t = Tester::new();
    let capacity = MAX_SEQUENCE_ARG_COUNT * ArgSpec::SERIALIZED_SIZE
        + MAX_SEQUENCE_STATEMENT_COUNT * Statement::SERIALIZED_SIZE;

    // argument specs
    let mut body = Vec::new();
    for i in 0..MAX_SEQUENCE_ARG_COUNT {
        body.extend(string(&format!("arg{i}")));
        body.extend(string("U32"));
        body.extend(4u32.to_be_bytes());
    }
    load_buffer(&t, capacity, &body);
    {
        let mut st = t.st();
        st.sequence_args.size = (MAX_SEQUENCE_ARG_COUNT * 4) as FwSizeType;
        st.sequence_obj.header.argument_count = MAX_SEQUENCE_ARG_COUNT as u8;
        st.sequence_obj.header.statement_count = 0;
    }
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Success);
    {
        let st = t.st();
        for i in 0..MAX_SEQUENCE_ARG_COUNT {
            let spec = &st.sequence_obj.args[i];
            assert_eq!(spec.arg_name, format!("arg{i}").as_str());
            assert_eq!(spec.type_name, "U32");
            assert_eq!(spec.arg_size, 4);
        }
        assert_eq!(st.total_expected_arg_size, 64);
    }

    // one argument spec short
    let mut body = Vec::new();
    for _ in 0..MAX_SEQUENCE_ARG_COUNT - 1 {
        body.extend(string(""));
        body.extend(string(""));
        body.extend(4u32.to_be_bytes());
    }
    load_buffer(&t, capacity, &body);
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Failure);

    // statements
    {
        let mut st = t.st();
        st.sequence_args = SeqArgs::default();
        st.sequence_obj.header.argument_count = 0;
        st.sequence_obj.header.statement_count = MAX_SEQUENCE_STATEMENT_COUNT as u16;
    }
    let no_op = [DirectiveId::NoOp.as_repr(), 0, 0];
    let body: Vec<u8> = no_op
        .iter()
        .copied()
        .cycle()
        .take(3 * MAX_SEQUENCE_STATEMENT_COUNT)
        .collect();
    load_buffer(&t, capacity, &body);
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Success);
    {
        let st = t.st();
        for stmt in st.sequence_obj.statements.iter() {
            assert_eq!(stmt.op_code, DirectiveId::NoOp);
            assert_eq!(stmt.arg_buf.get_size(), 0);
        }
    }

    // no statements written
    t.st().sequence_obj.header.statement_count = 1;
    load_buffer(&t, capacity, &[]);
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Failure);

    // one statement short
    t.st().sequence_obj.header.statement_count = MAX_SEQUENCE_STATEMENT_COUNT as u16;
    load_buffer(&t, capacity, &body[..body.len() - 3]);
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Failure);
}

#[test]
fn read_body_checks_the_argument_sizes() {
    let t = Tester::new();
    let mut body = Vec::new();
    for _ in 0..2 {
        body.extend(string("a"));
        body.extend(string("bytes"));
        body.extend(((MAX_STACK_SIZE / 2 + 1) as u32).to_be_bytes());
    }
    load_buffer(&t, 1024, &body);
    t.st().sequence_obj.header.argument_count = 2;
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Failure);
    assert_eq!(
        t.events(FpySequencer::EVENTID_ARG_TOTAL_SIZE_EXCEEDS_STACK_LIMIT),
        vec![((MAX_STACK_SIZE / 2 + 1) as u32 * 2).to_be_bytes().to_vec()]
    );

    // the sum wraps in a U32, as upstream's does
    let mut body = Vec::new();
    for size in [0xFFFF_FFFFu32, 5] {
        body.extend(string("a"));
        body.extend(string("b"));
        body.extend(size.to_be_bytes());
    }
    load_buffer(&t, 1024, &body);
    t.st().sequence_args.size = 4;
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Success);
    assert_eq!(t.st().total_expected_arg_size, 4);

    // a mismatch with the sequence arguments
    load_buffer(&t, 1024, &body);
    t.st().sequence_args.size = 8;
    t.clear_history();
    assert_eq!(t.with(|c, st| c.read_body(st)), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_ARG_SIZE_MISMATCH);
    assert_eq!(events, vec![vec![0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0]]);
}

use fprime_config::fpy_sequencer::MAX_STACK_SIZE;

#[test]
fn read_footer() {
    let t = Tester::new();
    t.st().computed_crc.set_hash_value(!0x1234_5678);
    let footer = Footer::new(!0x1234_5678);
    load_buffer(&t, 4, &footer.crc.to_be_bytes());
    assert_eq!(t.with(|c, st| c.read_footer(st)), Success::Success);
    assert_eq!(t.st().sequence_obj.footer, footer);

    // a wrong CRC
    t.st().sequence_buffer.reset_deser();
    t.st().computed_crc.set_hash_value(!0x4444_4444);
    assert_eq!(t.with(|c, st| c.read_footer(st)), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_WRONG_CRC);
    assert_eq!(events.len(), 1);
    // expected (the file's), actual (computed)
    let mut expected = (!0x1234_5678u32).to_be_bytes().to_vec();
    expected.extend((!0x4444_4444u32).to_be_bytes());
    assert_eq!(events[0], expected);

    // not enough bytes
    assert!(t.st().sequence_buffer.set_buff_len(3).is_ok());
    assert_eq!(t.with(|c, st| c.read_footer(st)), Success::Failure);
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_FILE_READ_DESERIALIZE_ERROR),
        1
    );
}

fn open(path: &str) -> File {
    let mut file = File::new();
    assert_eq!(
        file.open(path, FileMode::OpenRead),
        fprime_os::file::Status::OpOk
    );
    file
}

#[test]
fn read_bytes() {
    // no statements: header and footer only (15 bytes)
    let mut t = Tester::new();
    let path = t.write_to_file("test.bin");
    let read = |t: &Tester, file: &mut File, len: usize| {
        t.with(|c, st| c.read_bytes(st, file, len as FwSizeType, FileReadStage::Header, true))
    };
    t.st()
        .sequence_buffer
        .set_ext_buffer(Sequence::SERIALIZED_SIZE);
    let mut file = open(&path);
    assert_eq!(read(&t, &mut file, 11), Success::Success);
    assert_eq!(t.st().sequence_buffer.get_size(), 11);
    file.close();

    // capacity too low
    t.st().sequence_buffer.set_ext_buffer(10);
    let mut file = open(&path);
    assert_eq!(read(&t, &mut file, 11), Success::Failure);
    assert_eq!(
        t.events(FpySequencer::EVENTID_INSUFFICIENT_BUFFER_SPACE)[0][..8],
        [0, 0, 0, 0, 0, 0, 0, 10]
    );
    file.close();

    // not enough bytes in the file
    t.st().sequence_buffer.set_ext_buffer(16);
    let mut file = open(&path);
    assert_eq!(read(&t, &mut file, 16), Success::Failure);
    assert_eq!(t.event_count(FpySequencer::EVENTID_END_OF_FILE_ERROR), 1);
    file.close();
}

#[test]
#[should_panic(expected = "Assert")]
fn read_bytes_from_a_closed_file_asserts() {
    let t = Tester::new();
    let mut file = File::new();
    t.with(|c, st| c.read_bytes(st, &mut file, 11, FileReadStage::Header, true));
}

#[test]
fn read_bytes_feeds_the_crc_unless_told_not_to() {
    let mut t = Tester::new();
    let path = t.write_to_file("test.bin");
    t.st().sequence_buffer.set_ext_buffer(64);
    t.st().computed_crc.init();
    let mut file = open(&path);
    t.with(|c, st| c.read_bytes(st, &mut file, 4, FileReadStage::Header, false));
    assert_eq!(t.st().computed_crc.finalize(), Hash::new().finalize());
    t.with(|c, st| c.read_bytes(st, &mut file, 4, FileReadStage::Header, true));
    let mut expected = Hash::new();
    expected.update(&std::fs::read(&path).unwrap()[4..8]);
    assert_eq!(t.st().computed_crc.finalize(), expected.finalize());
}

fn run_validate(t: &Tester) -> Success {
    t.with(|c, st| c.validate(st))
}

fn set_path(t: &Tester, path: &str) {
    t.st().sequence_file_path.set(path);
}

#[test]
fn validate() {
    let mut t = Tester::new();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.st()
        .sequence_buffer
        .set_ext_buffer(Sequence::SERIALIZED_SIZE);
    set_path(&t, &path);
    assert_eq!(run_validate(&t), Success::Success);
    assert_eq!(t.st().sequence_obj.header.statement_count, 1);
    assert_eq!(t.st().sequence_obj.statements[0].op_code, DirectiveId::NoOp);

    // the file cannot be opened
    t.remove_file("test.bin");
    assert_eq!(run_validate(&t), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_FILE_OPEN_ERROR);
    assert_eq!(events.len(), 1);
    assert_eq!(first_string(&events[0]), path);
    t.clear_history();

    // a short header
    t.write_to_file_max("test.bin", 10);
    assert_eq!(run_validate(&t), Success::Failure);
    assert_eq!(t.event_count(FpySequencer::EVENTID_END_OF_FILE_ERROR), 1);
    t.clear_history();

    // a bad header
    t.seq.header.schema_version = SCHEMA_VERSION + 1;
    t.write_to_file("test.bin");
    assert_eq!(run_validate(&t), Success::Failure);
    assert_eq!(t.event_count(FpySequencer::EVENTID_WRONG_SCHEMA_VERSION), 1);
    t.seq.header.schema_version = SCHEMA_VERSION;
    t.clear_history();

    // a short body
    t.write_to_file_max("test.bin", 12);
    assert_eq!(run_validate(&t), Success::Failure);
    assert_eq!(t.event_count(FpySequencer::EVENTID_END_OF_FILE_ERROR), 1);
    t.clear_history();

    // an extra byte after the footer
    t.write_to_file("test.bin");
    let mut bytes = std::fs::read(&path).unwrap();
    bytes.push(0);
    std::fs::write(&path, &bytes).unwrap();
    assert_eq!(run_validate(&t), Success::Failure);
    assert_eq!(
        t.events(FpySequencer::EVENTID_EXTRA_BYTES_IN_SEQUENCE),
        vec![vec![0, 0, 0, 0, 0, 0, 0, 1]]
    );
}

#[test]
fn validate_rejects_a_bad_crc_and_a_short_footer() {
    let mut t = Tester::new();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.alloc_mem();
    set_path(&t, &path);
    let good = std::fs::read(&path).unwrap();

    let mut bad = good.clone();
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    std::fs::write(&path, &bad).unwrap();
    assert_eq!(run_validate(&t), Success::Failure);
    assert_eq!(t.event_count(FpySequencer::EVENTID_WRONG_CRC), 1);

    // a body byte flipped: the CRC no longer matches either
    let mut bad = good.clone();
    bad[11] ^= 0x01;
    std::fs::write(&path, &bad).unwrap();
    t.clear_history();
    assert_eq!(run_validate(&t), Success::Failure);
    assert_eq!(t.event_count(FpySequencer::EVENTID_WRONG_CRC), 1);

    std::fs::write(&path, &good[..good.len() - 2]).unwrap();
    t.clear_history();
    assert_eq!(run_validate(&t), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_END_OF_FILE_ERROR);
    // FOOTER
    assert_eq!(events[0][0], FileReadStage::Footer.as_repr());
}

#[test]
fn validate_reports_body_decode_errors() {
    let mut t = Tester::new();
    t.alloc_mem();
    // an undeclared directive id: DeserFormatError (4)
    t.add_no_op();
    t.seq.statements[0].op_code = DirectiveId::NoOp;
    let path = t.write_to_file("test.bin");
    let mut bytes = std::fs::read(&path).unwrap();
    bytes[11] = 200;
    // re-CRC the edited file
    let mut hash = Hash::new();
    hash.update(&bytes[..bytes.len() - 4]);
    let n = bytes.len();
    bytes[n - 4..].copy_from_slice(&hash.finalize().to_be_bytes());
    std::fs::write(&path, &bytes).unwrap();
    set_path(&t, &path);
    assert_eq!(run_validate(&t), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_FILE_READ_DESERIALIZE_ERROR);
    assert_eq!(events.len(), 1);
    // BODY, the path, then DeserFormatError
    let path_len = path.len();
    assert_eq!(events[0][0], FileReadStage::Body.as_repr());
    assert_eq!(&events[0][3 + path_len..7 + path_len], &4i32.to_be_bytes());

    // a statement argument buffer longer than StatementArgBuffer
    let mut bytes = Vec::new();
    let body_len: u32 = 1 + 2 + 507;
    bytes.extend([0, 0, 0, SCHEMA_VERSION, 0, 0, 1]);
    bytes.extend(body_len.to_be_bytes());
    bytes.push(DirectiveId::PushVal.as_repr());
    bytes.extend(507u16.to_be_bytes());
    bytes.extend([0u8; 507]);
    let mut hash = Hash::new();
    hash.update(&bytes);
    bytes.extend(hash.finalize().to_be_bytes());
    std::fs::write(&path, &bytes).unwrap();
    t.clear_history();
    assert_eq!(run_validate(&t), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_FILE_READ_DESERIALIZE_ERROR);
    // DeserSizeMismatch (5)
    assert_eq!(&events[0][3 + path_len..7 + path_len], &5i32.to_be_bytes());
}

#[test]
fn validate_needs_a_buffer_for_the_whole_body() {
    let mut t = Tester::new();
    for _ in 0..10 {
        t.add_no_op();
    }
    let path = t.write_to_file("test.bin");
    // the header fits, the 30-byte body does not
    t.comp.allocate_buffer(0, 20);
    set_path(&t, &path);
    assert_eq!(run_validate(&t), Success::Failure);
    let events = t.events(FpySequencer::EVENTID_INSUFFICIENT_BUFFER_SPACE);
    assert_eq!(&events[0][..8], &20u64.to_be_bytes());
    assert_eq!(first_string(&events[0][8..]), path);
}

#[test]
fn validate_rejects_arguments_beyond_the_seq_args_buffer() {
    // the size travels separately from the 12-byte buffer
    let mut t = Tester::new();
    t.add_argument_spec("big", "bytes", 13);
    let path = t.write_to_file("test.bin");
    t.alloc_mem();
    set_path(&t, &path);
    t.st().sequence_args.size = 13;
    assert_eq!(run_validate(&t), Success::Failure);
    let mut expected = 13u64.to_be_bytes().to_vec();
    expected.extend(12u64.to_be_bytes());
    assert_eq!(
        t.events(FpySequencer::EVENTID_ARG_SIZE_EXCEEDS_CAPACITY),
        vec![expected]
    );
}

#[test]
fn validate_of_an_empty_path_is_an_open_error() {
    let t = Tester::new();
    t.alloc_mem();
    assert_eq!(run_validate(&t), Success::Failure);
    // INVALID_ARGUMENT (9)
    assert_eq!(
        t.events(FpySequencer::EVENTID_FILE_OPEN_ERROR),
        vec![vec![0, 0, 0, 0, 0, 9]]
    );
}

#[test]
fn crc_hardcoded_values() {
    let t = Tester::new();
    let crc = |chunks: &[&[u8]]| {
        let mut st = t.st();
        st.computed_crc.init();
        for chunk in chunks {
            st.computed_crc.update(chunk);
        }
        st.computed_crc.finalize()
    };
    assert_eq!(crc(&[b"123456789"]), 0xCBF4_3926);
    assert_eq!(crc(&[&[0xDE, 0xAD, 0xBE, 0xEF]]), 0x7C9C_A35A);
    assert_eq!(crc(&[&[0x00]]), 0xD202_EF8D);
    assert_eq!(crc(&[b"1234", b"56789"]), 0xCBF4_3926);
    assert_eq!(crc(&[b"Hello"]), 0xF7D1_8982);
}

#[test]
fn allocate_buffer() {
    let t = Tester::new();
    t.comp.allocate_buffer(7, 100);
    assert!(t.st().sequence_buffer.is_allocated());
    assert_eq!(SerBufAny::capacity(&t.st().sequence_buffer), 100);
    assert_eq!(t.st().allocator_id, 7);
    t.comp.deallocate_buffer();
    assert!(!t.st().sequence_buffer.is_allocated());
    assert_eq!(SerBufAny::capacity(&t.st().sequence_buffer), 0);
}

#[test]
#[should_panic(expected = "Assert")]
fn allocate_buffer_smaller_than_a_header_asserts() {
    let t = Tester::new();
    t.comp.allocate_buffer(0, 10);
}
