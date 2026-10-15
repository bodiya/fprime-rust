//! PDU encode/decode tests — ports of `Types/test/ut/PduTests.cpp`, the PDU
//! half of `test/ut/CfdpManagerHelperTests.cpp`, and the engine-driven PDU
//! tests of `PduTester.cpp` (`testMetaDataPdu` ... `testNakPdu`).
//!
//! The literal byte vectors were cross-checked against the CFDP PDU
//! encoders of `spacepackets` 0.32.0 (the CCSDS packet library installed
//! with fprime-gds 4.3.1, which itself ships no CFDP engine). They agree
//! byte for byte except where upstream F´ departs from CCSDS 727.0-B-5,
//! which the port keeps (see `metadata_pdu_literal_bytes` and
//! `ack_pdu_literal_bytes`).

use super::*;
use crate::ccsds::cfdp_manager::pdu::{
    LargeFileFlag, PduHeader, PduType, Tlv, TlvList, TlvType, peek_pdu_type,
};
use fprime_config::cfdp::{MAX_TLV, NAK_MAX_SEGMENTS};

fn round_trip_header(h: &PduHeader) -> PduHeader {
    let mut out = [0u8; 64];
    let mut sb = ExtBuf::new(&mut out);
    assert_eq!(h.to_serial_buffer(&mut sb), SerializeStatus::Ok);
    let len = sb.ser_loc();
    let mut rx = PduHeader::default();
    assert_eq!(
        rx.from_serial_buffer(&mut PduReader::new(&out[..len])),
        SerializeStatus::Ok
    );
    rx
}

// ---- header ---------------------------------------------------------------

#[test]
fn header_round_trip_and_minimal_widths() {
    let mut h = PduHeader::default();
    h.initialize(
        PduTypeEnum::Metadata,
        PduDirection::TowardSender,
        Class::Class2,
        10,
        20,
        30,
    );
    h.set_pdu_data_length(100);
    assert_eq!(h.get_buffer_size(), 7);
    let rx = round_trip_header(&h);
    assert_eq!(rx.get_direction(), PduDirection::TowardSender);
    assert_eq!(rx.get_txm_mode(), Class::Class2);
    assert_eq!(rx.get_source_eid(), 10);
    assert_eq!(rx.get_transaction_seq(), 20);
    assert_eq!(rx.get_dest_eid(), 30);
    assert_eq!(rx.get_pdu_data_length(), 100);
    assert_eq!(rx.get_version(), 1);
    // A directive header decodes with type NONE until the code is read.
    assert_eq!(rx.get_type(), PduTypeEnum::None);
}

#[test]
fn header_value_encoded_size() {
    for (value, size) in [
        (0u64, 1u8),
        (0xFF, 1),
        (0x100, 2),
        (0xFFFF, 2),
        (0x1_0000, 3),
        (0xFFFF_FFFF, 4),
        (0x1_0000_0000, 5),
        (u64::MAX, 8),
    ] {
        assert_eq!(PduHeader::get_value_encoded_size(value), size, "{value:#x}");
    }
}

#[test]
fn header_eids_share_the_width_of_the_larger() {
    let mut h = PduHeader::default();
    h.initialize(
        PduTypeEnum::EndOfFile,
        PduDirection::TowardReceiver,
        Class::Class1,
        0x1234,
        0x0100_0000,
        0x56,
    );
    assert_eq!(h.get_buffer_size(), 4 + 2 + 4 + 2);
    let rx = round_trip_header(&h);
    assert_eq!(rx.get_source_eid(), 0x1234);
    assert_eq!(rx.get_dest_eid(), 0x56);
    assert_eq!(rx.get_transaction_seq(), 0x0100_0000);
    assert_eq!(rx.get_txm_mode(), Class::Class1);
}

#[test]
fn header_decode_reports_short_buffers() {
    let mut h = PduHeader::default();
    assert_eq!(
        h.from_serial_buffer(&mut PduReader::new(&[])),
        SerializeStatus::DeserBufferEmpty
    );
    assert_eq!(
        h.from_serial_buffer(&mut PduReader::new(&[0x20, 0x00])),
        SerializeStatus::DeserSizeMismatch
    );
    // Header claims 2-byte EIDs but stops after one byte of the source.
    assert_eq!(
        h.from_serial_buffer(&mut PduReader::new(&[0x20, 0x00, 0x00, 0x10, 0x01])),
        SerializeStatus::DeserBufferEmpty
    );
}

#[test]
fn peek_pdu_type_all_directives_and_edge_cases() {
    let cases = [
        (
            md_bytes(Class::Class2, 1, 2, 3, 10, "a", "b"),
            PduTypeEnum::Metadata,
        ),
        (
            fd_bytes(Class::Class2, 1, 2, 3, 0, &[1]),
            PduTypeEnum::FileData,
        ),
        (
            eof_bytes(Class::Class2, 1, 2, 3, ConditionCode::NO_ERROR, 0, 0),
            PduTypeEnum::EndOfFile,
        ),
        (
            fin_bytes(
                1,
                2,
                3,
                ConditionCode::NO_ERROR,
                FinDeliveryCode::Complete,
                FinFileStatus::Retained,
            ),
            PduTypeEnum::Finished,
        ),
        (
            ack_bytes(
                1,
                2,
                3,
                FileDirective::FIN,
                1,
                ConditionCode::NO_ERROR,
                AckTxnStatus::TERMINATED,
            ),
            PduTypeEnum::Acknowledgment,
        ),
        (nak_bytes(1, 2, 3, (0, 10), &[]), PduTypeEnum::NegativeAck),
    ];
    for (bytes, expected) in &cases {
        assert_eq!(peek_pdu_type(bytes), *expected);
    }
    // Too short for a header.
    assert_eq!(peek_pdu_type(&[0x20; 6]), PduTypeEnum::None);
    // Directive code beyond the buffer.
    assert_eq!(peek_pdu_type(&[0x20, 0, 0, 0, 1, 2, 3]), PduTypeEnum::None);
    // Unknown directive code (PROMPT).
    assert_eq!(
        peek_pdu_type(&[0x20, 0, 1, 0, 1, 2, 3, 9]),
        PduTypeEnum::None
    );
}

// ---- literal vectors (cross-checked against spacepackets 0.32.0) ----------

#[test]
fn metadata_pdu_literal_bytes() {
    let bytes = md_bytes(Class::Class2, 1, 3, 2, 1024, "src.txt", "dst.txt");
    // spacepackets puts closure-requested in bit 6 of the octet after the
    // directive code (0x40), as CCSDS 727.0-B-5 table 5-9 does; upstream
    // F´ puts it in the MSB (0x80). The port keeps the F´ encoding.
    assert_eq!(
        bytes,
        [
            0x20, 0x00, 0x16, 0x00, 0x01, 0x02, 0x03, 0x07, 0x80, 0x00, 0x00, 0x04, 0x00, 0x07,
            0x73, 0x72, 0x63, 0x2E, 0x74, 0x78, 0x74, 0x07, 0x64, 0x73, 0x74, 0x2E, 0x74, 0x78,
            0x74
        ]
    );
}

#[test]
fn file_data_pdu_literal_bytes() {
    assert_eq!(
        fd_bytes(Class::Class1, 10, 30, 20, 50, &[1, 2, 3, 4, 5]),
        [
            0x34, 0x00, 0x09, 0x00, 0x0A, 0x14, 0x1E, 0x00, 0x00, 0x00, 0x32, 0x01, 0x02, 0x03,
            0x04, 0x05
        ]
    );
}

#[test]
fn eof_pdu_literal_bytes() {
    assert_eq!(
        eof_bytes(
            Class::Class2,
            1,
            3,
            2,
            ConditionCode::NO_ERROR,
            0x1234_5678,
            4096
        ),
        [
            0x20, 0x00, 0x0A, 0x00, 0x01, 0x02, 0x03, 0x04, 0x00, 0x12, 0x34, 0x56, 0x78, 0x00,
            0x00, 0x10, 0x00
        ]
    );
    // Cancel, with the fault-location entity-ID TLV (4-byte EID).
    let mut eof = EofPdu::new();
    eof.initialize(
        PduDirection::TowardReceiver,
        Class::Class2,
        1,
        2,
        3,
        ConditionCode::CANCEL_REQUEST_RECEIVED,
        0x1234_5678,
        100,
    );
    let mut tlv = Tlv::new();
    tlv.initialize_entity_id(42);
    assert!(eof.append_tlv(&tlv));
    assert_eq!(
        encode(&eof),
        [
            0x20, 0x00, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x00,
            0x00, 0x00, 0x64, 0x06, 0x04, 0x00, 0x00, 0x00, 0x2A
        ]
    );
    // Wide fields: 2-byte EIDs, 4-byte sequence number.
    assert_eq!(
        eof_bytes(
            Class::Class2,
            0x1234,
            0x56,
            0x0100_0000,
            ConditionCode::NO_ERROR,
            0xDEAD_BEEF,
            70000
        ),
        [
            0x20, 0x00, 0x0A, 0x13, 0x12, 0x34, 0x01, 0x00, 0x00, 0x00, 0x00, 0x56, 0x04, 0x00,
            0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x11, 0x70
        ]
    );
}

#[test]
fn fin_pdu_literal_bytes() {
    assert_eq!(
        fin_bytes(
            100,
            42,
            7,
            ConditionCode::NO_ERROR,
            FinDeliveryCode::Complete,
            FinFileStatus::Retained
        ),
        [0x28, 0x00, 0x02, 0x00, 0x64, 0x07, 0x2A, 0x05, 0x02]
    );
    let mut fin = FinPdu::new();
    fin.initialize(
        PduDirection::TowardSender,
        Class::Class2,
        100,
        7,
        42,
        ConditionCode::FILE_CHECKSUM_FAILURE,
        FinDeliveryCode::Incomplete,
        FinFileStatus::Discarded,
    );
    let mut tlv = Tlv::new();
    tlv.initialize_entity_id(42);
    assert!(fin.append_tlv(&tlv));
    assert_eq!(
        encode(&fin),
        [
            0x28, 0x00, 0x08, 0x00, 0x64, 0x07, 0x2A, 0x05, 0x54, 0x06, 0x04, 0x00, 0x00, 0x00,
            0x2A
        ]
    );
}

#[test]
fn ack_pdu_literal_bytes() {
    // ACK(FIN): identical to spacepackets.
    assert_eq!(
        ack_bytes(
            42,
            100,
            7,
            FileDirective::FIN,
            1,
            ConditionCode::NO_ERROR,
            AckTxnStatus::TERMINATED
        ),
        [0x20, 0x00, 0x03, 0x00, 0x2A, 0x07, 0x64, 0x06, 0x51, 0x02]
    );
    // ACK(EOF): upstream F´ always sends directive subtype 1; the standard
    // (and spacepackets) use 0 for an EOF acknowledgment (0x40).
    assert_eq!(
        ack_bytes(
            100,
            42,
            7,
            FileDirective::END_OF_FILE,
            1,
            ConditionCode::NO_ERROR,
            AckTxnStatus::ACTIVE
        ),
        [0x28, 0x00, 0x03, 0x00, 0x64, 0x07, 0x2A, 0x06, 0x41, 0x01]
    );
}

#[test]
fn nak_pdu_literal_bytes() {
    assert_eq!(
        nak_bytes(
            100,
            42,
            7,
            (0, 5000),
            &[
                SegmentRequest {
                    offset_start: 992,
                    offset_end: 1984
                },
                SegmentRequest {
                    offset_start: 3968,
                    offset_end: 4960
                }
            ]
        ),
        [
            0x28, 0x00, 0x19, 0x00, 0x64, 0x07, 0x2A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x13, 0x88, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x0F, 0x80,
            0x00, 0x00, 0x13, 0x60
        ]
    );
}

// ---- metadata -------------------------------------------------------------

#[test]
fn metadata_round_trip() {
    let bytes = md_bytes(
        Class::Class2,
        100,
        300,
        200,
        2048,
        "/ground/test_source.bin",
        "test/ut/output/test_dest.bin",
    );
    let md = decode_md(&bytes);
    assert_eq!(md.as_header().get_type(), PduTypeEnum::Metadata);
    assert_eq!(md.as_header().get_direction(), PduDirection::TowardReceiver);
    assert_eq!(md.as_header().get_txm_mode(), Class::Class2);
    assert_eq!(md.as_header().get_source_eid(), 100);
    assert_eq!(md.as_header().get_transaction_seq(), 200);
    assert_eq!(md.as_header().get_dest_eid(), 300);
    assert_eq!(md.get_file_size(), 2048);
    assert_eq!(
        md.get_source_filename().as_bytes(),
        b"/ground/test_source.bin"
    );
    assert_eq!(
        md.get_dest_filename().as_bytes(),
        b"test/ut/output/test_dest.bin"
    );
    assert_eq!(md.get_closure_requested(), 1);
    assert_eq!(md.get_checksum_type(), ChecksumType::MODULAR);
    assert_eq!(md.get_buffer_size() as usize, bytes.len());
}

#[test]
fn metadata_empty_names_encode_but_do_not_decode() {
    let mut md = MetadataPdu::new();
    md.initialize(
        PduDirection::TowardReceiver,
        Class::Class2,
        1,
        2,
        3,
        0,
        b"",
        b"",
        ChecksumType::NULL_CHECKSUM,
        0,
    );
    let bytes = encode(&md);
    let mut rx = MetadataPdu::new();
    assert_eq!(
        rx.deserialize_from(&mut PduReader::new(&bytes)),
        SerializeStatus::DeserSizeMismatch
    );
}

#[test]
fn metadata_long_names_up_to_max_path() {
    let src = "s".repeat(200);
    let dst = "d".repeat(200);
    let bytes = md_bytes(Class::Class2, 1, 3, 2, 4096, &src, &dst);
    let md = decode_md(&bytes);
    assert_eq!(md.get_source_filename().len(), 200);
    assert_eq!(md.get_dest_filename().len(), 200);
    // A declared length over MaxFilePathSize is rejected.
    let mut over = bytes.clone();
    over[13] = 201;
    let mut rx = MetadataPdu::new();
    assert_eq!(
        rx.deserialize_from(&mut PduReader::new(&over)),
        SerializeStatus::DeserSizeMismatch
    );
}

#[test]
#[should_panic]
fn metadata_name_over_max_path_asserts() {
    let mut md = MetadataPdu::new();
    let name = [b'x'; 201];
    md.initialize(
        PduDirection::TowardReceiver,
        Class::Class2,
        1,
        2,
        3,
        0,
        &name,
        b"d",
        ChecksumType::MODULAR,
        0,
    );
}

#[test]
fn directive_decode_rejects_wrong_type_and_directive() {
    let eof = eof_bytes(Class::Class2, 1, 3, 2, ConditionCode::NO_ERROR, 0, 0);
    let mut md = MetadataPdu::new();
    assert_eq!(
        md.deserialize_from(&mut PduReader::new(&eof)),
        SerializeStatus::DeserTypeMismatch
    );
    let fd = fd_bytes(Class::Class2, 1, 3, 2, 0, &[1]);
    let mut nak = NakPdu::new();
    assert_eq!(
        nak.deserialize_from(&mut PduReader::new(&fd)),
        SerializeStatus::DeserTypeMismatch
    );
    let mut ack = AckPdu::new();
    assert_eq!(
        ack.deserialize_from(&mut PduReader::new(&eof)),
        SerializeStatus::DeserTypeMismatch
    );
    let mut fin = FinPdu::new();
    assert_eq!(
        fin.deserialize_from(&mut PduReader::new(&eof)),
        SerializeStatus::DeserTypeMismatch
    );
}

// ---- file data ------------------------------------------------------------

#[test]
fn file_data_round_trip_sizes_and_max_payload() {
    let data: Vec<u8> = (0..64u8).collect();
    let mut pdu = FileDataPdu::new();
    pdu.initialize(
        PduDirection::TowardReceiver,
        Class::Class2,
        1,
        2,
        3,
        50,
        64,
        &data,
    );
    assert_eq!(pdu.get_buffer_size(), 7 + 4 + 64);
    assert_eq!(pdu.get_max_file_data_size(), 1024 - 11);
    let mut out = [0u8; 256];
    let mut size = 0;
    assert_eq!(pdu.to_buffer(&mut out, &mut size), SerializeStatus::Ok);
    assert_eq!(size, 75);
    let rx = decode_fd(&out[..size]);
    assert_eq!(rx.get_offset(), 50);
    assert_eq!(rx.get_data_size(), 64);
    assert_eq!(rx.get_data(), &data[..]);
    assert_eq!(rx.as_header().get_type(), PduTypeEnum::FileData);
    // deserialize_from agrees with from_buffer.
    let mut rx2 = FileDataPdu::new();
    assert_eq!(
        rx2.deserialize_from(&mut PduReader::new(&out[..size])),
        SerializeStatus::Ok
    );
    assert_eq!(rx2.get_data(), &data[..]);
}

#[test]
fn file_data_empty_payload_and_large_file_flag() {
    let bytes = fd_bytes(Class::Class2, 1, 2, 3, 7, &[]);
    assert_eq!(bytes.len(), 11);
    let rx = decode_fd(&bytes);
    assert_eq!((rx.get_offset(), rx.get_data_size()), (7, 0));
    // The large-file flag widens the encoded offset to 8 bytes.
    let mut pdu = FileDataPdu::new();
    pdu.initialize(
        PduDirection::TowardReceiver,
        Class::Class2,
        1,
        2,
        3,
        0x0102_0304,
        1,
        &[0xAA],
    );
    pdu.header_mut()
        .set_large_file_flag(LargeFileFlag::LargeFile64Bit);
    let bytes = encode(&pdu);
    assert_eq!(bytes[0] & 0x01, 1);
    assert_eq!(&bytes[7..], &[0, 0, 0, 0, 0x01, 0x02, 0x03, 0x04, 0xAA]);
    assert_eq!(pdu.get_max_file_data_size(), 1024 - 15);
}

#[test]
fn file_data_from_buffer_rejects_directives_and_truncation() {
    let eof = eof_bytes(Class::Class2, 1, 3, 2, ConditionCode::NO_ERROR, 0, 0);
    let mut fd = FileDataPdu::new();
    assert_eq!(fd.from_buffer(&eof), SerializeStatus::DeserTypeMismatch);
    // Truncated inside the offset.
    let bytes = fd_bytes(Class::Class2, 1, 2, 3, 4096, &[1, 2, 3, 4]);
    let mut fd = FileDataPdu::new();
    assert_ne!(
        fd.deserialize_from(&mut PduReader::new(&bytes[..9])),
        SerializeStatus::Ok
    );
    // Data shorter than the declared PDU length.
    let mut fd = FileDataPdu::new();
    assert_eq!(
        fd.from_buffer(&bytes[..bytes.len() - 1]),
        SerializeStatus::DeserSizeMismatch
    );
    // A data-field length shorter than the offset.
    let mut short = bytes.clone();
    short[2] = 3;
    let mut fd = FileDataPdu::new();
    assert_eq!(fd.from_buffer(&short), SerializeStatus::DeserSizeMismatch);
}

// ---- EOF ------------------------------------------------------------------

#[test]
fn eof_round_trip_values_and_tlvs() {
    for (cc, checksum, size) in [
        (ConditionCode::NO_ERROR, 0, 0),
        (ConditionCode::FILE_CHECKSUM_FAILURE, 0xDEAD_BEEF, 4096),
        (ConditionCode::CANCEL_REQUEST_RECEIVED, u32::MAX, u32::MAX),
    ] {
        let bytes = eof_bytes(Class::Class2, 1, 3, 2, cc, checksum, size);
        let eof = decode_eof(&bytes);
        assert_eq!(eof.get_condition_code(), cc);
        assert_eq!(eof.get_checksum(), checksum);
        assert_eq!(eof.get_file_size(), size);
        assert_eq!(eof.get_num_tlv(), 0);
        assert_eq!(eof.get_buffer_size() as usize, bytes.len());
    }
    let mut eof = EofPdu::new();
    eof.initialize(
        PduDirection::TowardReceiver,
        Class::Class2,
        1,
        2,
        3,
        ConditionCode::FILE_SIZE_ERROR,
        1,
        2,
    );
    let mut tlv = Tlv::new();
    tlv.initialize_entity_id(0x1234_5678);
    assert!(eof.append_tlv(&tlv));
    let mut raw = Tlv::new();
    raw.initialize(TlvType::MESSAGE_TO_USER, b"hello");
    assert!(eof.append_tlv(&raw));
    let base = 7 + 1 + 1 + 4 + 4;
    assert_eq!(eof.get_buffer_size(), base + 6 + 7);
    let rx = decode_eof(&encode(&eof));
    assert_eq!(rx.get_num_tlv(), 2);
    assert_eq!(
        rx.get_tlv_list().get_tlv(0).get_data().get_entity_id(),
        0x1234_5678
    );
    let t1 = rx.get_tlv_list().get_tlv(1);
    assert_eq!(t1.get_type(), TlvType::MESSAGE_TO_USER);
    assert_eq!(&t1.get_data().get_data()[..5], b"hello");
}

// ---- FIN ------------------------------------------------------------------

#[test]
fn fin_bit_packing_all_combinations() {
    for dc in [FinDeliveryCode::Complete, FinDeliveryCode::Incomplete] {
        for fs in [
            FinFileStatus::Discarded,
            FinFileStatus::DiscardedFilestore,
            FinFileStatus::Retained,
            FinFileStatus::Unreported,
        ] {
            for cc in [
                ConditionCode::NO_ERROR,
                ConditionCode::CANCEL_REQUEST_RECEIVED,
            ] {
                let rx = decode_fin(&fin_bytes(1, 2, 3, cc, dc, fs));
                assert_eq!(
                    (
                        rx.get_condition_code(),
                        rx.get_delivery_code(),
                        rx.get_file_status()
                    ),
                    (cc, dc, fs)
                );
            }
        }
    }
}

#[test]
fn fin_with_max_tlvs_and_list_full() {
    let mut fin = FinPdu::new();
    fin.initialize(
        PduDirection::TowardSender,
        Class::Class2,
        1,
        2,
        3,
        ConditionCode::NO_ERROR,
        FinDeliveryCode::Complete,
        FinFileStatus::Retained,
    );
    for i in 0..MAX_TLV {
        let mut tlv = Tlv::new();
        tlv.initialize_entity_id(100 + i as u32);
        assert!(fin.append_tlv(&tlv));
    }
    assert!(!fin.append_tlv(&Tlv::new()));
    let rx = decode_fin(&encode(&fin));
    assert_eq!(usize::from(rx.get_num_tlv()), MAX_TLV);
    for i in 0..MAX_TLV {
        assert_eq!(
            rx.get_tlv_list()
                .get_tlv(i as u8)
                .get_data()
                .get_entity_id(),
            100 + i as u32
        );
    }
}

// ---- ACK ------------------------------------------------------------------

#[test]
fn ack_bit_packing_all_combinations() {
    for directive in [FileDirective::END_OF_FILE, FileDirective::FIN] {
        for status in [
            AckTxnStatus::UNDEFINED,
            AckTxnStatus::ACTIVE,
            AckTxnStatus::TERMINATED,
            AckTxnStatus::UNRECOGNIZED,
        ] {
            for cc in [
                ConditionCode::NO_ERROR,
                ConditionCode::FILE_CHECKSUM_FAILURE,
            ] {
                let rx = decode_ack(&ack_bytes(1, 2, 3, directive, 0, cc, status));
                assert_eq!(rx.get_directive_code(), directive);
                assert_eq!(rx.get_transaction_status(), status);
                assert_eq!(rx.get_condition_code(), cc);
                assert_eq!(rx.get_directive_subtype_code(), 0);
            }
        }
    }
    let rx = decode_ack(&ack_bytes(
        1,
        2,
        3,
        FileDirective::FIN,
        0x0F,
        ConditionCode::NO_ERROR,
        AckTxnStatus::ACTIVE,
    ));
    assert_eq!(rx.get_directive_subtype_code(), 0x0F);
    assert_eq!(rx.get_buffer_size(), 7 + 3);
}

// ---- NAK ------------------------------------------------------------------

#[test]
fn nak_round_trip_segments_and_sizes() {
    let segs: Vec<SegmentRequest> = (0..10)
        .map(|i| SegmentRequest {
            offset_start: i * 100,
            offset_end: i * 100 + 50,
        })
        .collect();
    let bytes = nak_bytes(1, 2, 3, (0, 1000), &segs);
    assert_eq!(bytes.len(), 7 + 1 + 8 + 10 * 8);
    let rx = decode_nak(&bytes);
    assert_eq!((rx.get_scope_start(), rx.get_scope_end()), (0, 1000));
    assert_eq!(rx.get_num_segments(), 10);
    for (i, s) in segs.iter().enumerate() {
        assert_eq!(rx.get_segment(i as u8), s);
    }
    // No segments.
    let rx = decode_nak(&nak_bytes(1, 2, 3, (5, 6), &[]));
    assert_eq!(rx.get_num_segments(), 0);
}

#[test]
fn nak_add_segment_full_clear_and_max_decode() {
    let mut nak = NakPdu::new();
    nak.initialize(PduDirection::TowardSender, Class::Class2, 1, 2, 3, 0, 0);
    for i in 0..NAK_MAX_SEGMENTS as u32 {
        assert!(nak.add_segment(i, i + 1));
    }
    assert!(!nak.add_segment(0, 1));
    let bytes = encode(&nak);
    assert_eq!(
        usize::from(decode_nak(&bytes).get_num_segments()),
        NAK_MAX_SEGMENTS
    );
    nak.clear_segments();
    assert_eq!(nak.get_num_segments(), 0);
    // Extra segment bytes beyond NakMaxSegments are ignored on decode.
    let mut long = bytes.clone();
    long.extend_from_slice(&[0, 0, 0, 1, 0, 0, 0, 2]);
    assert_eq!(
        usize::from(decode_nak(&long).get_num_segments()),
        NAK_MAX_SEGMENTS
    );
    // A partial trailing segment is not counted.
    let mut partial = nak_bytes(
        1,
        2,
        3,
        (0, 9),
        &[SegmentRequest {
            offset_start: 1,
            offset_end: 2,
        }],
    );
    partial.extend_from_slice(&[0, 0, 0]);
    assert_eq!(decode_nak(&partial).get_num_segments(), 1);
}

#[test]
fn nak_segment_count_truncates_to_u8_before_the_clamp() {
    // 300 whole segments: (300 as u8) = 44, below the clamp (C++ parity).
    let mut bytes = nak_bytes(1, 2, 3, (0, 0), &[]);
    bytes.extend(std::iter::repeat_n(0u8, 300 * 8));
    assert_eq!(decode_nak(&bytes).get_num_segments(), 44);
}

// ---- TLVs -----------------------------------------------------------------

#[test]
fn tlv_entity_id_and_raw_round_trips() {
    let mut tlv = Tlv::new();
    tlv.initialize_entity_id(0xABCD);
    assert_eq!(tlv.get_type(), TlvType::ENTITY_ID);
    assert_eq!(tlv.get_encoded_size(), 2 + 4);
    let mut out = [0u8; 300];
    let mut sb = ExtBuf::new(&mut out);
    assert_eq!(tlv.to_serial_buffer(&mut sb), SerializeStatus::Ok);
    let len = sb.ser_loc();
    assert_eq!(&out[..len], &[6, 4, 0, 0, 0xAB, 0xCD]);
    let mut rx = Tlv::new();
    assert_eq!(
        rx.from_serial_buffer(&mut PduReader::new(&out[..len])),
        SerializeStatus::Ok
    );
    assert_eq!(rx.get_data().get_entity_id(), 0xABCD);
    // A short EID decodes and re-encodes at full width.
    let mut rx = Tlv::new();
    assert_eq!(
        rx.from_serial_buffer(&mut PduReader::new(&[6, 2, 0x12, 0x34])),
        SerializeStatus::Ok
    );
    assert_eq!(rx.get_data().get_entity_id(), 0x1234);
    assert_eq!(rx.get_encoded_size(), 6);
    // Raw data, up to 255 bytes.
    let data = [0x5Au8; 255];
    let mut raw = Tlv::new();
    raw.initialize(TlvType::FLOW_LABEL, &data);
    let mut sb = ExtBuf::new(&mut out);
    assert_eq!(raw.to_serial_buffer(&mut sb), SerializeStatus::Ok);
    let len = sb.ser_loc();
    assert_eq!(len, 257);
    let mut rx = Tlv::new();
    assert_eq!(
        rx.from_serial_buffer(&mut PduReader::new(&out[..len])),
        SerializeStatus::Ok
    );
    assert_eq!(rx.get_data().get_length(), 255);
    assert_eq!(&rx.get_data().get_data()[..255], &data[..]);
}

#[test]
fn tlv_entity_id_too_long_is_a_format_error() {
    let mut rx = Tlv::new();
    assert_eq!(
        rx.from_serial_buffer(&mut PduReader::new(&[6, 5, 0, 0, 0, 0, 0, 0])),
        SerializeStatus::DeserFormatError
    );
}

#[test]
fn tlv_list_append_clear_and_partial_decode() {
    let mut list = TlvList::new();
    for i in 0..MAX_TLV as u32 {
        let mut tlv = Tlv::new();
        tlv.initialize_entity_id(100 + i);
        assert!(list.append_tlv(&tlv));
    }
    assert!(!list.append_tlv(&Tlv::new()));
    assert_eq!(list.get_encoded_size(), MAX_TLV as u32 * 6);
    list.clear();
    assert_eq!(list.get_num_tlv(), 0);
    // A bad first TLV fails the list; a bad later one ends it.
    let mut rx = TlvList::new();
    assert_eq!(
        rx.from_serial_buffer(&mut PduReader::new(&[6, 9])),
        SerializeStatus::DeserFormatError
    );
    let mut rx = TlvList::new();
    assert_eq!(
        rx.from_serial_buffer(&mut PduReader::new(&[6, 1, 7, 6, 9])),
        SerializeStatus::Ok
    );
    assert_eq!(rx.get_num_tlv(), 1);
    assert_eq!(rx.get_tlv(0).get_data().get_entity_id(), 7);
}

#[test]
fn header_pdu_type_bits() {
    let fd = fd_bytes(Class::Class2, 1, 2, 3, 0, &[1]);
    let mut h = PduHeader::default();
    assert_eq!(
        h.from_serial_buffer(&mut PduReader::new(&fd)),
        SerializeStatus::Ok
    );
    assert_eq!(h.get_type(), PduTypeEnum::FileData);
    assert_eq!(h.pdu_type, PduType::FileData);
}

// ---- engine-driven PDUs (PduTester.cpp) -----------------------------------

/// Set up an S/R transaction directly (upstream `setupTestTransaction`).
#[allow(clippy::too_many_arguments)]
fn setup_txn(
    h: &Harness,
    state: TxnState,
    ch: u8,
    src: &str,
    dst: &str,
    fsize: FileSize,
    seq: TransactionSeq,
    peer: EntityId,
) -> TxnIdx {
    h.chan(ch, |c| {
        let dir = if matches!(state, TxnState::S1 | TxnState::S2) {
            Direction::Tx
        } else {
            Direction::Rx
        };
        let t = c.find_unused_transaction(dir).expect("free transaction");
        let txn = c.txn_mut(t);
        txn.state = state;
        txn.fsize = fsize;
        txn.txn_class = if matches!(state, TxnState::S1 | TxnState::R1) {
            Class::Class1
        } else {
            Class::Class2
        };
        let hist = c.hist_mut(t);
        hist.seq_num = seq;
        hist.peer_eid = peer;
        hist.src_eid = 42;
        hist.fnames.src_filename.set(src);
        hist.fnames.dst_filename.set(dst);
        t
    })
}

#[test]
fn engine_send_md_emits_the_metadata_pdu() {
    let h = Harness::new();
    let t = setup_txn(
        &h,
        TxnState::S1,
        CH0,
        "/src/a.bin",
        "/dst/b.bin",
        1234,
        7,
        100,
    );
    let mgr = h.comp.clone();
    assert_eq!(h.chan(CH0, |c| c.send_md(&mgr, t)), Status::Success);
    let sent = h.sent();
    assert_eq!(sent.len(), 1);
    let md = decode_md(&sent[0]);
    assert_eq!(md.as_header().get_source_eid(), 42);
    assert_eq!(md.as_header().get_dest_eid(), 100);
    assert_eq!(md.as_header().get_transaction_seq(), 7);
    assert_eq!(md.as_header().get_txm_mode(), Class::Class1);
    assert_eq!(md.get_file_size(), 1234);
    assert_eq!(md.get_closure_requested(), 0);
    assert_eq!(md.get_source_filename().as_bytes(), b"/src/a.bin");
    assert_eq!(md.get_dest_filename().as_bytes(), b"/dst/b.bin");
    assert_eq!(h.chan(CH0, |c| c.tlm.sent_pdu), 1);
}

#[test]
fn engine_send_eof_fin_ack_nak_emit_their_pdus() {
    let h = Harness::new();
    let mgr = h.comp.clone();
    let s = setup_txn(&h, TxnState::S2, CH0, "a", "b", 4096, 9, 100);
    h.chan(CH0, |c| {
        c.txn_mut(s).crc = Checksum::from_value(0xCAFE_F00D);
        assert_eq!(c.send_eof(&mgr, s), Status::Success);
    });
    let eof = decode_eof(&h.sent()[0]);
    assert_eq!(eof.get_checksum(), 0xCAFE_F00D);
    assert_eq!(eof.get_file_size(), 4096);
    assert_eq!(eof.get_condition_code(), ConditionCode::NO_ERROR);
    assert_eq!(eof.get_num_tlv(), 0);

    // An error status adds the entity-ID TLV and counts the fault.
    h.chan(CH0, |c| {
        c.set_txn_status(s, TxnStatus::FILE_CHECKSUM_FAILURE);
        assert_eq!(c.send_eof(&mgr, s), Status::Success);
        assert_eq!(c.tlm.fault_tx_eof_error, 1);
    });
    let eof = decode_eof(&h.sent()[1]);
    assert_eq!(
        eof.get_condition_code(),
        ConditionCode::FILE_CHECKSUM_FAILURE
    );
    assert_eq!(eof.get_tlv_list().get_tlv(0).get_data().get_entity_id(), 42);

    // FIN from a receive transaction: peer -> local, toward the sender.
    let r = setup_txn(&h, TxnState::R2, CH0, "x", "y", 10, 11, 100);
    h.chan(CH0, |c| {
        assert_eq!(
            c.send_fin(
                &mgr,
                r,
                FinDeliveryCode::Complete,
                FinFileStatus::Retained,
                ConditionCode::NO_ERROR
            ),
            Status::Success
        );
    });
    let fin = decode_fin(&h.sent()[2]);
    assert_eq!(fin.as_header().get_source_eid(), 100);
    assert_eq!(fin.as_header().get_dest_eid(), 42);
    assert_eq!(fin.as_header().get_direction(), PduDirection::TowardSender);
    assert_eq!(fin.get_file_status(), FinFileStatus::Retained);

    // ACK(EOF) from the receive transaction.
    h.chan(CH0, |c| {
        assert_eq!(
            c.send_ack(
                &mgr,
                r,
                AckTxnStatus::ACTIVE,
                FileDirective::END_OF_FILE,
                ConditionCode::NO_ERROR,
                100,
                11
            ),
            Status::Success
        );
    });
    let ack = decode_ack(&h.sent()[3]);
    assert_eq!(ack.as_header().get_source_eid(), 100);
    assert_eq!(ack.as_header().get_dest_eid(), 42);
    assert_eq!(ack.as_header().get_direction(), PduDirection::TowardSender);
    assert_eq!(ack.get_directive_code(), FileDirective::END_OF_FILE);
    assert_eq!(ack.get_directive_subtype_code(), 1);

    // NAK.
    let mut nak = NakPdu::new();
    nak.initialize(
        PduDirection::TowardSender,
        Class::Class2,
        100,
        11,
        42,
        0,
        10,
    );
    assert!(nak.add_segment(2, 5));
    h.chan(CH0, |c| {
        assert_eq!(c.send_nak(&mgr, r, &nak), Status::Success)
    });
    let rx = decode_nak(&h.sent()[4]);
    assert_eq!(rx.get_num_segments(), 1);
    assert_eq!(rx.get_segment(0).offset_end, 5);
}

#[test]
fn engine_send_fd_counts_data_bytes() {
    let h = Harness::new();
    let mgr = h.comp.clone();
    let t = setup_txn(&h, TxnState::S1, CH1, "a", "b", 100, 3, 100);
    let data = [7u8; 64];
    let mut fd = FileDataPdu::new();
    fd.initialize(
        PduDirection::TowardReceiver,
        Class::Class1,
        42,
        3,
        100,
        50,
        64,
        &data,
    );
    h.chan(CH1, |c| {
        assert_eq!(c.send_fd(&mgr, &fd), Status::Success);
        assert_eq!(c.tlm.sent_file_data_bytes, 64);
    });
    let (port, bytes) = h.ground.data_out.lock()[0].clone();
    assert_eq!(port, 1, "dataOut port = channel");
    let rx = decode_fd(&bytes[2..]);
    assert_eq!((rx.get_offset(), rx.get_data()), (50, &data[..]));
    let _ = t;
}
