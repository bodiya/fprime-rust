#!/usr/bin/env python3
"""Cross-check the CFDP PDU byte vectors of the Rust CfdpManager port
against `spacepackets` (the CCSDS packet library fprime-gds 4.3.1 installs;
fprime-gds itself ships no CFDP engine).

Each vector below is the literal expected by
`crates/fprime-svc/src/ccsds/cfdp_manager/tests/pdus.rs`. The script encodes
the same PDU with spacepackets and compares. Two vectors differ on purpose:
upstream F' (and so the port) puts the metadata closure-requested flag in
the MSB of its octet and always sends ACK directive subtype 1, where
CCSDS 727.0-B-5 and spacepackets use bit 6 and subtype 0 for ACK(EOF).

    ~/.venvs/fprime-gds/bin/python -I tools/cfdp-pdu-vectors.py
"""

import sys

from spacepackets.cfdp import (
    ChecksumType,
    ConditionCode,
    DeliveryCode,
    Direction,
    EntityIdTlv,
    FileStatus,
    PduConfig,
    TransmissionMode,
)
from spacepackets.cfdp.pdu import (
    AckPdu,
    DirectiveType,
    EofPdu,
    FileDataParams,
    FileDataPdu,
    FinishedParams,
    FinishedPdu,
    MetadataParams,
    MetadataPdu,
    NakPdu,
    TransactionStatus,
)
from spacepackets.util import ByteFieldGenerator

C1 = TransmissionMode.UNACKNOWLEDGED
C2 = TransmissionMode.ACKNOWLEDGED
TO_SENDER = Direction.TOWARDS_SENDER
TO_RECEIVER = Direction.TOWARDS_RECEIVER


def conf(src, dst, seq, mode, direction=TO_RECEIVER, eid_len=1, seq_len=1):
    return PduConfig(
        source_entity_id=ByteFieldGenerator.from_int(eid_len, src),
        dest_entity_id=ByteFieldGenerator.from_int(eid_len, dst),
        transaction_seq_num=ByteFieldGenerator.from_int(seq_len, seq),
        trans_mode=mode,
        direction=direction,
    )


EID_42 = EntityIdTlv(bytes([0, 0, 0, 42]))

# name -> (spacepackets PDU, Rust literal, documented difference)
VECTORS = {
    "metadata": (
        MetadataPdu(conf(1, 3, 2, C2), MetadataParams(True, ChecksumType.MODULAR, 1024, "src.txt", "dst.txt")),
        "20 00 16 00 01 02 03 07 80 00 00 04 00 07 73 72 63 2e 74 78 74 07 64 73 74 2e 74 78 74",
        "closure-requested flag: F' MSB (0x80), standard bit 6 (0x40)",
    ),
    "file_data": (
        FileDataPdu(conf(10, 30, 20, C1), FileDataParams(bytes([1, 2, 3, 4, 5]), 50)),
        "34 00 09 00 0a 14 1e 00 00 00 32 01 02 03 04 05",
        None,
    ),
    "eof": (
        EofPdu(conf(1, 3, 2, C2), 0x12345678, 4096),
        "20 00 0a 00 01 02 03 04 00 12 34 56 78 00 00 10 00",
        None,
    ),
    "eof_cancel": (
        EofPdu(conf(1, 3, 2, C2), 0x12345678, 100, fault_location=EID_42,
               condition_code=ConditionCode.CANCEL_REQUEST_RECEIVED),
        "20 00 10 00 01 02 03 04 f0 12 34 56 78 00 00 00 64 06 04 00 00 00 2a",
        None,
    ),
    "eof_wide": (
        EofPdu(conf(0x1234, 0x56, 0x01000000, C2, eid_len=2, seq_len=4), 0xDEADBEEF, 70000),
        "20 00 0a 13 12 34 01 00 00 00 00 56 04 00 de ad be ef 00 01 11 70",
        None,
    ),
    "fin": (
        FinishedPdu(conf(100, 42, 7, C2, TO_SENDER),
                    FinishedParams(ConditionCode.NO_ERROR, DeliveryCode.DATA_COMPLETE, FileStatus.FILE_RETAINED)),
        "28 00 02 00 64 07 2a 05 02",
        None,
    ),
    "fin_error": (
        FinishedPdu(conf(100, 42, 7, C2, TO_SENDER),
                    FinishedParams(ConditionCode.FILE_CHECKSUM_FAILURE, DeliveryCode.DATA_INCOMPLETE,
                                   FileStatus.DISCARDED_DELIBERATELY, fault_location=EID_42)),
        "28 00 08 00 64 07 2a 05 54 06 04 00 00 00 2a",
        None,
    ),
    "ack_fin": (
        AckPdu(conf(42, 100, 7, C2, TO_RECEIVER), DirectiveType.FINISHED_PDU, ConditionCode.NO_ERROR,
               TransactionStatus.TERMINATED),
        "20 00 03 00 2a 07 64 06 51 02",
        None,
    ),
    "ack_eof": (
        AckPdu(conf(100, 42, 7, C2, TO_SENDER), DirectiveType.EOF_PDU, ConditionCode.NO_ERROR,
               TransactionStatus.ACTIVE),
        "28 00 03 00 64 07 2a 06 41 01",
        "ACK(EOF) directive subtype: F' 1, standard 0",
    ),
    "nak": (
        NakPdu(conf(100, 42, 7, C2, TO_SENDER), 0, 5000, [(992, 1984), (3968, 4960)]),
        "28 00 19 00 64 07 2a 08 00 00 00 00 00 00 13 88 00 00 03 e0 00 00 07 c0"
        " 00 00 0f 80 00 00 13 60",
        None,
    ),
}


def main() -> int:
    failures = 0
    for name, (pdu, rust_hex, difference) in VECTORS.items():
        theirs = bytes(pdu.pack())
        ours = bytes.fromhex(rust_hex)
        if theirs == ours:
            status = "match" if difference is None else "UNEXPECTED MATCH"
            failures += difference is not None
        elif difference is not None and len(theirs) == len(ours) and \
                sum(a != b for a, b in zip(theirs, ours)) == 1:
            status = f"differs as documented ({difference})"
        else:
            status = f"MISMATCH\n  spacepackets {theirs.hex(' ')}\n  rust         {ours.hex(' ')}"
            failures += 1
        print(f"{name:12} {status}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
