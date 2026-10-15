//! CFDP utility functions — port of `Svc/Ccsds/CfdpManager/Utils.{hpp,cpp}`,
//! derived from NASA CF 3.0.0 `cf_utils.{h,c}`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0.
//!
//! The list-traversal callbacks (`FindTransactionBySequenceNumberImpl`,
//! `PrioSearch`) are closures in `channel.rs`.

use super::types::{AckTxnStatus, ConditionCode, TxnState, TxnStatus};

/// C++ `GetTxnStatus`: the ACK transaction status for a transaction in
/// `state` (`None` is the null transaction: `UNRECOGNIZED`).
#[must_use]
pub fn get_txn_status(state: Option<TxnState>) -> AckTxnStatus {
    match state {
        None => AckTxnStatus::UNRECOGNIZED,
        Some(TxnState::S1 | TxnState::R1 | TxnState::S2 | TxnState::R2) => AckTxnStatus::ACTIVE,
        Some(TxnState::Drop | TxnState::Hold) => AckTxnStatus::TERMINATED,
        Some(_) => AckTxnStatus::INVALID,
    }
}

/// C++ `TxnStatusIsError`: anything above `NO_ERROR` (`UNDEFINED` is an
/// in-progress transaction, not an error).
#[must_use]
pub fn txn_status_is_error(txn_stat: TxnStatus) -> bool {
    txn_stat > TxnStatus::NO_ERROR
}

/// C++ `TxnStatusToConditionCode`: the condition code to report for a
/// transaction status.
#[must_use]
pub fn txn_status_to_condition_code(txn_stat: TxnStatus) -> ConditionCode {
    if !txn_status_is_error(txn_stat) {
        return ConditionCode::NO_ERROR;
    }
    match txn_stat {
        TxnStatus::NO_ERROR
        | TxnStatus::POS_ACK_LIMIT_REACHED
        | TxnStatus::KEEP_ALIVE_LIMIT_REACHED
        | TxnStatus::INVALID_TRANSMISSION_MODE
        | TxnStatus::FILESTORE_REJECTION
        | TxnStatus::FILE_CHECKSUM_FAILURE
        | TxnStatus::FILE_SIZE_ERROR
        | TxnStatus::NAK_LIMIT_REACHED
        | TxnStatus::INACTIVITY_DETECTED
        | TxnStatus::INVALID_FILE_STRUCTURE
        | TxnStatus::CHECK_LIMIT_REACHED
        | TxnStatus::UNSUPPORTED_CHECKSUM_TYPE
        | TxnStatus::SUSPEND_REQUEST_RECEIVED
        | TxnStatus::CANCEL_REQUEST_RECEIVED => ConditionCode(txn_stat.0 as u8),
        TxnStatus::ACK_LIMIT_NO_FIN | TxnStatus::ACK_LIMIT_NO_EOF => {
            ConditionCode::INACTIVITY_DETECTED
        }
        _ => ConditionCode::CANCEL_REQUEST_RECEIVED,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn txn_status_is_error_only_above_no_error() {
        assert!(!txn_status_is_error(TxnStatus::UNDEFINED));
        assert!(!txn_status_is_error(TxnStatus::NO_ERROR));
        assert!(txn_status_is_error(TxnStatus::POS_ACK_LIMIT_REACHED));
        assert!(txn_status_is_error(TxnStatus::EARLY_FIN));
    }

    #[test]
    fn condition_codes_map_directly_extended_codes_are_translated() {
        assert_eq!(
            txn_status_to_condition_code(TxnStatus::UNDEFINED),
            ConditionCode::NO_ERROR
        );
        assert_eq!(
            txn_status_to_condition_code(TxnStatus::NO_ERROR),
            ConditionCode::NO_ERROR
        );
        for code in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15] {
            assert_eq!(
                txn_status_to_condition_code(TxnStatus(code)),
                ConditionCode(code as u8)
            );
        }
        assert_eq!(
            txn_status_to_condition_code(TxnStatus::ACK_LIMIT_NO_FIN),
            ConditionCode::INACTIVITY_DETECTED
        );
        assert_eq!(
            txn_status_to_condition_code(TxnStatus::ACK_LIMIT_NO_EOF),
            ConditionCode::INACTIVITY_DETECTED
        );
        // Undeclared and other extended statuses fall to CANCEL.
        for code in [12, 13, 16, 19, 20, 21, 22] {
            assert_eq!(
                txn_status_to_condition_code(TxnStatus(code)),
                ConditionCode::CANCEL_REQUEST_RECEIVED
            );
        }
    }

    #[test]
    fn get_txn_status_by_state() {
        assert_eq!(get_txn_status(None), AckTxnStatus::UNRECOGNIZED);
        for s in [TxnState::S1, TxnState::R1, TxnState::S2, TxnState::R2] {
            assert_eq!(get_txn_status(Some(s)), AckTxnStatus::ACTIVE);
        }
        for s in [TxnState::Drop, TxnState::Hold] {
            assert_eq!(get_txn_status(Some(s)), AckTxnStatus::TERMINATED);
        }
        for s in [TxnState::Undef, TxnState::Init, TxnState::Invalid] {
            assert_eq!(get_txn_status(Some(s)), AckTxnStatus::INVALID);
        }
    }
}
