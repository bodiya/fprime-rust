//! CFDP data types — port of `Svc/Ccsds/CfdpManager/Types/Types.fpp` (the
//! FPP enums and structs) and `Types/Types.hpp` (the protocol enums and the
//! transaction bookkeeping structures, from CF's `cf_cfdp_types.h`).
//!
//! The protocol enums whose values come off the wire (`FileDirective`,
//! `ConditionCode`, `AckTxnStatus`, `ChecksumType`) and `TxnStatus` (which
//! is assigned from wire condition codes) are newtypes with associated
//! constants rather than Rust enums: the C++ `enum class` holds any value
//! of its underlying type, and upstream stores undeclared values from
//! received PDUs without validation.

use super::chunk::ChunkList;
use super::timer::Timer;
pub use fprime_config::cfdp::{EntityId, FileSize, TransactionSeq};
use fprime_config::cfdp::{
    MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN, MAX_COMMANDED_PLAYBACK_FILES_PER_CHAN,
    MAX_FILE_PATH_SIZE, MAX_POLLING_DIR_PER_CHAN, MAX_SIMULTANEOUS_RX, NUM_CHANNELS,
    NUM_TRANSACTIONS_PER_PLAYBACK,
};
use fprime_fw::{Enabled, FwDefaultString, FwString, fpp_array, fpp_enum, fpp_struct};
use fprime_os::directory::Directory;

/// `string size MaxFilePathSize`: command, event and PDU file names.
pub type PathString = FwString<MAX_FILE_PATH_SIZE>;

/// C++ `CFDP_NUM_TRANSACTIONS_PER_CHANNEL`: the transaction pool size of
/// one channel.
pub const NUM_TRANSACTIONS_PER_CHANNEL: usize = MAX_COMMANDED_PLAYBACK_FILES_PER_CHAN
    + MAX_SIMULTANEOUS_RX
    + (MAX_POLLING_DIR_PER_CHAN + MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN)
        * NUM_TRANSACTIONS_PER_PLAYBACK;

// ---------------------------------------------------------------------------
// FPP types (Types.fpp)
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `Svc.Ccsds.Cfdp.Status` (I32 representation).
    pub enum Status : i32 {
        /// The operation succeeded.
        Success = 0,
        /// Generic error.
        Error = 1,
        /// Invalid metadata PDU.
        PduMetadataError = 2,
        /// PDU too short.
        ShortPduError = 3,
        /// Received EOF file size mismatch.
        RecPduFsizeMismatchError = 4,
        /// Received an invalid EOF.
        RecPduBadEofError = 5,
        /// No send buffer available (throttle limit reached).
        SendPduNoBufAvailError = 6,
    }
    default Success
}

fpp_enum! {
    /// `Svc.Ccsds.Cfdp.PduTypeEnum`.
    pub enum PduTypeEnum : u8 {
        /// Metadata PDU.
        Metadata = 0,
        /// End of File PDU.
        EndOfFile = 1,
        /// Finished PDU.
        Finished = 2,
        /// Acknowledgment PDU.
        Acknowledgment = 3,
        /// Negative Acknowledgment PDU.
        NegativeAck = 4,
        /// File Data PDU.
        FileData = 5,
        /// Invalid or unknown PDU type.
        None = 255,
    }
    default Metadata
}

fpp_enum! {
    /// `Svc.Ccsds.Cfdp.Flow` (I32 representation).
    pub enum Flow : i32 {
        /// Channel operations execute nominally.
        NotFrozen = 0,
        /// Channel operations are frozen.
        Frozen = 1,
    }
    default NotFrozen
}

fpp_enum! {
    /// `Svc.Ccsds.Cfdp.Class` (CCSDS 727.0-B-5 section 7.1). The values are
    /// the header's transmission-mode bit.
    pub enum Class : u8 {
        /// Class 2: reliable (acknowledged).
        Class2 = 0,
        /// Class 1: unreliable (unacknowledged).
        Class1 = 1,
    }
    default Class2
}

fpp_enum! {
    /// `Svc.Ccsds.Cfdp.Keep`.
    pub enum Keep : u8 {
        /// Delete the file after the transaction.
        Delete = 0,
        /// Keep the file.
        Keep = 1,
    }
    default Delete
}

fpp_enum! {
    /// `Svc.Ccsds.Cfdp.SuspendResume`.
    pub enum SuspendResume : u8 {
        /// Suspend the transaction.
        Suspend = 0,
        /// Resume the transaction.
        Resume = 1,
    }
    default Suspend
}

fpp_enum! {
    /// `Svc.Ccsds.Cfdp.QueueId`: a channel's transaction and history queues.
    pub enum QueueId : u8 {
        /// Pending transmit transactions.
        Pend = 0,
        /// Active transmit transactions.
        Txa = 1,
        /// Transmit transactions waiting on the peer.
        Txw = 2,
        /// Receive transactions.
        Rx = 3,
        /// Completed-transaction history.
        Hist = 4,
        /// Free history entries.
        HistFree = 5,
        /// Free transactions.
        Free = 6,
        /// Number of queues.
        Num = 7,
    }
    default Pend
}

fpp_struct! {
    /// `Svc.Ccsds.Cfdp.ChannelParams`: one channel's configuration.
    #[derive(Clone)]
    pub struct ChannelParams {
        /// ACK retries (e.g. FIN until FIN-ACK).
        ack_limit: u8,
        /// NAK retries before giving up.
        nack_limit: u8,
        /// ACK timer, seconds.
        ack_timer: u32,
        /// Inactivity timer, seconds.
        inactivity_timer: u32,
        /// Whether pending transactions are made active.
        dequeue_enabled: Enabled,
        /// Where sent files are moved, if not empty.
        move_dir: PathString,
        /// Outgoing PDUs per cycle (throttle).
        max_outgoing_pdus_per_cycle: u32,
        /// Temporary directory for uplink files received without metadata.
        tmp_dir: PathString,
        /// Directory for failed poll files.
        fail_dir: PathString,
    }
}

fpp_array! {
    /// `Svc.Ccsds.Cfdp.ChannelArrayParams`.
    #[derive(Clone)]
    pub array ChannelArrayParams = [ChannelParams; NUM_CHANNELS]
}

impl ChannelParams {
    /// The per-channel default of the `ChannelConfig` parameter.
    #[must_use]
    pub fn fpp_default() -> Self {
        Self {
            ack_limit: 10,
            nack_limit: 10,
            ack_timer: 2,
            inactivity_timer: 120,
            dequeue_enabled: Enabled::Enabled,
            move_dir: PathString::from(""),
            max_outgoing_pdus_per_cycle: 64,
            tmp_dir: PathString::from("/tmp"),
            fail_dir: PathString::from("/fail"),
        }
    }
}

impl ChannelArrayParams {
    /// The FPP default of the `ChannelConfig` parameter.
    #[must_use]
    pub fn fpp_default() -> Self {
        Self(core::array::from_fn(|_| ChannelParams::fpp_default()))
    }
}

fpp_struct! {
    /// `Svc.Ccsds.Cfdp.ChannelTelemetry`: one channel's counters.
    #[derive(Clone, Copy, Eq)]
    pub struct ChannelTelemetry {
        /// PDU receive errors.
        recv_errors: u32,
        /// PDUs dropped for lack of resources.
        recv_dropped: u32,
        /// Spurious PDUs.
        recv_spurious: u32,
        /// File data bytes received.
        recv_file_data_bytes: u64,
        /// NAK segment requests received.
        recv_nak_segment_requests: u32,
        /// PDUs received with valid headers.
        recv_pdu: u32,
        /// EOFs received with the cancel condition code.
        recv_eof_canceled: u32,
        /// NAK segment requests sent.
        sent_nak_segment_requests: u32,
        /// File data bytes sent.
        sent_file_data_bytes: u64,
        /// PDUs sent.
        sent_pdu: u32,
        /// EOFs sent with the cancel condition code.
        sent_eof_canceled: u32,
        /// Transactions abandoned at the ACK limit.
        fault_ack_limit: u32,
        /// Transactions abandoned at the NAK limit.
        fault_nak_limit: u32,
        /// Inactivity timeouts.
        fault_inactivity_timer: u32,
        /// CRC mismatches.
        fault_crc_mismatch: u32,
        /// File size mismatches.
        fault_file_size_mismatch: u32,
        /// File open failures.
        fault_file_open: u32,
        /// File read failures.
        fault_file_read: u32,
        /// File write failures.
        fault_file_write: u32,
        /// File seek failures.
        fault_file_seek: u32,
        /// File rename failures.
        fault_file_rename: u32,
        /// Directory read failures.
        fault_directory_read: u32,
        /// EOFs received with an error (non-cancel) condition code.
        fault_rx_eof_error: u32,
        /// EOFs sent with an error (non-cancel) condition code.
        fault_tx_eof_error: u32,
        /// Transactions on the FREE queue.
        queue_free: u16,
        /// Transactions on the TXA queue.
        queue_tx_active: u16,
        /// Transactions on the TXW queue.
        queue_tx_waiting: u16,
        /// Transactions on the RX queue.
        queue_rx: u16,
        /// Entries on the history queue.
        queue_history: u16,
        /// Active directory playbacks.
        playback_counter: u8,
        /// Active directory polls.
        poll_counter: u8,
    }
}

fpp_array! {
    /// `Svc.Ccsds.Cfdp.ChannelTelemetryArray`.
    #[derive(Clone, Copy, Eq)]
    pub array ChannelTelemetryArray = [ChannelTelemetry; NUM_CHANNELS]
}

// ---------------------------------------------------------------------------
// Protocol enums (Types.hpp)
// ---------------------------------------------------------------------------

/// C++ `FileDirective` (Blue Book table 5-4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct FileDirective(pub u8);

impl FileDirective {
    /// Lower range marker.
    pub const INVALID_MIN: Self = Self(0);
    /// End of File.
    pub const END_OF_FILE: Self = Self(4);
    /// Finished.
    pub const FIN: Self = Self(5);
    /// Acknowledge.
    pub const ACK: Self = Self(6);
    /// Metadata.
    pub const METADATA: Self = Self(7);
    /// Negative Acknowledge.
    pub const NAK: Self = Self(8);
    /// Prompt.
    pub const PROMPT: Self = Self(9);
    /// Keep Alive.
    pub const KEEP_ALIVE: Self = Self(12);
    /// Upper range marker (one past the last directive).
    pub const INVALID_MAX: Self = Self(13);
}

/// C++ `ConditionCode` (Blue Book table 5-5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ConditionCode(pub u8);

impl ConditionCode {
    /// No error.
    pub const NO_ERROR: Self = Self(0);
    /// Positive ACK limit reached.
    pub const POS_ACK_LIMIT_REACHED: Self = Self(1);
    /// Keep-alive limit reached.
    pub const KEEP_ALIVE_LIMIT_REACHED: Self = Self(2);
    /// Invalid transmission mode.
    pub const INVALID_TRANSMISSION_MODE: Self = Self(3);
    /// Filestore rejection.
    pub const FILESTORE_REJECTION: Self = Self(4);
    /// File checksum failure.
    pub const FILE_CHECKSUM_FAILURE: Self = Self(5);
    /// File size error.
    pub const FILE_SIZE_ERROR: Self = Self(6);
    /// NAK limit reached.
    pub const NAK_LIMIT_REACHED: Self = Self(7);
    /// Inactivity detected.
    pub const INACTIVITY_DETECTED: Self = Self(8);
    /// Invalid file structure.
    pub const INVALID_FILE_STRUCTURE: Self = Self(9);
    /// Check limit reached.
    pub const CHECK_LIMIT_REACHED: Self = Self(10);
    /// Unsupported checksum type.
    pub const UNSUPPORTED_CHECKSUM_TYPE: Self = Self(11);
    /// Suspend request received.
    pub const SUSPEND_REQUEST_RECEIVED: Self = Self(14);
    /// Cancel request received.
    pub const CANCEL_REQUEST_RECEIVED: Self = Self(15);
}

/// C++ `AckTxnStatus` (Blue Book table 5-8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AckTxnStatus(pub u8);

impl AckTxnStatus {
    /// Undefined.
    pub const UNDEFINED: Self = Self(0);
    /// Active.
    pub const ACTIVE: Self = Self(1);
    /// Terminated.
    pub const TERMINATED: Self = Self(2);
    /// Unrecognized.
    pub const UNRECOGNIZED: Self = Self(3);
    /// Invalid (not encodable in the 2-bit field).
    pub const INVALID: Self = Self(4);
}

/// C++ `FinDeliveryCode` (Blue Book table 5-7).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FinDeliveryCode {
    /// Data complete.
    #[default]
    Complete = 0,
    /// Data incomplete.
    Incomplete = 1,
}

impl FinDeliveryCode {
    /// From the 1-bit wire field.
    #[must_use]
    pub fn from_bit(bit: u8) -> Self {
        if bit & 0x01 == 1 {
            Self::Incomplete
        } else {
            Self::Complete
        }
    }
}

/// C++ `FinFileStatus` (Blue Book table 5-7).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FinFileStatus {
    /// File discarded deliberately.
    #[default]
    Discarded = 0,
    /// File discarded by filestore rejection.
    DiscardedFilestore = 1,
    /// File retained.
    Retained = 2,
    /// File status unreported.
    Unreported = 3,
}

impl FinFileStatus {
    /// From the 2-bit wire field (all four values are declared).
    #[must_use]
    pub fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::Discarded,
            1 => Self::DiscardedFilestore,
            2 => Self::Retained,
            _ => Self::Unreported,
        }
    }
}

/// C++ `ChecksumType` (Blue Book table 5-9).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct ChecksumType(pub u8);

impl ChecksumType {
    /// The modular checksum (`CFDP::Checksum`).
    pub const MODULAR: Self = Self(0);
    /// CRC-32 (not supported).
    pub const CRC_32: Self = Self(1);
    /// The null checksum.
    pub const NULL_CHECKSUM: Self = Self(15);
}

/// C++ `TxnState`: a transaction's high-level state.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum TxnState {
    /// Unused (on the free list).
    #[default]
    Undef = 0,
    /// Freshly allocated.
    Init = 1,
    /// Receiving as class 1.
    R1 = 2,
    /// Sending as class 1.
    S1 = 3,
    /// Receiving as class 2.
    R2 = 4,
    /// Sending as class 2.
    S2 = 5,
    /// Dropping every PDU.
    Drop = 6,
    /// Finished; waiting to be recycled.
    Hold = 7,
    /// Upper range marker.
    Invalid = 8,
}

/// C++ `TxSubState`: a send transaction's sub-state.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum TxSubState {
    /// Sending the metadata directive.
    #[default]
    Metadata = 0,
    /// Sending file data.
    FileData = 1,
    /// Sending the EOF directive.
    Eof = 2,
    /// Waiting for the final acknowledgments.
    CloseoutSync = 3,
    /// Number of sub-states.
    NumStates = 4,
}

/// C++ `RxSubState`: a receive transaction's sub-state.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum RxSubState {
    /// Receiving file data.
    #[default]
    FileData = 0,
    /// Got the EOF directive.
    Eof = 1,
    /// Waiting for the final acknowledgments.
    CloseoutSync = 2,
    /// Number of sub-states.
    NumStates = 3,
}

/// C++ `Direction`: send or receive (history entries, chunk pools).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Direction {
    /// Receive.
    #[default]
    Rx = 0,
    /// Transmit.
    Tx = 1,
}

/// C++ `Direction::DIRECTION_NUM`.
pub const DIRECTION_NUM: usize = 2;

/// C++ `TransactionInitType`: who started a transmit transaction.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TransactionInitType {
    /// The `SendFile` command (or a directory playback).
    #[default]
    InitByCommand = 0,
    /// The `fileIn` port.
    InitByPort = 1,
}

/// C++ `CfdpTickType`: the queue a channel tick pass works on.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TickType {
    /// Receive transactions.
    Rx = 0,
    /// Waiting transmit transactions (timers, EOF, FIN-ACK).
    TxwNorm = 1,
    /// Waiting transmit transactions (NAK responses).
    TxwNak = 2,
}

/// C++ `CFDP_TICK_TYPE_NUM_TYPES`.
pub const TICK_TYPE_NUM_TYPES: u8 = 3;

/// C++ `TxnStatus`: a transaction's result — the condition codes (0-15)
/// plus local conditions (16-21); `UNDEFINED` (-1) while in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TxnStatus(pub i32);

impl Default for TxnStatus {
    fn default() -> Self {
        Self::UNDEFINED
    }
}

impl TxnStatus {
    /// In progress, no status yet.
    pub const UNDEFINED: Self = Self(-1);
    /// No error.
    pub const NO_ERROR: Self = Self(0);
    /// Positive ACK limit reached.
    pub const POS_ACK_LIMIT_REACHED: Self = Self(1);
    /// Keep-alive limit reached.
    pub const KEEP_ALIVE_LIMIT_REACHED: Self = Self(2);
    /// Invalid transmission mode.
    pub const INVALID_TRANSMISSION_MODE: Self = Self(3);
    /// Filestore rejection.
    pub const FILESTORE_REJECTION: Self = Self(4);
    /// File checksum failure.
    pub const FILE_CHECKSUM_FAILURE: Self = Self(5);
    /// File size error.
    pub const FILE_SIZE_ERROR: Self = Self(6);
    /// NAK limit reached.
    pub const NAK_LIMIT_REACHED: Self = Self(7);
    /// Inactivity detected.
    pub const INACTIVITY_DETECTED: Self = Self(8);
    /// Invalid file structure.
    pub const INVALID_FILE_STRUCTURE: Self = Self(9);
    /// Check limit reached.
    pub const CHECK_LIMIT_REACHED: Self = Self(10);
    /// Unsupported checksum type.
    pub const UNSUPPORTED_CHECKSUM_TYPE: Self = Self(11);
    /// Suspend request received.
    pub const SUSPEND_REQUEST_RECEIVED: Self = Self(14);
    /// Cancel request received.
    pub const CANCEL_REQUEST_RECEIVED: Self = Self(15);
    /// Protocol error.
    pub const PROTOCOL_ERROR: Self = Self(16);
    /// ACK limit reached without a FIN.
    pub const ACK_LIMIT_NO_FIN: Self = Self(17);
    /// ACK limit reached without an EOF.
    pub const ACK_LIMIT_NO_EOF: Self = Self(18);
    /// Error responding to a NAK.
    pub const NAK_RESPONSE_ERROR: Self = Self(19);
    /// Failed to send the EOF.
    pub const SEND_EOF_FAILURE: Self = Self(20);
    /// FIN received early.
    pub const EARLY_FIN: Self = Self(21);
    /// Upper range marker.
    pub const MAX: Self = Self(22);

    /// The status a condition code maps to (they share values 0-15).
    #[must_use]
    pub fn from_condition_code(cc: ConditionCode) -> Self {
        Self(i32::from(cc.0))
    }
}

// ---------------------------------------------------------------------------
// Transaction bookkeeping (Types.hpp)
// ---------------------------------------------------------------------------

/// C++ `CfdpTxnFilenames`: a transaction's source and destination names
/// (`Fw::String`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TxnFilenames {
    /// Source file name.
    pub src_filename: FwDefaultString,
    /// Destination file name.
    pub dst_filename: FwDefaultString,
}

/// C++ `History`: the record of one transaction, kept after it completes.
/// Its `cl_node` is the channel's history list link (`CListLinks`).
#[derive(Debug, Clone, Default)]
pub struct History {
    /// File names.
    pub fnames: TxnFilenames,
    /// Direction.
    pub dir: Direction,
    /// Final status.
    pub txn_stat: TxnStatus,
    /// The transaction's source entity.
    pub src_eid: EntityId,
    /// The other entity (same as `src_eid` for receive).
    pub peer_eid: EntityId,
    /// Transaction sequence number.
    pub seq_num: TransactionSeq,
}

/// C++ `CfdpChunkWrapper`: a pooled chunk list (its `cl_node` is the
/// channel's chunk-pool link).
#[derive(Debug)]
pub struct ChunkWrapper {
    /// The gap-tracking list.
    pub chunks: ChunkList,
}

/// C++ `Playback`: one directory playback.
#[derive(Default)]
pub struct Playback {
    /// The open directory.
    pub dir: Directory,
    /// Class of the transfers.
    pub cfdp_class: Class,
    /// Source and destination directories.
    pub fnames: TxnFilenames,
    /// Transactions in flight.
    pub num_ts: u16,
    /// Priority of the transfers.
    pub priority: u8,
    /// Destination entity.
    pub dest_id: EntityId,
    /// The next file name, read but not yet started.
    pub pending_file: PathString,
    /// Playback in progress.
    pub busy: bool,
    /// Directory open.
    pub diropen: bool,
    /// Keep or delete the sent files.
    pub keep: Keep,
    /// Counted in the active-playback telemetry.
    pub counted: bool,
}

/// C++ `CfdpPollDir`: one directory poll.
#[derive(Default)]
pub struct PollDir {
    /// The current playback of the directory.
    pub pb: Playback,
    /// Interval timer.
    pub interval_timer: Timer,
    /// Poll interval, seconds.
    pub interval_sec: u32,
    /// Priority of the transfers.
    pub priority: u8,
    /// Class of the transfers.
    pub cfdp_class: Class,
    /// Destination entity.
    pub dest_eid: EntityId,
    /// Source directory.
    pub src_dir: FwDefaultString,
    /// Destination directory.
    pub dst_dir: FwDefaultString,
    /// Whether the poll is running.
    pub enabled: Enabled,
}

/// C++ `CfdpTxS2Data`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TxS2Data {
    /// The condition code of the received FIN (echoed in the FIN-ACK).
    pub fin_cc: u8,
    /// ACK/NAK retry counter.
    pub acknak_count: u8,
}

/// C++ `CfdpTxStateData`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TxStateData {
    /// Sub-state.
    pub sub_state: TxSubState,
    /// The file position after the last read.
    pub cached_pos: FileSize,
    /// Class 2 data.
    pub s2: TxS2Data,
}

/// C++ `CfdpRxS2Data`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RxS2Data {
    /// The checksum from the EOF.
    pub eof_crc: u32,
    /// The file size from the EOF.
    pub eof_size: FileSize,
    /// Bytes checksummed so far.
    pub rx_crc_calc_bytes: FileSize,
    /// FIN delivery code.
    pub dc: FinDeliveryCode,
    /// FIN file status.
    pub fs: FinFileStatus,
    /// The condition code of the received EOF (echoed in the EOF-ACK).
    pub eof_cc: u8,
    /// ACK/NAK retry counter.
    pub acknak_count: u8,
}

/// C++ `CfdpRxStateData`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RxStateData {
    /// Sub-state.
    pub sub_state: RxSubState,
    /// The file position after the last write.
    pub cached_pos: FileSize,
    /// Class 2 data.
    pub r2: RxS2Data,
}

/// C++ `CfdpStateData`. Upstream it is a union of the send and receive
/// data; a transaction only ever uses the half matching its direction, so
/// the halves are kept side by side and both zeroed on reset.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StateData {
    /// Send transaction data.
    pub send: TxStateData,
    /// Receive transaction data.
    pub receive: RxStateData,
}

/// C++ `CfdpFlagsCommon`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlagsCommon {
    /// The queue the transaction is on.
    pub q_index: QueueId,
    /// ACK timer armed.
    pub ack_timer_armed: bool,
    /// Suspended by command.
    pub suspended: bool,
    /// Canceled.
    pub canceled: bool,
    /// The checksum has been computed.
    pub crc_calc: bool,
    /// The inactivity timer has expired.
    pub inactivity_fired: bool,
    /// Keep the history entry when recycled.
    pub keep_history: bool,
    /// Terminal-send retries since inactivity fired.
    pub post_inactivity_send_retries: u8,
}

/// C++ `CfdpFlagsRx`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlagsRx {
    /// Metadata received.
    pub md_recv: bool,
    /// EOF received.
    pub eof_recv: bool,
    /// A NAK is pending.
    pub send_nak: bool,
    /// A FIN is pending.
    pub send_fin: bool,
    /// An EOF-ACK is pending.
    pub send_eof_ack: bool,
    /// All data received (class 2).
    pub complete: bool,
    /// At least one file-data NAK was sent.
    pub fd_nak_sent: bool,
}

/// C++ `CfdpFlagsTx`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlagsTx {
    /// The metadata must be re-sent (NAKed).
    pub md_need_send: bool,
    /// An EOF is pending.
    pub send_eof: bool,
    /// EOF-ACK received.
    pub eof_ack_recv: bool,
    /// FIN received.
    pub fin_recv: bool,
    /// A FIN-ACK is pending.
    pub send_fin_ack: bool,
    /// A commanded (ground) transmit.
    pub cmd_tx: bool,
}

/// C++ `CfdpStateFlags`. Upstream it is a union whose `rx` and `tx`
/// members both begin with `com`; the direction-specific halves are kept
/// side by side here (only the half matching the direction is used).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StateFlags {
    /// Flags for every transaction.
    pub com: FlagsCommon,
    /// Receive flags.
    pub rx: FlagsRx,
    /// Transmit flags.
    pub tx: FlagsTx,
}
