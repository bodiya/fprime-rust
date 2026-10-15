//! # CfdpManager — port of `Svc::Ccsds::Cfdp::CfdpManager` (active)
//!
//! C++ sources: `Svc/Ccsds/CfdpManager/CfdpManager.{fpp,hpp,cpp}`,
//! `{Commands,Events,Parameters,Telemetry}.fppi`, `Engine`, `Channel`,
//! `Transaction{,Rx,Tx}`, `Chunk`, `Clist`, `Timer`, `Utils`, `Types/`
//! (the PDUs), `default/config/CfdpCfg.{fpp,hpp}`
//! ([`fprime_config::cfdp`]); design: `Svc/Ccsds/CfdpManager/docs/sdd.md`.
//!
//! The CCSDS File Delivery Protocol (CCSDS 727.0-B-5): class 1
//! (unacknowledged) and class 2 (acknowledged, with NAK-driven
//! retransmission) file transfers in both directions, over
//! [`NUM_CHANNELS`] channels, each with a preallocated pool of
//! transactions, chunk lists and history entries. Transfers start from the
//! `SendFile` command, the `fileIn` port, directory playbacks and directory
//! polls; received PDUs arrive on `dataIn[channel]` (with the 2-byte
//! `FW_PACKET_FILE` descriptor `FprimeRouter` leaves on them) and outgoing
//! PDUs leave on `dataOut[channel]` in buffers from
//! `bufferAllocate[channel]`, prefixed with the same descriptor for
//! `ComQueue`. `run1Hz` drives every protocol timer.
//!
//! ## Attribution
//!
//! Upstream's `ATTRIBUTION.md`: the engine, channel, transaction state
//! machines, chunk lists, circular lists and utilities are ports of NASA's
//! Core Flight System CFDP application (CF) 3.0.0
//! (<https://github.com/nasa/CF>), Copyright (c) 2019 United States
//! Government as represented by the Administrator of the National
//! Aeronautics and Space Administration, NASA Docket No. GSC-18,447-1,
//! licensed under the Apache License, Version 2.0; each of those modules
//! carries the notice. The component wrapper, the timer and the PDU classes
//! are new F´ implementations. This Rust port is a port of those files and
//! keeps the same attribution.
//!
//! ## Structure
//!
//! | module | C++ |
//! |--------|-----|
//! | this module | `CfdpManager` (ports, commands, events, telemetry, parameters) |
//! | [`engine`] | `Engine` |
//! | [`channel`] | `Channel` |
//! | [`transaction`], `transaction_tx`, `transaction_rx` | `Transaction`, `TransactionTx.cpp`, `TransactionRx.cpp` |
//! | [`pdu`] | `Types/*Pdu`, `Types/Tlv`, `Types/PduHeader` |
//! | [`chunk`], [`clist`], [`timer`], [`utils`], [`types`] | `Chunk`, `Clist`, `Timer`, `Utils`, `Types/Types.{fpp,hpp}` |
//!
//! Upstream links transactions, histories and chunk lists into intrusive
//! circular lists and keeps back-pointers from a transaction to its
//! channel, engine and manager. Here every pool element is an index and
//! the state-machine methods live on the channel that owns the pools; the
//! manager is passed down as `&CfdpManager`. List order — and so every
//! scheduling decision — is the C one.
//!
//! ## Threading
//!
//! ACTIVE. The engine (all protocol state) lives in one `Mutex` that only
//! the component thread locks: `run1Hz`, `dataIn`, `dataReturnIn` and the
//! commands are queued, as upstream. Output ports are invoked with it held
//! (the C++ engine calls them mid-transaction); every input that could loop
//! back into this component is async, so that cannot deadlock. The guarded
//! `fileIn` port only enqueues a request on its own `Os::Queue`, drained at
//! the start of `run1Hz`. The parameter commands are synchronous, as the
//! generated C++ handles them, and use their own lock.
//!
//! ## Deviations
//!
//! - Pools are `Box<[T]>`s allocated by [`CfdpManager::configure`] (upstream
//!   carves them from an `Fw::MemAllocator`); `cleanup` drops them.
//! - A buffer from `bufferAllocate` of the wrong size is dropped (upstream
//!   leaks it); a buffer that fails to encode goes back on
//!   `bufferDeallocate`, as upstream.
//! - `FileDataPdu::getMaxFileDataSize` is computed from a zeroed header
//!   (upstream reads an uninitialized one); with the default
//!   `OutgoingFileChunkSize` (992) the bound never binds.

pub mod channel;
pub mod chunk;
pub mod clist;
pub mod engine;
pub mod pdu;
pub mod timer;
pub mod transaction;
mod transaction_rx;
mod transaction_tx;
pub mod types;
pub mod utils;

#[cfg(test)]
mod tests;

pub use engine::Engine;
pub use types::*;

use crate::file_downlink::{
    SendFileCompletePort, SendFileNameArg, SendFileRequestPort, SendFileResponse, SendFileStatus,
};
use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferGetPort, BufferSendPort, CmdGlue, CmdPort,
    ComponentDispatch, EventGlue, MsgDispatchStatus, OutputPort, PingPort, PortRef, PrmGlue,
    QueueFullPolicy, SchedPort, TlmGlue, async_input_port_adapter, component_msg_types, msg,
};
use fprime_config::cfdp::{MAX_FILE_PATH_SIZE, MAX_POLLING_DIR_PER_CHAN, NUM_CHANNELS};
use fprime_config::{
    FW_CMD_ARG_BUFFER_MAX_SIZE, FW_LOG_STRING_MAX_SIZE, FW_LOG_TEXT_BUFFER_SIZE, FwChanIdType,
    FwEventIdType, FwIdType, FwIndexType, FwOpcodeType, FwPacketDescriptorType, FwPrmIdType,
    FwQueuePriorityType, FwSizeType,
};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, CmdStringArg, ComPacketType, Deserialize, Enabled,
    Endianness, FwDefaultString, LengthMode, LinearBuffer, LogBuffer, LogSeverity, ParamBuffer,
    ParamValid, SerBuf, SerBufAny, Serialize, SerializeStatus, fw_assert, fw_try,
};
use fprime_os::queue::{BlockingType, Queue, Status as QueueStatus};
use std::fmt::{self, Write as _};
use std::sync::{Arc, Mutex};

/// C++ `CfdpManager::PACKET_DESCRIPTOR_SIZE`: the `FwPacketDescriptorType`
/// prefixed to every PDU buffer.
pub const PACKET_DESCRIPTOR_SIZE: usize = size_of::<FwPacketDescriptorType>();

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel): the async input ports
    /// in FPP declaration order, then the async commands.
    impl CfdpManager {
        /// `run1Hz`.
        MSG_TYPE_RUN_1HZ,
        /// `pingIn` (drop).
        MSG_TYPE_PING_IN,
        /// `dataReturnIn[NumChannels]`.
        MSG_TYPE_DATA_RETURN_IN,
        /// `dataIn[NumChannels]`.
        MSG_TYPE_DATA_IN,
        /// The async commands.
        MSG_TYPE_CMD_IN,
    }
}

/// Queue message size: the command envelope, the largest message (6-byte
/// envelope + opcode + sequence + nested `CmdArgBuffer`).
pub const QUEUE_MSG_SIZE: usize =
    msg::ENVELOPE_HEADER_SIZE + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE;

/// Every async input and command has the default priority.
const PRIORITY: FwQueuePriorityType = 0;

/// A `fileIn` request on the handoff queue (C++ `FileInRequest`):
/// `[source (u16 len + bytes)][destination][context u32]`.
const FILE_IN_REQUEST_SIZE: usize = 2 * FwDefaultString::SERIALIZED_SIZE + 4;

/// Event string arguments (`string size MaxFilePathSize`) are truncated to
/// `min(MaxFilePathSize, FW_LOG_STRING_MAX_SIZE)`, as the generated
/// `log_*` functions do.
const EVENT_STRING_SIZE: usize = if MAX_FILE_PATH_SIZE < FW_LOG_STRING_MAX_SIZE {
    MAX_FILE_PATH_SIZE
} else {
    FW_LOG_STRING_MAX_SIZE
};

impl CfdpManager {
    /// `SendFile(channelId, destId, cfdpClass, keep, priority, sourceFileName, destFileName)`.
    pub const OPCODE_SEND_FILE: FwOpcodeType = 0;
    /// `PlaybackDirectory(channelId, destId, cfdpClass, keep, priority, sourceDirectory, destDirectory)`.
    pub const OPCODE_PLAYBACK_DIRECTORY: FwOpcodeType = 1;
    /// `PollDirectory(channelId, pollId, destId, cfdpClass, priority, interval, sourceDirectory, destDirectory)`.
    pub const OPCODE_POLL_DIRECTORY: FwOpcodeType = 2;
    /// `StopPollDirectory(channelId, pollId)`.
    pub const OPCODE_STOP_POLL_DIRECTORY: FwOpcodeType = 3;
    /// `SetChannelFlow(channelId, freeze)`.
    pub const OPCODE_SET_CHANNEL_FLOW: FwOpcodeType = 4;
    /// `SuspendResumeTransaction(channelId, transactionSeq, entityId, action)`.
    pub const OPCODE_SUSPEND_RESUME_TRANSACTION: FwOpcodeType = 5;
    /// `CancelTransaction(channelId, transactionSeq, entityId)`.
    pub const OPCODE_CANCEL_TRANSACTION: FwOpcodeType = 6;
    /// `AbandonTransaction(channelId, transactionSeq, entityId)`.
    pub const OPCODE_ABANDON_TRANSACTION: FwOpcodeType = 7;
    /// `ResetCounters(channelId)`.
    pub const OPCODE_RESET_COUNTERS: FwOpcodeType = 8;
    /// `LOCALEID_PRM_SET`.
    pub const OPCODE_LOCALEID_PRM_SET: FwOpcodeType = 9;
    /// `LOCALEID_PRM_SAVE`.
    pub const OPCODE_LOCALEID_PRM_SAVE: FwOpcodeType = 10;
    /// `OUTGOINGFILECHUNKSIZE_PRM_SET`.
    pub const OPCODE_OUTGOINGFILECHUNKSIZE_PRM_SET: FwOpcodeType = 11;
    /// `OUTGOINGFILECHUNKSIZE_PRM_SAVE`.
    pub const OPCODE_OUTGOINGFILECHUNKSIZE_PRM_SAVE: FwOpcodeType = 12;
    /// `RXCRCCALCBYTESPERCYCLE_PRM_SET`.
    pub const OPCODE_RXCRCCALCBYTESPERCYCLE_PRM_SET: FwOpcodeType = 13;
    /// `RXCRCCALCBYTESPERCYCLE_PRM_SAVE`.
    pub const OPCODE_RXCRCCALCBYTESPERCYCLE_PRM_SAVE: FwOpcodeType = 14;
    /// `POSTINACTIVITYSENDRETRIES_PRM_SET`.
    pub const OPCODE_POSTINACTIVITYSENDRETRIES_PRM_SET: FwOpcodeType = 15;
    /// `POSTINACTIVITYSENDRETRIES_PRM_SAVE`.
    pub const OPCODE_POSTINACTIVITYSENDRETRIES_PRM_SAVE: FwOpcodeType = 16;
    /// `FILEINDEFAULTCHANNEL_PRM_SET`.
    pub const OPCODE_FILEINDEFAULTCHANNEL_PRM_SET: FwOpcodeType = 17;
    /// `FILEINDEFAULTCHANNEL_PRM_SAVE`.
    pub const OPCODE_FILEINDEFAULTCHANNEL_PRM_SAVE: FwOpcodeType = 18;
    /// `FILEINDEFAULTDESTENTITYID_PRM_SET`.
    pub const OPCODE_FILEINDEFAULTDESTENTITYID_PRM_SET: FwOpcodeType = 19;
    /// `FILEINDEFAULTDESTENTITYID_PRM_SAVE`.
    pub const OPCODE_FILEINDEFAULTDESTENTITYID_PRM_SAVE: FwOpcodeType = 20;
    /// `FILEINDEFAULTCLASS_PRM_SET`.
    pub const OPCODE_FILEINDEFAULTCLASS_PRM_SET: FwOpcodeType = 21;
    /// `FILEINDEFAULTCLASS_PRM_SAVE`.
    pub const OPCODE_FILEINDEFAULTCLASS_PRM_SAVE: FwOpcodeType = 22;
    /// `FILEINDEFAULTKEEP_PRM_SET`.
    pub const OPCODE_FILEINDEFAULTKEEP_PRM_SET: FwOpcodeType = 23;
    /// `FILEINDEFAULTKEEP_PRM_SAVE`.
    pub const OPCODE_FILEINDEFAULTKEEP_PRM_SAVE: FwOpcodeType = 24;
    /// `FILEINDEFAULTPRIORITY_PRM_SET`.
    pub const OPCODE_FILEINDEFAULTPRIORITY_PRM_SET: FwOpcodeType = 25;
    /// `FILEINDEFAULTPRIORITY_PRM_SAVE`.
    pub const OPCODE_FILEINDEFAULTPRIORITY_PRM_SAVE: FwOpcodeType = 26;
    /// `CHANNELCONFIG_PRM_SET`.
    pub const OPCODE_CHANNELCONFIG_PRM_SET: FwOpcodeType = 27;
    /// `CHANNELCONFIG_PRM_SAVE`.
    pub const OPCODE_CHANNELCONFIG_PRM_SAVE: FwOpcodeType = 28;

    /// `ChannelTelemetry: Cfdp.ChannelTelemetryArray`.
    pub const CHANID_CHANNEL_TELEMETRY: FwChanIdType = 0;

    /// `LocalEid: Cfdp.EntityId default 42`.
    pub const PARAMID_LOCAL_EID: FwPrmIdType = 0;
    /// `OutgoingFileChunkSize: U32 default 992`.
    pub const PARAMID_OUTGOING_FILE_CHUNK_SIZE: FwPrmIdType = 1;
    /// `RxCrcCalcBytesPerCycle: U32 default 65536`.
    pub const PARAMID_RX_CRC_CALC_BYTES_PER_CYCLE: FwPrmIdType = 2;
    /// `PostInactivitySendRetries: U8 default 3`.
    pub const PARAMID_POST_INACTIVITY_SEND_RETRIES: FwPrmIdType = 3;
    /// `FileInDefaultChannel: U8 default 0`.
    pub const PARAMID_FILE_IN_DEFAULT_CHANNEL: FwPrmIdType = 4;
    /// `FileInDefaultDestEntityId: Cfdp.EntityId default 100`.
    pub const PARAMID_FILE_IN_DEFAULT_DEST_ENTITY_ID: FwPrmIdType = 5;
    /// `FileInDefaultClass: Cfdp.Class default CLASS_2`.
    pub const PARAMID_FILE_IN_DEFAULT_CLASS: FwPrmIdType = 6;
    /// `FileInDefaultKeep: Cfdp.Keep default DELETE`.
    pub const PARAMID_FILE_IN_DEFAULT_KEEP: FwPrmIdType = 7;
    /// `FileInDefaultPriority: U8 default 0`.
    pub const PARAMID_FILE_IN_DEFAULT_PRIORITY: FwPrmIdType = 8;
    /// `ChannelConfig: Cfdp.ChannelArrayParams`.
    pub const PARAMID_CHANNEL_CONFIG: FwPrmIdType = 9;

    /// Default of `LocalEid`.
    pub const LOCAL_EID_DEFAULT: EntityId = 42;
    /// Default of `OutgoingFileChunkSize`.
    pub const OUTGOING_FILE_CHUNK_SIZE_DEFAULT: u32 = 992;
    /// Default of `RxCrcCalcBytesPerCycle`.
    pub const RX_CRC_CALC_BYTES_PER_CYCLE_DEFAULT: u32 = 65536;
    /// Default of `PostInactivitySendRetries`.
    pub const POST_INACTIVITY_SEND_RETRIES_DEFAULT: u8 = 3;
    /// Default of `FileInDefaultChannel`.
    pub const FILE_IN_DEFAULT_CHANNEL_DEFAULT: u8 = 0;
    /// Default of `FileInDefaultDestEntityId`.
    pub const FILE_IN_DEFAULT_DEST_ENTITY_ID_DEFAULT: EntityId = 100;
    /// Default of `FileInDefaultClass`.
    pub const FILE_IN_DEFAULT_CLASS_DEFAULT: Class = Class::Class2;
    /// Default of `FileInDefaultKeep`.
    pub const FILE_IN_DEFAULT_KEEP_DEFAULT: Keep = Keep::Delete;
    /// Default of `FileInDefaultPriority`.
    pub const FILE_IN_DEFAULT_PRIORITY_DEFAULT: u8 = 0;
}

// ---------------------------------------------------------------------------
// FPP enum names (event text, the generated `toString`).
// ---------------------------------------------------------------------------

impl Class {
    /// The FPP constant name.
    #[must_use]
    pub fn fpp_name(self) -> &'static str {
        match self {
            Self::Class2 => "CLASS_2",
            Self::Class1 => "CLASS_1",
        }
    }
}

impl Flow {
    /// The FPP constant name.
    #[must_use]
    pub fn fpp_name(self) -> &'static str {
        match self {
            Self::NotFrozen => "NOT_FROZEN",
            Self::Frozen => "FROZEN",
        }
    }
}

impl PduTypeEnum {
    /// The FPP constant name.
    #[must_use]
    pub fn fpp_name(self) -> &'static str {
        match self {
            Self::Metadata => "METADATA",
            Self::EndOfFile => "END_OF_FILE",
            Self::Finished => "FINISHED",
            Self::Acknowledgment => "ACKNOWLEDGMENT",
            Self::NegativeAck => "NEGATIVE_ACK",
            Self::FileData => "FILE_DATA",
            Self::None => "NONE",
        }
    }
}

// ---------------------------------------------------------------------------
// Parameters.
// ---------------------------------------------------------------------------

/// One parameter: its value and validity (the generated `m_<param>` /
/// `m_param_<param>_valid`).
#[derive(Debug, Clone)]
struct Param<T> {
    value: T,
    valid: ParamValid,
}

impl<T> Param<T> {
    fn new(value: T) -> Self {
        Self {
            value,
            valid: ParamValid::Uninit,
        }
    }
}

/// The parameter store, guarded by the generated `m_paramLock`.
#[derive(Debug)]
struct CfdpParams {
    local_eid: Param<EntityId>,
    outgoing_file_chunk_size: Param<u32>,
    rx_crc_calc_bytes_per_cycle: Param<u32>,
    post_inactivity_send_retries: Param<u8>,
    file_in_default_channel: Param<u8>,
    file_in_default_dest_entity_id: Param<EntityId>,
    file_in_default_class: Param<Class>,
    file_in_default_keep: Param<Keep>,
    file_in_default_priority: Param<u8>,
    channel_config: Param<ChannelArrayParams>,
}

impl CfdpParams {
    fn new() -> Self {
        Self {
            local_eid: Param::new(CfdpManager::LOCAL_EID_DEFAULT),
            outgoing_file_chunk_size: Param::new(CfdpManager::OUTGOING_FILE_CHUNK_SIZE_DEFAULT),
            rx_crc_calc_bytes_per_cycle: Param::new(
                CfdpManager::RX_CRC_CALC_BYTES_PER_CYCLE_DEFAULT,
            ),
            post_inactivity_send_retries: Param::new(
                CfdpManager::POST_INACTIVITY_SEND_RETRIES_DEFAULT,
            ),
            file_in_default_channel: Param::new(CfdpManager::FILE_IN_DEFAULT_CHANNEL_DEFAULT),
            file_in_default_dest_entity_id: Param::new(
                CfdpManager::FILE_IN_DEFAULT_DEST_ENTITY_ID_DEFAULT,
            ),
            file_in_default_class: Param::new(CfdpManager::FILE_IN_DEFAULT_CLASS_DEFAULT),
            file_in_default_keep: Param::new(CfdpManager::FILE_IN_DEFAULT_KEEP_DEFAULT),
            file_in_default_priority: Param::new(CfdpManager::FILE_IN_DEFAULT_PRIORITY_DEFAULT),
            channel_config: Param::new(ChannelArrayParams::fpp_default()),
        }
    }
}

/// The generated `loadParameters` step for one parameter: the stored value
/// when the database says `VALID` and it deserializes, else the default
/// (flagged `DEFAULT`).
fn load_param<T: Deserialize + Default>(
    slot: &mut Param<T>,
    valid: ParamValid,
    buf: &mut ParamBuffer,
    default: T,
) {
    if valid == ParamValid::Valid {
        let mut value = T::default();
        if buf.deserialize(&mut value, Endianness::Big).is_ok() {
            slot.value = value;
            slot.valid = ParamValid::Valid;
            return;
        }
    }
    slot.value = default;
    slot.valid = ParamValid::Default;
}

/// The generated `<PARAM>_PRM_SET` body: deserialize (`FORMAT_ERROR` on a
/// failure or a leftover byte) and store as `VALID`.
fn set_param<T: Deserialize + Default>(
    slot: &mut Param<T>,
    args: &mut CmdArgBuffer,
) -> CmdResponse {
    let mut value = T::default();
    if !args.deserialize(&mut value, Endianness::Big).is_ok() || args.deserialize_size_left() != 0 {
        return CmdResponse::FormatError;
    }
    slot.value = value;
    slot.valid = ParamValid::Valid;
    CmdResponse::Ok
}

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// `Svc::Ccsds::Cfdp::CfdpManager` — the CFDP engine component.
pub struct CfdpManager {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// `cmdRegOut` / `cmdResponseOut`.
    pub cmd: CmdGlue,
    /// `logOut` / `logTextOut` / `timeCaller`.
    pub evt: EventGlue,
    /// `tlmOut`.
    pub tlm: TlmGlue,
    /// `prmGetOut` / `prmSetOut`.
    pub prm: PrmGlue,
    /// `pingOut`: `Svc.Ping` out.
    pub ping_out: OutputPort<dyn PingPort>,
    /// `dataOut[NumChannels]`: `Fw.BufferSend` out — encoded PDUs.
    pub data_out: [OutputPort<dyn BufferSendPort>; NUM_CHANNELS],
    /// `bufferAllocate[NumChannels]`: `Fw.BufferGet` out — PDU buffers.
    pub buffer_allocate: [OutputPort<dyn BufferGetPort>; NUM_CHANNELS],
    /// `bufferDeallocate[NumChannels]`: `Fw.BufferSend` out — PDU buffers
    /// back to the allocator (unsent, or returned on `dataReturnIn`).
    pub buffer_deallocate: [OutputPort<dyn BufferSendPort>; NUM_CHANNELS],
    /// `dataInReturn[NumChannels]`: `Fw.BufferSend` out — buffers received
    /// on `dataIn`.
    pub data_in_return: [OutputPort<dyn BufferSendPort>; NUM_CHANNELS],
    /// `fileDoneOut`: `Svc.SendFileComplete` out — results of `fileIn`
    /// transfers.
    pub file_done_out: OutputPort<dyn SendFileCompletePort>,
    /// Escrow for the async buffer-carrying ports.
    escrow: BufferEscrow,
    /// The CFDP engine (C++ `m_engine`), created by `configure`.
    pub(crate) engine: Mutex<Option<Engine>>,
    /// The `fileIn` handoff queue (C++ `m_fileInQueue`).
    file_in_queue: Queue,
    /// Its depth, also the most requests drained per cycle.
    file_in_queue_depth: Mutex<FwSizeType>,
    /// The guarded-port mutex (only `fileIn` is guarded).
    guard: Mutex<()>,
    params: Mutex<CfdpParams>,
}

impl CfdpManager {
    /// Construct (C++ constructor). Follow with `set_id_base`, wiring,
    /// [`Self::init`], [`Self::load_parameters`], [`Self::configure`],
    /// [`Self::reg_commands`] and `active.start`.
    #[must_use]
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            prm: PrmGlue::new(),
            ping_out: OutputPort::new(),
            data_out: core::array::from_fn(|_| OutputPort::new()),
            buffer_allocate: core::array::from_fn(|_| OutputPort::new()),
            buffer_deallocate: core::array::from_fn(|_| OutputPort::new()),
            data_in_return: core::array::from_fn(|_| OutputPort::new()),
            file_done_out: OutputPort::new(),
            escrow: BufferEscrow::new(),
            engine: Mutex::new(None),
            file_in_queue: Queue::new(),
            file_in_queue_depth: Mutex::new(0),
            guard: Mutex::new(()),
            params: Mutex::new(CfdpParams::new()),
        })
    }

    /// C++ `init(queueDepth, instance)`: create the component queue.
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    /// C++ `configure(allocator, fileQueueDepth)`: allocate the engine and
    /// every channel's pools, zero the telemetry, and create the `fileIn`
    /// handoff queue (asserts that it is created).
    pub fn configure(&self, file_queue_depth: FwSizeType) {
        *self.engine.lock().unwrap() = Some(Engine::new());
        *self.file_in_queue_depth.lock().unwrap() = file_queue_depth;
        let status = self.file_in_queue.create(
            "cfdpFileInQueue",
            file_queue_depth,
            FILE_IN_REQUEST_SIZE as FwSizeType,
        );
        fw_assert!(status == QueueStatus::OpOk, status as i32);
    }

    /// C++ `cleanup`: release the engine.
    pub fn cleanup(&self) {
        *self.engine.lock().unwrap() = None;
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        let opcodes: Vec<FwOpcodeType> =
            (Self::OPCODE_SEND_FILE..=Self::OPCODE_CHANNELCONFIG_PRM_SAVE).collect();
        self.cmd.reg_commands(self.id_base(), &opcodes);
    }

    /// Run `f` on the engine (asserts `configure` was called — C++
    /// `FW_ASSERT(m_engine != nullptr)`).
    pub(crate) fn with_engine<R>(&self, f: impl FnOnce(&mut Engine) -> R) -> R {
        let mut guard = self.engine.lock().unwrap();
        fw_assert!(guard.is_some());
        match guard.as_mut() {
            Some(engine) => f(engine),
            None => unreachable!("asserted above"),
        }
    }

    // -- parameters ----------------------------------------------------------

    /// The generated `loadParameters()`.
    pub fn load_parameters(&self) {
        let id_base = self.id_base();
        let mut p = self.params.lock().unwrap();
        macro_rules! load {
            ($field:ident, $id:expr, $default:expr) => {{
                let mut buf = ParamBuffer::new();
                let valid = self.prm.get_param(id_base, $id, &mut buf);
                load_param(&mut p.$field, valid, &mut buf, $default);
            }};
        }
        load!(local_eid, Self::PARAMID_LOCAL_EID, Self::LOCAL_EID_DEFAULT);
        load!(
            outgoing_file_chunk_size,
            Self::PARAMID_OUTGOING_FILE_CHUNK_SIZE,
            Self::OUTGOING_FILE_CHUNK_SIZE_DEFAULT
        );
        load!(
            rx_crc_calc_bytes_per_cycle,
            Self::PARAMID_RX_CRC_CALC_BYTES_PER_CYCLE,
            Self::RX_CRC_CALC_BYTES_PER_CYCLE_DEFAULT
        );
        load!(
            post_inactivity_send_retries,
            Self::PARAMID_POST_INACTIVITY_SEND_RETRIES,
            Self::POST_INACTIVITY_SEND_RETRIES_DEFAULT
        );
        load!(
            file_in_default_channel,
            Self::PARAMID_FILE_IN_DEFAULT_CHANNEL,
            Self::FILE_IN_DEFAULT_CHANNEL_DEFAULT
        );
        load!(
            file_in_default_dest_entity_id,
            Self::PARAMID_FILE_IN_DEFAULT_DEST_ENTITY_ID,
            Self::FILE_IN_DEFAULT_DEST_ENTITY_ID_DEFAULT
        );
        load!(
            file_in_default_class,
            Self::PARAMID_FILE_IN_DEFAULT_CLASS,
            Self::FILE_IN_DEFAULT_CLASS_DEFAULT
        );
        load!(
            file_in_default_keep,
            Self::PARAMID_FILE_IN_DEFAULT_KEEP,
            Self::FILE_IN_DEFAULT_KEEP_DEFAULT
        );
        load!(
            file_in_default_priority,
            Self::PARAMID_FILE_IN_DEFAULT_PRIORITY,
            Self::FILE_IN_DEFAULT_PRIORITY_DEFAULT
        );
        load!(
            channel_config,
            Self::PARAMID_CHANNEL_CONFIG,
            ChannelArrayParams::fpp_default()
        );
    }

    /// Read one parameter, asserting it is usable (`FW_PARAM_OK`: every
    /// CFDP parameter has a default, so anything else is a coding error).
    fn param<T: Clone>(&self, get: impl FnOnce(&CfdpParams) -> &Param<T>) -> T {
        let p = self.params.lock().unwrap();
        let param = get(&p);
        fw_assert!(param.valid.is_ok(), param.valid.as_repr() as i32);
        param.value.clone()
    }

    /// `paramGet_<name>` for tests and tools: the value and its validity.
    fn param_get<T: Clone>(&self, get: impl FnOnce(&CfdpParams) -> &Param<T>) -> (T, ParamValid) {
        let p = self.params.lock().unwrap();
        let param = get(&p);
        (param.value.clone(), param.valid)
    }

    /// `paramGet_LocalEid`.
    #[must_use]
    pub fn param_get_local_eid(&self) -> (EntityId, ParamValid) {
        self.param_get(|p| &p.local_eid)
    }

    /// `paramGet_OutgoingFileChunkSize`.
    #[must_use]
    pub fn param_get_outgoing_file_chunk_size(&self) -> (u32, ParamValid) {
        self.param_get(|p| &p.outgoing_file_chunk_size)
    }

    /// `paramGet_RxCrcCalcBytesPerCycle`.
    #[must_use]
    pub fn param_get_rx_crc_calc_bytes_per_cycle(&self) -> (u32, ParamValid) {
        self.param_get(|p| &p.rx_crc_calc_bytes_per_cycle)
    }

    /// `paramGet_PostInactivitySendRetries`.
    #[must_use]
    pub fn param_get_post_inactivity_send_retries(&self) -> (u8, ParamValid) {
        self.param_get(|p| &p.post_inactivity_send_retries)
    }

    /// `paramGet_FileInDefaultChannel`.
    #[must_use]
    pub fn param_get_file_in_default_channel(&self) -> (u8, ParamValid) {
        self.param_get(|p| &p.file_in_default_channel)
    }

    /// `paramGet_FileInDefaultDestEntityId`.
    #[must_use]
    pub fn param_get_file_in_default_dest_entity_id(&self) -> (EntityId, ParamValid) {
        self.param_get(|p| &p.file_in_default_dest_entity_id)
    }

    /// `paramGet_FileInDefaultClass`.
    #[must_use]
    pub fn param_get_file_in_default_class(&self) -> (Class, ParamValid) {
        self.param_get(|p| &p.file_in_default_class)
    }

    /// `paramGet_FileInDefaultKeep`.
    #[must_use]
    pub fn param_get_file_in_default_keep(&self) -> (Keep, ParamValid) {
        self.param_get(|p| &p.file_in_default_keep)
    }

    /// `paramGet_FileInDefaultPriority`.
    #[must_use]
    pub fn param_get_file_in_default_priority(&self) -> (u8, ParamValid) {
        self.param_get(|p| &p.file_in_default_priority)
    }

    /// `paramGet_ChannelConfig`.
    #[must_use]
    pub fn param_get_channel_config(&self) -> (ChannelArrayParams, ParamValid) {
        self.param_get(|p| &p.channel_config)
    }

    /// C++ `getLocalEidParam`.
    #[must_use]
    pub fn get_local_eid_param(&self) -> EntityId {
        self.param(|p| &p.local_eid)
    }

    /// C++ `getOutgoingFileChunkSizeParam`.
    #[must_use]
    pub fn get_outgoing_file_chunk_size_param(&self) -> u32 {
        self.param(|p| &p.outgoing_file_chunk_size)
    }

    /// C++ `getRxCrcCalcBytesPerCycleParam`.
    #[must_use]
    pub fn get_rx_crc_calc_bytes_per_cycle_param(&self) -> u32 {
        self.param(|p| &p.rx_crc_calc_bytes_per_cycle)
    }

    /// C++ `getPostInactivitySendRetriesParam`.
    #[must_use]
    pub fn get_post_inactivity_send_retries_param(&self) -> u8 {
        self.param(|p| &p.post_inactivity_send_retries)
    }

    /// One field of channel `channel`'s `ChannelConfig` entry (asserts the
    /// channel index and the parameter's validity).
    fn channel_param<T>(&self, channel: u8, get: impl FnOnce(&ChannelParams) -> T) -> T {
        fw_assert!(
            usize::from(channel) < NUM_CHANNELS,
            i32::from(channel),
            NUM_CHANNELS as i32
        );
        let p = self.params.lock().unwrap();
        fw_assert!(
            p.channel_config.valid.is_ok(),
            p.channel_config.valid.as_repr() as i32
        );
        get(&p.channel_config.value[usize::from(channel)])
    }

    /// C++ `getTmpDirParam`.
    #[must_use]
    pub fn get_tmp_dir_param(&self, channel: u8) -> PathString {
        self.channel_param(channel, |c| c.tmp_dir.clone())
    }

    /// C++ `getFailDirParam`.
    #[must_use]
    pub fn get_fail_dir_param(&self, channel: u8) -> PathString {
        self.channel_param(channel, |c| c.fail_dir.clone())
    }

    /// C++ `getAckLimitParam`.
    #[must_use]
    pub fn get_ack_limit_param(&self, channel: u8) -> u8 {
        self.channel_param(channel, |c| c.ack_limit)
    }

    /// C++ `getNackLimitParam`.
    #[must_use]
    pub fn get_nack_limit_param(&self, channel: u8) -> u8 {
        self.channel_param(channel, |c| c.nack_limit)
    }

    /// C++ `getAckTimerParam`.
    #[must_use]
    pub fn get_ack_timer_param(&self, channel: u8) -> u32 {
        self.channel_param(channel, |c| c.ack_timer)
    }

    /// C++ `getInactivityTimerParam`.
    #[must_use]
    pub fn get_inactivity_timer_param(&self, channel: u8) -> u32 {
        self.channel_param(channel, |c| c.inactivity_timer)
    }

    /// C++ `getDequeueEnabledParam`.
    #[must_use]
    pub fn get_dequeue_enabled_param(&self, channel: u8) -> Enabled {
        self.channel_param(channel, |c| c.dequeue_enabled)
    }

    /// C++ `getMoveDirParam`.
    #[must_use]
    pub fn get_move_dir_param(&self, channel: u8) -> PathString {
        self.channel_param(channel, |c| c.move_dir.clone())
    }

    /// C++ `getMaxOutgoingPdusPerCycleParam`.
    #[must_use]
    pub fn get_max_outgoing_pdus_per_cycle_param(&self, channel: u8) -> u32 {
        self.channel_param(channel, |c| c.max_outgoing_pdus_per_cycle)
    }

    /// The generated `<PARAM>_PRM_SET` handler.
    fn param_set(&self, id: FwPrmIdType, args: &mut CmdArgBuffer) -> CmdResponse {
        let mut p = self.params.lock().unwrap();
        match id {
            Self::PARAMID_LOCAL_EID => set_param(&mut p.local_eid, args),
            Self::PARAMID_OUTGOING_FILE_CHUNK_SIZE => {
                set_param(&mut p.outgoing_file_chunk_size, args)
            }
            Self::PARAMID_RX_CRC_CALC_BYTES_PER_CYCLE => {
                set_param(&mut p.rx_crc_calc_bytes_per_cycle, args)
            }
            Self::PARAMID_POST_INACTIVITY_SEND_RETRIES => {
                set_param(&mut p.post_inactivity_send_retries, args)
            }
            Self::PARAMID_FILE_IN_DEFAULT_CHANNEL => {
                set_param(&mut p.file_in_default_channel, args)
            }
            Self::PARAMID_FILE_IN_DEFAULT_DEST_ENTITY_ID => {
                set_param(&mut p.file_in_default_dest_entity_id, args)
            }
            Self::PARAMID_FILE_IN_DEFAULT_CLASS => set_param(&mut p.file_in_default_class, args),
            Self::PARAMID_FILE_IN_DEFAULT_KEEP => set_param(&mut p.file_in_default_keep, args),
            Self::PARAMID_FILE_IN_DEFAULT_PRIORITY => {
                set_param(&mut p.file_in_default_priority, args)
            }
            _ => set_param(&mut p.channel_config, args),
        }
    }

    /// The generated `<PARAM>_PRM_SAVE` handler: push the value through
    /// `prmSetOut` (`EXECUTION_ERROR` when unconnected).
    fn param_save(&self, id: FwPrmIdType, args: &CmdArgBuffer) -> CmdResponse {
        if args.deserialize_size_left() != 0 {
            return CmdResponse::FormatError;
        }
        if !self.prm.prm_set_out.is_connected() {
            return CmdResponse::ExecutionError;
        }
        let mut buf = ParamBuffer::new();
        let status = {
            let p = self.params.lock().unwrap();
            let value: &dyn Serialize = match id {
                Self::PARAMID_LOCAL_EID => &p.local_eid.value,
                Self::PARAMID_OUTGOING_FILE_CHUNK_SIZE => &p.outgoing_file_chunk_size.value,
                Self::PARAMID_RX_CRC_CALC_BYTES_PER_CYCLE => &p.rx_crc_calc_bytes_per_cycle.value,
                Self::PARAMID_POST_INACTIVITY_SEND_RETRIES => &p.post_inactivity_send_retries.value,
                Self::PARAMID_FILE_IN_DEFAULT_CHANNEL => &p.file_in_default_channel.value,
                Self::PARAMID_FILE_IN_DEFAULT_DEST_ENTITY_ID => {
                    &p.file_in_default_dest_entity_id.value
                }
                Self::PARAMID_FILE_IN_DEFAULT_CLASS => &p.file_in_default_class.value,
                Self::PARAMID_FILE_IN_DEFAULT_KEEP => &p.file_in_default_keep.value,
                Self::PARAMID_FILE_IN_DEFAULT_PRIORITY => &p.file_in_default_priority.value,
                _ => &p.channel_config.value,
            };
            value.serialize_to(&mut buf, Endianness::Big)
        };
        fw_assert!(status.is_ok(), status as i32);
        self.prm.set_param(self.id_base(), id, &mut buf);
        CmdResponse::Ok
    }

    // -- port calls made by the engine (C++ cf_cfdp_sbintf analogues) --------

    /// C++ `getPduBuffer`: a buffer of exactly `size` bytes from
    /// `bufferAllocate[channel]`, unless the channel's per-cycle PDU budget
    /// is spent (`SEND_PDU_NO_BUF_AVAIL_ERROR`, no allocation) or the
    /// allocator comes back short (`BuffersExhausted`, the buffer dropped).
    pub(crate) fn get_pdu_buffer(
        &self,
        channel: u8,
        outgoing_counter: &mut u32,
        size: usize,
    ) -> (Status, Buffer) {
        let max_pdus = self.get_max_outgoing_pdus_per_cycle_param(channel);
        if *outgoing_counter >= max_pdus {
            return (Status::SendPduNoBufAvailError, Buffer::empty());
        }
        let p = self.buffer_allocate[usize::from(channel)].get();
        let buffer = p.target.invoke(p.port_num, size as FwSizeType);
        if buffer.size() == size {
            *outgoing_counter += 1;
            (Status::Success, buffer)
        } else {
            self.log_warning_lo_buffers_exhausted();
            (Status::SendPduNoBufAvailError, Buffer::empty())
        }
    }

    /// C++ `returnPduBuffer`: an unsent buffer back on `bufferDeallocate`.
    pub(crate) fn return_pdu_buffer(&self, channel: u8, buffer: Buffer) {
        let p = self.buffer_deallocate[usize::from(channel)].get();
        p.target.invoke(p.port_num, buffer);
    }

    /// C++ `sendPduBuffer`: write the `FW_PACKET_FILE` descriptor in front
    /// of the encoded PDU and send it on `dataOut`.
    pub(crate) fn send_pdu_buffer(&self, channel: u8, mut buffer: Buffer) {
        let descriptor = ComPacketType::FwPacketFile.as_repr();
        buffer.data_mut()[..PACKET_DESCRIPTOR_SIZE].copy_from_slice(&descriptor.to_be_bytes());
        let p = self.data_out[usize::from(channel)].get();
        p.target.invoke(p.port_num, buffer);
    }

    /// C++ `sendFileComplete`: report a `fileIn` transfer (context 0).
    pub(crate) fn send_file_complete(&self, status: SendFileStatus) {
        let p = self.file_done_out.get();
        p.target
            .invoke(p.port_num, SendFileResponse::new(status, 0));
    }

    // -- input-port factories ------------------------------------------------

    /// `cmdIn` — the command port.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `dataIn[port_num]` — ASYNC `Fw.BufferSend`: received PDUs.
    pub fn data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(
            Arc::new(BufferInAdapter {
                comp: self.clone(),
                msg_type: Self::MSG_TYPE_DATA_IN,
            }),
            port_num,
        )
    }

    /// `dataReturnIn[port_num]` — ASYNC `Fw.BufferSend`: buffers sent on
    /// `dataOut` coming back.
    pub fn data_return_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(
            Arc::new(BufferInAdapter {
                comp: self.clone(),
                msg_type: Self::MSG_TYPE_DATA_RETURN_IN,
            }),
            port_num,
        )
    }

    /// `fileIn` — GUARDED `Svc.SendFileRequest`.
    pub fn file_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SendFileRequestPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- handlers ------------------------------------------------------------

    /// `run1Hz_handler`: drain `fileIn` requests, cycle the engine, write
    /// the telemetry.
    fn run_1hz_handler(&self, _port_num: FwIndexType, _context: u32) {
        let tlm = self.with_engine(|engine| {
            self.drain_file_in_queue(engine);
            engine.cycle(self);
            channel_telemetry(engine)
        });
        self.tlm_write_channel_telemetry(&tlm);
    }

    /// C++ `drainFileInQueue`: start every queued `fileIn` request with the
    /// `FileInDefault*` parameters on the component thread; a failed start
    /// (or an out-of-range default channel) is reported on `fileDoneOut`.
    pub(crate) fn drain_file_in_queue(&self, engine: &mut Engine) {
        let depth = *self.file_in_queue_depth.lock().unwrap();
        for _ in 0..depth {
            let mut msg = LinearBuffer::<FILE_IN_REQUEST_SIZE>::new();
            let mut priority: FwQueuePriorityType = 0;
            let status = self.file_in_queue.receive_serial(
                &mut msg,
                BlockingType::NonBlocking,
                &mut priority,
            );
            if status != QueueStatus::OpOk {
                break;
            }
            let mut source = FwDefaultString::new();
            let mut dest = FwDefaultString::new();
            let mut context = 0u32;
            if !msg.deserialize(&mut source, Endianness::Big).is_ok()
                || !msg.deserialize(&mut dest, Endianness::Big).is_ok()
                || !msg.deserialize_u32_be(&mut context).is_ok()
            {
                break;
            }
            let channel_id = self.param(|p| &p.file_in_default_channel);
            if usize::from(channel_id) >= NUM_CHANNELS {
                self.log_warning_lo_invalid_channel(channel_id, NUM_CHANNELS as u8);
                self.send_file_complete(SendFileStatus::StatusInvalid);
                continue;
            }
            let dest_eid = self.param(|p| &p.file_in_default_dest_entity_id);
            let cfdp_class = self.param(|p| &p.file_in_default_class);
            let keep = self.param(|p| &p.file_in_default_keep);
            let priority = self.param(|p| &p.file_in_default_priority);
            let status = engine.tx_file(
                self,
                source.as_bytes(),
                dest.as_bytes(),
                cfdp_class,
                keep,
                channel_id,
                priority,
                dest_eid,
                TransactionInitType::InitByPort,
            );
            if status != Status::Success {
                self.log_warning_lo_send_file_initiate_fail(source.as_bytes());
                self.send_file_complete(SendFileStatus::StatusError);
            }
        }
    }

    /// `pingIn_handler`.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `dataReturnIn_handler`: the buffer goes back to its allocator on the
    /// same port number.
    fn data_return_in_handler(&self, port_num: FwIndexType, buffer: Buffer) {
        fw_assert!(
            (port_num as usize) < NUM_CHANNELS,
            i32::from(port_num),
            NUM_CHANNELS as i32
        );
        let p = self.buffer_deallocate[port_num as usize].get();
        p.target.invoke(p.port_num, buffer);
    }

    /// `dataIn_handler`: strip the `FW_PACKET_FILE` descriptor (silently
    /// ignoring short buffers and other packet types), hand the PDU to the
    /// engine on channel `port_num`, and return the buffer.
    fn data_in_handler(&self, port_num: FwIndexType, buffer: Buffer) {
        fw_assert!(
            (port_num as usize) < NUM_CHANNELS,
            i32::from(port_num),
            NUM_CHANNELS as i32
        );
        fw_assert!(port_num >= 0, i32::from(port_num));
        let data = buffer.data();
        if data.len() >= PACKET_DESCRIPTOR_SIZE
            && u16::from_be_bytes([data[0], data[1]]) == ComPacketType::FwPacketFile.as_repr()
        {
            let pdu = &data[PACKET_DESCRIPTOR_SIZE..];
            self.with_engine(|engine| engine.receive_pdu(self, port_num as u8, pdu));
        }
        let p = self.data_in_return[port_num as usize].get();
        p.target.invoke(p.port_num, buffer);
    }

    // -- commands ------------------------------------------------------------

    /// C++ `checkCommandChannelIndex`.
    fn check_command_channel_index(&self, channel_index: u8) -> CmdResponse {
        if usize::from(channel_index) >= NUM_CHANNELS {
            self.log_warning_lo_invalid_channel(channel_index, NUM_CHANNELS as u8);
            CmdResponse::ValidationError
        } else {
            CmdResponse::Ok
        }
    }

    /// C++ `checkCommandChannelPollIndex`.
    fn check_command_channel_poll_index(&self, poll_index: u8) -> CmdResponse {
        if usize::from(poll_index) >= MAX_POLLING_DIR_PER_CHAN {
            self.log_warning_lo_invalid_channel_poll(poll_index, MAX_POLLING_DIR_PER_CHAN as u8);
            CmdResponse::ValidationError
        } else {
            CmdResponse::Ok
        }
    }

    /// C++ `checkCommandPollInterval`: a zero interval is rejected.
    fn check_command_poll_interval(&self, interval: u32) -> CmdResponse {
        if interval == 0 {
            self.log_warning_lo_invalid_poll_interval(interval);
            CmdResponse::ValidationError
        } else {
            CmdResponse::Ok
        }
    }

    /// `SendFile_cmdHandler`.
    #[allow(clippy::too_many_arguments)]
    fn send_file_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        dest_id: EntityId,
        cfdp_class: Class,
        keep: Keep,
        priority: u8,
        source_file_name: &CmdStringArg,
        dest_file_name: &CmdStringArg,
    ) {
        let mut rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            let status = self.with_engine(|engine| {
                engine.tx_file(
                    self,
                    source_file_name.as_bytes(),
                    dest_file_name.as_bytes(),
                    cfdp_class,
                    keep,
                    channel_id,
                    priority,
                    dest_id,
                    TransactionInitType::InitByCommand,
                )
            });
            if status != Status::Success {
                rsp = CmdResponse::ExecutionError;
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `PlaybackDirectory_cmdHandler`.
    #[allow(clippy::too_many_arguments)]
    fn playback_directory_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        dest_id: EntityId,
        cfdp_class: Class,
        keep: Keep,
        priority: u8,
        source_directory: &CmdStringArg,
        dest_directory: &CmdStringArg,
    ) {
        let mut rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            let status = self.with_engine(|engine| {
                engine.playback_dir(
                    self,
                    source_directory.as_bytes(),
                    dest_directory.as_bytes(),
                    cfdp_class,
                    keep,
                    channel_id,
                    priority,
                    dest_id,
                )
            });
            if status == Status::Success {
                self.log_activity_lo_playback_initiated(source_directory.as_bytes());
            } else {
                rsp = CmdResponse::ExecutionError;
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `PollDirectory_cmdHandler`.
    #[allow(clippy::too_many_arguments)]
    fn poll_directory_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        poll_id: u8,
        dest_id: EntityId,
        cfdp_class: Class,
        priority: u8,
        interval: u32,
        source_directory: &CmdStringArg,
        dest_directory: &CmdStringArg,
    ) {
        let mut rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            rsp = self.check_command_channel_poll_index(poll_id);
        }
        if rsp == CmdResponse::Ok {
            rsp = self.check_command_poll_interval(interval);
        }
        if rsp == CmdResponse::Ok {
            let status = self.with_engine(|engine| {
                engine.start_poll_dir(
                    self,
                    channel_id,
                    poll_id,
                    source_directory.as_bytes(),
                    dest_directory.as_bytes(),
                    cfdp_class,
                    priority,
                    dest_id,
                    interval,
                )
            });
            if status == Status::Success {
                self.log_activity_lo_poll_dir_initiated(source_directory.as_bytes(), poll_id);
            } else {
                rsp = CmdResponse::ExecutionError;
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `StopPollDirectory_cmdHandler`: a failed stop (slot not active) is
    /// reported by event but not failed (C++ parity).
    fn stop_poll_directory_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        poll_id: u8,
    ) {
        let mut rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            rsp = self.check_command_channel_poll_index(poll_id);
        }
        if rsp == CmdResponse::Ok
            && self.with_engine(|engine| engine.stop_poll_dir(self, channel_id, poll_id))
                == Status::Success
        {
            self.log_activity_lo_poll_dir_stopped(channel_id, poll_id);
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `SetChannelFlow_cmdHandler`.
    fn set_channel_flow_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        flow_state: Flow,
    ) {
        let rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            self.with_engine(|engine| engine.set_channel_flow_state(channel_id, flow_state));
            self.log_activity_lo_set_flow_state(channel_id, flow_state);
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `SuspendResumeTransaction_cmdHandler`.
    fn suspend_resume_transaction_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
        action: SuspendResume,
    ) {
        let mut rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            let status = self.with_engine(|engine| {
                engine.set_suspend_resume_transaction(
                    channel_id,
                    transaction_seq,
                    entity_id,
                    action,
                )
            });
            if status == Status::Success {
                if action == SuspendResume::Suspend {
                    self.log_activity_lo_transaction_suspended(transaction_seq, entity_id);
                } else {
                    self.log_activity_lo_transaction_resumed(transaction_seq, entity_id);
                }
            } else {
                self.log_warning_lo_transaction_not_found(transaction_seq, entity_id);
                rsp = CmdResponse::ExecutionError;
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `CancelTransaction_cmdHandler`.
    fn cancel_transaction_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) {
        let mut rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            let status = self.with_engine(|engine| {
                engine.cancel_transaction_by_seq(self, channel_id, transaction_seq, entity_id)
            });
            if status == Status::Success {
                self.log_activity_hi_transaction_canceled(transaction_seq, entity_id);
            } else {
                self.log_warning_lo_transaction_not_found(transaction_seq, entity_id);
                rsp = CmdResponse::ExecutionError;
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `AbandonTransaction_cmdHandler`.
    fn abandon_transaction_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        channel_id: u8,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) {
        let mut rsp = self.check_command_channel_index(channel_id);
        if rsp == CmdResponse::Ok {
            let status = self.with_engine(|engine| {
                engine.abandon_transaction(self, channel_id, transaction_seq, entity_id)
            });
            if status == Status::Success {
                self.log_activity_hi_transaction_abandoned(transaction_seq, entity_id);
            } else {
                self.log_warning_lo_transaction_not_found(transaction_seq, entity_id);
                rsp = CmdResponse::ExecutionError;
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, rsp);
    }

    /// `ResetCounters_cmdHandler`: one channel, or all with `0xFF`; the
    /// telemetry is written at once. An invalid channel reports
    /// `NumChannels - 1` as the maximum (C++ parity, unlike the other
    /// commands).
    fn reset_counters_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, channel_id: u8) {
        let tlm = self.with_engine(|engine| {
            if channel_id == 0xFF {
                for chan in engine.channels.iter_mut() {
                    chan.tlm = ChannelTelemetry::default();
                }
                Some(channel_telemetry(engine))
            } else if usize::from(channel_id) < NUM_CHANNELS {
                engine.channels[usize::from(channel_id)].tlm = ChannelTelemetry::default();
                Some(channel_telemetry(engine))
            } else {
                None
            }
        });
        match tlm {
            Some(tlm) => {
                self.log_activity_hi_reset_counters(channel_id);
                self.tlm_write_channel_telemetry(&tlm);
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            None => {
                self.log_warning_lo_invalid_channel(channel_id, NUM_CHANNELS as u8 - 1);
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            }
        }
    }

    /// The generated async command dispatch: deserialize the arguments in
    /// declaration order (`FORMAT_ERROR` on a failure or a leftover byte,
    /// `VALIDATION_ERROR` on an undeclared enum value) and call the handler.
    fn cmd_in_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let respond = |response| self.cmd.cmd_response(op_code, cmd_seq, response);
        let local = op_code.wrapping_sub(self.id_base());
        let mut r = CmdReader { args, error: None };
        match local {
            Self::OPCODE_SEND_FILE | Self::OPCODE_PLAYBACK_DIRECTORY => {
                let channel_id = r.u8();
                let dest_id = r.u32();
                let cfdp_class = r.enum_u8::<Class>();
                let keep = r.enum_u8::<Keep>();
                let priority = r.u8();
                let src = r.string();
                let dst = r.string();
                if let Some(rsp) = r.finish() {
                    respond(rsp);
                } else if local == Self::OPCODE_SEND_FILE {
                    self.send_file_cmd_handler(
                        op_code, cmd_seq, channel_id, dest_id, cfdp_class, keep, priority, &src,
                        &dst,
                    );
                } else {
                    self.playback_directory_cmd_handler(
                        op_code, cmd_seq, channel_id, dest_id, cfdp_class, keep, priority, &src,
                        &dst,
                    );
                }
            }
            Self::OPCODE_POLL_DIRECTORY => {
                let channel_id = r.u8();
                let poll_id = r.u8();
                let dest_id = r.u32();
                let cfdp_class = r.enum_u8::<Class>();
                let priority = r.u8();
                let interval = r.u32();
                let src = r.string();
                let dst = r.string();
                match r.finish() {
                    Some(rsp) => respond(rsp),
                    None => self.poll_directory_cmd_handler(
                        op_code, cmd_seq, channel_id, poll_id, dest_id, cfdp_class, priority,
                        interval, &src, &dst,
                    ),
                }
            }
            Self::OPCODE_STOP_POLL_DIRECTORY => {
                let channel_id = r.u8();
                let poll_id = r.u8();
                match r.finish() {
                    Some(rsp) => respond(rsp),
                    None => {
                        self.stop_poll_directory_cmd_handler(op_code, cmd_seq, channel_id, poll_id)
                    }
                }
            }
            Self::OPCODE_SET_CHANNEL_FLOW => {
                let channel_id = r.u8();
                let flow = r.enum_i32::<Flow>();
                match r.finish() {
                    Some(rsp) => respond(rsp),
                    None => self.set_channel_flow_cmd_handler(op_code, cmd_seq, channel_id, flow),
                }
            }
            Self::OPCODE_SUSPEND_RESUME_TRANSACTION => {
                let channel_id = r.u8();
                let seq = r.u32();
                let eid = r.u32();
                let action = r.enum_u8::<SuspendResume>();
                match r.finish() {
                    Some(rsp) => respond(rsp),
                    None => self.suspend_resume_transaction_cmd_handler(
                        op_code, cmd_seq, channel_id, seq, eid, action,
                    ),
                }
            }
            Self::OPCODE_CANCEL_TRANSACTION | Self::OPCODE_ABANDON_TRANSACTION => {
                let channel_id = r.u8();
                let seq = r.u32();
                let eid = r.u32();
                match r.finish() {
                    Some(rsp) => respond(rsp),
                    None if local == Self::OPCODE_CANCEL_TRANSACTION => {
                        self.cancel_transaction_cmd_handler(op_code, cmd_seq, channel_id, seq, eid);
                    }
                    None => {
                        self.abandon_transaction_cmd_handler(
                            op_code, cmd_seq, channel_id, seq, eid,
                        );
                    }
                }
            }
            Self::OPCODE_RESET_COUNTERS => {
                let channel_id = r.u8();
                match r.finish() {
                    Some(rsp) => respond(rsp),
                    None => self.reset_counters_cmd_handler(op_code, cmd_seq, channel_id),
                }
            }
            _ => respond(CmdResponse::InvalidOpcode),
        }
    }

    // -- telemetry -----------------------------------------------------------

    /// `tlmWrite_ChannelTelemetry` (written every time).
    fn tlm_write_channel_telemetry(&self, value: &ChannelTelemetryArray) {
        self.tlm.tlm_write(
            self.id_base(),
            Self::CHANID_CHANNEL_TELEMETRY,
            value,
            self.evt.time_get(),
        );
    }
}

/// The channels' counters as the telemetry array (C++
/// `m_channelTelemetry`).
fn channel_telemetry(engine: &Engine) -> ChannelTelemetryArray {
    ChannelTelemetryArray(core::array::from_fn(|i| engine.channels[i].tlm))
}

/// Command-argument reader: the first failure decides the response.
struct CmdReader<'a> {
    args: &'a mut CmdArgBuffer,
    error: Option<CmdResponse>,
}

impl CmdReader<'_> {
    fn fail(&mut self, rsp: CmdResponse) {
        if self.error.is_none() {
            self.error = Some(rsp);
        }
    }

    fn u8(&mut self) -> u8 {
        let mut v = 0u8;
        if self.error.is_none() && !self.args.deserialize_u8_be(&mut v).is_ok() {
            self.fail(CmdResponse::FormatError);
        }
        v
    }

    fn u32(&mut self) -> u32 {
        let mut v = 0u32;
        if self.error.is_none() && !self.args.deserialize_u32_be(&mut v).is_ok() {
            self.fail(CmdResponse::FormatError);
        }
        v
    }

    fn i32(&mut self) -> i32 {
        let mut v = 0i32;
        if self.error.is_none() && !self.args.deserialize_i32_be(&mut v).is_ok() {
            self.fail(CmdResponse::FormatError);
        }
        v
    }

    fn enum_u8<E: TryFrom<u8> + Default>(&mut self) -> E {
        let raw = self.u8();
        if self.error.is_some() {
            return E::default();
        }
        E::try_from(raw).unwrap_or_else(|_| {
            self.fail(CmdResponse::ValidationError);
            E::default()
        })
    }

    fn enum_i32<E: TryFrom<i32> + Default>(&mut self) -> E {
        let raw = self.i32();
        if self.error.is_some() {
            return E::default();
        }
        E::try_from(raw).unwrap_or_else(|_| {
            self.fail(CmdResponse::ValidationError);
            E::default()
        })
    }

    fn string(&mut self) -> CmdStringArg {
        let mut s = CmdStringArg::new();
        if self.error.is_none() && !self.args.deserialize(&mut s, Endianness::Big).is_ok() {
            self.fail(CmdResponse::FormatError);
        }
        s
    }

    /// The response to send instead of running the handler, if any (a
    /// leftover byte is a `FORMAT_ERROR`).
    fn finish(&mut self) -> Option<CmdResponse> {
        if self.error.is_none() && self.args.deserialize_size_left() != 0 {
            self.fail(CmdResponse::FormatError);
        }
        self.error
    }
}

impl CmdPort for CfdpManager {
    /// The generated `cmdIn` handler: the parameter commands run here, on
    /// the caller's thread; every other command is queued (default
    /// priority, `assert` policy).
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let local = op_code.wrapping_sub(self.id_base());
        let response = match local {
            Self::OPCODE_SEND_FILE..=Self::OPCODE_RESET_COUNTERS => None,
            Self::OPCODE_LOCALEID_PRM_SET..=Self::OPCODE_CHANNELCONFIG_PRM_SAVE => {
                let index = local - Self::OPCODE_LOCALEID_PRM_SET;
                let id = index / 2;
                Some(if index % 2 == 0 {
                    self.param_set(id, args)
                } else {
                    self.param_save(id, args)
                })
            }
            _ => Some(CmdResponse::InvalidOpcode),
        };
        if let Some(response) = response {
            self.cmd.cmd_response(op_code, cmd_seq, response);
            return;
        }
        let mut buf = LinearBuffer::<QUEUE_MSG_SIZE>::new();
        let status = msg::write_envelope_header(&mut buf, Self::MSG_TYPE_CMD_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .active
            .queued
            .send_message(&buf, PRIORITY, QueueFullPolicy::Assert);
    }
}

impl SendFileRequestPort for CfdpManager {
    /// `fileIn_handler` (GUARDED): validate and queue the request for the
    /// component thread. The response only reports acceptance (`OK`),
    /// a full queue (`BUSY`), or a partial-file request or over-long names
    /// (`INVALID`); the result arrives later on `fileDoneOut`.
    fn invoke(
        &self,
        port_num: FwIndexType,
        source_file_name: &SendFileNameArg,
        dest_file_name: &SendFileNameArg,
        offset: u32,
        length: u32,
    ) -> SendFileResponse {
        let _guard = self.guard.lock().unwrap();
        let context = port_num as u32;
        // Partial-file retransmit is not supported.
        if offset > 0 || length > 0 {
            self.log_warning_lo_unsupported_send_file_arguments(offset, length);
            return SendFileResponse::new(SendFileStatus::StatusInvalid, context);
        }
        if source_file_name.len() >= FwDefaultString::CAPACITY
            || dest_file_name.len() >= FwDefaultString::CAPACITY
        {
            self.log_warning_lo_send_file_initiate_fail(source_file_name.as_bytes());
            return SendFileResponse::new(SendFileStatus::StatusInvalid, context);
        }
        let mut msg = LinearBuffer::<FILE_IN_REQUEST_SIZE>::new();
        let mut source = FwDefaultString::new();
        source.set_bytes(source_file_name.as_bytes());
        let mut dest = FwDefaultString::new();
        dest.set_bytes(dest_file_name.as_bytes());
        let status = fw_try_status(&mut msg, &source, &dest, context);
        fw_assert!(status.is_ok(), status as i32);
        let status = self
            .file_in_queue
            .send_serial(&msg, 0, BlockingType::NonBlocking);
        if status != QueueStatus::OpOk {
            self.log_warning_lo_send_file_initiate_fail(source_file_name.as_bytes());
            SendFileResponse::new(SendFileStatus::StatusBusy, context)
        } else {
            SendFileResponse::new(SendFileStatus::StatusOk, context)
        }
    }
}

/// Serialize a `fileIn` request.
fn fw_try_status(
    msg: &mut dyn SerBufAny,
    source: &FwDefaultString,
    dest: &FwDefaultString,
    context: u32,
) -> SerializeStatus {
    fw_try!(source.serialize_to(msg, Endianness::Big));
    fw_try!(dest.serialize_to(msg, Endianness::Big));
    msg.serialize_u32_be(context)
}

// ---------------------------------------------------------------------------
// Async input adapters and dispatch.
// ---------------------------------------------------------------------------

async_input_port_adapter! {
    /// `run1Hz` — ASYNC `Svc.Sched` input: must be invoked at 1 Hz.
    component: CfdpManager;
    adapter: Run1HzAdapter;
    port: SchedPort;
    input: pub run_1hz;
    deserialize: run_1hz_deserialize;
    handler: run_1hz_handler;
    base: active.queued;
    msg_type: CfdpManager::MSG_TYPE_RUN_1HZ;
    msg_size: QUEUE_MSG_SIZE;
    priority: PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val context: u32 }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input (drop).
    component: CfdpManager;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: CfdpManager::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    args { val key: u32 }
}

/// The `dataIn` / `dataReturnIn` adapter: the buffer is deposited in the
/// escrow and the token serialized where C++ serializes the `Fw::Buffer`.
struct BufferInAdapter {
    comp: Arc<CfdpManager>,
    msg_type: i32,
}

impl BufferSendPort for BufferInAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut buf = LinearBuffer::<QUEUE_MSG_SIZE>::new();
        let status = msg::write_envelope_header(&mut buf, self.msg_type, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PRIORITY, QueueFullPolicy::Assert);
    }
}

impl ComponentDispatch for CfdpManager {
    fn dispatch_message(
        &self,
        msg_type: fprime_config::FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            Self::MSG_TYPE_RUN_1HZ => match Self::run_1hz_deserialize(buf) {
                Some((context,)) => {
                    self.run_1hz_handler(port_num, context);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_PING_IN => match Self::ping_in_deserialize(buf) {
                Some((key,)) => {
                    self.ping_in_handler(port_num, key);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_DATA_RETURN_IN | Self::MSG_TYPE_DATA_IN => {
                let mut token = 0u64;
                if !buf.deserialize_u64_be(&mut token).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                if msg_type == Self::MSG_TYPE_DATA_IN {
                    self.data_in_handler(port_num, buffer);
                } else {
                    self.data_return_in_handler(port_num, buffer);
                }
                MsgDispatchStatus::Ok
            }
            Self::MSG_TYPE_CMD_IN => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok()
                    || !buf.deserialize_u32_be(&mut cmd_seq).is_ok()
                    || !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.cmd_in_handler(op_code, cmd_seq, &mut args);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for CfdpManager {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Events (Events.fppi).
// ---------------------------------------------------------------------------

/// A text-event line built without allocating: formatted text truncated to
/// `FW_LOG_TEXT_BUFFER_SIZE` bytes on a character boundary.
struct EventText {
    buf: [u8; FW_LOG_TEXT_BUFFER_SIZE],
    len: usize,
}

impl EventText {
    fn new(args: fmt::Arguments<'_>) -> Self {
        let mut text = Self {
            buf: [0; FW_LOG_TEXT_BUFFER_SIZE],
            len: 0,
        };
        let _ = text.write_fmt(args);
        text
    }

    fn as_str(&self) -> &str {
        // Only whole characters are ever copied in.
        std::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl fmt::Write for EventText {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            let n = c.len_utf8();
            if self.len + n > self.buf.len() {
                break;
            }
            c.encode_utf8(&mut self.buf[self.len..self.len + n]);
            self.len += n;
        }
        Ok(())
    }
}

/// Lossy display of a byte string argument.
struct Lossy<'a>(&'a [u8]);

impl fmt::Display for Lossy<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let n = self.0.len().min(EVENT_STRING_SIZE);
        for chunk in self.0[..n].utf8_chunks() {
            f.write_str(chunk.valid())?;
            if !chunk.invalid().is_empty() {
                f.write_str("\u{FFFD}")?;
            }
        }
        Ok(())
    }
}

/// Serialize a `string size MaxFilePathSize` event argument, truncated as
/// the generated code does.
fn ser_str(buf: &mut LogBuffer, bytes: &[u8]) -> SerializeStatus {
    let n = bytes.len().min(EVENT_STRING_SIZE);
    buf.serialize_bytes(&bytes[..n], LengthMode::IncludeLength, Endianness::Big)
}

impl CfdpManager {
    fn log(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        text: fmt::Arguments<'_>,
        write: impl FnOnce(&mut LogBuffer) -> SerializeStatus,
    ) {
        let text = EventText::new(text);
        self.evt
            .log_event(self.id_base(), id, severity, text.as_str(), write);
    }
}

impl CfdpManager {
    /// `BuffersExhausted()` — WARNING_LO.
    pub const EVENTID_BUFFERS_EXHAUSTED: FwEventIdType = 0;
    /// `InvalidChannel(channelId, maxChannelId)` — WARNING_LO.
    pub const EVENTID_INVALID_CHANNEL: FwEventIdType = 1;
    /// `TxFileQueued(sourceFileName, transactionSeq)` — ACTIVITY_LO.
    pub const EVENTID_TX_FILE_QUEUED: FwEventIdType = 2;
    /// `UnsupportedSendFileArguments(offset, length)` — WARNING_LO.
    pub const EVENTID_UNSUPPORTED_SEND_FILE_ARGUMENTS: FwEventIdType = 3;
    /// `SendFileInitiateFail(sourceFileName)` — WARNING_LO.
    pub const EVENTID_SEND_FILE_INITIATE_FAIL: FwEventIdType = 4;
    /// `PlaybackInitiated(sourceDirectory)` — ACTIVITY_LO.
    pub const EVENTID_PLAYBACK_INITIATED: FwEventIdType = 5;
    /// `PollDirInitiated(sourceDirectory, pollId)` — ACTIVITY_LO.
    pub const EVENTID_POLL_DIR_INITIATED: FwEventIdType = 6;
    /// `PollDirStopped(channelId, pollId)` — ACTIVITY_LO.
    pub const EVENTID_POLL_DIR_STOPPED: FwEventIdType = 7;
    /// `PollDirBusy(channelId, pollId)` — WARNING_LO.
    pub const EVENTID_POLL_DIR_BUSY: FwEventIdType = 8;
    /// `PollDirNotActive(channelId, pollId)` — WARNING_LO.
    pub const EVENTID_POLL_DIR_NOT_ACTIVE: FwEventIdType = 9;
    /// `SetFlowState(channelId, flowState)` — ACTIVITY_LO.
    pub const EVENTID_SET_FLOW_STATE: FwEventIdType = 10;
    /// `InvalidChannelPoll(pollId, maxPollId)` — WARNING_LO.
    pub const EVENTID_INVALID_CHANNEL_POLL: FwEventIdType = 11;
    /// `InvalidPollInterval(interval)` — WARNING_LO.
    pub const EVENTID_INVALID_POLL_INTERVAL: FwEventIdType = 12;
    /// `FailKeepFileMove(srcFile, moveDir, status)` — WARNING_LO.
    pub const EVENTID_FAIL_KEEP_FILE_MOVE: FwEventIdType = 13;
    /// `FailPollFileMove(srcFile, failDir, status)` — WARNING_LO.
    pub const EVENTID_FAIL_POLL_FILE_MOVE: FwEventIdType = 14;
    /// `FailPduHeaderDeserialization(channelId, status)` — WARNING_LO.
    pub const EVENTID_FAIL_PDU_HEADER_DESERIALIZATION: FwEventIdType = 15;
    /// `FailPduSerialization(channelId, pduType, status)` — WARNING_LO.
    pub const EVENTID_FAIL_PDU_SERIALIZATION: FwEventIdType = 16;
    /// `FailMetadataPduDeserialization(channelId, status)` — WARNING_LO.
    pub const EVENTID_FAIL_METADATA_PDU_DESERIALIZATION: FwEventIdType = 17;
    /// `FailFileDataPduDeserialization(channelId, status)` — WARNING_LO.
    pub const EVENTID_FAIL_FILE_DATA_PDU_DESERIALIZATION: FwEventIdType = 18;
    /// `FailEofPduDeserialization(channelId, status)` — WARNING_LO.
    pub const EVENTID_FAIL_EOF_PDU_DESERIALIZATION: FwEventIdType = 19;
    /// `FailAckPduDeserialization(channelId, status)` — WARNING_LO.
    pub const EVENTID_FAIL_ACK_PDU_DESERIALIZATION: FwEventIdType = 20;
    /// `FailFinPduDeserialization(channelId, status)` — WARNING_LO.
    pub const EVENTID_FAIL_FIN_PDU_DESERIALIZATION: FwEventIdType = 21;
    /// `FailNakPduDeserialization(channelId, status)` — WARNING_LO.
    pub const EVENTID_FAIL_NAK_PDU_DESERIALIZATION: FwEventIdType = 22;
    /// `RxAckLimitReached(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_RX_ACK_LIMIT_REACHED: FwEventIdType = 23;
    /// `RxTempFileCreated(cfdpClass, srcEid, seqNum, filename)` — ACTIVITY_LO.
    pub const EVENTID_RX_TEMP_FILE_CREATED: FwEventIdType = 24;
    /// `RxFileCreateFailed(cfdpClass, srcEid, seqNum, filename, status)` — WARNING_LO.
    pub const EVENTID_RX_FILE_CREATE_FAILED: FwEventIdType = 25;
    /// `RxCrcMismatch(cfdpClass, srcEid, seqNum, expected, actual)` — WARNING_LO.
    pub const EVENTID_RX_CRC_MISMATCH: FwEventIdType = 26;
    /// `RxNakLimitReached(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_RX_NAK_LIMIT_REACHED: FwEventIdType = 27;
    /// `RxSeekFailed(cfdpClass, srcEid, seqNum, offset, status)` — WARNING_LO.
    pub const EVENTID_RX_SEEK_FAILED: FwEventIdType = 28;
    /// `RxWriteFailed(cfdpClass, srcEid, seqNum, expected, actual)` — WARNING_LO.
    pub const EVENTID_RX_WRITE_FAILED: FwEventIdType = 29;
    /// `RxFileDataOutOfBounds(cfdpClass, srcEid, seqNum, offset, dataSize, fileSize)` — WARNING_LO.
    pub const EVENTID_RX_FILE_DATA_OUT_OF_BOUNDS: FwEventIdType = 30;
    /// `RxFileSizeMismatch(cfdpClass, srcEid, seqNum, expected, actual)` — WARNING_LO.
    pub const EVENTID_RX_FILE_SIZE_MISMATCH: FwEventIdType = 31;
    /// `RxEofCancelReceived(cfdpClass, srcEid, seqNum)` — ACTIVITY_HI.
    pub const EVENTID_RX_EOF_CANCEL_RECEIVED: FwEventIdType = 32;
    /// `RxEofWithError(cfdpClass, srcEid, seqNum, conditionCode)` — WARNING_LO.
    pub const EVENTID_RX_EOF_WITH_ERROR: FwEventIdType = 33;
    /// `RxSeekCrcFailed(cfdpClass, srcEid, seqNum, offset, status)` — WARNING_LO.
    pub const EVENTID_RX_SEEK_CRC_FAILED: FwEventIdType = 34;
    /// `RxReadCrcFailed(cfdpClass, srcEid, seqNum, expected, actual)` — WARNING_LO.
    pub const EVENTID_RX_READ_CRC_FAILED: FwEventIdType = 35;
    /// `RxEofMdSizeMismatch(cfdpClass, srcEid, seqNum, mdSize, eofSize)` — WARNING_LO.
    pub const EVENTID_RX_EOF_MD_SIZE_MISMATCH: FwEventIdType = 36;
    /// `RxFileRenameFailed(cfdpClass, srcEid, seqNum, tempFile, finalFile, status)` — WARNING_LO.
    pub const EVENTID_RX_FILE_RENAME_FAILED: FwEventIdType = 37;
    /// `RxFileReopenFailed(cfdpClass, srcEid, seqNum, filename, status)` — WARNING_LO.
    pub const EVENTID_RX_FILE_REOPEN_FAILED: FwEventIdType = 38;
    /// `RxInactivityTimeout(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_RX_INACTIVITY_TIMEOUT: FwEventIdType = 39;
    /// `RxInvalidDirectiveCode(cfdpClass, srcEid, seqNum, directiveCode, substate)` — WARNING_LO.
    pub const EVENTID_RX_INVALID_DIRECTIVE_CODE: FwEventIdType = 40;
    /// `TxAckLimitReached(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_TX_ACK_LIMIT_REACHED: FwEventIdType = 41;
    /// `TxInactivityTimeout(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_TX_INACTIVITY_TIMEOUT: FwEventIdType = 42;
    /// `TxZeroLengthFile(cfdpClass, srcEid, seqNum, filename)` — WARNING_LO.
    pub const EVENTID_TX_ZERO_LENGTH_FILE: FwEventIdType = 43;
    /// `TxFileOpenFailed(cfdpClass, srcEid, seqNum, filename, status)` — WARNING_LO.
    pub const EVENTID_TX_FILE_OPEN_FAILED: FwEventIdType = 44;
    /// `TxFileSeekFailed(cfdpClass, srcEid, seqNum, status)` — WARNING_LO.
    pub const EVENTID_TX_FILE_SEEK_FAILED: FwEventIdType = 45;
    /// `TxSendMetadataFailed(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_TX_SEND_METADATA_FAILED: FwEventIdType = 46;
    /// `TxEarlyFinReceived(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_TX_EARLY_FIN_RECEIVED: FwEventIdType = 47;
    /// `TxInvalidNakPdu(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_TX_INVALID_NAK_PDU: FwEventIdType = 48;
    /// `TxInvalidSegmentRequests(cfdpClass, srcEid, seqNum, badCount)` — WARNING_LO.
    pub const EVENTID_TX_INVALID_SEGMENT_REQUESTS: FwEventIdType = 49;
    /// `TxNonFileDirectivePduReceived(cfdpClass, srcEid, seqNum)` — WARNING_LO.
    pub const EVENTID_TX_NON_FILE_DIRECTIVE_PDU_RECEIVED: FwEventIdType = 50;
    /// `TxInvalidDirectiveCode(cfdpClass, srcEid, seqNum, directiveCode, substate)` — WARNING_LO.
    pub const EVENTID_TX_INVALID_DIRECTIVE_CODE: FwEventIdType = 51;
    /// `TxFileTransferStarted(cfdpClass, seqNum, srcEid, srcFile, destEid, destFile, fileSize)` — ACTIVITY_HI.
    pub const EVENTID_TX_FILE_TRANSFER_STARTED: FwEventIdType = 52;
    /// `TxFileTransferCompleted(cfdpClass, seqNum, srcEid, srcFile, destEid, destFile, fileSize)` — ACTIVITY_HI.
    pub const EVENTID_TX_FILE_TRANSFER_COMPLETED: FwEventIdType = 53;
    /// `TxFileTransferFailed(cfdpClass, seqNum, srcEid, srcFile, destEid, destFile, conditionCode)` — WARNING_LO.
    pub const EVENTID_TX_FILE_TRANSFER_FAILED: FwEventIdType = 54;
    /// `RxFileTransferCompleted(cfdpClass, seqNum, srcEid, srcFile, destEid, destFile, fileSize)` — ACTIVITY_HI.
    pub const EVENTID_RX_FILE_TRANSFER_COMPLETED: FwEventIdType = 55;
    /// `RxFileTransferFailed(cfdpClass, seqNum, srcEid, srcFile, destEid, destFile, conditionCode)` — WARNING_LO.
    pub const EVENTID_RX_FILE_TRANSFER_FAILED: FwEventIdType = 56;
    /// `MetadataReceived(srcFile, destFile, transactionSeq)` — ACTIVITY_LO.
    pub const EVENTID_METADATA_RECEIVED: FwEventIdType = 57;
    /// `FileDataSegmentMetadata()` — WARNING_LO.
    pub const EVENTID_FILE_DATA_SEGMENT_METADATA: FwEventIdType = 58;
    /// `ChunklistUnavailable(transactionSeq)` — WARNING_LO.
    pub const EVENTID_CHUNKLIST_UNAVAILABLE: FwEventIdType = 59;
    /// `UnhandledPduInIdleState()` — WARNING_LO.
    pub const EVENTID_UNHANDLED_PDU_IN_IDLE_STATE: FwEventIdType = 60;
    /// `RxTransactionLimitReached(srcEid, transactionSeq)` — WARNING_LO.
    pub const EVENTID_RX_TRANSACTION_LIMIT_REACHED: FwEventIdType = 61;
    /// `InvalidDestinationEid(destEid)` — WARNING_LO.
    pub const EVENTID_INVALID_DESTINATION_EID: FwEventIdType = 62;
    /// `MaxTxTransactionsReached()` — WARNING_LO.
    pub const EVENTID_MAX_TX_TRANSACTIONS_REACHED: FwEventIdType = 63;
    /// `PlaybackDirOpenFailed(directory, status)` — WARNING_LO.
    pub const EVENTID_PLAYBACK_DIR_OPEN_FAILED: FwEventIdType = 64;
    /// `PlaybackDirSlotUnavailable()` — WARNING_LO.
    pub const EVENTID_PLAYBACK_DIR_SLOT_UNAVAILABLE: FwEventIdType = 65;
    /// `DanglingFileHandleClosed(channelId, transactionSeq)` — WARNING_LO.
    pub const EVENTID_DANGLING_FILE_HANDLE_CLOSED: FwEventIdType = 66;
    /// `PlaybackDirReadFailed(directory, status)` — WARNING_LO.
    pub const EVENTID_PLAYBACK_DIR_READ_FAILED: FwEventIdType = 67;
    /// `ResetFreedTransaction()` — DIAGNOSTIC.
    pub const EVENTID_RESET_FREED_TRANSACTION: FwEventIdType = 68;
    /// `FileRemoveFailed(filename, status)` — WARNING_LO.
    pub const EVENTID_FILE_REMOVE_FAILED: FwEventIdType = 69;
    /// `TransactionSuspended(transactionSeq, entityId)` — ACTIVITY_LO. Transaction was suspended
    pub const EVENTID_TRANSACTION_SUSPENDED: FwEventIdType = 70;
    /// `TransactionResumed(transactionSeq, entityId)` — ACTIVITY_LO. Transaction was resumed
    pub const EVENTID_TRANSACTION_RESUMED: FwEventIdType = 71;
    /// `TransactionCanceled(transactionSeq, entityId)` — ACTIVITY_HI. Transaction canceled
    pub const EVENTID_TRANSACTION_CANCELED: FwEventIdType = 72;
    /// `TransactionAbandoned(transactionSeq, entityId)` — ACTIVITY_HI. Transaction abandoned
    pub const EVENTID_TRANSACTION_ABANDONED: FwEventIdType = 73;
    /// `TransactionNotFound(transactionSeq, entityId)` — WARNING_LO. Transaction not found for command
    pub const EVENTID_TRANSACTION_NOT_FOUND: FwEventIdType = 74;
    /// `ResetCounters(channelId)` — ACTIVITY_HI.
    pub const EVENTID_RESET_COUNTERS: FwEventIdType = 75;
    /// `TxLateFinAcked(srcEid, seqNum)` — DIAGNOSTIC. A FIN arrived for a downlink transaction we sourced that has already completed and been recycled; acknowledged statelessly per CFDP so the peer can close out.
    pub const EVENTID_TX_LATE_FIN_ACKED: FwEventIdType = 76;
}

impl CfdpManager {
    /// `log_WARNING_LO_BuffersExhausted`
    pub(crate) fn log_warning_lo_buffers_exhausted(&self) {
        self.log(
            Self::EVENTID_BUFFERS_EXHAUSTED,
            LogSeverity::WarningLo,
            format_args!("Unable to allocate a PDU buffer"),
            |_| SerializeStatus::Ok,
        );
    }

    /// `log_WARNING_LO_InvalidChannel`
    pub(crate) fn log_warning_lo_invalid_channel(&self, channel_id: u8, max_channel_id: u8) {
        self.log(
            Self::EVENTID_INVALID_CHANNEL,
            LogSeverity::WarningLo,
            format_args!(
                "Invalid channel ID {}, maximum channel ID is {}",
                channel_id, max_channel_id
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_u8_be(max_channel_id)
            },
        );
    }

    /// `log_ACTIVITY_LO_TxFileQueued`
    pub(crate) fn log_activity_lo_tx_file_queued(
        &self,
        source_file_name: &[u8],
        transaction_seq: u32,
    ) {
        self.log(
            Self::EVENTID_TX_FILE_QUEUED,
            LogSeverity::ActivityLo,
            format_args!(
                "TX file queued for {} (transaction {})",
                Lossy(source_file_name),
                transaction_seq
            ),
            |b| {
                fw_try!(ser_str(b, source_file_name));
                b.serialize_u32_be(transaction_seq)
            },
        );
    }

    /// `log_WARNING_LO_UnsupportedSendFileArguments`
    pub(crate) fn log_warning_lo_unsupported_send_file_arguments(&self, offset: u32, length: u32) {
        self.log(
            Self::EVENTID_UNSUPPORTED_SEND_FILE_ARGUMENTS,
            LogSeverity::WarningLo,
            format_args!(
                "Invalid send file port request with offset {}, length {}",
                offset, length
            ),
            |b| {
                fw_try!(b.serialize_u32_be(offset));
                b.serialize_u32_be(length)
            },
        );
    }

    /// `log_WARNING_LO_SendFileInitiateFail`
    pub(crate) fn log_warning_lo_send_file_initiate_fail(&self, source_file_name: &[u8]) {
        self.log(
            Self::EVENTID_SEND_FILE_INITIATE_FAIL,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to initiate file send transfer for {}",
                Lossy(source_file_name)
            ),
            |b| ser_str(b, source_file_name),
        );
    }

    /// `log_ACTIVITY_LO_PlaybackInitiated`
    pub(crate) fn log_activity_lo_playback_initiated(&self, source_directory: &[u8]) {
        self.log(
            Self::EVENTID_PLAYBACK_INITIATED,
            LogSeverity::ActivityLo,
            format_args!(
                "Successfully initiated directory playback for {}",
                Lossy(source_directory)
            ),
            |b| ser_str(b, source_directory),
        );
    }

    /// `log_ACTIVITY_LO_PollDirInitiated`
    pub(crate) fn log_activity_lo_poll_dir_initiated(&self, source_directory: &[u8], poll_id: u8) {
        self.log(
            Self::EVENTID_POLL_DIR_INITIATED,
            LogSeverity::ActivityLo,
            format_args!(
                "Successfully initiated directory poll for {} (index {})",
                Lossy(source_directory),
                poll_id
            ),
            |b| {
                fw_try!(ser_str(b, source_directory));
                b.serialize_u8_be(poll_id)
            },
        );
    }

    /// `log_ACTIVITY_LO_PollDirStopped`
    pub(crate) fn log_activity_lo_poll_dir_stopped(&self, channel_id: u8, poll_id: u8) {
        self.log(
            Self::EVENTID_POLL_DIR_STOPPED,
            LogSeverity::ActivityLo,
            format_args!(
                "Successfully stopped directory poll for channel {}, index {}",
                channel_id, poll_id
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_u8_be(poll_id)
            },
        );
    }

    /// `log_WARNING_LO_PollDirBusy`
    pub(crate) fn log_warning_lo_poll_dir_busy(&self, channel_id: u8, poll_id: u8) {
        self.log(
            Self::EVENTID_POLL_DIR_BUSY,
            LogSeverity::WarningLo,
            format_args!(
                "Cannot start directory poll - channel {} poll {} already in use",
                channel_id, poll_id
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_u8_be(poll_id)
            },
        );
    }

    /// `log_WARNING_LO_PollDirNotActive`
    pub(crate) fn log_warning_lo_poll_dir_not_active(&self, channel_id: u8, poll_id: u8) {
        self.log(
            Self::EVENTID_POLL_DIR_NOT_ACTIVE,
            LogSeverity::WarningLo,
            format_args!(
                "Cannot stop directory poll - channel {} poll {} is not active",
                channel_id, poll_id
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_u8_be(poll_id)
            },
        );
    }

    /// `log_ACTIVITY_LO_SetFlowState`
    pub(crate) fn log_activity_lo_set_flow_state(&self, channel_id: u8, flow_state: Flow) {
        self.log(
            Self::EVENTID_SET_FLOW_STATE,
            LogSeverity::ActivityLo,
            format_args!("Set channel {} to {}", channel_id, flow_state.fpp_name()),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                flow_state.serialize_to(b, Endianness::Big)
            },
        );
    }

    /// `log_WARNING_LO_InvalidChannelPoll`
    pub(crate) fn log_warning_lo_invalid_channel_poll(&self, poll_id: u8, max_poll_id: u8) {
        self.log(
            Self::EVENTID_INVALID_CHANNEL_POLL,
            LogSeverity::WarningLo,
            format_args!(
                "Invalid poll ID {}, maximum poll ID is {}",
                poll_id, max_poll_id
            ),
            |b| {
                fw_try!(b.serialize_u8_be(poll_id));
                b.serialize_u8_be(max_poll_id)
            },
        );
    }

    /// `log_WARNING_LO_InvalidPollInterval`
    pub(crate) fn log_warning_lo_invalid_poll_interval(&self, interval: u32) {
        self.log(
            Self::EVENTID_INVALID_POLL_INTERVAL,
            LogSeverity::WarningLo,
            format_args!(
                "Invalid poll interval {} seconds, must be non-zero",
                interval
            ),
            |b| b.serialize_u32_be(interval),
        );
    }

    /// `log_WARNING_LO_FailKeepFileMove`
    pub(crate) fn log_warning_lo_fail_keep_file_move(
        &self,
        src_file: &[u8],
        move_dir: &[u8],
        status: i32,
    ) {
        self.log(
            Self::EVENTID_FAIL_KEEP_FILE_MOVE,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to move {} to {} error {}",
                Lossy(src_file),
                Lossy(move_dir),
                status
            ),
            |b| {
                fw_try!(ser_str(b, src_file));
                fw_try!(ser_str(b, move_dir));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailPollFileMove`
    pub(crate) fn log_warning_lo_fail_poll_file_move(
        &self,
        src_file: &[u8],
        fail_dir: &[u8],
        status: i32,
    ) {
        self.log(
            Self::EVENTID_FAIL_POLL_FILE_MOVE,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to move {} to {} error {}",
                Lossy(src_file),
                Lossy(fail_dir),
                status
            ),
            |b| {
                fw_try!(ser_str(b, src_file));
                fw_try!(ser_str(b, fail_dir));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailPduHeaderDeserialization`
    pub(crate) fn log_warning_lo_fail_pdu_header_deserialization(
        &self,
        channel_id: u8,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_FAIL_PDU_HEADER_DESERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to deserialize PDU header on channel {}, status {}",
                channel_id, status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailPduSerialization`
    pub(crate) fn log_warning_lo_fail_pdu_serialization(
        &self,
        channel_id: u8,
        pdu_type: PduTypeEnum,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_FAIL_PDU_SERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to serialize {} PDU on channel {}, status {}",
                channel_id,
                pdu_type.fpp_name(),
                status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                fw_try!(pdu_type.serialize_to(b, Endianness::Big));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailMetadataPduDeserialization`
    pub(crate) fn log_warning_lo_fail_metadata_pdu_deserialization(
        &self,
        channel_id: u8,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_FAIL_METADATA_PDU_DESERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to deserialize Metadata PDU on channel {}, status {}",
                channel_id, status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailFileDataPduDeserialization`
    pub(crate) fn log_warning_lo_fail_file_data_pdu_deserialization(
        &self,
        channel_id: u8,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_FAIL_FILE_DATA_PDU_DESERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to deserialize File Data PDU on channel {}, status {}",
                channel_id, status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailEofPduDeserialization`
    pub(crate) fn log_warning_lo_fail_eof_pdu_deserialization(&self, channel_id: u8, status: i32) {
        self.log(
            Self::EVENTID_FAIL_EOF_PDU_DESERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to deserialize EOF PDU on channel {}, status {}",
                channel_id, status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailAckPduDeserialization`
    pub(crate) fn log_warning_lo_fail_ack_pdu_deserialization(&self, channel_id: u8, status: i32) {
        self.log(
            Self::EVENTID_FAIL_ACK_PDU_DESERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to deserialize ACK PDU on channel {}, status {}",
                channel_id, status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailFinPduDeserialization`
    pub(crate) fn log_warning_lo_fail_fin_pdu_deserialization(&self, channel_id: u8, status: i32) {
        self.log(
            Self::EVENTID_FAIL_FIN_PDU_DESERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to deserialize FIN PDU on channel {}, status {}",
                channel_id, status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_FailNakPduDeserialization`
    pub(crate) fn log_warning_lo_fail_nak_pdu_deserialization(&self, channel_id: u8, status: i32) {
        self.log(
            Self::EVENTID_FAIL_NAK_PDU_DESERIALIZATION,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to deserialize NAK PDU on channel {}, status {}",
                channel_id, status
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_RxAckLimitReached`
    pub(crate) fn log_warning_lo_rx_ack_limit_reached(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_RX_ACK_LIMIT_REACHED,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} ACK limit reached for transaction {}:{}, no fin-ack sent",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_ACTIVITY_LO_RxTempFileCreated`
    pub(crate) fn log_activity_lo_rx_temp_file_created(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        filename: &[u8],
    ) {
        self.log(
            Self::EVENTID_RX_TEMP_FILE_CREATED,
            LogSeverity::ActivityLo,
            format_args!(
                "RX class {} transaction {}:{} creating temp file {} without metadata",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                Lossy(filename)
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                ser_str(b, filename)
            },
        );
    }

    /// `log_WARNING_LO_RxFileCreateFailed`
    pub(crate) fn log_warning_lo_rx_file_create_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        filename: &[u8],
        status: i32,
    ) {
        self.log(
            Self::EVENTID_RX_FILE_CREATE_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} failed to create file {}, error={}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                Lossy(filename),
                status
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(ser_str(b, filename));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_RxCrcMismatch`
    pub(crate) fn log_warning_lo_rx_crc_mismatch(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        expected: u32,
        actual: u32,
    ) {
        self.log(
            Self::EVENTID_RX_CRC_MISMATCH,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} CRC mismatch: expected 0x{:x} actual 0x{:x}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                expected,
                actual
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(expected));
                b.serialize_u32_be(actual)
            },
        );
    }

    /// `log_WARNING_LO_RxNakLimitReached`
    pub(crate) fn log_warning_lo_rx_nak_limit_reached(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_RX_NAK_LIMIT_REACHED,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} NAK limit reached",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_RxSeekFailed`
    pub(crate) fn log_warning_lo_rx_seek_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        offset: u32,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_RX_SEEK_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} failed to seek to offset {}, error={}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                offset,
                status
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(offset));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_RxWriteFailed`
    pub(crate) fn log_warning_lo_rx_write_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        expected: u32,
        actual: i32,
    ) {
        self.log(
            Self::EVENTID_RX_WRITE_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} write failed: expected {} bytes, got {}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                expected,
                actual
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(expected));
                b.serialize_i32_be(actual)
            },
        );
    }

    /// `log_WARNING_LO_RxFileDataOutOfBounds`
    pub(crate) fn log_warning_lo_rx_file_data_out_of_bounds(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        offset: u32,
        data_size: u32,
        file_size: u32,
    ) {
        self.log(
            Self::EVENTID_RX_FILE_DATA_OUT_OF_BOUNDS,
            LogSeverity::WarningLo,
            format_args!("RX class {} transaction {}:{} file data [{}, +{}] exceeds file size bound {}, PDU rejected", cfdp_class.fpp_name(), src_eid, seq_num, offset, data_size, file_size),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(offset));
                fw_try!(b.serialize_u32_be(data_size));
                b.serialize_u32_be(file_size)
            },
        );
    }

    /// `log_WARNING_LO_RxFileSizeMismatch`
    pub(crate) fn log_warning_lo_rx_file_size_mismatch(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        expected: u32,
        actual: u32,
    ) {
        self.log(
            Self::EVENTID_RX_FILE_SIZE_MISMATCH,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} EOF file size mismatch: expected {} got {}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                expected,
                actual
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(expected));
                b.serialize_u32_be(actual)
            },
        );
    }

    /// `log_ACTIVITY_HI_RxEofCancelReceived`
    pub(crate) fn log_activity_hi_rx_eof_cancel_received(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_RX_EOF_CANCEL_RECEIVED,
            LogSeverity::ActivityHi,
            format_args!(
                "RX class {} transaction {}:{} cancelled by sender",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_RxEofWithError`
    pub(crate) fn log_warning_lo_rx_eof_with_error(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        condition_code: u8,
    ) {
        self.log(
            Self::EVENTID_RX_EOF_WITH_ERROR,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} received EOF with error condition code {}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                condition_code
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                b.serialize_u8_be(condition_code)
            },
        );
    }

    /// `log_WARNING_LO_RxSeekCrcFailed`
    pub(crate) fn log_warning_lo_rx_seek_crc_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        offset: u32,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_RX_SEEK_CRC_FAILED,
            LogSeverity::WarningLo,
            format_args!("RX class {} transaction {}:{} failed to seek offset {} during CRC calculation, error={}", cfdp_class.fpp_name(), src_eid, seq_num, offset, status),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(offset));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_RxReadCrcFailed`
    pub(crate) fn log_warning_lo_rx_read_crc_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        expected: u32,
        actual: i32,
    ) {
        self.log(
            Self::EVENTID_RX_READ_CRC_FAILED,
            LogSeverity::WarningLo,
            format_args!("RX class {} transaction {}:{} failed to read during CRC calculation: expected {} bytes, got {}", cfdp_class.fpp_name(), src_eid, seq_num, expected, actual),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(expected));
                b.serialize_i32_be(actual)
            },
        );
    }

    /// `log_WARNING_LO_RxEofMdSizeMismatch`
    pub(crate) fn log_warning_lo_rx_eof_md_size_mismatch(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        md_size: u32,
        eof_size: u32,
    ) {
        self.log(
            Self::EVENTID_RX_EOF_MD_SIZE_MISMATCH,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} EOF/metadata size mismatch: metadata={}, EOF={}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                md_size,
                eof_size
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(md_size));
                b.serialize_u32_be(eof_size)
            },
        );
    }

    /// `log_WARNING_LO_RxFileRenameFailed`
    pub(crate) fn log_warning_lo_rx_file_rename_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        temp_file: &[u8],
        final_file: &[u8],
        status: i32,
    ) {
        self.log(
            Self::EVENTID_RX_FILE_RENAME_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} failed to rename {} to {}, error={}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                Lossy(temp_file),
                Lossy(final_file),
                status
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(ser_str(b, temp_file));
                fw_try!(ser_str(b, final_file));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_RxFileReopenFailed`
    pub(crate) fn log_warning_lo_rx_file_reopen_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        filename: &[u8],
        status: i32,
    ) {
        self.log(
            Self::EVENTID_RX_FILE_REOPEN_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} failed to reopen file {} after rename, error={}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                Lossy(filename),
                status
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(ser_str(b, filename));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_RxInactivityTimeout`
    pub(crate) fn log_warning_lo_rx_inactivity_timeout(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_RX_INACTIVITY_TIMEOUT,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} inactivity timer expired",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_RxInvalidDirectiveCode`
    pub(crate) fn log_warning_lo_rx_invalid_directive_code(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        directive_code: u8,
        substate: u8,
    ) {
        self.log(
            Self::EVENTID_RX_INVALID_DIRECTIVE_CODE,
            LogSeverity::WarningLo,
            format_args!(
                "RX class {} transaction {}:{} received invalid directive code {} for substate {}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                directive_code,
                substate
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u8_be(directive_code));
                b.serialize_u8_be(substate)
            },
        );
    }

    /// `log_WARNING_LO_TxAckLimitReached`
    pub(crate) fn log_warning_lo_tx_ack_limit_reached(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_TX_ACK_LIMIT_REACHED,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} ACK limit reached, no eof-ack received",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_TxInactivityTimeout`
    pub(crate) fn log_warning_lo_tx_inactivity_timeout(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_TX_INACTIVITY_TIMEOUT,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} inactivity timer expired",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_TxZeroLengthFile`
    pub(crate) fn log_warning_lo_tx_zero_length_file(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        filename: &[u8],
    ) {
        self.log(
            Self::EVENTID_TX_ZERO_LENGTH_FILE,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} cannot transfer zero-length file {}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                Lossy(filename)
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                ser_str(b, filename)
            },
        );
    }

    /// `log_WARNING_LO_TxFileOpenFailed`
    pub(crate) fn log_warning_lo_tx_file_open_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        filename: &[u8],
        status: i32,
    ) {
        self.log(
            Self::EVENTID_TX_FILE_OPEN_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} failed to open file {}, error={}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                Lossy(filename),
                status
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(ser_str(b, filename));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_TxFileSeekFailed`
    pub(crate) fn log_warning_lo_tx_file_seek_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_TX_FILE_SEEK_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} failed to seek to beginning of file, error={}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                status
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_TxSendMetadataFailed`
    pub(crate) fn log_warning_lo_tx_send_metadata_failed(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_TX_SEND_METADATA_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} failed to send metadata PDU",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_TxEarlyFinReceived`
    pub(crate) fn log_warning_lo_tx_early_fin_received(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_TX_EARLY_FIN_RECEIVED,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} received early FIN, cancelling transfer",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_TxInvalidNakPdu`
    pub(crate) fn log_warning_lo_tx_invalid_nak_pdu(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_TX_INVALID_NAK_PDU,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} received invalid NAK PDU",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_TxInvalidSegmentRequests`
    pub(crate) fn log_warning_lo_tx_invalid_segment_requests(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        bad_count: u32,
    ) {
        self.log(
            Self::EVENTID_TX_INVALID_SEGMENT_REQUESTS,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} received {} invalid NAK segment requests",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                bad_count
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                b.serialize_u32_be(bad_count)
            },
        );
    }

    /// `log_WARNING_LO_TxNonFileDirectivePduReceived`
    pub(crate) fn log_warning_lo_tx_non_file_directive_pdu_received(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
    ) {
        self.log(
            Self::EVENTID_TX_NON_FILE_DIRECTIVE_PDU_RECEIVED,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} received non-file-directive PDU",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }

    /// `log_WARNING_LO_TxInvalidDirectiveCode`
    pub(crate) fn log_warning_lo_tx_invalid_directive_code(
        &self,
        cfdp_class: Class,
        src_eid: u32,
        seq_num: u32,
        directive_code: u8,
        substate: u8,
    ) {
        self.log(
            Self::EVENTID_TX_INVALID_DIRECTIVE_CODE,
            LogSeverity::WarningLo,
            format_args!(
                "TX class {} transaction {}:{} received invalid directive code {} for substate {}",
                cfdp_class.fpp_name(),
                src_eid,
                seq_num,
                directive_code,
                substate
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u8_be(directive_code));
                b.serialize_u8_be(substate)
            },
        );
    }

    /// `log_ACTIVITY_HI_TxFileTransferStarted`
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn log_activity_hi_tx_file_transfer_started(
        &self,
        cfdp_class: Class,
        seq_num: u32,
        src_eid: u32,
        src_file: &[u8],
        dest_eid: u32,
        dest_file: &[u8],
        file_size: u32,
    ) {
        self.log(
            Self::EVENTID_TX_FILE_TRANSFER_STARTED,
            LogSeverity::ActivityHi,
            format_args!(
                "TX {} transaction {} transfer started, {}:{} -> {}:{}, {} bytes",
                cfdp_class.fpp_name(),
                seq_num,
                src_eid,
                Lossy(src_file),
                dest_eid,
                Lossy(dest_file),
                file_size
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(ser_str(b, src_file));
                fw_try!(b.serialize_u32_be(dest_eid));
                fw_try!(ser_str(b, dest_file));
                b.serialize_u32_be(file_size)
            },
        );
    }

    /// `log_ACTIVITY_HI_TxFileTransferCompleted`
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn log_activity_hi_tx_file_transfer_completed(
        &self,
        cfdp_class: Class,
        seq_num: u32,
        src_eid: u32,
        src_file: &[u8],
        dest_eid: u32,
        dest_file: &[u8],
        file_size: u32,
    ) {
        self.log(
            Self::EVENTID_TX_FILE_TRANSFER_COMPLETED,
            LogSeverity::ActivityHi,
            format_args!(
                "TX {} transaction {} completed, {}:{} -> {}:{}, {} bytes",
                cfdp_class.fpp_name(),
                seq_num,
                src_eid,
                Lossy(src_file),
                dest_eid,
                Lossy(dest_file),
                file_size
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(ser_str(b, src_file));
                fw_try!(b.serialize_u32_be(dest_eid));
                fw_try!(ser_str(b, dest_file));
                b.serialize_u32_be(file_size)
            },
        );
    }

    /// `log_WARNING_LO_TxFileTransferFailed`
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn log_warning_lo_tx_file_transfer_failed(
        &self,
        cfdp_class: Class,
        seq_num: u32,
        src_eid: u32,
        src_file: &[u8],
        dest_eid: u32,
        dest_file: &[u8],
        condition_code: u8,
    ) {
        self.log(
            Self::EVENTID_TX_FILE_TRANSFER_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "TX {} transaction {} FAILED, {}:{} -> {}:{}, error code {}",
                cfdp_class.fpp_name(),
                seq_num,
                src_eid,
                Lossy(src_file),
                dest_eid,
                Lossy(dest_file),
                condition_code
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(ser_str(b, src_file));
                fw_try!(b.serialize_u32_be(dest_eid));
                fw_try!(ser_str(b, dest_file));
                b.serialize_u8_be(condition_code)
            },
        );
    }

    /// `log_ACTIVITY_HI_RxFileTransferCompleted`
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn log_activity_hi_rx_file_transfer_completed(
        &self,
        cfdp_class: Class,
        seq_num: u32,
        src_eid: u32,
        src_file: &[u8],
        dest_eid: u32,
        dest_file: &[u8],
        file_size: u32,
    ) {
        self.log(
            Self::EVENTID_RX_FILE_TRANSFER_COMPLETED,
            LogSeverity::ActivityHi,
            format_args!(
                "RX {} transaction {} completed, {}:{} -> {}:{}, {} bytes",
                cfdp_class.fpp_name(),
                seq_num,
                src_eid,
                Lossy(src_file),
                dest_eid,
                Lossy(dest_file),
                file_size
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(ser_str(b, src_file));
                fw_try!(b.serialize_u32_be(dest_eid));
                fw_try!(ser_str(b, dest_file));
                b.serialize_u32_be(file_size)
            },
        );
    }

    /// `log_WARNING_LO_RxFileTransferFailed`
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn log_warning_lo_rx_file_transfer_failed(
        &self,
        cfdp_class: Class,
        seq_num: u32,
        src_eid: u32,
        src_file: &[u8],
        dest_eid: u32,
        dest_file: &[u8],
        condition_code: u8,
    ) {
        self.log(
            Self::EVENTID_RX_FILE_TRANSFER_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "RX {} transaction {} FAILED, {}:{} -> {}:{}, error code {}",
                cfdp_class.fpp_name(),
                seq_num,
                src_eid,
                Lossy(src_file),
                dest_eid,
                Lossy(dest_file),
                condition_code
            ),
            |b| {
                fw_try!(cfdp_class.serialize_to(b, Endianness::Big));
                fw_try!(b.serialize_u32_be(seq_num));
                fw_try!(b.serialize_u32_be(src_eid));
                fw_try!(ser_str(b, src_file));
                fw_try!(b.serialize_u32_be(dest_eid));
                fw_try!(ser_str(b, dest_file));
                b.serialize_u8_be(condition_code)
            },
        );
    }

    /// `log_ACTIVITY_LO_MetadataReceived`
    pub(crate) fn log_activity_lo_metadata_received(
        &self,
        src_file: &[u8],
        dest_file: &[u8],
        transaction_seq: u32,
    ) {
        self.log(
            Self::EVENTID_METADATA_RECEIVED,
            LogSeverity::ActivityLo,
            format_args!(
                "Metadata received for source: {}, dest: {} (transaction {})",
                Lossy(src_file),
                Lossy(dest_file),
                transaction_seq
            ),
            |b| {
                fw_try!(ser_str(b, src_file));
                fw_try!(ser_str(b, dest_file));
                b.serialize_u32_be(transaction_seq)
            },
        );
    }

    /// `log_WARNING_LO_FileDataSegmentMetadata`
    pub(crate) fn log_warning_lo_file_data_segment_metadata(&self) {
        self.log(
            Self::EVENTID_FILE_DATA_SEGMENT_METADATA,
            LogSeverity::WarningLo,
            format_args!("File data PDU with unsupported segment metadata received"),
            |_| SerializeStatus::Ok,
        );
    }

    /// `log_WARNING_LO_ChunklistUnavailable`
    pub(crate) fn log_warning_lo_chunklist_unavailable(&self, transaction_seq: u32) {
        self.log(
            Self::EVENTID_CHUNKLIST_UNAVAILABLE,
            LogSeverity::WarningLo,
            format_args!(
                "Cannot get chunklist, abandoning transaction {}",
                transaction_seq
            ),
            |b| b.serialize_u32_be(transaction_seq),
        );
    }

    /// `log_WARNING_LO_UnhandledPduInIdleState`
    pub(crate) fn log_warning_lo_unhandled_pdu_in_idle_state(&self) {
        self.log(
            Self::EVENTID_UNHANDLED_PDU_IN_IDLE_STATE,
            LogSeverity::WarningLo,
            format_args!("Unhandled PDU type received in idle state"),
            |_| SerializeStatus::Ok,
        );
    }

    /// `log_WARNING_LO_RxTransactionLimitReached`
    pub(crate) fn log_warning_lo_rx_transaction_limit_reached(
        &self,
        src_eid: u32,
        transaction_seq: u32,
    ) {
        self.log(
            Self::EVENTID_RX_TRANSACTION_LIMIT_REACHED,
            LogSeverity::WarningLo,
            format_args!(
                "Dropping packet from {} transaction {} due to max RX transactions reached",
                src_eid, transaction_seq
            ),
            |b| {
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(transaction_seq)
            },
        );
    }

    /// `log_WARNING_LO_InvalidDestinationEid`
    pub(crate) fn log_warning_lo_invalid_destination_eid(&self, dest_eid: u32) {
        self.log(
            Self::EVENTID_INVALID_DESTINATION_EID,
            LogSeverity::WarningLo,
            format_args!("Dropping packet for invalid destination EID {}", dest_eid),
            |b| b.serialize_u32_be(dest_eid),
        );
    }

    /// `log_WARNING_LO_MaxTxTransactionsReached`
    pub(crate) fn log_warning_lo_max_tx_transactions_reached(&self) {
        self.log(
            Self::EVENTID_MAX_TX_TRANSACTIONS_REACHED,
            LogSeverity::WarningLo,
            format_args!("Maximum number of commanded TX files reached"),
            |_| SerializeStatus::Ok,
        );
    }

    /// `log_WARNING_LO_PlaybackDirOpenFailed`
    pub(crate) fn log_warning_lo_playback_dir_open_failed(&self, directory: &[u8], status: i32) {
        self.log(
            Self::EVENTID_PLAYBACK_DIR_OPEN_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to open playback directory {}, error={}",
                Lossy(directory),
                status
            ),
            |b| {
                fw_try!(ser_str(b, directory));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_WARNING_LO_PlaybackDirSlotUnavailable`
    pub(crate) fn log_warning_lo_playback_dir_slot_unavailable(&self) {
        self.log(
            Self::EVENTID_PLAYBACK_DIR_SLOT_UNAVAILABLE,
            LogSeverity::WarningLo,
            format_args!("No playback directory slot available"),
            |_| SerializeStatus::Ok,
        );
    }

    /// `log_WARNING_LO_DanglingFileHandleClosed`
    pub(crate) fn log_warning_lo_dangling_file_handle_closed(
        &self,
        channel_id: u8,
        transaction_seq: u32,
    ) {
        self.log(
            Self::EVENTID_DANGLING_FILE_HANDLE_CLOSED,
            LogSeverity::WarningLo,
            format_args!(
                "Closed dangling file handle for channel {} transaction {}",
                channel_id, transaction_seq
            ),
            |b| {
                fw_try!(b.serialize_u8_be(channel_id));
                b.serialize_u32_be(transaction_seq)
            },
        );
    }

    /// `log_WARNING_LO_PlaybackDirReadFailed`
    pub(crate) fn log_warning_lo_playback_dir_read_failed(&self, directory: &[u8], status: i32) {
        self.log(
            Self::EVENTID_PLAYBACK_DIR_READ_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to read from playback directory {}, error={}",
                Lossy(directory),
                status
            ),
            |b| {
                fw_try!(ser_str(b, directory));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_DIAGNOSTIC_ResetFreedTransaction`
    pub(crate) fn log_diagnostic_reset_freed_transaction(&self) {
        self.log(
            Self::EVENTID_RESET_FREED_TRANSACTION,
            LogSeverity::Diagnostic,
            format_args!("Attempt to reset a transaction that has already been freed"),
            |_| SerializeStatus::Ok,
        );
    }

    /// `log_WARNING_LO_FileRemoveFailed`
    pub(crate) fn log_warning_lo_file_remove_failed(&self, filename: &[u8], status: i32) {
        self.log(
            Self::EVENTID_FILE_REMOVE_FAILED,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to remove file {}, error={}",
                Lossy(filename),
                status
            ),
            |b| {
                fw_try!(ser_str(b, filename));
                b.serialize_i32_be(status)
            },
        );
    }

    /// `log_ACTIVITY_LO_TransactionSuspended`: Transaction was suspended
    pub(crate) fn log_activity_lo_transaction_suspended(
        &self,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) {
        self.log(
            Self::EVENTID_TRANSACTION_SUSPENDED,
            LogSeverity::ActivityLo,
            format_args!("Transaction ({}, {}) suspended", transaction_seq, entity_id),
            |b| {
                fw_try!(b.serialize_u32_be(transaction_seq));
                b.serialize_u32_be(entity_id)
            },
        );
    }

    /// `log_ACTIVITY_LO_TransactionResumed`: Transaction was resumed
    pub(crate) fn log_activity_lo_transaction_resumed(
        &self,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) {
        self.log(
            Self::EVENTID_TRANSACTION_RESUMED,
            LogSeverity::ActivityLo,
            format_args!("Transaction ({}, {}) resumed", transaction_seq, entity_id),
            |b| {
                fw_try!(b.serialize_u32_be(transaction_seq));
                b.serialize_u32_be(entity_id)
            },
        );
    }

    /// `log_ACTIVITY_HI_TransactionCanceled`: Transaction canceled
    pub(crate) fn log_activity_hi_transaction_canceled(
        &self,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) {
        self.log(
            Self::EVENTID_TRANSACTION_CANCELED,
            LogSeverity::ActivityHi,
            format_args!("Transaction ({}, {}) canceled", transaction_seq, entity_id),
            |b| {
                fw_try!(b.serialize_u32_be(transaction_seq));
                b.serialize_u32_be(entity_id)
            },
        );
    }

    /// `log_ACTIVITY_HI_TransactionAbandoned`: Transaction abandoned
    pub(crate) fn log_activity_hi_transaction_abandoned(
        &self,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) {
        self.log(
            Self::EVENTID_TRANSACTION_ABANDONED,
            LogSeverity::ActivityHi,
            format_args!("Transaction ({}, {}) abandoned", transaction_seq, entity_id),
            |b| {
                fw_try!(b.serialize_u32_be(transaction_seq));
                b.serialize_u32_be(entity_id)
            },
        );
    }

    /// `log_WARNING_LO_TransactionNotFound`: Transaction not found for command
    pub(crate) fn log_warning_lo_transaction_not_found(
        &self,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) {
        self.log(
            Self::EVENTID_TRANSACTION_NOT_FOUND,
            LogSeverity::WarningLo,
            format_args!("Transaction ({}, {}) not found", transaction_seq, entity_id),
            |b| {
                fw_try!(b.serialize_u32_be(transaction_seq));
                b.serialize_u32_be(entity_id)
            },
        );
    }

    /// `log_ACTIVITY_HI_ResetCounters`
    pub(crate) fn log_activity_hi_reset_counters(&self, channel_id: u8) {
        self.log(
            Self::EVENTID_RESET_COUNTERS,
            LogSeverity::ActivityHi,
            format_args!("Reset telemetry counters for channel {}", channel_id),
            |b| b.serialize_u8_be(channel_id),
        );
    }

    /// `log_DIAGNOSTIC_TxLateFinAcked`: A FIN arrived for a downlink transaction we sourced that has already completed and been recycled; acknowledged statelessly per CFDP so the peer can close out.
    pub(crate) fn log_diagnostic_tx_late_fin_acked(
        &self,
        src_eid: EntityId,
        seq_num: TransactionSeq,
    ) {
        self.log(
            Self::EVENTID_TX_LATE_FIN_ACKED,
            LogSeverity::Diagnostic,
            format_args!(
                "Acknowledged late FIN for transaction {}:{}",
                src_eid, seq_num
            ),
            |b| {
                fw_try!(b.serialize_u32_be(src_eid));
                b.serialize_u32_be(seq_num)
            },
        );
    }
}
