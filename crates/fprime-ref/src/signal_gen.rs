//! # SignalGen — QUEUED demo component modeled on `Ref::SignalGen`
//!
//! C++ source: `TestDeploymentsProject/Ref/SignalGen/SignalGen.{fpp,cpp}`
//! (analysis: `docs/cpp-analysis/ref-topology.md`, "Example component
//! SignalGen"). Deliberately simplified — no history/pair arrays — but
//! real: it exercises async commands (with the exact response discipline),
//! events (one with a throttle), telemetry, the time port, one parameter
//! (the deployment's `prmDb` client) and the synchronous data-product
//! producer path.
//!
//! Queued-component semantics (the load-bearing part, C++ parity): the
//! component owns a message queue but NO thread. Async commands sit in the
//! queue until the sync `schedIn` handler — invoked on the rate-group
//! thread — drains it via `dispatch_available_messages`, so commands
//! execute at schedIn time on the rate group's thread.
//!
//! ## Dictionary (component-relative ids; all offset by the id base)
//!
//! | Kind      | Name             | Id  | Shape |
//! |-----------|------------------|-----|-------|
//! | command   | `SETTINGS`       | 0   | `(frequency: U32, amplitude: F32, phase: F32, sig_type: SignalType/U8)` |
//! | command   | `TOGGLE`         | 1   | no args — start/stop the generator |
//! | command   | `SKIP`           | 2   | no args — zero the next sample |
//! | command   | `DP`             | 3   | `(records: U32)` — produce one data product synchronously |
//! | command   | `AMPLITUDE_PARAM_SET`  | 4 | `(val: F32)` — stage the parameter |
//! | command   | `AMPLITUDE_PARAM_SAVE` | 5 | no args — write it to `prmDb` |
//! | event     | `SettingsChanged`| 0   | ACTIVITY_LO `(U32, F32, F32, U8)` |
//! | event     | `Toggled`        | 1   | ACTIVITY_LO `(running: bool)` |
//! | event     | `SampleSkipped`  | 2   | ACTIVITY_LO, **throttle 3** (cleared by `TOGGLE`), no args |
//! | event     | `DpSent`         | 3   | ACTIVITY_LO `(records: U32, bytes: U32)` |
//! | event     | `DpBufferFailed` | 4   | WARNING_HI `(records: U32)` |
//! | event     | `AmplitudeUpdated`| 5  | ACTIVITY_HI `(val: F32)` |
//! | telemetry | `SignalValue`    | 0   | F32 |
//! | telemetry | `SignalType`     | 1   | U8 enum ([`SignalType`]: `Sine = 0`, `Triangle = 1`) |
//! | parameter | `Amplitude`      | 0   | F32, default `0.0` |
//! | product   | `DataContainer`  | 0   | default priority 10, records of `[id U32][value F32]` |
//!
//! Data products: only the SYNCHRONOUS request kind is ported
//! (`productGetOut` + `productSendOut`, i.e. C++ `DpReqType::IMMEDIATE`).
//! The asynchronous pair (`productRequestOut`/`productRecvIn`) needs an
//! async `Fw.DpResponse` input port carrying an owned buffer; it buys
//! nothing the synchronous path does not already prove end to end, so it
//! is deliberately left unported and its ports are absent (rather than
//! present-but-unconnected).
//!
//! Divergence from the task's one-line command sketch (documented): the
//! task lists `SETTINGS (u32, f32, f32)`, but the signal *type* must be
//! settable for the `SignalType` channel (and the "sine or triangle based
//! on settings" sample math) to mean anything, so `SETTINGS` carries the
//! C++ component's fourth argument as a U8 enum. An invalid enum byte
//! answers `ValidationError` (workspace-wide discipline); short or
//! residual argument bytes answer `FormatError`.

use fprime_comp::msg;
use fprime_comp::{
    CmdGlue, CmdPort, ComponentDispatch, EventGlue, EventThrottle, MsgDispatchStatus, OutputPort,
    PortRef, PrmGlue, QueueFullPolicy, QueuedBase, SchedPort, TlmGlue,
};
use fprime_config::{
    FwChanIdType, FwDpIdType, FwEnumStoreType, FwEventIdType, FwIndexType, FwOpcodeType,
    FwPrmIdType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::dp::{DpContainer, DpGetPort, DpSendPort};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Endianness, LinearBuffer, ParamBuffer, ParamValid, SerBuf,
    SerBufAny, Success, fw_assert,
};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Dictionary constants (what the FPP model would declare)
// ---------------------------------------------------------------------------

/// Queue message type for the async command input (0 = EXIT sentinel).
const MSG_TYPE_CMD: FwEnumStoreType = 1;

/// Queue priority for async commands (single async input — pure FIFO).
const CMD_PRIORITY: FwQueuePriorityType = 1;

/// Queue message size: envelope 6 + opcode 4 + cmdSeq 4 + nested
/// length-prefixed `CmdArgBuffer` (2 + 506) = 522; rounded up to 524.
/// C++ parity: the autocoder sizes queue messages for a FULL
/// `CmdArgBuffer` (not just the largest declared command), so an
/// oversized uplinked arg buffer is enqueued intact and answered with
/// `FormatError` by the handler's residual-bytes check instead of
/// asserting in the port adapter.
pub const QUEUE_MSG_SIZE: FwSizeType = 524;

/// `SETTINGS` opcode (component-relative).
pub const OPCODE_SETTINGS: FwOpcodeType = 0;
/// `TOGGLE` opcode.
pub const OPCODE_TOGGLE: FwOpcodeType = 1;
/// `SKIP` opcode.
pub const OPCODE_SKIP: FwOpcodeType = 2;
/// `DP(records: U32)` opcode — produce one data product synchronously
/// (C++ `Dp` opcode 3, reduced to the IMMEDIATE/synchronous request kind).
pub const OPCODE_DP: FwOpcodeType = 3;
/// `AMPLITUDE_PARAM_SET(val: F32)` opcode — the autocoded parameter-set
/// command (stages the value in the component only).
pub const OPCODE_AMPLITUDE_PARAM_SET: FwOpcodeType = 4;
/// `AMPLITUDE_PARAM_SAVE` opcode — the autocoded parameter-save command
/// (pushes the staged value to the parameter database via `prmSetOut`).
pub const OPCODE_AMPLITUDE_PARAM_SAVE: FwOpcodeType = 5;

/// `Amplitude: F32` parameter id (component-relative).
pub const PARAMID_AMPLITUDE: FwPrmIdType = 0;
/// FPP `default` for the `Amplitude` parameter.
pub const AMPLITUDE_DEFAULT: f32 = 0.0;

/// `DataContainer` data-product container id (component-relative; C++
/// `container DataContainer id 0 default priority 10`).
pub const CONTAINER_ID_DATA: FwDpIdType = 0;
/// FPP `default priority 10` for [`CONTAINER_ID_DATA`].
pub const CONTAINER_PRIORITY_DATA: u32 = 10;
/// `DataRecord` record id inside [`CONTAINER_ID_DATA`] (C++ `record
/// DataRecord: SignalInfo id 0`). Each record is `[id u32][value f32]`.
pub const RECORD_ID_DATA: FwDpIdType = 0;
/// Serialized size of one `DataRecord` entry (`[id u32][value f32]`).
pub const RECORD_SIZE_DATA: FwSizeType = 8;
/// Records per `DP` command are capped so one container always fits the
/// data-product BufferManager bin.
pub const DP_MAX_RECORDS: u32 = 64;

/// `SettingsChanged(frequency: U32, amplitude: F32, phase: F32, sig_type: U8)`
/// — ACTIVITY_LO (C++ `SignalGen_SettingsChanged`).
pub const EVENTID_SETTINGS_CHANGED: FwEventIdType = 0;
/// `Toggled(running: bool)` — ACTIVITY_LO.
pub const EVENTID_TOGGLED: FwEventIdType = 1;
/// `SampleSkipped` — ACTIVITY_LO, `throttle 3` (cleared by `TOGGLE`).
pub const EVENTID_SAMPLE_SKIPPED: FwEventIdType = 2;
/// FPP-style `throttle 3` on [`EVENTID_SAMPLE_SKIPPED`].
pub const SAMPLE_SKIPPED_THROTTLE: u32 = 3;
/// `DpSent(records: U32, bytes: U32)` — ACTIVITY_LO.
pub const EVENTID_DP_SENT: FwEventIdType = 3;
/// `DpBufferFailed(records: U32)` — WARNING_HI (the allocation failed).
pub const EVENTID_DP_BUFFER_FAILED: FwEventIdType = 4;
/// `AmplitudeUpdated(val: F32)` — ACTIVITY_HI, the `parameterUpdated`
/// notification for the `Amplitude` parameter.
pub const EVENTID_AMPLITUDE_UPDATED: FwEventIdType = 5;

/// `SignalValue: F32` telemetry channel.
pub const CHANID_SIGNAL_VALUE: FwChanIdType = 0;
/// `SignalType: U8` telemetry channel.
pub const CHANID_SIGNAL_TYPE: FwChanIdType = 1;

// ---------------------------------------------------------------------------
// Signal type enum (U8 on the wire)
// ---------------------------------------------------------------------------

/// The generated waveform — a two-value subset of the C++
/// `Ref.SignalType`, `repr(u8)` per the deployment dictionary above.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SignalType {
    /// `amplitude * sin(2π·ticks/frequency + phase)`.
    #[default]
    Sine = 0,
    /// Symmetric triangle wave over `frequency` ticks, peak `amplitude`.
    Triangle = 1,
}

impl TryFrom<u8> for SignalType {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, u8> {
        match v {
            0 => Ok(Self::Sine),
            1 => Ok(Self::Triangle),
            other => Err(other),
        }
    }
}

// ---------------------------------------------------------------------------
// The component
// ---------------------------------------------------------------------------

/// Mutable component state (C++ member variables).
struct SignalGenState {
    /// Ticks-per-period divider (C++ `signalFrequency`; sample rate = the
    /// schedIn rate). Guaranteed nonzero by `SETTINGS` validation.
    frequency: u32,
    /// Peak amplitude (C++ `signalAmplitude`).
    amplitude: f32,
    /// Phase offset in radians (C++ `signalPhase`, simplified to radians).
    phase: f32,
    /// Waveform selector.
    sig_type: SignalType,
    /// Sample counter since the last `TOGGLE` (C++ `ticks`).
    ticks: u32,
    /// Generator on/off (C++ `running`, starts stopped).
    running: bool,
    /// Zero the next sample (C++ `skipOne`).
    skip_next: bool,
    /// Staged `Amplitude` parameter value (the autocoded `m_Amplitude`).
    amplitude_param: f32,
    /// Validity of the staged parameter (autocoded `m_Amplitude_valid`).
    amplitude_valid: ParamValid,
    /// Sample counter used to fill data-product records.
    dp_samples: u32,
}

impl Default for SignalGenState {
    /// C++ constructor defaults (frequency 1 keeps the math well-defined
    /// before the first `SETTINGS`).
    fn default() -> Self {
        Self {
            frequency: 1,
            amplitude: 0.0,
            phase: 0.0,
            sig_type: SignalType::Sine,
            ticks: 0,
            running: false,
            skip_next: false,
            amplitude_param: AMPLITUDE_DEFAULT,
            amplitude_valid: ParamValid::Uninit,
            dp_samples: 0,
        }
    }
}

/// `Ref::SignalGen` (simplified) — queued demo signal generator.
pub struct SignalGen {
    /// Queued core: `PassiveBase` + message queue (no thread).
    pub queued: QueuedBase,
    /// Command registration + response ports.
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// Parameter ports (`prmGetOut`/`prmSetOut` — wired to `prmDb`).
    pub prm: PrmGlue,
    /// `productGetOut` — SYNCHRONOUS data-product buffer request
    /// (`Fw.DpGet`, wired to `dpMgr.productGetIn`).
    pub product_get_out: OutputPort<dyn DpGetPort>,
    /// `productSendOut` — filled data-product container out
    /// (`Fw.DpSend`, wired to `dpMgr.productSendIn`).
    pub product_send_out: OutputPort<dyn DpSendPort>,
    /// Throttle for `SampleSkipped` (`throttle 3`).
    sample_skipped_throttle: EventThrottle,
    /// Guarded state (the component mutex).
    state: Mutex<SignalGenState>,
}

impl SignalGen {
    /// Construct (topology phase 1). Follow with `set_id_base`, wiring,
    /// [`init`](Self::init), [`reg_commands`](Self::reg_commands) and
    /// [`load_parameters`](Self::load_parameters).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            queued: QueuedBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            prm: PrmGlue::new(),
            product_get_out: OutputPort::new(),
            product_send_out: OutputPort::new(),
            sample_skipped_throttle: EventThrottle::new(SAMPLE_SKIPPED_THROTTLE),
            state: Mutex::new(SignalGenState::default()),
        })
    }

    /// C++ autocoded `loadParameters()`: read every declared parameter from
    /// the parameter database through `prmGetOut`, falling back to the FPP
    /// default when the database has no (valid) value.
    ///
    /// Topology phase 7 — after `prmDb.read_param_file()` and before tasks
    /// start. Requires `prm.prm_get_out` to be connected (C++ asserts too).
    pub fn load_parameters(&self) {
        let mut buffer = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(self.id_base(), PARAMID_AMPLITUDE, &mut buffer);
        let mut amplitude = AMPLITUDE_DEFAULT;
        let mut status = valid;
        if valid == ParamValid::Valid || valid == ParamValid::Default {
            let mut value = 0f32;
            if buffer.deserialize_f32_be(&mut value).is_ok() {
                amplitude = value;
            } else {
                // C++ parity: a corrupt stored value falls back to the FPP
                // default and reports INVALID.
                status = ParamValid::Invalid;
            }
        }
        let mut state = self.state.lock().unwrap();
        state.amplitude = amplitude;
        state.amplitude_param = amplitude;
        state.amplitude_valid = status;
    }

    /// The staged `Amplitude` parameter and its validity (introspection,
    /// equivalent to the autocoded `paramGet_Amplitude`).
    #[must_use]
    pub fn amplitude_param(&self) -> (f32, ParamValid) {
        let state = self.state.lock().unwrap();
        (state.amplitude_param, state.amplitude_valid)
    }

    /// `productGetOut` port factory helper: the data-product container id
    /// this component produces (`id base + DataContainer`).
    #[must_use]
    pub fn data_container_id(&self) -> FwDpIdType {
        self.id_base() + CONTAINER_ID_DATA
    }

    fn id_base(&self) -> u32 {
        self.queued.base.get_id_base()
    }

    /// Create the message queue (C++ `init(queueDepth)`).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.queued.create_queue(queue_depth, QUEUE_MSG_SIZE);
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                OPCODE_SETTINGS,
                OPCODE_TOGGLE,
                OPCODE_SKIP,
                OPCODE_DP,
                OPCODE_AMPLITUDE_PARAM_SET,
                OPCODE_AMPLITUDE_PARAM_SAVE,
            ],
        );
    }

    // -- Input-port factories (topology wiring surface) ---------------------

    /// `schedIn` — SYNC `Svc.Sched` input: drains the queue, then samples.
    pub fn sched_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `cmdIn` — ASYNC `Fw.Cmd` input (async commands; executed at
    /// `schedIn` time on the rate-group thread).
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Sample math ---------------------------------------------------------

    /// C++ `generateSample`, reduced to the two supported waveforms. The
    /// tick counter runs modulo `frequency` (the ticks-per-period divider).
    fn sample(state: &SignalGenState) -> f32 {
        let period = state.frequency as f32;
        let position = (state.ticks % state.frequency) as f32 / period;
        match state.sig_type {
            SignalType::Sine => {
                state.amplitude * (2.0 * std::f32::consts::PI * position + state.phase).sin()
            }
            SignalType::Triangle => {
                // Symmetric /\ ramp in [-1, 1] over one period.
                state.amplitude * (1.0 - 4.0 * (position - 0.5).abs())
            }
        }
    }

    // -- Handlers ------------------------------------------------------------

    /// `schedIn` handler (rate-group thread). C++ parity: a queued
    /// component must intentionally drain its own queue here — dropping
    /// this makes async commands silently never execute.
    fn sched_in_handler(&self, _port_num: FwIndexType, _context: u32) {
        let _ = self.queued.dispatch_available_messages(self);

        let (value, sig_type) = {
            let mut state = self.state.lock().unwrap();
            // Short-circuit when the generator is not running (C++ parity).
            if !state.running {
                return;
            }
            // A skip zeroes exactly one sample (C++ `skipOne`).
            let value = if state.skip_next {
                0.0
            } else {
                Self::sample(&state)
            };
            state.skip_next = false;
            state.ticks = state.ticks.wrapping_add(1);
            (value, state.sig_type)
        };

        // Telemetry (ports invoked outside the state lock).
        let now = self.evt.time_get();
        let id_base = self.id_base();
        self.tlm
            .tlm_write(id_base, CHANID_SIGNAL_VALUE, &value, now);
        self.tlm
            .tlm_write(id_base, CHANID_SIGNAL_TYPE, &(sig_type as u8), now);
    }

    /// `SETTINGS(frequency: U32, amplitude: F32, phase: F32, sig_type: U8)`
    /// command handler. Exactly-once response discipline: `FormatError` on
    /// short/residual args, `ValidationError` on a zero frequency or an
    /// invalid enum byte.
    fn settings_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut frequency = 0u32;
        let mut amplitude = 0f32;
        let mut phase = 0f32;
        let mut raw_type = 0u8;
        if !args.deserialize_u32_be(&mut frequency).is_ok()
            || !args.deserialize_f32_be(&mut amplitude).is_ok()
            || !args.deserialize_f32_be(&mut phase).is_ok()
            || !args.deserialize_u8_be(&mut raw_type).is_ok()
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        // C++ parity (FW_CMD_CHECK_RESIDUAL): leftover bytes are an error.
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        // Invalid enum values answer ValidationError (workspace discipline).
        let Ok(sig_type) = SignalType::try_from(raw_type) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        // C++ parity: reject a frequency that breaks the sample math.
        if frequency == 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }

        {
            let mut state = self.state.lock().unwrap();
            state.frequency = frequency;
            state.amplitude = amplitude;
            state.phase = phase;
            state.sig_type = sig_type;
        }

        self.evt.log_event(
            self.id_base(),
            EVENTID_SETTINGS_CHANGED,
            fprime_fw::LogSeverity::ActivityLo,
            &format!(
                "Set frequency {frequency}, amplitude {amplitude}, phase {phase}, type {sig_type:?}"
            ),
            |buf| {
                let status = buf.serialize_u32_be(frequency);
                if !status.is_ok() {
                    return status;
                }
                let status = buf.serialize_f32_be(amplitude);
                if !status.is_ok() {
                    return status;
                }
                let status = buf.serialize_f32_be(phase);
                if !status.is_ok() {
                    return status;
                }
                buf.serialize_u8_be(sig_type as u8)
            },
        );
        // C++ parity: the Settings handler re-emits the Type channel.
        self.tlm.tlm_write(
            self.id_base(),
            CHANID_SIGNAL_TYPE,
            &(sig_type as u8),
            self.evt.time_get(),
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `TOGGLE` command handler: flip running, reset ticks, clear the
    /// `SampleSkipped` throttle (the deployment's throttle-clear hook).
    fn toggle_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let running = {
            let mut state = self.state.lock().unwrap();
            state.running = !state.running;
            state.ticks = 0;
            state.running
        };
        self.sample_skipped_throttle.clear();
        self.evt.log_event(
            self.id_base(),
            EVENTID_TOGGLED,
            fprime_fw::LogSeverity::ActivityLo,
            &format!(
                "Signal generation {}",
                if running { "started" } else { "stopped" }
            ),
            |buf| buf.serialize_bool_be(running),
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `SKIP` command handler: zero the next sample; throttled event.
    fn skip_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.state.lock().unwrap().skip_next = true;
        // `throttle 3`: the counter guards both the binary and text paths.
        if self.sample_skipped_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                EVENTID_SAMPLE_SKIPPED,
                fprime_fw::LogSeverity::ActivityLo,
                "Skipping next sample",
                |_buf| fprime_fw::SerializeStatus::Ok,
            );
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `DP(records: U32)` command handler — the SYNCHRONOUS data-product
    /// path (C++ `SignalGen::Dp_cmdHandler` with `DpReqType::IMMEDIATE`,
    /// i.e. the autocoded `dpGet_DataContainer` + `dpSend`).
    ///
    /// The asynchronous request kind (`productRequestOut`/`productRecvIn`)
    /// is deliberately not ported — see the module header.
    fn dp_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut records = 0u32;
        if !args.deserialize_u32_be(&mut records).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        if records == 0 || records > DP_MAX_RECORDS {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }

        // Autocoded `dpGet_DataContainer(dataSize, container)`: ask for the
        // PACKET size that holds `dataSize` bytes of records.
        let container_id = self.data_container_id();
        let data_size = RECORD_SIZE_DATA * FwSizeType::from(records);
        let packet_size = DpContainer::packet_size_for_data_size(data_size);
        let mut buffer = Buffer::empty();
        let status = {
            let p = self.product_get_out.get();
            p.target
                .invoke(p.port_num, container_id, packet_size, &mut buffer)
        };
        if status != Success::Success {
            self.evt.log_event(
                self.id_base(),
                EVENTID_DP_BUFFER_FAILED,
                fprime_fw::LogSeverity::WarningHi,
                &format!("Data product buffer allocation failed for {records} records"),
                |buf| buf.serialize_u32_be(records),
            );
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }

        let mut container = DpContainer::with_buffer(container_id, buffer);
        container.set_priority(CONTAINER_PRIORITY_DATA);
        container.set_time_tag(self.evt.time_get());
        {
            // One `DataRecord` per requested record: `[id u32][value f32]`,
            // exactly what the autocoded `DataContainer_serializeRecord`
            // emits for a single-member record.
            let mut state = self.state.lock().unwrap();
            let mut ser = container.data_serializer();
            for _ in 0..records {
                let value = Self::sample(&state);
                state.dp_samples = state.dp_samples.wrapping_add(1);
                state.ticks = state.ticks.wrapping_add(1);
                let status = ser.serialize_u32_be(self.id_base() + RECORD_ID_DATA);
                fw_assert!(status.is_ok(), status as i32);
                let status = ser.serialize_f32_be(value);
                fw_assert!(status.is_ok(), status as i32);
            }
        }
        container.set_data_size(data_size);
        // Autocoded `dpSend`: finalize the header (which re-hashes it) and
        // hand the buffer to the data-product manager. The DATA hash is
        // `Svc::DpWriter`'s job (C++ parity).
        container.serialize_header();
        let buffer = container.take_buffer();
        let bytes = buffer.size() as u32;
        {
            let p = self.product_send_out.get();
            p.target.invoke(p.port_num, container_id, buffer);
        }

        self.evt.log_event(
            self.id_base(),
            EVENTID_DP_SENT,
            fprime_fw::LogSeverity::ActivityLo,
            &format!("Sent data product with {records} records ({bytes} bytes)"),
            |buf| {
                let status = buf.serialize_u32_be(records);
                if !status.is_ok() {
                    return status;
                }
                buf.serialize_u32_be(bytes)
            },
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `AMPLITUDE_PARAM_SET(val: F32)` — the autocoded parameter-set
    /// command: stage the value locally, run `parameterUpdated`, answer OK.
    /// It does NOT write the parameter database (C++ parity — that is what
    /// the SAVE command does).
    fn amplitude_param_set_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut value = 0f32;
        if !args.deserialize_f32_be(&mut value).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        {
            let mut state = self.state.lock().unwrap();
            state.amplitude_param = value;
            state.amplitude_valid = ParamValid::Valid;
            // `parameterUpdated(Amplitude)`: the live setting follows.
            state.amplitude = value;
        }
        self.evt.log_event(
            self.id_base(),
            EVENTID_AMPLITUDE_UPDATED,
            fprime_fw::LogSeverity::ActivityHi,
            &format!("Amplitude parameter updated to {value}"),
            |buf| buf.serialize_f32_be(value),
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `AMPLITUDE_PARAM_SAVE` — the autocoded parameter-save command:
    /// serialize the staged value and push it to the parameter database
    /// through `prmSetOut`.
    fn amplitude_param_save_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let value = self.state.lock().unwrap().amplitude_param;
        let mut buffer = ParamBuffer::new();
        let status = buffer.serialize_f32_be(value);
        fw_assert!(status.is_ok(), status as i32);
        self.prm
            .set_param(self.id_base(), PARAMID_AMPLITUDE, &mut buffer);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }
}

// -- The sync schedIn port, implemented directly on the component -----------

impl SchedPort for SignalGen {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        self.sched_in_handler(port_num, context);
    }
}

// -- Async command-input adapter --------------------------------------------

/// Adapter for the async `cmdIn` port: envelope args are
/// `[opCode u32][cmdSeq u32][u16 len + arg bytes]` (C++ parity).
struct CmdInAdapter {
    comp: Arc<SignalGen>,
}

impl CmdPort for CmdInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut buf = LinearBuffer::<{ QUEUE_MSG_SIZE as usize }>::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_CMD, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        // Default (`assert`) policy: a full queue is a design error.
        let _ = self
            .comp
            .queued
            .send_message(&buf, CMD_PRIORITY, QueueFullPolicy::Assert);
    }
}

// -- Dispatch: the hand-written doDispatch switch ---------------------------

impl ComponentDispatch for SignalGen {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            MSG_TYPE_CMD => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok()
                    || !buf.deserialize_u32_be(&mut cmd_seq).is_ok()
                    || !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                // C++ parity: dispatch on (opcode - idBase).
                match op_code.wrapping_sub(self.id_base()) {
                    OPCODE_SETTINGS => self.settings_cmd_handler(op_code, cmd_seq, &mut args),
                    OPCODE_TOGGLE => self.toggle_cmd_handler(op_code, cmd_seq, &mut args),
                    OPCODE_SKIP => self.skip_cmd_handler(op_code, cmd_seq, &mut args),
                    OPCODE_DP => self.dp_cmd_handler(op_code, cmd_seq, &mut args),
                    OPCODE_AMPLITUDE_PARAM_SET => {
                        self.amplitude_param_set_handler(op_code, cmd_seq, &mut args)
                    }
                    OPCODE_AMPLITUDE_PARAM_SAVE => {
                        self.amplitude_param_save_handler(op_code, cmd_seq, &mut args)
                    }
                    _ => self
                        .cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
                }
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{LogBuffer, LogSeverity, TextLogString, Time, TimeBase, TlmBuffer};

    const ID_BASE: u32 = 0x1001_1000;

    #[derive(Default)]
    struct Ground {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        /// `prmSetOut` traffic: (absolute id, serialized value).
        prm_sets: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
        /// What `prmGetOut` answers, and with which validity.
        prm_get: Mutex<Option<(Vec<u8>, ParamValid)>>,
    }

    impl fprime_comp::PrmGetPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwPrmIdType,
            val: &mut ParamBuffer,
        ) -> ParamValid {
            match &*self.prm_get.lock().unwrap() {
                Some((bytes, valid)) => {
                    let status = val.set_buff(bytes);
                    assert!(status.is_ok());
                    *valid
                }
                None => ParamValid::Invalid,
            }
        }
    }

    impl fprime_comp::PrmSetPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
            self.prm_sets
                .lock()
                .unwrap()
                .push((id, val.as_slice().to_vec()));
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
    impl LogTextPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _text: &mut TextLogString,
        ) {
        }
    }
    impl TlmPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm.lock().unwrap().push((id, val.as_slice().to_vec()));
        }
    }
    struct TimeStub;
    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 7, 0);
        }
    }

    fn build() -> (Arc<SignalGen>, Arc<Ground>) {
        let ground = Arc::new(Ground::default());
        let comp = SignalGen::new("SG1");
        comp.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.prm.prm_get_out.connect(ground.clone(), 0);
        comp.prm.prm_set_out.connect(ground.clone(), 0);
        comp.init(10);
        (comp, ground)
    }

    fn send_cmd(comp: &Arc<SignalGen>, local_opcode: FwOpcodeType, cmd_seq: u32, args: &[u8]) {
        let mut arg_buf = CmdArgBuffer::new();
        assert!(arg_buf.set_buff(args).is_ok());
        let port = comp.cmd_in(0);
        port.target
            .invoke(port.port_num, ID_BASE + local_opcode, cmd_seq, &mut arg_buf);
    }

    fn tick(comp: &Arc<SignalGen>) {
        let sched = comp.sched_in(0);
        sched.target.invoke(sched.port_num, 0);
    }

    fn settings_args(frequency: u32, amplitude: f32, phase: f32, sig_type: u8) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&frequency.to_be_bytes());
        bytes.extend_from_slice(&amplitude.to_be_bytes());
        bytes.extend_from_slice(&phase.to_be_bytes());
        bytes.push(sig_type);
        bytes
    }

    /// Commands queue up and only execute when schedIn drains the queue.
    #[test]
    fn commands_execute_at_sched_in_time() {
        let (comp, ground) = build();
        send_cmd(&comp, OPCODE_TOGGLE, 1, &[]);
        assert!(ground.responses.lock().unwrap().is_empty()); // still queued
        tick(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(ID_BASE + OPCODE_TOGGLE, 1, CmdResponse::Ok)]
        );
        // The toggle tick itself already samples (drain happens FIRST, so
        // the component runs on the same tick it was started).
        assert!(!ground.tlm.lock().unwrap().is_empty());
    }

    /// Not running -> no telemetry; running -> SignalValue + SignalType per
    /// tick; skip zeroes exactly one sample.
    #[test]
    fn sampling_telemetry_and_skip() {
        let (comp, ground) = build();
        tick(&comp);
        assert!(ground.tlm.lock().unwrap().is_empty());

        // amplitude 2.0, frequency 4, sine, phase 0; then start.
        send_cmd(&comp, OPCODE_SETTINGS, 1, &settings_args(4, 2.0, 0.0, 0));
        send_cmd(&comp, OPCODE_TOGGLE, 2, &[]);
        tick(&comp); // drains both, then samples tick 0 -> sin(0) = 0
        send_cmd(&comp, OPCODE_SKIP, 3, &[]);
        tick(&comp); // skip -> forced 0.0 (would be sin(π/2)*2 = 2)
        tick(&comp); // tick 2 -> sin(π)*2 ≈ 0
        tick(&comp); // tick 3 -> sin(3π/2)*2 = -2

        let tlm = ground.tlm.lock().unwrap();
        let values: Vec<f32> = tlm
            .iter()
            .filter(|(id, _)| *id == ID_BASE + CHANID_SIGNAL_VALUE)
            .map(|(_, bytes)| f32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            .collect();
        assert_eq!(values.len(), 4);
        assert!(values[0].abs() < 1e-5);
        assert_eq!(values[1], 0.0); // skipped
        assert!(values[2].abs() < 1e-5);
        assert!((values[3] + 2.0).abs() < 1e-5);
        // SignalType channel rides along (plus one write from SETTINGS).
        let types: Vec<&Vec<u8>> = tlm
            .iter()
            .filter(|(id, _)| *id == ID_BASE + CHANID_SIGNAL_TYPE)
            .map(|(_, bytes)| bytes)
            .collect();
        assert_eq!(types.len(), 5);
        assert!(types.iter().all(|b| **b == vec![SignalType::Sine as u8]));
    }

    /// Triangle math: peak at mid-period, -amplitude at period edges.
    #[test]
    fn triangle_waveform() {
        let (comp, ground) = build();
        send_cmd(&comp, OPCODE_SETTINGS, 1, &settings_args(4, 1.0, 0.0, 1));
        send_cmd(&comp, OPCODE_TOGGLE, 2, &[]);
        for _ in 0..4 {
            tick(&comp);
        }
        let tlm = ground.tlm.lock().unwrap();
        let values: Vec<f32> = tlm
            .iter()
            .filter(|(id, _)| *id == ID_BASE + CHANID_SIGNAL_VALUE)
            .map(|(_, bytes)| f32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            .collect();
        assert_eq!(values, vec![-1.0, 0.0, 1.0, 0.0]);
    }

    /// Response discipline: FormatError on short and residual args,
    /// ValidationError on a zero frequency and an invalid enum byte,
    /// InvalidOpcode on an unknown local opcode.
    #[test]
    fn command_response_discipline() {
        let (comp, ground) = build();
        send_cmd(&comp, OPCODE_SETTINGS, 1, &[0, 0, 0, 1]); // short
        let mut long = settings_args(1, 0.0, 0.0, 0);
        long.push(0xAA); // residual
        send_cmd(&comp, OPCODE_SETTINGS, 2, &long);
        send_cmd(&comp, OPCODE_SETTINGS, 3, &settings_args(0, 1.0, 0.0, 0)); // freq 0
        send_cmd(&comp, OPCODE_SETTINGS, 4, &settings_args(1, 1.0, 0.0, 9)); // bad enum
        send_cmd(&comp, OPCODE_TOGGLE, 5, &[1]); // residual
        send_cmd(&comp, OPCODE_SKIP, 6, &[2]); // residual
        send_cmd(&comp, 0xFF, 7, &[]); // unknown opcode
        tick(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE + OPCODE_SETTINGS, 1, CmdResponse::FormatError),
                (ID_BASE + OPCODE_SETTINGS, 2, CmdResponse::FormatError),
                (ID_BASE + OPCODE_SETTINGS, 3, CmdResponse::ValidationError),
                (ID_BASE + OPCODE_SETTINGS, 4, CmdResponse::ValidationError),
                (ID_BASE + OPCODE_TOGGLE, 5, CmdResponse::FormatError),
                (ID_BASE + OPCODE_SKIP, 6, CmdResponse::FormatError),
                (ID_BASE + 0xFF, 7, CmdResponse::InvalidOpcode),
            ]
        );
        // No event, no state change from any failed command.
        assert!(ground.events.lock().unwrap().is_empty());
        assert!(!comp.state.lock().unwrap().running);
    }

    /// Regression: an oversized uplinked arg buffer (larger than any
    /// declared command) must NOT assert in the async command adapter —
    /// C++ parity sizes the queue message for a full `CmdArgBuffer`, so
    /// the command is enqueued intact and the handler's residual-bytes
    /// check answers `FormatError`.
    #[test]
    fn oversized_command_args_answer_format_error() {
        let (comp, ground) = build();
        // 100 arg bytes for SETTINGS: the four fields deserialize (13
        // bytes), the 87 residual bytes trigger FormatError.
        let mut args = settings_args(4, 2.0, 0.0, 0);
        args.resize(100, 0xAB);
        send_cmd(&comp, OPCODE_SETTINGS, 1, &args);
        // A maximum-size arg buffer must also fit the queue message.
        let full = vec![0u8; CmdArgBuffer::new().capacity()];
        send_cmd(&comp, OPCODE_TOGGLE, 2, &full);
        tick(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE + OPCODE_SETTINGS, 1, CmdResponse::FormatError),
                (ID_BASE + OPCODE_TOGGLE, 2, CmdResponse::FormatError),
            ]
        );
        // No state change from the failed commands.
        assert!(!comp.state.lock().unwrap().running);
    }

    /// SettingsChanged event bytes are exact; SampleSkipped throttles at 3
    /// and TOGGLE clears the throttle.
    #[test]
    fn events_and_throttle() {
        let (comp, ground) = build();
        send_cmd(&comp, OPCODE_SETTINGS, 1, &settings_args(8, 1.5, 0.25, 1));
        tick(&comp);
        {
            let events = ground.events.lock().unwrap();
            assert_eq!(events.len(), 1);
            let (id, severity, args) = &events[0];
            assert_eq!(*id, ID_BASE + EVENTID_SETTINGS_CHANGED);
            assert_eq!(*severity, LogSeverity::ActivityLo);
            let mut expected = Vec::new();
            expected.extend_from_slice(&8u32.to_be_bytes());
            expected.extend_from_slice(&1.5f32.to_be_bytes());
            expected.extend_from_slice(&0.25f32.to_be_bytes());
            expected.push(1);
            assert_eq!(args, &expected);
        }

        // 5 skips: only 3 SampleSkipped events emitted.
        for seq in 2..7 {
            send_cmd(&comp, OPCODE_SKIP, seq, &[]);
        }
        tick(&comp);
        let skipped = |ground: &Ground| {
            ground
                .events
                .lock()
                .unwrap()
                .iter()
                .filter(|(id, _, _)| *id == ID_BASE + EVENTID_SAMPLE_SKIPPED)
                .count()
        };
        assert_eq!(skipped(&ground), 3);
        // TOGGLE clears the throttle; the next skip logs again.
        send_cmd(&comp, OPCODE_TOGGLE, 7, &[]);
        send_cmd(&comp, OPCODE_SKIP, 8, &[]);
        tick(&comp);
        assert_eq!(skipped(&ground), 4);
    }

    /// reg_commands registers every absolute opcode, in declaration order.
    #[test]
    fn registration() {
        let (comp, ground) = build();
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![
                ID_BASE + OPCODE_SETTINGS,
                ID_BASE + OPCODE_TOGGLE,
                ID_BASE + OPCODE_SKIP,
                ID_BASE + OPCODE_DP,
                ID_BASE + OPCODE_AMPLITUDE_PARAM_SET,
                ID_BASE + OPCODE_AMPLITUDE_PARAM_SAVE,
            ]
        );
    }
    // -- Parameters and data products --------------------------------------

    /// `load_parameters` takes the database value when it is valid.
    #[test]
    fn load_parameters_uses_the_stored_value() {
        let (comp, ground) = build();
        *ground.prm_get.lock().unwrap() = Some((2.5f32.to_be_bytes().to_vec(), ParamValid::Valid));
        comp.load_parameters();
        assert_eq!(comp.amplitude_param(), (2.5, ParamValid::Valid));
    }

    /// An empty database leaves the FPP default in force (C++ parity: the
    /// component keeps its declared default and records the validity).
    #[test]
    fn load_parameters_falls_back_to_the_default() {
        let (comp, _ground) = build();
        comp.load_parameters();
        assert_eq!(
            comp.amplitude_param(),
            (AMPLITUDE_DEFAULT, ParamValid::Invalid)
        );
    }

    /// A stored value that cannot be deserialized is reported INVALID and
    /// the default is used.
    #[test]
    fn load_parameters_rejects_a_corrupt_stored_value() {
        let (comp, ground) = build();
        *ground.prm_get.lock().unwrap() = Some((vec![0x01], ParamValid::Valid));
        comp.load_parameters();
        assert_eq!(
            comp.amplitude_param(),
            (AMPLITUDE_DEFAULT, ParamValid::Invalid)
        );
    }

    /// PARAM_SET stages the value locally and does NOT touch the database;
    /// PARAM_SAVE is what writes it, as the absolute parameter id.
    #[test]
    fn param_set_stages_and_param_save_writes_the_database() {
        let (comp, ground) = build();
        send_cmd(&comp, OPCODE_AMPLITUDE_PARAM_SET, 1, &2.5f32.to_be_bytes());
        tick(&comp);
        assert_eq!(comp.amplitude_param(), (2.5, ParamValid::Valid));
        assert!(ground.prm_sets.lock().unwrap().is_empty());

        send_cmd(&comp, OPCODE_AMPLITUDE_PARAM_SAVE, 2, &[]);
        tick(&comp);
        assert_eq!(
            *ground.prm_sets.lock().unwrap(),
            vec![(ID_BASE + PARAMID_AMPLITUDE, 2.5f32.to_be_bytes().to_vec())]
        );
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE + OPCODE_AMPLITUDE_PARAM_SET, 1, CmdResponse::Ok),
                (ID_BASE + OPCODE_AMPLITUDE_PARAM_SAVE, 2, CmdResponse::Ok),
            ]
        );
    }

    /// `DP` validates its record count before touching the data-product
    /// ports (which are unconnected here — reaching them would assert).
    #[test]
    fn dp_command_validates_the_record_count() {
        let (comp, ground) = build();
        send_cmd(&comp, OPCODE_DP, 1, &0u32.to_be_bytes());
        send_cmd(&comp, OPCODE_DP, 2, &(DP_MAX_RECORDS + 1).to_be_bytes());
        send_cmd(&comp, OPCODE_DP, 3, &[0x00]);
        tick(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE + OPCODE_DP, 1, CmdResponse::ValidationError),
                (ID_BASE + OPCODE_DP, 2, CmdResponse::ValidationError),
                (ID_BASE + OPCODE_DP, 3, CmdResponse::FormatError),
            ]
        );
    }
}
