//! # ExampleComponent — THE exemplar for hand-writing F Prime components
//!
//! This integration test is the normative template that `fprime-svc`
//! component implementations follow. It hand-writes everything the C++ FPP
//! autocoder would generate for a small **active** component:
//!
//! - an async input port (`run_in`, `Svc.Sched`) whose adapter serializes
//!   the byte-exact queue envelope and enqueues with the `Drop` policy,
//! - a sync **guarded** input port (`ping_in`, `Svc.Ping`) running on the
//!   caller's thread under the component's state mutex,
//! - an async command (`SET_VALUE`) with `CmdGlue` registration/response,
//!   argument deserialization, `FormatError` on residual bytes, and
//!   `InvalidOpcode` for unknown opcodes,
//! - an event with `throttle 2` plus a telemetry write,
//! - `preamble()` / `finalizer()` lifecycle hooks.
//!
//! Component pattern summary (copy this structure):
//!
//! 1. Declare FPP-dictionary constants: message types (starting at 1 — 0 is
//!    the EXIT sentinel), relative opcodes/event ids/channel ids, queue
//!    priorities, and the queue message size (max over async invocations).
//! 2. The component struct embeds `ActiveBase` (or `QueuedBase` /
//!    `PassiveBase`), the glue blocks it needs, per-event `EventThrottle`s,
//!    and a `Mutex<State>` for mutable state (this mutex doubles as the
//!    C++ guarded-port mutex).
//! 3. Each async input port gets an adapter struct holding
//!    `Arc<TheComponent>`: `invoke` = serialize envelope + `send_message`.
//!    Each sync/guarded input port is implemented directly on the component
//!    (or on an adapter) calling the handler on the caller's thread.
//!    Factory methods (`run_in(&arc, port_num) -> PortRef<dyn ...>`) expose
//!    them for topology wiring.
//! 4. `ComponentDispatch::dispatch_message` reads `port_num`, matches on
//!    `msg_type`, deserializes args in declaration order, and calls the
//!    handler — mirroring the generated `doDispatch` switch.
//! 5. Topology order: construct -> set_id_base -> wire ports ->
//!    create_queue -> reg_commands -> start -> ... -> exit -> join.

use fprime_comp::msg;
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, CmdRegPort, CmdResponsePort, ComponentDispatch,
    EventGlue, EventThrottle, LogPort, LogTextPort, MsgDispatchStatus, PingPort, PortRef,
    QueueFullPolicy, SchedPort, TimePort, TlmGlue, TlmPort,
};
use fprime_config::{
    FwChanIdType, FwEnumStoreType, FwEventIdType, FwIndexType, FwOpcodeType, FwQueuePriorityType,
    FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, Endianness, LinearBuffer, LogBuffer, LogSeverity, SerBuf, SerBufAny,
    TextLogString, Time, TimeBase, TlmBuffer, fw_assert,
};
use fprime_os::queue::{BlockingType, Status as QueueStatus};
use fprime_os::task::{Status as TaskStatus, TASK_DEFAULT};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Dictionary constants (what the FPP model would declare).
// ---------------------------------------------------------------------------

/// Queue message types. 0 is reserved for the EXIT sentinel; component
/// discriminants start at 1, one per async input port / async command /
/// internal interface.
const MSG_TYPE_RUN_IN: FwEnumStoreType = 1;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 2;

/// Relative (pre-`id_base`) command opcode, event id, telemetry channel id.
const OPCODE_SET_VALUE: FwOpcodeType = 0x10;
const EVENTID_VALUE_SET: FwEventIdType = 0x01;
const EVENTID_VALUE_SET_THROTTLE: u32 = 2; // FPP `throttle 2`
const CHANID_VALUE: FwChanIdType = 0x02;

/// A `run_in` context value that clears the VALUE_SET throttle (stands in
/// for the generated `..._ThrottleClear()` being called from a handler).
const RUN_CONTEXT_CLEAR_THROTTLE: u32 = 0xC1EA6;

/// Queue sizing: message size = max over all async invocations
/// (here the command envelope: 4 + 2 + 4 + 4 + 2 + args).
const QUEUE_DEPTH: FwSizeType = 16;
const MSG_SIZE: FwSizeType = 128;

/// FPP `priority` qualifiers. Both async inputs share one priority so the
/// dispatch order in the tests is pure FIFO (priority queues drain
/// numerically-higher priorities first).
const RUN_IN_PRIORITY: FwQueuePriorityType = 1;
const CMD_IN_PRIORITY: FwQueuePriorityType = 1;

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// Mutable component state. The `Mutex` around it is the component's
/// guarded-port mutex (C++: ONE mutex per component shared by all guarded
/// entry points) — guarded handlers and the component thread both lock it.
#[derive(Default)]
struct ExampleState {
    value: u32,
    run_contexts: Vec<u32>,
    ping_keys: Vec<u32>,
    journal: Vec<String>,
}

struct ExampleComponent {
    /// Active core: PassiveBase (name/id_base/instance) + queue + task.
    active: ActiveBase,
    /// Command registration + response ports.
    cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    evt: EventGlue,
    /// Telemetry port.
    tlm: TlmGlue,
    /// One throttle per `throttle N` event.
    value_set_throttle: EventThrottle,
    /// Guarded state (see [`ExampleState`]).
    state: Mutex<ExampleState>,
}

impl ExampleComponent {
    fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            value_set_throttle: EventThrottle::new(EVENTID_VALUE_SET_THROTTLE),
            state: Mutex::new(ExampleState::default()),
        })
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    fn reg_commands(&self) {
        self.cmd.reg_commands(self.id_base(), &[OPCODE_SET_VALUE]);
    }

    // -- Input-port factories (topology wiring surface) --------------------

    /// `run_in` — ASYNC `Svc.Sched` input, `drop` queue-full policy.
    fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(RunInAdapter { comp: self.clone() }), port_num)
    }

    /// `ping_in` — SYNC GUARDED `Svc.Ping` input: the component itself
    /// implements the port trait; the handler runs on the caller's thread
    /// under the component mutex.
    fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `cmd_in` — ASYNC `Fw.Cmd` input (`async command` in FPP), default
    /// (`assert`) queue-full policy.
    fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers (run on the component thread unless noted) ----------------

    /// `run_in` handler (from the dispatch loop).
    fn run_handler(&self, _port_num: FwIndexType, context: u32) {
        if context == RUN_CONTEXT_CLEAR_THROTTLE {
            // The generated `log_..._ThrottleClear()` equivalent.
            self.value_set_throttle.clear();
        }
        let mut state = self.state.lock().unwrap();
        state.run_contexts.push(context);
    }

    /// `ping_in` handler — GUARDED: runs on the CALLER's thread.
    fn ping_handler(&self, _port_num: FwIndexType, key: u32) {
        let mut state = self.state.lock().unwrap();
        state.ping_keys.push(key);
    }

    /// `SET_VALUE(value: U32)` command handler (async — runs on the
    /// component thread). Demonstrates the exactly-once response
    /// discipline: FormatError on short args AND on residual bytes.
    fn set_value_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut value = 0u32;
        if !args.deserialize_u32_be(&mut value).is_ok() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        // C++ parity (FW_CMD_CHECK_RESIDUAL): leftover bytes after the last
        // declared argument are a FormatError.
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }

        // State update under the component mutex; ports invoked AFTER the
        // lock is dropped (convention: no output-port calls under the lock).
        {
            let mut state = self.state.lock().unwrap();
            state.value = value;
        }

        // Event with `throttle 2`: the throttle guards BOTH the binary and
        // text paths (C++ parity: the counter is checked at the top of the
        // generated log_* method).
        if self.value_set_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                EVENTID_VALUE_SET,
                LogSeverity::ActivityHi,
                &format!("Value set to {value}"),
                |buf| buf.serialize_u32_be(value),
            );
        }

        // Telemetry write (tlmWrite_Value equivalent).
        self.tlm
            .tlm_write(self.id_base(), CHANID_VALUE, &value, self.evt.time_get());

        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }
}

// -- Async input adapters (the generated static thunks, hand-written) -------

/// Adapter for the async `run_in` port: serialize the envelope, enqueue
/// with the `Drop` policy (a full queue silently drops and counts).
struct RunInAdapter {
    comp: Arc<ExampleComponent>,
}

impl SchedPort for RunInAdapter {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        let mut buf = LinearBuffer::<{ MSG_SIZE as usize }>::new();
        // Envelope: [msg_type i32 BE][port_num i16 BE][args...] — serialize
        // failures are programmer errors (the buffer is sized for the worst
        // case), hence fw_assert (C++ parity).
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_RUN_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        // Drop policy: Full is not an error here — ignore the status.
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, RUN_IN_PRIORITY, QueueFullPolicy::Drop);
    }
}

/// Adapter for the async command input: envelope args are
/// `[opCode u32][cmdSeq u32][CmdArgBuffer: u16 len + bytes]` (C++ parity —
/// the arg buffer nests length-prefixed inside the message).
struct CmdInAdapter {
    comp: Arc<ExampleComponent>,
}

impl CmdPort for CmdInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut buf = LinearBuffer::<{ MSG_SIZE as usize }>::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_CMD_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        // Default (`assert`) policy: send_message fw_asserts on failure.
        let _ =
            self.comp
                .active
                .queued
                .send_message(&buf, CMD_IN_PRIORITY, QueueFullPolicy::Assert);
    }
}

// -- The guarded sync port, implemented directly on the component -----------

impl PingPort for ExampleComponent {
    fn invoke(&self, port_num: FwIndexType, key: u32) {
        // Guarded: caller's thread, component mutex taken inside the
        // handler.
        self.ping_handler(port_num, key);
    }
}

// -- Dispatch: the hand-written doDispatch switch ---------------------------

impl ComponentDispatch for ExampleComponent {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        // The base already consumed msg_type and handled EXIT; every
        // remaining message carries port_num next.
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            MSG_TYPE_RUN_IN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.run_handler(port_num, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_CMD_IN => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                if !buf.deserialize_u32_be(&mut cmd_seq).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                if !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                // C++ parity: dispatch on (opcode - idBase); unknown local
                // opcodes answer InvalidOpcode.
                match op_code.wrapping_sub(self.id_base()) {
                    OPCODE_SET_VALUE => self.set_value_cmd_handler(op_code, cmd_seq, &mut args),
                    _ => self
                        .cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
                }
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }

    fn preamble(&self) {
        self.state.lock().unwrap().journal.push("preamble".into());
    }

    fn finalizer(&self) {
        self.state.lock().unwrap().journal.push("finalizer".into());
    }
}

impl ActiveComponent for ExampleComponent {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Ground-side stubs: record every invocation for assertions.
// ---------------------------------------------------------------------------

/// (id, seconds, severity, raw arg bytes)
type EventRecord = (FwEventIdType, u32, LogSeverity, Vec<u8>);

#[derive(Default)]
struct GroundStub {
    regs: Mutex<Vec<FwOpcodeType>>,
    responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    events: Mutex<Vec<EventRecord>>,
    text_events: Mutex<Vec<(FwEventIdType, LogSeverity, String)>>,
    /// (id, seconds, raw value bytes)
    tlm: Mutex<Vec<(FwChanIdType, u32, Vec<u8>)>>,
}

impl CmdRegPort for GroundStub {
    fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
        self.regs.lock().unwrap().push(op_code);
    }
}

impl CmdResponsePort for GroundStub {
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

impl LogPort for GroundStub {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &mut Time,
        severity: LogSeverity,
        args: &mut LogBuffer,
    ) {
        self.events.lock().unwrap().push((
            id,
            time_tag.get_seconds(),
            severity,
            args.as_slice().to_vec(),
        ));
    }
}

impl LogTextPort for GroundStub {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        severity: LogSeverity,
        text: &mut TextLogString,
    ) {
        self.text_events.lock().unwrap().push((
            id,
            severity,
            text.as_str().unwrap_or_default().to_string(),
        ));
    }
}

impl TlmPort for GroundStub {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) {
        self.tlm
            .lock()
            .unwrap()
            .push((id, time_tag.get_seconds(), val.as_slice().to_vec()));
    }
}

/// Time source stub: always 100.000042 (TB_WORKSTATION_TIME).
struct TimeStub;

impl TimePort for TimeStub {
    fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
        *time = Time::new(TimeBase::TbWorkstationTime, 0, 100, 42);
    }
}

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

const ID_BASE: u32 = 0x100;

/// Build a wired (but not yet started) component + ground stub. Follows the
/// topology phase order: construct -> set_id_base -> connect -> create
/// queue.
fn build() -> (Arc<ExampleComponent>, Arc<GroundStub>) {
    let ground = Arc::new(GroundStub::default());
    let comp = ExampleComponent::new("exampleComp");
    comp.active.queued.base.set_id_base(ID_BASE);
    comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
    comp.cmd.cmd_response_out.connect(ground.clone(), 0);
    comp.evt.log_out.connect(ground.clone(), 0);
    comp.evt.text_log_out.connect(ground.clone(), 0);
    comp.evt.time_out.connect(Arc::new(TimeStub), 0);
    comp.tlm.tlm_out.connect(ground.clone(), 0);
    comp.active.queued.create_queue(QUEUE_DEPTH, MSG_SIZE);
    (comp, ground)
}

/// Send a raw command through the `cmd_in` port (what CmdDispatcher does).
fn send_cmd(comp: &Arc<ExampleComponent>, op_code: FwOpcodeType, cmd_seq: u32, arg_bytes: &[u8]) {
    let mut args = CmdArgBuffer::new();
    let status = args.set_buff(arg_bytes);
    assert!(status.is_ok());
    let port = comp.cmd_in(0);
    port.target
        .invoke(port.port_num, op_code, cmd_seq, &mut args);
}

/// Receive one raw message off the component queue (serial-level tap used
/// by the byte-exactness tests).
fn tap_queue(comp: &ExampleComponent) -> (Vec<u8>, FwQueuePriorityType) {
    let mut dest = [0u8; MSG_SIZE as usize];
    let mut size: FwSizeType = 0;
    let mut priority: FwQueuePriorityType = 0;
    let status = comp.active.queued.queue().receive(
        &mut dest,
        BlockingType::NonBlocking,
        &mut size,
        &mut priority,
    );
    assert_eq!(status, QueueStatus::OpOk);
    (dest[..size as usize].to_vec(), priority)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

/// Serial-level tap: the async sched invocation produces the byte-exact
/// C++ queue envelope `[msg_type i32 BE][port_num i16 BE][context u32 BE]`.
#[test]
fn sched_envelope_bytes_are_byte_exact() {
    let (comp, _ground) = build();
    let run = comp.run_in(3);
    run.target.invoke(run.port_num, 0x0102_0304);
    let (bytes, priority) = tap_queue(&comp);
    assert_eq!(
        bytes,
        vec![
            0x00, 0x00, 0x00, 0x01, // msg_type = MSG_TYPE_RUN_IN (i32 BE)
            0x00, 0x03, // port_num = 3 (i16 BE)
            0x01, 0x02, 0x03, 0x04, // context (u32 BE)
        ]
    );
    assert_eq!(priority, RUN_IN_PRIORITY);
}

/// Serial-level tap: the async command envelope nests the arg buffer
/// length-prefixed: `[msg_type][port_num][opCode][cmdSeq][u16 len][args]`.
#[test]
fn command_envelope_bytes_are_byte_exact() {
    let (comp, _ground) = build();
    send_cmd(&comp, ID_BASE + OPCODE_SET_VALUE, 7, &[0, 0, 0, 42]);
    let (bytes, priority) = tap_queue(&comp);
    assert_eq!(
        bytes,
        vec![
            0x00, 0x00, 0x00, 0x02, // msg_type = MSG_TYPE_CMD_IN (i32 BE)
            0x00, 0x00, // port_num = 0 (i16 BE)
            0x00, 0x00, 0x01, 0x10, // opCode = 0x110 (u32 BE)
            0x00, 0x00, 0x00, 0x07, // cmdSeq = 7 (u32 BE)
            0x00, 0x04, // arg buffer length prefix (u16 BE)
            0x00, 0x00, 0x00, 0x2A, // the U32 argument
        ]
    );
    assert_eq!(priority, CMD_IN_PRIORITY);
}

/// The `Drop` queue-full policy silently discards and counts.
#[test]
fn run_in_drop_policy_counts_overflow() {
    let comp = ExampleComponent::new("dropper");
    comp.active.queued.create_queue(2, MSG_SIZE); // tiny queue
    let run = comp.run_in(0);
    for context in 0..5 {
        run.target.invoke(run.port_num, context);
    }
    assert_eq!(comp.active.queued.get_num_msgs_dropped(), 3);
    assert_eq!(comp.active.queued.queue().get_messages_available(), 2);
}

/// End-to-end: register, start, drive async + guarded ports and commands,
/// exit, join — then assert the complete observable behavior.
#[test]
fn full_lifecycle_end_to_end() {
    let (comp, ground) = build();

    // Phase: reg_commands (before tasks start).
    comp.reg_commands();
    assert_eq!(
        *ground.regs.lock().unwrap(),
        vec![ID_BASE + OPCODE_SET_VALUE]
    );

    // Phase: start tasks.
    comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);

    // GUARDED sync port: executes immediately on THIS thread (no queue).
    let ping = comp.ping_in(4);
    ping.target.invoke(ping.port_num, 0xAB);
    assert_eq!(comp.state.lock().unwrap().ping_keys, vec![0xAB]);

    // Async traffic (all priority 1 => strict FIFO dispatch order):
    let run = comp.run_in(0);
    let opcode = ID_BASE + OPCODE_SET_VALUE;
    run.target.invoke(run.port_num, 500); //  sched tick
    send_cmd(&comp, opcode, 1, &[0, 0, 0, 42]); //  Ok + event 1 + tlm
    send_cmd(&comp, opcode, 2, &[0, 0, 0, 43]); //  Ok + event 2 + tlm
    send_cmd(&comp, opcode, 3, &[0, 0, 0, 44]); //  Ok + tlm, event THROTTLED
    run.target.invoke(run.port_num, RUN_CONTEXT_CLEAR_THROTTLE); // clears throttle
    send_cmd(&comp, opcode, 4, &[0, 0, 0, 45]); //  Ok + event 3 (throttle cleared) + tlm
    send_cmd(&comp, opcode, 5, &[0, 0, 0, 46, 9]); //  residual byte -> FormatError
    send_cmd(&comp, opcode, 6, &[0, 0]); //  short arg -> FormatError
    send_cmd(&comp, ID_BASE + 0xFF, 7, &[]); //  unknown opcode -> InvalidOpcode

    // Phase: teardown. EXIT is priority 0 and every message above is
    // priority 1, so the whole backlog drains before the loop exits —
    // join() is the synchronization point, no sleeps needed.
    comp.active.exit();
    assert_eq!(comp.active.join(), TaskStatus::OpOk);

    // Lifecycle hooks ran, on the component thread, in order.
    let state = comp.state.lock().unwrap();
    assert_eq!(state.journal, vec!["preamble", "finalizer"]);
    assert_eq!(state.run_contexts, vec![500, RUN_CONTEXT_CLEAR_THROTTLE]);
    assert_eq!(state.value, 45); // seq 5 (value 46) failed with FormatError
    drop(state);

    // Command responses: exactly one per command, in dispatch order.
    assert_eq!(
        *ground.responses.lock().unwrap(),
        vec![
            (opcode, 1, CmdResponse::Ok),
            (opcode, 2, CmdResponse::Ok),
            (opcode, 3, CmdResponse::Ok),
            (opcode, 4, CmdResponse::Ok),
            (opcode, 5, CmdResponse::FormatError),
            (opcode, 6, CmdResponse::FormatError),
            (ID_BASE + 0xFF, 7, CmdResponse::InvalidOpcode),
        ]
    );

    // Events: throttle 2 suppressed the third SET_VALUE event; the clear
    // re-enabled the fourth. Ids are id_base-offset, args are the raw
    // serialized U32, time comes from the TimeStub.
    let event_id = ID_BASE + EVENTID_VALUE_SET;
    assert_eq!(
        *ground.events.lock().unwrap(),
        vec![
            (event_id, 100, LogSeverity::ActivityHi, vec![0, 0, 0, 42]),
            (event_id, 100, LogSeverity::ActivityHi, vec![0, 0, 0, 43]),
            (event_id, 100, LogSeverity::ActivityHi, vec![0, 0, 0, 45]),
        ]
    );
    assert_eq!(
        *ground.text_events.lock().unwrap(),
        vec![
            (event_id, LogSeverity::ActivityHi, "Value set to 42".into()),
            (event_id, LogSeverity::ActivityHi, "Value set to 43".into()),
            (event_id, LogSeverity::ActivityHi, "Value set to 45".into()),
        ]
    );

    // Telemetry: one write per successful command (FormatError paths
    // return before the tlm write).
    let chan_id = ID_BASE + CHANID_VALUE;
    assert_eq!(
        *ground.tlm.lock().unwrap(),
        vec![
            (chan_id, 100, vec![0, 0, 0, 42]),
            (chan_id, 100, vec![0, 0, 0, 43]),
            (chan_id, 100, vec![0, 0, 0, 44]),
            (chan_id, 100, vec![0, 0, 0, 45]),
        ]
    );
}

/// A failed command must not touch state (response-then-return discipline).
#[test]
fn format_error_leaves_state_untouched() {
    let (comp, ground) = build();
    comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);
    let opcode = ID_BASE + OPCODE_SET_VALUE;
    send_cmd(&comp, opcode, 1, &[0, 0, 0, 9]);
    send_cmd(&comp, opcode, 2, &[1, 2, 3, 4, 5]); // residual byte
    comp.active.exit();
    assert_eq!(comp.active.join(), TaskStatus::OpOk);
    assert_eq!(comp.state.lock().unwrap().value, 9);
    assert_eq!(
        *ground.responses.lock().unwrap(),
        vec![
            (opcode, 1, CmdResponse::Ok),
            (opcode, 2, CmdResponse::FormatError),
        ]
    );
}
