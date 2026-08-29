//! # MacroComponent — the codegen layer proved against the hand-written one
//!
//! `example_component.rs` is the normative HAND-WRITTEN exemplar. This test
//! builds the same shapes with the `fprime_comp::macros` codegen layer
//! (`component_msg_types!`, `input_port_adapter!`,
//! `async_input_port_adapter!`) and asserts the generated code is
//! byte-for-byte identical to hand-written serialization — the property that
//! lets a real component be migrated to the macros without touching the wire.
//!
//! Covered: all four argument passing modes (`val`/`ref`/`mut`/`buf`), all
//! four FPP queue-full policies expressible here (`assert`, `drop`, `hook`),
//! `pre_msg_hook`, sync/guarded inputs including one with a return value,
//! and the `<name>_deserialize` helper matching the adapter's write side.

use fprime_comp::{
    ActiveBase, ActiveComponent, CmdPort, ComponentDispatch, MsgDispatchStatus, PrmGetPort,
    QueueFullPolicy, SchedPort, SuccessConditionPort, async_input_port_adapter,
    component_msg_types, input_port_adapter, msg,
};
use fprime_config::{
    FwEnumStoreType, FwIndexType, FwOpcodeType, FwPrmIdType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, ExtBuf, LinearBuffer, ParamBuffer, ParamValid, SerBuf, SerBufAny, Success,
};
use fprime_os::queue::{BlockingType, Status as QueueStatus};
use fprime_os::task::{Status as TaskStatus, TASK_DEFAULT};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Dictionary constants (what the FPP model would declare).
// ---------------------------------------------------------------------------

/// Queue sizing: message size = max over async invocations.
const MSG_SIZE: usize = 128;
const QUEUE_DEPTH: FwSizeType = 16;

const RUN_IN_PRIORITY: FwQueuePriorityType = 1;
const CMD_IN_PRIORITY: FwQueuePriorityType = 1;
const STATUS_IN_PRIORITY: FwQueuePriorityType = 1;

const ID_BASE: u32 = 0x100;

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

#[derive(Default)]
struct MacroState {
    run_contexts: Vec<u32>,
    commands: Vec<(FwOpcodeType, u32, Vec<u8>)>,
    statuses: Vec<Success>,
    prm_reads: Vec<FwPrmIdType>,
}

/// An active component whose entire input-port surface is macro-generated.
struct MacroComponent {
    active: ActiveBase,
    /// Set by the `run_in` `pre_msg_hook` on the SENDER's thread.
    pre_hook_calls: AtomicU32,
    /// Incremented by the `status_in` overflow hook (FPP `hook` policy).
    overflow_calls: AtomicU32,
    state: Mutex<MacroState>,
}

component_msg_types! {
    /// Queue message types — 0 is the EXIT sentinel, so these start at 1.
    impl MacroComponent {
        /// `runIn` async input port.
        MSG_TYPE_RUN_IN,
        /// `cmdIn` async command port.
        MSG_TYPE_CMD_IN,
        /// `statusIn` async input port (FPP `hook` queue-full policy).
        MSG_TYPE_STATUS_IN,
    }
}

impl MacroComponent {
    fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            pre_hook_calls: AtomicU32::new(0),
            overflow_calls: AtomicU32::new(0),
            state: Mutex::new(MacroState::default()),
        })
    }

    // -- Handlers ----------------------------------------------------------

    fn run_handler(&self, _port_num: FwIndexType, context: u32) {
        self.state.lock().unwrap().run_contexts.push(context);
    }

    fn cmd_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        self.state
            .lock()
            .unwrap()
            .commands
            .push((op_code, cmd_seq, args.as_slice().to_vec()));
    }

    fn status_handler(&self, _port_num: FwIndexType, condition: Success) {
        self.state.lock().unwrap().statuses.push(condition);
    }

    /// GUARDED sync handler: runs on the CALLER's thread.
    fn guarded_status_handler(&self, _port_num: FwIndexType, condition: &mut Success) {
        self.state.lock().unwrap().statuses.push(*condition);
        // C++ `ref` out-parameter: mutate in place for the caller.
        *condition = Success::Success;
    }

    /// SYNC handler with a return value.
    fn prm_get_handler(
        &self,
        _port_num: FwIndexType,
        id: FwPrmIdType,
        val: &mut ParamBuffer,
    ) -> ParamValid {
        self.state.lock().unwrap().prm_reads.push(id);
        let status = val.serialize_u32_be(0xABCD_1234);
        assert!(status.is_ok());
        ParamValid::Valid
    }
}

// -- Async inputs (generated) ------------------------------------------------

async_input_port_adapter! {
    /// `runIn` — ASYNC `Svc.Sched` input, `drop` queue-full policy.
    component: MacroComponent;
    adapter: RunInAdapter;
    port: SchedPort;
    input: run_in;
    deserialize: run_in_deserialize;
    handler: run_handler;
    base: active.queued;
    msg_type: MacroComponent::MSG_TYPE_RUN_IN;
    msg_size: MSG_SIZE;
    priority: RUN_IN_PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    args { val context: u32 }
    pre_msg_hook |comp, _port_num| {
        comp.pre_hook_calls.fetch_add(1, Ordering::Relaxed);
    }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input, default (`assert`) policy. The arg
    /// buffer nests length-prefixed inside the message (C++ parity).
    component: MacroComponent;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_handler;
    base: active.queued;
    msg_type: MacroComponent::MSG_TYPE_CMD_IN;
    msg_size: MSG_SIZE;
    priority: CMD_IN_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

async_input_port_adapter! {
    /// `statusIn` — ASYNC `Fw.SuccessCondition` input with the FPP `hook`
    /// queue-full policy: a full queue calls the overflow hook instead of
    /// asserting. The `mut` arg is copied into the message (C++ parity:
    /// the caller's variable is not written back across a queue).
    component: MacroComponent;
    adapter: StatusInAdapter;
    port: SuccessConditionPort;
    input: status_in;
    deserialize: status_in_deserialize;
    handler: status_handler;
    base: active.queued;
    msg_type: MacroComponent::MSG_TYPE_STATUS_IN;
    msg_size: MSG_SIZE;
    priority: STATUS_IN_PRIORITY;
    queue_full: QueueFullPolicy::Hook;
    args { mut condition: Success }
    overflow_hook |comp, _port_num| {
        comp.overflow_calls.fetch_add(1, Ordering::Relaxed);
    }
}

// -- Sync inputs (generated) -------------------------------------------------

input_port_adapter! {
    /// `guardedStatusIn` — GUARDED `Fw.SuccessCondition` input: runs on the
    /// caller's thread and writes the `ref` argument back.
    component: MacroComponent;
    adapter: GuardedStatusInAdapter;
    port: SuccessConditionPort;
    input: guarded_status_in;
    handler: guarded_status_handler;
    args { mut condition: Success }
}

input_port_adapter! {
    /// `prmGetIn` — SYNC `Fw.PrmGet` input, which RETURNS a value.
    component: MacroComponent;
    adapter: PrmGetInAdapter;
    port: PrmGetPort;
    input: prm_get_in;
    handler: prm_get_handler;
    returns: ParamValid;
    args { val id: FwPrmIdType, buf val: ParamBuffer }
}

// -- Dispatch ----------------------------------------------------------------

impl ComponentDispatch for MacroComponent {
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
            Self::MSG_TYPE_RUN_IN => match Self::run_in_deserialize(buf) {
                Some((context,)) => {
                    self.run_handler(port_num, context);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CMD_IN => match Self::cmd_in_deserialize(buf) {
                Some((op_code, cmd_seq, mut args)) => {
                    self.cmd_handler(port_num, op_code, cmd_seq, &mut args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_STATUS_IN => match Self::status_in_deserialize(buf) {
                Some((condition,)) => {
                    self.status_handler(port_num, condition);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for MacroComponent {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn build() -> Arc<MacroComponent> {
    let comp = MacroComponent::new("macroComp");
    comp.active.queued.base.set_id_base(ID_BASE);
    comp.active
        .queued
        .create_queue(QUEUE_DEPTH, MSG_SIZE as FwSizeType);
    comp
}

/// Receive one raw message off the component queue.
fn tap_queue(comp: &MacroComponent) -> (Vec<u8>, FwQueuePriorityType) {
    let mut dest = [0u8; MSG_SIZE];
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

/// `component_msg_types!` numbers from 1; 0 stays the EXIT sentinel.
#[test]
fn msg_types_are_numbered_from_one_after_exit() {
    assert_eq!(msg::EXIT_MSG_TYPE, 0);
    assert_eq!(MacroComponent::MSG_TYPE_RUN_IN, 1);
    assert_eq!(MacroComponent::MSG_TYPE_CMD_IN, 2);
    assert_eq!(MacroComponent::MSG_TYPE_STATUS_IN, 3);
}

/// The generated envelope is byte-for-byte what hand-written serialization
/// produces — the regression property that makes migration safe.
#[test]
fn generated_envelope_equals_hand_written_serialization() {
    let comp = build();
    let run = comp.run_in(3);
    run.target.invoke(run.port_num, 0x0102_0304);
    let (generated, priority) = tap_queue(&comp);

    // Hand-written equivalent, exactly as example_component.rs writes it.
    let mut hand = LinearBuffer::<MSG_SIZE>::new();
    assert!(msg::write_envelope_header(&mut hand, MacroComponent::MSG_TYPE_RUN_IN, 3).is_ok());
    assert!(hand.serialize_u32_be(0x0102_0304).is_ok());

    assert_eq!(generated, hand.as_slice());
    assert_eq!(
        generated,
        vec![
            0x00, 0x00, 0x00, 0x01, // msg_type = MSG_TYPE_RUN_IN (i32 BE)
            0x00, 0x03, // port_num = 3 (i16 BE)
            0x01, 0x02, 0x03, 0x04, // context (u32 BE)
        ]
    );
    assert_eq!(priority, RUN_IN_PRIORITY);
}

/// The `buf` mode nests the arg buffer length-prefixed, as C++ does.
#[test]
fn command_envelope_bytes_are_byte_exact() {
    let comp = build();
    let mut args = CmdArgBuffer::new();
    assert!(args.set_buff(&[0, 0, 0, 42]).is_ok());
    let port = comp.cmd_in(0);
    port.target
        .invoke(port.port_num, ID_BASE + 0x10, 7, &mut args);

    let (bytes, priority) = tap_queue(&comp);
    assert_eq!(
        bytes,
        vec![
            0x00, 0x00, 0x00, 0x02, // msg_type = MSG_TYPE_CMD_IN
            0x00, 0x00, // port_num = 0
            0x00, 0x00, 0x01, 0x10, // opCode
            0x00, 0x00, 0x00, 0x07, // cmdSeq
            0x00, 0x04, // arg buffer length prefix (u16 BE)
            0x00, 0x00, 0x00, 0x2A, // the U32 argument
        ]
    );
    assert_eq!(priority, CMD_IN_PRIORITY);
}

/// The `mut` mode serializes the pointed-to value (1 byte for an FPP
/// `enum : U8`) and does NOT write back across the queue.
#[test]
fn mut_mode_serializes_the_value_and_does_not_write_back() {
    let comp = build();
    let mut condition = Success::Failure;
    let port = comp.status_in(1);
    port.target.invoke(port.port_num, &mut condition);
    assert_eq!(
        condition,
        Success::Failure,
        "async ref args are copy-in only"
    );

    let (bytes, _) = tap_queue(&comp);
    assert_eq!(
        bytes,
        vec![
            0x00, 0x00, 0x00, 0x03, // msg_type = MSG_TYPE_STATUS_IN
            0x00, 0x01, // port_num = 1
            0x00, // Success::Failure at its u8 repr width
        ]
    );
}

/// The generated `<name>_deserialize` reads back exactly what the adapter
/// wrote, for every argument mode.
#[test]
fn deserialize_helpers_round_trip_the_adapter_write() {
    let comp = build();

    let run = comp.run_in(2);
    run.target.invoke(run.port_num, 0xDEAD_BEEF);
    let mut args = CmdArgBuffer::new();
    assert!(args.set_buff(b"argbytes").is_ok());
    let cmd = comp.cmd_in(0);
    cmd.target.invoke(cmd.port_num, 0x1234, 99, &mut args);
    let status = comp.status_in(0);
    let mut condition = Success::Success;
    status.target.invoke(status.port_num, &mut condition);

    for expect in [
        MacroComponent::MSG_TYPE_RUN_IN,
        MacroComponent::MSG_TYPE_CMD_IN,
        MacroComponent::MSG_TYPE_STATUS_IN,
    ] {
        let (mut bytes, _) = tap_queue(&comp);
        let len = bytes.len();
        let mut view = ExtBuf::new(&mut bytes);
        view.set_ser_loc(len);
        let mut msg_type: FwEnumStoreType = 0;
        let mut port_num: FwIndexType = -1;
        assert!(msg::read_msg_type(&mut view, &mut msg_type).is_ok());
        assert!(msg::read_port_num(&mut view, &mut port_num).is_ok());
        assert_eq!(msg_type, expect);
        match msg_type {
            MacroComponent::MSG_TYPE_RUN_IN => {
                let (context,) = MacroComponent::run_in_deserialize(&mut view).unwrap();
                assert_eq!(context, 0xDEAD_BEEF);
                assert_eq!(port_num, 2);
            }
            MacroComponent::MSG_TYPE_CMD_IN => {
                let (op_code, cmd_seq, decoded) =
                    MacroComponent::cmd_in_deserialize(&mut view).unwrap();
                assert_eq!(op_code, 0x1234);
                assert_eq!(cmd_seq, 99);
                assert_eq!(decoded.as_slice(), b"argbytes");
            }
            MacroComponent::MSG_TYPE_STATUS_IN => {
                let (condition,) = MacroComponent::status_in_deserialize(&mut view).unwrap();
                assert_eq!(condition, Success::Success);
            }
            other => panic!("unexpected msg_type {other}"),
        }
        assert_eq!(view.deserialize_size_left(), 0, "no residual bytes");
    }
}

/// A truncated message body makes the helper return `None`, which the
/// dispatch switch turns into `MsgDispatchStatus::Error`.
#[test]
fn deserialize_helper_reports_truncated_messages() {
    let mut bytes = [0u8; 2];
    let mut view = ExtBuf::new(&mut bytes);
    view.set_ser_loc(2);
    assert!(MacroComponent::run_in_deserialize(&mut view).is_none());
}

/// FPP `drop`: a full queue silently discards and counts.
#[test]
fn drop_policy_counts_overflow() {
    let comp = MacroComponent::new("dropper");
    comp.active.queued.create_queue(2, MSG_SIZE as FwSizeType);
    let run = comp.run_in(0);
    for context in 0..5 {
        run.target.invoke(run.port_num, context);
    }
    assert_eq!(comp.active.queued.get_num_msgs_dropped(), 3);
    assert_eq!(comp.active.queued.queue().get_messages_available(), 2);
    // The pre-message hook runs on every invocation, before the enqueue —
    // including the ones that end up dropped.
    assert_eq!(comp.pre_hook_calls.load(Ordering::Relaxed), 5);
}

/// FPP `hook`: a full queue calls the component's overflow hook instead of
/// asserting.
#[test]
fn hook_policy_invokes_the_overflow_hook() {
    let comp = MacroComponent::new("hooker");
    comp.active.queued.create_queue(2, MSG_SIZE as FwSizeType);
    let port = comp.status_in(0);
    for _ in 0..5 {
        port.target.invoke(port.port_num, &mut Success::Success);
    }
    assert_eq!(comp.overflow_calls.load(Ordering::Relaxed), 3);
    assert_eq!(comp.active.queued.queue().get_messages_available(), 2);
    // `hook` does not count drops — that is the `drop` policy's job.
    assert_eq!(comp.active.queued.get_num_msgs_dropped(), 0);
}

/// A guarded sync input runs on the CALLER's thread (no queue traffic) and
/// writes its `ref` argument back.
#[test]
fn guarded_sync_input_runs_inline_and_writes_back() {
    let comp = build();
    let mut condition = Success::Failure;
    let port = comp.guarded_status_in(4);
    port.target.invoke(port.port_num, &mut condition);
    assert_eq!(
        condition,
        Success::Success,
        "ref out-parameter written back"
    );
    assert_eq!(*comp.state.lock().unwrap().statuses, [Success::Failure]);
    assert_eq!(comp.active.queued.queue().get_messages_available(), 0);
}

/// A sync input whose port trait returns a value.
#[test]
fn sync_input_with_return_value() {
    let comp = build();
    let mut val = ParamBuffer::new();
    let port = comp.prm_get_in(0);
    let valid = port.target.invoke(port.port_num, 0x55, &mut val);
    assert_eq!(valid, ParamValid::Valid);
    assert_eq!(val.as_slice(), &[0xAB, 0xCD, 0x12, 0x34]);
    assert_eq!(*comp.state.lock().unwrap().prm_reads, [0x55]);
}

/// End-to-end through the real component task: everything enqueued by the
/// generated adapters is dispatched by the generated deserializers.
#[test]
fn full_lifecycle_through_the_component_thread() {
    let comp = build();
    comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);

    let run = comp.run_in(0);
    run.target.invoke(run.port_num, 11);
    run.target.invoke(run.port_num, 22);

    let mut args = CmdArgBuffer::new();
    assert!(args.set_buff(&[1, 2, 3]).is_ok());
    let cmd = comp.cmd_in(0);
    cmd.target.invoke(cmd.port_num, ID_BASE + 5, 3, &mut args);

    let status = comp.status_in(0);
    status.target.invoke(status.port_num, &mut Success::Success);

    comp.active.exit();
    assert_eq!(comp.active.join(), TaskStatus::OpOk);

    let state = comp.state.lock().unwrap();
    assert_eq!(state.run_contexts, vec![11, 22]);
    assert_eq!(state.commands, vec![(ID_BASE + 5, 3, vec![1, 2, 3])]);
    assert_eq!(state.statuses, vec![Success::Success]);
    assert_eq!(comp.pre_hook_calls.load(Ordering::Relaxed), 2);
    assert_eq!(comp.overflow_calls.load(Ordering::Relaxed), 0);
}
