//! Ports of `Svc/FpySequencer/test/ut` (`FpySequencerTestMain.cpp`,
//! `FpySequencerTestSequences.cpp`, the `FpySequencerTester` harness).
//!
//! The harness mirrors the C++ tester: the component is driven by hand
//! (`do_dispatch` one message at a time), its private handlers and state
//! are reached directly (the tests are a child module), and sequences are
//! built in memory with the `add_*` helpers and written byte-by-byte with
//! [`Tester::write_to_file`], exactly as upstream's `writeToFile` lays
//! them out. Files live in a per-test directory under the system temp dir
//! instead of the working directory, so tests can run in parallel; paths
//! stay under `FW_CMD_STRING_MAX_SIZE` because they travel in commands.

mod commands;
mod deserialize;
mod directives;
mod ops;
mod sequences;
mod stack;
mod validation;

use super::*;
use crate::cmd_sequencer::{CmdSeqInPort, SEQUENCE_ARGUMENTS_MAX_SIZE};
use crate::tlm_chan::TlmGetPort;
use fprime_comp::serial_port::SerializePort;
use fprime_comp::{CmdRegPort, LogPort, PrmSetPort, TimePort, TlmPort};
use fprime_config::FW_CMD_STRING_MAX_SIZE;
use fprime_fw::{ComBuffer, TimeBase, TlmBuffer, TlmValid};
use fprime_os::queue::BlockingType;
use std::path::PathBuf;
use std::sync::MutexGuard;
use std::sync::atomic::{AtomicU32, Ordering};

/// The id base of the component under test.
pub(super) const ID_BASE: FwIdType = 0x100;
/// C++ `TEST_INSTANCE_QUEUE_DEPTH`.
const QUEUE_DEPTH: FwSizeType = 10;

static COUNTER: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Ground stubs.
// ---------------------------------------------------------------------------

/// A recorded event: (FPP-relative id, severity, argument bytes).
pub(super) type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

/// Records every output of the component.
#[derive(Default)]
pub(super) struct Ground {
    pub regs: Mutex<Vec<FwOpcodeType>>,
    pub responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    pub events: Mutex<Vec<EventRecord>>,
    pub tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    pub pings: Mutex<Vec<u32>>,
    pub cmd_out: Mutex<Vec<(Vec<u8>, u32)>>,
    pub seq_start: Mutex<Vec<(Vec<u8>, SeqArgs)>>,
    pub prm_sets: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
    /// The test clock (C++ `setTestTime`).
    pub time: Mutex<Time>,
}

impl CmdRegPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
        self.regs.lock().unwrap().push(op_code - ID_BASE);
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
            .push((op_code.wrapping_sub(ID_BASE), cmd_seq, response));
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
            .push((id - ID_BASE, severity, args.as_slice().to_vec()));
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
        self.tlm
            .lock()
            .unwrap()
            .push((id - ID_BASE, val.as_slice().to_vec()));
    }
}

impl TimePort for Ground {
    fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
        *time = *self.time.lock().unwrap();
    }
}

impl PingPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, key: u32) {
        self.pings.lock().unwrap().push(key);
    }
}

impl ComPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        self.cmd_out
            .lock()
            .unwrap()
            .push((data.as_slice().to_vec(), context));
    }
}

impl CmdSeqInPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, filename: &FileNameString, args: &SeqArgs) {
        self.seq_start
            .lock()
            .unwrap()
            .push((filename.as_bytes().to_vec(), *args));
    }
}

impl PrmSetPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
        self.prm_sets
            .lock()
            .unwrap()
            .push((id - ID_BASE, val.as_slice().to_vec()));
    }
}

/// The parameter database on `prmGet`: knows no parameters.
struct PrmDbStub;

impl PrmGetPort for PrmDbStub {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        _id: FwPrmIdType,
        _val: &mut ParamBuffer,
    ) -> ParamValid {
        ParamValid::Invalid
    }
}

/// `seqDoneOut`, a second `Fw.CmdResponse` destination.
#[derive(Default)]
pub(super) struct SeqDone {
    pub calls: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
}

impl CmdResponsePort for SeqDone {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        self.calls
            .lock()
            .unwrap()
            .push((op_code, cmd_seq, response));
    }
}

/// `getTlmChan` / `getParam` mocks (C++ `from_getTlmChan_handler` /
/// `from_getParam_handler`): answer `next_*_id` with `next_*_value`.
#[derive(Default)]
pub(super) struct DataMock {
    pub next_tlm_id: Mutex<FwChanIdType>,
    pub next_tlm_time: Mutex<Time>,
    pub next_tlm_value: Mutex<TlmBuffer>,
    pub tlm_gets: Mutex<Vec<FwChanIdType>>,
    pub next_prm_id: Mutex<FwPrmIdType>,
    pub next_prm_value: Mutex<ParamBuffer>,
    pub prm_gets: Mutex<Vec<FwPrmIdType>>,
}

impl TlmGetPort for DataMock {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) -> TlmValid {
        self.tlm_gets.lock().unwrap().push(id);
        if id != *self.next_tlm_id.lock().unwrap() {
            let _ = val.set_buff_len(0);
            return TlmValid::Invalid;
        }
        *val = self.next_tlm_value.lock().unwrap().clone();
        *time_tag = *self.next_tlm_time.lock().unwrap();
        TlmValid::Valid
    }
}

impl PrmGetPort for DataMock {
    fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid {
        self.prm_gets.lock().unwrap().push(id);
        if id != *self.next_prm_id.lock().unwrap() {
            return ParamValid::Invalid;
        }
        *val = self.next_prm_value.lock().unwrap().clone();
        ParamValid::Valid
    }
}

/// The `serialOut` calls: (port, bytes).
pub(super) type SerialHistory = Arc<Mutex<Vec<(FwIndexType, Vec<u8>)>>>;

/// `serialOut` (C++ `from_serialOut_handler`): one per port index.
pub(super) struct SerialStub {
    pub calls: SerialHistory,
}

impl SerializePort for SerialStub {
    fn invoke_serial(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) -> SerializeStatus {
        let data = buffer.as_slice().to_vec();
        self.calls.lock().unwrap().push((port_num, data));
        SerializeStatus::Ok
    }
}

// ---------------------------------------------------------------------------
// The tester.
// ---------------------------------------------------------------------------

/// C++ `FpySequencerTester`.
pub(super) struct Tester {
    pub comp: Arc<FpySequencer>,
    pub ground: Arc<Ground>,
    pub seq_done: Arc<SeqDone>,
    pub data: Arc<DataMock>,
    pub serial: SerialHistory,
    /// The sequence being built (C++ `seq`).
    pub seq: Sequence,
    dir: PathBuf,
}

impl Drop for Tester {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

impl Tester {
    /// C++ `FpySequencerTester()`: connect every port (but `serialOut[1]`,
    /// left open for the not-connected test) and init.
    pub fn new() -> Self {
        Self::build(true)
    }

    /// Like [`Self::new`], with `getParam` unconnected (C++
    /// `tester_disconnect_getParam`).
    pub fn without_get_param() -> Self {
        Self::build(false)
    }

    fn build(connect_get_param: bool) -> Self {
        let ground = Arc::new(Ground::default());
        let seq_done = Arc::new(SeqDone::default());
        let data = Arc::new(DataMock::default());
        let serial = Arc::new(Mutex::new(Vec::new()));
        let comp = FpySequencer::new("fpySeq");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.prm.prm_get_out.connect(Arc::new(PrmDbStub), 0);
        comp.prm.prm_set_out.connect(ground.clone(), 0);
        comp.cmd_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.seq_start_out.connect(ground.clone(), 0);
        comp.seq_done_out.connect(seq_done.clone(), 0);
        comp.get_tlm_chan.connect(data.clone(), 0);
        if connect_get_param {
            comp.get_param.connect(data.clone(), 0);
        }
        for (i, port) in comp.serial_out.iter().enumerate() {
            if i == 1 {
                continue;
            }
            port.connect(
                Arc::new(SerialStub {
                    calls: serial.clone(),
                }),
                i as FwIndexType,
            );
        }
        comp.init(QUEUE_DEPTH);
        let mut dir = std::env::temp_dir();
        dir.push(format!(
            "fpy{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let mut t = Self {
            comp,
            ground,
            seq_done,
            data,
            serial,
            seq: Sequence::new(),
            dir,
        };
        t.clear_seq();
        t
    }

    /// The component state.
    pub fn st(&self) -> MutexGuard<'_, FpyState> {
        self.comp.state.lock().unwrap()
    }

    /// Run `f` with the component and its locked state (the C++ tester's
    /// direct calls of private handlers).
    pub fn with<R>(&self, f: impl FnOnce(&FpySequencer, &mut FpyState) -> R) -> R {
        let mut st = self.comp.state.lock().unwrap();
        f(&self.comp, &mut st)
    }

    /// The state machine state.
    pub fn state(&self) -> State {
        self.comp.sequencer_get_state()
    }

    /// C++ `tester_setState`.
    pub fn set_state(&self, state: State) {
        self.comp.sequencer.__set_state(state);
    }

    /// C++ `setTestTime`.
    pub fn set_time(&self, time: Time) {
        *self.ground.time.lock().unwrap() = time;
    }

    /// C++ `tester_doDispatch`: dispatch one message.
    pub fn dispatch(&self) {
        let status = self
            .comp
            .active
            .queued
            .do_dispatch(&*self.comp, BlockingType::NonBlocking);
        assert_eq!(status, MsgDispatchStatus::Ok);
    }

    /// C++ `dispatchCurrentMessages`.
    pub fn dispatch_current(&self) {
        let status = self
            .comp
            .active
            .queued
            .dispatch_available_messages(&*self.comp);
        assert_eq!(status, MsgDispatchStatus::Ok);
    }

    /// C++ `dispatchUntilState(state, bound = 100)`.
    pub fn dispatch_until_state(&self, state: State) {
        self.dispatch_until_state_bounded(state, 100);
    }

    /// C++ `dispatchUntilState(state, bound)`.
    pub fn dispatch_until_state_bounded(&self, state: State, bound: u32) {
        let mut iters = 0;
        while self.state() != state && iters < bound {
            if self.comp.active.queued.queue().get_messages_available() == 0 {
                break;
            }
            self.dispatch();
            iters += 1;
        }
        assert_eq!(self.state(), state);
    }

    /// The number of queued messages.
    pub fn queued(&self) -> FwSizeType {
        self.comp.active.queued.queue().get_messages_available()
    }

    // -- histories -----------------------------------------------------------

    /// C++ `clearHistory`.
    pub fn clear_history(&self) {
        self.ground.responses.lock().unwrap().clear();
        self.ground.events.lock().unwrap().clear();
        self.ground.tlm.lock().unwrap().clear();
        self.ground.pings.lock().unwrap().clear();
        self.ground.cmd_out.lock().unwrap().clear();
        self.ground.seq_start.lock().unwrap().clear();
        self.seq_done.calls.lock().unwrap().clear();
        self.data.tlm_gets.lock().unwrap().clear();
        self.data.prm_gets.lock().unwrap().clear();
        self.serial.lock().unwrap().clear();
    }

    /// The command responses, opcodes FPP-relative.
    pub fn responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
        self.ground.responses.lock().unwrap().clone()
    }

    /// C++ `ASSERT_CMD_RESPONSE_SIZE` + `ASSERT_CMD_RESPONSE(index, ...)`.
    pub fn assert_response(&self, index: usize, opcode: FwOpcodeType, response: CmdResponse) {
        let responses = self.responses();
        assert!(
            index < responses.len(),
            "only {} responses",
            responses.len()
        );
        assert_eq!(responses[index], (opcode, 0, response));
    }

    /// The argument bytes of every event `id`.
    pub fn events(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
        self.ground
            .events
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.0 == id)
            .map(|e| e.2.clone())
            .collect()
    }

    /// The number of events `id`.
    pub fn event_count(&self, id: FwEventIdType) -> usize {
        self.events(id).len()
    }

    /// The number of events.
    pub fn all_events(&self) -> usize {
        self.ground.events.lock().unwrap().len()
    }

    /// `seqDoneOut` calls.
    pub fn seq_done(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
        self.seq_done.calls.lock().unwrap().clone()
    }

    /// `seqStartOut` calls.
    pub fn seq_start(&self) -> Vec<(Vec<u8>, SeqArgs)> {
        self.ground.seq_start.lock().unwrap().clone()
    }

    /// `cmdOut` calls.
    pub fn cmd_out(&self) -> Vec<(Vec<u8>, u32)> {
        self.ground.cmd_out.lock().unwrap().clone()
    }

    /// Telemetry writes.
    pub fn tlm(&self) -> Vec<(FwChanIdType, Vec<u8>)> {
        self.ground.tlm.lock().unwrap().clone()
    }

    /// `serialOut` calls.
    pub fn serial_calls(&self) -> Vec<(FwIndexType, Vec<u8>)> {
        self.serial.lock().unwrap().clone()
    }

    // -- the runtime -----------------------------------------------------------

    /// C++ `tester_push<T>`.
    pub fn push<T: StackValue>(&self, value: T) {
        self.st().runtime.stack.push(value);
    }

    /// C++ `tester_pop<T>`.
    pub fn pop<T: StackValue>(&self) -> T {
        self.st().runtime.stack.pop()
    }

    /// The stack size.
    pub fn stack_size(&self) -> StackSizeType {
        self.st().runtime.stack.size
    }

    /// Set the stack size.
    pub fn set_stack_size(&self, size: StackSizeType) {
        self.st().runtime.stack.size = size;
    }

    /// One stack byte.
    pub fn byte(&self, index: usize) -> u8 {
        self.st().runtime.stack.bytes[index]
    }

    // -- files and sequences ---------------------------------------------------

    /// A path in this test's directory.
    pub fn path(&self, name: &str) -> String {
        let mut path = self.dir.clone();
        path.push(name);
        let path = path.to_str().unwrap().to_string();
        assert!(
            path.len() <= FW_CMD_STRING_MAX_SIZE,
            "test path {path} does not fit a command string argument"
        );
        path
    }

    /// C++ `allocMem()`: a buffer for the largest sequence.
    pub fn alloc_mem(&self) {
        self.comp
            .allocate_buffer(0, Sequence::SERIALIZED_SIZE as FwSizeType);
    }

    /// C++ `clearSeq`.
    pub fn clear_seq(&mut self) {
        self.seq.clear();
        self.seq.header.schema_version = SCHEMA_VERSION;
    }

    /// The bytes of the sequence being built, as upstream's `writeToFile`
    /// lays them out (it also sets the header's body size).
    pub fn sequence_bytes(&mut self) -> Vec<u8> {
        // heap storage: a full sequence is about 1 MiB
        let mut body_storage = vec![0u8; Sequence::SERIALIZED_SIZE];
        let mut body = fprime_fw::ExtBuf::new(&mut body_storage);
        for arg in &self.seq.args[..usize::from(self.seq.header.argument_count)] {
            assert!(body.serialize(arg, Endianness::Big).is_ok());
        }
        for stmt in &self.seq.statements[..usize::from(self.seq.header.statement_count)] {
            assert!(body.serialize(stmt, Endianness::Big).is_ok());
        }
        self.seq.header.body_size = body.get_size() as u32;
        let mut out_storage = vec![0u8; Sequence::SERIALIZED_SIZE];
        let mut out = fprime_fw::ExtBuf::new(&mut out_storage);
        assert!(out.serialize(&self.seq.header, Endianness::Big).is_ok());
        assert!(
            out.serialize_bytes(body.as_slice(), LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        let mut hash = Hash::new();
        hash.update(out.as_slice());
        self.seq.footer.crc = hash.finalize();
        assert!(out.serialize(&self.seq.footer, Endianness::Big).is_ok());
        out.as_slice().to_vec()
    }

    /// C++ `writeToFile(name)`: returns the path.
    pub fn write_to_file(&mut self, name: &str) -> String {
        self.write_to_file_max(name, usize::MAX)
    }

    /// C++ `writeToFile(name, maxBytes)`.
    pub fn write_to_file_max(&mut self, name: &str, max_bytes: usize) -> String {
        let bytes = self.sequence_bytes();
        let path = self.path(name);
        std::fs::write(&path, &bytes[..bytes.len().min(max_bytes)]).unwrap();
        path
    }

    /// C++ `removeFile`.
    pub fn remove_file(&self, name: &str) {
        let _ = std::fs::remove_file(self.path(name));
    }

    /// C++ `writeAndRun`: write `test.bin`, send `RUN` (blocking), and
    /// dispatch the command and the `cmd_RUN` signal.
    pub fn write_and_run(&mut self) {
        let path = self.write_to_file("test.bin");
        self.send_run(&path, BlockState::Block);
        self.dispatch();
        self.dispatch();
    }

    /// C++ `addDirective`.
    pub fn add_directive(&mut self, id: DirectiveId, args: &[u8]) {
        let count = self.seq.header.statement_count;
        let stmt = &mut self.seq.statements[usize::from(count)];
        stmt.op_code = id;
        stmt.arg_buf = StatementArgBuffer::new();
        assert!(stmt.arg_buf.set_buff(args).is_ok());
        self.seq.header.statement_count = count + 1;
    }

    /// Add a directive whose arguments are an FPP struct.
    pub fn add_fpp(&mut self, id: DirectiveId, args: &dyn Serialize) {
        let mut buf = StatementArgBuffer::new();
        assert!(buf.serialize(args, Endianness::Big).is_ok());
        self.add_directive(id, buf.as_slice());
    }

    /// C++ `addArgumentSpec`.
    pub fn add_argument_spec(&mut self, arg_name: &str, type_name: &str, arg_size: StackSizeType) {
        let count = self.seq.header.argument_count;
        let spec = &mut self.seq.args[usize::from(count)];
        spec.arg_name.set(arg_name);
        spec.type_name.set(type_name);
        spec.arg_size = arg_size;
        self.seq.header.argument_count = count + 1;
    }

    pub fn add_wait_rel(&mut self) {
        self.add_directive(DirectiveId::WaitRel, &[]);
    }
    pub fn add_wait_abs(&mut self) {
        self.add_directive(DirectiveId::WaitAbs, &[]);
    }
    pub fn add_goto(&mut self, stmt_idx: u32) {
        self.add_fpp(DirectiveId::Goto, &GotoDirective::new(stmt_idx));
    }
    pub fn add_if(&mut self, goto_if_false: u32) {
        self.add_fpp(DirectiveId::If, &IfDirective::new(goto_if_false));
    }
    pub fn add_no_op(&mut self) {
        self.add_directive(DirectiveId::NoOp, &[]);
    }
    pub fn add_push_tlm_val(&mut self, id: FwChanIdType) {
        self.add_fpp(DirectiveId::PushTlmVal, &PushTlmValDirective::new(id));
    }
    pub fn add_push_tlm_val_and_time(&mut self, id: FwChanIdType) {
        self.add_fpp(
            DirectiveId::PushTlmValAndTime,
            &PushTlmValAndTimeDirective::new(id),
        );
    }
    pub fn add_push_prm(&mut self, id: FwPrmIdType) {
        self.add_fpp(DirectiveId::PushPrm, &PushPrmDirective::new(id));
    }
    /// C++ `add_CONST_CMD(opcode)`: no arguments.
    pub fn add_const_cmd(&mut self, opcode: FwOpcodeType) {
        self.add_directive(DirectiveId::ConstCmd, &opcode.to_be_bytes());
    }
    pub fn add_stack_op(&mut self, op: DirectiveId) {
        self.add_directive(op, &[]);
    }
    pub fn add_exit(&mut self) {
        self.add_directive(DirectiveId::Exit, &[]);
    }
    pub fn add_allocate(&mut self, size: StackSizeType) {
        self.add_fpp(DirectiveId::Allocate, &AllocateDirective::new(size));
    }
    pub fn add_store_rel_const_offset(&mut self, offset: SignedStackSizeType, size: StackSizeType) {
        self.add_fpp(
            DirectiveId::StoreRelConstOffset,
            &StoreRelConstOffsetDirective::new(offset, size),
        );
    }
    pub fn add_load_rel(&mut self, offset: SignedStackSizeType, size: StackSizeType) {
        self.add_fpp(DirectiveId::LoadRel, &LoadRelDirective::new(offset, size));
    }
    /// C++ `add_PUSH_VAL<T>(val)`: the big-endian bytes of `val`.
    pub fn add_push_val<T: StackValue>(&mut self, value: T) {
        let mut bytes = [0u8; 8];
        value.to_be(&mut bytes[..T::SIZE]);
        self.add_directive(DirectiveId::PushVal, &bytes[..T::SIZE]);
    }
    pub fn add_discard(&mut self, size: StackSizeType) {
        self.add_fpp(DirectiveId::Discard, &DiscardDirective::new(size));
    }
    pub fn add_stack_cmd(&mut self, size: StackSizeType) {
        self.add_fpp(DirectiveId::StackCmd, &StackCmdDirective::new(size));
    }
    pub fn add_memcmp(&mut self, size: StackSizeType) {
        self.add_fpp(DirectiveId::Memcmp, &MemCmpDirective::new(size));
    }
    pub fn add_push_time(&mut self) {
        self.add_directive(DirectiveId::PushTime, &[]);
    }
    pub fn add_set_seed(&mut self) {
        self.add_directive(DirectiveId::SetSeed, &[]);
    }
    pub fn add_push_rand(&mut self) {
        self.add_directive(DirectiveId::PushRand, &[]);
    }
    pub fn add_get_field(&mut self, parent_size: StackSizeType, member_size: StackSizeType) {
        self.add_fpp(
            DirectiveId::GetField,
            &GetFieldDirective::new(parent_size, member_size),
        );
    }
    pub fn add_peek(&mut self) {
        self.add_directive(DirectiveId::Peek, &[]);
    }
    pub fn add_store_rel(&mut self, size: StackSizeType) {
        self.add_fpp(DirectiveId::StoreRel, &StoreRelDirective::new(size));
    }
    pub fn add_call(&mut self) {
        self.add_directive(DirectiveId::Call, &[]);
    }
    pub fn add_return(&mut self, return_val_size: StackSizeType, call_args_size: StackSizeType) {
        self.add_fpp(
            DirectiveId::Return,
            &ReturnDirective::new(return_val_size, call_args_size),
        );
    }
    pub fn add_load_abs(&mut self, offset: StackSizeType, size: StackSizeType) {
        self.add_fpp(DirectiveId::LoadAbs, &LoadAbsDirective::new(offset, size));
    }
    pub fn add_store_abs(&mut self, size: StackSizeType) {
        self.add_fpp(DirectiveId::StoreAbs, &StoreAbsDirective::new(size));
    }
    pub fn add_store_abs_const_offset(&mut self, offset: StackSizeType, size: StackSizeType) {
        self.add_fpp(
            DirectiveId::StoreAbsConstOffset,
            &StoreAbsConstOffsetDirective::new(offset, size),
        );
    }
    pub fn add_pop_event(&mut self) {
        self.add_directive(DirectiveId::PopEvent, &[]);
    }
    pub fn add_pop_serializable(&mut self, port_index: FwIndexType, size: StackSizeType) {
        self.add_fpp(
            DirectiveId::PopSerializable,
            &PopSerializableDirective::new(port_index, size),
        );
    }

    // -- commands and ports --------------------------------------------------

    /// Invoke `cmdIn` with `opcode` (FPP-relative), command sequence 0 and
    /// the argument bytes.
    pub fn send_cmd(&self, opcode: FwOpcodeType, args: &[u8]) {
        let mut buf = CmdArgBuffer::new();
        assert!(
            buf.serialize_bytes(args, LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        let p = self.comp.cmd_in(0);
        p.target.invoke(p.port_num, ID_BASE + opcode, 0, &mut buf);
    }

    fn string_arg(value: &str) -> Vec<u8> {
        let mut out = (value.len() as u16).to_be_bytes().to_vec();
        out.extend_from_slice(value.as_bytes());
        out
    }

    fn seq_args_bytes(args: &SeqArgs) -> Vec<u8> {
        let mut buf = LinearBuffer::<64>::new();
        assert!(buf.serialize(args, Endianness::Big).is_ok());
        buf.as_slice().to_vec()
    }

    /// C++ `sendCmd_RUN`.
    pub fn send_run(&self, file: &str, block: BlockState) {
        let mut args = Self::string_arg(file);
        args.push(block.as_repr());
        self.send_cmd(FpySequencer::OPCODE_RUN, &args);
    }

    /// C++ `sendCmd_RUN_ARGS`.
    pub fn send_run_args(&self, file: &str, block: BlockState, seq_args: &SeqArgs) {
        let mut args = Self::string_arg(file);
        args.push(block.as_repr());
        args.extend(Self::seq_args_bytes(seq_args));
        self.send_cmd(FpySequencer::OPCODE_RUN_ARGS, &args);
    }

    /// C++ `sendCmd_VALIDATE`.
    pub fn send_validate(&self, file: &str) {
        self.send_cmd(FpySequencer::OPCODE_VALIDATE, &Self::string_arg(file));
    }

    /// C++ `sendCmd_VALIDATE_ARGS`.
    pub fn send_validate_args(&self, file: &str, seq_args: &SeqArgs) {
        let mut args = Self::string_arg(file);
        args.extend(Self::seq_args_bytes(seq_args));
        self.send_cmd(FpySequencer::OPCODE_VALIDATE_ARGS, &args);
    }

    /// C++ `sendCmd_RUN_VALIDATED`.
    pub fn send_run_validated(&self, block: BlockState) {
        self.send_cmd(FpySequencer::OPCODE_RUN_VALIDATED, &[block.as_repr()]);
    }

    /// C++ `sendCmd_SET_BREAKPOINT`.
    pub fn send_set_breakpoint(&self, stmt_idx: u32, break_once: bool) {
        let mut args = stmt_idx.to_be_bytes().to_vec();
        args.push(if break_once { 0xFF } else { 0x00 });
        self.send_cmd(FpySequencer::OPCODE_SET_BREAKPOINT, &args);
    }

    /// C++ `sendCmd_DUMP_STACK_TO_FILE`.
    pub fn send_dump_stack_to_file(&self, file: &str) {
        self.send_cmd(
            FpySequencer::OPCODE_DUMP_STACK_TO_FILE,
            &Self::string_arg(file),
        );
    }

    /// C++ `paramSet_STATEMENT_TIMEOUT_SECS` + `paramSend_...`: the
    /// `PRM_SET` command.
    pub fn set_timeout_param(&self, value: f32) {
        self.send_cmd(
            FpySequencer::OPCODE_STATEMENT_TIMEOUT_SECS_PRM_SET,
            &value.to_be_bytes(),
        );
    }

    /// C++ `paramSet_SEQ_BASE_DIR` + `paramSend_...`.
    pub fn set_base_dir_param(&self, value: &str) {
        self.send_cmd(
            FpySequencer::OPCODE_SEQ_BASE_DIR_PRM_SET,
            &Self::string_arg(value),
        );
    }

    /// C++ `invoke_to_cmdResponseIn`.
    pub fn cmd_response_in(&self, opcode: FwOpcodeType, cmd_seq: u32, response: CmdResponse) {
        let p = self.comp.cmd_response_in(0);
        p.target.invoke(p.port_num, opcode, cmd_seq, response);
    }

    /// C++ `invoke_to_checkTimers`.
    pub fn check_timers(&self) {
        let p = self.comp.check_timers(0);
        p.target.invoke(p.port_num, 0);
    }

    /// C++ `invoke_to_seqRunIn`.
    pub fn seq_run_in(&self, file: &str, args: &SeqArgs) {
        let p = self.comp.seq_run_in(0);
        p.target
            .invoke(p.port_num, &FileNameString::from(file), args);
    }

    /// C++ `invoke_to_seqCancelIn`.
    pub fn seq_cancel_in(&self) {
        let p = self.comp.seq_cancel_in(0);
        p.target.invoke(p.port_num);
    }
}

/// `SeqArgs` holding the big-endian bytes of `values`.
pub(super) fn seq_args_u32(values: &[u32]) -> SeqArgs {
    let mut args = SeqArgs::default();
    let mut n = 0;
    for v in values {
        args.buffer.0[n..n + 4].copy_from_slice(&v.to_be_bytes());
        n += 4;
    }
    assert!(n <= SEQUENCE_ARGUMENTS_MAX_SIZE);
    args.size = n as FwSizeType;
    args
}

/// A deserialized `Fw::Time` from 11 stack bytes.
pub(super) fn time_from(bytes: &[u8]) -> Time {
    let mut copy = bytes.to_vec();
    let mut buf = fprime_fw::ExtBuf::with_len(&mut copy, Time::SERIALIZED_SIZE);
    let mut time = Time::default();
    assert!(buf.deserialize(&mut time, Endianness::Big).is_ok());
    time
}

/// The time `(base, context, seconds, useconds)`.
pub(super) fn time(base: TimeBase, context: u8, seconds: u32, useconds: u32) -> Time {
    Time::new(base, context, seconds, useconds)
}

/// The string argument at the start of event argument bytes.
pub(super) fn first_string(args: &[u8]) -> String {
    let len = usize::from(u16::from_be_bytes([args[0], args[1]]));
    String::from_utf8_lossy(&args[2..2 + len]).into_owned()
}

#[test]
fn harness_writes_upstreams_file_layout() {
    // An empty sequence: header (body size 0) and the CRC of the header.
    let mut t = Tester::new();
    let bytes = t.sequence_bytes();
    assert_eq!(bytes.len(), 11 + 4);
    assert_eq!(
        &bytes[..11],
        &[0, 0, 0, SCHEMA_VERSION, 0, 0, 0, 0, 0, 0, 0]
    );
    let mut hash = Hash::new();
    hash.update(&bytes[..11]);
    assert_eq!(&bytes[11..], &hash.finalize().to_be_bytes());
}
