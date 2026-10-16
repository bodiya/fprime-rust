//! Ports of `Svc/WasmSequencer/test/ut` (`WasmSequencerTestMain.cpp` and
//! the `WasmSequencerTester` harness), driven by upstream's golden modules
//! (`test/wasm/*.wasm`, copied byte for byte into `tests/wasm/`, with the
//! `.wat` sources and `build_wasm.py` that regenerate them).
//!
//! The harness mirrors the C++ tester: the component is driven by hand
//! (`dispatch_one` / `dispatch_all`, one queue message at a time), its
//! state is reached directly (the tests are a child module), and every
//! output port is recorded. Differences:
//!
//! - upstream stages a module in the working directory under its basename
//!   and passes the basename. Here each test stages its modules in its own
//!   temporary directory and points `SEQ_BASE_DIR` at it (upstream's
//!   default is empty), so tests run in parallel processes without
//!   clashing and the basename still fits a 40-byte command string; tests
//!   of `SEQ_BASE_DIR` itself set their own value.
//! - spacewasm's heap is process-wide with `MAX_SEQUENCERS` slots, so the
//!   tests take a global lock and run one at a time.
//! - ports cannot be disconnected after wiring, so a test that needs an
//!   unconnected or failing port builds its tester with [`Options`].
//! - death tests are `#[should_panic]` tests.

mod cancel;
mod commands;
mod config;
mod globals;
mod host_functions;
mod load;
mod memory;
mod ports;
mod run;
mod serial;
mod sleep;
mod telemetry;

use super::*;
use crate::tlm_chan::TlmGetPort;
use fprime_comp::{CmdRegPort, LogPort, PrmSetPort, TimePort, TlmPort};
use fprime_fw::{ComBuffer, LengthMode, LogBuffer, LogSeverity, TimeBase, TlmBuffer, TlmValid};
use fprime_os::queue::BlockingType;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};

include!("assets.rs");

/// The id base of the component under test.
pub(super) const ID_BASE: FwIdType = 0x100;
/// C++ `TEST_INSTANCE_QUEUE_DEPTH`.
const QUEUE_DEPTH: FwSizeType = 20;
/// C++ `DYNAMIC_POOL_PAGES`.
pub(super) const DYNAMIC_POOL_PAGES: FwSizeType = 4;
/// C++ `GUEST_POOL_SIZE`.
pub(super) const GUEST_POOL_SIZE: FwSizeType = 2048;
/// C++ `WASM_STACK_SIZE`.
pub(super) const WASM_STACK_SIZE: FwSizeType = 256;
/// C++ `SERIAL_OUT_MAX_SIZE`.
pub(super) const SERIAL_OUT_MAX_SIZE: FwSizeType = 256;
/// C++ `SERIAL_IN_QUEUE_SIZE`.
pub(super) const SERIAL_IN_QUEUE_SIZE: FwSizeType = 256;

/// Serializes the tests (spacewasm's heap registry is process-wide).
static TEST_LOCK: Mutex<()> = Mutex::new(());
static COUNTER: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Ground stubs.
// ---------------------------------------------------------------------------

/// A recorded event: (FPP-relative id, severity, argument bytes).
pub(super) type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

/// Records every output of the component and answers its queries.
#[derive(Default)]
pub(super) struct Ground {
    pub responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    pub events: Mutex<Vec<EventRecord>>,
    pub tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    pub cmd_out: Mutex<Vec<(Vec<u8>, u32)>>,
    pub tlm_gets: Mutex<Vec<FwChanIdType>>,
    pub prm_gets: Mutex<Vec<FwPrmIdType>>,
    pub prm_sets: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
    pub seq_start: Mutex<Vec<(Vec<u8>, SeqArgs)>>,
    pub seq_done: Mutex<Vec<CmdResponse>>,
    pub serial_out: Mutex<Vec<(FwIndexType, Vec<u8>)>>,
    /// The test clock (C++ `setTestTime`).
    pub time: Mutex<Time>,
    /// C++ `nextTlmId` / `nextTlmTime` / `nextTlmValue`.
    pub next_tlm: Mutex<(FwChanIdType, Time, TlmBuffer)>,
    /// C++ `nextPrmId` / `nextPrmValue`.
    pub next_prm: Mutex<(FwPrmIdType, ParamBuffer)>,
    /// The tester's parameter store (C++ `paramSet_*`), served on the
    /// component's `prmGet` port (FPP-relative ids).
    pub params: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
}

impl CmdRegPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, _op_code: FwOpcodeType) {}
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

/// C++ `from_getTlmChan_handler`: answer `next_tlm` for its id, else
/// invalid.
impl TlmGetPort for Ground {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) -> TlmValid {
        self.tlm_gets.lock().unwrap().push(id);
        let next = self.next_tlm.lock().unwrap();
        if id != next.0 {
            let _ = val.set_buff_len(0);
            return TlmValid::Invalid;
        }
        *val = next.2.clone();
        *time_tag = next.1;
        TlmValid::Valid
    }
}

/// C++ `from_getParam_handler`.
impl PrmGetPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid {
        self.prm_gets.lock().unwrap().push(id);
        let next = self.next_prm.lock().unwrap();
        if id != next.0 {
            return ParamValid::Invalid;
        }
        *val = next.1.clone();
        ParamValid::Valid
    }
}

/// `seqDoneOut` (C++ `from_seqDoneOut_handler`).
pub(super) struct SeqDone(Arc<Ground>);

impl CmdResponsePort for SeqDone {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        _op_code: FwOpcodeType,
        _cmd_seq: u32,
        response: CmdResponse,
    ) {
        self.0.seq_done.lock().unwrap().push(response);
    }
}

/// The component's own parameter port (`prmGet`, the generated tester's
/// `from_prmGet`): serves the values set with [`Tester::param_set`],
/// `INVALID` otherwise (so `load_parameters` installs the defaults).
struct PrmDbStub(Arc<Ground>);

impl PrmGetPort for PrmDbStub {
    fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid {
        let params = self.0.params.lock().unwrap();
        match params.iter().find(|(p, _)| *p == id - ID_BASE) {
            Some((_, bytes)) => {
                *val = ParamBuffer::new();
                assert!(
                    val.serialize_bytes(bytes, LengthMode::OmitLength, Endianness::Big)
                        .is_ok()
                );
                ParamValid::Valid
            }
            None => ParamValid::Invalid,
        }
    }
}

/// `serialOut` (C++ `from_serialOut_handler`).
struct SerialStub(Arc<Ground>);

impl SerializePort for SerialStub {
    fn invoke_serial(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) -> SerializeStatus {
        let data = buffer.as_slice().to_vec();
        self.0.serial_out.lock().unwrap().push((port_num, data));
        SerializeStatus::Ok
    }
}

/// C++ `FailingSerialInputPort`: always reports a serialization failure.
struct FailingSerial;

impl SerializePort for FailingSerial {
    fn invoke_serial(
        &self,
        _port_num: FwIndexType,
        _buffer: &mut dyn SerBufAny,
    ) -> SerializeStatus {
        SerializeStatus::NoRoomLeft
    }
}

// ---------------------------------------------------------------------------
// The tester.
// ---------------------------------------------------------------------------

/// How a `serialOut` port is wired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SerialOut {
    /// The recording stub.
    Stub,
    /// Not connected (C++ `disconnectSerialOut`).
    Unconnected,
    /// The failing stub (C++ `connectSerialOutTo(FailingSerialInputPort)`).
    Failing,
}

/// The wiring and configuration of a tester.
#[derive(Debug, Clone, Copy)]
pub(super) struct Options {
    /// Configure with [`Tester::standard_config`] (C++ `autoConfigure`).
    pub configure: bool,
    /// Connect `getTlmChan`.
    pub get_tlm_chan: bool,
    /// Connect `getParam`.
    pub get_param: bool,
    /// Connect `cmdOut`.
    pub cmd_out: bool,
    /// The `serialOut` ports.
    pub serial_out: [SerialOut; NUM_SERIAL_OUT_PORTS],
    /// Connect `seqStartOut` / `seqDoneOut` to the recorder.
    pub seq_out: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            configure: true,
            get_tlm_chan: true,
            get_param: true,
            cmd_out: true,
            serial_out: [SerialOut::Stub; NUM_SERIAL_OUT_PORTS],
            seq_out: true,
        }
    }
}

/// C++ `WasmSequencerTester` (and `WasmSequencerConfigTester` with
/// `configure: false`).
pub(super) struct Tester {
    pub comp: Arc<WasmSequencer>,
    pub g: Arc<Ground>,
    dir: PathBuf,
    leaked: bool,
    _lock: MutexGuard<'static, ()>,
}

impl Drop for Tester {
    fn drop(&mut self) {
        if self.leaked {
            // Keep the component (and its store) alive forever.
            std::mem::forget(self.comp.clone());
        } else {
            self.comp.deinit();
        }
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// `BLOCK`.
pub(super) const BLOCK: BlockState = BlockState::Block;
/// `NO_BLOCK`.
pub(super) const NO_BLOCK: BlockState = BlockState::NoBlock;

impl Tester {
    /// C++ `WasmSequencerTester()`.
    pub fn new() -> Self {
        Self::with(Options::default())
    }

    /// C++ `WasmSequencerConfigTester()`: wired and initialized, not
    /// configured.
    pub fn unconfigured() -> Self {
        Self::with(Options {
            configure: false,
            ..Options::default()
        })
    }

    /// A tester with custom wiring.
    pub fn with(options: Options) -> Self {
        let lock = TEST_LOCK.lock().unwrap_or_else(PoisonError::into_inner);
        let g = Arc::new(Ground::default());
        let comp = WasmSequencer::new("WasmSequencer");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(g.clone(), 0);
        comp.cmd.cmd_response_out.connect(g.clone(), 0);
        comp.evt.log_out.connect(g.clone(), 0);
        comp.evt.time_out.connect(g.clone(), 0);
        comp.tlm.tlm_out.connect(g.clone(), 0);
        comp.prm
            .prm_get_out
            .connect(Arc::new(PrmDbStub(g.clone())), 0);
        comp.prm.prm_set_out.connect(g.clone(), 0);
        if options.cmd_out {
            comp.cmd_out.connect(g.clone(), 0);
        }
        if options.get_tlm_chan {
            comp.get_tlm_chan.connect(g.clone(), 0);
        }
        if options.get_param {
            comp.get_param.connect(g.clone(), 0);
        }
        if options.seq_out {
            comp.seq_start_out.connect(g.clone(), 0);
            comp.seq_done_out.connect(Arc::new(SeqDone(g.clone())), 0);
        }
        for (i, mode) in options.serial_out.iter().enumerate() {
            match mode {
                SerialOut::Stub => {
                    comp.serial_out[i].connect(Arc::new(SerialStub(g.clone())), i as FwIndexType)
                }
                SerialOut::Failing => {
                    comp.serial_out[i].connect(Arc::new(FailingSerial), i as FwIndexType)
                }
                SerialOut::Unconnected => {}
            }
        }
        comp.init(QUEUE_DEPTH);
        let mut dir = std::env::temp_dir();
        dir.push(format!(
            "wsq{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let t = Self {
            comp,
            g,
            dir,
            leaked: false,
            _lock: lock,
        };
        if options.configure {
            t.comp.configure(&Self::standard_config());
        }
        // Stage modules in this test's directory, then load the parameters
        // so INSTRUCTION_FUEL (and friends) take their defaults.
        let base = t.base_dir();
        t.param_set(W::PARAMID_SEQ_BASE_DIR, &Bytes::new().str(&base).0);
        t.comp.load_parameters();
        t.clear_history();
        t
    }

    /// Never tear the component down: for the death tests that leave the
    /// heap bookkeeping inconsistent on purpose (C++ runs them in a forked
    /// child). One registry slot stays taken for the rest of the process.
    pub fn leak(mut self) {
        self.leaked = true;
    }

    /// C++ `standardConfig()`.
    pub fn standard_config() -> Config {
        let mut cfg = Config {
            heap_pages: DYNAMIC_POOL_PAGES,
            guest_memory_size: GUEST_POOL_SIZE,
            stack_size: WASM_STACK_SIZE,
            serial_out_max: SERIAL_OUT_MAX_SIZE,
            ..Config::default()
        };
        for port in &mut cfg.serial_in {
            port.size = SERIAL_IN_QUEUE_SIZE;
            port.full_behavior = SerialInQueueFullBehavior::DropOldest;
        }
        cfg
    }

    /// C++ `configureWith(cfg)`.
    pub fn configure_with(&self, cfg: &Config) {
        self.comp.configure(cfg);
    }

    /// This test's staging directory with a trailing `/` (it must fit the
    /// 40-byte `SEQ_BASE_DIR`).
    pub fn base_dir(&self) -> String {
        let base = format!("{}/", self.dir.to_str().unwrap());
        assert!(
            base.len() <= fprime_config::FW_PARAM_STRING_MAX_SIZE,
            "test directory {base} does not fit SEQ_BASE_DIR"
        );
        base
    }

    /// C++ `StagedAsset` / `copyAsset`: write golden module `name` into the
    /// staging directory; returns the name to pass in commands.
    pub fn stage(&self, name: &str) -> String {
        let bytes = ASSETS
            .iter()
            .find(|(n, _)| *n == name)
            .unwrap_or_else(|| panic!("no asset {name}"))
            .1;
        std::fs::write(self.dir.join(name), bytes).unwrap();
        name.to_string()
    }

    /// C++ `paramSet_<P>(value, VALID)`: store a parameter's serialized
    /// value in the tester's database. The component sees it on its next
    /// `load_parameters`.
    pub fn param_set(&self, id: FwPrmIdType, value: &[u8]) {
        let mut params = self.g.params.lock().unwrap();
        params.retain(|(p, _)| *p != id);
        params.push((id, value.to_vec()));
    }

    /// C++ `paramSet_INSTRUCTION_FUEL(value, VALID)` (without a reload, as
    /// upstream's tests call it: the running fuel is unchanged).
    pub fn param_set_fuel(&self, value: FwSizeType) {
        self.param_set(W::PARAMID_INSTRUCTION_FUEL, &value.to_be_bytes());
    }

    /// C++ `paramSet_SEQ_BASE_DIR` + `loadParameters`.
    pub fn set_base_dir(&self, base: &str) {
        self.param_set(W::PARAMID_SEQ_BASE_DIR, &Bytes::new().str(base).0);
        self.comp.load_parameters();
    }

    /// C++ `setTestTime`.
    pub fn set_time(&self, time: Time) {
        *self.g.time.lock().unwrap() = time;
    }

    // -- dispatch -----------------------------------------------------------

    /// C++ `messagesAvailable`.
    pub fn messages_available(&self) -> FwSizeType {
        self.comp.active.queued.queue().get_messages_available()
    }

    /// C++ `dispatchOne`.
    pub fn dispatch_one(&self) {
        assert!(
            self.messages_available() > 0,
            "no queued message to dispatch"
        );
        let status = self
            .comp
            .active
            .queued
            .do_dispatch(&*self.comp, BlockingType::NonBlocking);
        assert_eq!(status, MsgDispatchStatus::Ok);
    }

    /// C++ `dispatchAll(bound = 1000)`.
    pub fn dispatch_all(&self) {
        self.dispatch_all_bounded(1000);
    }

    /// C++ `dispatchAll(bound)`.
    pub fn dispatch_all_bounded(&self, bound: u32) {
        let mut iters = 0;
        while self.messages_available() > 0 && iters < bound {
            self.dispatch_one();
            iters += 1;
        }
        assert!(iters < bound, "dispatch_all exceeded bound (livelock?)");
    }

    /// C++ `dispatchUntilControllerState`.
    pub fn dispatch_until_controller(&self, state: ControllerState) {
        let mut iters = 0;
        while self.controller() != state && iters < 1000 {
            if self.messages_available() == 0 {
                break;
            }
            self.dispatch_one();
            iters += 1;
        }
        assert_eq!(
            self.controller(),
            state,
            "controller did not reach the state"
        );
    }

    /// C++ `dispatchUntilInterpreterState`.
    pub fn dispatch_until_interpreter(&self, state: InterpreterState) {
        let mut iters = 0;
        while self.interpreter() != state && iters < 1000 {
            if self.messages_available() == 0 {
                break;
            }
            self.dispatch_one();
            iters += 1;
        }
        assert_eq!(
            self.interpreter(),
            state,
            "interpreter did not reach the state"
        );
    }

    /// C++ `controllerState`.
    pub fn controller(&self) -> ControllerState {
        self.comp.controller_get_state()
    }

    /// C++ `interpreterState`.
    pub fn interpreter(&self) -> InterpreterState {
        self.comp.interpreter_get_state()
    }

    // -- commands and ports ---------------------------------------------------

    /// Invoke `cmdIn` with `opcode` (FPP-relative), `cmd_seq` and the
    /// argument bytes.
    pub fn send_cmd(&self, opcode: FwOpcodeType, cmd_seq: u32, args: &[u8]) {
        let mut buf = CmdArgBuffer::new();
        assert!(
            buf.serialize_bytes(args, LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        let p = self.comp.cmd_in(0);
        p.target
            .invoke(p.port_num, ID_BASE + opcode, cmd_seq, &mut buf);
    }

    /// C++ `sendCmd_RUN`.
    pub fn send_run(&self, cmd_seq: u32, file: &str, block: BlockState, args: &SeqArgs) {
        let a = Bytes::new().str(file).u8(block.as_repr()).seq_args(args);
        self.send_cmd(WasmSequencer::OPCODE_RUN, cmd_seq, &a.0);
    }

    /// C++ `sendCmd_LOAD`.
    pub fn send_load(&self, cmd_seq: u32, file: &str, name: &str) {
        let a = Bytes::new().str(file).str(name);
        self.send_cmd(WasmSequencer::OPCODE_LOAD, cmd_seq, &a.0);
    }

    /// C++ `sendCmd_INVOKE`.
    pub fn send_invoke(&self, cmd_seq: u32, module: &str, block: BlockState, args: &SeqArgs) {
        let a = Bytes::new().str(module).u8(block.as_repr()).seq_args(args);
        self.send_cmd(WasmSequencer::OPCODE_INVOKE, cmd_seq, &a.0);
    }

    /// C++ `sendCmd_WAIT`.
    pub fn send_wait(&self, cmd_seq: u32) {
        self.send_cmd(WasmSequencer::OPCODE_WAIT, cmd_seq, &[]);
    }

    /// C++ `sendCmd_CANCEL`.
    pub fn send_cancel(&self, cmd_seq: u32) {
        self.send_cmd(WasmSequencer::OPCODE_CANCEL, cmd_seq, &[]);
    }

    /// C++ `sendCmd_PAUSE`.
    pub fn send_pause(&self, cmd_seq: u32) {
        self.send_cmd(WasmSequencer::OPCODE_PAUSE, cmd_seq, &[]);
    }

    /// C++ `sendCmd_CONTINUE`.
    pub fn send_continue(&self, cmd_seq: u32) {
        self.send_cmd(WasmSequencer::OPCODE_CONTINUE, cmd_seq, &[]);
    }

    /// C++ `sendCmd_GLOBAL_SET_I32`.
    pub fn send_global_set_i32(&self, cmd_seq: u32, module: &str, name: &str, value: i32) {
        let a = Bytes::new().str(module).str(name).i32(value);
        self.send_cmd(WasmSequencer::OPCODE_GLOBAL_SET_I32, cmd_seq, &a.0);
    }

    /// C++ `sendCmd_GLOBAL_SET_I64`.
    pub fn send_global_set_i64(&self, cmd_seq: u32, module: &str, name: &str, value: i64) {
        let a = Bytes::new().str(module).str(name).i64(value);
        self.send_cmd(WasmSequencer::OPCODE_GLOBAL_SET_I64, cmd_seq, &a.0);
    }

    /// C++ `sendCmd_GLOBAL_SET_F32`.
    pub fn send_global_set_f32(&self, cmd_seq: u32, module: &str, name: &str, value: f32) {
        let a = Bytes::new().str(module).str(name).f32(value);
        self.send_cmd(WasmSequencer::OPCODE_GLOBAL_SET_F32, cmd_seq, &a.0);
    }

    /// C++ `sendCmd_GLOBAL_SET_F64`.
    pub fn send_global_set_f64(&self, cmd_seq: u32, module: &str, name: &str, value: f64) {
        let a = Bytes::new().str(module).str(name).f64(value);
        self.send_cmd(WasmSequencer::OPCODE_GLOBAL_SET_F64, cmd_seq, &a.0);
    }

    /// C++ `sendCmd_GLOBAL_GET`.
    pub fn send_global_get(&self, cmd_seq: u32, module: &str, name: &str) {
        let a = Bytes::new().str(module).str(name);
        self.send_cmd(WasmSequencer::OPCODE_GLOBAL_GET, cmd_seq, &a.0);
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

    /// C++ `invoke_to_serialIn` (no dispatch).
    pub fn serial_in(&self, port: FwIndexType, bytes: &[u8]) {
        let mut buf = LinearBuffer::<1024>::new();
        assert!(
            buf.serialize_bytes(bytes, LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        let p = self.comp.serial_in(port);
        let status = p.target.invoke_serial(p.port_num, &mut buf);
        assert_eq!(status, SerializeStatus::Ok);
    }

    /// C++ `enqueueSerialIn`: `serialIn` then drain the queue.
    pub fn enqueue_serial_in(&self, port: FwIndexType, bytes: &[u8]) {
        self.serial_in(port, bytes);
        self.dispatch_all();
    }

    /// C++ `flushTelemetry`.
    pub fn flush_telemetry(&self) {
        self.g.tlm.lock().unwrap().clear();
        let p = self.comp.write_telemetry(0);
        p.target.invoke(p.port_num, 0);
        self.dispatch_all();
    }

    /// C++ `lastCmdContext`.
    pub fn last_cmd_context(&self) -> u32 {
        let cmds = self.g.cmd_out.lock().unwrap();
        assert!(!cmds.is_empty(), "no cmdOut dispatch recorded");
        cmds.last().unwrap().1
    }

    /// C++ `currentCmdUid`.
    pub fn current_cmd_uid(&self) -> u32 {
        self.comp.state().make_cmd_uid()
    }

    // -- histories ----------------------------------------------------------

    /// C++ `clearHistory`.
    pub fn clear_history(&self) {
        self.g.responses.lock().unwrap().clear();
        self.g.events.lock().unwrap().clear();
        self.g.tlm.lock().unwrap().clear();
        self.g.cmd_out.lock().unwrap().clear();
        self.g.tlm_gets.lock().unwrap().clear();
        self.g.prm_gets.lock().unwrap().clear();
        self.g.seq_start.lock().unwrap().clear();
        self.g.seq_done.lock().unwrap().clear();
        self.g.serial_out.lock().unwrap().clear();
    }

    /// The command responses (opcode, cmdSeq, response).
    pub fn responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
        self.g.responses.lock().unwrap().clone()
    }

    /// C++ `ASSERT_CMD_RESPONSE_SIZE`.
    pub fn assert_responses(&self, size: usize) {
        assert_eq!(self.responses().len(), size, "{:?}", self.responses());
    }

    /// C++ `ASSERT_CMD_RESPONSE(index, opcode, cmdSeq, response)`.
    pub fn assert_response(
        &self,
        index: usize,
        opcode: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        let responses = self.responses();
        assert!(
            index < responses.len(),
            "only {} responses: {responses:?}",
            responses.len()
        );
        assert_eq!(
            responses[index],
            (opcode, cmd_seq, response),
            "{responses:?}"
        );
    }

    /// The argument bytes of every event `id`.
    pub fn events(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
        self.g
            .events
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.0 == id)
            .map(|e| e.2.clone())
            .collect()
    }

    /// C++ `ASSERT_EVENTS_<E>_SIZE`.
    pub fn assert_events(&self, id: FwEventIdType, size: usize) {
        assert_eq!(
            self.events(id).len(),
            size,
            "event {id}: {:?}",
            self.all_events()
        );
    }

    /// C++ `ASSERT_EVENTS_<E>(index, args...)`: the arguments as bytes.
    pub fn assert_event(&self, id: FwEventIdType, index: usize, args: Bytes) {
        let events = self.events(id);
        assert!(
            index < events.len(),
            "only {} events {id}: {:?}",
            events.len(),
            self.all_events()
        );
        assert_eq!(events[index], args.0, "event {id} #{index}");
    }

    /// Every event (id, severity, args).
    pub fn all_events(&self) -> Vec<EventRecord> {
        self.g.events.lock().unwrap().clone()
    }

    /// The values written to channel `id`.
    pub fn tlm(&self, id: FwChanIdType) -> Vec<Vec<u8>> {
        self.g
            .tlm
            .lock()
            .unwrap()
            .iter()
            .filter(|t| t.0 == id)
            .map(|t| t.1.clone())
            .collect()
    }

    /// C++ `ASSERT_TLM_<C>(index, value)`.
    pub fn assert_tlm(&self, id: FwChanIdType, index: usize, value: Bytes) {
        let values = self.tlm(id);
        assert!(
            index < values.len(),
            "only {} values of channel {id}",
            values.len()
        );
        assert_eq!(values[index], value.0, "channel {id} #{index}");
    }

    /// C++ `ASSERT_TLM_<C>_SIZE`.
    pub fn assert_tlm_size(&self, id: FwChanIdType, size: usize) {
        assert_eq!(self.tlm(id).len(), size, "channel {id}");
    }

    /// The `cmdOut` calls.
    pub fn cmd_out(&self) -> Vec<(Vec<u8>, u32)> {
        self.g.cmd_out.lock().unwrap().clone()
    }

    /// C++ `ASSERT_FROM_PORT_HISTORY_SIZE`: the typed output ports the
    /// tester records (`cmdOut`, `getTlmChan`, `getParam`).
    pub fn port_history_size(&self) -> usize {
        self.g.cmd_out.lock().unwrap().len()
            + self.g.tlm_gets.lock().unwrap().len()
            + self.g.prm_gets.lock().unwrap().len()
    }

    /// `seqStartOut` calls.
    pub fn seq_start(&self) -> Vec<(Vec<u8>, SeqArgs)> {
        self.g.seq_start.lock().unwrap().clone()
    }

    /// `seqDoneOut` calls.
    pub fn seq_done(&self) -> Vec<CmdResponse> {
        self.g.seq_done.lock().unwrap().clone()
    }

    /// `serialOut` calls.
    pub fn serial_out(&self) -> Vec<(FwIndexType, Vec<u8>)> {
        self.g.serial_out.lock().unwrap().clone()
    }

    /// C++ `assertSequenceFailureCount`: the four failure events together.
    pub fn assert_sequence_failure_count(&self, expected: usize) {
        let actual = self.events(WasmSequencer::EVENTID_SEQUENCE_EXITED).len()
            + self.events(WasmSequencer::EVENTID_SEQUENCE_PANIC).len()
            + self.events(WasmSequencer::EVENTID_SEQUENCE_TRAPPED).len()
            + self
                .events(WasmSequencer::EVENTID_SEQUENCE_HOST_FAILURE)
                .len();
        assert_eq!(actual, expected, "{:?}", self.all_events());
    }

    // -- white box ------------------------------------------------------------

    /// C++ `dynamicPagesUsed`.
    pub fn dynamic_pages_used(&self) -> usize {
        self.comp.heap_pages_used()
    }

    /// C++ `getGuestOffset`.
    pub fn guest_offset(&self) -> usize {
        self.comp.guest_pool().unwrap().offset()
    }

    /// C++ `getPendingHostFunctionKind`.
    pub fn pending_kind(&self) -> HostFunction {
        self.comp.state().pending.kind
    }

    /// C++ `hasPendingTimer`.
    pub fn has_pending_timer(&self) -> bool {
        self.comp.state().pending_timer.is_some()
    }

    /// Set `next_tlm` (C++ `nextTlmId` / `nextTlmTime` / `nextTlmValue`).
    pub fn set_next_tlm(&self, id: FwChanIdType, time: Time, value: &[u8]) {
        let mut buf = TlmBuffer::new();
        assert!(
            buf.serialize_bytes(value, LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        *self.g.next_tlm.lock().unwrap() = (id, time, buf);
    }

    /// Set `next_prm` (C++ `nextPrmId` / `nextPrmValue`).
    pub fn set_next_prm(&self, id: FwPrmIdType, value: &[u8]) {
        let mut buf = ParamBuffer::new();
        assert!(
            buf.serialize_bytes(value, LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        *self.g.next_prm.lock().unwrap() = (id, buf);
    }
}

/// C++ `makeSeqArgs`.
pub(super) fn make_seq_args(bytes: &[u8]) -> SeqArgs {
    let mut args = SeqArgs::default();
    args.buffer.0[..bytes.len()].copy_from_slice(bytes);
    args.size = bytes.len() as FwSizeType;
    args
}

/// C++ `Fw::Time(seconds, useconds)` (time base `TB_NONE`).
pub(super) fn time(seconds: u32, useconds: u32) -> Time {
    Time::from_seconds_useconds(seconds, useconds)
}

/// C++ `Fw::Time(base, 0, seconds, useconds)`.
pub(super) fn time_in(base: TimeBase, seconds: u32, useconds: u32) -> Time {
    Time::new(base, 0, seconds, useconds)
}

/// Serialized argument bytes, built as the generated code serializes them
/// (big-endian, strings `[u16 length][bytes]`).
#[derive(Debug, Clone, Default)]
pub(super) struct Bytes(pub Vec<u8>);

impl Bytes {
    pub fn new() -> Self {
        Self(Vec::new())
    }
    pub fn u8(mut self, v: u8) -> Self {
        self.0.push(v);
        self
    }
    pub fn u16(mut self, v: u16) -> Self {
        self.0.extend(v.to_be_bytes());
        self
    }
    pub fn u32(mut self, v: u32) -> Self {
        self.0.extend(v.to_be_bytes());
        self
    }
    pub fn i32(mut self, v: i32) -> Self {
        self.0.extend(v.to_be_bytes());
        self
    }
    pub fn u64(mut self, v: u64) -> Self {
        self.0.extend(v.to_be_bytes());
        self
    }
    pub fn i64(mut self, v: i64) -> Self {
        self.0.extend(v.to_be_bytes());
        self
    }
    pub fn f32(mut self, v: f32) -> Self {
        self.0.extend(v.to_be_bytes());
        self
    }
    pub fn f64(mut self, v: f64) -> Self {
        self.0.extend(v.to_be_bytes());
        self
    }
    pub fn str(mut self, s: &str) -> Self {
        self.0.extend((s.len() as u16).to_be_bytes());
        self.0.extend(s.as_bytes());
        self
    }
    pub fn seq_args(mut self, args: &SeqArgs) -> Self {
        let mut buf = LinearBuffer::<64>::new();
        assert!(buf.serialize(args, Endianness::Big).is_ok());
        self.0.extend(buf.as_slice());
        self
    }
    /// An FPP enum / serializable.
    pub fn ser<T: Serialize>(mut self, v: &T) -> Self {
        let mut buf = LinearBuffer::<64>::new();
        assert!(v.serialize_to(&mut buf, Endianness::Big).is_ok());
        self.0.extend(buf.as_slice());
        self
    }
}

/// Shorthands for the event ids and opcodes.
pub(super) type W = WasmSequencer;
