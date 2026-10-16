//! The controller state machine's actions and guards
//! (`WasmSequencerController.cpp`) and the helpers they share
//! (`WasmSequencerHelpers.cpp`): request responses, sequence path
//! resolution, module loading, start / main invocation and the completion
//! reports.

use super::controller_sm::{Handlers, Signal};
use super::{
    CmdResponse, InvokeRequest, LoadRequest, ModuleIdx, RequestContext, SequencePhase,
    SignalSource, Status, WasmSequencer, c_str, lock, status,
};
use crate::wasm_sequencer::ExitReason;
use fprime_config::wasm_sequencer::LOAD_READ_CHUNK_SIZE;
use fprime_config::{FwEnumStoreType, FwSizeType};
use fprime_fw::{FileNameString, FwDefaultString, fw_assert};
use fprime_os::file::{File, Mode as FileMode, Status as FileStatus, WaitType};

impl WasmSequencer {
    /// C++ `respondToRequest`: answer a command-sourced request.
    pub(crate) fn respond_to_request(&self, value: &RequestContext, response: CmdResponse) {
        match value.source {
            SignalSource::CommandRun | SignalSource::CommandInvoke | SignalSource::CommandLoad => {
                self.cmd
                    .cmd_response(value.cmd_ctx.opcode, value.cmd_ctx.cmd_seq, response);
            }
            // Port-driven requests have no command response to send.
            SignalSource::PortRun | SignalSource::PortInvoke => {}
        }
    }

    /// C++ `respondToWaiting`: answer every `WAIT` blocked on completion.
    pub(crate) fn respond_to_waiting(&self, response: CmdResponse) {
        loop {
            let next = lock(&self.state).waiting.pop_front();
            match next {
                Some(cmd) => self.cmd.cmd_response(cmd.op_code, cmd.cmd_seq, response),
                None => break,
            }
        }
    }

    /// C++ `reportSeqDone`: `seqDoneOut` for a `RUN` (the counterpart of the
    /// RUN-only `seqStartOut`), if connected.
    fn report_seq_done(&self, value: &RequestContext, response: CmdResponse) {
        if value.source != SignalSource::CommandRun && value.source != SignalSource::PortRun {
            return;
        }
        if let Some(p) = self.seq_done_out.try_get() {
            p.target.invoke(p.port_num, 0, 0, response);
        }
    }

    /// C++ `pathBaseName`: the segment after the last `/`.
    #[must_use]
    pub fn path_base_name(path: &[u8]) -> &[u8] {
        match path.iter().rposition(|&b| b == b'/') {
            Some(i) => &path[i + 1..],
            None => path,
        }
    }

    /// C++ `pathHasParentTraversal`: some `/`-delimited segment is exactly
    /// `..`.
    #[must_use]
    pub fn path_has_parent_traversal(path: &[u8]) -> bool {
        path.split(|&b| b == b'/').any(|segment| segment == b"..")
    }

    /// C++ `resolveSequencePath`: prefix `SEQ_BASE_DIR` (joined with
    /// exactly one `/`). With a base directory set, a `..` segment is
    /// rejected (it could escape the base), and so is a joined path that
    /// does not fit `Fw::String`; each reports its event and returns
    /// `None`.
    pub(crate) fn resolve_sequence_path(&self, file_name: &[u8]) -> Option<FwDefaultString> {
        let (base_dir, _valid) = self.param_get_seq_base_dir();
        let mut file_path = FwDefaultString::new();
        if base_dir.is_empty() {
            file_path.set_bytes(file_name);
            return Some(file_path);
        }
        if Self::path_has_parent_traversal(file_name) {
            self.log_warning_hi_sequence_file_path_not_contained(base_dir.as_bytes(), file_name);
            return None;
        }
        let base = base_dir.as_bytes();
        let separator: &[u8] = if base.last() == Some(&b'/') {
            b""
        } else {
            b"/"
        };
        // C++ `format("%s%s%s")` returns OVERFLOWED when the result does
        // not fit; the truncated path is never used.
        if base.len() + separator.len() + file_name.len() > FwDefaultString::CAPACITY {
            self.log_warning_hi_sequence_file_path_too_long(base, file_name);
            return None;
        }
        file_path.set_bytes(base);
        file_path.append_bytes(separator);
        file_path.append_bytes(file_name);
        Some(file_path)
    }

    /// C++ `setSequenceName`: the `SeqName` telemetry — the module name when
    /// one was given, else the file's basename without a `.wasm` suffix.
    pub(crate) fn set_sequence_name(&self, file_path: &[u8], module_name: &[u8]) {
        let mut st = lock(&self.state);
        if !module_name.is_empty() {
            st.tlm.sequence_name.set_bytes(module_name);
            return;
        }
        let mut base = Self::path_base_name(file_path);
        if let Some(stem) = base.strip_suffix(b".wasm") {
            base = stem;
        }
        // C++ copies into a `char[FileNameStringSize]`, so at most
        // `FileNameStringSize - 1` characters survive.
        let n = base.len().min(FileNameString::CAPACITY - 1);
        st.tlm.sequence_name.set_bytes(&base[..n]);
    }

    /// C++ `validateModuleMain`: `main` is exported with `[] -> []` or
    /// `[] -> i32`; the status says why not.
    pub(crate) fn validate_module_main(&self, module_idx: ModuleIdx) -> Status {
        let idx = u32::from(module_idx);
        self.with_engine(|e| {
            let main = e.find_export_func(idx, "main")?;
            // Checking against "" first yields PARAM_LEN_MISMATCH (not
            // BAD_SIGNATURE) for an i32 main, so fall back on any mismatch.
            match e.check_func_signature(idx, main, "", "") {
                Ok(()) => Ok(()),
                Err(_) => e.check_func_signature(idx, main, "", "i"),
            }
        })
        .map_or_else(|e| status::error_status(&e), |()| Status::Ok)
    }

    /// C++ `reportSequenceRuntimeFailure`: the completion event for a failed
    /// start or main, with the matching counter.
    fn report_sequence_runtime_failure(&self, module_idx: ModuleIdx, phase: SequencePhase) {
        let exit = {
            let mut st = lock(&self.state);
            if st.exit.reason == ExitReason::Cancel {
                st.tlm.sequences_cancelled += 1;
            } else {
                st.tlm.sequences_failed += 1;
            }
            st.exit
        };
        match exit.reason {
            ExitReason::Cancel => self.log_activity_hi_sequence_cancelled(module_idx, phase),
            ExitReason::HostPanic => {
                self.log_warning_hi_sequence_panic(module_idx, phase, exit.code)
            }
            ExitReason::InterpreterTrap => {
                self.log_warning_hi_sequence_trapped(module_idx, phase, exit.last_trap_reason);
            }
            // Only reached on the failure path: a non-zero main result or
            // a non-zero exit code.
            ExitReason::InterpreterFinished | ExitReason::HostExit => {
                self.log_warning_hi_sequence_exited(module_idx, phase, exit.code);
            }
            ExitReason::Unknown
            | ExitReason::ReplyTimeout
            | ExitReason::HostFailure
            | ExitReason::UnexpectedReply
            | ExitReason::TimerIncomparable => {
                self.log_warning_hi_sequence_host_failure(
                    module_idx,
                    phase,
                    exit.reason,
                    exit.last_host_function,
                );
            }
        }
    }

    /// The load action proper: resolve, open and stream the module into the
    /// store; `Ok` carries the module index.
    fn load_module(&self, value: &LoadRequest) -> Result<u32, ()> {
        let Some(file_path) = self.resolve_sequence_path(value.file_name.as_bytes()) else {
            return Err(());
        };
        // Record the sequence name for telemetry (module name, or stem).
        self.set_sequence_name(file_path.as_bytes(), value.module_name.as_bytes());
        // Os::File::open takes a path string: a non-UTF-8 path is opened
        // with replacement characters (and fails).
        let path = String::from_utf8_lossy(file_path.as_bytes());
        let mut file = File::new();
        let open_status = file.open(&path, FileMode::OpenRead);
        if open_status != FileStatus::OpOk {
            self.log_warning_hi_file_open_error(file_path.as_bytes(), open_status as i32);
            return Err(());
        }
        lock(&self.state).last_load_file_name = value.file_name.clone();
        let result = match c_str(value.module_name.as_bytes()) {
            Err(status) => Err(status),
            Ok(name) => {
                // C++ `WasmFileReader`: one LOAD_READ_CHUNK_SIZE chunk per
                // call, a zero-length read at the end of the file.
                let mut read = |buf: &mut [u8]| -> Result<usize, ()> {
                    let mut size = buf.len() as FwSizeType;
                    match file.read(buf, &mut size, WaitType::Wait) {
                        FileStatus::OpOk => Ok(size as usize),
                        _ => Err(()),
                    }
                };
                self.with_heap(|guard| {
                    self.with_engine(|e| {
                        e.load_module::<LOAD_READ_CHUNK_SIZE>(guard, name, &mut read)
                    })
                })
                .map_err(|e| status::error_status(&e))
            }
        };
        file.close();
        match result {
            Ok(idx) => Ok(idx),
            Err(status) => {
                self.log_warning_hi_module_load_failed(status);
                Err(())
            }
        }
    }
}

#[allow(non_snake_case)]
impl Handlers for WasmSequencer {
    /// Resolve the module name to its index and raise `invoked` (or
    /// `invokeFailed`).
    fn action_processInvoke(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &InvokeRequest,
    ) {
        self.assert_store();
        let found = c_str(value.module_name.as_bytes())
            .ok()
            .and_then(|name| self.with_engine(|e| e.find_module(name).ok()));
        match found {
            None => {
                self.log_warning_lo_module_not_found(value.module_name.as_bytes());
                self.ctl().send_signal_invokeFailed(&value.context);
            }
            Some(module_idx) => {
                // Keep the arguments for the `args` host function and carry
                // the resolved module index forward.
                lock(&self.state).args = value.args;
                let mut context = value.context;
                context.module_idx = module_idx as ModuleIdx;
                self.ctl().send_signal_invoked(&context);
            }
        }
    }

    fn action_setCancelRequested(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).cancel_requested = true;
    }

    fn action_clearCancelRequested(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).cancel_requested = false;
    }

    fn action_cancelPendingRequest(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        lock(&self.state).tlm.sequences_cancelled += 1;
        self.respond_to_request(value, CmdResponse::ExecutionError);
        self.respond_to_waiting(CmdResponse::ExecutionError);
    }

    fn action_respond_noblock_OK(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        if value.block == super::BlockState::NoBlock {
            self.respond_to_request(value, CmdResponse::Ok);
        }
        // Clear any pending waits that locked during load
        if value.source == SignalSource::CommandLoad {
            self.respond_to_waiting(CmdResponse::Ok);
        }
    }

    fn action_respond_ERROR(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        self.respond_to_request(value, CmdResponse::ExecutionError);
        self.respond_to_waiting(CmdResponse::ExecutionError);
    }

    fn action_incrementSequenceFailure(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).tlm.sequences_failed += 1;
    }

    fn action_respondInvoke_BUSY(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &InvokeRequest,
    ) {
        self.log_warning_lo_controller_busy(value.context.source, self.controller_get_state());
        self.respond_to_request(&value.context, CmdResponse::Busy);
    }

    fn action_respondInvoke_ERROR(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &InvokeRequest,
    ) {
        self.log_warning_lo_controller_cannot_invoke();
        self.respond_to_request(&value.context, CmdResponse::ExecutionError);
    }

    fn action_respondLoad_BUSY(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &LoadRequest,
    ) {
        self.log_warning_lo_controller_busy(value.context.source, self.controller_get_state());
        self.respond_to_request(&value.context, CmdResponse::Busy);
    }

    fn action_respond_block_OK(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        if value.block == super::BlockState::Block {
            self.respond_to_request(value, CmdResponse::Ok);
        }
        self.respond_to_waiting(CmdResponse::Ok);
        self.report_seq_done(value, CmdResponse::Ok);
    }

    fn action_respond_block_ERROR(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        if value.block == super::BlockState::Block {
            self.respond_to_request(value, CmdResponse::ExecutionError);
        }
        self.respond_to_waiting(CmdResponse::ExecutionError);
        self.report_seq_done(value, CmdResponse::ExecutionError);
    }

    /// Load a module into the store; raise `loadSucceeded` (with the module
    /// index) or `loadFailed`.
    fn action_load(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: &LoadRequest) {
        self.assert_store();
        fw_assert!(self.guest_pool().is_some());
        lock(&self.state).args = value.args;
        match self.load_module(value) {
            Ok(module_idx) => {
                let mut next = value.context;
                next.module_idx = module_idx as ModuleIdx;
                self.ctl().send_signal_loadSucceeded(&next);
            }
            Err(()) => self.ctl().send_signal_loadFailed(&value.context),
        }
    }

    /// Set up the call of the module's start function.
    fn action_invokeStart(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: &RequestContext) {
        let status = self.with_engine(|e| {
            let start = e.module_start(u32::from(value.module_idx));
            let Ok((module, function)) = start else {
                fw_assert!(false);
                unreachable!()
            };
            e.invoke(module, function, &[])
        });
        lock(&self.state).invoke_status =
            status.map_or_else(|e| status::error_status(&e), |()| Status::Ok);
    }

    /// Set up the call of the module's `main`.
    fn action_invokeMain(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: &RequestContext) {
        let idx = u32::from(value.module_idx);
        let status = self.with_engine(|e| {
            // The state machine checked that main exists and is valid.
            let Ok(main) = e.find_export_func(idx, "main") else {
                fw_assert!(false);
                unreachable!()
            };
            e.invoke(idx, main, &[])
        });
        lock(&self.state).invoke_status =
            status.map_or_else(|e| status::error_status(&e), |()| Status::Ok);
    }

    fn action_reportModuleInvalidMain(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        let problem = self.validate_module_main(value.module_idx);
        self.log_warning_hi_invalid_module_entrypoint(value.module_idx, problem);
    }

    fn action_reportModuleMainInvokeFailed(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        _value: &RequestContext,
    ) {
        let status = lock(&self.state).invoke_status;
        self.log_warning_hi_module_main_invoke_failed(status);
    }

    fn action_reportModuleStartInvokeFailed(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        _value: &RequestContext,
    ) {
        let status = lock(&self.state).invoke_status;
        self.log_warning_hi_module_start_invoke_failed(status);
    }

    /// `SequenceStarting`, and `seqStartOut` for a `RUN`.
    fn action_reportModuleStarted(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        self.log_activity_hi_sequence_starting(value.module_idx);
        if value.source == SignalSource::CommandRun || value.source == SignalSource::PortRun {
            if let Some(p) = self.seq_start_out.try_get() {
                let (file, args) = {
                    let st = lock(&self.state);
                    (st.last_load_file_name.clone(), st.args)
                };
                p.target.invoke(p.port_num, &file, &args);
            }
        }
    }

    fn action_reportModuleSucceeded(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        lock(&self.state).tlm.sequences_succeeded += 1;
        self.log_activity_hi_sequence_succeeded(value.module_idx);
    }

    fn action_reportModuleStartFailed(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        self.report_sequence_runtime_failure(value.module_idx, SequencePhase::Start);
    }

    fn action_reportModuleMainFailed(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) {
        self.report_sequence_runtime_failure(value.module_idx, SequencePhase::Main);
    }

    /// Replace the store with a fresh one. On the initial transition there
    /// is no store yet: `configure` creates the first.
    fn action_resetStore(&self, _sm_id: FwEnumStoreType, signal: Signal) {
        if !self.is_store_created() {
            fw_assert!(
                signal == Signal::__FPRIME_INITIAL_TRANSITION,
                signal.as_repr()
            );
        } else {
            self.destroy_store();
            self.create_store();
        }
    }

    fn action_runEngine(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: &RequestContext) {
        self.itp().send_signal_run(value);
    }

    fn guard_cancelRequested(&self, _sm_id: FwEnumStoreType, _signal: Signal) -> bool {
        lock(&self.state).cancel_requested
    }

    fn guard_moduleHasStart(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) -> bool {
        match self.with_engine(|e| e.module_start(u32::from(value.module_idx))) {
            Ok(_) => true,
            Err(fprime_spacewasm::Error::NotFound) => false,
            // Any other status means a bad call
            Err(e) => {
                fw_assert!(false, status::error_status(&e).as_repr());
                false
            }
        }
    }

    fn guard_moduleHasValidMain(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: &RequestContext,
    ) -> bool {
        self.validate_module_main(value.module_idx) == Status::Ok
    }

    fn guard_invokeSucceeded(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        _value: &RequestContext,
    ) -> bool {
        lock(&self.state).invoke_status == Status::Ok
    }

    /// A run succeeds only when it finished with code 0: main returned 0
    /// (a void main reports 0) or the guest called `exit(0)`.
    fn guard_interpreterSucceeded(&self, _sm_id: FwEnumStoreType, _signal: Signal) -> bool {
        let st = lock(&self.state);
        match st.exit.reason {
            ExitReason::InterpreterFinished | ExitReason::HostExit => st.exit.code == 0,
            _ => false,
        }
    }
}
