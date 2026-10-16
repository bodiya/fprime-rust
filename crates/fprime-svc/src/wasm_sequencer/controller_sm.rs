//! `Svc.WasmSequencer.ControllerStateMachine`
//! (`WasmSequencerControllerStateMachine.fppi`): the load / validate /
//! lifecycle machine. Its golden traces (from the C++ `fpp-to-cpp` 3.3.0
//! generates) are checked by `tests/wasm_sequencer_sm_traces.rs`.
#![allow(non_snake_case, non_camel_case_types)]

use super::types::{InvokeRequest, LoadRequest, RequestContext};

fprime_comp::fpp_state_machine! {
    /// `Svc.WasmSequencer.ControllerStateMachine`.
    state machine ControllerStateMachine: u8 {
        actions {
            processInvoke => action_processInvoke (ref InvokeRequest),
            setCancelRequested => action_setCancelRequested,
            clearCancelRequested => action_clearCancelRequested,
            cancelPendingRequest => action_cancelPendingRequest (ref RequestContext),
            respond_noblock_OK => action_respond_noblock_OK (ref RequestContext),
            respond_ERROR => action_respond_ERROR (ref RequestContext),
            incrementSequenceFailure => action_incrementSequenceFailure,
            respondInvoke_BUSY => action_respondInvoke_BUSY (ref InvokeRequest),
            respondInvoke_ERROR => action_respondInvoke_ERROR (ref InvokeRequest),
            respondLoad_BUSY => action_respondLoad_BUSY (ref LoadRequest),
            respond_block_OK => action_respond_block_OK (ref RequestContext),
            respond_block_ERROR => action_respond_block_ERROR (ref RequestContext),
            load => action_load (ref LoadRequest),
            invokeStart => action_invokeStart (ref RequestContext),
            invokeMain => action_invokeMain (ref RequestContext),
            reportModuleInvalidMain => action_reportModuleInvalidMain (ref RequestContext),
            reportModuleMainInvokeFailed => action_reportModuleMainInvokeFailed (ref RequestContext),
            reportModuleStartInvokeFailed => action_reportModuleStartInvokeFailed (ref RequestContext),
            reportModuleStarted => action_reportModuleStarted (ref RequestContext),
            reportModuleSucceeded => action_reportModuleSucceeded (ref RequestContext),
            reportModuleStartFailed => action_reportModuleStartFailed (ref RequestContext),
            reportModuleMainFailed => action_reportModuleMainFailed (ref RequestContext),
            resetStore => action_resetStore,
            runEngine => action_runEngine (ref RequestContext),
        }
        guards {
            cancelRequested => guard_cancelRequested,
            moduleHasStart => guard_moduleHasStart (ref RequestContext),
            moduleHasValidMain => guard_moduleHasValidMain (ref RequestContext),
            invokeSucceeded => guard_invokeSucceeded (ref RequestContext),
            interpreterSucceeded => guard_interpreterSucceeded,
        }
        signals {
            cancel = 1 => send_signal_cancel {
                IDLE => enter IDLE;
                READY => enter IDLE;
                INVOKING => do { setCancelRequested };
                LOADING_TO_READY => do { setCancelRequested };
                LOADING_TO_RUN => do { setCancelRequested };
                RUNNING_START_PENDING_MAIN => do { setCancelRequested };
            }
            engineFinished = 2 => send_signal_engineFinished (ref RequestContext) {
                LOAD_RUNNING_START => enter LOAD_RUNNING_START_INTERPRETER_STATUS;
                RUNNING_START_PENDING_MAIN => enter RUNNING_START_PENDING_MAIN_INTERPRETER_STATUS;
                RUNNING_MAIN => enter RUNNING_MAIN_INTERPRETER_STATUS;
            }
            invoke = 3 => send_signal_invoke (ref InvokeRequest) {
                IDLE => do { respondInvoke_ERROR };
                READY => do { processInvoke } enter INVOKING;
                INVOKING => do { respondInvoke_BUSY };
                LOADING_TO_READY => do { respondInvoke_BUSY };
                LOAD_RUNNING_START => do { respondInvoke_BUSY };
                LOADING_TO_RUN => do { respondInvoke_BUSY };
                RUNNING_START_PENDING_MAIN => do { respondInvoke_BUSY };
                RUNNING_MAIN => do { respondInvoke_BUSY };
            }
            invokeFailed = 4 => send_signal_invokeFailed (ref RequestContext) {
                INVOKING => do { incrementSequenceFailure, respond_ERROR } enter READY;
            }
            invoked = 5 => send_signal_invoked (ref RequestContext) {
                INVOKING => enter INVOKE_CANCEL_CHECK;
            }
            load = 6 => send_signal_load (ref LoadRequest) {
                IDLE => do { load } enter LOADING_TO_READY;
                READY => do { load } enter LOADING_TO_READY;
                INVOKING => do { respondLoad_BUSY };
                LOADING_TO_READY => do { respondLoad_BUSY };
                LOAD_RUNNING_START => do { respondLoad_BUSY };
                LOADING_TO_RUN => do { respondLoad_BUSY };
                RUNNING_START_PENDING_MAIN => do { respondLoad_BUSY };
                RUNNING_MAIN => do { respondLoad_BUSY };
            }
            loadFailed = 7 => send_signal_loadFailed (ref RequestContext) {
                LOADING_TO_READY => do { incrementSequenceFailure, respond_ERROR } enter IDLE;
                LOADING_TO_RUN => do { incrementSequenceFailure, respond_ERROR } enter IDLE;
            }
            loadSucceeded = 8 => send_signal_loadSucceeded (ref RequestContext) {
                LOADING_TO_READY => enter LOAD_READY_CANCEL_CHECK;
                LOADING_TO_RUN => enter LOAD_RUN_CANCEL_CHECK;
            }
            run = 9 => send_signal_run (ref LoadRequest) {
                IDLE => do { load } enter LOADING_TO_RUN;
                READY => do { resetStore, load } enter LOADING_TO_RUN;
                INVOKING => do { respondLoad_BUSY };
                LOADING_TO_READY => do { respondLoad_BUSY };
                LOAD_RUNNING_START => do { respondLoad_BUSY };
                LOADING_TO_RUN => do { respondLoad_BUSY };
                RUNNING_START_PENDING_MAIN => do { respondLoad_BUSY };
                RUNNING_MAIN => do { respondLoad_BUSY };
            }
        }
        initial enter IDLE;
        states {
            state IDLE = 1 {
                entry do { resetStore, clearCancelRequested }
            }
            state INVOKING = 2 {}
            state LOADING_TO_READY = 3 {}
            state LOADING_TO_RUN = 4 {}
            state LOAD_RUNNING_START = 5 {}
            state READY = 6 {
                entry do { clearCancelRequested }
            }
            state RUNNING_MAIN = 7 {}
            state RUNNING_START_PENDING_MAIN = 8 {}
        }
        choices {
            choice INVOKE_CANCEL_CHECK (ref RequestContext) {
                if cancelRequested do { cancelPendingRequest } enter IDLE
                else enter MAIN_CHECK
            }
            choice MAIN_CHECK (ref RequestContext) {
                if moduleHasValidMain do { invokeMain } enter MAIN_INVOKE_CHECK
                else do { reportModuleInvalidMain, incrementSequenceFailure, respond_ERROR } enter READY
            }
            choice MAIN_INVOKE_CHECK (ref RequestContext) {
                if invokeSucceeded do { reportModuleStarted, respond_noblock_OK, runEngine } enter RUNNING_MAIN
                else do { reportModuleMainInvokeFailed, incrementSequenceFailure, respond_ERROR } enter IDLE
            }
            choice LOAD_READY_CANCEL_CHECK (ref RequestContext) {
                if cancelRequested do { cancelPendingRequest } enter IDLE
                else enter LOAD_START_CHECK
            }
            choice LOAD_START_CHECK (ref RequestContext) {
                if moduleHasStart do { invokeStart } enter LOAD_START_INVOKE_CHECK
                else do { respond_block_OK } enter READY
            }
            choice LOAD_START_INVOKE_CHECK (ref RequestContext) {
                if invokeSucceeded do { runEngine } enter LOAD_RUNNING_START
                else do { reportModuleStartInvokeFailed, incrementSequenceFailure, respond_ERROR } enter IDLE
            }
            choice LOAD_RUNNING_START_INTERPRETER_STATUS in LOAD_RUNNING_START (ref RequestContext) {
                if interpreterSucceeded do { respond_block_OK } enter READY
                else do { reportModuleStartFailed, respond_ERROR } enter IDLE
            }
            choice LOAD_RUN_CANCEL_CHECK (ref RequestContext) {
                if cancelRequested do { cancelPendingRequest } enter IDLE
                else enter MAIN_CHECK_PENDING_START_AND_MAIN
            }
            choice MAIN_CHECK_PENDING_START_AND_MAIN (ref RequestContext) {
                if moduleHasValidMain enter START_CHECK_PENDING_MAIN
                else do { reportModuleInvalidMain, incrementSequenceFailure, respond_ERROR } enter READY
            }
            choice START_CHECK_PENDING_MAIN (ref RequestContext) {
                if moduleHasStart do { invokeStart } enter START_INVOKE_CHECK_PENDING_CHAIN
                else do { invokeMain } enter MAIN_INVOKE_CHECK
            }
            choice START_INVOKE_CHECK_PENDING_CHAIN (ref RequestContext) {
                if invokeSucceeded do { runEngine } enter RUNNING_START_PENDING_MAIN
                else do { reportModuleStartInvokeFailed, incrementSequenceFailure, respond_ERROR } enter IDLE
            }
            choice RUNNING_START_PENDING_MAIN_INTERPRETER_STATUS in RUNNING_START_PENDING_MAIN (ref RequestContext) {
                if interpreterSucceeded enter RUNNING_START_PENDING_MAIN_START_MAIN_CANCEL_CHECK
                else do { reportModuleStartFailed, respond_ERROR } enter IDLE
            }
            choice RUNNING_START_PENDING_MAIN_START_MAIN_CANCEL_CHECK in RUNNING_START_PENDING_MAIN (ref RequestContext) {
                if cancelRequested do { cancelPendingRequest } enter IDLE
                else do { invokeMain } enter MAIN_INVOKE_CHECK
            }
            choice RUNNING_MAIN_INTERPRETER_STATUS in RUNNING_MAIN (ref RequestContext) {
                if interpreterSucceeded do { reportModuleSucceeded, respond_block_OK } enter READY
                else do { reportModuleMainFailed, respond_block_ERROR } enter IDLE
            }
        }
    }
}
