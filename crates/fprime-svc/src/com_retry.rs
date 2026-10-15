//! # ComRetry — port of `Svc::ComRetry` (passive)
//!
//! C++ sources: `Svc/ComRetry/ComRetry.{cpp,hpp,fpp}`,
//! `Svc/ComRetry/docs/sdd.md`; the ports are the `Svc.Framer` interface
//! (`Svc/Interfaces/Framer.fpp`).
//!
//! Sits between an upstream sender (e.g. `ComQueue` or a framer) and a
//! communication adapter (`ComStub`, a radio) and resends a buffer the
//! adapter failed to deliver. `dataIn` is passed straight to `dataOut`;
//! the adapter hands the buffer back on `dataReturnIn` and ComRetry keeps
//! it until the adapter's status arrives on `comStatusIn`:
//!
//! - `SUCCESS`: the kept buffer goes back upstream on `dataReturnOut` and
//!   the status on `comStatusOut`;
//! - `FAILURE` with retries left: ComRetry waits (`RETRYING`) for the
//!   adapter's next `SUCCESS` (it is ready again), then resends the kept
//!   buffer on `dataOut`;
//! - `FAILURE` with no retries left: the buffer goes back upstream and the
//!   `FAILURE` on `comStatusOut`; the adapter's next `SUCCESS` then passes
//!   through and restarts the data flow.
//!
//! With no buffer in flight every status passes straight through (the
//! start-up `SUCCESS` that primes `ComQueue`). The retry budget is set
//! with [`ComRetry::configure`] (default 3).
//!
//! C++ parity kept: the protocol is asserted, not tolerated — a status
//! before the buffer came back, a second `dataIn` while one is in flight,
//! a `dataReturnIn` for a buffer that was never sent, a `SUCCESS`-only
//! state receiving `FAILURE`, and a kept buffer that is not valid
//! (`Fw::Buffer::isValid`, so a zero-size buffer too) are `fw_assert!`s.
//!
//! Rust ownership and concurrency: the kept buffer is an
//! `Option<Buffer>` that moves out on resend and return (C++ copies the
//! descriptor and clears it after the return). The state sits in a mutex
//! that is released before any output port is invoked, because a
//! synchronous adapter calls back into `dataReturnIn` / `comStatusIn` from
//! inside `dataOut`; every state change happens before the call that may
//! re-enter, as in C++. (C++ clears its kept buffer only after
//! `dataReturnOut` returns; here it has already moved out.)

use fprime_comp::{ComDataWithContextPort, OutputPort, PassiveBase, PortRef, SuccessConditionPort};
use fprime_config::FwIndexType;
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{Buffer, FrameContext, Success, fw_assert};

/// C++ `ComRetry::RetryState`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetryState {
    /// A buffer is downstream; waiting for the adapter's status.
    WaitingForStatus = 0,
    /// Idle: no buffer in flight.
    WaitingForSend = 1,
    /// The last delivery failed; the next `SUCCESS` triggers a resend.
    Retrying = 2,
}

/// C++ `Fw::Buffer::OwnershipState`, as ComRetry tracks it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ownership {
    NotOwned,
    Owned,
}

struct State {
    /// `m_num_retries`.
    num_retries: u32,
    /// `m_retry_count`.
    retry_count: u32,
    /// `m_retry_state`.
    retry_state: RetryState,
    /// `m_context` — the kept buffer's context.
    context: FrameContext,
    /// `m_buffer` — the kept buffer (moves out on resend/return).
    buffer: Option<Buffer>,
    /// `m_bufferState`.
    buffer_state: Ownership,
}

impl State {
    /// C++ `m_buffer.isValid()`.
    fn buffer_valid(&self) -> bool {
        self.buffer.as_ref().is_some_and(Buffer::is_valid)
    }

    /// Move the kept buffer out (C++ passes `m_buffer`; asserted present
    /// by the protocol — a returning hook gets the default buffer).
    fn take_buffer(&mut self) -> Buffer {
        let buffer = self.buffer.take();
        fw_assert!(buffer.is_some());
        buffer.unwrap_or_default()
    }
}

/// What `comStatusIn` does once the state lock is released.
enum Action {
    StatusOut(Success),
    ReturnThenStatus(Buffer, FrameContext, Success),
    Resend(Buffer, FrameContext),
    Nothing,
}

/// `Svc::ComRetry`.
pub struct ComRetry {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `dataOut` — to the communication adapter.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — buffer ownership back to the sender.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — delivery status to the sender.
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    state: Mutex<State>,
}

impl ComRetry {
    /// Default retry budget (C++ constructor: `m_num_retries(3)`).
    pub const DEFAULT_NUM_RETRIES: u32 = 3;

    /// Construct idle (`WAITING_FOR_SEND`, buffer owned, 3 retries).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            com_status_out: OutputPort::new(),
            state: Mutex::new(State {
                num_retries: Self::DEFAULT_NUM_RETRIES,
                retry_count: 0,
                retry_state: RetryState::WaitingForSend,
                context: FrameContext::default(),
                buffer: None,
                buffer_state: Ownership::Owned,
            }),
        })
    }

    /// C++ `configure(num_retries)`: the number of resends after the first
    /// delivery fails.
    pub fn configure(&self, num_retries: u32) {
        self.state.lock().num_retries = num_retries;
    }

    /// The current retry state (for tests and diagnostics).
    pub fn retry_state(&self) -> RetryState {
        self.state.lock().retry_state
    }

    /// `dataIn` — SYNC `Svc.ComDataWithContext` input.
    pub fn data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(Arc::new(DataInAdapter(self.clone())), port_num)
    }

    /// `dataReturnIn` — SYNC `Svc.ComDataWithContext` input: the adapter
    /// hands the sent buffer back.
    pub fn data_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(Arc::new(DataReturnInAdapter(self.clone())), port_num)
    }

    /// `comStatusIn` — SYNC `Fw.SuccessCondition` input.
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `comStatusIn_handler`.
    fn com_status_in_handler(&self, _port_num: FwIndexType, condition: Success) {
        let action = {
            let mut st = self.state.lock();
            fw_assert!(st.buffer_state == Ownership::Owned);
            match (st.retry_state, condition) {
                (RetryState::WaitingForSend, _) => {
                    fw_assert!(!st.buffer_valid());
                    Action::StatusOut(condition)
                }
                (RetryState::WaitingForStatus, Success::Success) => {
                    fw_assert!(st.buffer_valid());
                    // Successful transmission, reset state.
                    st.retry_state = RetryState::WaitingForSend;
                    Action::ReturnThenStatus(st.take_buffer(), st.context, condition)
                }
                (RetryState::Retrying, Success::Success) => {
                    fw_assert!(st.buffer_valid());
                    st.retry_count += 1;
                    st.retry_state = RetryState::WaitingForStatus;
                    st.buffer_state = Ownership::NotOwned;
                    Action::Resend(st.take_buffer(), st.context)
                }
                (retry_state, condition) => {
                    fw_assert!(
                        retry_state == RetryState::WaitingForStatus,
                        retry_state as i32
                    );
                    fw_assert!(condition == Success::Failure, condition as i32);
                    if st.retry_count < st.num_retries {
                        st.retry_state = RetryState::Retrying;
                        Action::Nothing
                    } else {
                        st.retry_state = RetryState::WaitingForSend;
                        Action::ReturnThenStatus(st.take_buffer(), st.context, condition)
                    }
                }
            }
        };
        match action {
            Action::StatusOut(mut condition) => self.com_status_out(&mut condition),
            Action::ReturnThenStatus(buffer, context, mut condition) => {
                let p = self.data_return_out.get();
                p.target.invoke(p.port_num, buffer, &context);
                self.com_status_out(&mut condition);
            }
            Action::Resend(buffer, context) => {
                let p = self.data_out.get();
                p.target.invoke(p.port_num, buffer, &context);
            }
            Action::Nothing => {}
        }
    }

    fn com_status_out(&self, condition: &mut Success) {
        let p = self.com_status_out.get();
        p.target.invoke(p.port_num, condition);
    }

    /// `dataIn_handler`.
    fn data_in_handler(&self, _port_num: FwIndexType, buffer: Buffer, context: &FrameContext) {
        {
            let mut st = self.state.lock();
            fw_assert!(st.buffer_state == Ownership::Owned);
            fw_assert!(
                st.retry_state == RetryState::WaitingForSend,
                st.retry_state as i32
            );
            st.buffer_state = Ownership::NotOwned;
            st.retry_state = RetryState::WaitingForStatus;
            st.retry_count = 0;
        }
        let p = self.data_out.get();
        p.target.invoke(p.port_num, buffer, context);
    }

    /// `dataReturnIn_handler`.
    fn data_return_in_handler(
        &self,
        _port_num: FwIndexType,
        buffer: Buffer,
        context: &FrameContext,
    ) {
        let mut st = self.state.lock();
        fw_assert!(st.buffer_state == Ownership::NotOwned);
        fw_assert!(
            st.retry_state == RetryState::WaitingForStatus,
            st.retry_state as i32
        );
        st.buffer_state = Ownership::Owned;
        st.buffer = Some(buffer);
        st.context = *context;
    }
}

impl SuccessConditionPort for ComRetry {
    fn invoke(&self, port_num: FwIndexType, condition: &mut Success) {
        self.com_status_in_handler(port_num, *condition);
    }
}

/// Adapter for the sync `dataIn` port.
struct DataInAdapter(Arc<ComRetry>);

impl ComDataWithContextPort for DataInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.0.data_in_handler(port_num, data, context);
    }
}

/// Adapter for the sync `dataReturnIn` port.
struct DataReturnInAdapter(Arc<ComRetry>);

impl ComDataWithContextPort for DataReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.0.data_return_in_handler(port_num, data, context);
    }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/ComRetry/test/ut` (`ComRetryTester`), plus the asserted
    //! protocol violations and a synchronous adapter that calls back from
    //! inside `dataOut`.
    use super::*;
    use std::sync::Mutex as StdMutex;
    use std::vec::Vec;

    const DATA_A: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
    const DATA_B: [u8; 4] = [0xFE, 0xED, 0xC0, 0xDE];

    /// The tester's view: the adapter side (`dataOut` keeps the buffer so
    /// the test can hand it back) and the sender side.
    #[derive(Default)]
    struct Tester {
        /// Every `dataOut` payload, in order (`fromPortHistory_dataOut`).
        data_out: StdMutex<Vec<Vec<u8>>>,
        /// The buffer currently held by the "adapter".
        held: StdMutex<Option<(Buffer, FrameContext)>>,
        /// `dataReturnOut` payloads and contexts.
        data_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// `comStatusOut` values.
        status: StdMutex<Vec<Success>>,
    }

    struct DataOut(Arc<Tester>);
    impl ComDataWithContextPort for DataOut {
        fn invoke(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            self.0.data_out.lock().unwrap().push(data.data().to_vec());
            *self.0.held.lock().unwrap() = Some((data, *context));
        }
    }

    struct DataReturnOut(Arc<Tester>);
    impl ComDataWithContextPort for DataReturnOut {
        fn invoke(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            self.0
                .data_return
                .lock()
                .unwrap()
                .push((data.data().to_vec(), *context));
        }
    }

    impl SuccessConditionPort for Tester {
        fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
            self.status.lock().unwrap().push(*condition);
        }
    }

    struct Harness {
        comp: Arc<ComRetry>,
        t: Arc<Tester>,
    }

    impl Harness {
        fn new() -> Self {
            let comp = ComRetry::new("ComRetry");
            let t = Arc::new(Tester::default());
            comp.data_out.connect(Arc::new(DataOut(t.clone())), 0);
            comp.data_return_out
                .connect(Arc::new(DataReturnOut(t.clone())), 0);
            comp.com_status_out.connect(t.clone(), 0);
            Self { comp, t }
        }

        fn status(&self, condition: Success) {
            let p = self.comp.com_status_in(0);
            let mut condition = condition;
            p.target.invoke(p.port_num, &mut condition);
        }

        fn data_in(&self, bytes: &[u8], context: &FrameContext) {
            let p = self.comp.data_in(0);
            let mut buffer = Buffer::allocate(bytes.len());
            buffer.data_mut().copy_from_slice(bytes);
            p.target.invoke(p.port_num, buffer, context);
        }

        /// The adapter returns the buffer it holds (`invoke_to_dataReturnIn`).
        fn data_return(&self) {
            let (buffer, context) = self.t.held.lock().unwrap().take().expect("a held buffer");
            let p = self.comp.data_return_in(0);
            p.target.invoke(p.port_num, buffer, &context);
        }

        /// `receiveBuffer`: `dataIn` then the adapter's `dataReturnIn`.
        fn receive_buffer(&self, bytes: &[u8], context: &FrameContext) {
            self.data_in(bytes, context);
            self.data_return();
        }

        fn data_out(&self) -> Vec<Vec<u8>> {
            self.t.data_out.lock().unwrap().clone()
        }

        fn returned(&self) -> Vec<(Vec<u8>, FrameContext)> {
            self.t.data_return.lock().unwrap().clone()
        }

        fn statuses(&self) -> Vec<Success> {
            self.t.status.lock().unwrap().clone()
        }
    }

    #[test]
    fn null_buffer_statuses_pass_through() {
        // TEST(Nominal, NullBuffer)
        let h = Harness::new();
        h.status(Success::Success);
        h.status(Success::Failure);
        assert_eq!(h.statuses(), vec![Success::Success, Success::Failure]);
        assert!(h.data_out().is_empty());
    }

    #[test]
    fn send() {
        // TEST(Nominal, Send)
        let h = Harness::new();
        h.comp.configure(1);
        let ctx = FrameContext::default();
        h.receive_buffer(&DATA_A, &ctx);
        h.status(Success::Success);
        assert_eq!(h.returned(), vec![(DATA_A.to_vec(), ctx)]);
        assert_eq!(h.statuses(), vec![Success::Success]);
        h.receive_buffer(&DATA_B, &ctx);
        h.status(Success::Success);
        assert_eq!(h.returned()[1], (DATA_B.to_vec(), ctx));
        assert_eq!(h.statuses(), vec![Success::Success, Success::Success]);
        assert_eq!(h.data_out(), vec![DATA_A.to_vec(), DATA_B.to_vec()]);
    }

    #[test]
    fn retry() {
        // TEST(Nominal, Retry)
        let h = Harness::new();
        h.comp.configure(1);
        let ctx = FrameContext::default();
        h.receive_buffer(&DATA_A, &ctx);
        h.status(Success::Failure); // first delivery is a failure
        assert_eq!(h.comp.retry_state(), RetryState::Retrying);
        assert!(h.statuses().is_empty());
        h.status(Success::Success); // adapter ready: resend
        h.data_return();
        h.status(Success::Success); // redelivery is successful
        assert_eq!(h.returned(), vec![(DATA_A.to_vec(), ctx)]);
        assert_eq!(h.statuses(), vec![Success::Success]);
        h.receive_buffer(&DATA_B, &ctx);
        h.status(Success::Success);
        assert_eq!(h.returned()[1], (DATA_B.to_vec(), ctx));
        assert_eq!(h.statuses(), vec![Success::Success, Success::Success]);
        assert_eq!(
            h.data_out(),
            vec![DATA_A.to_vec(), DATA_A.to_vec(), DATA_B.to_vec()]
        );
    }

    #[test]
    fn retry_till_failure() {
        // TEST(Nominal, RetryTillFailure), with the default budget of 3.
        let h = Harness::new();
        let ctx = FrameContext::default();
        h.receive_buffer(&DATA_A, &ctx);
        h.status(Success::Failure);
        assert_eq!(h.data_out().len(), 1);
        for i in 1..=ComRetry::DEFAULT_NUM_RETRIES as usize {
            h.status(Success::Success);
            h.data_return();
            h.status(Success::Failure);
            assert_eq!(h.data_out()[i], DATA_A.to_vec());
        }
        assert_eq!(h.returned(), vec![(DATA_A.to_vec(), ctx)]);
        assert_eq!(h.statuses(), vec![Success::Failure]);
        assert_eq!(h.comp.retry_state(), RetryState::WaitingForSend);
        h.receive_buffer(&DATA_B, &ctx);
        h.status(Success::Success);
        assert_eq!(h.returned()[1], (DATA_B.to_vec(), ctx));
        assert_eq!(h.statuses(), vec![Success::Failure, Success::Success]);
        assert_eq!(
            h.data_out()[ComRetry::DEFAULT_NUM_RETRIES as usize + 1],
            DATA_B.to_vec()
        );
    }

    #[test]
    fn zero_retries_returns_on_the_first_failure() {
        let h = Harness::new();
        h.comp.configure(0);
        let ctx = FrameContext::default();
        h.receive_buffer(&DATA_A, &ctx);
        h.status(Success::Failure);
        assert_eq!(h.returned(), vec![(DATA_A.to_vec(), ctx)]);
        assert_eq!(h.statuses(), vec![Success::Failure]);
        assert_eq!(h.data_out().len(), 1);
    }

    #[test]
    fn the_returned_context_is_the_adapters() {
        let h = Harness::new();
        let mut sent = FrameContext::default();
        sent.set_com_queue_index(3);
        h.data_in(&DATA_A, &sent);
        // the adapter returns the buffer with an updated context
        let (buffer, mut ctx) = h.t.held.lock().unwrap().take().unwrap();
        assert_eq!(*ctx.get_com_queue_index(), 3);
        ctx.set_sequence_count(0x42);
        let p = h.comp.data_return_in(0);
        p.target.invoke(p.port_num, buffer, &ctx);
        h.status(Success::Success);
        assert_eq!(h.returned(), vec![(DATA_A.to_vec(), ctx)]);
    }

    /// A synchronous adapter in the `ComStub` shape: returns the buffer
    /// and reports a scripted status from inside `dataOut`.
    struct SyncAdapter {
        comp: Arc<ComRetry>,
        script: StdMutex<Vec<Success>>,
        sent: StdMutex<usize>,
    }

    impl ComDataWithContextPort for SyncAdapter {
        fn invoke(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            *self.sent.lock().unwrap() += 1;
            let ret = self.comp.data_return_in(0);
            ret.target.invoke(ret.port_num, data, context);
            let next = self.script.lock().unwrap().remove(0);
            let st = self.comp.com_status_in(0);
            let mut next = next;
            st.target.invoke(st.port_num, &mut next);
            // After a failure the adapter becomes ready again.
            if next == Success::Failure {
                let mut ready = Success::Success;
                st.target.invoke(st.port_num, &mut ready);
            }
        }
    }

    #[test]
    fn a_synchronous_adapter_reenters_from_data_out() {
        let comp = ComRetry::new("ComRetry");
        let t = Arc::new(Tester::default());
        let adapter = Arc::new(SyncAdapter {
            comp: comp.clone(),
            script: StdMutex::new(vec![Success::Failure, Success::Failure, Success::Success]),
            sent: StdMutex::new(0),
        });
        comp.data_out.connect(adapter.clone(), 0);
        comp.data_return_out
            .connect(Arc::new(DataReturnOut(t.clone())), 0);
        comp.com_status_out.connect(t.clone(), 0);
        let p = comp.data_in(0);
        let mut buffer = Buffer::allocate(4);
        buffer.data_mut().copy_from_slice(&DATA_B);
        p.target
            .invoke(p.port_num, buffer, &FrameContext::default());
        // two failures, two resends, then delivered
        assert_eq!(*adapter.sent.lock().unwrap(), 3);
        assert_eq!(t.data_return.lock().unwrap().len(), 1);
        assert_eq!(*t.status.lock().unwrap(), vec![Success::Success]);
        assert_eq!(comp.retry_state(), RetryState::WaitingForSend);
    }

    #[test]
    #[should_panic]
    fn status_before_the_buffer_returns_asserts() {
        let h = Harness::new();
        h.data_in(&DATA_A, &FrameContext::default());
        h.status(Success::Success);
    }

    #[test]
    #[should_panic]
    fn a_second_send_while_in_flight_asserts() {
        let h = Harness::new();
        h.receive_buffer(&DATA_A, &FrameContext::default());
        h.data_in(&DATA_B, &FrameContext::default());
    }

    #[test]
    #[should_panic]
    fn a_return_without_a_send_asserts() {
        let h = Harness::new();
        let p = h.comp.data_return_in(0);
        p.target
            .invoke(p.port_num, Buffer::allocate(1), &FrameContext::default());
    }

    #[test]
    #[should_panic]
    fn failure_while_retrying_asserts() {
        let h = Harness::new();
        h.receive_buffer(&DATA_A, &FrameContext::default());
        h.status(Success::Failure);
        h.status(Success::Failure);
    }

    #[test]
    #[should_panic]
    fn an_empty_kept_buffer_asserts_on_success() {
        let h = Harness::new();
        h.receive_buffer(&[], &FrameContext::default());
        h.status(Success::Success);
    }

    #[test]
    fn retry_state_discriminants_match_cpp() {
        assert_eq!(RetryState::WaitingForStatus as u8, 0);
        assert_eq!(RetryState::WaitingForSend as u8, 1);
        assert_eq!(RetryState::Retrying as u8, 2);
    }
}
