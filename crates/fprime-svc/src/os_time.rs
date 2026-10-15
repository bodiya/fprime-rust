//! # OsTime — port of `Svc::OsTime` (passive time source)
//!
//! C++ sources: `Svc/OsTime/OsTime.{cpp,hpp,fpp}`, `Svc/OsTime/docs/sdd.md`,
//! `Svc/Ports/OsTimeEpoch/OsTimeEpoch.fpp`.
//!
//! Implements `Svc.Time` (`timeGetPort`) on `Os::RawTime`: an *epoch* pairs
//! an `Fw::Time` with the `Os::RawTime` of the same instant, and the time
//! returned is the epoch's `Fw::Time` plus the raw time elapsed since the
//! epoch's raw time (the time base and context are the epoch's). Until an
//! epoch is set — with [`OsTime::set_epoch`] at setup, the `setEpoch` port,
//! or the `SetCurrentTime` command (`seconds_now` paired with the raw time
//! now) — and whenever a raw-time call fails, the port returns
//! `Fw::ZERO_TIME`. A failed `SetCurrentTime` raw-time read emits
//! `SetCurrentTimeError` and answers `EXECUTION_ERROR` without touching the
//! epoch.
//!
//! The raw time is `fprime_os::RawTime` (the host's realtime clock with
//! `std`, the registered `fprime_os::platform::Platform` clock without),
//! so OsTime is available in `no_std` builds; with no platform registered
//! `RawTime::now` fails (`NOT_SUPPORTED`), which is the zero-time path.
//! The elapsed time is `RawTime::getTimeInterval`, i.e. the absolute
//! difference, as upstream.
//!
//! Upstream's unit tests swap the whole `Os::RawTime` implementation at
//! link time (`RawTimeTester`); the tests here install a scripted clock
//! through a test-only hook instead.

#[cfg(test)]
use alloc::boxed::Box;
use fprime_comp::{CmdGlue, CmdPort, EventGlue, PassiveBase, PortRef, TimePort};
use fprime_config::{FwEventIdType, FwIndexType, FwOpcodeType};
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{CmdArgBuffer, CmdResponse, LogSeverity, SerBuf, Time, TimeInterval};
use fprime_os::{RawTime, rawtime::Status as RawTimeStatus};

/// `Svc.OsTimeEpoch`: `port OsTimeEpoch(fw_time: Fw.Time, os_time:
/// Os.RawTime)` — a pair of timestamps naming the same instant.
pub trait OsTimeEpochPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, fw_time: Time, os_time: RawTime);
}

fprime_comp::serializable_port! {
    port: OsTimeEpochPort;
    args { val fw_time: Time, val os_time: RawTime }
}

/// The epoch, guarded by `m_epoch_lock`.
#[derive(Clone, Copy)]
struct Epoch {
    /// `m_epoch_fw_time`.
    fw_time: Time,
    /// `m_epoch_os_time`.
    os_time: RawTime,
    /// `m_epoch_valid`.
    valid: bool,
}

/// A scripted raw clock for the unit tests (upstream's `RawTimeTester`).
#[cfg(test)]
type TestClock = Box<dyn Fn(&mut RawTime) -> RawTimeStatus + Send + Sync>;

/// `Svc::OsTime`.
pub struct OsTime {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `CmdReg` / `CmdStatus` ports.
    pub cmd: CmdGlue,
    /// `EventOut` / `LogText` / `timeCaller` ports.
    pub evt: EventGlue,
    /// `m_epoch_*` under `m_epoch_lock`.
    epoch: Mutex<Epoch>,
    #[cfg(test)]
    test_clock: Mutex<Option<TestClock>>,
}

impl OsTime {
    /// `SetCurrentTime` opcode.
    pub const OPCODE_SET_CURRENT_TIME: FwOpcodeType = 0x00;
    /// `SetCurrentTimeError` event id (WARNING_HI).
    pub const EVENTID_SET_CURRENT_TIME_ERROR: FwEventIdType = 0x00;

    /// Construct with no epoch (C++ constructor: `ZERO_TIME`, a zero raw
    /// time, not valid).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            epoch: Mutex::new(Epoch {
                fw_time: Time::ZERO,
                os_time: RawTime::new(),
                valid: false,
            }),
            #[cfg(test)]
            test_clock: Mutex::new(None),
        })
    }

    fn id_base(&self) -> u32 {
        self.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_SET_CURRENT_TIME]);
    }

    /// C++ `set_epoch`: `fw_time` and `os_time` name the same instant;
    /// later readings are offset from them.
    pub fn set_epoch(&self, fw_time: &Time, os_time: &RawTime) {
        *self.epoch.lock() = Epoch {
            fw_time: *fw_time,
            os_time: *os_time,
            valid: true,
        };
    }

    /// `timeGetPort` — SYNC `Fw.Time` input.
    pub fn time_get_port_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn TimePort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `setEpoch` — SYNC `Svc.OsTimeEpoch` input.
    pub fn set_epoch_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn OsTimeEpochPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `CmdDisp` — command input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `Os::RawTime::now` (or the unit tests' scripted clock).
    fn raw_now(&self, time: &mut RawTime) -> RawTimeStatus {
        #[cfg(test)]
        if let Some(clock) = self.test_clock.lock().as_ref() {
            return clock(time);
        }
        time.now()
    }

    #[cfg(test)]
    fn set_test_clock(&self, clock: TestClock) {
        *self.test_clock.lock() = Some(clock);
    }

    /// `timeGetPort_handler`.
    fn time_get_port_handler(&self, _port_num: FwIndexType, time: &mut Time) {
        // Copy the epoch inside the lock.
        let epoch = *self.epoch.lock();

        *time = Time::ZERO;
        if !epoch.valid {
            return;
        }

        let mut time_now = RawTime::new();
        if self.raw_now(&mut time_now) != RawTimeStatus::OpOk {
            return;
        }

        let mut elapsed = TimeInterval::new(0, 0);
        if time_now.get_time_interval(&epoch.os_time, &mut elapsed) != RawTimeStatus::OpOk {
            return;
        }

        *time = epoch.fw_time;
        time.add_duration(elapsed.get_seconds(), elapsed.get_useconds());
    }

    /// `SetCurrentTime_cmdHandler`.
    fn set_current_time_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, seconds_now: u32) {
        let mut time_now = RawTime::new();
        let stat = self.raw_now(&mut time_now);
        if stat != RawTimeStatus::OpOk {
            self.log_warning_hi_set_current_time_error(stat as i32 as u32);
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        *self.epoch.lock() = Epoch {
            fw_time: Time::from_seconds_useconds(seconds_now, 0),
            os_time: time_now,
            valid: true,
        };
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `log_WARNING_HI_SetCurrentTimeError(status: U32)`.
    fn log_warning_hi_set_current_time_error(&self, status: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_SET_CURRENT_TIME_ERROR,
            LogSeverity::WarningHi,
            &alloc::format!("Could not set current time due to RawTime error status {status}"),
            |buf| buf.serialize_u32_be(status),
        );
    }

    /// The generated `SetCurrentTime` argument decoding.
    fn set_current_time_cmd(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut seconds_now: u32 = 0;
        if !args.deserialize_u32_be(&mut seconds_now).is_ok() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        // C++ parity (FW_CMD_CHECK_RESIDUAL).
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.set_current_time_cmd_handler(op_code, cmd_seq, seconds_now);
    }
}

impl TimePort for OsTime {
    fn invoke(&self, port_num: FwIndexType, time: &mut Time) {
        self.time_get_port_handler(port_num, time);
    }
}

/// `setEpoch_handler`.
impl OsTimeEpochPort for OsTime {
    fn invoke(&self, _port_num: FwIndexType, fw_time: Time, os_time: RawTime) {
        self.set_epoch(&fw_time, &os_time);
    }
}

impl CmdPort for OsTime {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_SET_CURRENT_TIME => self.set_current_time_cmd(op_code, cmd_seq, args),
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/OsTime/test/ut` (`OsTimeTester`) over a scripted raw
    //! clock (upstream's `RawTimeTester`: `setNowTime` / `setNowStatus`).
    use super::*;
    use fprime_comp::{CmdResponsePort, LogPort};
    use fprime_fw::{LogBuffer, TimeBase};
    use std::sync::Mutex as StdMutex;
    use std::vec::Vec;

    const ID_BASE: u32 = 0x4500;

    /// `RawTimeTester`'s static state, per test.
    struct RawClock {
        now: StdMutex<(u64, u32)>,
        status: StdMutex<RawTimeStatus>,
    }

    impl RawClock {
        /// `setNowTime(Fw::Time(seconds, useconds))`.
        fn set_now(&self, seconds: u64, useconds: u32) {
            *self.now.lock().unwrap() = (seconds, useconds);
        }
        fn set_status(&self, status: RawTimeStatus) {
            *self.status.lock().unwrap() = status;
        }
    }

    #[derive(Default)]
    struct Sink {
        responses: StdMutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: StdMutex<Vec<(FwEventIdType, LogSeverity, u32)>>,
    }

    impl CmdResponsePort for Sink {
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

    impl LogPort for Sink {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            let mut status = 0u32;
            assert!(args.deserialize_u32_be(&mut status).is_ok());
            assert_eq!(args.deserialize_size_left(), 0);
            self.events.lock().unwrap().push((id, severity, status));
        }
    }

    struct Harness {
        comp: Arc<OsTime>,
        clock: Arc<RawClock>,
        sink: Arc<Sink>,
    }

    impl Harness {
        fn new() -> Self {
            let comp = OsTime::new("OsTime");
            comp.base.set_id_base(ID_BASE);
            let clock = Arc::new(RawClock {
                now: StdMutex::new((0, 0)),
                status: StdMutex::new(RawTimeStatus::OpOk),
            });
            let script = clock.clone();
            comp.set_test_clock(Box::new(move |time: &mut RawTime| {
                let status = *script.status.lock().unwrap();
                if status != RawTimeStatus::OpOk {
                    return status;
                }
                let (seconds, useconds) = *script.now.lock().unwrap();
                *time = RawTime::from_parts(seconds, useconds * 1000);
                RawTimeStatus::OpOk
            }));
            let sink = Arc::new(Sink::default());
            comp.cmd.cmd_response_out.connect(sink.clone(), 0);
            comp.evt.log_out.connect(sink.clone(), 0);
            Self { comp, clock, sink }
        }

        fn get_time(&self) -> Time {
            let p = self.comp.time_get_port_in(0);
            let mut time = Time::new(TimeBase::TbWorkstationTime, 3, 99, 99);
            p.target.invoke(p.port_num, &mut time);
            time
        }

        /// `epoch_os_time.now()` against the scripted clock.
        fn raw_now(&self) -> RawTime {
            let mut raw = RawTime::new();
            assert_eq!(self.comp.raw_now(&mut raw), RawTimeStatus::OpOk);
            raw
        }

        fn send_set_current_time(&self, seconds_now: u32) {
            let mut args = CmdArgBuffer::new();
            assert!(args.serialize_u32_be(seconds_now).is_ok());
            self.send_cmd(ID_BASE + OsTime::OPCODE_SET_CURRENT_TIME, &mut args);
        }

        fn send_cmd(&self, op_code: FwOpcodeType, args: &mut CmdArgBuffer) {
            let p = self.comp.cmd_in(0);
            p.target.invoke(p.port_num, op_code, 0, args);
        }
    }

    /// Exact comparison, including the time base (`Fw::Time ==` compares
    /// base, seconds and microseconds).
    fn assert_time(actual: Time, base: TimeBase, context: u8, seconds: u32, useconds: u32) {
        assert_eq!(
            (
                actual.get_time_base(),
                actual.get_context(),
                actual.get_seconds(),
                actual.get_useconds()
            ),
            (base, context, seconds, useconds)
        );
    }

    #[test]
    fn zero_epoch() {
        // TEST(Nominal, ZeroEpochTest)
        let h = Harness::new();
        h.clock.set_now(0, 0);
        let epoch = h.raw_now();
        h.comp.set_epoch(&Time::ZERO, &epoch);
        for (seconds, useconds) in [(0, 200_000), (0, 400_000), (1, 200_000), (123_456, 987_654)] {
            h.clock.set_now(seconds, useconds);
            assert_time(h.get_time(), TimeBase::TbNone, 0, seconds as u32, useconds);
        }
    }

    #[test]
    fn offset_epoch() {
        // TEST(Nominal, OffsetEpochTest)
        let h = Harness::new();
        h.clock.set_now(0, 0);
        let epoch = h.raw_now();
        h.comp
            .set_epoch(&Time::from_seconds_useconds(12345, 800_000), &epoch);
        h.clock.set_now(0, 200_000);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 12346, 0);
    }

    #[test]
    fn no_epoch() {
        // TEST(Nominal, NoEpochTest): ZERO_TIME (TB_NONE, context 0)
        // replaces whatever the caller passed in.
        let h = Harness::new();
        h.clock.set_now(0, 0);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 0, 0);
        h.clock.set_now(0, 200_000);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 0, 0);
    }

    #[test]
    fn update_epoch() {
        // TEST(Nominal, UpdateEpochTest)
        let h = Harness::new();
        h.clock.set_now(0, 0);
        let epoch = h.raw_now();
        h.comp.set_epoch(&Time::ZERO, &epoch);
        h.clock.set_now(0, 200_000);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 0, 200_000);

        // Change the time base through the port.
        let new_base = Time::new(TimeBase::TbWorkstationTime, 7, 1234, 0);
        let epoch = h.raw_now();
        let p = h.comp.set_epoch_in(0);
        p.target.invoke(p.port_num, new_base, epoch);
        assert_time(h.get_time(), TimeBase::TbWorkstationTime, 7, 1234, 0);

        h.clock.set_now(0, 600_000);
        assert_time(h.get_time(), TimeBase::TbWorkstationTime, 7, 1234, 400_000);
    }

    #[test]
    fn set_current_time() {
        // TEST(Nominal, SetCurrentTimeTest)
        let h = Harness::new();
        h.clock.set_status(RawTimeStatus::OpOk);
        h.clock.set_now(0, 0);
        h.send_set_current_time(5000);
        assert_eq!(
            *h.sink.responses.lock().unwrap(),
            vec![(
                ID_BASE + OsTime::OPCODE_SET_CURRENT_TIME,
                0,
                CmdResponse::Ok
            )]
        );
        assert!(h.sink.events.lock().unwrap().is_empty());
        assert_time(h.get_time(), TimeBase::TbNone, 0, 5000, 0);
        h.clock.set_now(0, 200_000);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 5000, 200_000);
    }

    #[test]
    fn set_current_time_error() {
        // TEST(OffNominal, SetCurrentTimeErrorTest)
        let h = Harness::new();
        h.clock.set_now(0, 0);
        h.clock.set_status(RawTimeStatus::OtherError);
        h.send_set_current_time(5000);
        assert_eq!(
            *h.sink.responses.lock().unwrap(),
            vec![(
                ID_BASE + OsTime::OPCODE_SET_CURRENT_TIME,
                0,
                CmdResponse::ExecutionError
            )]
        );
        assert_eq!(
            *h.sink.events.lock().unwrap(),
            vec![(
                ID_BASE + OsTime::EVENTID_SET_CURRENT_TIME_ERROR,
                LogSeverity::WarningHi,
                RawTimeStatus::OtherError as u32
            )]
        );
        // A failed command must not establish an epoch.
        h.clock.set_status(RawTimeStatus::OpOk);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 0, 0);
    }

    #[test]
    fn a_failing_raw_clock_reads_as_zero_time() {
        // SVC-OS-TIME-002
        let h = Harness::new();
        h.clock.set_now(10, 0);
        let epoch = h.raw_now();
        h.comp
            .set_epoch(&Time::new(TimeBase::TbProcTime, 1, 50, 0), &epoch);
        h.clock.set_now(11, 0);
        assert_time(h.get_time(), TimeBase::TbProcTime, 1, 51, 0);
        h.clock.set_status(RawTimeStatus::NotSupported);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 0, 0);
    }

    #[test]
    fn a_raw_clock_behind_the_epoch_counts_forward() {
        // C++ parity: getTimeInterval is the absolute difference.
        let h = Harness::new();
        h.clock.set_now(100, 0);
        let epoch = h.raw_now();
        h.comp
            .set_epoch(&Time::from_seconds_useconds(1000, 0), &epoch);
        h.clock.set_now(98, 500_000);
        assert_time(h.get_time(), TimeBase::TbNone, 0, 1001, 500_000);
    }

    #[test]
    fn command_format_errors_and_bad_opcodes() {
        let h = Harness::new();
        let opcode = ID_BASE + OsTime::OPCODE_SET_CURRENT_TIME;
        let mut short = CmdArgBuffer::new();
        assert!(short.serialize_u16_be(1).is_ok());
        h.send_cmd(opcode, &mut short);
        let mut long = CmdArgBuffer::new();
        assert!(long.serialize_u32_be(1).is_ok());
        assert!(long.serialize_u8_be(1).is_ok());
        h.send_cmd(opcode, &mut long);
        h.send_cmd(ID_BASE + 1, &mut CmdArgBuffer::new());
        assert_eq!(
            *h.sink.responses.lock().unwrap(),
            vec![
                (opcode, 0, CmdResponse::FormatError),
                (opcode, 0, CmdResponse::FormatError),
                (ID_BASE + 1, 0, CmdResponse::InvalidOpcode),
            ]
        );
        // no epoch was set
        assert_time(h.get_time(), TimeBase::TbNone, 0, 0, 0);
    }

    #[test]
    fn reg_commands_registers_the_opcode() {
        use fprime_comp::CmdRegPort;
        #[derive(Default)]
        struct Reg(StdMutex<Vec<FwOpcodeType>>);
        impl CmdRegPort for Reg {
            fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
                self.0.lock().unwrap().push(op_code);
            }
        }
        let h = Harness::new();
        let reg = Arc::new(Reg::default());
        h.comp.cmd.cmd_reg_out.connect(reg.clone(), 0);
        h.comp.reg_commands();
        assert_eq!(*reg.0.lock().unwrap(), vec![ID_BASE]);
    }

    #[test]
    fn the_real_raw_clock_drives_the_port() {
        crate::test_platform::install();
        let comp = OsTime::new("OsTime");
        let mut epoch = RawTime::new();
        assert_eq!(epoch.now(), RawTimeStatus::OpOk);
        comp.set_epoch(&Time::new(TimeBase::TbWorkstationTime, 0, 1_000, 0), &epoch);
        let p = comp.time_get_port_in(0);
        let mut time = Time::ZERO;
        p.target.invoke(p.port_num, &mut time);
        assert_eq!(time.get_time_base(), TimeBase::TbWorkstationTime);
        // well under a minute has passed
        assert!((1_000..1_060).contains(&time.get_seconds()));
    }

    #[test]
    fn the_epoch_port_serializes_like_the_autocoder() {
        use fprime_comp::serial_port::SerializablePort;
        // Fw.Time (11) + Os.RawTime (8)
        assert_eq!(<dyn OsTimeEpochPort as SerializablePort>::CAPACITY, 19);
    }
}
