//! The `Demo.Console` implementation: the passive far side of the `Ops`
//! topology's special ports (command registration and responses, events,
//! time, parameters), the sink of the packetizer's packets and the source
//! of scheduler ticks. It records what it receives for the tests.

use crate::generated::Demo::{ConsoleBase, ConsoleHandlers};
use fprime_config::{FwEventIdType, FwIndexType, FwOpcodeType, FwPrmIdType};
use fprime_fw::{
    CmdResponse, ComBuffer, LogBuffer, LogSeverity, ParamBuffer, ParamValid, SerBuf, TextLogString,
    Time,
};
use std::sync::{Arc, Mutex};

/// Everything the console saw.
#[derive(Debug, Default)]
pub struct ConsoleLog {
    /// Registered opcodes.
    pub regs: Vec<FwOpcodeType>,
    /// Command responses.
    pub responses: Vec<(FwOpcodeType, u32, CmdResponse)>,
    /// Events (id, severity).
    pub events: Vec<(FwEventIdType, LogSeverity)>,
    /// Telemetry packets (port, bytes).
    pub packets: Vec<(FwIndexType, Vec<u8>)>,
    /// Parameter reads (id).
    pub prm_gets: Vec<FwPrmIdType>,
}

/// The console.
pub struct Console {
    base: ConsoleBase,
    /// The object name it was built with.
    pub name: String,
    /// The log.
    pub log: Mutex<ConsoleLog>,
}

impl Console {
    /// Construct with every port unconnected.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: ConsoleBase::new(name),
            name: name.to_string(),
            log: Mutex::new(ConsoleLog::default()),
        })
    }

    /// Send a command through `cmdOut` port `port` (registered opcodes
    /// fan out by the matched port number).
    pub fn send_command(&self, port: FwIndexType, op_code: FwOpcodeType, cmd_seq: u32) {
        let mut args = fprime_fw::CmdArgBuffer::new();
        self.base.cmd_out_out(port, op_code, cmd_seq, &mut args);
    }

    /// Tick scheduler output `port`.
    pub fn tick(&self, port: FwIndexType, context: u32) {
        self.base.sched_out_out(port, context);
    }
}

impl ConsoleHandlers for Console {
    fn base(&self) -> &ConsoleBase {
        &self.base
    }

    fn cmd_reg_in_handler(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
        self.log.lock().unwrap().regs.push(op_code);
    }

    fn cmd_resp_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        self.log
            .lock()
            .unwrap()
            .responses
            .push((op_code, cmd_seq, response));
    }

    fn log_in_handler(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        severity: LogSeverity,
        _args: &mut LogBuffer,
    ) {
        self.log.lock().unwrap().events.push((id, severity));
    }

    fn text_log_in_handler(
        &self,
        _port_num: FwIndexType,
        _id: FwEventIdType,
        _time_tag: &mut Time,
        _severity: LogSeverity,
        _text: &mut TextLogString,
    ) {
    }

    fn time_in_handler(&self, _port_num: FwIndexType, time: &mut Time) {
        *time = Time::new(fprime_fw::TimeBase::TbWorkstationTime, 0, 100, 0);
    }

    fn prm_get_in_handler(
        &self,
        _port_num: FwIndexType,
        id: FwPrmIdType,
        _val: &mut ParamBuffer,
    ) -> ParamValid {
        self.log.lock().unwrap().prm_gets.push(id);
        ParamValid::Invalid
    }

    fn prm_set_in_handler(&self, _port_num: FwIndexType, _id: FwPrmIdType, _val: &mut ParamBuffer) {
    }

    fn pkt_in_handler(&self, port_num: FwIndexType, data: &mut ComBuffer, _context: u32) {
        self.log
            .lock()
            .unwrap()
            .packets
            .push((port_num, data.as_slice().to_vec()));
    }
}

crate::generated::Demo::impl_console_component!(Console);
