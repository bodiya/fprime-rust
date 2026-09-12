//! The `Demo.Ground` implementation: a passive stub on the far side of
//! every one of the sensor's ports. It records everything it receives so
//! the tests can check the generated code's behavior byte for byte, and
//! answers parameter reads and buffer requests.

use crate::generated::Demo::{GroundBase, GroundHandlers, Reading};
use fprime_config::{
    FwChanIdType, FwDpIdType, FwEventIdType, FwIndexType, FwOpcodeType, FwPrmIdType, FwSizeType,
};
use fprime_fw::{
    Buffer, CmdResponse, Endianness, LengthMode, LogBuffer, LogSeverity, ParamBuffer, ParamValid,
    SerBuf, Success, TextLogString, Time, TlmBuffer,
};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Everything the ground stub saw.
#[derive(Debug, Default)]
pub struct GroundLog {
    pub regs: Vec<FwOpcodeType>,
    pub responses: Vec<(FwOpcodeType, u32, CmdResponse)>,
    pub events: Vec<(FwEventIdType, LogSeverity, Vec<u8>)>,
    pub texts: Vec<(FwEventIdType, String)>,
    pub tlm: Vec<(FwChanIdType, Vec<u8>)>,
    pub reports: Vec<(u32, Reading)>,
    pub ticks: Vec<FwIndexType>,
    /// Parameter values the stub answers with (id -> serialized value).
    pub prm_values: HashMap<FwPrmIdType, Vec<u8>>,
    pub prm_sets: Vec<(FwPrmIdType, Vec<u8>)>,
    pub dp_sent: Vec<(FwDpIdType, Vec<u8>)>,
    /// The time answered on the time port.
    pub time: Time,
    /// When true, buffer requests fail.
    pub starve: bool,
}

/// The ground stub.
pub struct Ground {
    base: GroundBase,
    /// The log (public so tests can prime and inspect it).
    pub log: Mutex<GroundLog>,
}

impl Ground {
    /// Construct with every port unconnected.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: GroundBase::new(name),
            log: Mutex::new(GroundLog::default()),
        })
    }

    /// The generated base (its output ports drive the sensor).
    pub fn base(&self) -> &GroundBase {
        &self.base
    }
}

impl GroundHandlers for Ground {
    fn base(&self) -> &GroundBase {
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
        args: &mut LogBuffer,
    ) {
        self.log
            .lock()
            .unwrap()
            .events
            .push((id, severity, args.as_slice().to_vec()));
    }

    fn text_log_in_handler(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        _severity: LogSeverity,
        text: &mut TextLogString,
    ) {
        self.log
            .lock()
            .unwrap()
            .texts
            .push((id, text.as_str().unwrap_or("").to_string()));
    }

    fn tlm_in_handler(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        _time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) {
        self.log
            .lock()
            .unwrap()
            .tlm
            .push((id, val.as_slice().to_vec()));
    }

    fn time_in_handler(&self, _port_num: FwIndexType, time: &mut Time) {
        *time = self.log.lock().unwrap().time;
    }

    fn prm_get_in_handler(
        &self,
        _port_num: FwIndexType,
        id: FwPrmIdType,
        val: &mut ParamBuffer,
    ) -> ParamValid {
        let log = self.log.lock().unwrap();
        match log.prm_values.get(&id) {
            Some(bytes) => {
                let status = val.serialize_bytes(bytes, LengthMode::OmitLength, Endianness::Big);
                assert!(status.is_ok());
                ParamValid::Valid
            }
            None => ParamValid::Invalid,
        }
    }

    fn prm_set_in_handler(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
        self.log
            .lock()
            .unwrap()
            .prm_sets
            .push((id, val.as_slice().to_vec()));
    }

    fn dp_get_in_handler(
        &self,
        _port_num: FwIndexType,
        _id: FwDpIdType,
        data_size: FwSizeType,
        buffer: &mut Buffer,
    ) -> Success {
        if self.log.lock().unwrap().starve {
            return Success::Failure;
        }
        *buffer = Buffer::allocate(data_size as usize);
        Success::Success
    }

    fn dp_send_in_handler(&self, _port_num: FwIndexType, id: FwDpIdType, buffer: Buffer) {
        self.log
            .lock()
            .unwrap()
            .dp_sent
            .push((id, buffer.data().to_vec()));
    }

    fn report_in_handler(&self, _port_num: FwIndexType, seq: u32, reading: &Reading) {
        self.log
            .lock()
            .unwrap()
            .reports
            .push((seq, reading.clone()));
    }

    fn tick_in_handler(&self, port_num: FwIndexType) {
        self.log.lock().unwrap().ticks.push(port_num);
    }
}

crate::generated::Demo::impl_ground_component!(Ground);
