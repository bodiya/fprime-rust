//! The `Demo.Sensor` implementation: what a developer writes against the
//! generated base. Everything the autocoder provides (ports, dispatch,
//! commands, events, telemetry, parameters, data products) comes from
//! `generated::Demo::{SensorBase, SensorHandlers, SensorComponent}`; this
//! file is only the handlers and the component's own state.

use crate::generated::Demo::{
    self, Counts, Frame, Level, Mode, Reading, Reading_samples_Array, SensorBase, SensorHandlers,
};
use fprime_config::{FwIndexType, FwOpcodeType, FwPrmIdType};
use fprime_fw::{Buffer, CmdResponse, FwString, SerializeStatus, Time};
use std::sync::{Arc, Mutex};

/// What the sensor remembers, so tests can observe the handlers.
#[derive(Debug, Default)]
pub struct SensorState {
    pub seq: u32,
    pub mode: Mode,
    pub gain: f32,
    pub label: FwString<8>,
    pub delivered: Vec<(usize, Time)>,
    pub measured: u32,
    pub recomputed: Vec<f32>,
    pub resets: Vec<u32>,
    pub pings: u32,
    pub param_updates: Vec<FwPrmIdType>,
    pub loaded: bool,
}

/// The sensor.
pub struct Sensor {
    base: SensorBase,
    state: Mutex<SensorState>,
}

impl Sensor {
    /// Construct with every port unconnected.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: SensorBase::new(name),
            state: Mutex::new(SensorState::default()),
        })
    }

    /// A snapshot of the state (for tests).
    pub fn state(&self) -> SensorState {
        let s = self.state.lock().unwrap();
        SensorState {
            seq: s.seq,
            mode: s.mode,
            gain: s.gain,
            label: s.label.clone(),
            delivered: s.delivered.clone(),
            measured: s.measured,
            recomputed: s.recomputed.clone(),
            resets: s.resets.clone(),
            pings: s.pings,
            param_updates: s.param_updates.clone(),
            loaded: s.loaded,
        }
    }

    /// Produce one `Samples` data product with a `Sample` record and a
    /// `Raw` record of `n` bytes; false if no buffer was available.
    pub fn produce(&self, n: u8) -> bool {
        let raw: Vec<u8> = (0..n).collect();
        let size = SensorBase::SIZE_OF_SAMPLE_RECORD
            + SensorBase::size_of_raw_record(raw.len() as fprime_config::FwSizeType);
        let Some(mut container) = self.base.dp_get_samples(size) else {
            return false;
        };
        let reading = Reading::default();
        assert_eq!(
            self.base.serialize_record_sample(&mut container, &reading),
            SerializeStatus::Ok
        );
        assert_eq!(
            self.base.serialize_record_raw(&mut container, &raw),
            SerializeStatus::Ok
        );
        self.base.dp_send(container);
        true
    }
}

impl SensorHandlers for Sensor {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    /// The rate-group tick: drain queued messages (commands, deliveries,
    /// internal requests), then report and write telemetry.
    fn tick_handler(&self, _port_num: FwIndexType, context: u32) {
        let _ = self.base.queued.dispatch_available_messages(self);
        let (seq, reading, gain, mode, label) = {
            let mut st = self.state.lock().unwrap();
            st.seq += 1;
            let reading = Reading::new(
                st.mode,
                Counts::fill(context as u16),
                Reading_samples_Array::new([1, 2]),
                st.label.clone(),
                true,
            );
            (st.seq, reading, st.gain, st.mode, st.label.clone())
        };
        self.base.report_out(0, seq, &reading);
        self.base.fanout_out(0);
        self.base.fanout_out(1);
        self.base.tlm_write_value(gain * context as f32);
        self.base.tlm_write_mode(mode);
        self.base.tlm_write_label(label.as_str().unwrap_or(""));
    }

    fn deliver_handler(&self, _port_num: FwIndexType, buffer: Buffer, time_tag: &mut Time) {
        self.state
            .lock()
            .unwrap()
            .delivered
            .push((buffer.size(), *time_tag));
    }

    fn measure_handler(
        &self,
        _port_num: FwIndexType,
        seq: Demo::Seq,
        mode: Mode,
        reading: &Reading,
        label: &FwString<8>,
        result: &mut Frame,
    ) -> Level {
        self.state.lock().unwrap().measured += 1;
        result.seq = seq;
        result.reading = reading.clone();
        result.reading.mode = mode;
        result.reading.label = label.clone();
        Level::HIGH
    }

    fn configure_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        mode: Mode,
        gain: f32,
        label: &FwString<8>,
    ) {
        {
            let mut st = self.state.lock().unwrap();
            st.mode = mode;
            st.gain = gain;
            st.label = label.clone();
        }
        self.base.log_activity_hi_configured(mode, gain);
        self.base
            .log_diagnostic_labelled(label.as_str().unwrap_or(""), &Reading::default());
        self.base
            .cmd
            .cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn ping_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.state.lock().unwrap().pings += 1;
        self.base
            .cmd
            .cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn reset_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, count: u32) {
        self.state.lock().unwrap().resets.push(count);
        if count > 3 {
            self.base.log_warning_lo_overrun(count);
        }
        self.base
            .cmd
            .cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn recompute_internal_interface_handler(&self, scale: f32) {
        self.state.lock().unwrap().recomputed.push(scale);
    }

    fn parameter_updated(&self, id: FwPrmIdType) {
        self.state.lock().unwrap().param_updates.push(id);
    }

    fn parameters_loaded(&self) {
        self.state.lock().unwrap().loaded = true;
    }
}

crate::generated::Demo::impl_sensor_component!(Sensor);
