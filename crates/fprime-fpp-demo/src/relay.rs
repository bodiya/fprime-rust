//! The `Demo.Relay` implementation: a hub in miniature. Each serial input
//! forwards the serialized call it receives, unchanged, on a serial output
//! (`serialIn[n]` -> `serialOut[n]`, `serialGuarded` -> `serialOut[2]`,
//! the queued `serialAsync` -> `serialOut[3]`), and records the status the
//! output answered, so the tests can see typed <-> serial conversions and
//! their deserialization errors.

use crate::generated::Demo::{RelayBase, RelayHandlers};
use fprime_config::FwIndexType;
use fprime_fw::{SerBufAny, SerializeStatus};
use std::sync::{Arc, Mutex};

/// What the relay saw.
#[derive(Debug, Default)]
pub struct RelayLog {
    /// (input port, port number, serialized bytes, status answered by the output).
    pub forwarded: Vec<(&'static str, FwIndexType, Vec<u8>, SerializeStatus)>,
    /// Serialized calls that overflowed the queue (port number, bytes).
    pub overflowed: Vec<(FwIndexType, Vec<u8>)>,
}

/// The relay.
pub struct Relay {
    base: RelayBase,
    /// The log (public so tests can inspect it).
    pub log: Mutex<RelayLog>,
}

impl Relay {
    /// Construct with every port unconnected.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: RelayBase::new(name),
            log: Mutex::new(RelayLog::default()),
        })
    }

    /// The generated base.
    pub fn base(&self) -> &RelayBase {
        &self.base
    }

    /// Dispatch every queued serial call (a queued component is drained by
    /// its user).
    pub fn drain(&self) {
        let _ = self.base.queued.dispatch_available_messages(self);
    }

    fn forward(
        &self,
        input: &'static str,
        port_num: FwIndexType,
        out: FwIndexType,
        buffer: &mut dyn SerBufAny,
    ) {
        let bytes = buffer.bytes()[..buffer.ser_loc()].to_vec();
        let status = self.base.serial_out_out(out, buffer);
        self.log
            .lock()
            .unwrap()
            .forwarded
            .push((input, port_num, bytes, status));
    }
}

impl RelayHandlers for Relay {
    fn base(&self) -> &RelayBase {
        &self.base
    }

    fn serial_in_handler(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) {
        self.forward("serialIn", port_num, port_num, buffer);
    }

    fn serial_guarded_handler(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) {
        self.forward("serialGuarded", port_num, 2, buffer);
    }

    fn serial_async_handler(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) {
        self.forward("serialAsync", port_num, 3, buffer);
    }

    fn serial_async_overflow_hook(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) {
        let bytes = buffer.bytes()[..buffer.ser_loc()].to_vec();
        self.log.lock().unwrap().overflowed.push((port_num, bytes));
    }
}

crate::generated::Demo::impl_relay_component!(Relay);
