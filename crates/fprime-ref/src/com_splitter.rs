//! # ComSplitter — PASSIVE `Fw.Com` stream tee
//!
//! Rust port of `Svc/ComSplitter/ComSplitter.{fpp,cpp}`, living in the
//! deployment crate rather than `fprime-svc` because `fprime-svc` is owned
//! by another wave (the file is a faithful port and should move there when
//! that crate is next touched).
//!
//! The C++ Ref-family deployments use it inside the `ComLoggerTee`
//! subtopology (`Svc/Subtopologies/ComLoggerTee/subtopology-template.fppi`)
//! to fan a `Fw.Com` packet stream out to both the normal downlink and
//! `Svc::ComLogger`. This deployment does the same for the event stream:
//!
//! ```text
//! events.PktSend -> comSplitter.comIn
//!                   comSplitter.comOut[0] -> comQueue.comPacketQueueIn[EVENTS]
//!                   comSplitter.comOut[1] -> comLog.comIn
//! ```
//!
//! C++ parity notes, both load-bearing:
//!
//! - the handler copies the `ComBuffer` per output port (C++ makes an
//!   explicit copy because the port passes by reference);
//! - the outgoing `context` argument is **0**, not the incoming one
//!   (`comOut_out(i, dataToSend, 0)` in `ComSplitter.cpp`);
//! - unconnected output ports are skipped (`isConnected_comOut_OutputPort`).

use std::sync::Arc;

use fprime_comp::{ComPort, OutputPort, PassiveBase, PortRef};
use fprime_config::FwIndexType;
use fprime_fw::{ComBuffer, fw_assert};

/// `comOut` port-array width (C++ `output port comOut: [5] Fw.Com`).
pub const COM_OUT_PORTS: usize = 5;

/// `Svc::ComSplitter` — passive component, no commands/events/telemetry.
pub struct ComSplitter {
    /// Passive core (object name + id base).
    pub base: PassiveBase,
    /// `comOut` — the fan-out ports.
    pub com_out: [OutputPort<dyn ComPort>; COM_OUT_PORTS],
}

impl ComSplitter {
    /// Construct (topology phase 1).
    #[must_use]
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            com_out: std::array::from_fn(|_| OutputPort::new()),
        })
    }

    /// `comIn` — SYNC `Fw.Com` input; the component implements the trait, so
    /// the handler runs on the caller's thread (C++ `sync input port`).
    pub fn com_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComPort> {
        PortRef::new(self.clone(), port_num)
    }
}

impl ComPort for ComSplitter {
    fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, _context: u32) {
        // C++ FW_ASSERT(portNum == 0).
        fw_assert!(port_num == 0, port_num);
        for port in &self.com_out {
            if let Some(p) = port.try_get() {
                // C++ copies the buffer per port; context is always 0.
                let mut copy = data.clone();
                p.target.invoke(p.port_num, &mut copy, 0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::SerBuf;
    use std::sync::Mutex;

    #[derive(Default)]
    struct Sink {
        packets: Mutex<Vec<(Vec<u8>, u32)>>,
    }

    impl ComPort for Sink {
        fn invoke(&self, _port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
            self.packets
                .lock()
                .unwrap()
                .push((data.as_slice().to_vec(), context));
        }
    }

    /// Every connected port sees an independent copy, with context 0.
    #[test]
    fn tees_to_every_connected_port_with_context_zero() {
        let splitter = ComSplitter::new("comSplitter");
        let a = Arc::new(Sink::default());
        let b = Arc::new(Sink::default());
        splitter.com_out[0].connect(a.clone(), 0);
        splitter.com_out[2].connect(b.clone(), 0);

        let mut packet = ComBuffer::new();
        assert!(packet.serialize_u32_be(0xDEAD_BEEF).is_ok());
        let port = splitter.com_in(0);
        port.target.invoke(port.port_num, &mut packet, 7);

        let expected = vec![0xDE, 0xAD, 0xBE, 0xEF];
        assert_eq!(*a.packets.lock().unwrap(), vec![(expected.clone(), 0)]);
        assert_eq!(*b.packets.lock().unwrap(), vec![(expected, 0)]);
        // The caller's buffer is untouched (the copies are per port).
        assert_eq!(packet.as_slice(), &[0xDE, 0xAD, 0xBE, 0xEF]);
    }

    /// Unconnected ports are skipped rather than asserting.
    #[test]
    fn unconnected_ports_are_skipped() {
        let splitter = ComSplitter::new("comSplitter");
        let mut packet = ComBuffer::new();
        assert!(packet.serialize_u8_be(1).is_ok());
        let port = splitter.com_in(0);
        port.target.invoke(port.port_num, &mut packet, 0);
    }
}
