//! # PassThroughRouter — port of `Svc::PassThroughRouter` (passive)
//!
//! C++ sources: `Svc/PassThroughRouter/PassThroughRouter.{cpp,hpp,fpp}`,
//! `Svc/PassThroughRouter/docs/sdd.md`.
//!
//! The single-route router: every packet received on `dataIn` goes out on
//! `allPacketsOut` with its frame context dropped (`Fw.BufferSend` carries
//! none), and every buffer coming back on `allPacketsReturnIn` is handed
//! upstream on `dataReturnOut` with the DEFAULT [`FrameContext`] — the
//! original context is not remembered (C++ parity; contrast
//! [`FprimeRouter`](crate::fprime_router), which keeps a context table).
//!
//! Ownership: the [`Buffer`] moves `dataIn -> allPacketsOut` and
//! `allPacketsReturnIn -> dataReturnOut`, the same path the C++ descriptor
//! takes, so the upstream allocator (the deframer's buffer manager) gets
//! its storage back.
//!
//! Both input ports are `sync` (no component mutex); the component has no
//! state. `allPacketsOut` and `dataReturnOut` are invoked unconditionally —
//! an unconnected one asserts (C++: "allPacketsOut must always be
//! connected. Users needing a null-router should write one explicitly").
//! The FPP model imports `Fw.Event` and a time port but declares no
//! events; [`PassThroughRouter::evt`] carries those ports for wiring
//! fidelity only.

use fprime_comp::{
    BufferSendPort, ComDataWithContextPort, EventGlue, OutputPort, PassiveBase, input_port_adapter,
};
use fprime_config::FwIndexType;
use fprime_fw::sync::Arc;
use fprime_fw::{Buffer, FrameContext};

/// `Svc::PassThroughRouter` — direct pass-through router.
pub struct PassThroughRouter {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `logOut` / `logTextOut` / `timeCaller` (the `Fw.Event` import; no
    /// events are declared).
    pub evt: EventGlue,
    /// `dataReturnOut` — returns ownership of `dataIn` buffers upstream.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `allPacketsOut` — every packet received on `dataIn`.
    pub all_packets_out: OutputPort<dyn BufferSendPort>,
}

impl PassThroughRouter {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            data_return_out: OutputPort::new(),
            all_packets_out: OutputPort::new(),
        })
    }

    /// `dataIn_handler`: forward the packet; its context is ignored.
    fn data_in_handler(
        &self,
        _port_num: FwIndexType,
        packet_buffer: Buffer,
        _context: &FrameContext,
    ) {
        // C++ parity: allPacketsOut must always be connected (get() asserts).
        let p = self.all_packets_out.get();
        p.target.invoke(0, packet_buffer);
    }

    /// `allPacketsReturnIn_handler`: return the buffer upstream with the
    /// default context ("used to satisfy dataReturnOut port interface").
    fn all_packets_return_in_handler(&self, _port_num: FwIndexType, fw_buffer: Buffer) {
        let context = FrameContext::default();
        let p = self.data_return_out.get();
        p.target.invoke(0, fw_buffer, &context);
    }
}

input_port_adapter! {
    /// `dataIn` — SYNC `Svc.ComDataWithContext` input: packets to route.
    component: PassThroughRouter;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `allPacketsReturnIn` — SYNC `Fw.BufferSend` input: buffers coming
    /// back from the `allPacketsOut` receiver.
    component: PassThroughRouter;
    adapter: AllPacketsReturnInAdapter;
    port: BufferSendPort;
    input: pub all_packets_return_in;
    handler: all_packets_return_in_handler;
    args { val buffer: Buffer }
}

#[cfg(test)]
mod tests {
    //! Ports of `Svc/PassThroughRouter/test/ut` (`testRouteAPacket`,
    //! `testAllPacketsReturn`) plus the ownership round trip.

    use super::*;
    use fprime_fw::Apid;
    use std::sync::Mutex as StdMutex;
    use std::vec::Vec;

    #[derive(Default)]
    struct Recorder {
        /// `allPacketsOut` invocations: (port, buffer).
        packets: StdMutex<Vec<(FwIndexType, Buffer)>>,
        /// `dataReturnOut` invocations: (port, buffer, context).
        returned: StdMutex<Vec<(FwIndexType, Buffer, FrameContext)>>,
    }

    impl BufferSendPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
            self.packets.lock().unwrap().push((port_num, buffer));
        }
    }

    impl ComDataWithContextPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            self.returned
                .lock()
                .unwrap()
                .push((port_num, data, *context));
        }
    }

    fn build() -> (Arc<PassThroughRouter>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let router = PassThroughRouter::new("PassThroughRouter");
        router.all_packets_out.connect(rec.clone(), 0);
        router.data_return_out.connect(rec.clone(), 0);
        (router, rec)
    }

    /// Upstream `mockReceivePacketType`: a buffer the size of the packet
    /// type, with the type as the context APID.
    fn receive_packet_type(router: &Arc<PassThroughRouter>, apid: Apid) {
        let buffer = Buffer::allocate(size_of::<u32>());
        let context = FrameContext {
            apid,
            ..FrameContext::default()
        };
        let port = router.data_in(0);
        port.target.invoke(port.port_num, buffer, &context);
    }

    #[test]
    fn route_a_packet() {
        // SVC-PASS_THROUGH_ROUTER-001.
        let (router, rec) = build();
        receive_packet_type(&router, Apid::FwPacketCommand);
        assert_eq!(rec.packets.lock().unwrap().len(), 1, "one packet emitted");
        assert!(rec.returned.lock().unwrap().is_empty());
    }

    #[test]
    fn every_packet_type_takes_the_single_route() {
        let (router, rec) = build();
        for apid in [
            Apid::FwPacketCommand,
            Apid::FwPacketFile,
            Apid::FwPacketTelem,
            Apid::FwPacketUnknown,
        ] {
            receive_packet_type(&router, apid);
        }
        let packets = rec.packets.lock().unwrap();
        assert_eq!(packets.len(), 4);
        assert!(packets.iter().all(|(port, _)| *port == 0));
    }

    #[test]
    fn all_packets_return() {
        // SVC-PASS_THROUGH_ROUTER-002: the returned buffer is the same
        // storage, with the default frame context.
        let (router, rec) = build();
        let mut buffer = Buffer::allocate(1);
        buffer.data_mut()[0] = 0x5A;
        let address = buffer.data().as_ptr();
        let port = router.all_packets_return_in(0);
        port.target.invoke(port.port_num, buffer);
        let returned = rec.returned.lock().unwrap();
        assert_eq!(returned.len(), 1);
        let (port_num, data, context) = &returned[0];
        assert_eq!(*port_num, 0);
        assert_eq!(data.data().as_ptr(), address);
        assert_eq!(data.size(), 1);
        assert_eq!(data.data(), &[0x5A]);
        assert_eq!(*context, FrameContext::default());
    }

    #[test]
    fn routed_buffer_round_trips_with_the_default_context() {
        // The context the packet arrived with is NOT restored on return
        // (C++ parity: PassThroughRouter keeps no context table).
        let (router, rec) = build();
        let mut buffer = Buffer::allocate(4);
        buffer.data_mut().copy_from_slice(&[1, 2, 3, 4]);
        let address = buffer.data().as_ptr();
        let context = FrameContext {
            apid: Apid::FwPacketFile,
            sequence_count: 7,
            ..FrameContext::default()
        };
        let port = router.data_in(0);
        port.target.invoke(port.port_num, buffer, &context);

        let (_, routed) = rec.packets.lock().unwrap().pop().unwrap();
        assert_eq!(routed.data().as_ptr(), address);
        assert_eq!(routed.data(), &[1, 2, 3, 4]);
        let port = router.all_packets_return_in(0);
        port.target.invoke(port.port_num, routed);

        let returned = rec.returned.lock().unwrap();
        assert_eq!(returned.len(), 1);
        assert_eq!(returned[0].1.data().as_ptr(), address);
        assert_eq!(returned[0].2, FrameContext::default());
        assert_ne!(returned[0].2, context);
    }

    #[test]
    #[should_panic]
    fn unconnected_all_packets_out_asserts() {
        let router = PassThroughRouter::new("router");
        let port = router.data_in(0);
        port.target
            .invoke(port.port_num, Buffer::allocate(1), &FrameContext::default());
    }
}
