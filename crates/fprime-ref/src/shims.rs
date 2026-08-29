//! # Byte-stream trait shims: `fprime_svc::com_stub` <-> `fprime_drv`
//!
//! `fprime-svc` and `fprime-drv` may not depend on each other (dependency
//! DAG, ARCHITECTURE.md), so each defines structurally-identical
//! byte-stream port traits (`fprime-drv`'s are canonical). This module is
//! the deployment-side bridge the two crates document: one tiny adapter
//! per crossing direction, each forwarding to a wired [`PortRef`] and
//! mapping the identical-discriminant [`ByteStreamStatus`] enums.
//!
//! Crossings in the reference topology:
//!
//! - driver `ready_out` -> [`DrvToSvcReadyShim`] -> `com_stub.drv_connected`
//! - driver `recv_out` -> [`DrvToSvcDataShim`] -> `com_stub.drv_receive_in`
//! - `com_stub.drv_send_out` -> [`SvcToDrvSendShim`] -> driver `send_in`
//! - `com_stub.drv_receive_return_out` -> driver `recv_return_in` needs NO
//!   shim: both sides are the framework `fprime_comp::BufferSendPort`.

use fprime_config::FwIndexType;
use fprime_fw::Buffer;

use fprime_comp::PortRef;
use fprime_drv::byte_stream as drv;
use fprime_svc::com_stub as svc;

/// Map the driver-side status to the service-side twin (identical
/// discriminants; an exhaustive match keeps the compiler checking that).
pub fn drv_to_svc_status(status: drv::ByteStreamStatus) -> svc::ByteStreamStatus {
    match status {
        drv::ByteStreamStatus::OpOk => svc::ByteStreamStatus::OpOk,
        drv::ByteStreamStatus::SendRetry => svc::ByteStreamStatus::SendRetry,
        drv::ByteStreamStatus::RecvNoData => svc::ByteStreamStatus::RecvNoData,
        drv::ByteStreamStatus::OtherError => svc::ByteStreamStatus::OtherError,
    }
}

/// Map the service-side status to the driver-side twin.
pub fn svc_to_drv_status(status: svc::ByteStreamStatus) -> drv::ByteStreamStatus {
    match status {
        svc::ByteStreamStatus::OpOk => drv::ByteStreamStatus::OpOk,
        svc::ByteStreamStatus::SendRetry => drv::ByteStreamStatus::SendRetry,
        svc::ByteStreamStatus::RecvNoData => drv::ByteStreamStatus::RecvNoData,
        svc::ByteStreamStatus::OtherError => drv::ByteStreamStatus::OtherError,
    }
}

/// ComStub `drvSendOut` (svc trait) -> driver `send_in` (drv trait).
pub struct SvcToDrvSendShim {
    /// The wired driver `send_in` port.
    pub target: PortRef<dyn drv::ByteStreamSendPort>,
}

impl svc::ByteStreamSendPort for SvcToDrvSendShim {
    fn invoke(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> svc::ByteStreamStatus {
        drv_to_svc_status(self.target.target.invoke(self.target.port_num, buffer))
    }
}

/// Driver `recv_out` (drv trait) -> ComStub `drvReceiveIn` (svc trait).
pub struct DrvToSvcDataShim {
    /// The wired ComStub `drv_receive_in` port.
    pub target: PortRef<dyn svc::ByteStreamDataPort>,
}

impl drv::ByteStreamDataPort for DrvToSvcDataShim {
    fn invoke(&self, _port_num: FwIndexType, buffer: Buffer, status: drv::ByteStreamStatus) {
        self.target
            .target
            .invoke(self.target.port_num, buffer, drv_to_svc_status(status));
    }
}

/// Driver `ready_out` (drv trait) -> ComStub `drvConnected` (svc trait).
pub struct DrvToSvcReadyShim {
    /// The wired ComStub `drv_connected` port.
    pub target: PortRef<dyn svc::ByteStreamReadyPort>,
}

impl drv::ByteStreamReadyPort for DrvToSvcReadyShim {
    fn invoke(&self, _port_num: FwIndexType) {
        self.target.target.invoke(self.target.port_num);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two status enums stay in lockstep (discriminant parity).
    #[test]
    fn status_maps_are_involutions() {
        for raw in 0u8..4 {
            let d = drv::ByteStreamStatus::try_from(raw).unwrap();
            assert_eq!(svc_to_drv_status(drv_to_svc_status(d)), d);
            assert_eq!(drv_to_svc_status(d) as u8, raw);
        }
    }
}
