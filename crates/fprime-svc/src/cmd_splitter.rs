//! # CmdSplitter — port of `Svc::CmdSplitter` (passive)
//!
//! C++ sources: `Svc/CmdSplitter/CmdSplitter.{cpp,hpp,fpp}`,
//! `Svc/CmdSplitter/docs/sdd.md`.
//!
//! Routes uplinked command packets to a local or a remote command
//! dispatcher by opcode: opcodes below the configured remote base go out
//! on `LocalCmd`, the others on `RemoteCmd` (the hub pattern: `RemoteCmd`
//! usually feeds a `GenericHub`'s `cmdDispIn`). A buffer that does not
//! deserialize as a command packet goes local, so the local dispatcher
//! reports it. Command responses coming back on `seqCmdStatus` are
//! forwarded on `forwardSeqCmdStatus`. Every port is an array of
//! [`CMD_SPLITTER_PORTS`] mirroring the command dispatcher's sequence
//! ports, and a call on index `i` leaves on index `i`.
//!
//! C++ parity kept: the packet is deserialized out of the caller's
//! `ComBuffer` itself, so its read cursor is left past the opcode and
//! arguments when it is forwarded (downstream consumers serialize or
//! re-read the whole buffer, as upstream's do); unconfigured, the remote
//! base is `FwOpcodeType::MAX`, so everything but opcode `0xFFFF_FFFF`
//! routes local; both output arrays are required at the index used.

use fprime_comp::{CmdResponsePort, ComPort, OutputPort, PassiveBase, PortRef};
pub use fprime_config::CMD_SPLITTER_PORTS;
use fprime_config::{FwIndexType, FwOpcodeType};
use fprime_fw::sync::{Arc, AtomicU32, Ordering};
use fprime_fw::{CmdPacket, CmdResponse, ComBuffer, Deserialize, Endianness, fw_assert};

/// `Svc::CmdSplitter`.
pub struct CmdSplitter {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `forwardSeqCmdStatus` — command responses back to the command
    /// sources (`Fw.CmdResponse`).
    pub forward_seq_cmd_status: [OutputPort<dyn CmdResponsePort>; CMD_SPLITTER_PORTS],
    /// `LocalCmd` — commands for the local dispatcher (`Fw.Com`).
    pub local_cmd: [OutputPort<dyn ComPort>; CMD_SPLITTER_PORTS],
    /// `RemoteCmd` — commands for the remote dispatcher (`Fw.Com`).
    pub remote_cmd: [OutputPort<dyn ComPort>; CMD_SPLITTER_PORTS],
    /// `m_remoteBase`: opcodes at or above it route remote. Written by
    /// `configure` at setup, read on every command (a plain member in C++;
    /// a relaxed atomic here).
    remote_base: AtomicU32,
}

impl CmdSplitter {
    /// Construct with the remote base at `FwOpcodeType::MAX` (C++
    /// constructor).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            forward_seq_cmd_status: core::array::from_fn(|_| OutputPort::new()),
            local_cmd: core::array::from_fn(|_| OutputPort::new()),
            remote_cmd: core::array::from_fn(|_| OutputPort::new()),
            remote_base: AtomicU32::new(FwOpcodeType::MAX),
        })
    }

    /// C++ `configure(remoteBaseOpcode)`: opcodes greater than or equal to
    /// `remote_base_opcode` route remotely.
    pub fn configure(&self, remote_base_opcode: FwOpcodeType) {
        self.remote_base
            .store(remote_base_opcode, Ordering::Relaxed);
    }

    /// The configured remote base opcode.
    pub fn remote_base(&self) -> FwOpcodeType {
        self.remote_base.load(Ordering::Relaxed)
    }

    /// `CmdBuff` — SYNC `Fw.Com` input array (`0..CMD_SPLITTER_PORTS`).
    pub fn cmd_buff_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `seqCmdStatus` — SYNC `Fw.CmdResponse` input array
    /// (`0..CMD_SPLITTER_PORTS`).
    pub fn seq_cmd_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn CmdResponsePort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `CmdBuff_handler`.
    fn cmd_buff_handler(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        let mut cmd_pkt = CmdPacket::new();
        let stat = cmd_pkt.deserialize_from(data, Endianness::Big);

        assert_port(port_num);
        let index = port_num as usize;
        let out = if !stat.is_ok() || cmd_pkt.get_opcode() < self.remote_base() {
            &self.local_cmd[index]
        } else {
            &self.remote_cmd[index]
        };
        let p = out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// `seqCmdStatus_handler`.
    fn seq_cmd_status_handler(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        assert_port(port_num);
        let p = self.forward_seq_cmd_status[port_num as usize].get();
        p.target.invoke(p.port_num, op_code, cmd_seq, response);
    }
}

/// C++ `FW_ASSERT(portNum < CmdSplitterPorts)` (a negative index is
/// rejected too: it could not index the output arrays).
fn assert_port(port_num: FwIndexType) {
    fw_assert!(
        port_num >= 0 && (port_num as usize) < CMD_SPLITTER_PORTS,
        port_num
    );
}

impl ComPort for CmdSplitter {
    fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        self.cmd_buff_handler(port_num, data, context);
    }
}

impl CmdResponsePort for CmdSplitter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        self.seq_cmd_status_handler(port_num, op_code, cmd_seq, response);
    }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/CmdSplitter/test/ut` (`CmdSplitterTester`). Upstream
    //! picks the opcode, base, context and source port at random
    //! (`STest::Pick`); these tests sweep fixed sets that include the
    //! edges instead.
    use super::*;
    use fprime_fw::{CmdArgBuffer, SerBuf};
    use std::sync::Mutex;
    use std::vec::Vec;

    /// Records `(port, bytes, context)` per Com output and the forwarded
    /// responses.
    #[derive(Default)]
    struct Sink {
        com: Mutex<Vec<(FwIndexType, Vec<u8>, u32)>>,
        responses: Mutex<Vec<(FwIndexType, FwOpcodeType, u32, CmdResponse)>>,
    }

    impl ComPort for Sink {
        fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
            self.com
                .lock()
                .unwrap()
                .push((port_num, data.as_slice().to_vec(), context));
        }
    }

    impl CmdResponsePort for Sink {
        fn invoke(
            &self,
            port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.responses
                .lock()
                .unwrap()
                .push((port_num, op_code, cmd_seq, response));
        }
    }

    struct Harness {
        comp: Arc<CmdSplitter>,
        local: Arc<Sink>,
        remote: Arc<Sink>,
        status: Arc<Sink>,
    }

    impl Harness {
        fn new() -> Self {
            let comp = CmdSplitter::new("CmdSplitter");
            let local = Arc::new(Sink::default());
            let remote = Arc::new(Sink::default());
            let status = Arc::new(Sink::default());
            for i in 0..CMD_SPLITTER_PORTS {
                comp.local_cmd[i].connect(local.clone(), i as FwIndexType);
                comp.remote_cmd[i].connect(remote.clone(), i as FwIndexType);
                comp.forward_seq_cmd_status[i].connect(status.clone(), i as FwIndexType);
            }
            Self {
                comp,
                local,
                remote,
                status,
            }
        }

        fn send(&self, port: FwIndexType, data: &mut ComBuffer, context: u32) {
            let p = self.comp.cmd_buff_in(port);
            p.target.invoke(p.port_num, data, context);
        }
    }

    /// `build_command_around_opcode`: `[FW_PACKET_COMMAND u16][opcode][args]`.
    fn command(opcode: FwOpcodeType, arg_len: usize) -> ComBuffer {
        let mut buffer = ComBuffer::new();
        assert!(buffer.serialize_u16_be(0).is_ok());
        assert!(buffer.serialize_u32_be(opcode).is_ok());
        let mut args = CmdArgBuffer::new();
        for i in 0..arg_len {
            assert!(args.serialize_u8_be(i as u8).is_ok());
        }
        assert!(
            buffer
                .serialize_bytes(
                    args.as_slice(),
                    fprime_fw::LengthMode::OmitLength,
                    Endianness::Big
                )
                .is_ok()
        );
        buffer
    }

    // (base, local opcode) pairs: base >= 1 leaves room for a local opcode.
    const LOCAL_CASES: [(FwOpcodeType, FwOpcodeType); 5] = [
        (1, 0),
        (0x100, 0),
        (0x100, 0xFF),
        (0x8000_0000, 0x1234),
        (FwOpcodeType::MAX, FwOpcodeType::MAX - 1),
    ];
    // (base, remote opcode) pairs: opcode >= base.
    const REMOTE_CASES: [(FwOpcodeType, FwOpcodeType); 5] = [
        (0, 0),
        (0, 0x55),
        (0x100, 0x100),
        (0x100, FwOpcodeType::MAX),
        (FwOpcodeType::MAX, FwOpcodeType::MAX),
    ];

    #[test]
    fn local_routing() {
        // SVC-CMD-SPLITTER-000/001/002
        for (case, &(base, opcode)) in LOCAL_CASES.iter().enumerate() {
            for port in 0..CMD_SPLITTER_PORTS as FwIndexType {
                let h = Harness::new();
                h.comp.configure(base);
                let mut buffer = command(opcode, case * 37);
                let expected = buffer.as_slice().to_vec();
                h.send(port, &mut buffer, 0xC0DE_0000 + case as u32);
                assert!(h.remote.com.lock().unwrap().is_empty());
                assert_eq!(
                    *h.local.com.lock().unwrap(),
                    vec![(port, expected, 0xC0DE_0000 + case as u32)]
                );
            }
        }
    }

    #[test]
    fn remote_routing() {
        // SVC-CMD-SPLITTER-000/001/003
        for (case, &(base, opcode)) in REMOTE_CASES.iter().enumerate() {
            for port in 0..CMD_SPLITTER_PORTS as FwIndexType {
                let h = Harness::new();
                h.comp.configure(base);
                let mut buffer = command(opcode, case * 101);
                let expected = buffer.as_slice().to_vec();
                h.send(port, &mut buffer, case as u32);
                assert!(h.local.com.lock().unwrap().is_empty());
                assert_eq!(
                    *h.remote.com.lock().unwrap(),
                    vec![(port, expected, case as u32)]
                );
            }
        }
    }

    #[test]
    fn full_argument_buffer_routes_by_opcode() {
        let h = Harness::new();
        h.comp.configure(0x10);
        let mut buffer = command(0x10, fprime_config::FW_CMD_ARG_BUFFER_MAX_SIZE);
        assert_eq!(
            buffer.as_slice().len(),
            fprime_config::FW_COM_BUFFER_MAX_SIZE
        );
        h.send(2, &mut buffer, 7);
        assert_eq!(h.remote.com.lock().unwrap().len(), 1);
        assert!(h.local.com.lock().unwrap().is_empty());
    }

    #[test]
    fn unconfigured_routes_everything_but_max_local() {
        let h = Harness::new();
        assert_eq!(h.comp.remote_base(), FwOpcodeType::MAX);
        h.send(0, &mut command(0x0100_0000, 0), 1);
        h.send(0, &mut command(FwOpcodeType::MAX - 1, 0), 2);
        h.send(0, &mut command(FwOpcodeType::MAX, 0), 3);
        let local: Vec<u32> = h.local.com.lock().unwrap().iter().map(|e| e.2).collect();
        let remote: Vec<u32> = h.remote.com.lock().unwrap().iter().map(|e| e.2).collect();
        assert_eq!(local, vec![1, 2]);
        assert_eq!(remote, vec![3]);
    }

    #[test]
    fn error_routing() {
        // SVC-CMD-SPLITTER-000/001/004: an empty buffer, a short opcode and
        // a non-command descriptor all go local, whatever the base.
        let mut not_a_command = ComBuffer::new();
        assert!(not_a_command.serialize_u16_be(1).is_ok()); // FW_PACKET_TELEM
        assert!(not_a_command.serialize_u32_be(0xFFFF_FFFF).is_ok());
        let mut short = ComBuffer::new();
        assert!(short.serialize_u16_be(0).is_ok());
        assert!(short.serialize_u8_be(0xAB).is_ok());
        for (i, mut buffer) in [ComBuffer::new(), short, not_a_command]
            .into_iter()
            .enumerate()
        {
            for port in 0..CMD_SPLITTER_PORTS as FwIndexType {
                let h = Harness::new();
                h.comp.configure(0);
                let expected = buffer.as_slice().to_vec();
                h.send(port, &mut buffer, i as u32);
                assert!(h.remote.com.lock().unwrap().is_empty());
                assert_eq!(
                    *h.local.com.lock().unwrap(),
                    vec![(port, expected, i as u32)]
                );
            }
        }
    }

    #[test]
    fn response_forwarding() {
        // SVC-CMD-SPLITTER-000/001/005
        let responses = [
            CmdResponse::Ok,
            CmdResponse::InvalidOpcode,
            CmdResponse::ValidationError,
            CmdResponse::FormatError,
            CmdResponse::ExecutionError,
            CmdResponse::Busy,
        ];
        for (i, &response) in responses.iter().enumerate() {
            for port in 0..CMD_SPLITTER_PORTS as FwIndexType {
                let h = Harness::new();
                let opcode = (i as u32).wrapping_mul(0x3333_3333);
                let p = h.comp.seq_cmd_status_in(port);
                p.target
                    .invoke(p.port_num, opcode, 0xFFFF_0000 + i as u32, response);
                assert_eq!(
                    *h.status.responses.lock().unwrap(),
                    vec![(port, opcode, 0xFFFF_0000 + i as u32, response)]
                );
            }
        }
    }

    #[test]
    fn the_callers_read_cursor_is_left_past_the_packet() {
        // C++ parity: CmdPacket::deserializeFrom reads out of the caller's
        // buffer, so the forwarded buffer has nothing left to deserialize.
        let h = Harness::new();
        h.comp.configure(5);
        let mut buffer = command(4, 3);
        h.send(1, &mut buffer, 0);
        assert_eq!(buffer.deserialize_size_left(), 0);
        assert_eq!(buffer.as_slice().len(), 2 + 4 + 3);
    }

    #[test]
    #[should_panic]
    fn out_of_range_command_port_asserts() {
        let h = Harness::new();
        let mut buffer = command(0, 0);
        h.send(CMD_SPLITTER_PORTS as FwIndexType, &mut buffer, 0);
    }

    #[test]
    #[should_panic]
    fn out_of_range_status_port_asserts() {
        let h = Harness::new();
        let p = h.comp.seq_cmd_status_in(CMD_SPLITTER_PORTS as FwIndexType);
        p.target.invoke(p.port_num, 0, 0, CmdResponse::Ok);
    }
}
