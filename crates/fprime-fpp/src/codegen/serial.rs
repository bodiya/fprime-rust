//! Serial ports in the Rust back end, on the `fprime_comp::serial_port`
//! runtime: the FPP `serial` port type is `fprime_comp::SerializePort`,
//! a port definition is made serializable with
//! `fprime_comp::serializable_port!` (the C++ `<P>PortSerializer` and the
//! `FW_PORT_SERIALIZATION` halves of the typed ports), and the topology
//! wires typed -> serial connections with `OutputPort::connect_serial` and
//! serial -> typed ones with `OutputPort::connect_typed`.
//!
//! A port can be serialized when it has no return type (FPP forbids
//! connecting those to serial ports) and no `Fw.Buffer` parameter: a Rust
//! `Fw::Buffer` owns its memory, so it cannot be flattened to bytes the
//! way C++ serializes the buffer's pointer and size (deliberate deviation,
//! shared with the runtime; such a connection is a generation error).
//! Bound framework ports are made serializable by `fprime_comp` itself.

use super::Generator;
use super::component::snake_ident;
use crate::analysis::SymId;
use crate::ast::SpecialPortKind;
use crate::error::{Diagnostic, Loc, Result};

/// The FPP framework port type of a special port kind.
pub fn special_port_type(kind: SpecialPortKind) -> &'static str {
    match kind {
        SpecialPortKind::CommandRecv => "Fw.Cmd",
        SpecialPortKind::CommandReg => "Fw.CmdReg",
        SpecialPortKind::CommandResp => "Fw.CmdResponse",
        SpecialPortKind::Event => "Fw.Log",
        SpecialPortKind::TextEvent => "Fw.LogText",
        SpecialPortKind::TimeGet => "Fw.Time",
        SpecialPortKind::Telemetry => "Fw.Tlm",
        SpecialPortKind::ParamGet => "Fw.PrmGet",
        SpecialPortKind::ParamSet => "Fw.PrmSet",
        SpecialPortKind::ProductGet => "Fw.DpGet",
        SpecialPortKind::ProductRecv => "Fw.DpResponse",
        SpecialPortKind::ProductRequest => "Fw.DpRequest",
        SpecialPortKind::ProductSend => "Fw.DpSend",
    }
}

impl Generator<'_, '_> {
    /// Why a port type cannot be serialized, or `None` when it can.
    pub(super) fn serial_obstacle(&self, psym: SymId) -> Option<String> {
        let def = &self.a.ports[&psym];
        let name = self.a.symbols.sym(psym).qualified_name();
        if def.ret.is_some() {
            return Some(format!("port {name} returns a value"));
        }
        if let Some((p, _)) = self
            .port_params(psym)
            .into_iter()
            .find(|(_, mode)| mode == "owned")
        {
            return Some(format!(
                "parameter {} of port {name} is an Fw.Buffer, which owns its memory in the Rust port and cannot be serialized",
                p.name
            ));
        }
        None
    }

    /// Emit `serializable_port!` for a generated port definition `psym`
    /// in the current module: the C++ `<P>PortSerializer` /
    /// `<P>PortBuffer` and the serial halves of the typed port, which make
    /// `OutputPort<dyn PPort>::connect_serial` and
    /// `OutputPort<dyn SerializePort>::connect_typed` available. The
    /// caller checks [`Generator::serial_obstacle`] first.
    pub(super) fn emit_serializable_port(&mut self, psym: SymId) -> Result<()> {
        if let Some(why) = self.serial_obstacle(psym) {
            return Err(Diagnostic::codegen(Loc::none(), why));
        }
        let name = format!("{}Port", self.a.symbols.sym(psym).name);
        let mut args = Vec::new();
        for (p, mode) in self.port_params(psym) {
            args.push(format!(
                "{mode} {}: {}",
                snake_ident(&p.name),
                self.rust_type(&p.ty)?
            ));
        }
        self.line(&format!(
            "::fprime_comp::serializable_port! {{ port: {name}; args {{ {} }} }}",
            args.join(", ")
        ));
        Ok(())
    }
}
