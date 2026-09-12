//! Emission of components: the Rust equivalent of the C++ autocoder's
//! `<Comp>ComponentAc`.
//!
//! For an FPP component `C` the back end emits, in the component's module:
//!
//! - `CBase` — the generated state: the kind-specific core (`PassiveBase`,
//!   `QueuedBase` or `ActiveBase`), the command/event/telemetry/parameter
//!   glue, the data-product output ports, every general output port (an
//!   `OutputPort` or an array of them), event throttles, the parameter
//!   store, the telemetry "update on change" cache, the guarded-port mutex
//!   and a buffer escrow for async ports that carry `Fw.Buffer`. Its
//!   associated constants are the dictionary (opcodes, event/channel/
//!   parameter/container/record ids, message types, `MSG_SIZE`), and its
//!   methods are the autocoded helpers: `reg_commands`, output-port
//!   invocation (`<port>_out`), `log_<SEVERITY>_<Event>`, `tlm_write_<Chan>`,
//!   `param_get_<Param>`, `dp_get_<Container>`, `dp_send`,
//!   `serialize_record_<Record>`.
//! - `CHandlers` — the trait the implementation provides: `base()`, one
//!   handler per input port / command / internal port, plus the hooks with
//!   default bodies (`parameter_updated`, `parameters_loaded`, pre-message
//!   and overflow hooks).
//! - `CComponent` — a blanket extension trait over `CHandlers` with the
//!   input-port factories (`<port>_in`), the internal-port invocations,
//!   `load_parameters`, `cmd_dispatch` and `dispatch_message` (the queue
//!   message decoder, byte-exact with the hand-written components).
//! - `impl_<c>_component!` — a macro that implements `ComponentDispatch`
//!   (and `ActiveComponent` for active components) for the implementation
//!   type in one line.
//!
//! The wire formats are exactly those of the hand-written components:
//! queue envelopes are `[msg_type i32][port_num i16][args]`, command
//! messages nest the argument buffer with a length prefix, `Fw.Buffer`
//! arguments cross the queue as escrow tokens.

use super::names::{ident, snake};

/// Snake-case identifier for an FPP parameter, kept clear of the names the
/// generated signatures and locals use themselves.
fn snake_ident(name: &str) -> String {
    let s = super::names::snake_ident(name);
    match s.as_str() {
        "port_num" | "op_code" | "cmd_seq" | "args" | "value" | "time_tag" | "container"
        | "data_size" | "elements" | "ps" => format!("{s}_arg"),
        _ if s.starts_with("fpp_") => format!("{s}_arg"),
        _ => s,
    }
}
use super::{ArgKind, Generator};
use crate::analysis::format::{Field, Format, IntField, RatField};
use crate::analysis::{
    CommandDef, ComponentModel, EventDef, ParamDef, PortInstance, PortType, SymId, Type,
};
use crate::ast::{
    CommandKind, ComponentKind, GeneralPortKind, QueueFull, Severity, SpecialInputKind,
    SpecialPortKind,
};
use crate::error::{Diagnostic, Result};

/// One kind of queued message.
#[derive(Clone)]
struct AsyncMsg {
    /// `MSG_TYPE_<NAME>` constant name.
    const_name: String,
    /// Which construct it carries.
    kind: MsgKind,
    /// The arguments carried after the envelope (empty for commands), with
    /// their passing modes.
    params: Vec<(ParamDef, String)>,
    /// Queue priority.
    priority: i128,
    /// Queue-full policy.
    queue_full: QueueFull,
}

#[derive(Clone)]
enum MsgKind {
    /// A general async input port (port name).
    Port(String),
    /// The async command message.
    Cmd,
    /// The async product receive port (port name).
    ProductRecv(String),
    /// An internal port (name).
    Internal(String),
}

/// What the generator knows about the component being emitted.
struct Ctx<'m> {
    m: &'m ComponentModel,
    name: String,
    base: String,
    handlers: String,
    comp: String,
    kind: ComponentKind,
    has_cmd: bool,
    has_evt: bool,
    has_tlm: bool,
    has_prm: bool,
    /// The product recv port, if any, with its input kind.
    dp_recv: Option<(String, SpecialInputKind)>,
    msgs: Vec<AsyncMsg>,
    /// Any async message carries an `Fw.Buffer` (escrow needed).
    needs_escrow: bool,
    /// Any guarded port or command.
    needs_guard: bool,
}

impl Generator<'_, '_> {
    /// Emit everything for a component.
    pub(super) fn emit_component(&mut self, sym: SymId) -> Result<()> {
        let m = self.a.components[&sym].clone();
        let s = self.a.symbols.sym(sym);
        let name = s.name.clone();
        let docs = s.docs.clone();
        let loc = s.loc.clone();

        // Unsupported constructs.
        for p in &m.ports {
            if let PortInstance::General {
                port: PortType::Serial,
                name: pn,
                ..
            } = p
            {
                return Err(Diagnostic::codegen(
                    p.loc().clone(),
                    format!(
                        "component {name}: serial port {pn} is not supported by the Rust back end (typed ports only)"
                    ),
                ));
            }
        }
        if let Some((smi, _)) = m.state_machine_instances.first() {
            return Err(Diagnostic::codegen(
                loc,
                format!(
                    "component {name}: state machine instance {smi} is not supported by the Rust back end yet"
                ),
            ));
        }

        let has = |k: SpecialPortKind| m.special_port(k).is_some();
        let dp_recv = m.ports.iter().find_map(|p| match p {
            PortInstance::Special {
                kind: SpecialPortKind::ProductRecv,
                name,
                input_kind,
                ..
            } => Some((name.clone(), input_kind.unwrap_or(SpecialInputKind::Async))),
            _ => None,
        });
        let mut ctx = Ctx {
            m: &m,
            name: name.clone(),
            base: format!("{name}Base"),
            handlers: format!("{name}Handlers"),
            comp: format!("{name}Component"),
            kind: m.kind,
            has_cmd: has(SpecialPortKind::CommandRecv)
                || has(SpecialPortKind::CommandReg)
                || has(SpecialPortKind::CommandResp),
            has_evt: has(SpecialPortKind::Event)
                || has(SpecialPortKind::TextEvent)
                || has(SpecialPortKind::TimeGet),
            has_tlm: has(SpecialPortKind::Telemetry),
            has_prm: has(SpecialPortKind::ParamGet) || has(SpecialPortKind::ParamSet),
            dp_recv,
            msgs: Vec::new(),
            needs_escrow: false,
            needs_guard: false,
        };
        self.collect_msgs(&mut ctx)?;

        self.line("");
        self.line(&format!(
            "// ---- component {} ({}) ----",
            s.qualified_name(),
            m.kind
        ));
        self.line("");
        self.line(
            "use ::fprime_fw::{SerBuf as _, SerBufAny as _, Serialize as _, Deserialize as _};",
        );
        self.emit_params_struct(&ctx)?;
        self.emit_tlm_cache(&ctx)?;
        self.emit_base_struct(&ctx, &docs)?;
        self.emit_base_consts(&ctx)?;
        self.emit_base_impl(&ctx)?;
        self.emit_handlers_trait(&ctx)?;
        self.emit_component_trait(&ctx)?;
        self.emit_adapters(&ctx)?;
        self.emit_impl_macro(&ctx)?;
        Ok(())
    }

    // -- message inventory -------------------------------------------------------------

    fn collect_msgs(&self, ctx: &mut Ctx<'_>) -> Result<()> {
        let m = ctx.m;
        for p in &m.ports {
            match p {
                PortInstance::General {
                    name,
                    kind: GeneralPortKind::AsyncInput,
                    port: PortType::Typed(psym),
                    priority,
                    queue_full,
                    ..
                } => {
                    let params = self.port_params(*psym);
                    ctx.msgs.push(AsyncMsg {
                        const_name: format!("MSG_TYPE_{}", snake(name).to_ascii_uppercase()),
                        kind: MsgKind::Port(name.clone()),
                        params,
                        priority: priority.unwrap_or(0),
                        queue_full: *queue_full,
                    });
                }
                PortInstance::General {
                    kind: GeneralPortKind::GuardedInput,
                    ..
                } => ctx.needs_guard = true,
                _ => {}
            }
        }
        let queued_cmds = m.commands.iter().any(|c| c.kind == CommandKind::Async);
        if queued_cmds {
            let max_priority = m
                .commands
                .iter()
                .filter_map(|c| c.priority)
                .max()
                .unwrap_or(0);
            let policy = m
                .commands
                .iter()
                .find(|c| c.kind == CommandKind::Async && c.param_cmd.is_none())
                .map(|c| c.queue_full)
                .unwrap_or(QueueFull::Assert);
            ctx.msgs.push(AsyncMsg {
                const_name: "MSG_TYPE_CMD_IN".into(),
                kind: MsgKind::Cmd,
                params: Vec::new(),
                priority: max_priority,
                queue_full: policy,
            });
        }
        if m.commands.iter().any(|c| c.kind == CommandKind::Guarded) {
            ctx.needs_guard = true;
        }
        if let Some((pname, SpecialInputKind::Async)) = &ctx.dp_recv {
            let (priority, queue_full) = m
                .ports
                .iter()
                .find_map(|p| match p {
                    PortInstance::Special {
                        name,
                        priority,
                        queue_full,
                        ..
                    } if name == pname => Some((priority.unwrap_or(0), *queue_full)),
                    _ => None,
                })
                .unwrap_or((0, QueueFull::Assert));
            ctx.msgs.push(AsyncMsg {
                const_name: "MSG_TYPE_PRODUCT_RECV_IN".into(),
                kind: MsgKind::ProductRecv(pname.clone()),
                params: self.dp_response_params()?,
                priority,
                queue_full,
            });
            ctx.needs_escrow = true;
        }
        if matches!(ctx.dp_recv, Some((_, SpecialInputKind::Guarded))) {
            ctx.needs_guard = true;
        }
        for ip in &m.internal_ports {
            ctx.msgs.push(AsyncMsg {
                const_name: format!("MSG_TYPE_{}_INTERNAL", snake(&ip.name).to_ascii_uppercase()),
                kind: MsgKind::Internal(ip.name.clone()),
                params: self.plain_params(&ip.params),
                priority: ip.priority.unwrap_or(0),
                queue_full: ip.queue_full,
            });
        }
        for msg in &ctx.msgs {
            for (_, mode) in &msg.params {
                if mode == "owned" {
                    ctx.needs_escrow = true;
                }
            }
        }
        Ok(())
    }

    /// A port definition's parameters with their passing modes (bound
    /// modes win over the default rules).
    fn port_params(&self, psym: SymId) -> Vec<(ParamDef, String)> {
        let def = &self.a.ports[&psym];
        def.params
            .iter()
            .enumerate()
            .map(|(i, p)| (p.clone(), self.mode_in(Some(psym), i, p)))
            .collect()
    }

    /// Parameters with the default modes (commands, events, internal ports).
    fn plain_params(&self, params: &[ParamDef]) -> Vec<(ParamDef, String)> {
        params
            .iter()
            .map(|p| (p.clone(), self.default_mode(p).to_string()))
            .collect()
    }

    /// The `Fw.DpResponse` port's parameters (the framework definition
    /// must be imported).
    fn dp_response_params(&self) -> Result<Vec<(ParamDef, String)>> {
        match self.find(crate::analysis::NameGroup::Port, "Fw.DpResponse") {
            Some(p) if self.a.ports.contains_key(&p) => Ok(self.port_params(p)),
            _ => Err(Diagnostic::codegen(
                crate::error::Loc::none(),
                "a product recv port needs the framework definition of Fw.DpResponse; import Fw/Dp/Dp.fpp",
            )),
        }
    }

    // -- params / tlm cache ------------------------------------------------------------------

    fn emit_params_struct(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        if ctx.m.params.is_empty() {
            return Ok(());
        }
        let pname = format!("{}Params", ctx.name);
        self.line("");
        self.line(&format!(
            "/// Parameter store of `{}` (staged values and their validity).",
            ctx.name
        ));
        self.line("#[derive(Debug, Clone)]");
        self.line(&format!("pub struct {pname} {{"));
        self.indent();
        for p in &ctx.m.params {
            let f = snake_ident(&p.name);
            let t = self.rust_type(&p.ty)?;
            self.doc(&p.docs);
            self.line(&format!("pub {f}: {t},"));
            self.line(&format!("/// Validity of `{}`.", p.name));
            self.line(&format!(
                "pub {}_valid: ::fprime_fw::ParamValid,",
                snake(&p.name)
            ));
        }
        self.dedent();
        self.line("}");
        self.line(&format!("impl Default for {pname} {{"));
        self.indent();
        self.line("fn default() -> Self {");
        self.indent();
        self.line("Self {");
        self.indent();
        for p in &ctx.m.params {
            let f = snake_ident(&p.name);
            let v = match &p.default {
                Some(v) => self.render_value(v)?,
                None => format!("<{} as Default>::default()", self.rust_type(&p.ty)?),
            };
            self.line(&format!("{f}: {v},"));
            self.line(&format!(
                "{}_valid: ::fprime_fw::ParamValid::Uninit,",
                snake(&p.name)
            ));
        }
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");
        Ok(())
    }

    fn emit_tlm_cache(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let on_change: Vec<_> = ctx
            .m
            .tlm_channels
            .iter()
            .filter(|t| t.update == crate::ast::TlmUpdate::OnChange)
            .collect();
        if on_change.is_empty() {
            return Ok(());
        }
        let name = format!("{}TlmLast", ctx.name);
        self.line("");
        self.line(&format!(
            "/// Last written values of the `update on change` channels of `{}`.",
            ctx.name
        ));
        self.line("#[derive(Debug, Default)]");
        self.line(&format!("pub struct {name} {{"));
        self.indent();
        for t in &on_change {
            let ty = self.rust_type(&t.ty)?;
            self.line(&format!("pub {}: Option<{ty}>,", snake_ident(&t.name)));
        }
        self.dedent();
        self.line("}");
        Ok(())
    }

    // -- base struct ---------------------------------------------------------------------------

    fn emit_base_struct(&mut self, ctx: &Ctx<'_>, docs: &[String]) -> Result<()> {
        let base = &ctx.base;
        self.line("");
        self.doc(docs);
        self.line(&format!(
            "/// Generated base of the `{}` {} component (the C++ `{}ComponentBase`).",
            ctx.name, ctx.kind, ctx.name
        ));
        self.line(&format!("pub struct {base} {{"));
        self.indent();
        match ctx.kind {
            ComponentKind::Passive => self.line("/// Object state (name, id base, instance).\n    pub base: ::fprime_comp::PassiveBase,"),
            ComponentKind::Queued => self.line("/// Queued core: object state + message queue.\n    pub queued: ::fprime_comp::QueuedBase,"),
            ComponentKind::Active => self.line("/// Active core: object state + queue + task.\n    pub active: ::fprime_comp::ActiveBase,"),
        }
        if ctx.has_cmd {
            self.line("/// Command registration/response ports.");
            self.line("pub cmd: ::fprime_comp::CmdGlue,");
        }
        if ctx.has_evt {
            self.line("/// Event, text event and time ports.");
            self.line("pub evt: ::fprime_comp::EventGlue,");
        }
        if ctx.has_tlm {
            self.line("/// Telemetry port.");
            self.line("pub tlm: ::fprime_comp::TlmGlue,");
        }
        if ctx.has_prm {
            self.line("/// Parameter get/set ports.");
            self.line("pub prm: ::fprime_comp::PrmGlue,");
        }
        for p in &ctx.m.ports {
            match p {
                PortInstance::Special {
                    kind: SpecialPortKind::ProductGet,
                    name,
                    docs,
                    ..
                } => {
                    self.doc(docs);
                    self.line(&format!(
                        "pub {}: ::fprime_comp::OutputPort<dyn ::fprime_fw::dp::DpGetPort>,",
                        snake_ident(name)
                    ));
                }
                PortInstance::Special {
                    kind: SpecialPortKind::ProductSend,
                    name,
                    docs,
                    ..
                } => {
                    self.doc(docs);
                    self.line(&format!(
                        "pub {}: ::fprime_comp::OutputPort<dyn ::fprime_fw::dp::DpSendPort>,",
                        snake_ident(name)
                    ));
                }
                PortInstance::Special {
                    kind: SpecialPortKind::ProductRequest,
                    name,
                    docs,
                    ..
                } => {
                    self.doc(docs);
                    self.line(&format!(
                        "pub {}: ::fprime_comp::OutputPort<dyn ::fprime_fw::dp::DpRequestPort>,",
                        snake_ident(name)
                    ));
                }
                PortInstance::General {
                    kind: GeneralPortKind::Output,
                    name,
                    size,
                    port: PortType::Typed(psym),
                    docs,
                    ..
                } => {
                    let trait_path = self.port_path(*psym);
                    self.doc(docs);
                    if *size == 1 {
                        self.line(&format!(
                            "pub {}: ::fprime_comp::OutputPort<dyn {trait_path}>,",
                            snake_ident(name)
                        ));
                    } else {
                        self.line(&format!(
                            "pub {}: [::fprime_comp::OutputPort<dyn {trait_path}>; {size}],",
                            snake_ident(name)
                        ));
                    }
                }
                _ => {}
            }
        }
        for e in &ctx.m.events {
            if e.throttle.is_some() {
                self.line(&format!("/// Throttle of event `{}`.", e.name));
                self.line(&format!(
                    "pub throttle_{}: ::fprime_comp::EventThrottle,",
                    snake(&e.name)
                ));
            }
        }
        if !ctx.m.params.is_empty() {
            self.line("/// The parameter store.");
            self.line(&format!(
                "pub params: ::std::sync::Mutex<{}Params>,",
                ctx.name
            ));
        }
        if ctx
            .m
            .tlm_channels
            .iter()
            .any(|t| t.update == crate::ast::TlmUpdate::OnChange)
        {
            self.line("/// Last values of the `update on change` channels.");
            self.line(&format!(
                "pub tlm_last: ::std::sync::Mutex<{}TlmLast>,",
                ctx.name
            ));
        }
        if ctx.needs_guard {
            self.line("/// The guarded-port mutex (C++ `m_guardedPortMutex`).");
            self.line("pub guard: ::std::sync::Mutex<()>,");
        }
        if ctx.needs_escrow {
            self.line("/// Escrow for `Fw.Buffer`s crossing the queue.");
            self.line("pub escrow: ::fprime_comp::BufferEscrow,");
        }
        self.dedent();
        self.line("}");
        Ok(())
    }

    // -- constants ------------------------------------------------------------------------------

    fn emit_base_consts(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let base = &ctx.base;
        self.line("");
        self.line(&format!("impl {base} {{"));
        self.indent();
        for c in &ctx.m.commands {
            self.doc(&c.docs);
            self.line(&format!(
                "pub const OPCODE_{}: ::fprime_config::FwOpcodeType = {:#x};",
                snake(&c.name).to_ascii_uppercase(),
                c.opcode
            ));
        }
        for e in &ctx.m.events {
            self.doc(&e.docs);
            self.line(&format!(
                "pub const EVENTID_{}: ::fprime_config::FwEventIdType = {:#x};",
                snake(&e.name).to_ascii_uppercase(),
                e.id
            ));
            if let Some(t) = e.throttle {
                self.line(&format!("/// Throttle count of `{}`.", e.name));
                self.line(&format!(
                    "pub const {}_THROTTLE: u32 = {t};",
                    snake(&e.name).to_ascii_uppercase()
                ));
            }
        }
        for t in &ctx.m.tlm_channels {
            self.doc(&t.docs);
            self.line(&format!(
                "pub const CHANID_{}: ::fprime_config::FwChanIdType = {:#x};",
                snake(&t.name).to_ascii_uppercase(),
                t.id
            ));
        }
        for p in &ctx.m.params {
            self.doc(&p.docs);
            self.line(&format!(
                "pub const PARAMID_{}: ::fprime_config::FwPrmIdType = {:#x};",
                snake(&p.name).to_ascii_uppercase(),
                p.id
            ));
        }
        for c in &ctx.m.containers {
            self.doc(&c.docs);
            self.line(&format!(
                "pub const CONTAINER_ID_{}: ::fprime_config::FwDpIdType = {:#x};",
                snake(&c.name).to_ascii_uppercase(),
                c.id
            ));
            self.line(&format!("/// Default priority of container `{}`.", c.name));
            self.line(&format!(
                "pub const CONTAINER_PRIORITY_{}: ::fprime_config::FwDpPriorityType = {};",
                snake(&c.name).to_ascii_uppercase(),
                c.default_priority.unwrap_or(0)
            ));
        }
        for r in &ctx.m.records {
            self.doc(&r.docs);
            self.line(&format!(
                "pub const RECORD_ID_{}: ::fprime_config::FwDpIdType = {:#x};",
                snake(&r.name).to_ascii_uppercase(),
                r.id
            ));
            let elt = self.size_expr(&r.ty)?;
            if r.is_array {
                self.line(&format!(
                    "/// Autocoded `SIZE_OF_{}_RECORD(n)`: id, element count and `n` elements.",
                    r.name
                ));
                self.line(&format!(
                    "pub const fn size_of_{}_record(elements: ::fprime_config::FwSizeType) -> ::fprime_config::FwSizeType {{ (::std::mem::size_of::<::fprime_config::FwDpIdType>() + ::std::mem::size_of::<::fprime_config::FwSizeStoreType>()) as ::fprime_config::FwSizeType + elements * ({elt}) as ::fprime_config::FwSizeType }}",
                    snake(&r.name)
                ));
            } else {
                self.line(&format!(
                    "/// Autocoded `SIZE_OF_{}_RECORD`: id plus the serialized value.",
                    r.name
                ));
                self.line(&format!(
                    "pub const SIZE_OF_{}_RECORD: ::fprime_config::FwSizeType = (::std::mem::size_of::<::fprime_config::FwDpIdType>() + ({elt})) as ::fprime_config::FwSizeType;",
                    snake(&r.name).to_ascii_uppercase()
                ));
            }
        }
        // Message types and size.
        if !ctx.msgs.is_empty() {
            self.line("/// Queue message types (0 is the EXIT sentinel).");
            for (i, msg) in ctx.msgs.iter().enumerate() {
                self.line(&format!(
                    "pub const {}: ::fprime_config::FwEnumStoreType = {};",
                    msg.const_name,
                    i + 1
                ));
            }
            let mut sizes: Vec<String> = Vec::new();
            for msg in &ctx.msgs {
                let mut parts = vec!["6usize".to_string()];
                match &msg.kind {
                    MsgKind::Cmd => {
                        parts
                            .push("4 + 4 + 2 + ::fprime_config::FW_CMD_ARG_BUFFER_MAX_SIZE".into());
                    }
                    _ => {
                        for p in &msg.params {
                            parts.push(format!("({})", self.size_expr(&p.0.ty)?));
                        }
                    }
                }
                sizes.push(parts.join(" + "));
            }
            let mut expr = sizes.pop().expect("at least one");
            while let Some(s) = sizes.pop() {
                expr = format!("Self::__max({s}, {expr})");
            }
            self.line("const fn __max(a: usize, b: usize) -> usize { if a > b { a } else { b } }");
            self.line("/// Queue message size: the largest envelope + arguments.");
            self.line(&format!("pub const MSG_SIZE: usize = {expr};"));
        }
        self.dedent();
        self.line("}");
        Ok(())
    }

    // -- base impl ------------------------------------------------------------------------------

    fn core_expr(ctx: &Ctx<'_>) -> &'static str {
        match ctx.kind {
            ComponentKind::Passive => "self.base",
            ComponentKind::Queued => "self.queued.base",
            ComponentKind::Active => "self.active.queued.base",
        }
    }

    fn queued_expr(ctx: &Ctx<'_>) -> &'static str {
        match ctx.kind {
            ComponentKind::Passive => "",
            ComponentKind::Queued => "self.queued",
            ComponentKind::Active => "self.active.queued",
        }
    }

    fn emit_base_impl(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let base = ctx.base.clone();
        let core = Self::core_expr(ctx);
        self.line("");
        self.line(&format!("impl {base} {{"));
        self.indent();
        // new
        self.line("/// Construct the base with every port unconnected (C++ constructor).");
        self.line("pub fn new(name: &str) -> Self {");
        self.indent();
        self.line("Self {");
        self.indent();
        match ctx.kind {
            ComponentKind::Passive => self.line("base: ::fprime_comp::PassiveBase::new(name),"),
            ComponentKind::Queued => self.line("queued: ::fprime_comp::QueuedBase::new(name),"),
            ComponentKind::Active => self.line("active: ::fprime_comp::ActiveBase::new(name),"),
        }
        if ctx.has_cmd {
            self.line("cmd: ::fprime_comp::CmdGlue::new(),");
        }
        if ctx.has_evt {
            self.line("evt: ::fprime_comp::EventGlue::new(),");
        }
        if ctx.has_tlm {
            self.line("tlm: ::fprime_comp::TlmGlue::new(),");
        }
        if ctx.has_prm {
            self.line("prm: ::fprime_comp::PrmGlue::new(),");
        }
        for p in &ctx.m.ports {
            match p {
                PortInstance::Special {
                    kind:
                        SpecialPortKind::ProductGet
                        | SpecialPortKind::ProductSend
                        | SpecialPortKind::ProductRequest,
                    name,
                    ..
                } => {
                    self.line(&format!(
                        "{}: ::fprime_comp::OutputPort::new(),",
                        snake_ident(name)
                    ));
                }
                PortInstance::General {
                    kind: GeneralPortKind::Output,
                    name,
                    size,
                    ..
                } => {
                    if *size == 1 {
                        self.line(&format!(
                            "{}: ::fprime_comp::OutputPort::new(),",
                            snake_ident(name)
                        ));
                    } else {
                        self.line(&format!(
                            "{}: ::std::array::from_fn(|_| ::fprime_comp::OutputPort::new()),",
                            snake_ident(name)
                        ));
                    }
                }
                _ => {}
            }
        }
        for e in &ctx.m.events {
            if e.throttle.is_some() {
                self.line(&format!(
                    "throttle_{}: ::fprime_comp::EventThrottle::new(Self::{}_THROTTLE),",
                    snake(&e.name),
                    snake(&e.name).to_ascii_uppercase()
                ));
            }
        }
        if !ctx.m.params.is_empty() {
            self.line("params: ::std::sync::Mutex::new(Default::default()),");
        }
        if ctx
            .m
            .tlm_channels
            .iter()
            .any(|t| t.update == crate::ast::TlmUpdate::OnChange)
        {
            self.line("tlm_last: ::std::sync::Mutex::new(Default::default()),");
        }
        if ctx.needs_guard {
            self.line("guard: ::std::sync::Mutex::new(()),");
        }
        if ctx.needs_escrow {
            self.line("escrow: ::fprime_comp::BufferEscrow::new(),");
        }
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");
        // init / id base
        if ctx.kind != ComponentKind::Passive {
            let q = Self::queued_expr(ctx);
            self.line("/// Create the message queue (C++ `init(queueDepth)`).");
            self.line("pub fn init(&self, queue_depth: ::fprime_config::FwSizeType) {");
            self.line(&format!(
                "    {q}.create_queue(queue_depth, Self::MSG_SIZE as ::fprime_config::FwSizeType);"
            ));
            self.line("}");
        }
        self.line("/// Set the id base (C++ `setIdBase`).");
        self.line("pub fn set_id_base(&self, base: ::fprime_config::FwIdType) {");
        self.line(&format!("    {core}.set_id_base(base);"));
        self.line("}");
        self.line("/// The id base.");
        self.line("pub fn id_base(&self) -> ::fprime_config::FwIdType {");
        self.line(&format!("    {core}.get_id_base()"));
        self.line("}");
        // reg_commands
        if ctx.has_cmd && !ctx.m.commands.is_empty() {
            self.line("/// Register every opcode with the dispatcher (C++ `regCommands`).");
            self.line("pub fn reg_commands(&self) {");
            self.indent();
            let list: Vec<String> = ctx
                .m
                .commands
                .iter()
                .map(|c| format!("Self::OPCODE_{}", snake(&c.name).to_ascii_uppercase()))
                .collect();
            self.line(&format!(
                "self.cmd.reg_commands(self.id_base(), &[{}]);",
                list.join(", ")
            ));
            self.dedent();
            self.line("}");
        }
        // output port helpers
        for p in &ctx.m.ports {
            if let PortInstance::General {
                kind: GeneralPortKind::Output,
                name,
                size,
                port: PortType::Typed(psym),
                ..
            } = p
            {
                let def = self.a.ports[psym].clone();
                let params = self.port_params(*psym);
                let sig = self.handler_sig(&params)?;
                let call: Vec<String> = params
                    .iter()
                    .map(|(prm, _)| snake_ident(&prm.name))
                    .collect();
                let ret = match &def.ret {
                    Some(t) => format!(" -> {}", self.rust_type(t)?),
                    None => String::new(),
                };
                let f = snake_ident(name);
                let plain = snake(name);
                let index = if *size == 1 {
                    String::new()
                } else {
                    "[port_num as usize]".to_string()
                };
                self.line(&format!(
                    "/// Invoke output port `{name}` (C++ `{name}_out`). Asserts if unconnected."
                ));
                if def.params.len() > 6 {
                    self.line("#[allow(clippy::too_many_arguments)]");
                }
                let sigs = if sig.is_empty() {
                    String::new()
                } else {
                    format!(", {}", sig.join(", "))
                };
                self.line(&format!("pub fn {plain}_out(&self, port_num: ::fprime_config::FwIndexType{sigs}){ret} {{"));
                self.indent();
                if *size == 1 {
                    self.line("::fprime_fw::fw_assert!(port_num == 0, port_num);");
                }
                self.line(&format!("let p = self.{f}{index}.get();"));
                let calls = if call.is_empty() {
                    String::new()
                } else {
                    format!(", {}", call.join(", "))
                };
                self.line(&format!("p.target.invoke(p.port_num{calls})"));
                self.dedent();
                self.line("}");
                self.line(&format!("/// Whether output port `{name}` is connected (C++ `isConnected_{name}_OutputPort`)."));
                self.line(&format!("pub fn is_connected_{plain}_out(&self, port_num: ::fprime_config::FwIndexType) -> bool {{"));
                if *size == 1 {
                    self.line(&format!("    port_num == 0 && self.{f}.is_connected()"));
                } else {
                    self.line(&format!("    (port_num as usize) < {size} && self.{f}[port_num as usize].is_connected()"));
                }
                self.line("}");
            }
        }
        // events
        for e in &ctx.m.events {
            self.emit_event_fn(ctx, e)?;
        }
        // telemetry
        for t in &ctx.m.tlm_channels {
            self.emit_tlm_fn(ctx, t)?;
        }
        // params
        for p in &ctx.m.params {
            let f = snake(&p.name);
            let ty = self.rust_type(&p.ty)?;
            self.line(&format!(
                "/// Read parameter `{}` and its validity (C++ `paramGet_{}`).",
                p.name, p.name
            ));
            self.line(&format!(
                "pub fn param_get_{f}(&self) -> ({ty}, ::fprime_fw::ParamValid) {{"
            ));
            self.line("    let fpp_ps = self.params.lock().unwrap();");
            self.line(&format!(
                "    (fpp_ps.{}.clone(), fpp_ps.{f}_valid)",
                snake_ident(&p.name)
            ));
            self.line("}");
        }
        // data products
        self.emit_dp_fns(ctx)?;
        self.dedent();
        self.line("}");
        Ok(())
    }

    fn severity_variant(sev: Severity) -> &'static str {
        match sev {
            Severity::ActivityHigh => "ActivityHi",
            Severity::ActivityLow => "ActivityLo",
            Severity::Command => "Command",
            Severity::Diagnostic => "Diagnostic",
            Severity::Fatal => "Fatal",
            Severity::WarningHigh => "WarningHi",
            Severity::WarningLow => "WarningLo",
        }
    }

    fn severity_snake(sev: Severity) -> &'static str {
        match sev {
            Severity::ActivityHigh => "activity_hi",
            Severity::ActivityLow => "activity_lo",
            Severity::Command => "command",
            Severity::Diagnostic => "diagnostic",
            Severity::Fatal => "fatal",
            Severity::WarningHigh => "warning_hi",
            Severity::WarningLow => "warning_lo",
        }
    }

    /// The Rust type a helper takes for a value parameter: `T` for Copy,
    /// `&str` for strings, `&T` otherwise.
    fn helper_param_type(&self, t: &Type) -> Result<(String, bool)> {
        Ok(match self.a.underlying(t) {
            Type::String(_) => ("&str".into(), true),
            _ => match self.arg_kind(t) {
                ArgKind::Copy => (self.rust_type(t)?, false),
                _ => (format!("&{}", self.rust_type(t)?), false),
            },
        })
    }

    /// Rust `format!` string for an FPP format and its parameters.
    fn rust_format(&self, fmt: &Format, params: &[ParamDef]) -> String {
        fn esc(s: &str) -> String {
            s.replace('{', "{{").replace('}', "}}")
        }
        let mut out = esc(&fmt.prefix);
        for ((field, suffix), p) in fmt.fields.iter().zip(params.iter()) {
            let under = self.a.underlying(&p.ty);
            let debug = !matches!(
                under,
                Type::Int(_) | Type::Float(_) | Type::Bool | Type::Integer | Type::String(_)
            );
            let spec = match field {
                Field::Default => if debug { "{:?}" } else { "{}" }.to_string(),
                Field::Integer(IntField::Decimal) | Field::Integer(IntField::Character) => {
                    "{}".into()
                }
                Field::Integer(IntField::Hexadecimal) => "{:x}".into(),
                Field::Integer(IntField::Octal) => "{:o}".into(),
                Field::Rational(p, RatField::Exponent) => match p {
                    Some(n) => format!("{{:.{n}e}}"),
                    None => "{:e}".into(),
                },
                Field::Rational(p, RatField::Fixed) => format!("{{:.{}}}", p.unwrap_or(6)),
                Field::Rational(p, RatField::General) => match p {
                    Some(n) => format!("{{:.{n}}}"),
                    None => "{}".into(),
                },
            };
            out.push_str(&spec);
            out.push_str(&esc(suffix));
        }
        out
    }

    fn emit_event_fn(&mut self, ctx: &Ctx<'_>, e: &EventDef) -> Result<()> {
        let _ = ctx;
        let sev = Self::severity_variant(e.severity);
        let fname = format!(
            "log_{}_{}",
            Self::severity_snake(e.severity),
            snake(&e.name)
        );
        let mut sig = Vec::new();
        let mut fmt_args = Vec::new();
        for p in &e.params {
            let (t, _) = self.helper_param_type(&p.ty)?;
            sig.push(format!("{}: {t}", snake_ident(&p.name)));
            fmt_args.push(snake_ident(&p.name));
        }
        self.line(&format!(
            "/// Emit event `{}` ({}, id {:#x}; C++ `log_{}_{}`).",
            e.name,
            e.severity,
            e.id,
            sev.to_ascii_uppercase(),
            e.name
        ));
        if e.params.len() > 6 {
            self.line("#[allow(clippy::too_many_arguments)]");
        }
        let sigs = if sig.is_empty() {
            String::new()
        } else {
            format!(", {}", sig.join(", "))
        };
        self.line(&format!("pub fn {fname}(&self{sigs}) {{"));
        self.indent();
        if e.throttle.is_some() {
            self.line(&format!(
                "if !self.throttle_{}.ok_to_emit() {{ return; }}",
                snake(&e.name)
            ));
        }
        let fmt = self.rust_format(&e.format, &e.params);
        if fmt_args.is_empty() {
            self.line(&format!("let fpp_text = {fmt:?}.to_string();"));
        } else {
            self.line(&format!(
                "let fpp_text = format!({fmt:?}, {});",
                fmt_args.join(", ")
            ));
        }
        self.line(&format!(
            "self.evt.log_event(self.id_base(), Self::EVENTID_{}, ::fprime_fw::LogSeverity::{sev}, &fpp_text, |fpp_buf| {{",
            snake(&e.name).to_ascii_uppercase()
        ));
        self.indent();
        if e.params.is_empty() {
            self.line("let _ = fpp_buf;");
            self.line("::fprime_fw::SerializeStatus::Ok");
        } else {
            let n = e.params.len();
            for (i, p) in e.params.iter().enumerate() {
                let arg = snake_ident(&p.name);
                let expr = match self.a.underlying(&p.ty) {
                    Type::String(size) => format!(
                        "::fprime_fw::FwString::<{}>::from({arg}).serialize_to(fpp_buf, ::fprime_fw::Endianness::Big)",
                        size.unwrap_or(crate::analysis::types::DEFAULT_STRING_SIZE)
                    ),
                    _ => format!("{arg}.serialize_to(fpp_buf, ::fprime_fw::Endianness::Big)"),
                };
                if i + 1 == n {
                    self.line(&expr);
                } else {
                    self.line(&format!("::fprime_fw::fw_try!({expr});"));
                }
            }
        }
        self.dedent();
        self.line("});");
        self.dedent();
        self.line("}");
        if e.throttle.is_some() {
            self.line(&format!(
                "/// Clear the throttle of event `{}` (C++ `log_{}_{}_ThrottleClear`).",
                e.name,
                sev.to_ascii_uppercase(),
                e.name
            ));
            self.line(&format!("pub fn {fname}_throttle_clear(&self) {{"));
            self.line(&format!("    self.throttle_{}.clear();", snake(&e.name)));
            self.line("}");
        }
        Ok(())
    }

    fn emit_tlm_fn(&mut self, _ctx: &Ctx<'_>, t: &crate::analysis::TlmChannelDef) -> Result<()> {
        let f = snake(&t.name);
        let (pt, is_str) = self.helper_param_type(&t.ty)?;
        let on_change = t.update == crate::ast::TlmUpdate::OnChange;
        self.line(&format!("/// Write telemetry channel `{}` (id {:#x}) with the current time (C++ `tlmWrite_{}`).", t.name, t.id, t.name));
        self.line(&format!("pub fn tlm_write_{f}(&self, value: {pt}) {{"));
        self.line("    let time = self.evt.time_get();");
        self.line(&format!("    self.tlm_write_{f}_at(value, time);"));
        self.line("}");
        self.line(&format!(
            "/// Write telemetry channel `{}` with an explicit time tag.",
            t.name
        ));
        self.line(&format!(
            "pub fn tlm_write_{f}_at(&self, value: {pt}, time_tag: ::fprime_fw::Time) {{"
        ));
        self.indent();
        if is_str {
            let size = match self.a.underlying(&t.ty) {
                Type::String(s) => s.unwrap_or(crate::analysis::types::DEFAULT_STRING_SIZE),
                _ => crate::analysis::types::DEFAULT_STRING_SIZE,
            };
            self.line(&format!(
                "let value = ::fprime_fw::FwString::<{size}>::from(value);"
            ));
        }
        // After the string conversion, `value` is owned unless the channel
        // type is passed by shared reference (the same rule as
        // `helper_param_type`: everything but `Copy` primitives/enums).
        let by_ref = !is_str && self.arg_kind(&t.ty) != ArgKind::Copy;
        if on_change {
            self.line("{");
            self.line("    let mut last = self.tlm_last.lock().unwrap();");
            let cmp = if by_ref { "value" } else { "&value" };
            let store = if by_ref || is_str {
                "value.clone()"
            } else {
                "value"
            };
            self.line(&format!(
                "    if last.{}.as_ref() == Some({cmp}) {{ return; }}",
                snake_ident(&t.name)
            ));
            self.line(&format!(
                "    last.{} = Some({store});",
                snake_ident(&t.name)
            ));
            self.line("}");
        }
        let val_ref = if by_ref { "value" } else { "&value" };
        self.line(&format!(
            "self.tlm.tlm_write(self.id_base(), Self::CHANID_{}, {val_ref}, time_tag);",
            f.to_ascii_uppercase()
        ));
        self.dedent();
        self.line("}");
        Ok(())
    }

    fn emit_dp_fns(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let get_port = ctx
            .m
            .special_port(SpecialPortKind::ProductGet)
            .map(|p| snake_ident(p.name()));
        let send_port = ctx
            .m
            .special_port(SpecialPortKind::ProductSend)
            .map(|p| snake_ident(p.name()));
        let req_port = ctx
            .m
            .special_port(SpecialPortKind::ProductRequest)
            .map(|p| snake_ident(p.name()));
        for c in &ctx.m.containers {
            let cn = snake(&c.name);
            let cu = cn.to_ascii_uppercase();
            if let Some(gp) = &get_port {
                self.line(&format!("/// Synchronously get a `{}` container for `data_size` bytes of records (C++ `dpGet_{}`); `None` on failure.", c.name, c.name));
                self.line(&format!("pub fn dp_get_{cn}(&self, data_size: ::fprime_config::FwSizeType) -> Option<::fprime_fw::dp::DpContainer> {{"));
                self.indent();
                self.line(&format!(
                    "let id = self.id_base() + Self::CONTAINER_ID_{cu};"
                ));
                self.line("let mut buffer = ::fprime_fw::Buffer::empty();");
                self.line(&format!("let p = self.{gp}.get();"));
                self.line("let status = p.target.invoke(p.port_num, id, ::fprime_fw::dp::DpContainer::packet_size_for_data_size(data_size), &mut buffer);");
                self.line("if status != ::fprime_fw::Success::Success { return None; }");
                self.line(
                    "let mut container = ::fprime_fw::dp::DpContainer::with_buffer(id, buffer);",
                );
                self.line(&format!(
                    "container.set_priority(Self::CONTAINER_PRIORITY_{cu});"
                ));
                self.line("Some(container)");
                self.dedent();
                self.line("}");
            }
            if let Some(rp) = &req_port {
                self.line(&format!("/// Asynchronously request a `{}` container for `data_size` bytes of records (C++ `dpRequest_{}`).", c.name, c.name));
                self.line(&format!(
                    "pub fn dp_request_{cn}(&self, data_size: ::fprime_config::FwSizeType) {{"
                ));
                self.line(&format!("    let p = self.{rp}.get();"));
                self.line(&format!("    p.target.invoke(p.port_num, self.id_base() + Self::CONTAINER_ID_{cu}, ::fprime_fw::dp::DpContainer::packet_size_for_data_size(data_size));"));
                self.line("}");
            }
        }
        if let Some(sp) = &send_port {
            if !ctx.m.containers.is_empty() {
                self.line(
                    "/// Send a filled container, stamped with the current time (C++ `dpSend`).",
                );
                self.line("pub fn dp_send(&self, container: ::fprime_fw::dp::DpContainer) {");
                self.line("    let time = self.evt.time_get();");
                self.line("    self.dp_send_at(container, time);");
                self.line("}");
                self.line("/// Send a filled container with an explicit time tag; the header is finalized here, the data hash is the writer's job.");
                self.line("pub fn dp_send_at(&self, mut container: ::fprime_fw::dp::DpContainer, time_tag: ::fprime_fw::Time) {");
                self.line("    container.set_time_tag(time_tag);");
                self.line("    container.serialize_header();");
                self.line("    let id = container.id();");
                self.line("    let buffer = container.take_buffer();");
                self.line(&format!("    let p = self.{sp}.get();"));
                self.line("    p.target.invoke(p.port_num, id, buffer);");
                self.line("}");
            }
        }
        for r in &ctx.m.records {
            let rn = snake(&r.name);
            let ru = rn.to_ascii_uppercase();
            let ty = self.rust_type(&r.ty)?;
            if r.is_array {
                self.line(&format!("/// Append a `{}` array record (`[id][count][elements]`; C++ `serializeRecord_{}`).", r.name, r.name));
                self.line(&format!("pub fn serialize_record_{rn}(&self, container: &mut ::fprime_fw::dp::DpContainer, elements: &[{ty}]) -> ::fprime_fw::SerializeStatus {{"));
                self.indent();
                self.line("let start = container.data_size() as usize;");
                self.line("let mut ser = ::fprime_fw::ExtBuf::new(container.data_region_mut());");
                self.line("ser.set_ser_loc(start);");
                self.line(&format!("::fprime_fw::fw_try!(ser.serialize_u32_be(self.id_base() + Self::RECORD_ID_{ru}));"));
                self.line("::fprime_fw::fw_try!(ser.serialize_size(elements.len() as ::fprime_config::FwSizeType, ::fprime_fw::Endianness::Big));");
                self.line("for e in elements { ::fprime_fw::fw_try!(e.serialize_to(&mut ser, ::fprime_fw::Endianness::Big)); }");
                self.line("let end = ser.ser_loc();");
                self.line("container.set_data_size(end as ::fprime_config::FwSizeType);");
                self.line("::fprime_fw::SerializeStatus::Ok");
                self.dedent();
                self.line("}");
            } else {
                let (pt, is_str) = self.helper_param_type(&r.ty)?;
                self.line(&format!(
                    "/// Append a `{}` record (`[id][value]`; C++ `serializeRecord_{}`).",
                    r.name, r.name
                ));
                self.line(&format!("pub fn serialize_record_{rn}(&self, container: &mut ::fprime_fw::dp::DpContainer, value: {pt}) -> ::fprime_fw::SerializeStatus {{"));
                self.indent();
                if is_str {
                    let size = match self.a.underlying(&r.ty) {
                        Type::String(s) => s.unwrap_or(crate::analysis::types::DEFAULT_STRING_SIZE),
                        _ => crate::analysis::types::DEFAULT_STRING_SIZE,
                    };
                    self.line(&format!(
                        "let value = ::fprime_fw::FwString::<{size}>::from(value);"
                    ));
                }
                self.line("let start = container.data_size() as usize;");
                self.line("let mut ser = ::fprime_fw::ExtBuf::new(container.data_region_mut());");
                self.line("ser.set_ser_loc(start);");
                self.line(&format!("::fprime_fw::fw_try!(ser.serialize_u32_be(self.id_base() + Self::RECORD_ID_{ru}));"));
                self.line("::fprime_fw::fw_try!(value.serialize_to(&mut ser, ::fprime_fw::Endianness::Big));");
                self.line("let end = ser.ser_loc();");
                self.line("container.set_data_size(end as ::fprime_config::FwSizeType);");
                self.line("::fprime_fw::SerializeStatus::Ok");
                self.dedent();
                self.line("}");
            }
        }
        Ok(())
    }

    // -- handlers trait ------------------------------------------------------------------------------

    /// Handler parameter list for a set of params (`name: type` strings).
    fn handler_sig(&self, params: &[(ParamDef, String)]) -> Result<Vec<String>> {
        let mut out = Vec::new();
        for (p, mode) in params {
            out.push(format!(
                "{}: {}",
                snake_ident(&p.name),
                self.type_for_mode(&p.ty, mode)?
            ));
        }
        Ok(out)
    }

    fn emit_handlers_trait(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let (handlers, base) = (ctx.handlers.clone(), ctx.base.clone());
        self.line("");
        self.line(&format!("/// What an implementation of `{}` provides: access to its base and the handlers the autocoder calls.", ctx.name));
        self.line(&format!("pub trait {handlers}: Send + Sync + 'static {{"));
        self.indent();
        self.line("/// The generated base embedded in the implementation.");
        self.line(&format!("fn base(&self) -> &{base};"));
        // General input ports.
        for p in &ctx.m.ports {
            if let PortInstance::General {
                name,
                kind,
                port: PortType::Typed(psym),
                docs,
                ..
            } = p
            {
                if *kind == GeneralPortKind::Output {
                    continue;
                }
                let def = self.a.ports[psym].clone();
                let params = self.port_params(*psym);
                let sig = self.handler_sig(&params)?;
                let ret = match &def.ret {
                    Some(t) => format!(" -> {}", self.rust_type(t)?),
                    None => String::new(),
                };
                self.doc(docs);
                self.line(&format!("/// Handler for {kind} port `{name}`."));
                if def.params.len() > 6 {
                    self.line("#[allow(clippy::too_many_arguments)]");
                }
                let sigs = if sig.is_empty() {
                    String::new()
                } else {
                    format!(", {}", sig.join(", "))
                };
                self.line(&format!(
                    "fn {}_handler(&self, port_num: ::fprime_config::FwIndexType{sigs}){ret};",
                    snake(name)
                ));
                if *kind == GeneralPortKind::AsyncInput {
                    let sigs2 = sig.iter().map(|s| format!("_{s}")).collect::<Vec<_>>();
                    let sigs2 = if sigs2.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", sigs2.join(", "))
                    };
                    self.line(&format!("/// Runs on the sender's thread before `{name}` is queued (C++ `{name}_preMsgHook`)."));
                    self.line(&format!("fn {}_pre_msg_hook(&self, _port_num: ::fprime_config::FwIndexType{sigs2}) {{}}", snake(name)));
                    self.line(&format!("/// Called when the queue is full and `{name}` has the `hook` policy (C++ `{name}_overflowHook`)."));
                    self.line(&format!("fn {}_overflow_hook(&self, _port_num: ::fprime_config::FwIndexType{sigs2}) {{}}", snake(name)));
                }
            }
        }
        // Commands.
        for c in &ctx.m.commands {
            if c.param_cmd.is_some() {
                continue;
            }
            let sig = self.handler_sig(&self.plain_params(&c.params))?;
            self.doc(&c.docs);
            self.line(&format!("/// Handler for {} command `{}` (opcode {:#x}); must answer through `base().cmd.cmd_response`.", match c.kind { CommandKind::Async => "async", CommandKind::Guarded => "guarded", CommandKind::Sync => "sync" }, c.name, c.opcode));
            if c.params.len() > 5 {
                self.line("#[allow(clippy::too_many_arguments)]");
            }
            let sigs = if sig.is_empty() {
                String::new()
            } else {
                format!(", {}", sig.join(", "))
            };
            self.line(&format!("fn {}_cmd_handler(&self, op_code: ::fprime_config::FwOpcodeType, cmd_seq: u32{sigs});", snake(&c.name)));
        }
        // Product recv.
        if let Some((pname, _)) = &ctx.dp_recv {
            let params = self.dp_response_params()?;
            let sig = self.handler_sig(&params)?;
            self.line(&format!("/// Handler for product receive port `{pname}`."));
            let sigs = if sig.is_empty() {
                String::new()
            } else {
                format!(", {}", sig.join(", "))
            };
            self.line(&format!(
                "fn {}_handler(&self, port_num: ::fprime_config::FwIndexType{sigs});",
                snake(pname)
            ));
        }
        // Internal ports.
        for ip in &ctx.m.internal_ports {
            let sig = self.handler_sig(&self.plain_params(&ip.params))?;
            self.doc(&ip.docs);
            self.line(&format!(
                "/// Handler for internal port `{}` (C++ `{}_internalInterfaceHandler`).",
                ip.name, ip.name
            ));
            let sigs = if sig.is_empty() {
                String::new()
            } else {
                format!(", {}", sig.join(", "))
            };
            self.line(&format!(
                "fn {}_internal_interface_handler(&self{sigs});",
                snake(&ip.name)
            ));
        }
        // Parameter hooks.
        if !ctx.m.params.is_empty() {
            self.line(
                "/// Called after a `PARAM_SET` staged a new value (C++ `parameterUpdated`).",
            );
            self.line("fn parameter_updated(&self, _id: ::fprime_config::FwPrmIdType) {}");
            self.line("/// Called after `load_parameters` (C++ `parametersLoaded`).");
            self.line("fn parameters_loaded(&self) {}");
        }
        self.dedent();
        self.line("}");
        Ok(())
    }

    // -- component trait ------------------------------------------------------------------------------

    fn emit_component_trait(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let (handlers, comp, base) = (ctx.handlers.clone(), ctx.comp.clone(), ctx.base.clone());
        self.line("");
        self.line(&format!("/// Generated operations on any `{}` implementation: input-port factories, internal-port invocations, parameter loading and queue dispatch.", ctx.name));
        self.line(&format!("pub trait {comp}: {handlers} + Sized {{"));
        self.indent();
        // Input port factories.
        for p in &ctx.m.ports {
            match p {
                PortInstance::General {
                    name,
                    kind,
                    port: PortType::Typed(psym),
                    ..
                } if *kind != GeneralPortKind::Output => {
                    let trait_path = self.port_path(*psym);
                    let adapter =
                        format!("{}{}Adapter", ctx.name, super::names::pascal(&snake(name)));
                    self.line(&format!("/// Input port `{name}` ({kind}): a port reference to hand to a connection."));
                    self.line(&format!("fn {}(self: &::std::sync::Arc<Self>, port_num: ::fprime_config::FwIndexType) -> ::fprime_comp::PortRef<dyn {trait_path}> {{", snake_ident(name)));
                    self.line(&format!("    ::fprime_comp::PortRef::new(::std::sync::Arc::new({adapter} {{ comp: ::std::sync::Arc::clone(self) }}), port_num)"));
                    self.line("}");
                }
                PortInstance::Special {
                    kind: SpecialPortKind::CommandRecv,
                    name,
                    ..
                } => {
                    let adapter = format!("{}CmdInAdapter", ctx.name);
                    self.line(&format!("/// Command input port `{name}`."));
                    self.line(&format!("fn {}(self: &::std::sync::Arc<Self>, port_num: ::fprime_config::FwIndexType) -> ::fprime_comp::PortRef<dyn ::fprime_comp::CmdPort> {{", snake_ident(name)));
                    self.line(&format!("    ::fprime_comp::PortRef::new(::std::sync::Arc::new({adapter} {{ comp: ::std::sync::Arc::clone(self) }}), port_num)"));
                    self.line("}");
                }
                PortInstance::Special {
                    kind: SpecialPortKind::ProductRecv,
                    name,
                    ..
                } => {
                    let adapter = format!("{}ProductRecvInAdapter", ctx.name);
                    self.line(&format!("/// Product receive port `{name}`."));
                    self.line(&format!("fn {}(self: &::std::sync::Arc<Self>, port_num: ::fprime_config::FwIndexType) -> ::fprime_comp::PortRef<dyn ::fprime_fw::dp::DpResponsePort> {{", snake_ident(name)));
                    self.line(&format!("    ::fprime_comp::PortRef::new(::std::sync::Arc::new({adapter} {{ comp: ::std::sync::Arc::clone(self) }}), port_num)"));
                    self.line("}");
                }
                _ => {}
            }
        }
        // Internal port invocations.
        for ip in &ctx.m.internal_ports {
            let msg = ctx
                .msgs
                .iter()
                .find(|m| matches!(&m.kind, MsgKind::Internal(n) if *n == ip.name))
                .expect("internal msg")
                .clone();
            let sig = self.handler_sig(&msg.params)?;
            let sigs = if sig.is_empty() {
                String::new()
            } else {
                format!(", {}", sig.join(", "))
            };
            self.line(&format!(
                "/// Queue a message for internal port `{}` (C++ `{}_internalInterfaceInvoke`).",
                ip.name, ip.name
            ));
            self.line(&format!(
                "fn {}_internal_interface_invoke(&self{sigs}) {{",
                snake(&ip.name)
            ));
            self.indent();
            let params = msg.params.clone();
            self.emit_enqueue(ctx, &msg, "self", "0", &params, None)?;
            self.dedent();
            self.line("}");
        }
        // load_parameters
        if !ctx.m.params.is_empty() {
            self.line("/// Load every parameter from the parameter database (C++ `loadParameters`), then call `parameters_loaded`.");
            self.line("fn load_parameters(&self) {");
            self.indent();
            self.line("let fpp_base = self.base();");
            for p in &ctx.m.params {
                let f = snake_ident(&p.name);
                let fv = format!("{}_valid", snake(&p.name));
                let pu = snake(&p.name).to_ascii_uppercase();
                let has_default = p.default.is_some();
                self.line("{");
                self.indent();
                self.line("let mut fpp_buf = ::fprime_fw::ParamBuffer::new();");
                self.line(&format!("let fpp_valid = fpp_base.prm.get_param(fpp_base.id_base(), {base}::PARAMID_{pu}, &mut fpp_buf);"));
                self.line("let mut fpp_ps = fpp_base.params.lock().unwrap();");
                self.line("match fpp_valid {");
                self.line(
                    "    ::fprime_fw::ParamValid::Valid | ::fprime_fw::ParamValid::Default => {",
                );
                self.line(&format!("        let mut value = fpp_ps.{f}.clone();"));
                self.line("        if fpp_buf.deserialize(&mut value, ::fprime_fw::Endianness::Big).is_ok() {");
                self.line(&format!("            fpp_ps.{f} = value;"));
                self.line(&format!("            fpp_ps.{fv} = fpp_valid;"));
                self.line("        } else {");
                self.line(&format!(
                    "            fpp_ps.{fv} = ::fprime_fw::ParamValid::Invalid;"
                ));
                self.line("        }");
                self.line("    }");
                self.line("    _ => {");
                if has_default {
                    self.line(&format!(
                        "        fpp_ps.{fv} = ::fprime_fw::ParamValid::Default;"
                    ));
                } else {
                    self.line(&format!(
                        "        fpp_ps.{fv} = ::fprime_fw::ParamValid::Invalid;"
                    ));
                }
                self.line("    }");
                self.line("}");
                self.dedent();
                self.line("}");
            }
            self.line("self.parameters_loaded();");
            self.dedent();
            self.line("}");
        }
        // cmd_dispatch
        if ctx.has_cmd && !ctx.m.commands.is_empty() {
            self.emit_cmd_dispatch(ctx)?;
        }
        // dispatch_message
        if !ctx.msgs.is_empty() {
            self.emit_dispatch_message(ctx)?;
        }
        self.dedent();
        self.line("}");
        self.line(&format!("impl<T: {handlers}> {comp} for T {{}}"));
        Ok(())
    }

    /// Emit the code that builds and sends a queue message for `msg`
    /// carrying `params` (argument identifiers already bound), from an
    /// expression `comp` for the component (`self` or `self.comp`) and
    /// `port_num` expression.
    fn emit_enqueue(
        &mut self,
        ctx: &Ctx<'_>,
        msg: &AsyncMsg,
        comp: &str,
        port_num: &str,
        params: &[(ParamDef, String)],
        pre_hook: Option<&str>,
    ) -> Result<()> {
        let base = ctx.base.clone();
        let q = match ctx.kind {
            ComponentKind::Queued => "queued",
            ComponentKind::Active => "active.queued",
            ComponentKind::Passive => unreachable!("passive components have no queue"),
        };
        if let Some(hook) = pre_hook {
            let args: Vec<String> = params.iter().map(|(p, m)| Self::hook_arg(p, m)).collect();
            let args = if args.is_empty() {
                String::new()
            } else {
                format!(", {}", args.join(", "))
            };
            self.line(&format!("{comp}.{hook}({port_num}{args});"));
        }
        self.line(&format!(
            "let mut fpp_msg = ::fprime_fw::LinearBuffer::<{{ {base}::MSG_SIZE }}>::new();"
        ));
        self.line(&format!("let fpp_status = ::fprime_comp::msg::write_envelope_header(&mut fpp_msg, {base}::{}, {port_num});", msg.const_name));
        self.line("::fprime_fw::fw_assert!(fpp_status.is_ok(), fpp_status as i32);");
        for (p, mode) in params {
            let arg = snake_ident(&p.name);
            let expr = match mode.as_str() {
                "owned" => format!("fpp_msg.serialize_u64_be({comp}.base().escrow.deposit({arg}))"),
                "buf" => format!("fpp_msg.serialize_buffer({arg}, ::fprime_fw::Endianness::Big)"),
                "ref" => format!("fpp_msg.serialize({arg}, ::fprime_fw::Endianness::Big)"),
                "mut" => format!("fpp_msg.serialize(&*{arg}, ::fprime_fw::Endianness::Big)"),
                _ => format!("fpp_msg.serialize(&{arg}, ::fprime_fw::Endianness::Big)"),
            };
            self.line(&format!("let fpp_status = {expr};"));
            self.line("::fprime_fw::fw_assert!(fpp_status.is_ok(), fpp_status as i32);");
        }
        let policy = match msg.queue_full {
            QueueFull::Assert => "Assert",
            QueueFull::Block => "Block",
            QueueFull::Drop => "Drop",
            QueueFull::Hook => "Hook",
        };
        self.line(&format!(
            "let fpp_send_status = {comp}.base().{q}.send_message(&fpp_msg, {} as ::fprime_config::FwQueuePriorityType, ::fprime_comp::QueueFullPolicy::{policy});",
            msg.priority
        ));
        if msg.queue_full == QueueFull::Hook {
            if let MsgKind::Port(name) = &msg.kind {
                let args: Vec<String> = params.iter().map(|(p, m)| Self::hook_arg(p, m)).collect();
                let args = if args.is_empty() {
                    String::new()
                } else {
                    format!(", {}", args.join(", "))
                };
                self.line("if fpp_send_status == ::fprime_os::queue::Status::Full {");
                self.line(&format!(
                    "    {comp}.{}_overflow_hook({port_num}{args});",
                    snake(name)
                ));
                self.line("}");
            } else {
                self.line("let _ = fpp_send_status;");
            }
        } else {
            self.line("let _ = fpp_send_status;");
        }
        Ok(())
    }

    /// How a bound argument identifier is passed to a hook after being
    /// serialized (an owned buffer is in escrow by then, so hooks see an
    /// empty one).
    fn hook_arg(p: &ParamDef, mode: &str) -> String {
        let arg = snake_ident(&p.name);
        match mode {
            "owned" => "::fprime_fw::Buffer::empty()".into(),
            "buf" | "mut" => format!("&mut *{arg}"),
            _ => arg,
        }
    }

    /// Emit deserialization of `params` from `msg` into local bindings.
    /// Returns the call arguments.
    fn emit_deser_args(
        &mut self,
        base: &str,
        params: &[(ParamDef, String)],
    ) -> Result<Vec<String>> {
        let mut call = Vec::new();
        for (p, mode) in params {
            let arg = snake_ident(&p.name);
            match mode.as_str() {
                "owned" => {
                    self.line("let mut fpp_token = 0u64;");
                    self.line("if !fpp_msg.deserialize_u64_be(&mut fpp_token).is_ok() { return ::fprime_comp::MsgDispatchStatus::Error; }");
                    self.line(&format!("let {arg} = {base}.escrow.claim(fpp_token);"));
                    call.push(arg);
                }
                "buf" => {
                    let t = self.rust_type(&p.ty)?;
                    self.line(&format!("let mut {arg} = <{t} as Default>::default();"));
                    self.line(&format!("if !fpp_msg.deserialize_buffer(&mut {arg}, ::fprime_fw::Endianness::Big).is_ok() {{ return ::fprime_comp::MsgDispatchStatus::Error; }}"));
                    call.push(format!("&mut {arg}"));
                }
                m => {
                    let t = self.rust_type(&p.ty)?;
                    self.line(&format!("let mut {arg} = <{t} as Default>::default();"));
                    self.line(&format!("if !fpp_msg.deserialize(&mut {arg}, ::fprime_fw::Endianness::Big).is_ok() {{ return ::fprime_comp::MsgDispatchStatus::Error; }}"));
                    call.push(match m {
                        "mut" => format!("&mut {arg}"),
                        "ref" => format!("&{arg}"),
                        _ => arg,
                    });
                }
            }
        }
        Ok(call)
    }

    fn emit_dispatch_message(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let base = ctx.base.clone();
        self.line("/// Decode one queue message and call its handler (the C++ `doDispatch` body); wire it to `ComponentDispatch` with the `impl_..._component!` macro.");
        self.line("fn dispatch_message(&self, fpp_msg_type: ::fprime_config::FwEnumStoreType, fpp_msg: &mut dyn ::fprime_fw::SerBufAny) -> ::fprime_comp::MsgDispatchStatus {");
        self.indent();
        self.line("let fpp_base = self.base();");
        self.line("let mut port_num: ::fprime_config::FwIndexType = 0;");
        self.line("if !::fprime_comp::msg::read_port_num(fpp_msg, &mut port_num).is_ok() { return ::fprime_comp::MsgDispatchStatus::Error; }");
        self.line("match fpp_msg_type {");
        self.indent();
        for msg in &ctx.msgs.clone() {
            self.line(&format!("{base}::{} => {{", msg.const_name));
            self.indent();
            match &msg.kind {
                MsgKind::Port(name) => {
                    let call = self.emit_deser_args("fpp_base", &msg.params)?;
                    let calls = if call.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", call.join(", "))
                    };
                    self.line(&format!("self.{}_handler(port_num{calls});", snake(name)));
                }
                MsgKind::Cmd => {
                    self.line("let mut fpp_op_code: ::fprime_config::FwOpcodeType = 0;");
                    self.line("let mut fpp_cmd_seq = 0u32;");
                    self.line("let mut fpp_args = ::fprime_fw::CmdArgBuffer::new();");
                    self.line("if !fpp_msg.deserialize_u32_be(&mut fpp_op_code).is_ok() || !fpp_msg.deserialize_u32_be(&mut fpp_cmd_seq).is_ok() || !fpp_msg.deserialize_buffer(&mut fpp_args, ::fprime_fw::Endianness::Big).is_ok() { return ::fprime_comp::MsgDispatchStatus::Error; }");
                    self.line("self.cmd_dispatch(fpp_op_code, fpp_cmd_seq, &mut fpp_args);");
                }
                MsgKind::ProductRecv(name) => {
                    let call = self.emit_deser_args("fpp_base", &msg.params)?;
                    let calls = if call.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", call.join(", "))
                    };
                    self.line(&format!("self.{}_handler(port_num{calls});", snake(name)));
                }
                MsgKind::Internal(name) => {
                    let call = self.emit_deser_args("fpp_base", &msg.params)?;
                    self.line("let _ = port_num;");
                    self.line(&format!(
                        "self.{}_internal_interface_handler({});",
                        snake(name),
                        call.join(", ")
                    ));
                }
            }
            self.line("::fprime_comp::MsgDispatchStatus::Ok");
            self.dedent();
            self.line("}");
        }
        self.line("_ => ::fprime_comp::MsgDispatchStatus::Error,");
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");
        Ok(())
    }

    fn emit_cmd_dispatch(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let base = ctx.base.clone();
        self.line("/// Deserialize a command's arguments and call its handler (the autocoded command dispatch). Answers `FORMAT_ERROR` on bad arguments and `INVALID_OPCODE` for unknown opcodes; parameter set/save commands are handled here.");
        self.line("fn cmd_dispatch(&self, op_code: ::fprime_config::FwOpcodeType, cmd_seq: u32, fpp_args: &mut ::fprime_fw::CmdArgBuffer) {");
        self.indent();
        self.line("let fpp_base = self.base();");
        self.line("match op_code.wrapping_sub(fpp_base.id_base()) {");
        self.indent();
        for c in &ctx.m.commands {
            let cu = snake(&c.name).to_ascii_uppercase();
            self.line(&format!("{base}::OPCODE_{cu} => {{"));
            self.indent();
            // Deserialize params.
            let mut call = Vec::new();
            for (p, mode) in self.plain_params(&c.params) {
                let arg = snake_ident(&p.name);
                let t = self.rust_type(&p.ty)?;
                self.line(&format!("let mut {arg} = <{t} as Default>::default();"));
                self.line(&format!("if !fpp_args.deserialize(&mut {arg}, ::fprime_fw::Endianness::Big).is_ok() {{ fpp_base.cmd.cmd_response(op_code, cmd_seq, ::fprime_fw::CmdResponse::FormatError); return; }}"));
                call.push(match mode.as_str() {
                    "buf" | "mut" => format!("&mut {arg}"),
                    "ref" => format!("&{arg}"),
                    _ => arg,
                });
            }
            self.line("if fpp_args.deserialize_size_left() != 0 { fpp_base.cmd.cmd_response(op_code, cmd_seq, ::fprime_fw::CmdResponse::FormatError); return; }");
            match c.param_cmd {
                Some((idx, true)) => {
                    let p = &ctx.m.params[idx];
                    let f = snake_ident(&p.name);
                    let pu = snake(&p.name).to_ascii_uppercase();
                    self.line("{");
                    self.line("    let mut fpp_ps = fpp_base.params.lock().unwrap();");
                    self.line(&format!("    fpp_ps.{f} = val;"));
                    self.line(&format!(
                        "    fpp_ps.{}_valid = ::fprime_fw::ParamValid::Valid;",
                        snake(&p.name)
                    ));
                    self.line("}");
                    self.line(&format!("self.parameter_updated({base}::PARAMID_{pu});"));
                    self.line(
                        "fpp_base.cmd.cmd_response(op_code, cmd_seq, ::fprime_fw::CmdResponse::Ok);",
                    );
                }
                Some((idx, false)) => {
                    let p = &ctx.m.params[idx];
                    let f = snake_ident(&p.name);
                    let pu = snake(&p.name).to_ascii_uppercase();
                    self.line("if !fpp_base.prm.prm_set_out.is_connected() { fpp_base.cmd.cmd_response(op_code, cmd_seq, ::fprime_fw::CmdResponse::ExecutionError); return; }");
                    self.line("let mut fpp_buf = ::fprime_fw::ParamBuffer::new();");
                    self.line(&format!(
                        "let fpp_value = fpp_base.params.lock().unwrap().{f}.clone();"
                    ));
                    self.line(
                        "let fpp_status = fpp_value.serialize_to(&mut fpp_buf, ::fprime_fw::Endianness::Big);",
                    );
                    self.line("::fprime_fw::fw_assert!(fpp_status.is_ok(), fpp_status as i32);");
                    self.line(&format!(
                        "fpp_base.prm.set_param(fpp_base.id_base(), {base}::PARAMID_{pu}, &mut fpp_buf);"
                    ));
                    self.line(
                        "fpp_base.cmd.cmd_response(op_code, cmd_seq, ::fprime_fw::CmdResponse::Ok);",
                    );
                }
                None => {
                    if c.kind == CommandKind::Guarded {
                        self.line("let _fpp_guard = fpp_base.guard.lock().unwrap();");
                    }
                    let calls = if call.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", call.join(", "))
                    };
                    self.line(&format!(
                        "self.{}_cmd_handler(op_code, cmd_seq{calls});",
                        snake(&c.name)
                    ));
                }
            }
            self.dedent();
            self.line("}");
        }
        self.line("_ => fpp_base.cmd.cmd_response(op_code, cmd_seq, ::fprime_fw::CmdResponse::InvalidOpcode),");
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");
        Ok(())
    }

    // -- adapters -----------------------------------------------------------------------------------------

    fn emit_adapters(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let handlers = ctx.handlers.clone();
        for p in &ctx.m.ports.clone() {
            match p {
                PortInstance::General {
                    name,
                    kind,
                    port: PortType::Typed(psym),
                    ..
                } if *kind != GeneralPortKind::Output => {
                    let trait_path = self.port_path(*psym);
                    let def = self.a.ports[psym].clone();
                    let params = self.port_params(*psym);
                    let adapter =
                        format!("{}{}Adapter", ctx.name, super::names::pascal(&snake(name)));
                    let sig = self.handler_sig(&params)?;
                    let ret = match &def.ret {
                        Some(t) => format!(" -> {}", self.rust_type(t)?),
                        None => String::new(),
                    };
                    self.line("");
                    self.line(&format!(
                        "/// Adapter of input port `{name}` (the C++ static thunk)."
                    ));
                    self.line(&format!(
                        "pub struct {adapter}<C: {handlers}> {{ comp: ::std::sync::Arc<C> }}"
                    ));
                    self.line(&format!(
                        "impl<C: {handlers}> {trait_path} for {adapter}<C> {{"
                    ));
                    self.indent();
                    if def.params.len() > 6 {
                        self.line("#[allow(clippy::too_many_arguments)]");
                    }
                    let sigs = if sig.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", sig.join(", "))
                    };
                    self.line(&format!(
                        "fn invoke(&self, port_num: ::fprime_config::FwIndexType{sigs}){ret} {{"
                    ));
                    self.indent();
                    let call: Vec<String> =
                        params.iter().map(|(p, _)| snake_ident(&p.name)).collect();
                    let calls = if call.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", call.join(", "))
                    };
                    match kind {
                        GeneralPortKind::SyncInput => {
                            self.line(&format!(
                                "self.comp.{}_handler(port_num{calls})",
                                snake(name)
                            ));
                        }
                        GeneralPortKind::GuardedInput => {
                            self.line("let _fpp_guard = self.comp.base().guard.lock().unwrap();");
                            self.line(&format!(
                                "self.comp.{}_handler(port_num{calls})",
                                snake(name)
                            ));
                        }
                        GeneralPortKind::AsyncInput => {
                            let msg = ctx
                                .msgs
                                .iter()
                                .find(|m| matches!(&m.kind, MsgKind::Port(n) if n == name))
                                .expect("async msg")
                                .clone();
                            let hook = format!("{}_pre_msg_hook", snake(name));
                            self.emit_enqueue(
                                ctx,
                                &msg,
                                "self.comp",
                                "port_num",
                                &params,
                                Some(&hook),
                            )?;
                        }
                        GeneralPortKind::Output => unreachable!(),
                    }
                    self.dedent();
                    self.line("}");
                    self.dedent();
                    self.line("}");
                }
                PortInstance::Special {
                    kind: SpecialPortKind::CommandRecv,
                    name,
                    ..
                } => {
                    let adapter = format!("{}CmdInAdapter", ctx.name);
                    let base = ctx.base.clone();
                    self.line("");
                    self.line(&format!("/// Adapter of command port `{name}`: async commands are queued, sync and guarded ones run on the caller's thread."));
                    self.line(&format!(
                        "pub struct {adapter}<C: {handlers}> {{ comp: ::std::sync::Arc<C> }}"
                    ));
                    self.line(&format!(
                        "impl<C: {handlers}> ::fprime_comp::CmdPort for {adapter}<C> {{"
                    ));
                    self.indent();
                    self.line("fn invoke(&self, port_num: ::fprime_config::FwIndexType, op_code: ::fprime_config::FwOpcodeType, cmd_seq: u32, args: &mut ::fprime_fw::CmdArgBuffer) {");
                    self.indent();
                    self.line("let fpp_base = self.comp.base();");
                    let async_ops: Vec<String> = ctx
                        .m
                        .commands
                        .iter()
                        .filter(|c| c.kind == CommandKind::Async)
                        .map(|c| format!("{base}::OPCODE_{}", snake(&c.name).to_ascii_uppercase()))
                        .collect();
                    let sync_ops: Vec<String> = ctx
                        .m
                        .commands
                        .iter()
                        .filter(|c| c.kind != CommandKind::Async)
                        .map(|c| format!("{base}::OPCODE_{}", snake(&c.name).to_ascii_uppercase()))
                        .collect();
                    self.line("match op_code.wrapping_sub(fpp_base.id_base()) {");
                    self.indent();
                    if !async_ops.is_empty() {
                        let msg = ctx
                            .msgs
                            .iter()
                            .find(|m| matches!(m.kind, MsgKind::Cmd))
                            .expect("cmd msg")
                            .clone();
                        self.line(&format!("{} => {{", async_ops.join(" | ")));
                        self.indent();
                        self.line(&format!("let mut fpp_msg = ::fprime_fw::LinearBuffer::<{{ {base}::MSG_SIZE }}>::new();"));
                        self.line(&format!("let fpp_status = ::fprime_comp::msg::write_envelope_header(&mut fpp_msg, {base}::{}, port_num);", msg.const_name));
                        self.line(
                            "::fprime_fw::fw_assert!(fpp_status.is_ok(), fpp_status as i32);",
                        );
                        self.line("let fpp_status = fpp_msg.serialize_u32_be(op_code);");
                        self.line(
                            "::fprime_fw::fw_assert!(fpp_status.is_ok(), fpp_status as i32);",
                        );
                        self.line("let fpp_status = fpp_msg.serialize_u32_be(cmd_seq);");
                        self.line(
                            "::fprime_fw::fw_assert!(fpp_status.is_ok(), fpp_status as i32);",
                        );
                        self.line("let fpp_status = fpp_msg.serialize_buffer(args, ::fprime_fw::Endianness::Big);");
                        self.line(
                            "::fprime_fw::fw_assert!(fpp_status.is_ok(), fpp_status as i32);",
                        );
                        let q = match ctx.kind {
                            ComponentKind::Queued => "queued",
                            _ => "active.queued",
                        };
                        let policy = match msg.queue_full {
                            QueueFull::Assert => "Assert",
                            QueueFull::Block => "Block",
                            QueueFull::Drop => "Drop",
                            QueueFull::Hook => "Hook",
                        };
                        self.line(&format!("let _ = fpp_base.{q}.send_message(&fpp_msg, {} as ::fprime_config::FwQueuePriorityType, ::fprime_comp::QueueFullPolicy::{policy});", msg.priority));
                        self.dedent();
                        self.line("}");
                    }
                    if !sync_ops.is_empty() {
                        self.line(&format!(
                            "{} => self.comp.cmd_dispatch(op_code, cmd_seq, args),",
                            sync_ops.join(" | ")
                        ));
                    }
                    self.line("_ => fpp_base.cmd.cmd_response(op_code, cmd_seq, ::fprime_fw::CmdResponse::InvalidOpcode),");
                    self.dedent();
                    self.line("}");
                    self.dedent();
                    self.line("}");
                    self.dedent();
                    self.line("}");
                }
                PortInstance::Special {
                    kind: SpecialPortKind::ProductRecv,
                    name,
                    input_kind,
                    ..
                } => {
                    let adapter = format!("{}ProductRecvInAdapter", ctx.name);
                    let params = self.dp_response_params()?;
                    let sig = self.handler_sig(&params)?;
                    let sigs = if sig.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", sig.join(", "))
                    };
                    self.line("");
                    self.line(&format!("/// Adapter of product receive port `{name}`."));
                    self.line(&format!(
                        "pub struct {adapter}<C: {handlers}> {{ comp: ::std::sync::Arc<C> }}"
                    ));
                    self.line(&format!(
                        "impl<C: {handlers}> ::fprime_fw::dp::DpResponsePort for {adapter}<C> {{"
                    ));
                    self.indent();
                    self.line(&format!(
                        "fn invoke(&self, port_num: ::fprime_config::FwIndexType{sigs}) {{"
                    ));
                    self.indent();
                    let call: Vec<String> =
                        params.iter().map(|(p, _)| snake_ident(&p.name)).collect();
                    let calls = if call.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", call.join(", "))
                    };
                    match input_kind.unwrap_or(SpecialInputKind::Async) {
                        SpecialInputKind::Sync => self.line(&format!(
                            "self.comp.{}_handler(port_num{calls})",
                            snake(name)
                        )),
                        SpecialInputKind::Guarded => {
                            self.line("let _fpp_guard = self.comp.base().guard.lock().unwrap();");
                            self.line(&format!(
                                "self.comp.{}_handler(port_num{calls})",
                                snake(name)
                            ));
                        }
                        SpecialInputKind::Async => {
                            let msg = ctx
                                .msgs
                                .iter()
                                .find(|m| matches!(m.kind, MsgKind::ProductRecv(_)))
                                .expect("recv msg")
                                .clone();
                            self.emit_enqueue(ctx, &msg, "self.comp", "port_num", &params, None)?;
                        }
                    }
                    self.dedent();
                    self.line("}");
                    self.dedent();
                    self.line("}");
                }
                _ => {}
            }
        }
        Ok(())
    }

    // -- impl macro ------------------------------------------------------------------------------------------

    fn emit_impl_macro(&mut self, ctx: &Ctx<'_>) -> Result<()> {
        let mac = format!("impl_{}_component", snake(&ctx.name));
        // With a known include path the macro names its traits absolutely;
        // otherwise the traits must be in scope where it is invoked.
        let (comp, handlers) = match &self.opts.include_path {
            Some(inc) => {
                let mut prefix = format!("$crate::{inc}");
                for part in self.module_path() {
                    prefix.push_str("::");
                    prefix.push_str(&super::names::ident(part));
                }
                (
                    format!("{prefix}::{}", ctx.comp),
                    format!("{prefix}::{}", ctx.handlers),
                )
            }
            None => (ctx.comp.clone(), ctx.handlers.clone()),
        };
        self.line("");
        self.line(&format!("/// Implement the framework traits for an implementation of `{}` in one line: `{mac}!(MyImpl);`.", ctx.name));
        if self.opts.include_path.is_none() {
            self.line(&format!(
                "/// `{}` and `{}` must be in scope at the invocation.",
                ctx.handlers, ctx.comp
            ));
        }
        self.line(&format!("macro_rules! {mac} {{"));
        self.indent();
        self.line("($t:ty) => {");
        self.indent();
        if !ctx.msgs.is_empty() {
            self.line("impl ::fprime_comp::ComponentDispatch for $t {");
            self.line("    fn dispatch_message(&self, fpp_msg_type: ::fprime_config::FwEnumStoreType, fpp_msg: &mut dyn ::fprime_fw::SerBufAny) -> ::fprime_comp::MsgDispatchStatus {");
            self.line(&format!(
                "        {comp}::dispatch_message(self, fpp_msg_type, fpp_msg)"
            ));
            self.line("    }");
            self.line("}");
        } else if ctx.kind != ComponentKind::Passive {
            self.line("impl ::fprime_comp::ComponentDispatch for $t {");
            self.line("    fn dispatch_message(&self, _msg_type: ::fprime_config::FwEnumStoreType, _msg: &mut dyn ::fprime_fw::SerBufAny) -> ::fprime_comp::MsgDispatchStatus {");
            self.line("        ::fprime_comp::MsgDispatchStatus::Error");
            self.line("    }");
            self.line("}");
        }
        if ctx.kind == ComponentKind::Active {
            self.line("impl ::fprime_comp::ActiveComponent for $t {");
            self.line("    fn active_base(&self) -> &::fprime_comp::ActiveBase {");
            self.line(&format!("        &{handlers}::base(self).active"));
            self.line("    }");
            self.line("}");
        }
        self.dedent();
        self.line("};");
        self.dedent();
        self.line("}");
        self.line(&format!("pub(crate) use {mac};"));
        let _ = ident;
        Ok(())
    }
}

/// Convenience used by the topology back end: the base field holding the
/// kind-specific core.
pub fn core_field(kind: ComponentKind) -> &'static str {
    match kind {
        ComponentKind::Passive => "base",
        ComponentKind::Queued => "queued",
        ComponentKind::Active => "active",
    }
}

/// The C++-style name of a command def (used in messages).
pub fn command_display(c: &CommandDef) -> String {
    format!("{} ({:#x})", c.name, c.opcode)
}
