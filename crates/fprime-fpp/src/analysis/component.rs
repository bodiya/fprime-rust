//! Component, interface and component-instance models
//! (`Component.scala`, `PortInstance.scala`, `Command.scala`, `Event.scala`,
//! `TlmChannel.scala`, `Param.scala`, `Container.scala`, `Record.scala`,
//! `ComponentInstance.scala` in the reference compiler).
//!
//! Implicit numbering follows the reference exactly: each kind of id has a
//! "next default" counter that an explicit id resets to `id + 1`; a
//! parameter consumes two opcodes (set, then save) from the *command*
//! opcode counter at the point where it is declared.

use super::format::Format;
use super::symbols::{Def, NameGroup, ScopeId, SymId};
use super::types::{Type, Value};
use super::{Analysis, ParamDef};
use crate::ast::*;
use crate::error::{Diagnostic, Loc, Result};

/// The type of a general port instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortType {
    /// A typed port: the port definition symbol.
    Typed(SymId),
    /// A `serial` port.
    Serial,
}

/// A port instance of a component or interface.
#[derive(Debug, Clone)]
pub enum PortInstance {
    /// A general (typed or serial) port.
    General {
        name: String,
        kind: GeneralPortKind,
        size: u64,
        port: PortType,
        priority: Option<i128>,
        queue_full: QueueFull,
        docs: Vec<String>,
        loc: Loc,
    },
    /// A special (framework-role) port.
    Special {
        name: String,
        kind: SpecialPortKind,
        input_kind: Option<SpecialInputKind>,
        priority: Option<i128>,
        queue_full: QueueFull,
        docs: Vec<String>,
        loc: Loc,
    },
}

impl PortInstance {
    /// The instance name.
    pub fn name(&self) -> &str {
        match self {
            PortInstance::General { name, .. } | PortInstance::Special { name, .. } => name,
        }
    }

    /// The location.
    pub fn loc(&self) -> &Loc {
        match self {
            PortInstance::General { loc, .. } | PortInstance::Special { loc, .. } => loc,
        }
    }

    /// Whether this port receives (any input kind).
    pub fn is_input(&self) -> bool {
        match self {
            PortInstance::General { kind, .. } => *kind != GeneralPortKind::Output,
            PortInstance::Special { kind, .. } => matches!(
                kind,
                SpecialPortKind::CommandRecv | SpecialPortKind::ProductRecv
            ),
        }
    }

    /// Whether this port is async (queued).
    pub fn is_async(&self) -> bool {
        match self {
            PortInstance::General { kind, .. } => *kind == GeneralPortKind::AsyncInput,
            PortInstance::Special {
                kind, input_kind, ..
            } => match kind {
                SpecialPortKind::CommandRecv => true, // decided per command
                SpecialPortKind::ProductRecv => matches!(input_kind, Some(SpecialInputKind::Async)),
                _ => false,
            },
        }
    }

    /// The array size (1 for special ports).
    pub fn size(&self) -> u64 {
        match self {
            PortInstance::General { size, .. } => *size,
            PortInstance::Special { .. } => 1,
        }
    }
}

/// A command.
#[derive(Debug, Clone)]
pub struct CommandDef {
    pub name: String,
    pub kind: CommandKind,
    pub params: Vec<ParamDef>,
    pub opcode: u64,
    pub priority: Option<i128>,
    pub queue_full: QueueFull,
    pub docs: Vec<String>,
    pub loc: Loc,
    /// For the implicit parameter commands: the parameter's index and
    /// whether this is the set (`true`) or save (`false`) command.
    pub param_cmd: Option<(usize, bool)>,
}

/// An event.
#[derive(Debug, Clone)]
pub struct EventDef {
    pub name: String,
    pub params: Vec<ParamDef>,
    pub severity: Severity,
    pub id: u64,
    pub format: Format,
    /// Throttle count, if throttled.
    pub throttle: Option<u64>,
    /// Throttle reset interval `every { seconds, useconds }`, if any.
    pub every: Option<(u64, u32)>,
    pub docs: Vec<String>,
    pub loc: Loc,
}

/// A telemetry channel.
#[derive(Debug, Clone)]
pub struct TlmChannelDef {
    pub name: String,
    pub ty: Type,
    pub id: u64,
    pub update: TlmUpdate,
    pub format: Option<Format>,
    pub low: Vec<(LimitKind, Value)>,
    pub high: Vec<(LimitKind, Value)>,
    pub docs: Vec<String>,
    pub loc: Loc,
}

/// A parameter.
#[derive(Debug, Clone)]
pub struct ParamSpecDef {
    pub name: String,
    pub ty: Type,
    pub default: Option<Value>,
    pub id: u64,
    pub set_opcode: u64,
    pub save_opcode: u64,
    pub is_external: bool,
    pub docs: Vec<String>,
    pub loc: Loc,
}

/// A data product container.
#[derive(Debug, Clone)]
pub struct ContainerDef {
    pub name: String,
    pub id: u64,
    pub default_priority: Option<u64>,
    pub docs: Vec<String>,
    pub loc: Loc,
}

/// A data product record.
#[derive(Debug, Clone)]
pub struct RecordDef {
    pub name: String,
    pub ty: Type,
    pub is_array: bool,
    pub id: u64,
    pub docs: Vec<String>,
    pub loc: Loc,
}

/// An internal port.
#[derive(Debug, Clone)]
pub struct InternalPortDef {
    pub name: String,
    pub params: Vec<ParamDef>,
    pub priority: Option<i128>,
    pub queue_full: QueueFull,
    pub docs: Vec<String>,
    pub loc: Loc,
}

/// A resolved component.
#[derive(Debug, Clone)]
pub struct ComponentModel {
    /// The component symbol.
    pub sym: SymId,
    /// Kind.
    pub kind: ComponentKind,
    /// All port instances, in declaration order (interface imports
    /// expanded in place).
    pub ports: Vec<PortInstance>,
    /// Commands in declaration order (parameter set/save commands appear
    /// where their parameter is declared).
    pub commands: Vec<CommandDef>,
    /// Events.
    pub events: Vec<EventDef>,
    /// Telemetry channels.
    pub tlm_channels: Vec<TlmChannelDef>,
    /// Parameters.
    pub params: Vec<ParamSpecDef>,
    /// Data product containers.
    pub containers: Vec<ContainerDef>,
    /// Data product records.
    pub records: Vec<RecordDef>,
    /// Internal ports.
    pub internal_ports: Vec<InternalPortDef>,
    /// `match p1 with p2` specifiers.
    pub port_matchings: Vec<(String, String)>,
    /// State machine instances (name, state machine symbol).
    pub state_machine_instances: Vec<(String, SymId)>,
    /// Annotation lines.
    pub docs: Vec<String>,
}

impl ComponentModel {
    /// A port instance by name.
    pub fn port(&self, name: &str) -> Option<&PortInstance> {
        self.ports.iter().find(|p| p.name() == name)
    }

    /// The special port of a kind, if present.
    pub fn special_port(&self, kind: SpecialPortKind) -> Option<&PortInstance> {
        self.ports
            .iter()
            .find(|p| matches!(p, PortInstance::Special { kind: k, .. } if *k == kind))
    }

    /// Whether the component has any async input (general or special).
    pub fn has_async_input(&self) -> bool {
        self.ports.iter().any(|p| match p {
            PortInstance::General { kind, .. } => *kind == GeneralPortKind::AsyncInput,
            PortInstance::Special {
                kind, input_kind, ..
            } => {
                *kind == SpecialPortKind::ProductRecv
                    && *input_kind == Some(SpecialInputKind::Async)
            }
        }) || self
            .commands
            .iter()
            .any(|c| c.kind == CommandKind::Async && c.param_cmd.is_none())
            || !self.internal_ports.is_empty()
            || !self.state_machine_instances.is_empty()
    }

    /// Whether the component has a message queue (active or queued).
    pub fn has_queue(&self) -> bool {
        self.kind != ComponentKind::Passive
    }
}

/// A resolved component instance.
#[derive(Debug, Clone)]
pub struct InstanceModel {
    pub sym: SymId,
    pub component: SymId,
    pub base_id: u64,
    pub queue_size: Option<u64>,
    pub stack_size: Option<u64>,
    pub priority: Option<u64>,
    /// May be negative (the C++ convention for "no affinity").
    pub cpu: Option<i128>,
    /// The `type "..."` clause: the implementation type.
    pub impl_type: Option<String>,
    pub docs: Vec<String>,
}

impl<'a> Analysis<'a> {
    /// Resolve an interface's port instances on demand (imports expanded).
    pub fn ensure_interface(&mut self, sym: SymId, use_loc: &Loc) -> Result<()> {
        if self.interfaces.contains_key(&sym) {
            return Ok(());
        }
        let Def::Interface(node) = self.symbols.sym(sym).def else {
            return Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "{} is not an interface",
                    self.symbols.sym(sym).qualified_name()
                ),
            ));
        };
        if !self.in_progress.insert(sym) {
            return Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "cyclic interface import of {}",
                    self.symbols.sym(sym).qualified_name()
                ),
            ));
        }
        let stack = self.symbols.sym(sym).def_stack.clone();
        let mut ports = Vec::new();
        for m in &node.data.members {
            let docs: Vec<String> = m.doc_lines().cloned().collect();
            match &m.node {
                InterfaceMemberNode::SpecPortInstance(pi) => {
                    ports.push(self.resolve_port_instance(&stack, pi, docs)?);
                }
                InterfaceMemberNode::SpecImportInterface(imp) => {
                    let isym = self.resolve_use(&stack, NameGroup::PortInterface, &imp.data.sym)?;
                    self.ensure_interface(isym, &imp.loc)?;
                    ports.extend(self.interfaces[&isym].iter().cloned());
                }
            }
        }
        self.in_progress.remove(&sym);
        self.interfaces.insert(sym, ports);
        Ok(())
    }

    fn resolve_port_instance(
        &mut self,
        stack: &[ScopeId],
        pi: &Node<SpecPortInstance>,
        docs: Vec<String>,
    ) -> Result<PortInstance> {
        Ok(match &pi.data {
            SpecPortInstance::General {
                kind,
                name,
                size,
                port,
                priority,
                queue_full,
            } => {
                let size = match size {
                    Some(e) => {
                        let n = self.eval_nonneg_int(stack, e, "port array size")?;
                        if n == 0 {
                            return Err(Diagnostic::semantic(
                                e.loc.clone(),
                                "port array size must be positive",
                            ));
                        }
                        n
                    }
                    None => 1,
                };
                let port = match port {
                    Some(qi) => {
                        let psym = self.resolve_use(stack, NameGroup::Port, qi)?;
                        self.ensure_port(psym, &qi.loc)?;
                        PortType::Typed(psym)
                    }
                    None => PortType::Serial,
                };
                if *kind != GeneralPortKind::AsyncInput
                    && (priority.is_some() || queue_full.is_some())
                {
                    return Err(Diagnostic::semantic(
                        pi.loc.clone(),
                        format!(
                            "{kind} port {name} may not specify priority or queue full behavior"
                        ),
                    ));
                }
                let priority = self.opt_int(stack, priority, "priority")?;
                PortInstance::General {
                    name: name.clone(),
                    kind: *kind,
                    size,
                    port,
                    priority,
                    queue_full: queue_full
                        .as_ref()
                        .map(|q| q.data)
                        .unwrap_or(QueueFull::Assert),
                    docs,
                    loc: pi.loc.clone(),
                }
            }
            SpecPortInstance::Special {
                input_kind,
                kind,
                name,
                priority,
                queue_full,
            } => {
                if input_kind.is_some() && *kind != SpecialPortKind::ProductRecv {
                    return Err(Diagnostic::semantic(
                        pi.loc.clone(),
                        format!("{kind} port may not specify input kind"),
                    ));
                }
                let is_async = *kind == SpecialPortKind::ProductRecv
                    && matches!(input_kind, None | Some(SpecialInputKind::Async));
                if !is_async && (priority.is_some() || queue_full.is_some()) {
                    return Err(Diagnostic::semantic(
                        pi.loc.clone(),
                        format!(
                            "{kind} port {name} may not specify priority or queue full behavior"
                        ),
                    ));
                }
                let input_kind = if *kind == SpecialPortKind::ProductRecv {
                    Some(input_kind.unwrap_or(SpecialInputKind::Async))
                } else {
                    None
                };
                let priority = self.opt_int(stack, priority, "priority")?;
                PortInstance::Special {
                    name: name.clone(),
                    kind: *kind,
                    input_kind,
                    priority,
                    queue_full: queue_full
                        .as_ref()
                        .map(|q| q.data)
                        .unwrap_or(QueueFull::Assert),
                    docs,
                    loc: pi.loc.clone(),
                }
            }
        })
    }

    /// Build the model of a component.
    pub fn resolve_component(&mut self, sym: SymId) -> Result<()> {
        let Def::Component(node) = self.symbols.sym(sym).def else {
            unreachable!()
        };
        let docs = self.symbols.sym(sym).docs.clone();
        let mut stack = self.symbols.sym(sym).def_stack.clone();
        stack.push(self.symbols.sym(sym).scope.expect("component scope"));
        let kind = node.data.kind;

        let mut ports: Vec<PortInstance> = Vec::new();
        let mut commands: Vec<CommandDef> = Vec::new();
        let mut events: Vec<EventDef> = Vec::new();
        let mut tlm_channels: Vec<TlmChannelDef> = Vec::new();
        let mut params: Vec<ParamSpecDef> = Vec::new();
        let mut containers: Vec<ContainerDef> = Vec::new();
        let mut records: Vec<RecordDef> = Vec::new();
        let mut internal_ports: Vec<InternalPortDef> = Vec::new();
        let mut port_matchings = Vec::new();
        let mut state_machine_instances = Vec::new();

        let mut next_opcode: u64 = 0;
        let mut next_event: u64 = 0;
        let mut next_chan: u64 = 0;
        let mut next_param: u64 = 0;
        let mut next_container: u64 = 0;
        let mut next_record: u64 = 0;

        fn add_port(ports: &mut Vec<PortInstance>, p: PortInstance) -> Result<()> {
            if let Some(prev) = ports.iter().find(|x| x.name() == p.name()) {
                return Err(Diagnostic::semantic(
                    p.loc().clone(),
                    format!("duplicate port instance {}", p.name()),
                )
                .with_note(prev.loc().clone(), "previous definition is here"));
            }
            if let PortInstance::Special { kind, .. } = &p {
                if let Some(prev) = ports
                    .iter()
                    .find(|x| matches!(x, PortInstance::Special { kind: k, .. } if k == kind))
                {
                    return Err(Diagnostic::semantic(
                        p.loc().clone(),
                        format!("duplicate {kind} port"),
                    )
                    .with_note(prev.loc().clone(), "previous definition is here"));
                }
            }
            ports.push(p);
            Ok(())
        }

        fn add_command(commands: &mut Vec<CommandDef>, c: CommandDef) -> Result<()> {
            if let Some(prev) = commands.iter().find(|x| x.opcode == c.opcode) {
                return Err(Diagnostic::semantic(
                    c.loc.clone(),
                    format!("duplicate opcode value {} ({:#x})", c.opcode, c.opcode),
                )
                .with_note(prev.loc.clone(), "previous occurrence is here"));
            }
            if let Some(prev) = commands.iter().find(|x| x.name == c.name) {
                return Err(Diagnostic::semantic(
                    c.loc.clone(),
                    format!("duplicate command {}", c.name),
                )
                .with_note(prev.loc.clone(), "previous definition is here"));
            }
            commands.push(c);
            Ok(())
        }

        for m in &node.data.members {
            let docs: Vec<String> = m.doc_lines().cloned().collect();
            match &m.node {
                ComponentMemberNode::SpecPortInstance(pi) => {
                    let p = self.resolve_port_instance(&stack, pi, docs)?;
                    add_port(&mut ports, p)?;
                }
                ComponentMemberNode::SpecImportInterface(imp) => {
                    let isym = self.resolve_use(&stack, NameGroup::PortInterface, &imp.data.sym)?;
                    self.ensure_interface(isym, &imp.loc)?;
                    for p in self.interfaces[&isym].clone() {
                        add_port(&mut ports, p)?;
                    }
                }
                ComponentMemberNode::SpecCommand(c) => {
                    let cd = &c.data;
                    if cd.kind != CommandKind::Async
                        && (cd.priority.is_some() || cd.queue_full.is_some())
                    {
                        return Err(Diagnostic::semantic(
                            c.loc.clone(),
                            "only async commands may specify priority or queue full behavior",
                        ));
                    }
                    let params = self.resolve_params(&stack, &cd.params)?;
                    let opcode = match self.opt_nonneg(&stack, &cd.opcode, "opcode")? {
                        Some(o) => o,
                        None => next_opcode,
                    };
                    next_opcode = opcode + 1;
                    let priority = self.opt_int(&stack, &cd.priority, "priority")?;
                    add_command(
                        &mut commands,
                        CommandDef {
                            name: cd.name.clone(),
                            kind: cd.kind,
                            params,
                            opcode,
                            priority,
                            queue_full: cd
                                .queue_full
                                .as_ref()
                                .map(|q| q.data)
                                .unwrap_or(QueueFull::Assert),
                            docs,
                            loc: c.loc.clone(),
                            param_cmd: None,
                        },
                    )?;
                }
                ComponentMemberNode::SpecEvent(e) => {
                    let ed = &e.data;
                    let params = self.resolve_params(&stack, &ed.params)?;
                    let id = match self.opt_nonneg(&stack, &ed.id, "event id")? {
                        Some(i) => i,
                        None => next_event,
                    };
                    next_event = id + 1;
                    let format = Format::parse(&ed.format.data, &ed.format.loc)?;
                    if format.num_fields() != params.len() {
                        return Err(Diagnostic::semantic(
                            ed.format.loc.clone(),
                            format!(
                                "format string has {} replacement fields but the event has {} parameters",
                                format.num_fields(),
                                params.len()
                            ),
                        ));
                    }
                    for ((field, _), p) in format.fields.iter().zip(params.iter()) {
                        if field.is_numeric() && !self.underlying(&p.ty).is_numeric() {
                            return Err(Diagnostic::semantic(
                                ed.format.loc.clone(),
                                format!(
                                    "numeric format field used for non-numeric parameter {}",
                                    p.name
                                ),
                            ));
                        }
                    }
                    let (throttle, every) = match &ed.throttle {
                        Some(t) => {
                            let count =
                                self.eval_nonneg_int(&stack, &t.data.count, "throttle count")?;
                            if count == 0 {
                                return Err(Diagnostic::semantic(
                                    t.data.count.loc.clone(),
                                    "event throttle count must be greater than zero",
                                ));
                            }
                            let every = match &t.data.every {
                                Some(e) => Some(self.eval_time_interval(&stack, e)?),
                                None => None,
                            };
                            (Some(count), every)
                        }
                        None => (None, None),
                    };
                    if let Some(prev) = events.iter().find(|x| x.id == id) {
                        return Err(Diagnostic::semantic(
                            e.loc.clone(),
                            format!("duplicate event id {id}"),
                        )
                        .with_note(prev.loc.clone(), "previous occurrence is here"));
                    }
                    if let Some(prev) = events.iter().find(|x| x.name == ed.name) {
                        return Err(Diagnostic::semantic(
                            e.loc.clone(),
                            format!("duplicate event {}", ed.name),
                        )
                        .with_note(prev.loc.clone(), "previous definition is here"));
                    }
                    events.push(EventDef {
                        name: ed.name.clone(),
                        params,
                        severity: ed.severity,
                        id,
                        format,
                        throttle,
                        every,
                        docs,
                        loc: e.loc.clone(),
                    });
                }
                ComponentMemberNode::SpecTlmChannel(t) => {
                    let td = &t.data;
                    let ty = self.resolve_type_name(&stack, &td.type_name)?;
                    let id = match self.opt_nonneg(&stack, &td.id, "telemetry channel id")? {
                        Some(i) => i,
                        None => next_chan,
                    };
                    next_chan = id + 1;
                    let format = match &td.format {
                        Some(f) => {
                            let fmt = Format::parse(&f.data, &f.loc)?;
                            if fmt.num_fields() != 1 {
                                return Err(Diagnostic::semantic(
                                    f.loc.clone(),
                                    "telemetry format string must have exactly one replacement field",
                                ));
                            }
                            Some(fmt)
                        }
                        None => None,
                    };
                    let mut low = Vec::new();
                    for (k, e) in &td.low {
                        let v = self.eval(&stack, e)?;
                        let v = self.convert(v, &ty, &e.loc)?;
                        low.push((k.data, v));
                    }
                    let mut high = Vec::new();
                    for (k, e) in &td.high {
                        let v = self.eval(&stack, e)?;
                        let v = self.convert(v, &ty, &e.loc)?;
                        high.push((k.data, v));
                    }
                    if let Some(prev) = tlm_channels.iter().find(|x| x.id == id) {
                        return Err(Diagnostic::semantic(
                            t.loc.clone(),
                            format!("duplicate telemetry channel id {id}"),
                        )
                        .with_note(prev.loc.clone(), "previous occurrence is here"));
                    }
                    if let Some(prev) = tlm_channels.iter().find(|x| x.name == td.name) {
                        return Err(Diagnostic::semantic(
                            t.loc.clone(),
                            format!("duplicate telemetry channel {}", td.name),
                        )
                        .with_note(prev.loc.clone(), "previous definition is here"));
                    }
                    tlm_channels.push(TlmChannelDef {
                        name: td.name.clone(),
                        ty,
                        id,
                        update: td.update.unwrap_or(TlmUpdate::Always),
                        format,
                        low,
                        high,
                        docs,
                        loc: t.loc.clone(),
                    });
                }
                ComponentMemberNode::SpecParam(p) => {
                    let pd = &p.data;
                    let ty = self.resolve_type_name(&stack, &pd.type_name)?;
                    let default = match &pd.default {
                        Some(e) => {
                            let v = self.eval(&stack, e)?;
                            Some(self.convert(v, &ty, &e.loc)?)
                        }
                        None => None,
                    };
                    let id = match self.opt_nonneg(&stack, &pd.id, "parameter id")? {
                        Some(i) => i,
                        None => next_param,
                    };
                    next_param = id + 1;
                    // Reference rule (Component.addCommand): every command
                    // added, explicit or implicit, moves the default opcode
                    // to one past it; set is added before save.
                    let set_opcode = match self.opt_nonneg(&stack, &pd.set_opcode, "set opcode")? {
                        Some(o) => o,
                        None => next_opcode,
                    };
                    next_opcode = set_opcode + 1;
                    let save_opcode =
                        match self.opt_nonneg(&stack, &pd.save_opcode, "save opcode")? {
                            Some(o) => o,
                            None => next_opcode,
                        };
                    next_opcode = save_opcode + 1;
                    if let Some(prev) = params.iter().find(|x| x.id == id) {
                        return Err(Diagnostic::semantic(
                            p.loc.clone(),
                            format!("duplicate parameter id {id}"),
                        )
                        .with_note(prev.loc.clone(), "previous occurrence is here"));
                    }
                    if let Some(prev) = params.iter().find(|x| x.name == pd.name) {
                        return Err(Diagnostic::semantic(
                            p.loc.clone(),
                            format!("duplicate parameter {}", pd.name),
                        )
                        .with_note(prev.loc.clone(), "previous definition is here"));
                    }
                    let index = params.len();
                    params.push(ParamSpecDef {
                        name: pd.name.clone(),
                        ty: ty.clone(),
                        default,
                        id,
                        set_opcode,
                        save_opcode,
                        is_external: pd.is_external,
                        docs: docs.clone(),
                        loc: p.loc.clone(),
                    });
                    // The implicit set and save commands. They have no
                    // user-declared kind: they are queued like async
                    // commands on components with a queue and run on the
                    // caller's thread on passive ones.
                    let param_cmd_kind = if kind == ComponentKind::Passive {
                        CommandKind::Sync
                    } else {
                        CommandKind::Async
                    };
                    let set_params = vec![ParamDef {
                        name: "val".into(),
                        kind: FormalParamKind::Value,
                        ty,
                        docs: vec!["The parameter value".into()],
                    }];
                    add_command(
                        &mut commands,
                        CommandDef {
                            name: format!("{}_PARAM_SET", pd.name),
                            kind: param_cmd_kind,
                            params: set_params,
                            opcode: set_opcode,
                            priority: None,
                            queue_full: QueueFull::Assert,
                            docs: vec![format!("Set parameter {}", pd.name)],
                            loc: p.loc.clone(),
                            param_cmd: Some((index, true)),
                        },
                    )?;
                    add_command(
                        &mut commands,
                        CommandDef {
                            name: format!("{}_PARAM_SAVE", pd.name),
                            kind: param_cmd_kind,
                            params: Vec::new(),
                            opcode: save_opcode,
                            priority: None,
                            queue_full: QueueFull::Assert,
                            docs: vec![format!("Save parameter {}", pd.name)],
                            loc: p.loc.clone(),
                            param_cmd: Some((index, false)),
                        },
                    )?;
                }
                ComponentMemberNode::SpecContainer(c) => {
                    let cd = &c.data;
                    let id = match self.opt_nonneg(&stack, &cd.id, "container id")? {
                        Some(i) => i,
                        None => next_container,
                    };
                    next_container = id + 1;
                    let default_priority =
                        self.opt_nonneg(&stack, &cd.default_priority, "default priority")?;
                    if let Some(prev) = containers.iter().find(|x| x.id == id) {
                        return Err(Diagnostic::semantic(
                            c.loc.clone(),
                            format!("duplicate container id {id}"),
                        )
                        .with_note(prev.loc.clone(), "previous occurrence is here"));
                    }
                    containers.push(ContainerDef {
                        name: cd.name.clone(),
                        id,
                        default_priority,
                        docs,
                        loc: c.loc.clone(),
                    });
                }
                ComponentMemberNode::SpecRecord(r) => {
                    let rd = &r.data;
                    let ty = self.resolve_type_name(&stack, &rd.record_type)?;
                    let id = match self.opt_nonneg(&stack, &rd.id, "record id")? {
                        Some(i) => i,
                        None => next_record,
                    };
                    next_record = id + 1;
                    if let Some(prev) = records.iter().find(|x| x.id == id) {
                        return Err(Diagnostic::semantic(
                            r.loc.clone(),
                            format!("duplicate record id {id}"),
                        )
                        .with_note(prev.loc.clone(), "previous occurrence is here"));
                    }
                    records.push(RecordDef {
                        name: rd.name.clone(),
                        ty,
                        is_array: rd.is_array,
                        id,
                        docs,
                        loc: r.loc.clone(),
                    });
                }
                ComponentMemberNode::SpecInternalPort(ip) => {
                    let ipd = &ip.data;
                    let params = self.resolve_params(&stack, &ipd.params)?;
                    let priority = self.opt_int(&stack, &ipd.priority, "priority")?;
                    internal_ports.push(InternalPortDef {
                        name: ipd.name.clone(),
                        params,
                        priority,
                        queue_full: ipd.queue_full.unwrap_or(QueueFull::Assert),
                        docs,
                        loc: ip.loc.clone(),
                    });
                }
                ComponentMemberNode::SpecPortMatching(pm) => {
                    port_matchings.push((pm.data.port1.data.clone(), pm.data.port2.data.clone()));
                }
                ComponentMemberNode::SpecStateMachineInstance(smi) => {
                    let ssym =
                        self.resolve_use(&stack, NameGroup::StateMachine, &smi.data.state_machine)?;
                    state_machine_instances.push((smi.data.name.clone(), ssym));
                }
                // Nested definitions were entered in pass 1 and resolve on demand.
                ComponentMemberNode::DefAbsType(_)
                | ComponentMemberNode::DefAliasType(_)
                | ComponentMemberNode::DefArray(_)
                | ComponentMemberNode::DefConstant(_)
                | ComponentMemberNode::DefEnum(_)
                | ComponentMemberNode::DefStateMachine(_)
                | ComponentMemberNode::DefStruct(_) => {}
                ComponentMemberNode::SpecInclude(_) => unreachable!("includes are spliced"),
            }
        }

        // Kind constraints (CheckComponentDefs).
        let model = ComponentModel {
            sym,
            kind,
            ports,
            commands,
            events,
            tlm_channels,
            params,
            containers,
            records,
            internal_ports,
            port_matchings,
            state_machine_instances,
            docs,
        };
        let has_async = model.has_async_input();
        match kind {
            ComponentKind::Passive if has_async => {
                return Err(Diagnostic::semantic(
                    node.loc.clone(),
                    "passive component may not have async input ports, async commands, internal ports or state machine instances",
                ));
            }
            ComponentKind::Active | ComponentKind::Queued if !has_async => {
                return Err(Diagnostic::semantic(
                    node.loc.clone(),
                    format!(
                        "{kind} component must have at least one async input port, async command, internal port or state machine instance"
                    ),
                ));
            }
            _ => {}
        }
        self.check_special_port_dependencies(&model, &node.loc)?;
        // Port matchings must name existing port arrays of equal size.
        for (p1, p2) in &model.port_matchings {
            let (a, b) = (model.port(p1), model.port(p2));
            let (Some(a), Some(b)) = (a, b) else {
                return Err(Diagnostic::semantic(
                    node.loc.clone(),
                    format!("port matching refers to unknown port ({p1} with {p2})"),
                ));
            };
            if a.size() != b.size() {
                return Err(Diagnostic::semantic(
                    a.loc().clone(),
                    format!("matched ports {p1} and {p2} must have the same array size"),
                ));
            }
        }
        self.components.insert(sym, model);
        Ok(())
    }

    /// The reference compiler's rules about which special ports must
    /// accompany commands, events, telemetry, parameters and data products.
    fn check_special_port_dependencies(&self, m: &ComponentModel, loc: &Loc) -> Result<()> {
        let has = |k: SpecialPortKind| m.special_port(k).is_some();
        let need = |cond: bool, what: &str, ports: &[SpecialPortKind]| -> Result<()> {
            if cond {
                for p in ports {
                    if !has(*p) {
                        return Err(Diagnostic::semantic(
                            loc.clone(),
                            format!("component has {what} but no {p} port"),
                        ));
                    }
                }
            }
            Ok(())
        };
        need(
            !m.commands.is_empty(),
            "commands",
            &[
                SpecialPortKind::CommandRecv,
                SpecialPortKind::CommandReg,
                SpecialPortKind::CommandResp,
            ],
        )?;
        need(
            !m.events.is_empty(),
            "events",
            &[SpecialPortKind::Event, SpecialPortKind::TimeGet],
        )?;
        need(
            !m.tlm_channels.is_empty(),
            "telemetry",
            &[SpecialPortKind::Telemetry, SpecialPortKind::TimeGet],
        )?;
        need(
            !m.params.is_empty(),
            "parameters",
            &[SpecialPortKind::ParamGet, SpecialPortKind::ParamSet],
        )?;
        need(
            !m.containers.is_empty() || !m.records.is_empty(),
            "data products",
            &[SpecialPortKind::ProductSend, SpecialPortKind::TimeGet],
        )?;
        let has_dp = !m.containers.is_empty() || !m.records.is_empty();
        let has_get = has(SpecialPortKind::ProductGet);
        let has_request_pair =
            has(SpecialPortKind::ProductRequest) && has(SpecialPortKind::ProductRecv);
        if has_dp && !has_get && !has_request_pair {
            return Err(Diagnostic::semantic(
                loc.clone(),
                "component has data products but neither a product get port nor a product request/recv port pair",
            ));
        }
        Ok(())
    }

    /// Build the model of a component instance.
    /// Evaluate a throttle interval `{ seconds = s, useconds = u }`
    /// (`Event.scala`: converted to `{ seconds: U32, useconds: U32 }`,
    /// useconds at most 999999).
    fn eval_time_interval(&mut self, stack: &[ScopeId], e: &Node<Expr>) -> Result<(u64, u32)> {
        let v = self.eval(stack, e)?;
        let target = Type::AnonStruct(vec![
            ("seconds".to_string(), Type::Int(IntKind::U32)),
            ("useconds".to_string(), Type::Int(IntKind::U32)),
        ]);
        let v = self.convert(v, &target, &e.loc)?;
        let Value::Struct(_, members) = v else {
            unreachable!("converted to a struct")
        };
        let get = |name: &str| -> i128 {
            members
                .iter()
                .find(|(n, _)| n == name)
                .and_then(|(_, v)| v.as_int())
                .unwrap_or(0)
        };
        let seconds = get("seconds");
        let useconds = get("useconds");
        if useconds > 999_999 {
            return Err(Diagnostic::semantic(
                e.loc.clone(),
                format!("useconds must be in the range [0, 999999], got {useconds}"),
            ));
        }
        Ok((seconds as u64, useconds as u32))
    }

    pub fn resolve_instance(&mut self, sym: SymId) -> Result<()> {
        let Def::ComponentInstance(node) = self.symbols.sym(sym).def else {
            unreachable!()
        };
        let stack = self.symbols.sym(sym).def_stack.clone();
        let d = &node.data;
        let component = self.resolve_use(&stack, NameGroup::Component, &d.component)?;
        if !matches!(self.symbols.sym(component).def, Def::Component(_)) {
            return Err(Diagnostic::semantic(
                d.component.loc.clone(),
                format!(
                    "{} is not a component",
                    self.symbols.sym(component).qualified_name()
                ),
            ));
        }
        let base_id = self.eval_nonneg_int(&stack, &d.base_id, "base id")?;
        let queue_size = self.opt_nonneg(&stack, &d.queue_size, "queue size")?;
        let stack_size = self.opt_nonneg(&stack, &d.stack_size, "stack size")?;
        let priority = self.opt_nonneg(&stack, &d.priority, "priority")?;
        let cpu = self.opt_int(&stack, &d.cpu, "cpu")?;
        let kind = self.components[&component].kind;
        if kind == ComponentKind::Passive && queue_size.is_some() {
            return Err(Diagnostic::semantic(
                node.loc.clone(),
                "passive component instance may not specify queue size",
            ));
        }
        if kind != ComponentKind::Active
            && (stack_size.is_some() || priority.is_some() || cpu.is_some())
        {
            return Err(Diagnostic::semantic(
                node.loc.clone(),
                format!("{kind} component instance may not specify stack size, priority or cpu"),
            ));
        }
        let docs = self.symbols.sym(sym).docs.clone();
        self.instances.insert(
            sym,
            InstanceModel {
                sym,
                component,
                base_id,
                queue_size,
                stack_size,
                priority,
                cpu,
                impl_type: d.impl_type.as_ref().map(|t| t.data.clone()),
                docs,
            },
        );
        Ok(())
    }

    /// Map from instance symbol to model, for callers iterating topologies.
    pub fn instance(&self, sym: SymId) -> &InstanceModel {
        &self.instances[&sym]
    }

    /// The component model of an instance.
    pub fn component_of_instance(&self, inst: SymId) -> &ComponentModel {
        &self.components[&self.instances[&inst].component]
    }

    /// All components, keyed by symbol (convenience for the back end).
    pub fn components_in_order(&self) -> Vec<&ComponentModel> {
        let mut out: Vec<&ComponentModel> = Vec::new();
        for s in &self.order {
            if let Some(c) = self.components.get(s) {
                out.push(c);
            }
        }
        out
    }
}
