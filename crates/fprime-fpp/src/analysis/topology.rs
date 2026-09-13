//! Topology resolution (`Topology.scala`, `ResolvePartiallyNumbered`,
//! `PatternResolver`, `GeneralPortNumbering`, `MatchedPortNumbering` in the
//! reference compiler).
//!
//! A topology's instances are its own plus those of the topologies it
//! imports, transitively; its connections are its direct graphs, the
//! imported topologies' connections whose endpoints are present, and the
//! expansion of its pattern graphs. Then port numbers are assigned: every
//! matched pair first, then unnumbered connections *into* an input port
//! array get index 0, and unnumbered connections *out of* an output port
//! array get the lowest free index in source order.

use super::Analysis;
use super::component::{PortInstance, PortType};
use super::symbols::{Def, NameGroup, SymId};
use crate::ast::*;
use crate::error::{Diagnostic, Loc, Result};
use std::collections::{BTreeMap, HashMap, HashSet};

/// One end of a connection.
#[derive(Debug, Clone)]
pub struct Endpoint {
    /// The component instance symbol.
    pub instance: SymId,
    /// The port instance name.
    pub port: String,
    /// The port number (explicit or assigned).
    pub num: Option<u64>,
    /// Where the endpoint was written.
    pub loc: Loc,
}

/// A resolved connection.
#[derive(Debug, Clone)]
pub struct ConnectionModel {
    /// The graph name (`connections X` or the pattern's name).
    pub graph: String,
    /// Source (output) end.
    pub from: Endpoint,
    /// Target (input) end.
    pub to: Endpoint,
    /// `unmatched` marker.
    pub is_unmatched: bool,
    /// Location of the connection.
    pub loc: Loc,
}

/// A reference to a telemetry channel of an instance (`inst.channel`).
#[derive(Debug, Clone)]
pub struct TlmChannelRef {
    /// The component instance.
    pub instance: SymId,
    /// The channel name in the instance's component.
    pub channel: String,
    /// Where it is written.
    pub loc: Loc,
}

/// A telemetry packet.
#[derive(Debug, Clone)]
pub struct TlmPacketModel {
    pub name: String,
    pub id: u64,
    pub group: u64,
    pub members: Vec<TlmChannelRef>,
    pub loc: Loc,
}

/// A telemetry packet set.
#[derive(Debug, Clone)]
pub struct TlmPacketSetModel {
    pub name: String,
    pub packets: Vec<TlmPacketModel>,
    pub omitted: Vec<TlmChannelRef>,
    pub loc: Loc,
}

/// A resolved topology.
#[derive(Debug, Clone)]
pub struct TopologyModel {
    /// The topology symbol.
    pub sym: SymId,
    /// Whether declared `deployment topology`.
    pub is_deployment: bool,
    /// Instances, in first-mention order (own then imported).
    pub instances: Vec<SymId>,
    /// Connections with port numbers assigned.
    pub connections: Vec<ConnectionModel>,
    /// The topology's own (non-imported) connections, before numbering; kept
    /// for imports by other topologies.
    pub local_connections: Vec<ConnectionModel>,
    /// Directly imported topologies.
    pub imports: Vec<SymId>,
    /// Topology ports (`port p = i.q`), resolved to the underlying
    /// component instance and port instance name.
    pub top_ports: Vec<(String, SymId, String)>,
    /// The topology's own telemetry packet sets (packet ids assigned).
    pub packet_sets: Vec<TlmPacketSetModel>,
    /// Annotation lines.
    pub docs: Vec<String>,
}

impl<'a> Analysis<'a> {
    /// Resolve a topology on demand.
    pub fn ensure_topology(&mut self, sym: SymId, use_loc: &Loc) -> Result<()> {
        if self.topologies.contains_key(&sym) {
            return Ok(());
        }
        let Def::Topology(node) = self.symbols.sym(sym).def else {
            return Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "{} is not a topology",
                    self.symbols.sym(sym).qualified_name()
                ),
            ));
        };
        if !self.in_progress.insert(sym) {
            return Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "cyclic topology import of {}",
                    self.symbols.sym(sym).qualified_name()
                ),
            ));
        }
        let stack = self.symbols.sym(sym).def_stack.clone();
        let docs = self.symbols.sym(sym).docs.clone();

        let mut instances: Vec<SymId> = Vec::new();
        let mut imports: Vec<SymId> = Vec::new();
        let mut local: Vec<ConnectionModel> = Vec::new();
        let mut patterns: Vec<(PatternKind, SymId, Vec<SymId>, Loc)> = Vec::new();

        // Instances and imports.
        for m in &node.data.members {
            if let TopologyMemberNode::SpecInstance(si) = &m.node {
                let s =
                    self.resolve_use(&stack, NameGroup::PortInterfaceInstance, &si.data.instance)?;
                match self.symbols.sym(s).def {
                    Def::ComponentInstance(_) => {
                        if !instances.contains(&s) {
                            instances.push(s);
                        }
                    }
                    Def::Topology(_) => {
                        self.ensure_topology(s, &si.loc)?;
                        if !imports.contains(&s) {
                            imports.push(s);
                        }
                    }
                    other => {
                        return Err(Diagnostic::semantic(
                            si.loc.clone(),
                            format!(
                                "{} {} is not a component instance or topology",
                                other.kind_name(),
                                self.symbols.sym(s).qualified_name()
                            ),
                        ));
                    }
                }
            }
        }
        // Transitively imported instances.
        let mut all_imports: Vec<SymId> = Vec::new();
        for imp in &imports {
            let t = &self.topologies[imp];
            for i in &t.instances {
                if !instances.contains(i) {
                    instances.push(*i);
                }
            }
            if !all_imports.contains(imp) {
                all_imports.push(*imp);
            }
        }

        // Topology ports: resolved to component-instance endpoints.
        let mut top_ports: Vec<(String, SymId, String)> = Vec::new();
        for m in &node.data.members {
            if let TopologyMemberNode::SpecTopPort(tp) = &m.node {
                let ep = self.resolve_endpoint_with(
                    &stack,
                    &instances,
                    &top_ports,
                    &tp.data.underlying_port,
                    &None,
                )?;
                if top_ports.iter().any(|(n, _, _)| *n == tp.data.name) {
                    return Err(Diagnostic::semantic(
                        tp.loc.clone(),
                        format!("duplicate topology port {}", tp.data.name),
                    ));
                }
                top_ports.push((tp.data.name.clone(), ep.instance, ep.port));
            }
        }

        // Direct connection graphs and patterns.
        for m in &node.data.members {
            match &m.node {
                TopologyMemberNode::SpecConnectionGraph(g) => match &g.data {
                    SpecConnectionGraph::Direct { name, connections } => {
                        for c in connections {
                            let from = self.resolve_endpoint_with(
                                &stack,
                                &instances,
                                &top_ports,
                                &c.from_port,
                                &c.from_index,
                            )?;
                            let to = self.resolve_endpoint_with(
                                &stack,
                                &instances,
                                &top_ports,
                                &c.to_port,
                                &c.to_index,
                            )?;
                            local.push(ConnectionModel {
                                graph: name.clone(),
                                from,
                                to,
                                is_unmatched: c.is_unmatched,
                                loc: c.from_port.loc.clone(),
                            });
                        }
                    }
                    SpecConnectionGraph::Pattern {
                        kind,
                        source,
                        targets,
                    } => {
                        let src =
                            self.resolve_use(&stack, NameGroup::PortInterfaceInstance, source)?;
                        let mut tgts = Vec::new();
                        for t in targets {
                            tgts.push(self.resolve_use(
                                &stack,
                                NameGroup::PortInterfaceInstance,
                                t,
                            )?);
                        }
                        patterns.push((*kind, src, tgts, g.loc.clone()));
                    }
                },
                TopologyMemberNode::SpecTopPort(_)
                | TopologyMemberNode::SpecTlmPacketSet(_)
                | TopologyMemberNode::SpecInstance(_) => {}
                TopologyMemberNode::SpecInclude(_) => unreachable!("includes are spliced"),
            }
        }

        // Imported connections whose endpoints are present.
        let mut connections: Vec<ConnectionModel> = local.clone();
        for imp in &imports {
            let t = self.topologies[imp].clone();
            for c in t.connections {
                if instances.contains(&c.from.instance) && instances.contains(&c.to.instance) {
                    // Imported connections keep their (already assigned) numbers.
                    connections.push(c);
                }
            }
        }

        // Patterns.
        for (kind, src, tgts, loc) in patterns {
            let expanded = self.expand_pattern(kind, src, &tgts, &instances, &loc)?;
            for c in expanded {
                let exists = connections.iter().any(|x| {
                    x.from.instance == c.from.instance
                        && x.from.port == c.from.port
                        && x.to.instance == c.to.instance
                        && x.to.port == c.to.port
                });
                if !exists {
                    connections.push(c);
                }
            }
        }

        self.check_connections(&connections)?;
        self.number_ports(&instances, &mut connections)?;

        // Telemetry packet sets.
        let mut packet_sets: Vec<TlmPacketSetModel> = Vec::new();
        for m in &node.data.members {
            let TopologyMemberNode::SpecTlmPacketSet(set) = &m.node else {
                continue;
            };
            if let Some(prev) = packet_sets.iter().find(|p| p.name == set.data.name) {
                return Err(Diagnostic::semantic(
                    set.loc.clone(),
                    format!("duplicate telemetry packet set {}", set.data.name),
                )
                .with_note(prev.loc.clone(), "previous occurrence is here"));
            }
            let ps = self.resolve_packet_set(&stack, set, &instances)?;
            packet_sets.push(ps);
        }

        self.in_progress.remove(&sym);
        self.topologies.insert(
            sym,
            TopologyModel {
                sym,
                is_deployment: node.data.is_deployment,
                instances,
                connections,
                local_connections: local,
                imports,
                top_ports,
                packet_sets,
                docs,
            },
        );
        Ok(())
    }

    /// Resolve a telemetry packet set (`TlmPacketSet.scala`): packet ids
    /// default to the previous id + 1 from 0; names and ids are unique;
    /// every channel reference names a channel of an instance of the
    /// topology. Whether every channel is used or omitted is checked
    /// when the dictionary is built.
    fn resolve_packet_set(
        &mut self,
        stack: &[super::ScopeId],
        set: &Node<SpecTlmPacketSet>,
        instances: &[SymId],
    ) -> Result<TlmPacketSetModel> {
        let mut packets: Vec<TlmPacketModel> = Vec::new();
        let mut next_id: u64 = 0;
        for pm in &set.data.members {
            let TlmPacketSetMemberNode::SpecTlmPacket(p) = &pm.node else {
                unreachable!("includes are spliced")
            };
            let pd = &p.data;
            let id = match self.opt_nonneg(stack, &pd.id, "packet id")? {
                Some(i) => i,
                None => next_id,
            };
            next_id = id + 1;
            let group = self.eval_nonneg_int(stack, &pd.group, "packet group")?;
            if let Some(prev) = packets.iter().find(|x| x.id == id) {
                return Err(Diagnostic::semantic(
                    p.loc.clone(),
                    format!("duplicate packet id {id}"),
                )
                .with_note(prev.loc.clone(), "previous occurrence is here"));
            }
            if let Some(prev) = packets.iter().find(|x| x.name == pd.name) {
                return Err(Diagnostic::semantic(
                    p.loc.clone(),
                    format!("duplicate packet {}", pd.name),
                )
                .with_note(prev.loc.clone(), "previous occurrence is here"));
            }
            let mut members = Vec::new();
            for cm in &pd.members {
                let TlmPacketMember::TlmChannelIdentifier(ci) = cm else {
                    unreachable!("includes are spliced")
                };
                members.push(self.resolve_channel_ref(stack, ci, instances)?);
            }
            packets.push(TlmPacketModel {
                name: pd.name.clone(),
                id,
                group,
                members,
                loc: p.loc.clone(),
            });
        }
        let mut omitted = Vec::new();
        for ci in &set.data.omitted {
            omitted.push(self.resolve_channel_ref(stack, ci, instances)?);
        }
        Ok(TlmPacketSetModel {
            name: set.data.name.clone(),
            packets,
            omitted,
            loc: set.loc.clone(),
        })
    }

    /// Resolve `inst.channel` against the instances of a topology.
    fn resolve_channel_ref(
        &mut self,
        stack: &[super::ScopeId],
        ci: &Node<TlmChannelIdentifier>,
        instances: &[SymId],
    ) -> Result<TlmChannelRef> {
        let inst = self.resolve_use(
            stack,
            NameGroup::PortInterfaceInstance,
            &ci.data.component_instance,
        )?;
        if !matches!(self.symbols.sym(inst).def, Def::ComponentInstance(_)) {
            return Err(Diagnostic::semantic(
                ci.data.component_instance.loc.clone(),
                format!(
                    "{} is not a component instance",
                    self.symbols.sym(inst).qualified_name()
                ),
            ));
        }
        if !instances.contains(&inst) {
            return Err(Diagnostic::semantic(
                ci.data.component_instance.loc.clone(),
                format!(
                    "component instance {} is not in this topology",
                    self.symbols.sym(inst).qualified_name()
                ),
            ));
        }
        let name = &ci.data.channel_name.data;
        let comp = self.component_of_instance(inst);
        if !comp.tlm_channels.iter().any(|c| c.name == *name) {
            return Err(Diagnostic::semantic(
                ci.data.channel_name.loc.clone(),
                format!(
                    "{} has no telemetry channel {name}",
                    self.symbols.sym(comp.sym).qualified_name()
                ),
            ));
        }
        Ok(TlmChannelRef {
            instance: inst,
            channel: name.clone(),
            loc: ci.loc.clone(),
        })
    }

    /// Resolve `i.p[n]` to a component-instance endpoint. `i` may be an
    /// imported topology, in which case `p` is one of its topology ports
    /// (or, for the topology being defined, one of `own_ports`).
    fn resolve_endpoint_with(
        &mut self,
        stack: &[super::ScopeId],
        instances: &[SymId],
        own_ports: &[(String, SymId, String)],
        pii: &Node<PortInstanceIdentifier>,
        index: &Option<Node<Expr>>,
    ) -> Result<Endpoint> {
        let inst = self.resolve_use(
            stack,
            NameGroup::PortInterfaceInstance,
            &pii.data.interface_instance,
        )?;
        if let Def::Topology(_) = self.symbols.sym(inst).def {
            let pname = &pii.data.port_name.data;
            let found = if self.topologies.contains_key(&inst) {
                self.topologies[&inst]
                    .top_ports
                    .iter()
                    .find(|(n, _, _)| n == pname)
                    .map(|(_, i, p)| (*i, p.clone()))
            } else {
                own_ports
                    .iter()
                    .find(|(n, _, _)| n == pname)
                    .map(|(_, i, p)| (*i, p.clone()))
            };
            let Some((under_inst, under_port)) = found else {
                return Err(Diagnostic::semantic(
                    pii.data.port_name.loc.clone(),
                    format!(
                        "topology {} has no port {}",
                        self.symbols.sym(inst).qualified_name(),
                        pname
                    ),
                ));
            };
            let num = match index {
                Some(e) => Some(self.eval_nonneg_int(stack, e, "port number")?),
                None => None,
            };
            return Ok(Endpoint {
                instance: under_inst,
                port: under_port,
                num,
                loc: pii.loc.clone(),
            });
        }
        if !matches!(self.symbols.sym(inst).def, Def::ComponentInstance(_)) {
            return Err(Diagnostic::semantic(
                pii.loc.clone(),
                format!(
                    "{} is not a component instance",
                    self.symbols.sym(inst).qualified_name()
                ),
            ));
        }
        if !instances.contains(&inst) {
            return Err(Diagnostic::semantic(
                pii.loc.clone(),
                format!(
                    "component instance {} is not a member of this topology",
                    self.symbols.sym(inst).qualified_name()
                ),
            ));
        }
        let comp = self.component_of_instance(inst);
        let port = &pii.data.port_name.data;
        if comp.port(port).is_none() {
            return Err(Diagnostic::semantic(
                pii.data.port_name.loc.clone(),
                format!(
                    "component {} has no port instance {}",
                    self.symbols.sym(comp.sym).qualified_name(),
                    port
                ),
            ));
        }
        let num = match index {
            Some(e) => Some(self.eval_nonneg_int(stack, e, "port number")?),
            None => None,
        };
        Ok(Endpoint {
            instance: inst,
            port: port.clone(),
            num,
            loc: pii.loc.clone(),
        })
    }

    /// The single general port of an instance's component with the given
    /// direction and typed port definition name (e.g. `Fw.Cmd`).
    fn general_port_named(
        &self,
        inst: SymId,
        input: bool,
        port_type: &str,
        what: &str,
        loc: &Loc,
    ) -> Result<String> {
        let comp = self.component_of_instance(inst);
        let matches: Vec<&PortInstance> = comp
            .ports
            .iter()
            .filter(|p| match p {
                PortInstance::General { kind, port, .. } => {
                    let dir_ok = if input {
                        *kind != GeneralPortKind::Output
                    } else {
                        *kind == GeneralPortKind::Output
                    };
                    dir_ok
                        && matches!(port, PortType::Typed(s) if self.symbols.sym(*s).qualified_name() == port_type)
                }
                PortInstance::Special { .. } => false,
            })
            .collect();
        match matches.len() {
            1 => Ok(matches[0].name().to_string()),
            0 => Err(Diagnostic::semantic(
                loc.clone(),
                format!(
                    "instance {} has no {what} port",
                    self.symbols.sym(inst).qualified_name()
                ),
            )),
            _ => Err(Diagnostic::semantic(
                loc.clone(),
                format!(
                    "ambiguous pattern: instance {} has {what} ports {}",
                    self.symbols.sym(inst).qualified_name(),
                    matches
                        .iter()
                        .map(|p| p.name())
                        .collect::<Vec<_>>()
                        .join(", ")
                ),
            )),
        }
    }

    fn special_port_of(&self, inst: SymId, kind: SpecialPortKind) -> Option<String> {
        self.component_of_instance(inst)
            .special_port(kind)
            .map(|p| p.name().to_string())
    }

    fn expand_pattern(
        &self,
        kind: PatternKind,
        source: SymId,
        targets: &[SymId],
        instances: &[SymId],
        loc: &Loc,
    ) -> Result<Vec<ConnectionModel>> {
        let explicit = !targets.is_empty();
        let candidates: Vec<SymId> = if explicit {
            targets.to_vec()
        } else {
            instances.to_vec()
        };
        for s in std::iter::once(&source).chain(targets.iter()) {
            if !instances.contains(s) {
                return Err(Diagnostic::semantic(
                    loc.clone(),
                    format!(
                        "component instance {} is not a member of this topology",
                        self.symbols.sym(*s).qualified_name()
                    ),
                ));
            }
        }
        let ep = |inst: SymId, port: String| Endpoint {
            instance: inst,
            port,
            num: None,
            loc: loc.clone(),
        };
        let conn = |graph: &str, from: Endpoint, to: Endpoint| ConnectionModel {
            graph: graph.to_string(),
            from,
            to,
            is_unmatched: false,
            loc: loc.clone(),
        };
        let mut out = Vec::new();
        match kind {
            PatternKind::Command => {
                let reg_in =
                    self.general_port_named(source, true, "Fw.CmdReg", "command reg", loc)?;
                let cmd_out =
                    self.general_port_named(source, false, "Fw.Cmd", "command send", loc)?;
                let resp_in =
                    self.general_port_named(source, true, "Fw.CmdResponse", "command resp", loc)?;
                for t in candidates {
                    let ports = (
                        self.special_port_of(t, SpecialPortKind::CommandReg),
                        self.special_port_of(t, SpecialPortKind::CommandRecv),
                        self.special_port_of(t, SpecialPortKind::CommandResp),
                    );
                    match ports {
                        (Some(reg), Some(recv), Some(resp)) => {
                            out.push(conn(
                                "CommandRegistration",
                                ep(t, reg),
                                ep(source, reg_in.clone()),
                            ));
                            out.push(conn("Command", ep(source, cmd_out.clone()), ep(t, recv)));
                            out.push(conn(
                                "CommandResponse",
                                ep(t, resp),
                                ep(source, resp_in.clone()),
                            ));
                        }
                        _ if explicit => {
                            return Err(Diagnostic::semantic(
                                loc.clone(),
                                format!(
                                    "instance {} has no command ports",
                                    self.symbols.sym(t).qualified_name()
                                ),
                            ));
                        }
                        _ => {}
                    }
                }
            }
            PatternKind::Event
            | PatternKind::Telemetry
            | PatternKind::TextEvent
            | PatternKind::Time => {
                let (special, port_type, graph, what) = match kind {
                    PatternKind::Event => (SpecialPortKind::Event, "Fw.Log", "Events", "event"),
                    PatternKind::Telemetry => (
                        SpecialPortKind::Telemetry,
                        "Fw.Tlm",
                        "Telemetry",
                        "telemetry",
                    ),
                    PatternKind::TextEvent => (
                        SpecialPortKind::TextEvent,
                        "Fw.LogText",
                        "TextEvents",
                        "text event",
                    ),
                    _ => (SpecialPortKind::TimeGet, "Fw.Time", "Time", "time get"),
                };
                let sink = self.general_port_named(source, true, port_type, what, loc)?;
                for t in candidates {
                    match self.special_port_of(t, special) {
                        Some(p) => out.push(conn(graph, ep(t, p), ep(source, sink.clone()))),
                        None if explicit => {
                            return Err(Diagnostic::semantic(
                                loc.clone(),
                                format!(
                                    "instance {} has no {what} port",
                                    self.symbols.sym(t).qualified_name()
                                ),
                            ));
                        }
                        None => {}
                    }
                }
            }
            PatternKind::Health => {
                let ping_in = self.general_port_named(source, true, "Svc.Ping", "ping", loc)?;
                let ping_out = self.general_port_named(source, false, "Svc.Ping", "ping", loc)?;
                for t in candidates {
                    if t == source {
                        continue;
                    }
                    let tin = self.general_port_named(t, true, "Svc.Ping", "ping", loc);
                    let tout = self.general_port_named(t, false, "Svc.Ping", "ping", loc);
                    match (tin, tout) {
                        (Ok(tin), Ok(tout)) => {
                            out.push(conn("Health", ep(source, ping_out.clone()), ep(t, tin)));
                            out.push(conn("Health", ep(t, tout), ep(source, ping_in.clone())));
                        }
                        (Err(e), _) | (_, Err(e)) if explicit => return Err(e),
                        _ => {}
                    }
                }
            }
            PatternKind::Param => {
                let get_in =
                    self.general_port_named(source, true, "Fw.PrmGet", "param get", loc)?;
                let set_in =
                    self.general_port_named(source, true, "Fw.PrmSet", "param set", loc)?;
                for t in candidates {
                    let ports = (
                        self.special_port_of(t, SpecialPortKind::ParamGet),
                        self.special_port_of(t, SpecialPortKind::ParamSet),
                    );
                    match ports {
                        (Some(g), Some(s)) => {
                            out.push(conn("Parameters", ep(t, g), ep(source, get_in.clone())));
                            out.push(conn("Parameters", ep(t, s), ep(source, set_in.clone())));
                        }
                        _ if explicit => {
                            return Err(Diagnostic::semantic(
                                loc.clone(),
                                format!(
                                    "instance {} has no param ports",
                                    self.symbols.sym(t).qualified_name()
                                ),
                            ));
                        }
                        _ => {}
                    }
                }
            }
        }
        Ok(out)
    }

    /// Direction and type compatibility of every connection.
    fn check_connections(&self, connections: &[ConnectionModel]) -> Result<()> {
        for c in connections {
            let from_comp = self.component_of_instance(c.from.instance);
            let to_comp = self.component_of_instance(c.to.instance);
            let fp = from_comp.port(&c.from.port).expect("resolved");
            let tp = to_comp.port(&c.to.port).expect("resolved");
            if fp.is_input() {
                return Err(Diagnostic::semantic(
                    c.from.loc.clone(),
                    format!("port {} is not an output port", c.from.port),
                ));
            }
            if !tp.is_input() {
                return Err(Diagnostic::semantic(
                    c.to.loc.clone(),
                    format!("port {} is not an input port", c.to.port),
                ));
            }
            // Typed ports must match unless one side is serial; special
            // ports are checked by role.
            if let (PortInstance::General { port: a, .. }, PortInstance::General { port: b, .. }) =
                (fp, tp)
            {
                if let (PortType::Typed(x), PortType::Typed(y)) = (a, b) {
                    if x != y {
                        return Err(Diagnostic::semantic(
                            c.loc.clone(),
                            format!(
                                "mismatched port types: {} is {} but {} is {}",
                                c.from.port,
                                self.symbols.sym(*x).qualified_name(),
                                c.to.port,
                                self.symbols.sym(*y).qualified_name()
                            ),
                        ));
                    }
                }
            }
            if let Some(n) = c.from.num {
                if n >= fp.size() {
                    return Err(Diagnostic::semantic(
                        c.from.loc.clone(),
                        format!(
                            "port number {n} is out of range for {} (size {})",
                            c.from.port,
                            fp.size()
                        ),
                    ));
                }
            }
            if let Some(n) = c.to.num {
                if n >= tp.size() {
                    return Err(Diagnostic::semantic(
                        c.to.loc.clone(),
                        format!(
                            "port number {n} is out of range for {} (size {})",
                            c.to.port,
                            tp.size()
                        ),
                    ));
                }
            }
        }
        Ok(())
    }

    /// Assign port numbers (matched numbering, then general numbering).
    /// Connections are numbered in source order of their `from` endpoint
    /// (the reference sorts by location), so the vector is sorted first.
    fn number_ports(&self, instances: &[SymId], connections: &mut [ConnectionModel]) -> Result<()> {
        connections.sort_by(|a, b| loc_key(&a.from.loc).cmp(&loc_key(&b.from.loc)));
        // Matched numbering: for each instance with `match p1 with p2`,
        // pair connections at p1 and p2 by their remote instance and give
        // both the same number.
        for inst in instances {
            let comp = self.component_of_instance(*inst);
            for (p1, p2) in &comp.port_matchings {
                let size = comp.port(p1).expect("checked").size();
                // Connections at p1 / p2 keyed by the other instance.
                let mut at1: BTreeMap<SymId, Vec<usize>> = BTreeMap::new();
                let mut at2: BTreeMap<SymId, Vec<usize>> = BTreeMap::new();
                for (i, c) in connections.iter().enumerate() {
                    if c.from.instance == *inst && c.from.port == *p1 {
                        at1.entry(c.to.instance).or_default().push(i);
                    } else if c.to.instance == *inst && c.to.port == *p1 {
                        at1.entry(c.from.instance).or_default().push(i);
                    }
                    if c.from.instance == *inst && c.from.port == *p2 {
                        at2.entry(c.to.instance).or_default().push(i);
                    } else if c.to.instance == *inst && c.to.port == *p2 {
                        at2.entry(c.from.instance).or_default().push(i);
                    }
                }
                let mut used: HashSet<u64> = HashSet::new();
                let end_num = |c: &ConnectionModel, inst: SymId, port: &str| -> Option<u64> {
                    if c.from.instance == inst && c.from.port == port {
                        c.from.num
                    } else {
                        c.to.num
                    }
                };
                for (_, idxs) in at1.iter().chain(at2.iter()) {
                    for i in idxs {
                        let c = &connections[*i];
                        let p = if at1.values().any(|v| v.contains(i)) {
                            p1
                        } else {
                            p2
                        };
                        if let Some(n) = end_num(c, *inst, p) {
                            used.insert(n);
                        }
                    }
                }
                let mut next = 0u64;
                for (remote, i1s) in &at1 {
                    let Some(i2s) = at2.get(remote) else {
                        return Err(Diagnostic::semantic(
                            connections[i1s[0]].loc.clone(),
                            format!(
                                "connection at matched port {p1} has no matching connection at {p2} for instance {}",
                                self.symbols.sym(*remote).qualified_name()
                            ),
                        ));
                    };
                    if i1s.len() != 1 || i2s.len() != 1 {
                        return Err(Diagnostic::semantic(
                            connections[i1s[0]].loc.clone(),
                            format!(
                                "matched ports {p1}/{p2} have more than one connection to the same instance"
                            ),
                        ));
                    }
                    let (i1, i2) = (i1s[0], i2s[0]);
                    let n1 = end_num(&connections[i1], *inst, p1);
                    let n2 = end_num(&connections[i2], *inst, p2);
                    let n = match (n1, n2) {
                        (Some(a), Some(b)) if a == b => a,
                        (Some(a), Some(b)) => {
                            return Err(Diagnostic::semantic(
                                connections[i1].loc.clone(),
                                format!(
                                    "mismatched port numbers {a} and {b} at matched ports {p1}/{p2}"
                                ),
                            ));
                        }
                        (Some(a), None) | (None, Some(a)) => a,
                        (None, None) => {
                            while used.contains(&next) {
                                next += 1;
                            }
                            let n = next;
                            used.insert(n);
                            n
                        }
                    };
                    if n >= size {
                        return Err(Diagnostic::semantic(
                            connections[i1].loc.clone(),
                            format!("no port number available for matched ports {p1}/{p2}"),
                        ));
                    }
                    for (i, p) in [(i1, p1), (i2, p2)] {
                        let c = &mut connections[i];
                        if c.from.instance == *inst && c.from.port == *p {
                            c.from.num = Some(n);
                        } else {
                            c.to.num = Some(n);
                        }
                    }
                }
            }
        }

        // General numbering. Inputs: unnumbered -> 0. Outputs: lowest
        // free number, in connection order.
        let mut used_out: HashMap<(SymId, String), HashSet<u64>> = HashMap::new();
        for c in connections.iter() {
            if let Some(n) = c.from.num {
                used_out
                    .entry((c.from.instance, c.from.port.clone()))
                    .or_default()
                    .insert(n);
            }
        }
        for c in connections.iter_mut() {
            if c.to.num.is_none() {
                c.to.num = Some(0);
            }
            if c.from.num.is_none() {
                let used = used_out
                    .entry((c.from.instance, c.from.port.clone()))
                    .or_default();
                let mut n = 0u64;
                while used.contains(&n) {
                    n += 1;
                }
                used.insert(n);
                c.from.num = Some(n);
            }
        }
        // Range check the assigned output numbers.
        for c in connections.iter() {
            let comp = self.component_of_instance(c.from.instance);
            let size = comp.port(&c.from.port).expect("resolved").size();
            let n = c.from.num.expect("assigned");
            if n >= size {
                return Err(Diagnostic::semantic(
                    c.from.loc.clone(),
                    format!(
                        "too many connections from output port {}.{} (array size {size})",
                        self.symbols.sym(c.from.instance).qualified_name(),
                        c.from.port
                    ),
                ));
            }
        }
        Ok(())
    }
}

/// A total order on locations: file, then line, then column.
fn loc_key(l: &Loc) -> (String, u32, u32) {
    (l.file.display().to_string(), l.line, l.col)
}
