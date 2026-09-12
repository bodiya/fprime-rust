//! Emission of topologies: the Rust equivalent of the C++ autocoder's
//! `<Top>TopologyAc`.
//!
//! For an FPP topology `T` the back end emits `TTopology`, a struct with
//! one `Arc<Impl>` field per component instance, where `Impl` is the
//! instance's implementation type: the `type "path"` clause of the
//! instance definition (interpreted as a Rust path), or
//! `<impl prefix><Module>::<Component>` by default. Its methods are the
//! C++ topology phases:
//!
//! - `set_id_bases()` — every instance's base id;
//! - `init()` — queue creation for queued and active instances;
//! - `connect()` — every connection of the resolved graph, port numbers
//!   included, in graph order;
//! - `reg_commands()` and `load_parameters()`;
//! - `start_tasks()` / `exit_tasks()` / `join_tasks()` for active
//!   instances.
//!
//! Every instance must be of a generated component (the port field and
//! factory names are the generated ones).

use super::Generator;
use super::names::{snake, snake_ident};
use crate::analysis::{ComponentModel, InstanceModel, PortInstance, SymId};
use crate::ast::{ComponentKind, GeneralPortKind, SpecialPortKind};
use crate::error::{Diagnostic, Result};

impl Generator<'_, '_> {
    /// The Rust path of an instance's implementation type.
    fn impl_path(&self, inst: &InstanceModel) -> String {
        if let Some(t) = &inst.impl_type {
            return t.clone();
        }
        let comp = self.a.symbols.sym(inst.component);
        let mut p = self.opts.impl_prefix.clone();
        for part in &comp.qualified {
            p.push_str(&super::names::ident(part));
            p.push_str("::");
        }
        p.truncate(p.len() - 2);
        p
    }

    /// Path to a generated component item (`Base`, `Handlers`, ...) from
    /// the current module.
    fn comp_item(&self, comp_sym: SymId, suffix: &str) -> String {
        let name = format!("{}{suffix}", self.a.symbols.sym(comp_sym).name);
        self.path_to(comp_sym, &name)
    }

    /// `<Impl as Handlers>::base(&*self.field)`.
    fn base_of(&self, inst: &InstanceModel, field: &str) -> String {
        format!(
            "<{} as {}>::base(&*self.{field})",
            self.impl_path(inst),
            self.comp_item(inst.component, "Handlers")
        )
    }

    /// The expression for an output endpoint: an `OutputPort` reference.
    fn output_expr(
        &self,
        inst: &InstanceModel,
        field: &str,
        comp: &ComponentModel,
        port: &str,
        num: u64,
    ) -> Result<String> {
        let base = self.base_of(inst, field);
        let p = comp.port(port).expect("resolved port");
        Ok(match p {
            PortInstance::General {
                kind: GeneralPortKind::Output,
                size,
                ..
            } => {
                if *size == 1 {
                    format!("{base}.{}", snake_ident(port))
                } else {
                    format!("{base}.{}[{num}]", snake_ident(port))
                }
            }
            PortInstance::Special { kind, .. } => {
                let glue = match kind {
                    SpecialPortKind::CommandReg => "cmd.cmd_reg_out",
                    SpecialPortKind::CommandResp => "cmd.cmd_response_out",
                    SpecialPortKind::Event => "evt.log_out",
                    SpecialPortKind::TextEvent => "evt.text_log_out",
                    SpecialPortKind::TimeGet => "evt.time_out",
                    SpecialPortKind::Telemetry => "tlm.tlm_out",
                    SpecialPortKind::ParamGet => "prm.prm_get_out",
                    SpecialPortKind::ParamSet => "prm.prm_set_out",
                    SpecialPortKind::ProductGet
                    | SpecialPortKind::ProductSend
                    | SpecialPortKind::ProductRequest => {
                        return Ok(format!("{base}.{}", snake_ident(port)));
                    }
                    SpecialPortKind::CommandRecv | SpecialPortKind::ProductRecv => {
                        return Err(Diagnostic::codegen(
                            p.loc().clone(),
                            format!("{port} is not an output port"),
                        ));
                    }
                };
                format!("{base}.{glue}")
            }
            _ => {
                return Err(Diagnostic::codegen(
                    p.loc().clone(),
                    format!("{port} is not an output port"),
                ));
            }
        })
    }

    /// The expression for an input endpoint: a `PortRef` built by the
    /// generated factory.
    fn input_expr(&self, inst: &InstanceModel, field: &str, port: &str, num: u64) -> String {
        format!(
            "<{} as {}>::{}(&self.{field}, {num})",
            self.impl_path(inst),
            self.comp_item(inst.component, "Component"),
            snake_ident(port)
        )
    }

    /// Emit everything for a topology.
    pub(super) fn emit_topology(&mut self, sym: SymId) -> Result<()> {
        let t = self.a.topologies[&sym].clone();
        let s = self.a.symbols.sym(sym);
        let name = format!("{}Topology", s.name);
        let docs = s.docs.clone();
        let instances: Vec<(String, InstanceModel)> = t
            .instances
            .iter()
            .map(|i| {
                (
                    snake_ident(&self.a.symbols.sym(*i).name),
                    self.a.instances[i].clone(),
                )
            })
            .collect();
        for (_, inst) in &instances {
            if !self.a.components.contains_key(&inst.component) {
                return Err(Diagnostic::codegen(
                    self.a.symbols.sym(inst.sym).loc.clone(),
                    "instance of an unresolved component",
                ));
            }
        }

        self.line("");
        self.line(&format!("// ---- topology {} ----", s.qualified_name()));
        self.line("");
        self.doc(&docs);
        self.line(&format!("/// The `{}` topology: its component instances and the wiring phases (C++ `{}TopologyAc`).", s.name, s.name));
        self.line(&format!("pub struct {name} {{"));
        self.indent();
        for (field, inst) in &instances {
            let isym = self.a.symbols.sym(inst.sym);
            self.doc(&isym.docs);
            self.line(&format!(
                "/// Instance `{}` of `{}` (base id {:#x}).",
                isym.name,
                self.a.symbols.sym(inst.component).qualified_name(),
                inst.base_id
            ));
            self.line(&format!(
                "pub {field}: ::std::sync::Arc<{}>,",
                self.impl_path(inst)
            ));
        }
        self.dedent();
        self.line("}");
        self.line("");
        self.line(&format!("impl {name} {{"));
        self.indent();

        // set_id_bases
        self.line("/// Set every instance's id base (phase: set id bases).");
        self.line("pub fn set_id_bases(&self) {");
        self.indent();
        for (field, inst) in &instances {
            self.line(&format!(
                "{}.set_id_base({:#x});",
                self.base_of(inst, field),
                inst.base_id
            ));
        }
        self.dedent();
        self.line("}");

        // init
        self.line("/// Create the message queues of queued and active instances (phase: init).");
        self.line("pub fn init(&self) {");
        self.indent();
        for (field, inst) in &instances {
            let comp = &self.a.components[&inst.component];
            if comp.kind != ComponentKind::Passive {
                let Some(depth) = inst.queue_size else {
                    return Err(Diagnostic::codegen(
                        self.a.symbols.sym(inst.sym).loc.clone(),
                        format!(
                            "instance {} of a {} component needs a queue size",
                            self.a.symbols.sym(inst.sym).name,
                            comp.kind
                        ),
                    ));
                };
                self.line(&format!("{}.init({depth});", self.base_of(inst, field)));
            }
        }
        self.dedent();
        self.line("}");

        // connect
        self.line("/// Make every connection of the resolved graph (phase: connect ports).");
        self.line("pub fn connect(&self) {");
        self.indent();
        let by_sym = |s: SymId| -> (String, InstanceModel) {
            instances
                .iter()
                .find(|(_, i)| i.sym == s)
                .cloned()
                .expect("instance in topology")
        };
        let mut last_graph = String::new();
        for c in &t.connections {
            if c.graph != last_graph {
                self.line(&format!("// {}", c.graph));
                last_graph = c.graph.clone();
            }
            let (ff, fi) = by_sym(c.from.instance);
            let (tf, ti) = by_sym(c.to.instance);
            let fcomp = self.a.components[&fi.component].clone();
            let out = self.output_expr(&fi, &ff, &fcomp, &c.from.port, c.from.num.unwrap_or(0))?;
            let inp = self.input_expr(&ti, &tf, &c.to.port, c.to.num.unwrap_or(0));
            self.line(&format!("{out}.connect_to({inp});"));
        }
        self.dedent();
        self.line("}");

        // reg_commands
        self.line("/// Register commands (phase: register commands).");
        self.line("pub fn reg_commands(&self) {");
        self.indent();
        for (field, inst) in &instances {
            let comp = &self.a.components[&inst.component];
            if !comp.commands.is_empty() && comp.special_port(SpecialPortKind::CommandReg).is_some()
            {
                self.line(&format!("{}.reg_commands();", self.base_of(inst, field)));
            }
        }
        self.dedent();
        self.line("}");

        // load_parameters
        self.line("/// Load parameters (phase: load parameters).");
        self.line("pub fn load_parameters(&self) {");
        self.indent();
        for (field, inst) in &instances {
            let comp = &self.a.components[&inst.component];
            if !comp.params.is_empty() {
                self.line(&format!(
                    "<{} as {}>::load_parameters(&*self.{field});",
                    self.impl_path(inst),
                    self.comp_item(inst.component, "Component")
                ));
            }
        }
        self.dedent();
        self.line("}");

        // tasks
        let actives: Vec<&(String, InstanceModel)> = instances
            .iter()
            .filter(|(_, i)| self.a.components[&i.component].kind == ComponentKind::Active)
            .collect();
        self.line("/// Start the tasks of active instances (phase: start tasks).");
        self.line("pub fn start_tasks(&self) {");
        self.indent();
        for (field, inst) in &actives {
            self.line(&format!(
                "{}.active.start(&self.{field}, {} as ::fprime_config::FwTaskPriorityType, {} as ::fprime_config::FwSizeType, {} as ::fprime_config::FwSizeType);",
                self.base_of(inst, field),
                inst.priority.unwrap_or(0),
                inst.stack_size.unwrap_or(0),
                inst.cpu.unwrap_or(0).max(0)
            ));
        }
        self.dedent();
        self.line("}");
        self.line("/// Ask every active instance to exit its loop (phase: teardown).");
        self.line("pub fn exit_tasks(&self) {");
        self.indent();
        for (field, inst) in &actives {
            self.line(&format!("{}.active.exit();", self.base_of(inst, field)));
        }
        self.dedent();
        self.line("}");
        self.line("/// Join every active instance's task.");
        self.line("pub fn join_tasks(&self) {");
        self.indent();
        for (field, inst) in &actives {
            self.line(&format!(
                "let _ = {}.active.join();",
                self.base_of(inst, field)
            ));
        }
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");
        let _ = snake;
        Ok(())
    }
}
