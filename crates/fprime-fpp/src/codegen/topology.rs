//! Emission of topologies: the Rust equivalent of the C++ autocoder's
//! `<Top>TopologyAc`.
//!
//! For an FPP topology `T` the back end emits `TTopology`, a struct with
//! one `Arc<Impl>` field per component instance, where `Impl` is the
//! instance's implementation type: the `type "path"` clause of the
//! instance definition (interpreted as a Rust path), the bound Rust type
//! of a hand-written component (see below), or
//! `<impl prefix><Module>::<Component>` by default. Its methods are the
//! C++ topology phases (C++ name in parentheses):
//!
//! - `new()` (instances) — `Impl::new("<qualified instance name>")` for
//!   every instance;
//! - `init()` (`initComponents`) — queue creation for queued and active
//!   instances; a component with state machine instances initializes them
//!   first (its generated `init`);
//! - `config_components()` (`configComponents`);
//! - `set_id_bases()` (`setBaseIds`) — every instance's base id;
//! - `connect()` (`connectComponents`) — every connection of the resolved
//!   graph, port numbers included, in graph order; a typed output to a
//!   serial input is `connect_serial`, a serial output to a typed input
//!   `connect_typed` (the call is serialized / deserialized, C++
//!   `invokeSerial`);
//! - `reg_commands()`, `read_parameters()`, `load_parameters()`;
//! - `start_tasks()` / `exit_tasks()` (`stopTasks`) / `join_tasks()`
//!   (`freeThreads`) for active instances, `tear_down_components()`,
//!   `deinit_components()`;
//! - `setup()` / `teardown()`, which run the phases in the C++ order.
//!
//! **Init specifiers.** An instance's `phase P """code"""` specifiers are
//! Rust code, placed as `fpp-to-cpp` places the C++ snippets: for the
//! phases that have a method, the code replaces the instance's default
//! code there (in a block, with `self` the topology, so the instances are
//! `self.<field>` and the topology module's items — packet tables,
//! constants — are in scope); `instances` code is the expression that
//! constructs the instance in `new()`; `configConstants` and
//! `configObjects` code are items in modules `T_ConfigConstants::<inst>` /
//! `T_ConfigObjects::<inst>` (C++ namespaces `ConfigConstants::<inst>` /
//! `ConfigObjects::<inst>`), which glob-import the topology's module.
//! Deviation: the phase methods take no `TopologyState` (C++ `state`).
//!
//! **Hand-written components.** An instance of a component bound with
//! `Bindings::bind_component` (`--bind-component`) is used through the
//! workspace's hand-written conventions (fields `active` / `queued` /
//! `base`, the glue, output-port fields, `<port>_in` factories, `init`,
//! `reg_commands`, `load_parameters`); see [`super::ComponentBinding`].
//! Every other instance must be of a generated component. A component
//! with async serial input ports is initialized with a com-buffer message
//! size (see `init`). Each telemetry packet set of the topology becomes a
//! packet table module next to the topology struct (see
//! [`super::packets`]).

use super::Generator;
use super::names::{snake, snake_ident};
use crate::analysis::{ComponentModel, InstanceModel, NameGroup, PortInstance, PortType, SymId};
use crate::ast::{ComponentKind, GeneralPortKind, SpecialPortKind};
use crate::error::{Diagnostic, Loc, Result};

/// The type of a connection endpoint.
#[derive(Clone, Copy, PartialEq, Eq)]
enum EndType {
    /// A typed port (general or special) of this port definition.
    Typed(SymId),
    /// A `serial` port.
    Serial,
}

/// The `Fpp.ToCpp.Phases` numbers (C++ `CppWriter.Phases`).
pub mod phases {
    /// Configuration constants (module `<T>_ConfigConstants::<instance>`).
    pub const CONFIG_CONSTANTS: i128 = 0;
    /// Configuration objects (module `<T>_ConfigObjects::<instance>`).
    pub const CONFIG_OBJECTS: i128 = 1;
    /// Instance construction (`new`).
    pub const INSTANCES: i128 = 2;
    /// `init`.
    pub const INIT_COMPONENTS: i128 = 3;
    /// `config_components`.
    pub const CONFIG_COMPONENTS: i128 = 4;
    /// `reg_commands`.
    pub const REG_COMMANDS: i128 = 5;
    /// `read_parameters`.
    pub const READ_PARAMETERS: i128 = 6;
    /// `load_parameters`.
    pub const LOAD_PARAMETERS: i128 = 7;
    /// `start_tasks`.
    pub const START_TASKS: i128 = 8;
    /// `exit_tasks`.
    pub const STOP_TASKS: i128 = 9;
    /// `join_tasks`.
    pub const FREE_THREADS: i128 = 10;
    /// `tear_down_components`.
    pub const TEAR_DOWN_COMPONENTS: i128 = 11;
    /// `deinit_components`.
    pub const DEINIT_COMPONENTS: i128 = 12;
}

/// Default code of one instance in a phase function.
type PhaseDefault<'f> =
    dyn Fn(&Generator<'_, '_>, &str, &InstanceModel) -> Result<Vec<String>> + 'f;

impl Generator<'_, '_> {
    /// The init specifier code of an instance for a phase.
    fn phase_code(&self, inst: &InstanceModel, phase: i128) -> Option<String> {
        inst.init_specs
            .iter()
            .find(|s| s.phase == phase)
            .map(|s| s.code.clone())
    }

    /// Emit init specifier code verbatim, one level in.
    fn emit_code(&mut self, code: &str) {
        self.indent();
        for l in code.lines() {
            let l = l.trim_end();
            if !l.trim().is_empty() {
                self.line(l);
            }
        }
        self.dedent();
    }

    /// Emit one phase method: per instance, its init specifier code for
    /// the phase (in a block) or else the default code.
    fn phase_fn<F>(
        &mut self,
        instances: &[(String, InstanceModel)],
        fname: &str,
        doc: &str,
        phase: i128,
        default: F,
    ) -> Result<()>
    where
        F: Fn(&Generator<'_, '_>, &str, &InstanceModel) -> Result<Vec<String>>,
    {
        let default: &PhaseDefault<'_> = &default;
        self.line(&format!("/// {doc}"));
        self.line(&format!("pub fn {fname}(&self) {{"));
        self.indent();
        for (field, inst) in instances {
            match self.phase_code(inst, phase) {
                Some(code) => {
                    self.line(&format!(
                        "// {} (init specifier)",
                        self.a.symbols.sym(inst.sym).name
                    ));
                    self.line("{");
                    self.emit_code(&code);
                    self.line("}");
                }
                None => {
                    for l in default(self, field, inst)? {
                        self.line(&l);
                    }
                }
            }
        }
        self.dedent();
        self.line("}");
        Ok(())
    }

    /// Emit the `configConstants` / `configObjects` code of the instances
    /// as `pub mod <module> { pub mod <instance> { .. } }` (C++ namespaces
    /// `ConfigConstants::<instance>` / `ConfigObjects::<instance>`).
    fn emit_config_modules(
        &mut self,
        instances: &[(String, InstanceModel)],
        module: &str,
        phase: i128,
        what: &str,
    ) -> Result<()> {
        let with: Vec<(String, String)> = instances
            .iter()
            .filter_map(|(_, i)| {
                self.phase_code(i, phase)
                    .map(|c| (self.a.symbols.sym(i.sym).name.clone(), c))
            })
            .collect();
        if with.is_empty() {
            return Ok(());
        }
        self.line("");
        self.line(&format!("/// Instance {what} (init specifier code)."));
        self.line("#[allow(non_snake_case, dead_code, unused_imports)]");
        self.line(&format!("pub mod {module} {{"));
        self.indent();
        for (inst, code) in with {
            self.line(&format!("/// Instance `{inst}`."));
            self.line(&format!("pub mod {} {{", super::names::ident(&inst)));
            self.indent();
            self.line("use super::super::*;");
            self.dedent();
            self.emit_code(&code);
            self.line("}");
        }
        self.dedent();
        self.line("}");
        Ok(())
    }

    /// The Rust path of an instance's implementation type.
    fn impl_path(&self, inst: &InstanceModel) -> String {
        if let Some(t) = &inst.impl_type {
            return t.clone();
        }
        let comp = self.a.symbols.sym(inst.component);
        if let Some(b) = self.opts.bindings.components.get(&comp.qualified_name()) {
            return b.path.clone();
        }
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

    /// Whether an instance is of a hand-written (bound) component.
    fn is_bound(&self, inst: &InstanceModel) -> bool {
        self.is_bound_component(inst.component)
    }

    /// The object holding an instance's ports and core: the generated
    /// base (`<Impl as Handlers>::base(&*self.field)`), or the
    /// hand-written component itself.
    fn base_of(&self, inst: &InstanceModel, field: &str) -> String {
        if self.is_bound(inst) {
            return format!("self.{field}");
        }
        format!(
            "<{} as {}>::base(&*self.{field})",
            self.impl_path(inst),
            self.comp_item(inst.component, "Handlers")
        )
    }

    /// The name of an output port field.
    fn out_field(&self, inst: &InstanceModel, port: &str) -> String {
        if self.is_bound(inst) {
            snake(port)
        } else {
            snake_ident(port)
        }
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
                let f = self.out_field(inst, port);
                if *size == 1 {
                    format!("{base}.{f}")
                } else {
                    format!("{base}.{f}[{num}]")
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
                        return Ok(format!("{base}.{}", self.out_field(inst, port)));
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
        if self.is_bound(inst) {
            // Hand-written convention: `<port>_in`.
            let s = snake(port);
            let f = if s.ends_with("_in") {
                s
            } else {
                format!("{s}_in")
            };
            return format!("{}::{f}(&self.{field}, {num})", self.impl_path(inst));
        }
        format!(
            "<{} as {}>::{}(&self.{field}, {num})",
            self.impl_path(inst),
            self.comp_item(inst.component, "Component"),
            snake_ident(port)
        )
    }

    /// The type of port `port` of `comp`: its port definition, the
    /// framework port of a special port, or serial.
    fn end_type(&self, comp: &ComponentModel, port: &str, loc: &Loc) -> Result<EndType> {
        match comp.port(port).expect("resolved port") {
            PortInstance::General {
                port: PortType::Typed(p),
                ..
            } => Ok(EndType::Typed(*p)),
            PortInstance::General {
                port: PortType::Serial,
                ..
            } => Ok(EndType::Serial),
            PortInstance::Special { kind, .. } => {
                let fpp = super::serial::special_port_type(*kind);
                match self.find(NameGroup::Port, fpp) {
                    Some(p) if self.a.ports.contains_key(&p) => Ok(EndType::Typed(p)),
                    _ => Err(Diagnostic::codegen(
                        loc.clone(),
                        format!(
                            "connecting special port {port} to a serial port needs the framework definition of {fpp}"
                        ),
                    )),
                }
            }
        }
    }

    /// Check that port type `psym` can cross a typed <-> serial
    /// connection.
    fn check_serializable(&self, psym: SymId, loc: &Loc) -> Result<()> {
        match self.serial_obstacle(psym) {
            Some(why) => Err(Diagnostic::codegen(
                loc.clone(),
                format!("cannot connect a serial port here: {why}"),
            )),
            None => Ok(()),
        }
    }

    /// Emit everything for a topology.
    pub(super) fn emit_topology(&mut self, sym: SymId) -> Result<()> {
        let t = self.a.topologies[&sym].clone();
        let s = self.a.symbols.sym(sym);
        let s_name = s.name.clone();
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

        // new (phase: instances)
        self.line("/// Construct every instance (phase: instances): `Impl::new(\"<qualified instance name>\")`, or the instance's `instances` init specifier (a Rust expression of type `Arc<Impl>`).");
        self.line("pub fn new() -> Self {");
        self.indent();
        self.line("Self {");
        self.indent();
        for (field, inst) in &instances {
            match self.phase_code(inst, phases::INSTANCES) {
                Some(code) => {
                    self.line(&format!("{field}: {{"));
                    self.emit_code(&code);
                    self.line("},");
                }
                None => {
                    let q = self.a.symbols.sym(inst.sym).qualified_name();
                    self.line(&format!("{field}: {}::new({q:?}),", self.impl_path(inst)));
                }
            }
        }
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");

        // set_id_bases
        self.line("/// Set every instance's id base (C++ `setBaseIds`).");
        self.line("pub fn set_id_bases(&self) {");
        self.indent();
        for (field, inst) in &instances {
            let core = if self.is_bound(inst) {
                let kind = self.a.components[&inst.component].kind;
                format!(
                    ".{}",
                    match kind {
                        ComponentKind::Passive => "base",
                        ComponentKind::Queued => "queued.base",
                        ComponentKind::Active => "active.queued.base",
                    }
                )
            } else {
                String::new()
            };
            self.line(&format!(
                "{}{core}.set_id_base({:#x});",
                self.base_of(inst, field),
                inst.base_id
            ));
        }
        self.dedent();
        self.line("}");

        // init
        self.phase_fn(&instances, "init", "Initialize components (C++ `initComponents`): create the message queues of queued and active instances, after initializing their state machines.", phases::INIT_COMPONENTS, |g, field, inst| {
            let comp = &g.a.components[&inst.component];
            if comp.kind == ComponentKind::Passive {
                return Ok(Vec::new());
            }
            let Some(depth) = inst.queue_size else {
                return Err(Diagnostic::codegen(
                    g.a.symbols.sym(inst.sym).loc.clone(),
                    format!(
                        "instance {} of a {} component needs a queue size",
                        g.a.symbols.sym(inst.sym).name,
                        comp.kind
                    ),
                ));
            };
            let serial_async = comp.ports.iter().any(|p| {
                matches!(
                    p,
                    PortInstance::General {
                        kind: GeneralPortKind::AsyncInput,
                        port: PortType::Serial,
                        ..
                    }
                )
            });
            // Deliberate deviation: the C++ topology calls
            // `init(QueueSizes::x, InstanceIds::x)`, so the instance index
            // lands in `msgSize` and leaves no room for a serialized call;
            // the Rust topology sizes serial calls for a com buffer.
            let args = if serial_async {
                format!("{depth}, ::fprime_config::FW_COM_BUFFER_MAX_SIZE as ::fprime_config::FwSizeType")
            } else {
                depth.to_string()
            };
            // A component with state machines initializes them first (its
            // generated `init`); otherwise the base creates the queue.
            if !g.is_bound(inst) && !comp.state_machine_instances.is_empty() {
                Ok(vec![format!(
                    "<{} as {}>::init(&*self.{field}, {args});",
                    g.impl_path(inst),
                    g.comp_item(inst.component, "Component")
                )])
            } else {
                Ok(vec![format!("{}.init({args});", g.base_of(inst, field))])
            }
        })?;

        // config_components
        self.phase_fn(
            &instances,
            "config_components",
            "Configure components (C++ `configComponents`): only init specifier code.",
            phases::CONFIG_COMPONENTS,
            |_, _, _| Ok(Vec::new()),
        )?;

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
            let tcomp = self.a.components[&ti.component].clone();
            let out = self.output_expr(&fi, &ff, &fcomp, &c.from.port, c.from.num.unwrap_or(0))?;
            let inp = self.input_expr(&ti, &tf, &c.to.port, c.to.num.unwrap_or(0));
            // Typed <-> serial connections serialize or deserialize the
            // call (C++ `registerSerialPort` + `invokeSerial`).
            let from_type = self.end_type(&fcomp, &c.from.port, &c.from.loc)?;
            let to_type = self.end_type(&tcomp, &c.to.port, &c.to.loc)?;
            let connect = match (from_type, to_type) {
                (EndType::Typed(p), EndType::Serial) => {
                    self.check_serializable(p, &c.loc)?;
                    "connect_serial"
                }
                (EndType::Serial, EndType::Typed(p)) => {
                    self.check_serializable(p, &c.loc)?;
                    "connect_typed"
                }
                _ => "connect_to",
            };
            self.line(&format!("{out}.{connect}({inp});"));
        }
        self.dedent();
        self.line("}");

        // reg_commands
        self.phase_fn(
            &instances,
            "reg_commands",
            "Register commands (C++ `regCommands`).",
            phases::REG_COMMANDS,
            |g, field, inst| {
                let comp = &g.a.components[&inst.component];
                if !comp.commands.is_empty()
                    && comp.special_port(SpecialPortKind::CommandReg).is_some()
                {
                    Ok(vec![format!("{}.reg_commands();", g.base_of(inst, field))])
                } else {
                    Ok(Vec::new())
                }
            },
        )?;

        // read_parameters
        self.phase_fn(
            &instances,
            "read_parameters",
            "Read parameters (C++ `readParameters`): only init specifier code.",
            phases::READ_PARAMETERS,
            |_, _, _| Ok(Vec::new()),
        )?;

        // load_parameters
        self.phase_fn(
            &instances,
            "load_parameters",
            "Load parameters (C++ `loadParameters`).",
            phases::LOAD_PARAMETERS,
            |g, field, inst| {
                let comp = &g.a.components[&inst.component];
                if comp.params.is_empty() {
                    Ok(Vec::new())
                } else if g.is_bound(inst) {
                    Ok(vec![format!("self.{field}.load_parameters();")])
                } else {
                    Ok(vec![format!(
                        "<{} as {}>::load_parameters(&*self.{field});",
                        g.impl_path(inst),
                        g.comp_item(inst.component, "Component")
                    )])
                }
            },
        )?;

        // tasks
        self.phase_fn(&instances, "start_tasks", "Start the tasks of active instances (C++ `startTasks`).", phases::START_TASKS, |g, field, inst| {
            if g.a.components[&inst.component].kind != ComponentKind::Active {
                return Ok(Vec::new());
            }
            Ok(vec![format!(
                "{}.active.start(&self.{field}, {} as ::fprime_config::FwTaskPriorityType, {} as ::fprime_config::FwSizeType, {} as ::fprime_config::FwSizeType);",
                g.base_of(inst, field),
                inst.priority.unwrap_or(0),
                inst.stack_size.unwrap_or(0),
                inst.cpu.unwrap_or(0).max(0)
            )])
        })?;
        self.phase_fn(
            &instances,
            "exit_tasks",
            "Ask every active instance to exit its loop (C++ `stopTasks`).",
            phases::STOP_TASKS,
            |g, field, inst| {
                if g.a.components[&inst.component].kind != ComponentKind::Active {
                    return Ok(Vec::new());
                }
                Ok(vec![format!("{}.active.exit();", g.base_of(inst, field))])
            },
        )?;
        self.phase_fn(
            &instances,
            "join_tasks",
            "Join every active instance's task (C++ `freeThreads`).",
            phases::FREE_THREADS,
            |g, field, inst| {
                if g.a.components[&inst.component].kind != ComponentKind::Active {
                    return Ok(Vec::new());
                }
                Ok(vec![format!(
                    "let _ = {}.active.join();",
                    g.base_of(inst, field)
                )])
            },
        )?;
        self.phase_fn(
            &instances,
            "tear_down_components",
            "Tear down components (C++ `tearDownComponents`): only init specifier code.",
            phases::TEAR_DOWN_COMPONENTS,
            |_, _, _| Ok(Vec::new()),
        )?;
        // C++ calls `deinit()` on every instance by default; Rust components
        // release their resources on drop, so only specifier code runs.
        self.phase_fn(&instances, "deinit_components", "Deinitialize components (C++ `deinitComponents`): only init specifier code (Rust components release their resources when dropped).", phases::DEINIT_COMPONENTS, |_, _, _| Ok(Vec::new()))?;

        // setup / teardown
        self.line("/// Set up the topology (C++ `setup`): `init`, `config_components`, `set_id_bases`, `connect`, `reg_commands`, `read_parameters`, `load_parameters`, `start_tasks`.");
        self.line("pub fn setup(&self) {");
        for f in [
            "init",
            "config_components",
            "set_id_bases",
            "connect",
            "reg_commands",
            "read_parameters",
            "load_parameters",
            "start_tasks",
        ] {
            self.line(&format!("    self.{f}();"));
        }
        self.line("}");
        self.line("/// Tear down the topology (C++ `teardown`): `exit_tasks`, `join_tasks`, `tear_down_components`, `deinit_components`.");
        self.line("pub fn teardown(&self) {");
        for f in [
            "exit_tasks",
            "join_tasks",
            "tear_down_components",
            "deinit_components",
        ] {
            self.line(&format!("    self.{f}();"));
        }
        self.line("}");
        self.dedent();
        self.line("}");

        // Configuration constants and objects (C++ namespaces
        // `ConfigConstants::<instance>` / `ConfigObjects::<instance>`).
        let tname = s_name.clone();
        self.emit_config_modules(
            &instances,
            &format!("{tname}_ConfigConstants"),
            phases::CONFIG_CONSTANTS,
            "configuration constants",
        )?;
        self.emit_config_modules(
            &instances,
            &format!("{tname}_ConfigObjects"),
            phases::CONFIG_OBJECTS,
            "configuration objects",
        )?;

        // The packet tables of the topology's telemetry packet sets.
        self.emit_packet_sets(sym)?;

        let _ = snake;
        Ok(())
    }
}
