//! The Rust back end: from the analysis model to Rust source targeting
//! this workspace's framework crates.
//!
//! What is emitted, per FPP construct:
//!
//! | FPP | Rust |
//! |-----|------|
//! | `module M` | `pub mod M` (nested as in the model) |
//! | `constant` | `pub const` (scalars, strings) or a `pub fn` returning the value (arrays, structs) |
//! | `enum` | `fprime_fw::fpp_enum!` |
//! | `struct` | `fprime_fw::fpp_struct!` (array members get a helper `fpp_array!` type) |
//! | `array` | `fprime_fw::fpp_array!` |
//! | `type T = U` | `pub type T = U` |
//! | `type T` | a `pub use` of the bound Rust type (abstract types must be bound) |
//! | `port P` | `pub trait PPort: Send + Sync { fn invoke(&self, port_num, ..) }` |
//! | `component C` | `CBase` (the autocoded base: ports, glue, queue, ids), `CHandlers` (the handler trait the implementation provides), `CComponent` (base access), generated adapters, dispatch, event/telemetry/parameter helpers |
//! | `topology T` | `TTopology`: instance fields, `set_id_bases`, `connect`, `reg_commands`, `load_parameters`, task start/exit/join |
//!
//! Framework definitions the port already hand-writes (`Fw.Time`,
//! `Fw.Cmd`, config aliases, ...) are *bound* to their existing Rust items
//! instead of being regenerated; see [`Bindings`].
//!
//! Every path in the output is absolute (`::fprime_fw::...`) or relative
//! to the generated module tree (`super::super::Fw::X`), so the output can
//! be `include!`d at any module of a crate that depends on the framework
//! crates.

pub mod component;
pub mod names;
pub mod ports;
pub mod topology;
pub mod types;

use crate::analysis::{Analysis, Def, NameGroup, SymId, Type, TypeDef};
use crate::error::{Diagnostic, Loc, Result};
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};

/// How a Rust type is passed as a port/command argument.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArgKind {
    /// Passed by value when an FPP value parameter (`Copy` types).
    Copy,
    /// Passed by shared reference when an FPP value parameter.
    Ref,
    /// A serializable buffer: always `&mut T`, nested with a length prefix
    /// in queue messages (`buf` mode).
    Buf,
    /// Always owned (`Fw.Buffer`: moves through the port graph).
    Owned,
}

/// A bound Rust type.
#[derive(Debug, Clone)]
pub struct TypeBinding {
    /// Absolute Rust path.
    pub path: String,
    /// Passing mode.
    pub kind: ArgKind,
    /// A Rust constant expression for the maximum serialized size, when
    /// the type does not implement `FppSized` (buffers: `2 + capacity`).
    pub size_expr: Option<String>,
}

/// A bound Rust port trait.
#[derive(Debug, Clone)]
pub struct PortBinding {
    /// Absolute Rust path of the trait.
    pub path: String,
    /// Per-parameter passing modes (`val`, `ref`, `mut`, `buf`, `owned`)
    /// when the hand-written trait deviates from the default rules.
    pub modes: Option<Vec<String>>,
}

/// The mapping from FPP framework definitions to existing Rust items.
#[derive(Debug, Clone)]
pub struct Bindings {
    /// FPP qualified type name -> Rust type.
    pub types: HashMap<String, TypeBinding>,
    /// FPP qualified port name -> Rust trait.
    pub ports: HashMap<String, PortBinding>,
}

impl Default for Bindings {
    fn default() -> Self {
        Self::framework()
    }
}

impl Bindings {
    /// No bindings at all (everything is generated).
    pub fn empty() -> Self {
        Self {
            types: HashMap::new(),
            ports: HashMap::new(),
        }
    }

    /// The bindings for this workspace's `fprime-config`, `fprime-fw` and
    /// `fprime-comp` crates.
    pub fn framework() -> Self {
        let mut b = Self::empty();
        // Config type aliases.
        for name in [
            "FwSizeType",
            "FwSignedSizeType",
            "FwIndexType",
            "FwAssertArgType",
            "FwTaskPriorityType",
            "FwQueuePriorityType",
            "FwIdType",
            "FwTaskIdType",
            "FwChanIdType",
            "FwDpIdType",
            "FwDpPriorityType",
            "FwEventIdType",
            "FwOpcodeType",
            "FwPrmIdType",
            "FwSizeStoreType",
            "FwTimeContextStoreType",
            "FwTlmPacketizeIdType",
            "FwTraceIdType",
            "FwEnumStoreType",
            "FwTimeBaseStoreType",
            "FwPacketDescriptorType",
            "FwBuffSizeType",
        ] {
            b.bind_type(name, &format!("::fprime_config::{name}"), ArgKind::Copy);
        }
        for (name, rust) in [
            ("PlatformSizeType", "u64"),
            ("PlatformSignedSizeType", "i64"),
            ("PlatformIndexType", "i16"),
            ("PlatformAssertArgType", "i32"),
            ("PlatformTaskPriorityType", "u8"),
            ("PlatformQueuePriorityType", "u8"),
            ("PlatformTaskIdType", "i32"),
            ("PlatformPointerCastType", "u64"),
            ("PlatformIntType", "i32"),
            ("PlatformUIntType", "u32"),
        ] {
            b.bind_type(name, rust, ArgKind::Copy);
        }
        b.bind_type("TimeBase", "::fprime_fw::TimeBase", ArgKind::Copy);
        // Fw abstract types.
        b.bind_type_sized("Fw.Buffer", "::fprime_fw::Buffer", ArgKind::Owned, "8");
        b.bind_type("Fw.Time", "::fprime_fw::Time", ArgKind::Ref);
        b.bind_type("Fw.TimeInterval", "::fprime_fw::TimeInterval", ArgKind::Ref);
        b.bind_type_sized(
            "Fw.CmdArgBuffer",
            "::fprime_fw::CmdArgBuffer",
            ArgKind::Buf,
            "2 + ::fprime_config::FW_CMD_ARG_BUFFER_MAX_SIZE",
        );
        b.bind_type_sized(
            "Fw.LogBuffer",
            "::fprime_fw::LogBuffer",
            ArgKind::Buf,
            "2 + ::fprime_config::FW_LOG_BUFFER_MAX_SIZE",
        );
        b.bind_type_sized(
            "Fw.TlmBuffer",
            "::fprime_fw::TlmBuffer",
            ArgKind::Buf,
            "2 + ::fprime_config::FW_TLM_BUFFER_MAX_SIZE",
        );
        b.bind_type_sized(
            "Fw.ParamBuffer",
            "::fprime_fw::ParamBuffer",
            ArgKind::Buf,
            "2 + ::fprime_config::FW_PARAM_BUFFER_MAX_SIZE",
        );
        b.bind_type_sized(
            "Fw.ComBuffer",
            "::fprime_fw::ComBuffer",
            ArgKind::Buf,
            "2 + ::fprime_config::FW_COM_BUFFER_MAX_SIZE",
        );
        b.bind_type(
            "Fw.TextLogString",
            "::fprime_fw::TextLogString",
            ArgKind::Ref,
        );
        b.bind_type("Fw.String", "::fprime_fw::FwDefaultString", ArgKind::Ref);
        b.bind_type("Fw.PolyType", "::fprime_fw::PolyType", ArgKind::Ref);
        b.bind_type("Fw.CmdStringArg", "::fprime_fw::CmdStringArg", ArgKind::Ref);
        b.bind_type("Fw.LogStringArg", "::fprime_fw::LogStringArg", ArgKind::Ref);
        b.bind_type(
            "Fw.FileNameString",
            "::fprime_fw::FileNameString",
            ArgKind::Ref,
        );
        b.bind_type("Fw.ObjectName", "::fprime_fw::ObjectName", ArgKind::Ref);
        // Fw enums the workspace hand-writes.
        for (name, rust) in [
            ("Fw.CmdResponse", "::fprime_fw::CmdResponse"),
            ("Fw.LogSeverity", "::fprime_fw::LogSeverity"),
            ("Fw.ParamValid", "::fprime_fw::ParamValid"),
            ("Fw.TlmValid", "::fprime_fw::TlmValid"),
            ("Fw.Success", "::fprime_fw::Success"),
            ("Fw.Enabled", "::fprime_fw::Enabled"),
            ("Fw.Health", "::fprime_fw::Health"),
            ("Fw.Wait", "::fprime_fw::Wait"),
            ("Fw.Completed", "::fprime_fw::Completed"),
            ("Fw.DeserialStatus", "::fprime_fw::DeserialStatus"),
            ("Fw.DpState", "::fprime_fw::dp::DpState"),
            ("Fw.DpCfg.ProcType", "::fprime_fw::dp::ProcType"),
            ("ComCfg.Apid", "::fprime_fw::Apid"),
            ("ComCfg.Pvn", "::fprime_fw::Pvn"),
        ] {
            b.bind_type(name, rust, ArgKind::Copy);
        }
        b.bind_type(
            "ComCfg.FrameContext",
            "::fprime_fw::FrameContext",
            ArgKind::Ref,
        );
        // Ports.
        for (name, rust) in [
            ("Fw.Cmd", "::fprime_comp::CmdPort"),
            ("Fw.CmdReg", "::fprime_comp::CmdRegPort"),
            ("Fw.CmdResponse", "::fprime_comp::CmdResponsePort"),
            ("Fw.Log", "::fprime_comp::LogPort"),
            ("Fw.LogText", "::fprime_comp::LogTextPort"),
            ("Fw.Tlm", "::fprime_comp::TlmPort"),
            ("Fw.Time", "::fprime_comp::TimePort"),
            ("Fw.Com", "::fprime_comp::ComPort"),
            ("Fw.BufferSend", "::fprime_comp::BufferSendPort"),
            ("Fw.BufferGet", "::fprime_comp::BufferGetPort"),
            ("Fw.PrmGet", "::fprime_comp::PrmGetPort"),
            ("Fw.PrmSet", "::fprime_comp::PrmSetPort"),
            ("Fw.SuccessCondition", "::fprime_comp::SuccessConditionPort"),
            ("Fw.FatalEvent", "::fprime_comp::FatalEventPort"),
            ("Fw.DpGet", "::fprime_fw::dp::DpGetPort"),
            ("Fw.DpSend", "::fprime_fw::dp::DpSendPort"),
            ("Fw.DpRequest", "::fprime_fw::dp::DpRequestPort"),
            ("Fw.DpResponse", "::fprime_fw::dp::DpResponsePort"),
            ("Svc.Sched", "::fprime_comp::SchedPort"),
            ("Svc.Cycle", "::fprime_comp::CyclePort"),
            ("Svc.Ping", "::fprime_comp::PingPort"),
            ("Svc.WatchDog", "::fprime_comp::WatchDogPort"),
            (
                "Svc.ComDataWithContext",
                "::fprime_comp::ComDataWithContextPort",
            ),
        ] {
            b.bind_port(name, rust);
        }
        // Hand-written traits whose parameter passing deviates from the
        // default rules (FPP `ref` on an owned `Fw.Buffer` is a move for
        // sends and an out-parameter for gets).
        b.bind_port_modes("Fw.BufferSend", "::fprime_comp::BufferSendPort", &["owned"]);
        b.bind_port_modes(
            "Fw.DpGet",
            "::fprime_fw::dp::DpGetPort",
            &["val", "val", "mut"],
        );
        b.bind_port_modes(
            "Fw.DpSend",
            "::fprime_fw::dp::DpSendPort",
            &["val", "owned"],
        );
        b.bind_port_modes(
            "Fw.DpResponse",
            "::fprime_fw::dp::DpResponsePort",
            &["val", "owned", "val"],
        );
        b.bind_port_modes(
            "Svc.ComDataWithContext",
            "::fprime_comp::ComDataWithContextPort",
            &["owned", "ref"],
        );
        b
    }

    /// Bind an FPP type to a Rust path.
    pub fn bind_type(&mut self, fpp: &str, rust: &str, kind: ArgKind) {
        self.types.insert(
            fpp.to_string(),
            TypeBinding {
                path: rust.to_string(),
                kind,
                size_expr: None,
            },
        );
    }

    /// Bind an FPP type with an explicit maximum serialized size
    /// expression (for types without `FppSized`).
    pub fn bind_type_sized(&mut self, fpp: &str, rust: &str, kind: ArgKind, size_expr: &str) {
        self.types.insert(
            fpp.to_string(),
            TypeBinding {
                path: rust.to_string(),
                kind,
                size_expr: Some(size_expr.to_string()),
            },
        );
    }

    /// Bind an FPP port to a Rust trait path.
    pub fn bind_port(&mut self, fpp: &str, rust: &str) {
        self.ports.insert(
            fpp.to_string(),
            PortBinding {
                path: rust.to_string(),
                modes: None,
            },
        );
    }

    /// Bind an FPP port to a Rust trait path with explicit parameter
    /// passing modes.
    pub fn bind_port_modes(&mut self, fpp: &str, rust: &str, modes: &[&str]) {
        self.ports.insert(
            fpp.to_string(),
            PortBinding {
                path: rust.to_string(),
                modes: Some(modes.iter().map(|m| m.to_string()).collect()),
            },
        );
    }
}

/// Generator options.
#[derive(Debug, Clone)]
pub struct Options {
    /// Bindings to existing Rust items.
    pub bindings: Bindings,
    /// Rust path prefix for component implementation types named by
    /// instances without a `type` clause (default `crate::`): an instance
    /// of `Ref.SignalGen` is assumed to be implemented by
    /// `crate::Ref::SignalGen`.
    pub impl_prefix: String,
    /// Files whose definitions are generated (everything else is imported
    /// for resolution only). Empty means all files.
    pub targets: Vec<PathBuf>,
    /// The module path, inside the crate that includes the output, at
    /// which the output is included (e.g. `generated`). When set, the
    /// generated `impl_<c>_component!` macros name their traits absolutely
    /// (`$crate::generated::...`) instead of relying on imports at the
    /// invocation site.
    pub include_path: Option<String>,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            bindings: Bindings::framework(),
            impl_prefix: "crate::".into(),
            targets: Vec::new(),
            include_path: None,
        }
    }
}

/// Generate Rust source for the target definitions of an analysis.
pub fn generate(analysis: &Analysis<'_>, options: &Options) -> Result<String> {
    let mut g = Generator {
        a: analysis,
        opts: options,
        out: String::new(),
        module: Vec::new(),
        indent: 0,
        helper_arrays: Vec::new(),
    };
    g.run()?;
    Ok(g.out)
}

/// The generator state.
pub struct Generator<'g, 'a> {
    /// The analysis.
    pub a: &'g Analysis<'a>,
    /// Options.
    pub opts: &'g Options,
    /// Output buffer.
    out: String,
    /// The FPP module path being emitted.
    module: Vec<String>,
    /// Current indentation (levels of 4 spaces).
    indent: usize,
    /// Helper array types synthesized for struct members `x: [n] T`, as
    /// (owner module path, type name, element type, size).
    helper_arrays: Vec<(Vec<String>, String, Type, u64)>,
}

/// A module tree of symbols to generate.
#[derive(Default)]
struct ModuleTree {
    symbols: Vec<SymId>,
    children: BTreeMap<String, ModuleTree>,
}

impl<'g, 'a> Generator<'g, 'a> {
    // -- output helpers --------------------------------------------------------------

    /// Emit a line at the current indentation.
    pub fn line(&mut self, s: &str) {
        if s.is_empty() {
            self.out.push('\n');
            return;
        }
        for _ in 0..self.indent {
            self.out.push_str("    ");
        }
        self.out.push_str(s);
        self.out.push('\n');
    }

    /// Emit several lines.
    pub fn lines(&mut self, s: &str) {
        for l in s.lines() {
            self.line(l);
        }
    }

    /// Emit `/// ` doc lines.
    pub fn doc(&mut self, lines: &[String]) {
        for l in lines {
            let l = l.trim();
            if l.is_empty() {
                self.line("///");
            } else {
                self.line(&format!("/// {l}"));
            }
        }
    }

    /// Increase indentation.
    pub fn indent(&mut self) {
        self.indent += 1;
    }

    /// Decrease indentation.
    pub fn dedent(&mut self) {
        self.indent = self.indent.saturating_sub(1);
    }

    /// The current module path (for relative paths).
    pub fn module_path(&self) -> &[String] {
        &self.module
    }

    // -- target selection --------------------------------------------------------------

    /// Whether a symbol is defined in a target file.
    fn is_target(&self, sym: SymId) -> bool {
        if self.opts.targets.is_empty() {
            return true;
        }
        let mut loc: &Loc = &self.a.symbols.sym(sym).loc;
        while let Some(inc) = &loc.including {
            loc = inc;
        }
        let file = canonical(&loc.file);
        self.opts.targets.iter().any(|t| canonical(t) == file)
    }

    /// Whether a symbol is bound to an existing Rust item (so not
    /// generated).
    pub fn is_bound_type(&self, sym: SymId) -> bool {
        self.opts
            .bindings
            .types
            .contains_key(&self.a.symbols.sym(sym).qualified_name())
    }

    /// Whether a port is bound.
    pub fn is_bound_port(&self, sym: SymId) -> bool {
        self.opts
            .bindings
            .ports
            .contains_key(&self.a.symbols.sym(sym).qualified_name())
    }

    fn run(&mut self) -> Result<()> {
        // Collect generated symbols into a module tree.
        let mut tree = ModuleTree::default();
        for &sym in &self.a.order {
            let s = self.a.symbols.sym(sym);
            let generated = match s.def {
                Def::AbsType(_)
                | Def::AliasType(_)
                | Def::Array(_)
                | Def::Constant(_)
                | Def::Enum(_)
                | Def::Struct(_) => !self.is_bound_type(sym),
                Def::Port(_) => !self.is_bound_port(sym),
                Def::Component(_) | Def::Topology(_) => true,
                Def::Module
                | Def::ComponentInstance(_)
                | Def::EnumConstant(..)
                | Def::Interface(_)
                | Def::StateMachine(_)
                | Def::System(_) => false,
            };
            if !generated || !self.is_target(sym) {
                continue;
            }
            let path = self.module_of(sym);
            let mut node = &mut tree;
            for part in &path {
                node = node.children.entry(part.clone()).or_default();
            }
            node.symbols.push(sym);
        }
        self.line("// Generated by fpp-to-rust from the FPP model. Do not edit.");
        self.line("");
        self.emit_tree(&tree)?;
        Ok(())
    }

    /// The Rust module path of a symbol: its enclosing FPP modules, plus
    /// the component/state-machine module for nested definitions.
    pub fn module_of(&self, sym: SymId) -> Vec<String> {
        let s = self.a.symbols.sym(sym);
        let mut cur = s.parent;
        let mut parts: Vec<String> = Vec::new();
        while let Some(p) = cur {
            let ps = self.a.symbols.sym(p);
            match ps.def {
                Def::Module | Def::Component(_) | Def::StateMachine(_) => {
                    parts.push(ps.name.clone())
                }
                // An enum constant's enum is not a module.
                _ => {}
            }
            cur = ps.parent;
        }
        parts.reverse();
        parts
    }

    fn emit_tree(&mut self, tree: &ModuleTree) -> Result<()> {
        for &sym in &tree.symbols {
            self.emit_symbol(sym)?;
        }
        // Helper arrays synthesized while emitting this module's structs.
        let helpers: Vec<_> = self
            .helper_arrays
            .iter()
            .filter(|(m, _, _, _)| *m == self.module)
            .cloned()
            .collect();
        for (_, name, elt, size) in helpers {
            self.emit_helper_array(&name, &elt, size)?;
        }
        for (name, child) in &tree.children {
            self.line("");
            // FPP names are kept verbatim (modules and enum constants are
            // not Rust-cased), and generated code is not held to clippy.
            self.line("#[allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]");
            self.line("#[allow(clippy::all, dead_code, unused_imports, unused_macros)]");
            self.line(&format!("pub mod {} {{", names::ident(name)));
            self.indent();
            self.module.push(name.clone());
            self.emit_tree(child)?;
            self.module.pop();
            self.dedent();
            self.line("}");
        }
        Ok(())
    }

    fn emit_symbol(&mut self, sym: SymId) -> Result<()> {
        let def = self.a.symbols.sym(sym).def;
        match def {
            Def::AbsType(_) => self.emit_abs_type(sym),
            Def::AliasType(_) => self.emit_alias(sym),
            Def::Array(_) => self.emit_array(sym),
            Def::Constant(_) => self.emit_constant(sym),
            Def::Enum(_) => self.emit_enum(sym),
            Def::Struct(_) => self.emit_struct(sym),
            Def::Port(_) => self.emit_port(sym),
            Def::Component(_) => self.emit_component(sym),
            Def::Topology(_) => self.emit_topology(sym),
            _ => Ok(()),
        }
    }

    // -- paths and types ---------------------------------------------------------------

    /// A path from the current module to a generated symbol's module,
    /// then the item name.
    pub fn path_to(&self, sym: SymId, item: &str) -> String {
        let target = self.module_of(sym);
        self.path_to_module(&target, item)
    }

    /// A path from the current module to `target` module, then `item`.
    pub fn path_to_module(&self, target: &[String], item: &str) -> String {
        let mut p = String::new();
        let common = self
            .module
            .iter()
            .zip(target.iter())
            .take_while(|(a, b)| a == b)
            .count();
        for _ in common..self.module.len() {
            p.push_str("super::");
        }
        if p.is_empty() {
            p.push_str("self::");
        }
        for part in &target[common..] {
            p.push_str(&names::ident(part));
            p.push_str("::");
        }
        p.push_str(item);
        p
    }

    /// The Rust path of a type symbol (bound or generated).
    pub fn type_path(&self, sym: SymId) -> Result<String> {
        let s = self.a.symbols.sym(sym);
        if let Some(b) = self.opts.bindings.types.get(&s.qualified_name()) {
            return Ok(b.path.clone());
        }
        match s.def {
            Def::AbsType(_) => Err(Diagnostic::codegen(
                s.loc.clone(),
                format!(
                    "abstract type {} has no Rust binding; pass --bind-type {}=<rust path>",
                    s.qualified_name(),
                    s.qualified_name()
                ),
            )),
            _ => Ok(self.path_to(sym, &names::ident(&s.name))),
        }
    }

    /// The Rust path of a port trait (bound or generated).
    pub fn port_path(&self, sym: SymId) -> String {
        let s = self.a.symbols.sym(sym);
        if let Some(b) = self.opts.bindings.ports.get(&s.qualified_name()) {
            return b.path.clone();
        }
        self.path_to(sym, &format!("{}Port", s.name))
    }

    /// The passing mode of parameter `idx` of port `port` (bound modes win
    /// over the default rules). Modes: `val`, `ref`, `mut`, `buf`, `owned`.
    pub fn mode_in(
        &self,
        port: Option<SymId>,
        idx: usize,
        p: &crate::analysis::ParamDef,
    ) -> String {
        if let Some(port) = port {
            let q = self.a.symbols.sym(port).qualified_name();
            if let Some(PortBinding { modes: Some(m), .. }) = self.opts.bindings.ports.get(&q) {
                if let Some(mode) = m.get(idx) {
                    return mode.clone();
                }
            }
        }
        self.default_mode(p).to_string()
    }

    /// The default passing mode of a parameter.
    pub fn default_mode(&self, p: &crate::analysis::ParamDef) -> &'static str {
        match (self.arg_kind(&p.ty), p.kind) {
            (ArgKind::Owned, _) => "owned",
            (ArgKind::Buf, _) => "buf",
            (_, crate::ast::FormalParamKind::Ref) => "mut",
            (ArgKind::Copy, _) => "val",
            (ArgKind::Ref, _) => "ref",
        }
    }

    /// The Rust parameter type for a parameter passed in `mode`.
    pub fn type_for_mode(&self, ty: &Type, mode: &str) -> Result<String> {
        let base = self.rust_type(ty)?;
        Ok(match mode {
            "owned" | "val" => base,
            "buf" | "mut" => format!("&mut {base}"),
            _ => format!("&{base}"),
        })
    }

    /// Render a type as Rust.
    pub fn rust_type(&self, t: &Type) -> Result<String> {
        Ok(match t {
            Type::Int(k) => k.rust_name().to_string(),
            Type::Float(crate::ast::FloatKind::F32) => "f32".into(),
            Type::Float(crate::ast::FloatKind::F64) => "f64".into(),
            Type::Bool => "bool".into(),
            Type::Integer => "i64".into(),
            Type::String(n) => format!(
                "::fprime_fw::FwString<{}>",
                n.unwrap_or(crate::analysis::types::DEFAULT_STRING_SIZE)
            ),
            Type::Abs(s) | Type::Alias(s) | Type::Enum(s) | Type::Array(s) | Type::Struct(s) => {
                self.type_path(*s)?
            }
            Type::AnonArray(Some(n), e) => format!("[{}; {n}]", self.rust_type(e)?),
            Type::AnonArray(None, e) => format!("[{}]", self.rust_type(e)?),
            Type::AnonStruct(_) => {
                return Err(Diagnostic::codegen(
                    Loc::none(),
                    "anonymous struct types cannot be rendered in Rust",
                ));
            }
        })
    }

    /// The passing mode of a type.
    pub fn arg_kind(&self, t: &Type) -> ArgKind {
        match self.a.underlying(t) {
            Type::Int(_) | Type::Float(_) | Type::Bool | Type::Integer | Type::Enum(_) => {
                ArgKind::Copy
            }
            Type::String(_) => ArgKind::Ref,
            Type::Abs(s) | Type::Alias(s) | Type::Array(s) | Type::Struct(s) => self
                .opts
                .bindings
                .types
                .get(&self.a.symbols.sym(s).qualified_name())
                .map(|b| b.kind)
                .unwrap_or(ArgKind::Ref),
            Type::AnonArray(..) | Type::AnonStruct(_) => ArgKind::Ref,
        }
    }

    /// Whether a type can derive `Eq` (no floats anywhere inside).
    pub fn is_eq(&self, t: &Type) -> bool {
        match self.a.underlying(t) {
            Type::Float(_) => false,
            Type::Int(_) | Type::Bool | Type::Integer | Type::Enum(_) | Type::String(_) => true,
            Type::Abs(_) | Type::Alias(_) => false,
            Type::Array(s) => match self.a.type_defs.get(&s) {
                Some(TypeDef::Array { elt, .. }) => self.is_eq(elt),
                _ => false,
            },
            Type::Struct(s) => match self.a.type_defs.get(&s) {
                Some(TypeDef::Struct { members, .. }) => members.iter().all(|m| self.is_eq(&m.ty)),
                _ => false,
            },
            Type::AnonArray(_, e) => self.is_eq(&e),
            Type::AnonStruct(ms) => ms.iter().all(|(_, t)| self.is_eq(t)),
        }
    }

    /// Whether a type is `Copy` in Rust (for derives).
    pub fn is_copy(&self, t: &Type) -> bool {
        match self.a.underlying(t) {
            Type::Int(_) | Type::Float(_) | Type::Bool | Type::Integer | Type::Enum(_) => true,
            Type::String(_) => false,
            Type::Abs(_) | Type::Alias(_) => false,
            Type::Array(s) => match self.a.type_defs.get(&s) {
                Some(TypeDef::Array { elt, .. }) => self.is_copy(elt),
                _ => false,
            },
            Type::Struct(s) => match self.a.type_defs.get(&s) {
                Some(TypeDef::Struct { members, .. }) => {
                    members.iter().all(|m| self.is_copy(&m.ty))
                }
                _ => false,
            },
            Type::AnonArray(_, e) => self.is_copy(&e),
            Type::AnonStruct(ms) => ms.iter().all(|(_, t)| self.is_copy(t)),
        }
    }

    /// The Rust parameter type for a formal parameter (default rules).
    pub fn param_type(&self, p: &crate::analysis::ParamDef) -> Result<String> {
        self.type_for_mode(&p.ty, self.default_mode(p))
    }

    /// A Rust constant expression for the maximum serialized size of a
    /// type (for queue message sizing).
    pub fn size_expr(&self, t: &Type) -> Result<String> {
        if let Type::Abs(s) | Type::Alias(s) | Type::Array(s) | Type::Struct(s) | Type::Enum(s) = t
        {
            if let Some(b) = self
                .opts
                .bindings
                .types
                .get(&self.a.symbols.sym(*s).qualified_name())
            {
                if let Some(e) = &b.size_expr {
                    return Ok(e.clone());
                }
            }
        }
        Ok(format!(
            "<{} as ::fprime_fw::FppSized>::SERIALIZED_SIZE",
            self.rust_type(t)?
        ))
    }

    /// Find a symbol by qualified name in a group.
    pub fn find(&self, group: NameGroup, qualified: &str) -> Option<SymId> {
        let parts: Vec<&str> = qualified.split('.').collect();
        self.a.symbols.resolve_absolute(group, &parts)
    }

    /// Register a helper array type for a struct member and return its
    /// name.
    pub fn helper_array(&mut self, owner: &str, member: &str, elt: &Type, size: u64) -> String {
        let name = format!("{owner}_{member}_Array");
        let m = self.module.clone();
        if !self
            .helper_arrays
            .iter()
            .any(|(mm, n, _, _)| *mm == m && *n == name)
        {
            self.helper_arrays
                .push((m, name.clone(), elt.clone(), size));
        }
        name
    }
}

fn canonical(p: &Path) -> PathBuf {
    std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf())
}
