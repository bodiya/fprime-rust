//! Emission of state machine definitions: the Rust equivalent of the C++
//! autocoder's `<M>StateMachineAc` (and the implicit `<M>_State` enum).
//!
//! An internal state machine `M` becomes an invocation of
//! `fprime_comp::fpp_state_machine!` in `M`'s module (the module that
//! also holds `M`'s nested type and constant definitions), written in the
//! form the macro documents and `docs/api-notes.md` ("fprime-comp: state
//! machines") specifies for a generator:
//!
//! - signals numbered from 1 in name order, each listing the states that
//!   have a transition on it;
//! - every state under its qualified name with `.` replaced by `_`, with
//!   its parent; leaf states numbered from 1 in the order of those names
//!   (the `State` enum), composites with their initial transition;
//! - typed actions, guards, signals and choices with a passing mode:
//!   `val` for `Copy` types, `str` for strings, `ref` otherwise; a
//!   choice's type is the one FPP infers.
//!
//! The implicit `State` enum symbol is not emitted on its own: the macro
//! defines `State` in the same module, so `M.State` resolves to it. An
//! external state machine (no members) is not generated; it must be bound
//! to a Rust type implementing `fprime_comp::sm::ExternalStateMachine`
//! (`--bind-type M=path`).

use super::Generator;
use super::names::ident;
use crate::analysis::{Def, StateMachineModel, SymId, Type};
use crate::error::{Diagnostic, Result};

impl Generator<'_, '_> {
    /// The passing mode of a state machine value type (`val`, `ref` or
    /// `str`).
    pub(super) fn sm_mode(&self, ty: &Type) -> &'static str {
        match self.a.underlying(ty) {
            Type::String(_) => "str",
            _ => match self.arg_kind(ty) {
                super::ArgKind::Copy => "val",
                _ => "ref",
            },
        }
    }

    /// ` (mode T)` for a typed element, empty for an untyped one.
    fn sm_typed(&self, ty: &Option<Type>) -> Result<String> {
        match ty {
            None => Ok(String::new()),
            Some(t) => Ok(format!(" ({} {})", self.sm_mode(t), self.rust_type(t)?)),
        }
    }

    /// The Rust parameter type of a state machine value in a handler or
    /// send function (`T`, `&T` or `&str`).
    pub(super) fn sm_param_type(&self, ty: &Type) -> Result<String> {
        Ok(match self.sm_mode(ty) {
            "val" => self.rust_type(ty)?,
            "str" => "&str".to_string(),
            _ => format!("&{}", self.rust_type(ty)?),
        })
    }

    /// The module of state machine `sym` (where the macro defines its
    /// items).
    pub(super) fn sm_module(&self, sym: SymId) -> Vec<String> {
        let mut m = self.module_of(sym);
        m.push(self.a.symbols.sym(sym).name.clone());
        m
    }

    /// Path from the current module to an item of state machine `sym`'s
    /// module.
    pub(super) fn sm_item(&self, sym: SymId, item: &str) -> String {
        let m = self.sm_module(sym);
        self.path_to_module(&m, item)
    }

    /// Whether `sym` is the implicit `State` enum of an internal state
    /// machine (defined by the macro, not emitted on its own).
    pub(super) fn is_sm_state_enum(&self, sym: SymId) -> bool {
        let s = self.a.symbols.sym(sym);
        if s.name != "State" || !matches!(s.def, Def::Enum(_)) {
            return false;
        }
        match s.parent.map(|p| self.a.symbols.sym(p).def) {
            Some(Def::StateMachine(n)) => {
                n.data.members.is_some() && self.is_implicit_state_enum(sym)
            }
            _ => false,
        }
    }

    /// Emit `fpp_state_machine!` for an internal state machine.
    pub(super) fn emit_state_machine(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let Some(m) = self.a.state_machines.get(&sym).cloned() else {
            return Ok(());
        };
        let docs = s.docs.clone();
        let qualified = s.qualified_name();
        let name = s.name.clone();
        if let Some(e) = self.a.symbols.scopes[s.scope.expect("state machine scope")]
            .get(crate::analysis::NameGroup::Type, "State")
            .filter(|e| !self.is_sm_state_enum(*e))
        {
            return Err(Diagnostic::codegen(
                self.a.symbols.sym(e).loc.clone(),
                format!(
                    "state machine {qualified}: a user-defined `State` type clashes with the implicit state enum"
                ),
            ));
        }
        let leaves = m.states.iter().filter(|s| s.is_leaf()).count() as u64 + 1;
        let repr = if leaves <= u64::from(u8::MAX) {
            "u8"
        } else if leaves <= u64::from(u16::MAX) {
            "u16"
        } else {
            "u32"
        };
        self.line("");
        self.line(&format!("// ---- state machine {qualified} ----"));
        self.line("");
        self.line("::fprime_comp::fpp_state_machine! {");
        self.indent();
        self.doc(&docs);
        self.line(&format!("state machine {name}: {repr} {{"));
        self.indent();
        // Actions and guards.
        self.line("actions {");
        self.indent();
        for a in &m.actions {
            self.doc(&a.docs);
            let t = self.sm_typed(&a.ty)?;
            self.line(&format!("{} => action_{}{t},", ident(&a.name), a.name));
        }
        self.dedent();
        self.line("}");
        self.line("guards {");
        self.indent();
        for g in &m.guards {
            self.doc(&g.docs);
            let t = self.sm_typed(&g.ty)?;
            self.line(&format!("{} => guard_{}{t},", ident(&g.name), g.name));
        }
        self.dedent();
        self.line("}");
        // Signals, numbered in name order, with their transitions.
        self.line("signals {");
        self.indent();
        for (k, sig) in m.signals_by_name() {
            self.doc(&sig.docs);
            let t = self.sm_typed(&sig.ty)?;
            self.line(&format!(
                "{} = {k} => send_signal_{}{t} {{",
                ident(&sig.name),
                sig.name
            ));
            self.indent();
            for st in &m.states {
                for tr in st.transitions.iter().filter(|t| t.signal == sig.name) {
                    let mut l = format!("{} =>", ident(&st.flat_name()));
                    if let Some(g) = &tr.guard {
                        l.push_str(&format!(" if {}", ident(g)));
                    }
                    l.push_str(&Self::sm_do(&tr.actions));
                    if let Some(target) = tr.target {
                        l.push_str(&format!(" enter {}", ident(&m.target_name(target))));
                    }
                    l.push(';');
                    self.line(&l);
                }
            }
            self.dedent();
            self.line("}");
        }
        self.dedent();
        self.line("}");
        // The machine's initial transition.
        self.line(&format!(
            "initial{} enter {};",
            Self::sm_do(&m.initial.actions),
            ident(&m.target_name(m.initial.target))
        ));
        // States.
        let numbers = m.leaf_numbers();
        self.line("states {");
        self.indent();
        for (i, st) in m.states.iter().enumerate() {
            self.doc(&st.docs);
            let mut head = format!("state {}", ident(&st.flat_name()));
            if let Some(k) = numbers.get(&i) {
                head.push_str(&format!(" = {k}"));
            }
            if let Some(p) = st.parent {
                head.push_str(&format!(" in {}", ident(&m.states[p].flat_name())));
            }
            let mut body: Vec<String> = Vec::new();
            if let Some(init) = &st.initial {
                body.push(format!(
                    "initial{} enter {};",
                    Self::sm_do(&init.actions),
                    ident(&m.target_name(init.target))
                ));
            }
            if !st.entry.is_empty() {
                body.push(format!("entry{}", Self::sm_do(&st.entry)));
            }
            if !st.exit.is_empty() {
                body.push(format!("exit{}", Self::sm_do(&st.exit)));
            }
            if body.is_empty() {
                self.line(&format!("{head} {{}}"));
            } else {
                self.line(&format!("{head} {{"));
                self.indent();
                for l in body {
                    self.line(&l);
                }
                self.dedent();
                self.line("}");
            }
        }
        self.dedent();
        self.line("}");
        // Choices.
        self.line("choices {");
        self.indent();
        for c in &m.choices {
            self.doc(&c.docs);
            let mut head = format!("choice {}", ident(&c.flat_name()));
            if let Some(p) = c.parent {
                head.push_str(&format!(" in {}", ident(&m.states[p].flat_name())));
            }
            head.push_str(&self.sm_typed(&c.ty)?);
            self.line(&format!(
                "{head} {{ if {}{} enter {} else{} enter {} }}",
                ident(&c.guard),
                Self::sm_do(&c.if_transition.actions),
                ident(&m.target_name(c.if_transition.target)),
                Self::sm_do(&c.else_transition.actions),
                ident(&m.target_name(c.else_transition.target)),
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

    /// ` do { a, b }`, or nothing for no actions.
    fn sm_do(actions: &[String]) -> String {
        if actions.is_empty() {
            String::new()
        } else {
            let list: Vec<String> = actions.iter().map(|a| ident(a)).collect();
            format!(" do {{ {} }}", list.join(", "))
        }
    }

    /// Whether an enum symbol named `State` in a state machine is the one
    /// the front end added (its first constant is `__FPRIME_UNINITIALIZED`).
    fn is_implicit_state_enum(&self, sym: SymId) -> bool {
        match self.a.symbols.sym(sym).def {
            Def::Enum(n) => n
                .data
                .constants
                .first()
                .is_some_and(|c| c.node.data.name == "__FPRIME_UNINITIALIZED"),
            _ => false,
        }
    }

    /// The state machine model of an internal machine.
    pub(super) fn sm_model(&self, sym: SymId) -> Option<&StateMachineModel> {
        self.a.state_machines.get(&sym)
    }
}
