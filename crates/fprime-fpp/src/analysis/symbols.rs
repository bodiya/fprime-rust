//! Symbols and scopes (`Symbol.scala`, `NameGroup.scala`, `NestedScope`
//! in the reference compiler).
//!
//! Names live in *name groups*: a type and a constant may share a name. A
//! module is entered into every group so that it can qualify anything; an
//! enum is entered into the type and value groups (it qualifies its
//! constants); a component is entered into the component, state machine,
//! type and value groups (it qualifies its nested definitions).
//! Modules may be reopened across files: all `module M { .. }` blocks with
//! the same qualified name share one symbol and one scope.

use crate::ast::*;
use crate::error::{Diagnostic, Loc, Result};
use std::collections::HashMap;

/// Symbol id: index into [`SymbolTable::symbols`].
pub type SymId = usize;

/// Scope id: index into [`SymbolTable::scopes`].
pub type ScopeId = usize;

/// The name groups.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NameGroup {
    Component,
    Port,
    StateMachine,
    /// Component instances and topologies.
    PortInterfaceInstance,
    /// Interfaces.
    PortInterface,
    System,
    Type,
    Value,
}

impl NameGroup {
    /// All groups.
    pub const ALL: [NameGroup; 8] = [
        NameGroup::Component,
        NameGroup::Port,
        NameGroup::StateMachine,
        NameGroup::PortInterfaceInstance,
        NameGroup::PortInterface,
        NameGroup::System,
        NameGroup::Type,
        NameGroup::Value,
    ];

    fn index(self) -> usize {
        match self {
            NameGroup::Component => 0,
            NameGroup::Port => 1,
            NameGroup::StateMachine => 2,
            NameGroup::PortInterfaceInstance => 3,
            NameGroup::PortInterface => 4,
            NameGroup::System => 5,
            NameGroup::Type => 6,
            NameGroup::Value => 7,
        }
    }

    /// Human-readable name for messages.
    pub fn describe(self) -> &'static str {
        match self {
            NameGroup::Component => "component",
            NameGroup::Port => "port",
            NameGroup::StateMachine => "state machine",
            NameGroup::PortInterfaceInstance => "component instance or topology",
            NameGroup::PortInterface => "interface",
            NameGroup::System => "system",
            NameGroup::Type => "type",
            NameGroup::Value => "constant",
        }
    }
}

/// A scope: one name map per group.
#[derive(Debug, Default)]
pub struct Scope {
    maps: [HashMap<String, SymId>; 8],
}

impl Scope {
    /// Look a name up in one group.
    pub fn get(&self, group: NameGroup, name: &str) -> Option<SymId> {
        self.maps[group.index()].get(name).copied()
    }

    /// All symbols in a group, in an unspecified order.
    pub fn entries(&self, group: NameGroup) -> impl Iterator<Item = (&String, &SymId)> {
        self.maps[group.index()].iter()
    }
}

/// What a symbol defines. Definitions borrow the AST.
#[derive(Debug, Clone, Copy)]
pub enum Def<'a> {
    AbsType(&'a Node<DefAbsType>),
    AliasType(&'a Node<DefAliasType>),
    Array(&'a Node<DefArray>),
    Component(&'a Node<DefComponent>),
    ComponentInstance(&'a Node<DefComponentInstance>),
    Constant(&'a Node<DefConstant>),
    Enum(&'a Node<DefEnum>),
    /// An enum constant and its enum's symbol.
    EnumConstant(&'a Node<DefEnumConstant>, SymId),
    Interface(&'a Node<DefInterface>),
    Module,
    Port(&'a Node<DefPort>),
    StateMachine(&'a Node<DefStateMachine>),
    Struct(&'a Node<DefStruct>),
    System(&'a Node<DefSystem>),
    Topology(&'a Node<DefTopology>),
}

impl Def<'_> {
    /// The kind name for messages.
    pub fn kind_name(&self) -> &'static str {
        match self {
            Def::AbsType(_) => "abstract type",
            Def::AliasType(_) => "type alias",
            Def::Array(_) => "array type",
            Def::Component(_) => "component",
            Def::ComponentInstance(_) => "component instance",
            Def::Constant(_) => "constant",
            Def::Enum(_) => "enum",
            Def::EnumConstant(..) => "enum constant",
            Def::Interface(_) => "interface",
            Def::Module => "module",
            Def::Port(_) => "port",
            Def::StateMachine(_) => "state machine",
            Def::Struct(_) => "struct type",
            Def::System(_) => "system",
            Def::Topology(_) => "topology",
        }
    }
}

/// A symbol.
#[derive(Debug, Clone)]
pub struct Symbol<'a> {
    /// Unqualified name.
    pub name: String,
    /// Fully qualified name parts (enclosing modules/components/enums, then
    /// the name).
    pub qualified: Vec<String>,
    /// Where it is defined (the first definition for modules).
    pub loc: Loc,
    /// The definition.
    pub def: Def<'a>,
    /// The scope this symbol *owns* (modules, enums, components).
    pub scope: Option<ScopeId>,
    /// The scope stack in effect at the definition (outermost first), used
    /// to resolve names inside the definition later.
    pub def_stack: Vec<ScopeId>,
    /// Annotation lines attached to the definition.
    pub docs: Vec<String>,
    /// The symbol that syntactically encloses this one (module, component
    /// or enum), if any.
    pub parent: Option<SymId>,
}

impl Symbol<'_> {
    /// `A.B.C` form.
    pub fn qualified_name(&self) -> String {
        self.qualified.join(".")
    }
}

/// All symbols and scopes of a session.
#[derive(Debug, Default)]
pub struct SymbolTable<'a> {
    /// Symbols by id.
    pub symbols: Vec<Symbol<'a>>,
    /// Scopes by id; scope 0 is the root.
    pub scopes: Vec<Scope>,
}

impl<'a> SymbolTable<'a> {
    /// A table with an empty root scope.
    pub fn new() -> Self {
        Self {
            symbols: Vec::new(),
            scopes: vec![Scope::default()],
        }
    }

    /// The root scope id.
    pub const ROOT: ScopeId = 0;

    /// Create an empty scope.
    pub fn new_scope(&mut self) -> ScopeId {
        self.scopes.push(Scope::default());
        self.scopes.len() - 1
    }

    /// Enter `sym` under `name` in `scope` for `group`; duplicates are an
    /// error unless the existing symbol is the same one.
    pub fn put(&mut self, scope: ScopeId, group: NameGroup, name: &str, sym: SymId) -> Result<()> {
        let map = &mut self.scopes[scope].maps[group.index()];
        if let Some(prev) = map.get(name) {
            if *prev == sym {
                return Ok(());
            }
            let prev = &self.symbols[*prev];
            let cur = &self.symbols[sym];
            return Err(Diagnostic::semantic(
                cur.loc.clone(),
                format!("redefinition of {} {}", group.describe(), name),
            )
            .with_note(prev.loc.clone(), "previous definition is here"));
        }
        map.insert(name.to_string(), sym);
        Ok(())
    }

    /// Add a symbol, returning its id.
    pub fn add(&mut self, sym: Symbol<'a>) -> SymId {
        self.symbols.push(sym);
        self.symbols.len() - 1
    }

    /// The symbol by id.
    pub fn sym(&self, id: SymId) -> &Symbol<'a> {
        &self.symbols[id]
    }

    /// Look an unqualified name up along a scope stack, innermost first.
    pub fn lookup(&self, stack: &[ScopeId], group: NameGroup, name: &str) -> Option<SymId> {
        stack
            .iter()
            .rev()
            .find_map(|s| self.scopes[*s].get(group, name))
    }

    /// Resolve a qualified name in `group`: the first part is looked up
    /// along the stack, each further part inside the scope owned by the
    /// previously found symbol.
    pub fn resolve(&self, stack: &[ScopeId], group: NameGroup, qi: &QualIdent) -> Result<SymId> {
        let first = &qi.parts[0];
        let mut cur = self.lookup(stack, group, &first.data).ok_or_else(|| {
            Diagnostic::semantic(
                first.loc.clone(),
                format!("undefined {} symbol {}", group.describe(), first.data),
            )
        })?;
        for part in &qi.parts[1..] {
            let owner = &self.symbols[cur];
            let Some(scope) = owner.scope else {
                return Err(Diagnostic::semantic(
                    part.loc.clone(),
                    format!(
                        "{} {} is not a scope; cannot select {} from it",
                        owner.def.kind_name(),
                        owner.qualified_name(),
                        part.data
                    ),
                ));
            };
            cur = self.scopes[scope].get(group, &part.data).ok_or_else(|| {
                Diagnostic::semantic(
                    part.loc.clone(),
                    format!(
                        "undefined {} symbol {} in {}",
                        group.describe(),
                        part.data,
                        owner.qualified_name()
                    ),
                )
            })?;
        }
        Ok(cur)
    }

    /// Resolve the qualified name of a symbol from the root (for names
    /// spelled fully, e.g. in bindings).
    pub fn resolve_absolute(&self, group: NameGroup, parts: &[&str]) -> Option<SymId> {
        let mut cur = self.scopes[Self::ROOT].get(group, parts[0])?;
        for part in &parts[1..] {
            let scope = self.symbols[cur].scope?;
            cur = self.scopes[scope].get(group, part)?;
        }
        Some(cur)
    }
}
