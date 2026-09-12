//! Semantic analysis: from parsed translation units to a resolved model.
//!
//! Two passes, as in the reference compiler. The first enters every
//! definition into its scope (`EnterSymbols`); the second resolves uses
//! lazily and on demand — a type or constant is resolved the first time
//! something needs it, in the scope stack that was in effect at its
//! definition — and then builds the port, component, instance and topology
//! models that the back end consumes.

pub mod component;
pub mod eval;
pub mod format;
pub mod symbols;
pub mod topology;
pub mod types;

use crate::Session;
use crate::ast::*;
use crate::error::{Diagnostic, Loc, Result};
use std::collections::{HashMap, HashSet};

pub use component::*;
pub use eval::{StructMemberDef, TypeDef};
pub use format::Format;
pub use symbols::{Def, NameGroup, ScopeId, SymId, Symbol, SymbolTable};
pub use topology::*;
pub use types::{Type, Value};

/// A resolved port definition.
#[derive(Debug, Clone)]
pub struct PortDef {
    /// The port symbol.
    pub sym: SymId,
    /// Formal parameters.
    pub params: Vec<ParamDef>,
    /// Return type, if any.
    pub ret: Option<Type>,
}

/// A formal parameter of a port, command, event or internal port.
#[derive(Debug, Clone)]
pub struct ParamDef {
    /// Name.
    pub name: String,
    /// `ref` or value.
    pub kind: FormalParamKind,
    /// Type.
    pub ty: Type,
    /// Annotation lines.
    pub docs: Vec<String>,
}

/// The analysis state and results.
#[derive(Debug)]
pub struct Analysis<'a> {
    /// Symbols and scopes.
    pub symbols: SymbolTable<'a>,
    /// Definition order of all symbols (for deterministic output).
    pub order: Vec<SymId>,
    /// Resolved named-type definitions.
    pub type_defs: HashMap<SymId, TypeDef>,
    /// Evaluated constants and enum constants.
    pub values: HashMap<SymId, Value>,
    /// Resolved ports.
    pub ports: HashMap<SymId, PortDef>,
    /// Resolved interfaces: their port instances (imports expanded).
    pub interfaces: HashMap<SymId, Vec<PortInstance>>,
    /// Resolved components.
    pub components: HashMap<SymId, ComponentModel>,
    /// Resolved component instances.
    pub instances: HashMap<SymId, InstanceModel>,
    /// Resolved topologies.
    pub topologies: HashMap<SymId, TopologyModel>,
    /// Cycle detection for on-demand resolution.
    in_progress: HashSet<SymId>,
}

/// Run the analysis over every file of a session.
pub fn analyze(session: &Session) -> Result<Analysis<'_>> {
    let mut a = Analysis {
        symbols: SymbolTable::new(),
        order: Vec::new(),
        type_defs: HashMap::new(),
        values: HashMap::new(),
        ports: HashMap::new(),
        interfaces: HashMap::new(),
        components: HashMap::new(),
        instances: HashMap::new(),
        topologies: HashMap::new(),
        in_progress: HashSet::new(),
    };
    // Pass 1: enter symbols.
    for f in &session.files {
        let mut stack = vec![SymbolTable::ROOT];
        let mut qual = Vec::new();
        a.enter_module_members(&f.tu.members, &mut stack, &mut qual, None)?;
    }
    // Pass 2: resolve everything in definition order.
    for i in 0..a.order.len() {
        let sym = a.order[i];
        let def = a.symbols.sym(sym).def;
        let loc = a.symbols.sym(sym).loc.clone();
        match def {
            Def::AliasType(_) | Def::Enum(_) | Def::Array(_) | Def::Struct(_) => {
                a.ensure_type_def(sym, &loc)?;
            }
            Def::Constant(_) => {
                a.value_of_sym(sym, &loc)?;
            }
            Def::Port(_) => {
                a.ensure_port(sym, &loc)?;
            }
            _ => {}
        }
    }
    for i in 0..a.order.len() {
        let sym = a.order[i];
        if let Def::Interface(_) = a.symbols.sym(sym).def {
            let loc = a.symbols.sym(sym).loc.clone();
            a.ensure_interface(sym, &loc)?;
        }
    }
    for i in 0..a.order.len() {
        let sym = a.order[i];
        if let Def::Component(_) = a.symbols.sym(sym).def {
            a.resolve_component(sym)?;
        }
    }
    for i in 0..a.order.len() {
        let sym = a.order[i];
        if let Def::ComponentInstance(_) = a.symbols.sym(sym).def {
            a.resolve_instance(sym)?;
        }
    }
    for i in 0..a.order.len() {
        let sym = a.order[i];
        if let Def::Topology(_) = a.symbols.sym(sym).def {
            let loc = a.symbols.sym(sym).loc.clone();
            a.ensure_topology(sym, &loc)?;
        }
    }
    Ok(a)
}

impl<'a> Analysis<'a> {
    // -- pass 1 -----------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn add_symbol(
        &mut self,
        name: &str,
        loc: &Loc,
        def: Def<'a>,
        stack: &[ScopeId],
        qual: &[String],
        docs: Vec<String>,
        parent: Option<SymId>,
        owns_scope: bool,
    ) -> SymId {
        let scope = if owns_scope {
            Some(self.symbols.new_scope())
        } else {
            None
        };
        let mut qualified = qual.to_vec();
        qualified.push(name.to_string());
        let id = self.symbols.add(Symbol {
            name: name.to_string(),
            qualified,
            loc: loc.clone(),
            def,
            scope,
            def_stack: stack.to_vec(),
            docs,
            parent,
        });
        self.order.push(id);
        id
    }

    /// Enter a plain (non-scope-owning) definition into one group.
    #[allow(clippy::too_many_arguments)]
    fn enter_plain(
        &mut self,
        name: &str,
        loc: &Loc,
        def: Def<'a>,
        group: NameGroup,
        stack: &[ScopeId],
        qual: &[String],
        docs: Vec<String>,
        parent: Option<SymId>,
    ) -> Result<SymId> {
        let cur = *stack.last().expect("non-empty stack");
        let s = self.add_symbol(name, loc, def, stack, qual, docs, parent, false);
        self.symbols.put(cur, group, name, s)?;
        Ok(s)
    }

    fn enter_module_members(
        &mut self,
        members: &'a [ModuleMember],
        stack: &mut Vec<ScopeId>,
        qual: &mut Vec<String>,
        parent: Option<SymId>,
    ) -> Result<()> {
        let cur = *stack.last().expect("non-empty stack");
        for m in members {
            let docs: Vec<String> = m.doc_lines().cloned().collect();
            match &m.node {
                ModuleMemberNode::DefModule(node) => {
                    let name = &node.data.name;
                    // Reopen an existing module or create a new one.
                    let existing = self.symbols.scopes[cur]
                        .get(NameGroup::Value, name)
                        .filter(|s| matches!(self.symbols.sym(*s).def, Def::Module));
                    let sym = match existing {
                        Some(s) => {
                            if !docs.is_empty() {
                                self.symbols.symbols[s].docs.extend(docs);
                            }
                            s
                        }
                        None => {
                            let s = self.add_symbol(
                                name,
                                &node.loc,
                                Def::Module,
                                stack,
                                qual,
                                docs,
                                parent,
                                true,
                            );
                            for g in NameGroup::ALL {
                                self.symbols.put(cur, g, name, s)?;
                            }
                            s
                        }
                    };
                    let scope = self.symbols.sym(sym).scope.expect("module scope");
                    stack.push(scope);
                    qual.push(name.clone());
                    self.enter_module_members(&node.data.members, stack, qual, Some(sym))?;
                    qual.pop();
                    stack.pop();
                }
                ModuleMemberNode::DefAbsType(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::AbsType(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefAliasType(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::AliasType(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefArray(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Array(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefStruct(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Struct(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefConstant(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Constant(n),
                        NameGroup::Value,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefEnum(n) => {
                    self.enter_enum(n, stack, qual, docs, parent)?;
                }
                ModuleMemberNode::DefComponent(n) => {
                    self.enter_component(n, stack, qual, docs, parent)?;
                }
                ModuleMemberNode::DefComponentInstance(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::ComponentInstance(n),
                        NameGroup::PortInterfaceInstance,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefInterface(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Interface(n),
                        NameGroup::PortInterface,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefPort(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Port(n),
                        NameGroup::Port,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefStateMachine(n) => {
                    self.enter_state_machine(n, stack, qual, docs, parent)?;
                }
                ModuleMemberNode::DefSystem(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::System(n),
                        NameGroup::System,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::DefTopology(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Topology(n),
                        NameGroup::PortInterfaceInstance,
                        stack,
                        qual,
                        docs,
                        parent,
                    )?;
                }
                ModuleMemberNode::SpecInclude(_) => {
                    unreachable!("includes are spliced before analysis")
                }
                ModuleMemberNode::SpecLoc(_) => {}
            }
        }
        Ok(())
    }

    fn enter_enum(
        &mut self,
        node: &'a Node<DefEnum>,
        stack: &mut Vec<ScopeId>,
        qual: &mut Vec<String>,
        docs: Vec<String>,
        parent: Option<SymId>,
    ) -> Result<()> {
        let cur = *stack.last().expect("non-empty stack");
        let sym = self.add_symbol(
            &node.data.name,
            &node.loc,
            Def::Enum(node),
            stack,
            qual,
            docs,
            parent,
            true,
        );
        self.symbols
            .put(cur, NameGroup::Type, &node.data.name, sym)?;
        self.symbols
            .put(cur, NameGroup::Value, &node.data.name, sym)?;
        let scope = self.symbols.sym(sym).scope.expect("enum scope");
        stack.push(scope);
        qual.push(node.data.name.clone());
        for c in &node.data.constants {
            let cdocs: Vec<String> = c.doc_lines().cloned().collect();
            let cs = self.add_symbol(
                &c.node.data.name,
                &c.node.loc,
                Def::EnumConstant(&c.node, sym),
                stack,
                qual,
                cdocs,
                Some(sym),
                false,
            );
            self.symbols
                .put(scope, NameGroup::Value, &c.node.data.name, cs)?;
        }
        qual.pop();
        stack.pop();
        Ok(())
    }

    fn enter_component(
        &mut self,
        node: &'a Node<DefComponent>,
        stack: &mut Vec<ScopeId>,
        qual: &mut Vec<String>,
        docs: Vec<String>,
        parent: Option<SymId>,
    ) -> Result<()> {
        let cur = *stack.last().expect("non-empty stack");
        let sym = self.add_symbol(
            &node.data.name,
            &node.loc,
            Def::Component(node),
            stack,
            qual,
            docs,
            parent,
            true,
        );
        for g in [
            NameGroup::Component,
            NameGroup::StateMachine,
            NameGroup::Type,
            NameGroup::Value,
        ] {
            self.symbols.put(cur, g, &node.data.name, sym)?;
        }
        let scope = self.symbols.sym(sym).scope.expect("component scope");
        stack.push(scope);
        qual.push(node.data.name.clone());
        for m in &node.data.members {
            let mdocs: Vec<String> = m.doc_lines().cloned().collect();
            match &m.node {
                ComponentMemberNode::DefAbsType(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::AbsType(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        mdocs,
                        Some(sym),
                    )?;
                }
                ComponentMemberNode::DefAliasType(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::AliasType(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        mdocs,
                        Some(sym),
                    )?;
                }
                ComponentMemberNode::DefArray(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Array(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        mdocs,
                        Some(sym),
                    )?;
                }
                ComponentMemberNode::DefStruct(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Struct(n),
                        NameGroup::Type,
                        stack,
                        qual,
                        mdocs,
                        Some(sym),
                    )?;
                }
                ComponentMemberNode::DefConstant(n) => {
                    self.enter_plain(
                        &n.data.name,
                        &n.loc,
                        Def::Constant(n),
                        NameGroup::Value,
                        stack,
                        qual,
                        mdocs,
                        Some(sym),
                    )?;
                }
                ComponentMemberNode::DefEnum(n) => {
                    self.enter_enum(n, stack, qual, mdocs, Some(sym))?;
                }
                ComponentMemberNode::DefStateMachine(n) => {
                    self.enter_state_machine(n, stack, qual, mdocs, Some(sym))?;
                }
                _ => {}
            }
        }
        qual.pop();
        stack.pop();
        Ok(())
    }

    fn enter_state_machine(
        &mut self,
        node: &'a Node<DefStateMachine>,
        stack: &mut Vec<ScopeId>,
        qual: &mut Vec<String>,
        docs: Vec<String>,
        parent: Option<SymId>,
    ) -> Result<()> {
        let cur = *stack.last().expect("non-empty stack");
        let sym = self.add_symbol(
            &node.data.name,
            &node.loc,
            Def::StateMachine(node),
            stack,
            qual,
            docs,
            parent,
            true,
        );
        for g in [NameGroup::StateMachine, NameGroup::Type, NameGroup::Value] {
            self.symbols.put(cur, g, &node.data.name, sym)?;
        }
        let scope = self.symbols.sym(sym).scope.expect("state machine scope");
        if let Some(members) = &node.data.members {
            stack.push(scope);
            qual.push(node.data.name.clone());
            for m in members {
                let mdocs: Vec<String> = m.doc_lines().cloned().collect();
                match &m.node {
                    StateMachineMemberNode::DefAbsType(n) => {
                        self.enter_plain(
                            &n.data.name,
                            &n.loc,
                            Def::AbsType(n),
                            NameGroup::Type,
                            stack,
                            qual,
                            mdocs,
                            Some(sym),
                        )?;
                    }
                    StateMachineMemberNode::DefAliasType(n) => {
                        self.enter_plain(
                            &n.data.name,
                            &n.loc,
                            Def::AliasType(n),
                            NameGroup::Type,
                            stack,
                            qual,
                            mdocs,
                            Some(sym),
                        )?;
                    }
                    StateMachineMemberNode::DefArray(n) => {
                        self.enter_plain(
                            &n.data.name,
                            &n.loc,
                            Def::Array(n),
                            NameGroup::Type,
                            stack,
                            qual,
                            mdocs,
                            Some(sym),
                        )?;
                    }
                    StateMachineMemberNode::DefStruct(n) => {
                        self.enter_plain(
                            &n.data.name,
                            &n.loc,
                            Def::Struct(n),
                            NameGroup::Type,
                            stack,
                            qual,
                            mdocs,
                            Some(sym),
                        )?;
                    }
                    StateMachineMemberNode::DefConstant(n) => {
                        self.enter_plain(
                            &n.data.name,
                            &n.loc,
                            Def::Constant(n),
                            NameGroup::Value,
                            stack,
                            qual,
                            mdocs,
                            Some(sym),
                        )?;
                    }
                    StateMachineMemberNode::DefEnum(n) => {
                        self.enter_enum(n, stack, qual, mdocs, Some(sym))?;
                    }
                    _ => {}
                }
            }
            qual.pop();
            stack.pop();
        }
        Ok(())
    }

    // -- pass 2 helpers -------------------------------------------------------------------

    /// Resolve a formal parameter list in a scope stack.
    pub fn resolve_params(
        &mut self,
        stack: &[ScopeId],
        params: &FormalParamList,
    ) -> Result<Vec<ParamDef>> {
        let mut out: Vec<ParamDef> = Vec::new();
        for p in params {
            let pd = &p.node.data;
            if out.iter().any(|x| x.name == pd.name) {
                return Err(Diagnostic::semantic(
                    p.node.loc.clone(),
                    format!("duplicate parameter {}", pd.name),
                ));
            }
            let ty = self.resolve_type_name(stack, &pd.type_name)?;
            out.push(ParamDef {
                name: pd.name.clone(),
                kind: pd.kind,
                ty,
                docs: p.doc_lines().cloned().collect(),
            });
        }
        Ok(out)
    }

    /// Resolve a port definition on demand.
    pub fn ensure_port(&mut self, sym: SymId, use_loc: &Loc) -> Result<()> {
        if self.ports.contains_key(&sym) {
            return Ok(());
        }
        let Def::Port(node) = self.symbols.sym(sym).def else {
            return Err(Diagnostic::semantic(
                use_loc.clone(),
                format!("{} is not a port", self.symbols.sym(sym).qualified_name()),
            ));
        };
        let stack = self.symbols.sym(sym).def_stack.clone();
        let params = self.resolve_params(&stack, &node.data.params)?;
        let ret = match &node.data.return_type {
            Some(t) => Some(self.resolve_type_name(&stack, t)?),
            None => None,
        };
        self.ports.insert(sym, PortDef { sym, params, ret });
        Ok(())
    }

    /// The symbol of an interface/component/topology use, or an error.
    pub fn resolve_use(
        &self,
        stack: &[ScopeId],
        group: NameGroup,
        qi: &Node<QualIdent>,
    ) -> Result<SymId> {
        self.symbols.resolve(stack, group, &qi.data)
    }

    /// Evaluate an optional expression to an optional integer.
    pub fn opt_int(
        &mut self,
        stack: &[ScopeId],
        e: &Option<Node<Expr>>,
        what: &str,
    ) -> Result<Option<i128>> {
        match e {
            None => Ok(None),
            Some(e) => {
                let v = self.eval(stack, e)?;
                v.as_int().map(Some).ok_or_else(|| {
                    Diagnostic::semantic(e.loc.clone(), format!("{what} must be an integer"))
                })
            }
        }
    }

    /// Evaluate an optional expression to an optional non-negative integer.
    pub fn opt_nonneg(
        &mut self,
        stack: &[ScopeId],
        e: &Option<Node<Expr>>,
        what: &str,
    ) -> Result<Option<u64>> {
        match e {
            None => Ok(None),
            Some(e) => Ok(Some(self.eval_nonneg_int(stack, e, what)?)),
        }
    }
}
