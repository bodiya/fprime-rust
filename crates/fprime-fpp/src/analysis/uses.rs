//! Used-symbol resolution (`UsedSymbols.scala`, deep form): the closure of
//! type and constant definitions that a set of seed symbols depends on,
//! following uses from definitions to definitions. The dictionary back
//! end seeds it with the types and constants named by the commands,
//! events, telemetry channels, parameters, records and containers of a
//! topology's instances, the implied uses of dictionary generation and
//! every `dictionary`-marked definition.

use super::Analysis;
use super::symbols::{Def, NameGroup, ScopeId, SymId};
use crate::ast::*;
use crate::error::Result;
use std::collections::BTreeSet;

/// A used-symbol collector over one scope stack.
pub struct Uses<'u, 'a> {
    a: &'u Analysis<'a>,
    /// Symbols found so far (ordered for deterministic output).
    pub found: BTreeSet<SymId>,
}

impl<'u, 'a> Uses<'u, 'a> {
    /// An empty collector.
    pub fn new(a: &'u Analysis<'a>) -> Self {
        Self {
            a,
            found: BTreeSet::new(),
        }
    }

    /// Record a use. Enum constants are recorded as uses of their enum,
    /// which provides their definition.
    fn add(&mut self, sym: SymId) {
        match self.a.symbols.sym(sym).def {
            Def::EnumConstant(_, e) => self.found.insert(e),
            _ => self.found.insert(sym),
        };
    }

    /// The uses in an expression.
    pub fn expr(&mut self, stack: &[ScopeId], e: &Node<Expr>) -> Result<()> {
        match &e.data {
            Expr::Array(elts) => {
                for x in elts {
                    self.expr(stack, x)?;
                }
            }
            Expr::ArraySubscript(a, i) => {
                self.expr(stack, a)?;
                self.expr(stack, i)?;
            }
            Expr::Binop(l, _, r) => {
                self.expr(stack, l)?;
                self.expr(stack, r)?;
            }
            Expr::Dot(lhs, _) => {
                // Either a qualified constant name or a member selection.
                match self.a.try_resolve_qualified_value(stack, e)? {
                    Some(sym) => self.add(sym),
                    None => self.expr(stack, lhs)?,
                }
            }
            Expr::Ident(name) => {
                if let Some(sym) = self.a.symbols.lookup(stack, NameGroup::Value, name) {
                    self.add(sym);
                }
            }
            Expr::LiteralBool(_)
            | Expr::LiteralFloat(_)
            | Expr::LiteralInt(_)
            | Expr::LiteralString(_) => {}
            Expr::Paren(x) | Expr::Unop(_, x) => self.expr(stack, x)?,
            Expr::SizeOf(t) => self.type_name(stack, t)?,
            Expr::Struct(members) => {
                for m in members {
                    self.expr(stack, &m.data.value)?;
                }
            }
        }
        Ok(())
    }

    /// The uses in an optional expression.
    pub fn opt_expr(&mut self, stack: &[ScopeId], e: &Option<Node<Expr>>) -> Result<()> {
        match e {
            Some(e) => self.expr(stack, e),
            None => Ok(()),
        }
    }

    /// The uses in a type name.
    pub fn type_name(&mut self, stack: &[ScopeId], t: &Node<TypeName>) -> Result<()> {
        match &t.data {
            TypeName::QualIdent(qi) => {
                let sym = self.a.symbols.resolve(stack, NameGroup::Type, &qi.data)?;
                self.add(sym);
            }
            TypeName::String(Some(e)) => self.expr(stack, e)?,
            _ => {}
        }
        Ok(())
    }

    /// The uses in a formal parameter list.
    pub fn params(&mut self, stack: &[ScopeId], params: &FormalParamList) -> Result<()> {
        for p in params {
            self.type_name(stack, &p.node.data.type_name)?;
        }
        Ok(())
    }

    /// The shallow uses of a definition (what its own text names).
    pub fn def(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let stack = s.def_stack.clone();
        match s.def {
            Def::AliasType(n) => self.type_name(&stack, &n.data.type_name)?,
            Def::Array(n) => {
                self.expr(&stack, &n.data.size)?;
                self.type_name(&stack, &n.data.elt_type)?;
                self.opt_expr(&stack, &n.data.default)?;
            }
            Def::Struct(n) => {
                for m in &n.data.members {
                    self.opt_expr(&stack, &m.node.data.size)?;
                    self.type_name(&stack, &m.node.data.type_name)?;
                }
                self.opt_expr(&stack, &n.data.default)?;
            }
            Def::Enum(n) => {
                if let Some(t) = &n.data.type_name {
                    self.type_name(&stack, t)?;
                }
                let mut inner = stack.clone();
                inner.extend(s.scope);
                for c in &n.data.constants {
                    self.opt_expr(&inner, &c.node.data.value)?;
                }
                // A default that names one of the enum's own constants
                // adds nothing; anything else is an ordinary use.
                if let Some(d) = &n.data.default {
                    let own = matches!(
                        self.a
                            .try_resolve_qualified_value(&stack, d)?
                            .or_else(|| match &d.data {
                                Expr::Ident(name) => {
                                    self.a.symbols.lookup(&stack, NameGroup::Value, name)
                                }
                                _ => None,
                            })
                            .map(|c| self.a.symbols.sym(c).def),
                        Some(Def::EnumConstant(..))
                    );
                    if !own {
                        self.expr(&stack, d)?;
                    }
                }
            }
            Def::Constant(n) => self.expr(&stack, &n.data.value)?,
            Def::EnumConstant(n, _) => {
                let mut inner = stack.clone();
                inner.extend(self.a.symbols.sym(s.parent.expect("enum")).scope);
                self.opt_expr(&inner, &n.data.value)?;
            }
            _ => {}
        }
        Ok(())
    }

    /// The shallow uses of the dictionary-relevant specifiers of a
    /// component: commands, events, telemetry channels, parameters,
    /// records and containers.
    pub fn component_specifiers(&mut self, comp: SymId) -> Result<()> {
        let s = self.a.symbols.sym(comp);
        let Def::Component(node) = s.def else {
            return Ok(());
        };
        let mut stack = s.def_stack.clone();
        stack.extend(s.scope);
        for m in &node.data.members {
            match &m.node {
                ComponentMemberNode::SpecCommand(c) => {
                    self.params(&stack, &c.data.params)?;
                    self.opt_expr(&stack, &c.data.opcode)?;
                    self.opt_expr(&stack, &c.data.priority)?;
                }
                ComponentMemberNode::SpecEvent(e) => {
                    self.params(&stack, &e.data.params)?;
                    self.opt_expr(&stack, &e.data.id)?;
                    if let Some(t) = &e.data.throttle {
                        self.expr(&stack, &t.data.count)?;
                        self.opt_expr(&stack, &t.data.every)?;
                    }
                }
                ComponentMemberNode::SpecTlmChannel(t) => {
                    self.type_name(&stack, &t.data.type_name)?;
                    self.opt_expr(&stack, &t.data.id)?;
                    for (_, e) in t.data.low.iter().chain(t.data.high.iter()) {
                        self.expr(&stack, e)?;
                    }
                }
                ComponentMemberNode::SpecParam(p) => {
                    self.type_name(&stack, &p.data.type_name)?;
                    self.opt_expr(&stack, &p.data.default)?;
                    self.opt_expr(&stack, &p.data.id)?;
                    self.opt_expr(&stack, &p.data.set_opcode)?;
                    self.opt_expr(&stack, &p.data.save_opcode)?;
                }
                ComponentMemberNode::SpecRecord(r) => {
                    self.type_name(&stack, &r.data.record_type)?;
                    self.opt_expr(&stack, &r.data.id)?;
                }
                ComponentMemberNode::SpecContainer(c) => {
                    self.opt_expr(&stack, &c.data.id)?;
                    self.opt_expr(&stack, &c.data.default_priority)?;
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Close the found set under definition-to-definition uses.
    pub fn resolve_deep(&mut self) -> Result<()> {
        let mut pending: Vec<SymId> = self.found.iter().copied().collect();
        while let Some(sym) = pending.pop() {
            let before = self.found.clone();
            self.def(sym)?;
            for s in self.found.difference(&before) {
                pending.push(*s);
            }
        }
        Ok(())
    }
}
