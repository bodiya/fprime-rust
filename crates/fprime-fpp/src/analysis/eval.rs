//! Type resolution and constant-expression evaluation.
//!
//! Follows `CheckExprTypes.scala` / `EvalConstantExprs.scala` /
//! `Value.scala`: integer literals have the unbounded `Integer` type, the
//! common type of two numeric operands is the wider one (float wins),
//! shifts are integer-only, `+` also concatenates strings, a dot selects
//! either a constant/enum constant (when the left side names a scope) or a
//! struct member, and conversions to a declared type happen at definitions
//! (array/struct/enum/param defaults, array sizes, ids).

use super::Analysis;
use super::symbols::{Def, NameGroup, ScopeId, SymId};
use super::types::{DEFAULT_STRING_SIZE, Type, Value, int_range, wider_int};
use crate::ast::*;
use crate::error::{Diagnostic, Loc, Result};

/// An enum's representation type and constants (name, value, symbol).
pub type EnumConstants = (IntKind, Vec<(String, i128, SymId)>);

/// A named type's definition, resolved.
#[derive(Debug, Clone)]
pub enum TypeDef {
    /// `type T`
    Abs,
    /// `type T = U`
    Alias(Type),
    /// `enum E : R { .. } default D`
    Enum {
        repr: IntKind,
        /// Constants in declaration order with their values.
        constants: Vec<(String, i128, SymId)>,
        /// The default constant's name.
        default: String,
    },
    /// `array A = [n] T default d`
    Array {
        size: u64,
        elt: Type,
        default: Value,
        format: Option<String>,
    },
    /// `struct S { .. } default d`
    Struct {
        members: Vec<StructMemberDef>,
        default: Value,
    },
}

/// A struct member definition.
#[derive(Debug, Clone)]
pub struct StructMemberDef {
    /// Member name.
    pub name: String,
    /// Member type (an anonymous array type for `x: [n] T`).
    pub ty: Type,
    /// Optional format string.
    pub format: Option<String>,
    /// Annotation lines.
    pub docs: Vec<String>,
}

impl<'a> Analysis<'a> {
    // -- types ----------------------------------------------------------------------

    /// Resolve a type name in a scope stack.
    pub fn resolve_type_name(&mut self, stack: &[ScopeId], tn: &Node<TypeName>) -> Result<Type> {
        Ok(match &tn.data {
            TypeName::Float(k) => Type::Float(*k),
            TypeName::Int(k) => Type::Int(*k),
            TypeName::Bool => Type::Bool,
            TypeName::String(None) => Type::String(None),
            TypeName::String(Some(e)) => {
                let n = self.eval_nonneg_int(stack, e, "string size")?;
                Type::String(Some(n))
            }
            TypeName::QualIdent(qi) => {
                let sym = self.symbols.resolve(stack, NameGroup::Type, &qi.data)?;
                self.type_of_sym(sym, &qi.loc)?
            }
        })
    }

    /// The type denoted by a type symbol, resolving its definition on
    /// first use (with cycle detection).
    pub fn type_of_sym(&mut self, sym: SymId, use_loc: &Loc) -> Result<Type> {
        let def = self.symbols.sym(sym).def;
        match def {
            Def::AbsType(_) => Ok(Type::Abs(sym)),
            Def::AliasType(_) => {
                self.ensure_type_def(sym, use_loc)?;
                Ok(Type::Alias(sym))
            }
            Def::Enum(_) => {
                self.ensure_type_def(sym, use_loc)?;
                Ok(Type::Enum(sym))
            }
            Def::Array(_) => {
                self.ensure_type_def(sym, use_loc)?;
                Ok(Type::Array(sym))
            }
            Def::Struct(_) => {
                self.ensure_type_def(sym, use_loc)?;
                Ok(Type::Struct(sym))
            }
            other => Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "{} {} is not a type",
                    other.kind_name(),
                    self.symbols.sym(sym).qualified_name()
                ),
            )),
        }
    }

    /// Make sure the definition of a named type has been resolved into the
    /// type table.
    pub fn ensure_type_def(&mut self, sym: SymId, use_loc: &Loc) -> Result<()> {
        if self.type_defs.contains_key(&sym) {
            return Ok(());
        }
        if !self.in_progress.insert(sym) {
            return Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "cyclic definition of type {}",
                    self.symbols.sym(sym).qualified_name()
                ),
            ));
        }
        let stack = self.symbols.sym(sym).def_stack.clone();
        let def = self.symbols.sym(sym).def;
        let result = match def {
            Def::AliasType(node) => {
                let t = self.resolve_type_name(&stack, &node.data.type_name)?;
                Ok(TypeDef::Alias(t))
            }
            Def::Enum(node) => self.resolve_enum_def(sym, &stack, node),
            Def::Array(node) => self.resolve_array_def(sym, &stack, node),
            Def::Struct(node) => self.resolve_struct_def(sym, &stack, node),
            _ => unreachable!("not a named type"),
        };
        self.in_progress.remove(&sym);
        let td = result?;
        self.type_defs.insert(sym, td);
        Ok(())
    }

    fn resolve_enum_def(
        &mut self,
        sym: SymId,
        stack: &[ScopeId],
        node: &'a Node<DefEnum>,
    ) -> Result<TypeDef> {
        self.ensure_enum_constants(sym, &node.loc)?;
        let (repr, constants) = self.enum_constants[&sym].clone();
        // The default is evaluated after the constants (it may name one of
        // them, directly or through a constant defined elsewhere).
        let enum_scope = self.symbols.sym(sym).scope.expect("enum owns a scope");
        let mut inner = stack.to_vec();
        inner.push(enum_scope);
        let default = match &node.data.default {
            None => constants[0].0.clone(),
            Some(e) => {
                let v = self.eval(&inner, e)?;
                match v {
                    Value::Enum(s, name, _) if s == sym => name,
                    other => {
                        return Err(Diagnostic::semantic(
                            e.loc.clone(),
                            format!("enum default must be a constant of the enum, not {other}"),
                        ));
                    }
                }
            }
        };
        Ok(TypeDef::Enum {
            repr,
            constants,
            default,
        })
    }

    /// Resolve an enum's representation type and constant values (without
    /// its default, so that a constant elsewhere may be defined as one of
    /// the enum's constants and still serve as the enum's default).
    fn ensure_enum_constants(&mut self, sym: SymId, use_loc: &Loc) -> Result<()> {
        if self.enum_constants.contains_key(&sym) {
            return Ok(());
        }
        let Def::Enum(node) = self.symbols.sym(sym).def else {
            unreachable!("not an enum")
        };
        if !self.enum_in_progress.insert(sym) {
            return Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "cyclic definition of enum {}",
                    self.symbols.sym(sym).qualified_name()
                ),
            ));
        }
        let stack = self.symbols.sym(sym).def_stack.clone();
        let result = self.eval_enum_constants(sym, &stack, node);
        self.enum_in_progress.remove(&sym);
        let (repr, constants) = result?;
        self.enum_constants.insert(sym, (repr, constants));
        Ok(())
    }

    fn eval_enum_constants(
        &mut self,
        sym: SymId,
        stack: &[ScopeId],
        node: &'a Node<DefEnum>,
    ) -> Result<EnumConstants> {
        let repr = match &node.data.type_name {
            None => IntKind::I32,
            Some(tn) => {
                let t = self.resolve_type_name(stack, tn)?;
                match self.underlying(&t) {
                    Type::Int(k) => k,
                    other => {
                        return Err(Diagnostic::semantic(
                            tn.loc.clone(),
                            format!(
                                "enum representation type must be an integer type, not {other}"
                            ),
                        ));
                    }
                }
            }
        };
        // Constants are evaluated in the enum's own scope so that a
        // constant may refer to an earlier one unqualified.
        let enum_scope = self.symbols.sym(sym).scope.expect("enum owns a scope");
        let mut inner = stack.to_vec();
        inner.push(enum_scope);
        let mut constants = Vec::new();
        let mut next: i128 = 0;
        for c in &node.data.constants {
            let cn = &c.node;
            let value = match &cn.data.value {
                Some(e) => {
                    let v = self.eval(&inner, e)?;
                    v.as_int().ok_or_else(|| {
                        Diagnostic::semantic(
                            e.loc.clone(),
                            "enum constant value must be an integer",
                        )
                    })?
                }
                None => next,
            };
            let (lo, hi) = int_range(repr);
            if value < lo || value > hi {
                return Err(Diagnostic::semantic(
                    cn.loc.clone(),
                    format!(
                        "enum constant value {value} is out of range for {}",
                        repr.fpp_name()
                    ),
                ));
            }
            if let Some((prev, _, _)) = constants.iter().find(|(_, v, _)| *v == value) {
                return Err(Diagnostic::semantic(
                    cn.loc.clone(),
                    format!("duplicate enum value {value} (also used by {prev})"),
                ));
            }
            let csym = self.symbols.scopes[enum_scope]
                .get(NameGroup::Value, &cn.data.name)
                .expect("enum constant entered");
            self.values
                .insert(csym, Value::Enum(sym, cn.data.name.clone(), value));
            constants.push((cn.data.name.clone(), value, csym));
            next = value + 1;
        }
        if constants.is_empty() {
            return Err(Diagnostic::semantic(
                node.loc.clone(),
                "enum has no constants",
            ));
        }
        Ok((repr, constants))
    }

    fn resolve_array_def(
        &mut self,
        sym: SymId,
        stack: &[ScopeId],
        node: &'a Node<DefArray>,
    ) -> Result<TypeDef> {
        let size = self.eval_nonneg_int(stack, &node.data.size, "array size")?;
        if size == 0 {
            return Err(Diagnostic::semantic(
                node.data.size.loc.clone(),
                "array size must be positive",
            ));
        }
        let elt = self.resolve_type_name(stack, &node.data.elt_type)?;
        let anon = Type::AnonArray(Some(size), Box::new(elt.clone()));
        let default = match &node.data.default {
            Some(e) => {
                let v = self.eval(stack, e)?;
                self.convert(v, &anon, &e.loc)?
            }
            None => self.default_value(&anon, &node.loc)?,
        };
        let default = match default {
            Value::Array(_, elts) => Value::Array(Some(sym), elts),
            other => other,
        };
        let format = node.data.format.as_ref().map(|f| f.data.clone());
        Ok(TypeDef::Array {
            size,
            elt,
            default,
            format,
        })
    }

    fn resolve_struct_def(
        &mut self,
        sym: SymId,
        stack: &[ScopeId],
        node: &'a Node<DefStruct>,
    ) -> Result<TypeDef> {
        let mut members: Vec<StructMemberDef> = Vec::new();
        for m in &node.data.members {
            let md = &m.node.data;
            if members.iter().any(|x| x.name == md.name) {
                return Err(Diagnostic::semantic(
                    m.node.loc.clone(),
                    format!("duplicate struct member {}", md.name),
                ));
            }
            let mut ty = self.resolve_type_name(stack, &md.type_name)?;
            if let Some(size) = &md.size {
                let n = self.eval_nonneg_int(stack, size, "array size")?;
                ty = Type::AnonArray(Some(n), Box::new(ty));
            }
            members.push(StructMemberDef {
                name: md.name.clone(),
                ty,
                format: md.format.as_ref().map(|f| f.data.clone()),
                docs: m.doc_lines().cloned().collect(),
            });
        }
        let anon = Type::AnonStruct(
            members
                .iter()
                .map(|m| (m.name.clone(), m.ty.clone()))
                .collect(),
        );
        let default = match &node.data.default {
            Some(e) => {
                let v = self.eval(stack, e)?;
                self.convert(v, &anon, &e.loc)?
            }
            None => self.default_value(&anon, &node.loc)?,
        };
        let default = match default {
            Value::Struct(_, ms) => Value::Struct(Some(sym), ms),
            other => other,
        };
        Ok(TypeDef::Struct { members, default })
    }

    /// Strip aliases.
    pub fn underlying(&self, t: &Type) -> Type {
        let mut t = t.clone();
        loop {
            match t {
                Type::Alias(s) => match self.type_defs.get(&s) {
                    Some(TypeDef::Alias(inner)) => t = inner.clone(),
                    _ => return Type::Alias(s),
                },
                other => return other,
            }
        }
    }

    /// The structural form of a type: named arrays/structs expanded one
    /// level to their anonymous form.
    fn structural(&self, t: &Type) -> Type {
        match self.underlying(t) {
            Type::Array(s) => match self.type_defs.get(&s) {
                Some(TypeDef::Array { size, elt, .. }) => {
                    Type::AnonArray(Some(*size), Box::new(elt.clone()))
                }
                _ => Type::Array(s),
            },
            Type::Struct(s) => match self.type_defs.get(&s) {
                Some(TypeDef::Struct { members, .. }) => Type::AnonStruct(
                    members
                        .iter()
                        .map(|m| (m.name.clone(), m.ty.clone()))
                        .collect(),
                ),
                _ => Type::Struct(s),
            },
            other => other,
        }
    }

    /// The default value of a type.
    pub fn default_value(&mut self, t: &Type, loc: &Loc) -> Result<Value> {
        Ok(match self.underlying(t) {
            Type::Int(k) => Value::Int(0, Some(k)),
            Type::Integer => Value::Int(0, None),
            Type::Float(k) => Value::Float(0.0, k),
            Type::Bool => Value::Bool(false),
            Type::String(_) => Value::Str(String::new()),
            // Reference: an abstract type has an opaque default value.
            Type::Abs(s) => Value::Abs(s),
            Type::Alias(_) => unreachable!("underlying strips aliases"),
            Type::Enum(s) => {
                self.ensure_type_def(s, loc)?;
                let Some(TypeDef::Enum {
                    constants, default, ..
                }) = self.type_defs.get(&s)
                else {
                    unreachable!()
                };
                let (name, v, _) = constants
                    .iter()
                    .find(|c| c.0 == *default)
                    .expect("default exists");
                Value::Enum(s, name.clone(), *v)
            }
            Type::Array(s) => {
                self.ensure_type_def(s, loc)?;
                let Some(TypeDef::Array { default, .. }) = self.type_defs.get(&s) else {
                    unreachable!()
                };
                match default.clone() {
                    Value::Array(_, elts) => Value::Array(Some(s), elts),
                    other => other,
                }
            }
            Type::Struct(s) => {
                self.ensure_type_def(s, loc)?;
                let Some(TypeDef::Struct { default, .. }) = self.type_defs.get(&s) else {
                    unreachable!()
                };
                match default.clone() {
                    Value::Struct(_, ms) => Value::Struct(Some(s), ms),
                    other => other,
                }
            }
            Type::AnonArray(Some(n), elt) => {
                let e = self.default_value(&elt, loc)?;
                Value::Array(None, vec![e; n as usize])
            }
            Type::AnonArray(None, _) => Value::Array(None, Vec::new()),
            Type::AnonStruct(ms) => {
                let mut out = Vec::new();
                for (n, t) in ms {
                    out.push((n, self.default_value(&t, loc)?));
                }
                Value::Struct(None, out)
            }
        })
    }

    /// Serialized size of a type in bytes (`sizeof`).
    pub fn serialized_size(&mut self, t: &Type, loc: &Loc) -> Result<u64> {
        Ok(match self.underlying(t) {
            Type::Int(k) => u64::from(k.bits() / 8),
            Type::Integer => 8,
            Type::Float(FloatKind::F32) => 4,
            Type::Float(FloatKind::F64) => 8,
            Type::Bool => 1,
            Type::String(n) => 2 + n.unwrap_or(DEFAULT_STRING_SIZE),
            Type::Abs(s) => {
                return Err(Diagnostic::semantic(
                    loc.clone(),
                    format!(
                        "cannot compute the size of abstract type {}",
                        self.symbols.sym(s).qualified_name()
                    ),
                ));
            }
            Type::Alias(_) => unreachable!(),
            Type::Enum(s) => {
                self.ensure_type_def(s, loc)?;
                let Some(TypeDef::Enum { repr, .. }) = self.type_defs.get(&s) else {
                    unreachable!()
                };
                u64::from(repr.bits() / 8)
            }
            Type::Array(s) => {
                self.ensure_type_def(s, loc)?;
                let Some(TypeDef::Array { size, elt, .. }) = self.type_defs.get(&s) else {
                    unreachable!()
                };
                let (size, elt) = (*size, elt.clone());
                size * self.serialized_size(&elt, loc)?
            }
            Type::Struct(s) => {
                self.ensure_type_def(s, loc)?;
                let Some(TypeDef::Struct { members, .. }) = self.type_defs.get(&s) else {
                    unreachable!()
                };
                let tys: Vec<Type> = members.iter().map(|m| m.ty.clone()).collect();
                let mut total = 0;
                for t in tys {
                    total += self.serialized_size(&t, loc)?;
                }
                total
            }
            Type::AnonArray(n, elt) => n.unwrap_or(0) * self.serialized_size(&elt, loc)?,
            Type::AnonStruct(ms) => {
                let mut total = 0;
                for (_, t) in ms {
                    total += self.serialized_size(&t, loc)?;
                }
                total
            }
        })
    }

    // -- conversion ---------------------------------------------------------------------

    /// Convert a value to a declared type (`Value.convertToType`), filling
    /// arrays/structs from scalars and struct defaults for missing members.
    pub fn convert(&mut self, v: Value, target: &Type, loc: &Loc) -> Result<Value> {
        let under = self.underlying(target);
        let vt = v.ty();
        // Identical (or same-definition) types need no work.
        if self.same_type(&vt, &under) {
            return Ok(v);
        }
        let fail = |this: &Self, v: &Value| {
            Diagnostic::semantic(
                loc.clone(),
                format!(
                    "cannot convert {} of type {} to type {}",
                    v,
                    this.describe_type(&v.ty()),
                    this.describe_type(target)
                ),
            )
        };
        match (&v, &under) {
            (Value::Int(i, _), Type::Int(k)) => {
                let (lo, hi) = int_range(*k);
                if *i < lo || *i > hi {
                    return Err(Diagnostic::semantic(
                        loc.clone(),
                        format!("value {i} is out of range for {}", k.fpp_name()),
                    ));
                }
                Ok(Value::Int(*i, Some(*k)))
            }
            (Value::Int(i, _), Type::Integer) => Ok(Value::Int(*i, None)),
            (Value::Int(i, _), Type::Float(k)) => Ok(Value::Float(*i as f64, *k)),
            (Value::Float(f, _), Type::Float(k)) => Ok(Value::Float(*f, *k)),
            (Value::Float(f, _), Type::Int(k)) => {
                let i = f.trunc() as i128;
                let (lo, hi) = int_range(*k);
                if i < lo || i > hi {
                    return Err(Diagnostic::semantic(
                        loc.clone(),
                        format!("value {f} is out of range for {}", k.fpp_name()),
                    ));
                }
                Ok(Value::Int(i, Some(*k)))
            }
            (Value::Float(f, _), Type::Integer) => Ok(Value::Int(f.trunc() as i128, None)),
            (Value::Enum(_, _, i), Type::Int(k)) => {
                let (lo, hi) = int_range(*k);
                if *i < lo || *i > hi {
                    return Err(fail(self, &v));
                }
                Ok(Value::Int(*i, Some(*k)))
            }
            (Value::Enum(_, _, i), Type::Integer) => Ok(Value::Int(*i, None)),
            (Value::Enum(_, _, i), Type::Float(k)) => Ok(Value::Float(*i as f64, *k)),
            (Value::Str(s), Type::String(_)) => Ok(Value::Str(s.clone())),
            (Value::Bool(b), Type::Bool) => Ok(Value::Bool(*b)),
            (Value::Enum(s, n, i), Type::Enum(t)) if s == t => Ok(Value::Enum(*s, n.clone(), *i)),
            // Arrays.
            (Value::Array(_, elts), Type::Array(s)) => {
                let Some(TypeDef::Array { size, elt, .. }) = self.type_defs.get(s) else {
                    unreachable!()
                };
                let (size, elt) = (*size, elt.clone());
                if elts.len() as u64 != size {
                    return Err(Diagnostic::semantic(
                        loc.clone(),
                        format!("array has {} elements, expected {size}", elts.len()),
                    ));
                }
                let mut out = Vec::new();
                for e in elts.clone() {
                    out.push(self.convert(e, &elt, loc)?);
                }
                Ok(Value::Array(Some(*s), out))
            }
            (Value::Array(_, elts), Type::AnonArray(size, elt)) => {
                if let Some(size) = size {
                    if elts.len() as u64 != *size {
                        return Err(Diagnostic::semantic(
                            loc.clone(),
                            format!("array has {} elements, expected {size}", elts.len()),
                        ));
                    }
                }
                let elt = (**elt).clone();
                let mut out = Vec::new();
                for e in elts.clone() {
                    out.push(self.convert(e, &elt, loc)?);
                }
                Ok(Value::Array(None, out))
            }
            (scalar, Type::Array(s)) if scalar.ty().is_promotable() => {
                let Some(TypeDef::Array { size, elt, .. }) = self.type_defs.get(s) else {
                    unreachable!()
                };
                let (size, elt) = (*size, elt.clone());
                let e = self.convert(scalar.clone(), &elt, loc)?;
                Ok(Value::Array(Some(*s), vec![e; size as usize]))
            }
            (scalar, Type::AnonArray(Some(size), elt)) if scalar.ty().is_promotable() => {
                let elt = (**elt).clone();
                let e = self.convert(scalar.clone(), &elt, loc)?;
                Ok(Value::Array(None, vec![e; *size as usize]))
            }
            // Structs.
            (Value::Struct(_, given), Type::Struct(s)) => {
                let Some(TypeDef::Struct { members, .. }) = self.type_defs.get(s) else {
                    unreachable!()
                };
                let members: Vec<(String, Type)> = members
                    .iter()
                    .map(|m| (m.name.clone(), m.ty.clone()))
                    .collect();
                let out = self.convert_struct_members(given, &members, loc)?;
                Ok(Value::Struct(Some(*s), out))
            }
            (Value::Struct(_, given), Type::AnonStruct(members)) => {
                let members = members.clone();
                let out = self.convert_struct_members(given, &members, loc)?;
                Ok(Value::Struct(None, out))
            }
            (scalar, Type::Struct(s)) if scalar.ty().is_promotable() => {
                let Some(TypeDef::Struct { members, .. }) = self.type_defs.get(s) else {
                    unreachable!()
                };
                let members: Vec<(String, Type)> = members
                    .iter()
                    .map(|m| (m.name.clone(), m.ty.clone()))
                    .collect();
                let mut out = Vec::new();
                for (n, t) in members {
                    out.push((n, self.convert(scalar.clone(), &t, loc)?));
                }
                Ok(Value::Struct(Some(*s), out))
            }
            (scalar, Type::AnonStruct(members)) if scalar.ty().is_promotable() => {
                let members = members.clone();
                let mut out = Vec::new();
                for (n, t) in members {
                    out.push((n, self.convert(scalar.clone(), &t, loc)?));
                }
                Ok(Value::Struct(None, out))
            }
            _ => Err(fail(self, &v)),
        }
    }

    fn convert_struct_members(
        &mut self,
        given: &[(String, Value)],
        members: &[(String, Type)],
        loc: &Loc,
    ) -> Result<Vec<(String, Value)>> {
        for (n, _) in given {
            if !members.iter().any(|(m, _)| m == n) {
                return Err(Diagnostic::semantic(
                    loc.clone(),
                    format!("struct has no member named {n}"),
                ));
            }
        }
        let mut out = Vec::new();
        for (n, t) in members {
            let v = match given.iter().find(|(g, _)| g == n) {
                Some((_, v)) => self.convert(v.clone(), t, loc)?,
                None => self.default_value(t, loc)?,
            };
            out.push((n.clone(), v));
        }
        Ok(out)
    }

    /// Structural type identity after alias stripping.
    fn same_type(&self, a: &Type, b: &Type) -> bool {
        let (a, b) = (self.underlying(a), self.underlying(b));
        match (&a, &b) {
            (Type::Int(x), Type::Int(y)) => x == y,
            (Type::Float(x), Type::Float(y)) => x == y,
            (Type::Bool, Type::Bool) | (Type::Integer, Type::Integer) => true,
            (Type::String(_), Type::String(_)) => true,
            (Type::Abs(x), Type::Abs(y))
            | (Type::Enum(x), Type::Enum(y))
            | (Type::Array(x), Type::Array(y))
            | (Type::Struct(x), Type::Struct(y)) => x == y,
            _ => false,
        }
    }

    /// A readable rendering of a type (named types by qualified name).
    pub fn describe_type(&self, t: &Type) -> String {
        match t {
            Type::Abs(s) | Type::Alias(s) | Type::Enum(s) | Type::Array(s) | Type::Struct(s) => {
                self.symbols.sym(*s).qualified_name()
            }
            Type::AnonArray(Some(n), e) => format!("[{n}] {}", self.describe_type(e)),
            Type::AnonArray(None, e) => format!("[] {}", self.describe_type(e)),
            Type::AnonStruct(ms) => {
                let parts: Vec<String> = ms
                    .iter()
                    .map(|(n, t)| format!("{n}: {}", self.describe_type(t)))
                    .collect();
                format!("{{ {} }}", parts.join(", "))
            }
            other => other.to_string(),
        }
    }

    // -- evaluation ----------------------------------------------------------------------

    /// Evaluate an expression to a value in a scope stack.
    pub fn eval(&mut self, stack: &[ScopeId], e: &Node<Expr>) -> Result<Value> {
        let v = match &e.data {
            Expr::LiteralBool(b) => Value::Bool(*b),
            Expr::LiteralInt(s) => Value::Int(parse_int(s, &e.loc)?, None),
            Expr::LiteralFloat(s) => Value::Float(
                s.parse::<f64>().map_err(|_| {
                    Diagnostic::semantic(e.loc.clone(), format!("invalid float literal {s}"))
                })?,
                FloatKind::F64,
            ),
            Expr::LiteralString(s) => Value::Str(s.clone()),
            Expr::Paren(inner) => self.eval(stack, inner)?,
            Expr::Unop(Unop::Minus, inner) => match self.eval(stack, inner)? {
                Value::Int(i, k) => Value::Int(-i, k),
                Value::Float(f, k) => Value::Float(-f, k),
                Value::Enum(_, _, i) => Value::Int(-i, None),
                other => {
                    return Err(Diagnostic::semantic(
                        e.loc.clone(),
                        format!("cannot negate {} of non-numeric type", other),
                    ));
                }
            },
            Expr::Binop(l, op, r) => {
                let lv = self.eval(stack, l)?;
                let rv = self.eval(stack, r)?;
                self.binop(lv, *op, rv, &e.loc)?
            }
            Expr::Array(elts) => {
                let mut vs = Vec::new();
                for x in elts {
                    vs.push(self.eval(stack, x)?);
                }
                if vs.is_empty() {
                    return Err(Diagnostic::semantic(
                        e.loc.clone(),
                        "array literal may not be empty",
                    ));
                }
                // Elements take the common type of all elements.
                let mut common = vs[0].ty();
                for v in &vs[1..] {
                    common = self.common_type(&common, &v.ty(), &e.loc)?;
                }
                let mut out = Vec::new();
                for v in vs {
                    out.push(self.convert(v, &common, &e.loc)?);
                }
                Value::Array(None, out)
            }
            Expr::Struct(members) => {
                let mut out: Vec<(String, Value)> = Vec::new();
                for m in members {
                    if out.iter().any(|(n, _)| *n == m.data.name) {
                        return Err(Diagnostic::semantic(
                            m.loc.clone(),
                            format!("duplicate struct member {}", m.data.name),
                        ));
                    }
                    let v = self.eval(stack, &m.data.value)?;
                    out.push((m.data.name.clone(), v));
                }
                Value::Struct(None, out)
            }
            Expr::SizeOf(tn) => {
                let t = self.resolve_type_name(stack, tn)?;
                Value::Int(self.serialized_size(&t, &tn.loc)? as i128, None)
            }
            Expr::ArraySubscript(arr, idx) => {
                let av = self.eval(stack, arr)?;
                let iv = self.eval(stack, idx)?;
                let Value::Array(_, elts) = av else {
                    return Err(Diagnostic::semantic(
                        arr.loc.clone(),
                        "subscripted value is not an array",
                    ));
                };
                let i = iv.as_int().ok_or_else(|| {
                    Diagnostic::semantic(idx.loc.clone(), "array index must be an integer")
                })?;
                if i < 0 || i as usize >= elts.len() {
                    return Err(Diagnostic::semantic(
                        idx.loc.clone(),
                        format!(
                            "index {i} is not in the range [0, {}]",
                            elts.len() as i128 - 1
                        ),
                    ));
                }
                elts[i as usize].clone()
            }
            Expr::Ident(name) => {
                let sym = self
                    .symbols
                    .lookup(stack, NameGroup::Value, name)
                    .ok_or_else(|| {
                        Diagnostic::semantic(
                            e.loc.clone(),
                            format!("undefined constant symbol {name}"),
                        )
                    })?;
                self.value_of_sym(sym, &e.loc)?
            }
            Expr::Dot(lhs, id) => {
                // Either a qualified constant name or a member selection.
                if let Some(sym) = self.try_resolve_qualified_value(stack, e)? {
                    self.value_of_sym(sym, &e.loc)?
                } else {
                    let lv = self.eval(stack, lhs)?;
                    match lv {
                        Value::Struct(_, ms) => ms
                            .iter()
                            .find(|(n, _)| *n == id.data)
                            .map(|(_, v)| v.clone())
                            .ok_or_else(|| {
                                Diagnostic::semantic(
                                    id.loc.clone(),
                                    format!("struct has no member named {}", id.data),
                                )
                            })?,
                        other => {
                            return Err(Diagnostic::semantic(
                                id.loc.clone(),
                                format!("cannot select member {} from {}", id.data, other),
                            ));
                        }
                    }
                }
            }
        };
        Ok(v)
    }

    /// If `e` is a chain of dots over identifiers that names a constant
    /// or enum constant through scopes (`M.E.X`), resolve it.
    pub(crate) fn try_resolve_qualified_value(
        &self,
        stack: &[ScopeId],
        e: &Node<Expr>,
    ) -> Result<Option<SymId>> {
        let mut parts: Vec<&Node<Ident>> = Vec::new();
        let mut cur = e;
        let first: &str = loop {
            match &cur.data {
                Expr::Dot(lhs, id) => {
                    parts.push(id);
                    cur = lhs;
                }
                Expr::Ident(name) => break name,
                _ => return Ok(None),
            }
        };
        parts.reverse();
        let Some(mut sym) = self.symbols.lookup(stack, NameGroup::Value, first) else {
            return Ok(None);
        };
        for part in parts {
            let owner = self.symbols.sym(sym);
            match owner.def {
                Def::Constant(_) | Def::EnumConstant(..) => return Ok(None),
                _ => {}
            }
            let Some(scope) = owner.scope else {
                return Ok(None);
            };
            match self.symbols.scopes[scope].get(NameGroup::Value, &part.data) {
                Some(s) => sym = s,
                None => {
                    return Err(Diagnostic::semantic(
                        part.loc.clone(),
                        format!(
                            "undefined constant symbol {} in {}",
                            part.data,
                            owner.qualified_name()
                        ),
                    ));
                }
            }
        }
        match self.symbols.sym(sym).def {
            Def::Constant(_) | Def::EnumConstant(..) => Ok(Some(sym)),
            _ => Ok(None),
        }
    }

    /// The value of a constant or enum-constant symbol, evaluated on first
    /// use in its own scope, with cycle detection.
    pub fn value_of_sym(&mut self, sym: SymId, use_loc: &Loc) -> Result<Value> {
        if let Some(v) = self.values.get(&sym) {
            return Ok(v.clone());
        }
        let def = self.symbols.sym(sym).def;
        match def {
            Def::Constant(node) => {
                if !self.in_progress.insert(sym) {
                    return Err(Diagnostic::semantic(
                        use_loc.clone(),
                        format!(
                            "cyclic definition of constant {}",
                            self.symbols.sym(sym).qualified_name()
                        ),
                    ));
                }
                let stack = self.symbols.sym(sym).def_stack.clone();
                let result = self.eval(&stack, &node.data.value);
                self.in_progress.remove(&sym);
                let v = result?;
                self.values.insert(sym, v.clone());
                Ok(v)
            }
            Def::EnumConstant(_, enum_sym) => {
                self.ensure_enum_constants(enum_sym, use_loc)?;
                self.values.get(&sym).cloned().ok_or_else(|| {
                    Diagnostic::semantic(use_loc.clone(), "enum constant has no value")
                })
            }
            other => Err(Diagnostic::semantic(
                use_loc.clone(),
                format!(
                    "{} {} is not a constant",
                    other.kind_name(),
                    self.symbols.sym(sym).qualified_name()
                ),
            )),
        }
    }

    /// Common type of two operand types (`Type.commonType`).
    pub fn common_type(&self, a: &Type, b: &Type, loc: &Loc) -> Result<Type> {
        let (ua, ub) = (self.underlying(a), self.underlying(b));
        if self.same_type(&ua, &ub) {
            return Ok(a.clone());
        }
        let numeric = match (&ua, &ub) {
            (Type::Float(x), Type::Float(y)) => Some(Type::Float(
                if *x == FloatKind::F64 || *y == FloatKind::F64 {
                    FloatKind::F64
                } else {
                    FloatKind::F32
                },
            )),
            (Type::Float(x), Type::Int(_) | Type::Integer | Type::Enum(_))
            | (Type::Int(_) | Type::Integer | Type::Enum(_), Type::Float(x)) => {
                Some(Type::Float(*x))
            }
            (Type::Integer, Type::Int(k)) | (Type::Int(k), Type::Integer) => Some(Type::Int(*k)),
            (Type::Int(x), Type::Int(y)) => Some(Type::Int(wider_int(*x, *y))),
            (Type::Enum(_), Type::Int(k)) | (Type::Int(k), Type::Enum(_)) => Some(Type::Int(*k)),
            (Type::Enum(_), Type::Integer) | (Type::Integer, Type::Enum(_)) => Some(Type::Integer),
            (Type::String(_), Type::String(_)) => Some(Type::String(None)),
            _ => None,
        };
        if let Some(t) = numeric {
            return Ok(t);
        }
        // Arrays: elementwise common type. Structs: member-wise common
        // type over the union of the member names (reference rule for
        // anonymous struct literals in one array).
        match (self.structural(&ua), self.structural(&ub)) {
            (Type::AnonArray(na, ea), Type::AnonArray(nb, eb)) if na == nb => {
                let e = self.common_type(&ea, &eb, loc)?;
                return Ok(Type::AnonArray(na, Box::new(e)));
            }
            (Type::AnonStruct(ma), Type::AnonStruct(mb)) => {
                let mut out: Vec<(String, Type)> = Vec::new();
                for (n, t) in &ma {
                    let t = match mb.iter().find(|(m, _)| m == n) {
                        Some((_, u)) => self.common_type(t, u, loc)?,
                        None => t.clone(),
                    };
                    out.push((n.clone(), t));
                }
                for (n, t) in &mb {
                    if !ma.iter().any(|(m, _)| m == n) {
                        out.push((n.clone(), t.clone()));
                    }
                }
                return Ok(Type::AnonStruct(out));
            }
            _ => {}
        }
        Err(Diagnostic::semantic(
            loc.clone(),
            format!(
                "cannot compute common type of {} and {}",
                self.describe_type(a),
                self.describe_type(b)
            ),
        ))
    }

    fn binop(&mut self, l: Value, op: Binop, r: Value, loc: &Loc) -> Result<Value> {
        // String concatenation.
        if let (Value::Str(a), Binop::Add, Value::Str(b)) = (&l, op, &r) {
            return Ok(Value::Str(format!("{a}{b}")));
        }
        match op {
            Binop::LShift | Binop::RShift => {
                let (a, b) = (l.as_int(), r.as_int());
                let (Some(a), Some(b)) = (a, b) else {
                    return Err(Diagnostic::semantic(
                        loc.clone(),
                        "shift operands must be integers",
                    ));
                };
                if !(0..128).contains(&b) {
                    return Err(Diagnostic::semantic(
                        loc.clone(),
                        format!("invalid shift amount {b}"),
                    ));
                }
                let v = if op == Binop::LShift { a << b } else { a >> b };
                Ok(Value::Int(v, None))
            }
            _ => {
                let t = self.common_type(&l.ty(), &r.ty(), loc)?;
                if !t.is_numeric() {
                    return Err(Diagnostic::semantic(
                        loc.clone(),
                        format!("operator {op} requires numeric operands"),
                    ));
                }
                let l = self.convert(l, &t, loc)?;
                let r = self.convert(r, &t, loc)?;
                match (l, r) {
                    (Value::Int(a, k), Value::Int(b, _)) => {
                        let v = match op {
                            Binop::Add => a.checked_add(b),
                            Binop::Sub => a.checked_sub(b),
                            Binop::Mul => a.checked_mul(b),
                            Binop::Div => {
                                if b == 0 {
                                    return Err(Diagnostic::semantic(
                                        loc.clone(),
                                        "division by zero",
                                    ));
                                }
                                a.checked_div(b)
                            }
                            _ => unreachable!(),
                        }
                        .ok_or_else(|| Diagnostic::semantic(loc.clone(), "integer overflow"))?;
                        if let Some(k) = k {
                            let (lo, hi) = int_range(k);
                            if v < lo || v > hi {
                                return Err(Diagnostic::semantic(
                                    loc.clone(),
                                    format!("value {v} is out of range for {}", k.fpp_name()),
                                ));
                            }
                        }
                        Ok(Value::Int(v, k))
                    }
                    (Value::Float(a, k), Value::Float(b, _)) => {
                        let v = match op {
                            Binop::Add => a + b,
                            Binop::Sub => a - b,
                            Binop::Mul => a * b,
                            Binop::Div => {
                                if b == 0.0 {
                                    return Err(Diagnostic::semantic(
                                        loc.clone(),
                                        "division by zero",
                                    ));
                                }
                                a / b
                            }
                            _ => unreachable!(),
                        };
                        Ok(Value::Float(v, k))
                    }
                    _ => unreachable!("converted to a common numeric type"),
                }
            }
        }
    }

    /// Evaluate an expression that must be a non-negative integer.
    pub fn eval_nonneg_int(
        &mut self,
        stack: &[ScopeId],
        e: &Node<Expr>,
        what: &str,
    ) -> Result<u64> {
        let v = self.eval(stack, e)?;
        let i = v.as_int().ok_or_else(|| {
            Diagnostic::semantic(e.loc.clone(), format!("{what} must be an integer"))
        })?;
        if i < 0 {
            return Err(Diagnostic::semantic(
                e.loc.clone(),
                format!("{what} may not be negative"),
            ));
        }
        u64::try_from(i)
            .map_err(|_| Diagnostic::semantic(e.loc.clone(), format!("{what} is too large")))
    }
}

/// Parse an FPP integer literal (decimal or `0x` hex).
fn parse_int(s: &str, loc: &Loc) -> Result<i128> {
    let r = if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        i128::from_str_radix(hex, 16)
    } else {
        s.parse::<i128>()
    };
    r.map_err(|_| Diagnostic::semantic(loc.clone(), format!("invalid integer literal {s}")))
}
