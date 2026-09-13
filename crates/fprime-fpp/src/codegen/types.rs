//! Emission of constants, enums, structs, arrays and type aliases onto the
//! `fprime_fw` macro layer.

use super::{Generator, names};
use crate::analysis::{Def, SymId, Type, TypeDef, Value};
use crate::ast::{FloatKind, IntKind};
use crate::error::{Diagnostic, Result};

impl Generator<'_, '_> {
    /// `type T` (abstract): a `pub use` of the binding, or an error.
    pub(super) fn emit_abs_type(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let docs = s.docs.clone();
        let name = names::ident(&s.name);
        let path = self.type_path(sym)?;
        self.line("");
        self.doc(&docs);
        self.line(&format!("pub use {path} as {name};"));
        Ok(())
    }

    /// `type T = U`
    pub(super) fn emit_alias(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let docs = s.docs.clone();
        let name = names::ident(&s.name);
        let Some(TypeDef::Alias(target)) = self.a.type_defs.get(&sym) else {
            unreachable!()
        };
        let target = self.rust_type(&target.clone())?;
        self.line("");
        self.doc(&docs);
        self.line(&format!("pub type {name} = {target};"));
        Ok(())
    }

    /// `enum E : R { .. } default D` -> `fpp_enum!`.
    pub(super) fn emit_enum(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let docs = s.docs.clone();
        let name = names::ident(&s.name);
        let Some(TypeDef::Enum {
            repr,
            constants,
            default,
        }) = self.a.type_defs.get(&sym).cloned()
        else {
            unreachable!()
        };
        self.line("");
        self.line("::fprime_fw::fpp_enum! {");
        self.indent();
        self.doc(&docs);
        self.line(&format!("pub enum {name} : {} {{", repr.rust_name()));
        self.indent();
        for (cname, value, csym) in &constants {
            let cdocs = self.a.symbols.sym(*csym).docs.clone();
            self.doc(&cdocs);
            self.line(&format!("{} = {value},", names::ident(cname)));
        }
        self.dedent();
        self.line("}");
        self.line(&format!("default {}", names::ident(&default)));
        self.dedent();
        self.line("}");
        Ok(())
    }

    /// `array A = [n] T default d` -> `fpp_array!`.
    pub(super) fn emit_array(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let docs = s.docs.clone();
        let name = names::ident(&s.name);
        let Some(TypeDef::Array {
            size, elt, default, ..
        }) = self.a.type_defs.get(&sym).cloned()
        else {
            unreachable!()
        };
        let elt_rs = self.rust_type(&elt)?;
        let derives = self.derives_for(&Type::Array(sym));
        let default_rs = self.render_value(&default)?;
        self.line("");
        self.line("::fprime_fw::fpp_array! {");
        self.indent();
        self.doc(&docs);
        self.line(&format!("#[derive({derives})]"));
        self.line(&format!("pub array {name} = [{elt_rs}; {size}]"));
        let Value::Array(_, elts) = &default else {
            unreachable!()
        };
        let all_same = elts.windows(2).all(|w| w[0] == w[1]);
        if all_same && !elts.is_empty() && self.is_copy(&elt) {
            let one = self.render_value(&elts[0])?;
            self.line(&format!("default fill {one}"));
        } else {
            let _ = default_rs;
            let mut parts = Vec::new();
            for e in elts {
                parts.push(self.render_value(e)?);
            }
            self.line(&format!("default [{}]", parts.join(", ")));
        }
        self.dedent();
        self.line("}");
        Ok(())
    }

    /// A helper array type for a struct member `x: [n] T`.
    pub(super) fn emit_helper_array(&mut self, name: &str, elt: &Type, size: u64) -> Result<()> {
        let elt_rs = self.rust_type(elt)?;
        let derives = self.derives_for(&Type::AnonArray(Some(size), Box::new(elt.clone())));
        let default_loc = crate::error::Loc::none();
        // Element default: rendered from the analysis default value.
        let mut a = self.a.clone_for_defaults();
        let d = a.default_value(elt, &default_loc)?;
        let one = self.render_value(&d)?;
        self.line("");
        self.line("::fprime_fw::fpp_array! {");
        self.indent();
        self.line(&format!(
            "/// Array member type (`[{size}] T` inside a struct)."
        ));
        self.line(&format!("#[derive({derives})]"));
        self.line(&format!("pub array {name} = [{elt_rs}; {size}]"));
        self.line(&format!("default fill {one}"));
        self.dedent();
        self.line("}");
        Ok(())
    }

    /// `struct S { .. } default d` -> `fpp_struct!`.
    pub(super) fn emit_struct(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let docs = s.docs.clone();
        let sname = s.name.clone();
        let name = names::ident(&sname);
        let Some(TypeDef::Struct { members, default }) = self.a.type_defs.get(&sym).cloned() else {
            unreachable!()
        };
        // Member types; array members get helper named arrays because the
        // macro layer needs `FppSized` on every member type.
        let mut member_types: Vec<String> = Vec::new();
        for m in &members {
            let t = match &m.ty {
                Type::AnonArray(Some(n), elt) => {
                    let helper = self.helper_array(&sname, &m.name, elt, *n);
                    names::ident(&helper)
                }
                other => self.rust_type(other)?,
            };
            member_types.push(t);
        }
        let derives = self.derives_for(&Type::Struct(sym));
        self.line("");
        self.line("::fprime_fw::fpp_struct! {");
        self.indent();
        self.doc(&docs);
        self.line(&format!("#[derive({derives})]"));
        self.line(&format!("pub struct {name} {{"));
        self.indent();
        for (m, t) in members.iter().zip(member_types.iter()) {
            self.doc(&m.docs);
            let f = names::snake_ident(&m.name);
            let plain = names::snake(&m.name);
            self.line(&format!("{f}: {t} {{ get_{plain}, set_{plain} }},"));
        }
        self.dedent();
        self.line("}");
        // Default clause: every member, from the resolved default value.
        let Value::Struct(_, dvals) = &default else {
            unreachable!()
        };
        self.line("default {");
        self.indent();
        for (m, t) in members.iter().zip(member_types.iter()) {
            let v = dvals
                .iter()
                .find(|(n, _)| *n == m.name)
                .map(|(_, v)| v.clone())
                .expect("default has every member");
            let rendered = match (&m.ty, &v) {
                (Type::AnonArray(..), Value::Array(_, elts)) => {
                    let mut parts = Vec::new();
                    for e in elts {
                        parts.push(self.render_value(e)?);
                    }
                    format!("{t}::new([{}])", parts.join(", "))
                }
                _ => self.render_value(&v)?,
            };
            self.line(&format!("{} = {rendered},", names::snake_ident(&m.name)));
        }
        self.dedent();
        self.line("}");
        self.dedent();
        self.line("}");
        Ok(())
    }

    /// `constant c = v`.
    pub(super) fn emit_constant(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let docs = s.docs.clone();
        let name = names::ident(&s.name);
        let v = self
            .a
            .values
            .get(&sym)
            .cloned()
            .ok_or_else(|| Diagnostic::codegen(s.loc.clone(), "constant was not evaluated"))?;
        self.line("");
        self.doc(&docs);
        match &v {
            Value::Int(i, None) => {
                let ty = if i64::try_from(*i).is_ok() {
                    "i64"
                } else if u64::try_from(*i).is_ok() {
                    "u64"
                } else {
                    "i128"
                };
                self.line(&format!("pub const {name}: {ty} = {i};"));
            }
            Value::Int(i, Some(k)) => {
                self.line(&format!("pub const {name}: {} = {i};", k.rust_name()));
            }
            Value::Float(f, k) => {
                let ty = match k {
                    FloatKind::F32 => "f32",
                    FloatKind::F64 => "f64",
                };
                self.line(&format!("pub const {name}: {ty} = {};", render_float(*f)));
            }
            Value::Bool(b) => self.line(&format!("pub const {name}: bool = {b};")),
            Value::Str(st) => self.line(&format!("pub const {name}: &str = {st:?};")),
            Value::Enum(e, c, _) => {
                let path = self.type_path(*e)?;
                self.line(&format!(
                    "pub const {name}: {path} = {path}::{};",
                    names::ident(c)
                ));
            }
            Value::Array(Some(t), _) | Value::Struct(Some(t), _) => {
                let path = self.type_path(*t)?;
                let body = self.render_value(&v)?;
                self.line(&format!("pub fn {name}() -> {path} {{"));
                self.indent();
                self.line(&body);
                self.dedent();
                self.line("}");
            }
            Value::Array(None, elts) => {
                let ety = self.rust_type(&v.ty())?;
                let mut parts = Vec::new();
                for e in elts {
                    parts.push(self.render_value(e)?);
                }
                self.line(&format!(
                    "pub const {name}: {ety} = [{}];",
                    parts.join(", ")
                ));
            }
            Value::Struct(None, _) => {
                self.line(&format!(
                    "// constant {name}: anonymous struct constants are not representable in Rust"
                ));
            }
            Value::Abs(_) => {
                self.line(&format!(
                    "// constant {name}: abstract type values are not representable in Rust"
                ));
            }
        }
        Ok(())
    }

    /// Derives for a generated named array/struct type. The macros derive
    /// `Debug` and `PartialEq` themselves.
    fn derives_for(&self, t: &Type) -> String {
        let mut d = vec!["Clone"];
        if self.is_copy(t) {
            d.push("Copy");
        }
        if self.is_eq(t) {
            d.push("Eq");
        }
        d.join(", ")
    }

    /// Render a value as a Rust expression of its (named) type.
    pub fn render_value(&self, v: &Value) -> Result<String> {
        Ok(match v {
            Value::Int(i, Some(k)) => format!("{i}{}", k.rust_name()),
            Value::Int(i, None) => i.to_string(),
            Value::Float(f, k) => format!(
                "{}{}",
                render_float(*f),
                match k {
                    FloatKind::F32 => "f32",
                    FloatKind::F64 => "f64",
                }
            ),
            Value::Bool(b) => b.to_string(),
            Value::Str(s) => format!("::fprime_fw::FwString::from({s:?})"),
            Value::Enum(e, c, _) => format!("{}::{}", self.type_path(*e)?, names::ident(c)),
            Value::Array(Some(t), elts) => {
                let mut parts = Vec::new();
                for e in elts {
                    parts.push(self.render_value(e)?);
                }
                format!("{}::new([{}])", self.type_path(*t)?, parts.join(", "))
            }
            Value::Array(None, elts) => {
                let mut parts = Vec::new();
                for e in elts {
                    parts.push(self.render_value(e)?);
                }
                format!("[{}]", parts.join(", "))
            }
            Value::Struct(Some(t), ms) => {
                let Some(TypeDef::Struct { members, .. }) = self.a.type_defs.get(t) else {
                    unreachable!()
                };
                let mut parts = Vec::new();
                for m in members {
                    let (_, mv) = ms
                        .iter()
                        .find(|(n, _)| *n == m.name)
                        .expect("struct value has every member");
                    let rendered = match (&m.ty, mv) {
                        (Type::AnonArray(Some(n), elt), Value::Array(_, elts)) => {
                            let owner = &self.a.symbols.sym(*t).name;
                            let helper = format!("{owner}_{}_Array", m.name);
                            let _ = (n, elt);
                            let mut inner = Vec::new();
                            for e in elts {
                                inner.push(self.render_value(e)?);
                            }
                            format!(
                                "{}::new([{}])",
                                self.path_to(*t, &names::ident(&helper)),
                                inner.join(", ")
                            )
                        }
                        _ => self.render_value(mv)?,
                    };
                    parts.push(rendered);
                }
                format!("{}::new({})", self.type_path(*t)?, parts.join(", "))
            }
            Value::Struct(None, _) => {
                return Err(Diagnostic::codegen(
                    crate::error::Loc::none(),
                    "anonymous struct values cannot be rendered in Rust",
                ));
            }
            Value::Abs(s) => format!("<{} as Default>::default()", self.type_path(*s)?),
        })
    }
}

/// A float literal that round-trips and always has a decimal point.
pub fn render_float(f: f64) -> String {
    if f.is_finite() && f.fract() == 0.0 && f.abs() < 1e15 {
        format!("{f:.1}")
    } else {
        let s = format!("{f:?}");
        if s.contains('.') || s.contains('e') || s.contains("inf") || s.contains("NaN") {
            s
        } else {
            format!("{s}.0")
        }
    }
}

/// Integer kind suffix helper for tests.
pub fn int_suffix(k: IntKind) -> &'static str {
    k.rust_name()
}

impl<'a> crate::analysis::Analysis<'a> {
    /// A cheap clone of the state needed to compute default values during
    /// generation (the generator holds the analysis immutably).
    pub fn clone_for_defaults(&self) -> DefaultsView<'_, 'a> {
        DefaultsView { a: self }
    }
}

/// Read-only default-value computation over an analysis.
pub struct DefaultsView<'g, 'a> {
    a: &'g crate::analysis::Analysis<'a>,
}

impl DefaultsView<'_, '_> {
    /// The default value of a (fully resolved) type.
    pub fn default_value(&mut self, t: &Type, loc: &crate::error::Loc) -> Result<Value> {
        let a = self.a;
        Ok(match a.underlying(t) {
            Type::Int(k) => Value::Int(0, Some(k)),
            Type::Integer => Value::Int(0, None),
            Type::Float(k) => Value::Float(0.0, k),
            Type::Bool => Value::Bool(false),
            Type::String(_) => Value::Str(String::new()),
            Type::Abs(s) => Value::Abs(s),
            Type::Alias(_) => unreachable!(),
            Type::Enum(s) => {
                let Some(TypeDef::Enum {
                    constants, default, ..
                }) = a.type_defs.get(&s)
                else {
                    return Err(Diagnostic::codegen(loc.clone(), "unresolved enum"));
                };
                let (name, v, _) = constants.iter().find(|c| c.0 == *default).expect("default");
                Value::Enum(s, name.clone(), *v)
            }
            Type::Array(s) => match a.type_defs.get(&s) {
                Some(TypeDef::Array { default, .. }) => default.clone(),
                _ => return Err(Diagnostic::codegen(loc.clone(), "unresolved array")),
            },
            Type::Struct(s) => match a.type_defs.get(&s) {
                Some(TypeDef::Struct { default, .. }) => default.clone(),
                _ => return Err(Diagnostic::codegen(loc.clone(), "unresolved struct")),
            },
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
}

/// Whether a symbol is a type definition (used by the driver).
pub fn is_type_def(def: Def<'_>) -> bool {
    matches!(
        def,
        Def::AbsType(_) | Def::AliasType(_) | Def::Array(_) | Def::Enum(_) | Def::Struct(_)
    )
}
