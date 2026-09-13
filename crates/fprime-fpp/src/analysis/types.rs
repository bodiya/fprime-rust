//! Types and values of the FPP constant language (`Type.scala` /
//! `Value.scala` in the reference compiler).
//!
//! Named types (enums, structs, arrays, aliases, abstract types) are
//! represented by the id of their defining symbol; their definitions live
//! in [`super::Analysis`]'s type table, so `Type` itself stays small and
//! non-recursive.

use super::symbols::SymId;
use crate::ast::{FloatKind, IntKind};
use std::fmt;

/// Default FPP string size (`string` with no `size`).
pub const DEFAULT_STRING_SIZE: u64 = 80;

/// An FPP type.
#[derive(Debug, Clone, PartialEq)]
pub enum Type {
    /// A primitive integer type.
    Int(IntKind),
    /// A primitive float type.
    Float(FloatKind),
    /// `bool`
    Bool,
    /// `string [size n]`
    String(Option<u64>),
    /// The type of an integer literal (unbounded).
    Integer,
    /// An abstract type.
    Abs(SymId),
    /// A named alias of another type.
    Alias(SymId),
    /// A named enum.
    Enum(SymId),
    /// A named array.
    Array(SymId),
    /// A named struct.
    Struct(SymId),
    /// An array literal's type: optional size and element type.
    AnonArray(Option<u64>, Box<Type>),
    /// A struct literal's type.
    AnonStruct(Vec<(String, Type)>),
}

impl Type {
    /// Whether this is a numeric (int or float) type.
    pub fn is_numeric(&self) -> bool {
        matches!(self, Type::Int(_) | Type::Float(_) | Type::Integer)
    }

    /// Whether this is a primitive type (numeric or bool).
    pub fn is_primitive(&self) -> bool {
        self.is_numeric() || matches!(self, Type::Bool)
    }

    /// Whether a scalar of this type may fill an array/struct.
    pub fn is_promotable(&self) -> bool {
        self.is_numeric() || matches!(self, Type::Bool | Type::String(_))
    }
}

impl fmt::Display for Type {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Type::Int(k) => f.write_str(k.fpp_name()),
            Type::Float(FloatKind::F32) => f.write_str("F32"),
            Type::Float(FloatKind::F64) => f.write_str("F64"),
            Type::Bool => f.write_str("bool"),
            Type::String(None) => f.write_str("string"),
            Type::String(Some(n)) => write!(f, "string size {n}"),
            Type::Integer => f.write_str("Integer"),
            Type::Abs(s) | Type::Alias(s) | Type::Enum(s) | Type::Array(s) | Type::Struct(s) => {
                write!(f, "<type #{s}>")
            }
            Type::AnonArray(Some(n), t) => write!(f, "[{n}] {t}"),
            Type::AnonArray(None, t) => write!(f, "[] {t}"),
            Type::AnonStruct(ms) => {
                f.write_str("{ ")?;
                for (i, (n, t)) in ms.iter().enumerate() {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    write!(f, "{n}: {t}")?;
                }
                f.write_str(" }")
            }
        }
    }
}

/// An FPP constant value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// An integer; `None` kind is the unbounded literal type.
    Int(i128, Option<IntKind>),
    /// A float.
    Float(f64, FloatKind),
    /// A bool.
    Bool(bool),
    /// A string.
    Str(String),
    /// An enum constant: enum symbol, constant name, numeric value.
    Enum(SymId, String, i128),
    /// An array (named type if `Some`).
    Array(Option<SymId>, Vec<Value>),
    /// A struct (named type if `Some`), members in declaration order.
    Struct(Option<SymId>, Vec<(String, Value)>),
    /// The (opaque) default value of an abstract type.
    Abs(SymId),
}

impl Value {
    /// The type of the value.
    pub fn ty(&self) -> Type {
        match self {
            Value::Int(_, Some(k)) => Type::Int(*k),
            Value::Int(_, None) => Type::Integer,
            Value::Float(_, k) => Type::Float(*k),
            Value::Bool(_) => Type::Bool,
            Value::Str(_) => Type::String(None),
            Value::Enum(s, _, _) => Type::Enum(*s),
            Value::Array(Some(s), _) => Type::Array(*s),
            Value::Array(None, elts) => Type::AnonArray(
                Some(elts.len() as u64),
                Box::new(elts.first().map(Value::ty).unwrap_or(Type::Integer)),
            ),
            Value::Struct(Some(s), _) => Type::Struct(*s),
            Value::Struct(None, ms) => {
                Type::AnonStruct(ms.iter().map(|(n, v)| (n.clone(), v.ty())).collect())
            }
            Value::Abs(s) => Type::Abs(*s),
        }
    }

    /// The integer value, if this is an integer or enum constant.
    pub fn as_int(&self) -> Option<i128> {
        match self {
            Value::Int(v, _) => Some(*v),
            Value::Enum(_, _, v) => Some(*v),
            _ => None,
        }
    }

    /// The float value, if numeric.
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Int(v, _) => Some(*v as f64),
            Value::Float(v, _) => Some(*v),
            Value::Enum(_, _, v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Whether the value is numerically zero (for division checks).
    pub fn is_zero(&self) -> bool {
        match self {
            Value::Int(v, _) => *v == 0,
            Value::Float(v, _) => *v == 0.0,
            _ => false,
        }
    }
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Int(v, _) => write!(f, "{v}"),
            Value::Float(v, _) => write!(f, "{v}"),
            Value::Bool(v) => write!(f, "{v}"),
            Value::Str(s) => write!(f, "{s:?}"),
            Value::Enum(_, n, v) => write!(f, "{n} ({v})"),
            Value::Array(_, elts) => {
                f.write_str("[ ")?;
                for (i, e) in elts.iter().enumerate() {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    write!(f, "{e}")?;
                }
                f.write_str(" ]")
            }
            Value::Struct(_, ms) => {
                f.write_str("{ ")?;
                for (i, (n, v)) in ms.iter().enumerate() {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    write!(f, "{n} = {v}")?;
                }
                f.write_str(" }")
            }
            Value::Abs(_) => f.write_str("<abstract type value>"),
        }
    }
}

/// Range of a primitive integer kind.
pub fn int_range(kind: IntKind) -> (i128, i128) {
    let bits = kind.bits();
    if kind.signed() {
        (-(1i128 << (bits - 1)), (1i128 << (bits - 1)) - 1)
    } else {
        (0, (1i128 << bits) - 1)
    }
}

/// Wider of two integer kinds for the common-type rule: the larger width,
/// signed if either is signed.
pub fn wider_int(a: IntKind, b: IntKind) -> IntKind {
    let bits = a.bits().max(b.bits());
    let signed = a.signed() || b.signed();
    match (bits, signed) {
        (8, true) => IntKind::I8,
        (8, false) => IntKind::U8,
        (16, true) => IntKind::I16,
        (16, false) => IntKind::U16,
        (32, true) => IntKind::I32,
        (32, false) => IntKind::U32,
        (_, true) => IntKind::I64,
        (_, false) => IntKind::U64,
    }
}
