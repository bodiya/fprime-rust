//! FPP presentation format strings (`Format.scala`): a prefix, then a
//! sequence of replacement fields each followed by a suffix. Fields are
//! `{}`, integer forms `{c}` `{d}` `{x}` `{o}`, and rational forms `{e}`
//! `{f}` `{g}` with an optional precision `{.3f}`. `{{` and `}}` escape
//! the braces.

use crate::error::{Diagnostic, Loc, Result};

/// A parsed format string.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Format {
    /// Text before the first field.
    pub prefix: String,
    /// Each field with the text that follows it.
    pub fields: Vec<(Field, String)>,
}

/// A replacement field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Field {
    /// `{}`
    Default,
    /// An integer field.
    Integer(IntField),
    /// A rational field with optional precision.
    Rational(Option<u32>, RatField),
}

/// Integer field types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntField {
    Character,
    Decimal,
    Hexadecimal,
    Octal,
}

/// Rational field types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RatField {
    Exponent,
    Fixed,
    General,
}

impl Field {
    /// Whether the field is numeric (integer or rational).
    pub fn is_numeric(&self) -> bool {
        !matches!(self, Field::Default)
    }
}

impl Format {
    /// Parse a format string.
    pub fn parse(s: &str, loc: &Loc) -> Result<Format> {
        let chars: Vec<char> = s.chars().collect();
        let mut i = 0;
        let mut prefix = String::new();
        let mut fields: Vec<(Field, String)> = Vec::new();
        let err = |msg: &str| {
            Diagnostic::semantic(loc.clone(), format!("invalid format string {s:?}: {msg}"))
        };
        // The string currently being accumulated: prefix or a suffix.
        let mut cur = String::new();
        while i < chars.len() {
            let c = chars[i];
            match c {
                '{' if chars.get(i + 1) == Some(&'{') => {
                    cur.push('{');
                    i += 2;
                }
                '}' if chars.get(i + 1) == Some(&'}') => {
                    cur.push('}');
                    i += 2;
                }
                '}' => return Err(err("unmatched '}'")),
                '{' => {
                    // A field.
                    let end = chars[i + 1..]
                        .iter()
                        .position(|c| *c == '}')
                        .ok_or_else(|| err("unterminated replacement field"))?;
                    let spec: String = chars[i + 1..i + 1 + end].iter().collect();
                    let field = Self::parse_field(&spec)
                        .ok_or_else(|| err(&format!("invalid replacement field {{{spec}}}")))?;
                    if fields.is_empty() {
                        prefix = std::mem::take(&mut cur);
                    } else {
                        let last = fields.len() - 1;
                        fields[last].1 = std::mem::take(&mut cur);
                    }
                    fields.push((field, String::new()));
                    i += end + 2;
                }
                other => {
                    cur.push(other);
                    i += 1;
                }
            }
        }
        if fields.is_empty() {
            prefix = cur;
        } else {
            let last = fields.len() - 1;
            fields[last].1 = cur;
        }
        Ok(Format { prefix, fields })
    }

    fn parse_field(spec: &str) -> Option<Field> {
        if spec.is_empty() {
            return Some(Field::Default);
        }
        match spec {
            "c" => return Some(Field::Integer(IntField::Character)),
            "d" => return Some(Field::Integer(IntField::Decimal)),
            "x" => return Some(Field::Integer(IntField::Hexadecimal)),
            "o" => return Some(Field::Integer(IntField::Octal)),
            _ => {}
        }
        let (precision, ty) = if let Some(rest) = spec.strip_prefix('.') {
            let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
            if digits.is_empty() {
                return None;
            }
            let p: u32 = digits.parse().ok()?;
            if p > 100 {
                return None;
            }
            (Some(p), &rest[digits.len()..])
        } else {
            (None, spec)
        };
        let ty = match ty {
            "e" => RatField::Exponent,
            "f" => RatField::Fixed,
            "g" => RatField::General,
            _ => return None,
        };
        Some(Field::Rational(precision, ty))
    }

    /// Number of replacement fields.
    pub fn num_fields(&self) -> usize {
        self.fields.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_fields_and_escapes() {
        let f = Format::parse("a {{b}} {} c {x} {.3f} d", &Loc::none()).unwrap();
        assert_eq!(f.prefix, "a {b} ");
        assert_eq!(
            f.fields,
            vec![
                (Field::Default, " c ".to_string()),
                (Field::Integer(IntField::Hexadecimal), " ".to_string()),
                (Field::Rational(Some(3), RatField::Fixed), " d".to_string()),
            ]
        );
        assert!(Format::parse("{q}", &Loc::none()).is_err());
        assert!(Format::parse("}", &Loc::none()).is_err());
        assert!(Format::parse("{", &Loc::none()).is_err());
    }
}
