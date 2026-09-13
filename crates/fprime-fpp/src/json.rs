//! A minimal JSON value with a pretty printer and a parser. The dictionary
//! back end writes JSON; the tests read the reference compiler's output
//! back to compare structurally. No third-party dependencies, like the rest
//! of the workspace.

use std::fmt::Write as _;

/// A JSON value. Objects keep insertion order.
#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    /// An integer (kept exact; F Prime ids and U64 constants exceed f64).
    Int(i128),
    Float(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    /// An empty object.
    pub fn obj() -> Self {
        Json::Obj(Vec::new())
    }

    /// Append a member to an object (panics on non-objects).
    pub fn push(&mut self, key: &str, value: Json) {
        match self {
            Json::Obj(members) => members.push((key.to_string(), value)),
            _ => panic!("push on a non-object"),
        }
    }

    /// Append a member if `value` is `Some`.
    pub fn push_opt(&mut self, key: &str, value: Option<Json>) {
        if let Some(v) = value {
            self.push(key, v);
        }
    }

    /// The member `key` of an object, if any.
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(members) => members.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    /// The string value, if this is a string.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }

    /// The elements, if this is an array.
    pub fn as_arr(&self) -> Option<&[Json]> {
        match self {
            Json::Arr(a) => Some(a),
            _ => None,
        }
    }

    /// Pretty-print with two-space indentation and a trailing newline.
    pub fn to_pretty(&self) -> String {
        let mut out = String::new();
        self.write(&mut out, 0);
        out.push('\n');
        out
    }

    fn write(&self, out: &mut String, depth: usize) {
        match self {
            Json::Null => out.push_str("null"),
            Json::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            Json::Int(i) => {
                let _ = write!(out, "{i}");
            }
            Json::Float(f) => {
                if f.is_finite() {
                    let _ = write!(out, "{f:?}");
                } else {
                    out.push_str("null");
                }
            }
            Json::Str(s) => write_string(out, s),
            Json::Arr(elts) => {
                if elts.is_empty() {
                    out.push_str("[]");
                    return;
                }
                out.push('[');
                for (i, e) in elts.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    out.push('\n');
                    indent(out, depth + 1);
                    e.write(out, depth + 1);
                }
                out.push('\n');
                indent(out, depth);
                out.push(']');
            }
            Json::Obj(members) => {
                if members.is_empty() {
                    out.push_str("{}");
                    return;
                }
                out.push('{');
                for (i, (k, v)) in members.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    out.push('\n');
                    indent(out, depth + 1);
                    write_string(out, k);
                    out.push_str(": ");
                    v.write(out, depth + 1);
                }
                out.push('\n');
                indent(out, depth);
                out.push('}');
            }
        }
    }

    /// Parse a JSON text.
    pub fn parse(text: &str) -> Result<Json, String> {
        let mut p = Parser {
            chars: text.chars().collect(),
            pos: 0,
        };
        p.skip_ws();
        let v = p.value()?;
        p.skip_ws();
        if p.pos != p.chars.len() {
            return Err(p.err("trailing characters"));
        }
        Ok(v)
    }
}

fn indent(out: &mut String, depth: usize) {
    for _ in 0..depth {
        out.push_str("  ");
    }
}

fn write_string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

struct Parser {
    chars: Vec<char>,
    pos: usize,
}

impl Parser {
    fn err(&self, msg: &str) -> String {
        format!("JSON parse error at offset {}: {msg}", self.pos)
    }

    fn peek(&self) -> Option<char> {
        self.chars.get(self.pos).copied()
    }

    fn skip_ws(&mut self) {
        while matches!(self.peek(), Some(' ' | '\t' | '\n' | '\r')) {
            self.pos += 1;
        }
    }

    fn expect(&mut self, c: char) -> Result<(), String> {
        if self.peek() == Some(c) {
            self.pos += 1;
            Ok(())
        } else {
            Err(self.err(&format!("expected {c:?}")))
        }
    }

    fn literal(&mut self, word: &str, v: Json) -> Result<Json, String> {
        for c in word.chars() {
            self.expect(c)?;
        }
        Ok(v)
    }

    fn value(&mut self) -> Result<Json, String> {
        match self.peek() {
            Some('n') => self.literal("null", Json::Null),
            Some('t') => self.literal("true", Json::Bool(true)),
            Some('f') => self.literal("false", Json::Bool(false)),
            Some('"') => Ok(Json::Str(self.string()?)),
            Some('[') => {
                self.pos += 1;
                let mut elts = Vec::new();
                self.skip_ws();
                if self.peek() == Some(']') {
                    self.pos += 1;
                    return Ok(Json::Arr(elts));
                }
                loop {
                    self.skip_ws();
                    elts.push(self.value()?);
                    self.skip_ws();
                    match self.peek() {
                        Some(',') => self.pos += 1,
                        Some(']') => {
                            self.pos += 1;
                            return Ok(Json::Arr(elts));
                        }
                        _ => return Err(self.err("expected ',' or ']'")),
                    }
                }
            }
            Some('{') => {
                self.pos += 1;
                let mut members = Vec::new();
                self.skip_ws();
                if self.peek() == Some('}') {
                    self.pos += 1;
                    return Ok(Json::Obj(members));
                }
                loop {
                    self.skip_ws();
                    let k = self.string()?;
                    self.skip_ws();
                    self.expect(':')?;
                    self.skip_ws();
                    let v = self.value()?;
                    members.push((k, v));
                    self.skip_ws();
                    match self.peek() {
                        Some(',') => self.pos += 1,
                        Some('}') => {
                            self.pos += 1;
                            return Ok(Json::Obj(members));
                        }
                        _ => return Err(self.err("expected ',' or '}'")),
                    }
                }
            }
            Some(c) if c == '-' || c.is_ascii_digit() => self.number(),
            _ => Err(self.err("unexpected character")),
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.expect('"')?;
        let mut s = String::new();
        loop {
            let Some(c) = self.peek() else {
                return Err(self.err("unterminated string"));
            };
            self.pos += 1;
            match c {
                '"' => return Ok(s),
                '\\' => {
                    let Some(e) = self.peek() else {
                        return Err(self.err("unterminated escape"));
                    };
                    self.pos += 1;
                    match e {
                        '"' => s.push('"'),
                        '\\' => s.push('\\'),
                        '/' => s.push('/'),
                        'b' => s.push('\u{8}'),
                        'f' => s.push('\u{c}'),
                        'n' => s.push('\n'),
                        'r' => s.push('\r'),
                        't' => s.push('\t'),
                        'u' => {
                            let mut code = 0u32;
                            for _ in 0..4 {
                                let d = self
                                    .peek()
                                    .and_then(|c| c.to_digit(16))
                                    .ok_or_else(|| self.err("bad \\u escape"))?;
                                self.pos += 1;
                                code = code * 16 + d;
                            }
                            // Surrogate pairs.
                            if (0xD800..0xDC00).contains(&code) {
                                let mut low = 0u32;
                                if self.peek() == Some('\\') {
                                    self.pos += 1;
                                    self.expect('u')?;
                                    for _ in 0..4 {
                                        let d = self
                                            .peek()
                                            .and_then(|c| c.to_digit(16))
                                            .ok_or_else(|| self.err("bad \\u escape"))?;
                                        self.pos += 1;
                                        low = low * 16 + d;
                                    }
                                }
                                code =
                                    0x10000 + ((code - 0xD800) << 10) + (low.wrapping_sub(0xDC00));
                            }
                            s.push(char::from_u32(code).unwrap_or('\u{FFFD}'));
                        }
                        _ => return Err(self.err("bad escape")),
                    }
                }
                c => s.push(c),
            }
        }
    }

    fn number(&mut self) -> Result<Json, String> {
        let start = self.pos;
        if self.peek() == Some('-') {
            self.pos += 1;
        }
        let mut is_float = false;
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() {
                self.pos += 1;
            } else if c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-' {
                is_float = true;
                self.pos += 1;
            } else {
                break;
            }
        }
        let text: String = self.chars[start..self.pos].iter().collect();
        if is_float {
            text.parse::<f64>()
                .map(Json::Float)
                .map_err(|_| self.err("bad number"))
        } else {
            text.parse::<i128>()
                .map(Json::Int)
                .map_err(|_| self.err("bad number"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips() {
        let mut o = Json::obj();
        o.push("a", Json::Int(1));
        o.push(
            "b",
            Json::Arr(vec![Json::Float(1.5), Json::Str("x\"y\n".into())]),
        );
        o.push("c", Json::Obj(Vec::new()));
        o.push("d", Json::Null);
        o.push("e", Json::Bool(true));
        let text = o.to_pretty();
        assert_eq!(Json::parse(&text).unwrap(), o);
        assert_eq!(
            Json::parse(" [ -3 , 2.5e3, \"\\u0041\" ] ").unwrap(),
            Json::Arr(vec![
                Json::Int(-3),
                Json::Float(2500.0),
                Json::Str("A".into())
            ])
        );
        assert!(Json::parse("[1,]").is_err());
        assert!(Json::parse("{\"a\":1} x").is_err());
    }
}
