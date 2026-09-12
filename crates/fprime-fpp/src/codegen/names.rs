//! Rust naming for FPP identifiers.
//!
//! FPP names are kept verbatim where Rust allows (modules, types,
//! constants, enum constants), so the generated code greps like the model
//! and the C++ output do. Functions, fields and port/handler names use
//! `snake_case` derived from the FPP `camelCase`, per the workspace
//! conventions. Rust keywords are escaped with `r#` (or a trailing `_`
//! for the few that cannot be raw).

/// Rust keywords that cannot be used as raw identifiers.
const NOT_RAWABLE: &[&str] = &["self", "Self", "super", "crate", "_"];

/// Rust keywords (strict and reserved).
const KEYWORDS: &[&str] = &[
    "as", "break", "const", "continue", "crate", "else", "enum", "extern", "false", "fn", "for",
    "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub", "ref", "return",
    "self", "Self", "static", "struct", "super", "trait", "true", "type", "unsafe", "use", "where",
    "while", "async", "await", "dyn", "abstract", "become", "box", "do", "final", "macro",
    "override", "priv", "typeof", "unsized", "virtual", "yield", "try", "gen",
];

/// Escape an identifier for use in Rust source, verbatim otherwise.
pub fn ident(name: &str) -> String {
    if NOT_RAWABLE.contains(&name) {
        return format!("{name}_");
    }
    if KEYWORDS.contains(&name) {
        return format!("r#{name}");
    }
    name.to_string()
}

/// `camelCase` / `PascalCase` / `SCREAMING_CASE` to `snake_case`.
pub fn snake(name: &str) -> String {
    let chars: Vec<char> = name.chars().collect();
    let mut out = String::with_capacity(name.len() + 4);
    for (i, c) in chars.iter().enumerate() {
        if c.is_ascii_uppercase() {
            let prev_lower_or_digit =
                i > 0 && (chars[i - 1].is_ascii_lowercase() || chars[i - 1].is_ascii_digit());
            let next_lower = i + 1 < chars.len() && chars[i + 1].is_ascii_lowercase();
            let prev_upper = i > 0 && chars[i - 1].is_ascii_uppercase();
            if i > 0 && chars[i - 1] != '_' && (prev_lower_or_digit || (prev_upper && next_lower)) {
                out.push('_');
            }
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(*c);
        }
    }
    out
}

/// A snake_case identifier, keyword-escaped.
pub fn snake_ident(name: &str) -> String {
    ident(&snake(name))
}

/// `SCREAMING_SNAKE_CASE` for generated constants.
pub fn screaming(name: &str) -> String {
    snake(name).to_ascii_uppercase()
}

/// `PascalCase` from a snake or camel name (for generated type names).
pub fn pascal(name: &str) -> String {
    let mut out = String::new();
    let mut upper = true;
    for c in name.chars() {
        if c == '_' {
            upper = true;
        } else if upper {
            out.push(c.to_ascii_uppercase());
            upper = false;
        } else {
            out.push(c);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snake_case_rules() {
        assert_eq!(snake("schedIn"), "sched_in");
        assert_eq!(snake("dataReturnOut"), "data_return_out");
        assert_eq!(snake("CMD_ASYNC"), "cmd_async");
        assert_eq!(snake("TlmU32"), "tlm_u32");
        assert_eq!(snake("ParamF64Ext"), "param_f64_ext");
        assert_eq!(snake("SG1"), "sg1");
        assert_eq!(snake("compCmdReg"), "comp_cmd_reg");
        assert_eq!(snake("SignalGen"), "signal_gen");
        assert_eq!(snake("HTTPServer"), "http_server");
        assert_eq!(snake("already_snake"), "already_snake");
    }

    #[test]
    fn keyword_escapes() {
        assert_eq!(ident("type"), "r#type");
        assert_eq!(ident("self"), "self_");
        assert_eq!(ident("normal"), "normal");
        assert_eq!(snake_ident("Match"), "r#match");
    }

    #[test]
    fn other_cases() {
        assert_eq!(screaming("productGetOut"), "PRODUCT_GET_OUT");
        assert_eq!(pascal("sched_in"), "SchedIn");
        assert_eq!(pascal("SG1"), "SG1");
    }
}
