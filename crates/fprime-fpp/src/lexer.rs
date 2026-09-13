//! The FPP lexer — a direct port of the reference compiler's `Lexer.scala`.
//!
//! The token stream carries the language's newline discipline: a run of
//! newlines and `#` comments becomes a single [`Tok::Eol`], *except* that
//! newlines are swallowed after tokens that cannot end an element (`=`,
//! `:`, `,`, `(`, `{`, `[`, the arithmetic operators, `;`) and before the
//! closing brackets `)`, `]`, `}`. Annotations (`@ ...` / `@< ...`) also
//! swallow the newlines that follow them. `\` at the end of a line is a
//! continuation. Identifiers may be escaped with `$` to use a keyword as a
//! name.

use crate::error::{Diagnostic, Loc, Phase, Result};
use std::path::PathBuf;
use std::rc::Rc;

/// Token kinds. Keywords are individual variants so the parser can match on
/// them directly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Tok {
    // ---- literals / names ---------------------------------------------
    /// An identifier (a `$`-escaped keyword is also an identifier).
    Ident(String),
    /// `@ text`
    PreAnnotation(String),
    /// `@< text`
    PostAnnotation(String),
    /// A floating-point literal, verbatim.
    LitFloat(String),
    /// An integer literal, verbatim (decimal or `0x` hex).
    LitInt(String),
    /// A string literal, unescaped and with multi-line indentation removed.
    LitString(String),
    // ---- keywords --------------------------------------------------------
    F32,
    F64,
    I16,
    I32,
    I64,
    I8,
    U16,
    U32,
    U64,
    U8,
    Action,
    Active,
    Activity,
    Always,
    Array,
    Assert,
    Async,
    At,
    Base,
    Block,
    Bool,
    Change,
    Choice,
    Command,
    Component,
    Connections,
    Constant,
    Container,
    Cpu,
    Default,
    Deployment,
    Diagnostic,
    Dictionary,
    Do,
    Drop,
    Else,
    Enter,
    Entry,
    Enum,
    Event,
    Every,
    Exit,
    External,
    False,
    Fatal,
    Format,
    Get,
    Group,
    Guard,
    Guarded,
    Health,
    High,
    Hook,
    Id,
    If,
    Implements,
    Import,
    Include,
    Initial,
    Input,
    Instance,
    Interface,
    Internal,
    Locate,
    Low,
    Machine,
    Match,
    Module,
    Omit,
    On,
    Opcode,
    Orange,
    Output,
    Packet,
    Packets,
    Param,
    Passive,
    Phase,
    Port,
    Priority,
    Product,
    Queue,
    Queued,
    Record,
    Recv,
    Red,
    Ref,
    Reg,
    Request,
    Resp,
    Save,
    Send,
    Serial,
    Set,
    Severity,
    Signal,
    Size,
    Sizeof,
    Stack,
    State,
    String,
    Struct,
    Sync,
    System,
    Telemetry,
    Text,
    Throttle,
    Time,
    Topology,
    True,
    Type,
    Unmatched,
    Update,
    Warning,
    With,
    Yellow,
    // ---- symbols ---------------------------------------------------------
    Colon,
    Comma,
    Dot,
    Eol,
    Equals,
    LBrace,
    LBracket,
    LParen,
    Minus,
    Plus,
    RArrow,
    RBrace,
    RBracket,
    RParen,
    Semi,
    Slash,
    Star,
    LShift,
    RShift,
    /// End of input.
    Eof,
}

impl Tok {
    /// The keyword for a word, if it is one.
    fn keyword(word: &str) -> Option<Tok> {
        Some(match word {
            "F32" => Tok::F32,
            "F64" => Tok::F64,
            "I16" => Tok::I16,
            "I32" => Tok::I32,
            "I64" => Tok::I64,
            "I8" => Tok::I8,
            "U16" => Tok::U16,
            "U32" => Tok::U32,
            "U64" => Tok::U64,
            "U8" => Tok::U8,
            "action" => Tok::Action,
            "active" => Tok::Active,
            "activity" => Tok::Activity,
            "always" => Tok::Always,
            "array" => Tok::Array,
            "assert" => Tok::Assert,
            "async" => Tok::Async,
            "at" => Tok::At,
            "base" => Tok::Base,
            "block" => Tok::Block,
            "bool" => Tok::Bool,
            "change" => Tok::Change,
            "choice" => Tok::Choice,
            "command" => Tok::Command,
            "component" => Tok::Component,
            "connections" => Tok::Connections,
            "constant" => Tok::Constant,
            "container" => Tok::Container,
            "cpu" => Tok::Cpu,
            "default" => Tok::Default,
            "deployment" => Tok::Deployment,
            "diagnostic" => Tok::Diagnostic,
            "dictionary" => Tok::Dictionary,
            "do" => Tok::Do,
            "drop" => Tok::Drop,
            "else" => Tok::Else,
            "enter" => Tok::Enter,
            "entry" => Tok::Entry,
            "enum" => Tok::Enum,
            "event" => Tok::Event,
            "every" => Tok::Every,
            "exit" => Tok::Exit,
            "external" => Tok::External,
            "false" => Tok::False,
            "fatal" => Tok::Fatal,
            "format" => Tok::Format,
            "get" => Tok::Get,
            "group" => Tok::Group,
            "guard" => Tok::Guard,
            "guarded" => Tok::Guarded,
            "health" => Tok::Health,
            "high" => Tok::High,
            "hook" => Tok::Hook,
            "id" => Tok::Id,
            "if" => Tok::If,
            "implements" => Tok::Implements,
            "import" => Tok::Import,
            "include" => Tok::Include,
            "initial" => Tok::Initial,
            "input" => Tok::Input,
            "instance" => Tok::Instance,
            "interface" => Tok::Interface,
            "internal" => Tok::Internal,
            "locate" => Tok::Locate,
            "low" => Tok::Low,
            "machine" => Tok::Machine,
            "match" => Tok::Match,
            "module" => Tok::Module,
            "omit" => Tok::Omit,
            "on" => Tok::On,
            "opcode" => Tok::Opcode,
            "orange" => Tok::Orange,
            "output" => Tok::Output,
            "packet" => Tok::Packet,
            "packets" => Tok::Packets,
            "param" => Tok::Param,
            "passive" => Tok::Passive,
            "phase" => Tok::Phase,
            "port" => Tok::Port,
            "priority" => Tok::Priority,
            "product" => Tok::Product,
            "queue" => Tok::Queue,
            "queued" => Tok::Queued,
            "record" => Tok::Record,
            "recv" => Tok::Recv,
            "red" => Tok::Red,
            "ref" => Tok::Ref,
            "reg" => Tok::Reg,
            "request" => Tok::Request,
            "resp" => Tok::Resp,
            "save" => Tok::Save,
            "send" => Tok::Send,
            "serial" => Tok::Serial,
            "set" => Tok::Set,
            "severity" => Tok::Severity,
            "signal" => Tok::Signal,
            "size" => Tok::Size,
            "sizeof" => Tok::Sizeof,
            "stack" => Tok::Stack,
            "state" => Tok::State,
            "string" => Tok::String,
            "struct" => Tok::Struct,
            "sync" => Tok::Sync,
            "system" => Tok::System,
            "telemetry" => Tok::Telemetry,
            "text" => Tok::Text,
            "throttle" => Tok::Throttle,
            "time" => Tok::Time,
            "topology" => Tok::Topology,
            "true" => Tok::True,
            "type" => Tok::Type,
            "unmatched" => Tok::Unmatched,
            "update" => Tok::Update,
            "warning" => Tok::Warning,
            "with" => Tok::With,
            "yellow" => Tok::Yellow,
            _ => return None,
        })
    }

    /// Whether `word` is a reserved word (needs `$` to be an identifier).
    pub fn is_keyword(word: &str) -> bool {
        Self::keyword(word).is_some()
    }

    /// Human-readable description for error messages.
    pub fn describe(&self) -> String {
        match self {
            Tok::Ident(s) => format!("identifier `{s}`"),
            Tok::PreAnnotation(_) => "pre annotation".into(),
            Tok::PostAnnotation(_) => "post annotation".into(),
            Tok::LitFloat(s) => format!("floating-point literal `{s}`"),
            Tok::LitInt(s) => format!("integer literal `{s}`"),
            Tok::LitString(_) => "string literal".into(),
            Tok::Eol => "end of line".into(),
            Tok::Eof => "end of file".into(),
            Tok::Colon => "`:`".into(),
            Tok::Comma => "`,`".into(),
            Tok::Dot => "`.`".into(),
            Tok::Equals => "`=`".into(),
            Tok::LBrace => "`{`".into(),
            Tok::LBracket => "`[`".into(),
            Tok::LParen => "`(`".into(),
            Tok::Minus => "`-`".into(),
            Tok::Plus => "`+`".into(),
            Tok::RArrow => "`->`".into(),
            Tok::RBrace => "`}`".into(),
            Tok::RBracket => "`]`".into(),
            Tok::RParen => "`)`".into(),
            Tok::Semi => "`;`".into(),
            Tok::Slash => "`/`".into(),
            Tok::Star => "`*`".into(),
            Tok::LShift => "`<<`".into(),
            Tok::RShift => "`>>`".into(),
            other => format!("keyword `{}`", format!("{other:?}").to_lowercase()),
        }
    }
}

/// A token with its location.
#[derive(Debug, Clone)]
pub struct Token {
    /// The token.
    pub tok: Tok,
    /// Where it starts.
    pub loc: Loc,
}

/// Lex a whole file into tokens (the trailing [`Tok::Eof`] included).
pub fn lex(file: Rc<PathBuf>, text: &str, including: Option<Rc<Loc>>) -> Result<Vec<Token>> {
    let mut lexer = Lexer {
        chars: text.chars().collect(),
        pos: 0,
        line: 1,
        col: 1,
        file,
        including,
        out: Vec::new(),
    };
    lexer.run()?;
    Ok(lexer.out)
}

struct Lexer {
    chars: Vec<char>,
    pos: usize,
    line: u32,
    col: u32,
    file: Rc<PathBuf>,
    including: Option<Rc<Loc>>,
    out: Vec<Token>,
}

const EOF_CHAR: char = '\0';

impl Lexer {
    fn ch(&self) -> char {
        self.chars.get(self.pos).copied().unwrap_or(EOF_CHAR)
    }

    fn peek(&self, n: usize) -> char {
        self.chars.get(self.pos + n).copied().unwrap_or(EOF_CHAR)
    }

    fn at_end(&self) -> bool {
        self.pos >= self.chars.len()
    }

    fn advance(&mut self) -> char {
        let c = self.ch();
        if !self.at_end() {
            self.pos += 1;
            if c == '\n' {
                self.line += 1;
                self.col = 1;
            } else {
                self.col += 1;
            }
        }
        c
    }

    fn loc(&self) -> Loc {
        Loc {
            file: Rc::clone(&self.file),
            line: self.line,
            col: self.col,
            including: self.including.clone(),
        }
    }

    fn error<T>(&self, loc: Loc, msg: impl Into<String>) -> Result<T> {
        Err(Diagnostic::new(Phase::Lexer, loc, msg))
    }

    fn push(&mut self, tok: Tok, loc: Loc) {
        self.out.push(Token { tok, loc });
    }

    fn is_ident_start(c: char) -> bool {
        c.is_ascii_alphabetic() || c == '_'
    }

    fn is_ident_part(c: char) -> bool {
        c.is_ascii_alphanumeric() || c == '_'
    }

    /// Skip newlines, spaces and `#` comments without producing a token.
    fn eat_newlines(&mut self) {
        loop {
            match self.ch() {
                '\n' | ' ' | '\r' => {
                    self.advance();
                }
                '#' => {
                    while !self.at_end() && self.ch() != '\n' {
                        self.advance();
                    }
                }
                _ => return,
            }
        }
    }

    fn run(&mut self) -> Result<()> {
        loop {
            let loc = self.loc();
            let c = self.ch();
            if self.at_end() {
                self.push(Tok::Eof, loc);
                return Ok(());
            }
            match c {
                ' ' | '\u{c}' | '\r' => {
                    self.advance();
                }
                '\t' => {
                    return self.error(
                        loc,
                        "unicode value 9, hex 0x9\nnote: embedded tab characters are not allowed in FPP source files\ntry configuring your editor to convert tabs to spaces",
                    );
                }
                '$' => {
                    if Self::is_ident_start(self.peek(1)) {
                        self.advance();
                        let word = self.ident_rest();
                        self.push(Tok::Ident(word), loc);
                    } else {
                        return self.error(loc, "invalid usage of '$', expected identifier");
                    }
                }
                c if Self::is_ident_start(c) => {
                    let word = self.ident_rest();
                    let tok = Tok::keyword(&word).unwrap_or(Tok::Ident(word));
                    self.push(tok, loc);
                }
                '0'..='9' => {
                    self.number(loc)?;
                }
                '\\' => {
                    self.advance();
                    while self.ch() == ' ' || self.ch() == '\r' {
                        self.advance();
                    }
                    if self.ch() == '\n' {
                        self.advance();
                    } else {
                        return self.error(self.loc(), "expected line continuation");
                    }
                }
                '"' => {
                    self.string(loc)?;
                }
                '.' => {
                    if self.peek(1).is_ascii_digit() {
                        // A fractional literal like `.5`.
                        let mut text = String::from("0.");
                        self.advance();
                        self.fraction(&mut text)?;
                        self.push(Tok::LitFloat(text), loc);
                    } else {
                        self.advance();
                        self.push(Tok::Dot, loc);
                    }
                }
                '\n' | '#' => {
                    self.eat_newlines();
                    match self.ch() {
                        ')' | ']' | '}' => {}
                        _ if self.at_end() => {}
                        _ => self.push(Tok::Eol, loc),
                    }
                }
                '@' => {
                    self.advance();
                    let post = self.ch() == '<';
                    if post {
                        self.advance();
                    }
                    let mut text = String::new();
                    while !self.at_end() && self.ch() != '\n' {
                        text.push(self.advance());
                    }
                    let text = text.trim().to_string();
                    self.push(
                        if post {
                            Tok::PostAnnotation(text)
                        } else {
                            Tok::PreAnnotation(text)
                        },
                        loc,
                    );
                    self.eat_newlines();
                }
                '*' | '+' | '/' | '=' | ';' | ':' | ',' | '(' | '{' | '[' => {
                    self.advance();
                    let tok = match c {
                        '*' => Tok::Star,
                        '+' => Tok::Plus,
                        '/' => Tok::Slash,
                        '=' => Tok::Equals,
                        ';' => Tok::Semi,
                        ':' => Tok::Colon,
                        ',' => Tok::Comma,
                        '(' => Tok::LParen,
                        '{' => Tok::LBrace,
                        _ => Tok::LBracket,
                    };
                    self.push(tok, loc);
                    self.eat_newlines();
                }
                '-' => {
                    self.advance();
                    if self.ch() == '>' {
                        self.advance();
                        self.push(Tok::RArrow, loc);
                    } else {
                        self.push(Tok::Minus, loc);
                    }
                    self.eat_newlines();
                }
                '<' => {
                    self.advance();
                    if self.ch() == '<' {
                        self.advance();
                        self.push(Tok::LShift, loc);
                    } else {
                        return self.error(loc, "'<'");
                    }
                }
                '>' => {
                    self.advance();
                    if self.ch() == '>' {
                        self.advance();
                        self.push(Tok::RShift, loc);
                    } else {
                        return self.error(loc, "'>'");
                    }
                }
                ')' => {
                    self.advance();
                    self.push(Tok::RParen, loc);
                }
                '}' => {
                    self.advance();
                    self.push(Tok::RBrace, loc);
                }
                ']' => {
                    self.advance();
                    self.push(Tok::RBracket, loc);
                }
                other => {
                    return self.error(
                        loc,
                        format!("unicode value {}, hex 0x{:x}", other as u32, other as u32),
                    );
                }
            }
        }
    }

    fn ident_rest(&mut self) -> String {
        let mut word = String::new();
        while Self::is_ident_part(self.ch()) {
            word.push(self.advance());
        }
        word
    }

    fn number(&mut self, loc: Loc) -> Result<()> {
        let mut text = String::new();
        let hex = self.ch() == '0' && matches!(self.peek(1), 'x' | 'X');
        if hex {
            text.push(self.advance());
            self.advance();
            text.push('x');
            if !self.ch().is_ascii_hexdigit() {
                return self.error(loc, "invalid literal number");
            }
            while self.ch().is_ascii_hexdigit() {
                text.push(self.advance());
            }
            self.check_no_letter(&loc)?;
            self.push(Tok::LitInt(text), loc);
            return Ok(());
        }
        while self.ch().is_ascii_digit() {
            text.push(self.advance());
        }
        if self.ch() == '.' {
            text.push(self.advance());
            self.fraction(&mut text)?;
            self.push(Tok::LitFloat(text), loc);
            return Ok(());
        }
        if matches!(self.ch(), 'e' | 'E') {
            let is_float = self.exponent(&mut text)?;
            if is_float {
                self.push(Tok::LitFloat(text), loc);
                return Ok(());
            }
        }
        self.check_no_letter(&loc)?;
        self.push(Tok::LitInt(text), loc);
        Ok(())
    }

    /// Digits after the decimal point plus an optional exponent.
    fn fraction(&mut self, text: &mut String) -> Result<()> {
        while self.ch().is_ascii_digit() {
            text.push(self.advance());
        }
        if matches!(self.ch(), 'e' | 'E') {
            self.exponent(text)?;
        }
        let loc = self.loc();
        self.check_no_letter(&loc)
    }

    /// An exponent, only consumed when digits follow (`1e5`, `2E-3`);
    /// returns whether one was consumed.
    fn exponent(&mut self, text: &mut String) -> Result<bool> {
        let mut n = 1;
        if matches!(self.peek(n), '+' | '-') {
            n += 1;
        }
        if !self.peek(n).is_ascii_digit() {
            return Ok(false);
        }
        text.push(self.advance());
        if matches!(self.ch(), '+' | '-') {
            text.push(self.advance());
        }
        while self.ch().is_ascii_digit() {
            text.push(self.advance());
        }
        Ok(true)
    }

    fn check_no_letter(&self, loc: &Loc) -> Result<()> {
        if Self::is_ident_part(self.ch()) {
            return self.error(loc.clone(), "invalid literal number");
        }
        Ok(())
    }

    fn escape(&mut self, out: &mut String) {
        // `\\` -> `\`, `\"` -> `"`, any other `\c` -> `c`.
        let c = self.advance();
        out.push(c);
    }

    fn string(&mut self, loc: Loc) -> Result<()> {
        self.advance(); // opening quote
        if self.ch() == '"' {
            self.advance();
            if self.ch() == '"' {
                self.advance();
                return self.multi_string(loc);
            }
            self.push(Tok::LitString(String::new()), loc);
            return Ok(());
        }
        let mut out = String::new();
        loop {
            if self.at_end() || self.ch() == '\n' {
                return self.error(loc, "missing string double quote closure");
            }
            match self.ch() {
                '\\' => {
                    self.advance();
                    self.escape(&mut out);
                }
                '"' => {
                    self.advance();
                    self.push(Tok::LitString(out), loc);
                    return Ok(());
                }
                _ => out.push(self.advance()),
            }
        }
    }

    /// `"""..."""`: the initial newline is dropped, the indentation of the
    /// first content line is removed from every line.
    fn multi_string(&mut self, loc: Loc) -> Result<()> {
        if self.ch() == '\n' {
            self.advance();
        }
        let mut indent = 0usize;
        while self.ch() == ' ' {
            self.advance();
            indent += 1;
        }
        let mut out = String::new();
        let mut skip = indent; // spaces still to strip on the current line
        let mut at_line_start = true;
        loop {
            if self.at_end() {
                return self.error(loc, "missing string triple quote closure");
            }
            match self.ch() {
                ' ' => {
                    if at_line_start && skip > 0 {
                        skip -= 1;
                    } else {
                        out.push(' ');
                    }
                    self.advance();
                }
                '\n' => {
                    out.push('\n');
                    self.advance();
                    skip = indent;
                    at_line_start = true;
                }
                '"' => {
                    self.advance();
                    if self.ch() == '"' {
                        self.advance();
                        if self.ch() == '"' {
                            self.advance();
                            self.push(Tok::LitString(out), loc);
                            return Ok(());
                        }
                        out.push_str("\"\"");
                    } else {
                        out.push('"');
                    }
                    at_line_start = false;
                }
                '\\' => {
                    self.advance();
                    self.escape(&mut out);
                    at_line_start = false;
                }
                _ => {
                    out.push(self.advance());
                    at_line_start = false;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn toks(src: &str) -> Vec<Tok> {
        lex(Rc::new(PathBuf::from("t.fpp")), src, None)
            .unwrap()
            .into_iter()
            .map(|t| t.tok)
            .collect()
    }

    #[test]
    fn keywords_identifiers_and_escapes() {
        assert_eq!(
            toks("module $module x_1"),
            vec![
                Tok::Module,
                Tok::Ident("module".into()),
                Tok::Ident("x_1".into()),
                Tok::Eof
            ]
        );
    }

    #[test]
    fn numbers() {
        assert_eq!(
            toks("0 12 0x1F 1.5 .25 1e3 2E-2"),
            vec![
                Tok::LitInt("0".into()),
                Tok::LitInt("12".into()),
                Tok::LitInt("0x1F".into()),
                Tok::LitFloat("1.5".into()),
                Tok::LitFloat("0.25".into()),
                Tok::LitFloat("1e3".into()),
                Tok::LitFloat("2E-2".into()),
                Tok::Eof
            ]
        );
        // A trailing letter is not a separate token (reference behavior).
        assert!(lex(Rc::new(PathBuf::from("t.fpp")), "3e", None).is_err());
        assert!(lex(Rc::new(PathBuf::from("t.fpp")), "0x", None).is_err());
    }

    #[test]
    fn newline_discipline() {
        // Newlines after `=` and `{` are swallowed; before `}` too; a run of
        // newlines is one EOL.
        assert_eq!(
            toks("a =\n 1\n\n# c\nb { x\n}\n"),
            vec![
                Tok::Ident("a".into()),
                Tok::Equals,
                Tok::LitInt("1".into()),
                Tok::Eol,
                Tok::Ident("b".into()),
                Tok::LBrace,
                Tok::Ident("x".into()),
                Tok::RBrace,
                Tok::Eof
            ]
        );
    }

    #[test]
    fn annotations_swallow_following_newlines() {
        assert_eq!(
            toks("@ pre\nx @< post \n\ny"),
            vec![
                Tok::PreAnnotation("pre".into()),
                Tok::Ident("x".into()),
                Tok::PostAnnotation("post".into()),
                Tok::Ident("y".into()),
                Tok::Eof
            ]
        );
    }

    #[test]
    fn strings_single_and_multi() {
        assert_eq!(
            toks(
                r#""a\"b\\c" "" """
              line 1
                line 2
              """"#
            ),
            vec![
                Tok::LitString("a\"b\\c".into()),
                Tok::LitString(String::new()),
                Tok::LitString("line 1\n  line 2\n".into()),
                Tok::Eof
            ]
        );
    }

    #[test]
    fn line_continuation_and_arrow() {
        assert_eq!(
            toks("a \\\n -> b << c >> d"),
            vec![
                Tok::Ident("a".into()),
                Tok::RArrow,
                Tok::Ident("b".into()),
                Tok::LShift,
                Tok::Ident("c".into()),
                Tok::RShift,
                Tok::Ident("d".into()),
                Tok::Eof
            ]
        );
    }

    #[test]
    fn tab_is_an_error() {
        let err = lex(Rc::new(PathBuf::from("t.fpp")), "a\tb", None).unwrap_err();
        assert_eq!(err.phase, Phase::Lexer);
        assert!(err.message.contains("tab"));
    }
}
