//! Source locations and diagnostics.
//!
//! Every AST node carries a [`Loc`]; every failure in the front end or the
//! generator is a [`Diagnostic`] pointing at one. Locations of included
//! files chain back to the `include` specifier that pulled them in, exactly
//! as the reference compiler reports them.

use std::fmt;
use std::path::PathBuf;
use std::rc::Rc;

/// A position in a source file (1-based line and column).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Loc {
    /// The file the position is in.
    pub file: Rc<PathBuf>,
    /// 1-based line.
    pub line: u32,
    /// 1-based column.
    pub col: u32,
    /// The location of the `include` specifier that brought this file in,
    /// if any.
    pub including: Option<Rc<Loc>>,
}

impl Loc {
    /// A location with no file (used for synthesized nodes and tests).
    pub fn none() -> Self {
        Self {
            file: Rc::new(PathBuf::new()),
            line: 0,
            col: 0,
            including: None,
        }
    }

    /// Whether this is [`Loc::none`].
    pub fn is_none(&self) -> bool {
        self.line == 0
    }
}

impl fmt::Display for Loc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_none() {
            return write!(f, "<unknown>");
        }
        write!(f, "{}:{}:{}", self.file.display(), self.line, self.col)?;
        if let Some(inc) = &self.including {
            write!(f, "\n  included at {inc}")?;
        }
        Ok(())
    }
}

/// The phase a diagnostic came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Lexing.
    Lexer,
    /// Parsing.
    Syntax,
    /// Include resolution / file I/O.
    Include,
    /// Semantic analysis.
    Semantic,
    /// Code generation.
    Codegen,
}

impl fmt::Display for Phase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Phase::Lexer => "lexical error",
            Phase::Syntax => "syntax error",
            Phase::Include => "include error",
            Phase::Semantic => "semantic error",
            Phase::Codegen => "code generation error",
        })
    }
}

/// A single error with a location and a message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Diagnostic {
    /// Which phase produced it.
    pub phase: Phase,
    /// Where.
    pub loc: Loc,
    /// What.
    pub message: String,
    /// Optional secondary locations with a note each.
    pub notes: Vec<(Loc, String)>,
}

impl Diagnostic {
    /// Build a diagnostic.
    pub fn new(phase: Phase, loc: Loc, message: impl Into<String>) -> Self {
        Self {
            phase,
            loc,
            message: message.into(),
            notes: Vec::new(),
        }
    }

    /// Attach a note at another location.
    #[must_use]
    pub fn with_note(mut self, loc: Loc, note: impl Into<String>) -> Self {
        self.notes.push((loc, note.into()));
        self
    }

    /// A semantic error.
    pub fn semantic(loc: Loc, message: impl Into<String>) -> Self {
        Self::new(Phase::Semantic, loc, message)
    }

    /// A code generation error.
    pub fn codegen(loc: Loc, message: impl Into<String>) -> Self {
        Self::new(Phase::Codegen, loc, message)
    }
}

impl fmt::Display for Diagnostic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}\n{}: {}", self.loc, self.phase, self.message)?;
        for (loc, note) in &self.notes {
            write!(f, "\n{loc}\nnote: {note}")?;
        }
        Ok(())
    }
}

impl std::error::Error for Diagnostic {}

/// The crate-wide result type.
pub type Result<T> = std::result::Result<T, Diagnostic>;
