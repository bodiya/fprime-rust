//! # fprime-fpp — an FPP front end and Rust back end for the port
//!
//! [FPP](https://github.com/nasa/fpp) is F Prime's modeling language. This
//! crate is `fpp-to-rust`: a lexer, parser and semantic analysis that
//! mirror the reference compiler production for production, and a code
//! generator that emits Rust against this workspace's framework crates —
//! the declarative macro layer (`fpp_enum!`, `fpp_struct!`, `fpp_array!`)
//! for data types, object-safe port traits, a component base (the
//! equivalent of the C++ `<Comp>ComponentAc` class) with a handler trait
//! for the user's implementation, and topology wiring with the reference
//! compiler's port-numbering and pattern-expansion rules.
//!
//! Zero third-party dependencies, like the rest of the workspace.
//!
//! ## Pipeline
//!
//! ```text
//! .fpp files --lexer--> tokens --parser--> AST --include--> AST (spliced)
//!            --analysis--> Model (symbols, types, values, ids, topology)
//!            --codegen--> Rust source
//! ```
//!
//! See [`Session`] for the driver and `src/bin/fpp_to_rust.rs` for the CLI.

#![forbid(unsafe_code)]

pub mod analysis;
pub mod ast;
pub mod codegen;
pub mod error;
pub mod include;
pub mod json;
pub mod lexer;
pub mod parser;
pub mod transform;

use std::path::{Path, PathBuf};
use std::rc::Rc;

pub use error::{Diagnostic, Loc, Phase, Result};

/// A parsed file with its origin.
#[derive(Debug, Clone)]
pub struct ParsedFile {
    /// The path as given.
    pub path: PathBuf,
    /// The translation unit, with includes already spliced in.
    pub tu: ast::TransUnit,
}

/// The front-end driver: parses files (resolving includes) with one shared
/// node-id space so that the analysis can key side tables by node id.
#[derive(Debug, Default)]
pub struct Session {
    next_id: ast::NodeId,
    /// Files parsed so far, in order.
    pub files: Vec<ParsedFile>,
}

impl Session {
    /// A fresh session.
    pub fn new() -> Self {
        Self::default()
    }

    /// Parse `text` as the contents of `path` (which is used for locations
    /// and for resolving relative `include`s).
    pub fn parse_str(&mut self, path: &Path, text: &str) -> Result<()> {
        let mut tu = include::parse_with_includes(path, text, None, &mut self.next_id)?;
        transform::add_state_enums(&mut tu, &mut self.next_id);
        self.files.push(ParsedFile {
            path: path.to_path_buf(),
            tu,
        });
        Ok(())
    }

    /// Read and parse a file.
    pub fn parse_file(&mut self, path: &Path) -> Result<()> {
        let text = std::fs::read_to_string(path).map_err(|e| {
            Diagnostic::new(
                Phase::Include,
                Loc {
                    file: Rc::new(path.to_path_buf()),
                    line: 1,
                    col: 1,
                    including: None,
                },
                format!("cannot read file {}: {e}", path.display()),
            )
        })?;
        self.parse_str(path, &text)
    }

    /// The next unused node id (useful for tests that synthesize nodes).
    pub fn next_id(&self) -> ast::NodeId {
        self.next_id
    }
}
