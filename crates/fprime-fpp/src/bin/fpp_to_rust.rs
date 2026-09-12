//! `fpp-to-rust` — the command-line front end.
//!
//! ```text
//! fpp-to-rust [MODE] [OPTIONS] FILE...
//!
//! Modes:
//!   --syntax              parse only
//!   --check               parse and analyze
//!   (default)             parse, analyze and generate Rust
//!
//! Options:
//!   -i FILE               import FILE for name resolution (not generated)
//!   -o FILE               write generated Rust to FILE (default stdout)
//!   --impl-prefix PATH    Rust path prefix for instance implementation
//!                         types (default `crate::`)
//!   --bind-type FPP=RUST[:copy|ref|buf|owned]
//!                         use an existing Rust type for an FPP type
//!   --bind-port FPP=RUST  use an existing Rust trait for an FPP port
//!   --no-framework-bindings
//!                         start from an empty binding table
//! ```

use fprime_fpp::codegen::{self, ArgKind, Bindings, Options};
use std::path::PathBuf;
use std::process::ExitCode;

fn usage() -> ExitCode {
    eprintln!(
        "usage: fpp-to-rust [--syntax | --check] [-i FILE]... [-o FILE] [--impl-prefix PATH] \
         [--bind-type FPP=RUST[:kind]]... [--bind-port FPP=RUST]... [--no-framework-bindings] FILE..."
    );
    ExitCode::FAILURE
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut files = Vec::new();
    let mut imports = Vec::new();
    let mut mode = "gen";
    let mut output: Option<PathBuf> = None;
    let mut options = Options::default();
    let mut i = 0;
    let take = |i: &mut usize| -> Option<String> {
        *i += 1;
        args.get(*i).cloned()
    };
    while i < args.len() {
        match args[i].as_str() {
            "--syntax" => mode = "syntax",
            "--check" => mode = "check",
            "-i" => match take(&mut i) {
                Some(f) => imports.push(PathBuf::from(f)),
                None => return usage(),
            },
            "-o" => match take(&mut i) {
                Some(f) => output = Some(PathBuf::from(f)),
                None => return usage(),
            },
            "--impl-prefix" => match take(&mut i) {
                Some(p) => options.impl_prefix = p,
                None => return usage(),
            },
            "--no-framework-bindings" => options.bindings = Bindings::empty(),
            "--bind-type" => {
                let Some(spec) = take(&mut i) else {
                    return usage();
                };
                let Some((fpp, rest)) = spec.split_once('=') else {
                    return usage();
                };
                let (rust, kind) = match rest.rsplit_once(':') {
                    Some((r, k)) if !r.ends_with(':') => (
                        r.to_string(),
                        match k {
                            "copy" => ArgKind::Copy,
                            "ref" => ArgKind::Ref,
                            "buf" => ArgKind::Buf,
                            "owned" => ArgKind::Owned,
                            _ => return usage(),
                        },
                    ),
                    _ => (rest.to_string(), ArgKind::Ref),
                };
                options.bindings.bind_type(fpp, &rust, kind);
            }
            "--bind-port" => {
                let Some(spec) = take(&mut i) else {
                    return usage();
                };
                let Some((fpp, rust)) = spec.split_once('=') else {
                    return usage();
                };
                options.bindings.bind_port(fpp, rust);
            }
            "-h" | "--help" => return usage(),
            a if a.starts_with('-') => return usage(),
            a => files.push(PathBuf::from(a)),
        }
        i += 1;
    }
    if files.is_empty() && imports.is_empty() {
        return usage();
    }
    let mut session = fprime_fpp::Session::new();
    for f in imports.iter().chain(files.iter()) {
        if let Err(e) = session.parse_file(f) {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    }
    if mode == "syntax" {
        return ExitCode::SUCCESS;
    }
    let analysis = match fprime_fpp::analysis::analyze(&session) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    if mode == "check" {
        eprintln!(
            "ok: {} symbols, {} components, {} instances, {} topologies",
            analysis.symbols.symbols.len(),
            analysis.components.len(),
            analysis.instances.len(),
            analysis.topologies.len()
        );
        return ExitCode::SUCCESS;
    }
    options.targets = files.clone();
    let code = match codegen::generate(&analysis, &options) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    match output {
        Some(path) => {
            if let Err(e) = std::fs::write(&path, code) {
                eprintln!("cannot write {}: {e}", path.display());
                return ExitCode::FAILURE;
            }
        }
        None => print!("{code}"),
    }
    ExitCode::SUCCESS
}
