//! `fpp-to-rust` — the command-line front end.
//!
//! ```text
//! fpp-to-rust [--syntax | --check] [-i FILE]... FILE...
//! ```
//!
//! `--syntax` only parses; `--check` parses and analyzes. Files given with
//! `-i` are imported for name resolution but not generated. Without a mode
//! flag, Rust is generated (see later options in `--help`).

use std::path::PathBuf;
use std::process::ExitCode;

fn usage() -> ExitCode {
    eprintln!("usage: fpp-to-rust [--syntax | --check] [-i FILE]... FILE...");
    ExitCode::FAILURE
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut files = Vec::new();
    let mut imports = Vec::new();
    let mut mode = "gen";
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--syntax" => mode = "syntax",
            "--check" => mode = "check",
            "-i" => {
                i += 1;
                match args.get(i) {
                    Some(f) => imports.push(PathBuf::from(f)),
                    None => return usage(),
                }
            }
            "-h" | "--help" => return usage(),
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
    eprintln!("code generation is not wired up yet");
    ExitCode::FAILURE
}
