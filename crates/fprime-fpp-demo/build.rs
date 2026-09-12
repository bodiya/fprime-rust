//! Runs `fpp-to-rust` over `fpp/` at build time. The framework files under
//! `fpp/framework` are imported for name resolution (they are bound to the
//! hand-written framework crates); the model under `fpp/demo` is generated
//! into `$OUT_DIR/generated.rs`, which `src/lib.rs` includes.

use std::path::{Path, PathBuf};

fn fpp_files(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", dir.display()))
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "fpp"))
        .collect();
    out.sort();
    out
}

fn main() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fpp");
    println!("cargo:rerun-if-changed={}", root.display());
    let framework = fpp_files(&root.join("framework"));
    let demo = fpp_files(&root.join("demo"));
    for f in framework.iter().chain(demo.iter()) {
        println!("cargo:rerun-if-changed={}", f.display());
    }

    let mut session = fprime_fpp::Session::new();
    for f in framework.iter().chain(demo.iter()) {
        if let Err(e) = session.parse_file(f) {
            panic!("{e}");
        }
    }
    let analysis = match fprime_fpp::analysis::analyze(&session) {
        Ok(a) => a,
        Err(e) => panic!("{e}"),
    };
    let options = fprime_fpp::codegen::Options {
        targets: demo,
        ..Default::default()
    };
    let code = match fprime_fpp::codegen::generate(&analysis, &options) {
        Ok(c) => c,
        Err(e) => panic!("{e}"),
    };
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR")).join("generated.rs");
    std::fs::write(&out, code).expect("write generated.rs");
}
