//! Runs `fpp-to-rust` over `fpp/` at build time. The framework files under
//! `fpp/framework` are imported for name resolution (they are bound to the
//! hand-written framework crates); the model under `fpp/demo` is generated
//! into `$OUT_DIR/generated.rs`, which `src/lib.rs` includes, and its
//! ground dictionaries into `$OUT_DIR/<Topology>TopologyDictionary.json`.
//!
//! The state machine models under `fpp/sm` (upstream FppTest, FpySequencer
//! and ComAggregator machines, copied unchanged, plus the `Probe` machine
//! of fprime-comp's trace tests) are generated separately into
//! `$OUT_DIR/sm_models.rs` (`src/lib.rs`: `sm_models`).

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

/// Every `.fpp` file under `dir`, recursively, sorted.
fn fpp_files_deep(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut work = vec![dir.to_path_buf()];
    while let Some(d) = work.pop() {
        for e in std::fs::read_dir(&d)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", d.display()))
            .filter_map(|e| e.ok())
        {
            let p = e.path();
            if p.is_dir() {
                work.push(p);
            } else if p.extension().is_some_and(|x| x == "fpp" || x == "fppi") {
                out.push(p);
            }
        }
    }
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
    let mut options = fprime_fpp::codegen::Options {
        targets: demo.clone(),
        include_path: Some("generated".into()),
        ..Default::default()
    };
    // The controller's external state machine is hand-written.
    options.bindings.bind_type(
        "Demo.Watchdog",
        "crate::controller::Watchdog",
        fprime_fpp::codegen::ArgKind::Ref,
    );
    let code = match fprime_fpp::codegen::generate(&analysis, &options) {
        Ok(c) => c,
        Err(e) => panic!("{e}"),
    };
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
    std::fs::write(out_dir.join("generated.rs"), code).expect("write generated.rs");

    // The ground dictionary of the deployment (what fprime-gds reads).
    let dict_options = fprime_fpp::codegen::dictionary::DictOptions {
        project_version: env!("CARGO_PKG_VERSION").into(),
        framework_version: "fprime-rust".into(),
        library_versions: Vec::new(),
        targets: demo,
    };
    let dicts =
        match fprime_fpp::codegen::dictionary::generate_dictionaries(&analysis, &dict_options) {
            Ok(d) => d,
            Err(e) => panic!("{e}"),
        };
    for d in dicts {
        std::fs::write(out_dir.join(&d.name), d.json.to_pretty()).expect("write dictionary");
    }

    // The state machine models.
    let sm_all = fpp_files_deep(&root.join("sm"));
    for f in &sm_all {
        println!("cargo:rerun-if-changed={}", f.display());
    }
    let sm: Vec<PathBuf> = sm_all
        .into_iter()
        .filter(|p| p.extension().is_some_and(|x| x == "fpp"))
        .collect();
    let mut session = fprime_fpp::Session::new();
    for f in framework.iter().chain(sm.iter()) {
        if let Err(e) = session.parse_file(f) {
            panic!("{e}");
        }
    }
    let analysis = match fprime_fpp::analysis::analyze(&session) {
        Ok(a) => a,
        Err(e) => panic!("{e}"),
    };
    let mut options = fprime_fpp::codegen::Options {
        targets: sm,
        include_path: Some("sm_models".into()),
        ..Default::default()
    };
    let b = &mut options.bindings;
    // The FppTest harness's abstract type and the two external machines
    // are hand-written (`src/sm_support.rs`).
    b.bind_type(
        "FppTest.SmHarness.TestAbsType",
        "crate::sm_support::TestAbsType",
        fprime_fpp::codegen::ArgKind::Ref,
    );
    b.bind_type(
        "FppTest.DeviceSm",
        "crate::sm_support::DeviceSm",
        fprime_fpp::codegen::ArgKind::Ref,
    );
    b.bind_type(
        "FppTest.HackSm",
        "crate::sm_support::HackSm",
        fprime_fpp::codegen::ArgKind::Ref,
    );
    let code = match fprime_fpp::codegen::generate(&analysis, &options) {
        Ok(c) => c,
        Err(e) => panic!("{e}"),
    };
    std::fs::write(out_dir.join("sm_models.rs"), code).expect("write sm_models.rs");
}
