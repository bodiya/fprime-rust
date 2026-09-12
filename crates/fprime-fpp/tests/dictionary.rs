//! The JSON dictionary back end against the reference compiler's
//! `fpp-to-dict` test corpus (`tests/dict`, copied from nasa/fpp): every
//! generated dictionary must be structurally identical to the reference
//! `.ref.json`, and every error case must be rejected.

use fprime_fpp::codegen::dictionary::{DictFile, DictOptions, generate_dictionaries};
use fprime_fpp::json::Json;
use std::path::{Path, PathBuf};

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dict")
}

/// Run the generator over `imports` + `file` with the reference's `-p`,
/// `-f` and `-l` values.
fn run(imports: &[&str], file: &str, libs: &[&str]) -> Result<Vec<DictFile>, String> {
    let mut session = fprime_fpp::Session::new();
    for f in imports.iter().chain(std::iter::once(&file)) {
        session
            .parse_file(&dir().join(f))
            .map_err(|e| e.to_string())?;
    }
    let analysis = fprime_fpp::analysis::analyze(&session).map_err(|e| e.to_string())?;
    let opts = DictOptions {
        project_version: "1.0.0".into(),
        framework_version: "3.4.3".into(),
        library_versions: libs.iter().map(|s| s.to_string()).collect(),
        targets: vec![dir().join(file)],
    };
    generate_dictionaries(&analysis, &opts).map_err(|e| e.to_string())
}

/// Canonical form for comparison: object members sorted by key, arrays of
/// objects sorted by their identifying member, integral floats as ints.
fn normalize(j: &Json) -> Json {
    match j {
        Json::Float(f) if f.fract() == 0.0 && f.abs() < 1e15 => Json::Int(*f as i128),
        Json::Arr(elts) => {
            let mut out: Vec<Json> = elts.iter().map(normalize).collect();
            if out.iter().all(|e| matches!(e, Json::Obj(_))) {
                out.sort_by_key(|e| {
                    ["qualifiedName", "name", "id", "opcode"]
                        .iter()
                        .map(|k| format!("{:?}", e.get(k)))
                        .collect::<Vec<_>>()
                });
            }
            Json::Arr(out)
        }
        Json::Obj(members) => {
            let mut out: Vec<(String, Json)> = members
                .iter()
                .map(|(k, v)| (k.clone(), normalize(v)))
                .collect();
            out.sort_by(|a, b| a.0.cmp(&b.0));
            Json::Obj(out)
        }
        other => other.clone(),
    }
}

/// The first difference between two normalized values, as a path.
fn diff(path: &str, a: &Json, b: &Json) -> Option<String> {
    match (a, b) {
        (Json::Arr(x), Json::Arr(y)) => {
            if x.len() != y.len() {
                return Some(format!(
                    "{path}: array length {} vs {}\n  got: {}\n  expected: {}",
                    x.len(),
                    y.len(),
                    a.to_pretty(),
                    b.to_pretty()
                ));
            }
            x.iter()
                .zip(y)
                .enumerate()
                .find_map(|(i, (p, q))| diff(&format!("{path}[{i}]"), p, q))
        }
        (Json::Obj(x), Json::Obj(y)) => {
            for (k, v) in x {
                match b.get(k) {
                    Some(w) => {
                        if let Some(d) = diff(&format!("{path}.{k}"), v, w) {
                            return Some(d);
                        }
                    }
                    None => {
                        return Some(format!("{path}.{k}: unexpected member {}", v.to_pretty()));
                    }
                }
            }
            for (k, _) in y {
                if a.get(k).is_none() {
                    return Some(format!("{path}.{k}: missing member"));
                }
            }
            None
        }
        _ => {
            if a == b {
                None
            } else {
                Some(format!(
                    "{path}: got {} expected {}",
                    a.to_pretty().trim(),
                    b.to_pretty().trim()
                ))
            }
        }
    }
}

fn check(dicts: &[DictFile], name: &str) {
    let d = dicts
        .iter()
        .find(|d| d.name == format!("{name}.json"))
        .unwrap_or_else(|| {
            panic!(
                "no dictionary {name}; generated: {:?}",
                dicts.iter().map(|d| &d.name).collect::<Vec<_>>()
            )
        });
    let ref_text = std::fs::read_to_string(dir().join(format!("{name}.ref.json"))).unwrap();
    let expected = normalize(&Json::parse(&ref_text).unwrap());
    // Round-trip through the printer and parser too.
    let got = normalize(&Json::parse(&d.json.to_pretty()).unwrap());
    if let Some(d) = diff(name, &got, &expected) {
        panic!("{name} differs from the reference:\n{d}");
    }
}

#[test]
fn basic() {
    let dicts = run(&["config.fpp"], "basic.fpp", &[]).unwrap();
    assert_eq!(dicts.len(), 2);
    check(&dicts, "BasicTopologyDictionary");
    check(&dicts, "M_BasicSystemDictionary");
}

#[test]
fn data_products() {
    let dicts = run(&["builtin.fpp", "config.fpp"], "dataProducts.fpp", &[]).unwrap();
    check(&dicts, "BasicDpTopologyDictionary");
    check(&dicts, "FppTest_BasicDpSystemDictionary");
}

#[test]
fn dictionary_defs() {
    let dicts = run(
        &["builtin.fpp", "config.fpp"],
        "dictionaryDefs.fpp",
        &["lib1-1.0.0", "lib2-2.0.0"],
    )
    .unwrap();
    check(&dicts, "DictionaryDefsTopologyDictionary");
    check(&dicts, "DictionaryDefsSystemDictionary");
}

#[test]
fn multiple_tops() {
    let dicts = run(
        &["builtin.fpp", "config.fpp"],
        "multipleTops.fpp",
        &["lib1-1.0.0", "lib2-2.0.0"],
    )
    .unwrap();
    check(&dicts, "FirstTopTopologyDictionary");
    check(&dicts, "SecondTopTopologyDictionary");
}

#[test]
fn unqualified_component_instances() {
    let dicts = run(
        &["builtin.fpp", "config.fpp"],
        "unqualifiedComponentInstances.fpp",
        &["lib1-1.0.0", "lib2-2.0.0"],
    )
    .unwrap();
    check(&dicts, "QualifiedCompInstTopologyDictionary");
    check(&dicts, "UnqualifiedCompInstTopologyDictionary");
    check(&dicts, "UnqualifiedCompInstSystemDictionary");
}

#[test]
fn errors() {
    let cases: &[(&[&str], &str, &str)] = &[
        (
            &["config.fpp"],
            "duplicate.fpp",
            "duplicate JSON file DuplicateTopologyDictionary.json",
        ),
        (
            &["builtin.fpp", "config.fpp"],
            "invalidDictDefConstant.fpp",
            "dictionary constant must have a numeric, Boolean, string, or enum type",
        ),
        (
            &["builtin.fpp", "config.fpp"],
            "invalidDictDefType.fpp",
            "dictionary type is not displayable",
        ),
        (
            &[],
            "missingFwFixedLengthStringSizeConstant.fpp",
            "symbol FW_FIXED_LENGTH_STRING_SIZE is not defined",
        ),
        (
            &[],
            "missingFwOpcodeType.fpp",
            "symbol FwOpcodeType is not defined",
        ),
        (
            &[],
            "missingUserDataSizeConstant.fpp",
            "symbol Fw.DpCfg.CONTAINER_USER_DATA_SIZE is not defined",
        ),
        (&[], "userDataSizeAsMember.fpp", "DpCfg"),
    ];
    for (imports, file, expected) in cases {
        let err = run(imports, file, &[]).expect_err(file);
        assert!(
            err.contains(expected),
            "{file}: expected an error containing {expected:?}, got:\n{err}"
        );
    }
}
