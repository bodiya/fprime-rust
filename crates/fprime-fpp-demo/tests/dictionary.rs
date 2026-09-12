//! The demo's ground dictionary (written by `build.rs` next to the
//! generated code) agrees with the generated code: every command, event,
//! channel, parameter, container and record of the `sensor` instance is
//! listed under its global id, with the instance's base id applied.

use fprime_fpp::json::Json;
use fprime_fpp_demo::generated::Demo::SensorBase;

const BASE_ID: i128 = 0x1000;

fn dictionary() -> Json {
    let text = std::fs::read_to_string(concat!(env!("OUT_DIR"), "/DemoTopologyDictionary.json"))
        .expect("build.rs writes the dictionary");
    Json::parse(&text).expect("valid JSON")
}

/// The numeric `key` of the entry named `name` in the array `section`.
fn id_of(d: &Json, section: &str, name: &str, key: &str) -> i128 {
    let entries = d.get(section).and_then(Json::as_arr).expect(section);
    let entry = entries
        .iter()
        .find(|e| e.get("name").and_then(Json::as_str) == Some(name))
        .unwrap_or_else(|| panic!("{section} has no entry {name}"));
    match entry.get(key) {
        Some(Json::Int(i)) => *i,
        other => panic!("{name}.{key} is {other:?}"),
    }
}

#[test]
fn metadata_and_sections() {
    let d = dictionary();
    let m = d.get("metadata").expect("metadata");
    assert_eq!(
        m.get("deploymentName").and_then(Json::as_str),
        Some("Demo.Demo")
    );
    assert_eq!(
        m.get("dictionarySpecVersion").and_then(Json::as_str),
        Some("1.0.0")
    );
    for section in [
        "typeDefinitions",
        "constants",
        "commands",
        "parameters",
        "events",
        "telemetryChannels",
        "records",
        "containers",
        "telemetryPacketSets",
    ] {
        assert!(d.get(section).and_then(Json::as_arr).is_some(), "{section}");
    }
    // The types the sensor's interface uses are defined.
    let types: Vec<&str> = d
        .get("typeDefinitions")
        .and_then(Json::as_arr)
        .unwrap()
        .iter()
        .filter_map(|t| t.get("qualifiedName").and_then(Json::as_str))
        .collect();
    for t in [
        "Demo.Mode",
        "Demo.Reading",
        "Demo.Counts",
        "Demo.Ident",
        "FwOpcodeType",
    ] {
        assert!(types.contains(&t), "type {t} missing from {types:?}");
    }
}

#[test]
fn ids_match_the_generated_code() {
    let d = dictionary();
    let cmd = |n: &str| id_of(&d, "commands", &format!("Demo.sensor.{n}"), "opcode");
    assert_eq!(
        cmd("CONFIGURE"),
        BASE_ID + i128::from(SensorBase::OPCODE_CONFIGURE)
    );
    assert_eq!(cmd("PING"), BASE_ID + i128::from(SensorBase::OPCODE_PING));
    assert_eq!(cmd("RESET"), BASE_ID + i128::from(SensorBase::OPCODE_RESET));
    assert_eq!(
        cmd("GAIN_PRM_SET"),
        BASE_ID + i128::from(SensorBase::OPCODE_GAIN_PARAM_SET)
    );
    assert_eq!(
        cmd("GAIN_PRM_SAVE"),
        BASE_ID + i128::from(SensorBase::OPCODE_GAIN_PARAM_SAVE)
    );
    assert_eq!(
        cmd("THRESHOLD_PRM_SET"),
        BASE_ID + i128::from(SensorBase::OPCODE_THRESHOLD_PARAM_SET)
    );
    assert_eq!(
        cmd("THRESHOLD_PRM_SAVE"),
        BASE_ID + i128::from(SensorBase::OPCODE_THRESHOLD_PARAM_SAVE)
    );

    let ev = |n: &str| id_of(&d, "events", &format!("Demo.sensor.{n}"), "id");
    assert_eq!(
        ev("Configured"),
        BASE_ID + i128::from(SensorBase::EVENTID_CONFIGURED)
    );
    assert_eq!(
        ev("Overrun"),
        BASE_ID + i128::from(SensorBase::EVENTID_OVERRUN)
    );
    assert_eq!(
        ev("Labelled"),
        BASE_ID + i128::from(SensorBase::EVENTID_LABELLED)
    );

    let ch = |n: &str| id_of(&d, "telemetryChannels", &format!("Demo.sensor.{n}"), "id");
    assert_eq!(ch("Value"), BASE_ID + i128::from(SensorBase::CHANID_VALUE));
    assert_eq!(ch("Mode"), BASE_ID + i128::from(SensorBase::CHANID_MODE));
    assert_eq!(ch("Label"), BASE_ID + i128::from(SensorBase::CHANID_LABEL));

    let prm = |n: &str| id_of(&d, "parameters", &format!("Demo.sensor.{n}"), "id");
    assert_eq!(prm("Gain"), BASE_ID + i128::from(SensorBase::PARAMID_GAIN));
    assert_eq!(
        prm("Threshold"),
        BASE_ID + i128::from(SensorBase::PARAMID_THRESHOLD)
    );

    assert_eq!(
        id_of(&d, "containers", "Demo.sensor.Samples", "id"),
        BASE_ID + i128::from(SensorBase::CONTAINER_ID_SAMPLES)
    );
    assert_eq!(
        id_of(&d, "records", "Demo.sensor.Sample", "id"),
        BASE_ID + i128::from(SensorBase::RECORD_ID_SAMPLE)
    );
    assert_eq!(
        id_of(&d, "records", "Demo.sensor.Raw", "id"),
        BASE_ID + i128::from(SensorBase::RECORD_ID_RAW)
    );
}
