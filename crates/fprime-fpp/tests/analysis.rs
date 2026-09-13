//! Semantic analysis against the reference compiler's rules, on small
//! inline models: implicit numbering, parameter opcodes, enum values,
//! defaults and conversions, interface imports, topology port numbering
//! and pattern expansion.

use fprime_fpp::Session;
use fprime_fpp::analysis::{self, Analysis, NameGroup, PortInstance, Type, Value};
use fprime_fpp::ast::{CommandKind, GeneralPortKind, SpecialPortKind};
use std::path::Path;

/// The framework ports the patterns need, in their real shapes.
const FRAMEWORK: &str = r#"
module Fw {
  type CmdArgBuffer
  type LogBuffer
  type Time
  type TlmBuffer
  type ParamBuffer
  type TextLogString
  type Buffer
  type StatementArgBuffer
  enum CmdResponse : U8 { OK = 0, INVALID_OPCODE = 1 }
  enum LogSeverity : U8 { FATAL = 1, WARNING_HI = 2 }
  enum ParamValid : U8 { UNINIT = 0, VALID = 1 }
  enum Success : U8 { FAILURE, SUCCESS }
  port CmdReg(opCode: FwOpcodeType)
  port Cmd(opCode: FwOpcodeType, cmdSeq: U32, ref args: CmdArgBuffer)
  port CmdResponse(opCode: FwOpcodeType, cmdSeq: U32, response: CmdResponse)
  port Log($id: FwEventIdType, ref timeTag: Fw.Time, $severity: LogSeverity, ref args: LogBuffer)
  port LogText($id: FwEventIdType, ref timeTag: Fw.Time, $severity: LogSeverity, ref $text: Fw.TextLogString)
  port Tlm($id: FwChanIdType, ref timeTag: Fw.Time, ref val: TlmBuffer)
  port Time(ref $time: Fw.Time)
  port PrmGet($id: FwPrmIdType, ref val: ParamBuffer) -> ParamValid
  port PrmSet($id: FwPrmIdType, ref val: ParamBuffer)
  port DpGet($id: FwDpIdType, dataSize: FwSizeType, ref buffer: Fw.Buffer) -> Fw.Success
  port DpSend($id: FwDpIdType, buffer: Fw.Buffer)
}
module Svc {
  port Sched(context: U32)
  port Ping(key: U32)
}
type FwIdType = U32
type FwOpcodeType = FwIdType
type FwEventIdType = FwIdType
type FwChanIdType = FwIdType
type FwPrmIdType = FwIdType
type FwDpIdType = FwIdType
type FwSizeType = U64
"#;

fn analyze<'a>(session: &'a mut Session, sources: &[(&str, &str)]) -> Analysis<'a> {
    session
        .parse_str(Path::new("framework.fpp"), FRAMEWORK)
        .expect("framework parses");
    for (name, src) in sources {
        session
            .parse_str(Path::new(name), src)
            .expect("source parses");
    }
    analysis::analyze(session).unwrap_or_else(|e| panic!("analysis failed: {e}"))
}

fn analyze_err(sources: &[(&str, &str)]) -> String {
    let mut session = Session::new();
    session
        .parse_str(Path::new("framework.fpp"), FRAMEWORK)
        .expect("framework parses");
    for (name, src) in sources {
        session
            .parse_str(Path::new(name), src)
            .expect("source parses");
    }
    match analysis::analyze(&session) {
        Ok(_) => panic!("expected an error"),
        Err(e) => e.message,
    }
}

fn component<'a, 'b>(a: &'b Analysis<'a>, qualified: &str) -> &'b analysis::ComponentModel {
    let parts: Vec<&str> = qualified.split('.').collect();
    let sym = a
        .symbols
        .resolve_absolute(NameGroup::Component, &parts)
        .unwrap_or_else(|| panic!("no component {qualified}"));
    &a.components[&sym]
}

fn constant(a: &mut Analysis<'_>, qualified: &str) -> Value {
    let parts: Vec<&str> = qualified.split('.').collect();
    let sym = a
        .symbols
        .resolve_absolute(NameGroup::Value, &parts)
        .unwrap_or_else(|| panic!("no constant {qualified}"));
    a.value_of_sym(sym, &fprime_fpp::Loc::none()).unwrap()
}

#[test]
fn implicit_opcodes_and_ids_follow_the_reference_counters() {
    let src = r#"
      module M {
        enum E { A, B }
        active component C {
          command recv port cmdIn
          command reg port cmdRegOut
          command resp port cmdResponseOut
          event port eventOut
          time get port timeGetOut
          telemetry port tlmOut
          param get port prmGetOut
          param set port prmSetOut
          async command A
          async command B opcode 0x10
          async command C
          param P1: U32
          param P2: F32 set opcode 0x20
          param P3: E save opcode 0x30
          async command D
          event E1 severity activity low format "e1"
          event E2 severity activity low id 5 format "e2"
          event E3 severity activity low format "e3"
          telemetry T1: U32
          telemetry T2: U32 id 7
          telemetry T3: U32
        }
      }"#;
    let mut session = Session::new();
    let a = analyze(&mut session, &[("m.fpp", src)]);
    let c = component(&a, "M.C");
    let opcodes: Vec<(String, u64)> = c
        .commands
        .iter()
        .map(|c| (c.name.clone(), c.opcode))
        .collect();
    assert_eq!(
        opcodes,
        vec![
            ("A".into(), 0),
            ("B".into(), 0x10),
            ("C".into(), 0x11),
            ("P1_PARAM_SET".into(), 0x12),
            ("P1_PARAM_SAVE".into(), 0x13),
            ("P2_PARAM_SET".into(), 0x20),
            ("P2_PARAM_SAVE".into(), 0x21),
            ("P3_PARAM_SET".into(), 0x22),
            ("P3_PARAM_SAVE".into(), 0x30),
            ("D".into(), 0x31),
        ]
    );
    let ids: Vec<u64> = c.events.iter().map(|e| e.id).collect();
    assert_eq!(ids, vec![0, 5, 6]);
    let ids: Vec<u64> = c.tlm_channels.iter().map(|t| t.id).collect();
    assert_eq!(ids, vec![0, 7, 8]);
    let ids: Vec<u64> = c.params.iter().map(|p| p.id).collect();
    assert_eq!(ids, vec![0, 1, 2]);
    assert!(c.commands[3].param_cmd == Some((0, true)));
    assert_eq!(c.commands[3].kind, CommandKind::Async);
}

#[test]
fn enum_values_defaults_and_conversions() {
    let src = r#"
      module M {
        enum E : U8 { A, B = 5, C } default B
        constant x = E.C
        constant y = 1 + 2 * 3 - -4 * 5 + 6 << 1
        constant z = (1 + 2) * 3 / 2
        constant s = "a" + "b"
        constant f = 1.5 * 2
        constant arr = [1, 2, 3]
        constant sub = arr[1]
        constant st = { a = 1, b = 2.0 }
        constant member = st.b
        constant sz = sizeof(E) + sizeof(string size 10) + sizeof(F64)
        array A = [3] F32 default 1.0
        struct S { a: U32, b: E, c: A } default { a = 7 }
        constant sdef = S.SERIALIZED_SIZE_PLACEHOLDER
      }"#;
    // The last constant is invalid; drop it.
    let src = src.replace(
        "        constant sdef = S.SERIALIZED_SIZE_PLACEHOLDER\n",
        "",
    );
    let mut session = Session::new();
    let mut a = analyze(&mut session, &[("m.fpp", &src)]);
    let e = a
        .symbols
        .resolve_absolute(NameGroup::Type, &["M", "E"])
        .unwrap();
    let analysis::TypeDef::Enum {
        repr,
        constants,
        default,
    } = &a.type_defs[&e]
    else {
        panic!()
    };
    assert_eq!(*repr, fprime_fpp::ast::IntKind::U8);
    let vals: Vec<(String, i128)> = constants.iter().map(|(n, v, _)| (n.clone(), *v)).collect();
    assert_eq!(
        vals,
        vec![("A".into(), 0), ("B".into(), 5), ("C".into(), 6)]
    );
    assert_eq!(default, "B");
    assert_eq!(constant(&mut a, "M.x"), Value::Enum(e, "C".into(), 6));
    // ((1 + 6) - (-20) + 6) << 1 = 33 << 1
    assert_eq!(constant(&mut a, "M.y"), Value::Int(66, None));
    assert_eq!(constant(&mut a, "M.z"), Value::Int(4, None));
    assert_eq!(constant(&mut a, "M.s"), Value::Str("ab".into()));
    assert_eq!(
        constant(&mut a, "M.f"),
        Value::Float(3.0, fprime_fpp::ast::FloatKind::F64)
    );
    assert_eq!(constant(&mut a, "M.sub"), Value::Int(2, None));
    assert_eq!(
        constant(&mut a, "M.member"),
        Value::Float(2.0, fprime_fpp::ast::FloatKind::F64)
    );
    // 1 + (2 + 10) + 8
    assert_eq!(constant(&mut a, "M.sz"), Value::Int(21, None));
    let s = a
        .symbols
        .resolve_absolute(NameGroup::Type, &["M", "S"])
        .unwrap();
    let arr = a
        .symbols
        .resolve_absolute(NameGroup::Type, &["M", "A"])
        .unwrap();
    let analysis::TypeDef::Struct { members, default } = &a.type_defs[&s] else {
        panic!()
    };
    assert_eq!(members.len(), 3);
    assert_eq!(members[2].ty, Type::Array(arr));
    // a = 7 explicit; b = enum default B; c = array default [1.0; 3].
    let Value::Struct(Some(_), ms) = default else {
        panic!()
    };
    assert_eq!(ms[0].1, Value::Int(7, Some(fprime_fpp::ast::IntKind::U32)));
    assert_eq!(ms[1].1, Value::Enum(e, "B".into(), 5));
    let Value::Array(Some(_), elts) = &ms[2].1 else {
        panic!()
    };
    assert_eq!(elts.len(), 3);
    assert_eq!(elts[0], Value::Float(1.0, fprime_fpp::ast::FloatKind::F32));
}

#[test]
fn errors_are_reported_with_reference_semantics() {
    assert!(analyze_err(&[("m.fpp", "constant x = 1 / 0")]).contains("division by zero"));
    assert!(analyze_err(&[("m.fpp", "constant x = y\nconstant y = x")]).contains("cyclic"));
    assert!(analyze_err(&[("m.fpp", "enum E : U8 { A = 300 }")]).contains("out of range"));
    assert!(analyze_err(&[("m.fpp", "enum E { A = 1, B = 1 }")]).contains("duplicate enum value"));
    assert!(analyze_err(&[("m.fpp", "constant a = 1\nconstant a = 2")]).contains("redefinition"));
    assert!(
        analyze_err(&[(
            "m.fpp",
            "passive component C { async input port p: Svc.Sched }"
        )])
        .contains("passive component may not")
    );
    assert!(
        analyze_err(&[(
            "m.fpp",
            "active component C { sync input port p: Svc.Sched }"
        )])
        .contains("must have at least one async")
    );
    assert!(
        analyze_err(&[(
            "m.fpp",
            "passive component C { sync command X\n command recv port cmdIn }"
        )])
        .contains("no command reg port")
    );
    assert!(analyze_err(&[(
        "m.fpp",
        "passive component C { event port e\n time get port t\n event E(a: U32) severity activity low format \"none\" }"
    )])
    .contains("replacement fields"));
}

#[test]
fn interfaces_import_ports_in_place_and_nested_definitions_resolve() {
    let src = r#"
      module M {
        interface I {
          sync input port a: Svc.Sched
          import J
        }
        interface J {
          output port b: [3] Svc.Ping
        }
        passive component C {
          enum Inner { X = 3 }
          constant K = Inner.X
          sync input port before: Svc.Sched
          import I
          output port after: [3] Svc.Ping
          match after with b
        }
        constant outside = C.K
      }"#;
    let mut session = Session::new();
    let mut a = analyze(&mut session, &[("m.fpp", src)]);
    let c = component(&a, "M.C");
    let names: Vec<&str> = c.ports.iter().map(|p| p.name()).collect();
    assert_eq!(names, vec!["before", "a", "b", "after"]);
    assert!(matches!(
        c.ports[2],
        PortInstance::General {
            size: 3,
            kind: GeneralPortKind::Output,
            ..
        }
    ));
    assert!(matches!(c.ports[3], PortInstance::General { size: 3, .. }));
    let inner = a
        .symbols
        .resolve_absolute(NameGroup::Type, &["M", "C", "Inner"])
        .unwrap();
    assert_eq!(
        constant(&mut a, "M.outside"),
        Value::Enum(inner, "X".into(), 3)
    );
}

#[test]
fn topology_numbering_patterns_and_imports() {
    let src = r#"
      module M {
        passive component Disp {
          sync input port compCmdReg: [4] Fw.CmdReg
          output port compCmdSend: [4] Fw.Cmd
          sync input port compCmdStat: Fw.CmdResponse
          sync input port pingIn: Svc.Ping
          output port pingOut: Svc.Ping
        }
        passive component Health {
          output port pingSend: [4] Svc.Ping
          sync input port pingReturn: [4] Svc.Ping
          match pingSend with pingReturn
        }
        passive component Worker {
          command recv port cmdIn
          command reg port cmdRegOut
          command resp port cmdResponseOut
          sync command GO
          sync input port pingIn: Svc.Ping
          output port pingOut: Svc.Ping
          sync input port dataIn: [2] Svc.Sched
          output port dataOut: [2] Svc.Sched
        }
        instance disp: Disp base id 0x100
        instance $health: Health base id 0x200
        instance w1: Worker base id 0x300
        instance w2: Worker base id 0x400
        topology Inner {
          instance w1
          instance w2
          connections Data {
            w1.dataOut -> w2.dataIn[1]
            w1.dataOut[1] -> w2.dataIn
            w2.dataOut -> w1.dataIn
          }
          port dataTap = w2.dataOut
        }
        topology Outer {
          import Inner
          instance disp
          instance $health
          command connections instance disp
          health connections instance $health
          connections Extra {
            Inner.dataTap -> w1.dataIn
          }
        }
      }"#;
    let mut session = Session::new();
    let a = analyze(&mut session, &[("m.fpp", src)]);
    let outer = a
        .symbols
        .resolve_absolute(NameGroup::PortInterfaceInstance, &["M", "Outer"])
        .unwrap();
    let t = &a.topologies[&outer];
    let name = |s: usize| a.symbols.sym(s).name.clone();
    assert_eq!(
        t.instances.iter().map(|s| name(*s)).collect::<Vec<_>>(),
        vec!["disp", "health", "w1", "w2"]
    );
    let find = |from: &str, fp: &str, to: &str, tp: &str| -> (u64, u64) {
        let c = t
            .connections
            .iter()
            .find(|c| {
                name(c.from.instance) == from
                    && c.from.port == fp
                    && name(c.to.instance) == to
                    && c.to.port == tp
            })
            .unwrap_or_else(|| panic!("no connection {from}.{fp} -> {to}.{tp}"));
        (c.from.num.unwrap(), c.to.num.unwrap())
    };
    // Imported direct connections keep explicit numbers; unnumbered
    // outputs get the lowest free index, unnumbered inputs get 0.
    assert_eq!(find("w1", "dataOut", "w2", "dataIn"), (0, 1));
    assert_eq!(find("w2", "dataOut", "w1", "dataIn"), (0, 0));
    // The topology port resolves to the underlying instance port and
    // takes the next free output index.
    assert_eq!(find("w2", "dataOut", "w1", "dataIn"), (0, 0));
    let taps: Vec<(u64, u64)> = t
        .connections
        .iter()
        .filter(|c| name(c.from.instance) == "w2" && c.from.port == "dataOut")
        .map(|c| (c.from.num.unwrap(), c.to.num.unwrap()))
        .collect();
    assert_eq!(taps, vec![(0, 0), (1, 0)]);
    // Command pattern: each worker gets registration, dispatch and
    // response; dispatcher port indices are assigned in instance order.
    assert_eq!(find("w1", "cmdRegOut", "disp", "compCmdReg"), (0, 0));
    assert_eq!(find("w2", "cmdRegOut", "disp", "compCmdReg"), (0, 0));
    assert_eq!(find("disp", "compCmdSend", "w1", "cmdIn"), (0, 0));
    assert_eq!(find("disp", "compCmdSend", "w2", "cmdIn"), (1, 0));
    assert_eq!(find("w1", "cmdResponseOut", "disp", "compCmdStat"), (0, 0));
    // Health pattern with matched ports: send/return share the index per
    // instance; health does not ping itself.
    let (s1, _) = find("health", "pingSend", "w1", "pingIn");
    let (_, r1) = find("w1", "pingOut", "health", "pingReturn");
    assert_eq!(s1, r1);
    let (s2, _) = find("health", "pingSend", "w2", "pingIn");
    let (_, r2) = find("w2", "pingOut", "health", "pingReturn");
    assert_eq!(s2, r2);
    assert_ne!(s1, s2);
    let (s3, _) = find("health", "pingSend", "disp", "pingIn");
    assert_ne!(s3, s1);
    assert_ne!(s3, s2);
    assert!(
        t.connections
            .iter()
            .all(|c| !(name(c.from.instance) == "health" && name(c.to.instance) == "health"))
    );
    // The special ports of the workers are recognized.
    let w = component(&a, "M.Worker");
    assert!(w.special_port(SpecialPortKind::CommandRecv).is_some());
}

#[test]
fn topology_errors() {
    let base = r#"
      module M {
        passive component A { output port o: [1] Svc.Sched }
        passive component B { sync input port i: Svc.Sched }
        instance a: A base id 1
        instance b: B base id 2
        instance b2: B base id 3
      }"#;
    let too_many = "module M { topology T { instance a\n instance b\n instance b2\n connections C { a.o -> b.i, a.o -> b2.i } } }";
    assert!(
        analyze_err(&[("base.fpp", base), ("t.fpp", too_many)]).contains("too many connections")
    );
    let wrong_dir =
        "module M { topology T { instance a\n instance b\n connections C { b.i -> a.o } } }";
    assert!(
        analyze_err(&[("base.fpp", base), ("t.fpp", wrong_dir)]).contains("not an output port")
    );
    let not_member = "module M { topology T { instance a\n connections C { a.o -> b.i } } }";
    assert!(analyze_err(&[("base.fpp", base), ("t.fpp", not_member)]).contains("not a member"));
}
