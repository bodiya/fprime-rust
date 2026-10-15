//! The Rust back end on small inline models: serial ports (adapters,
//! typed <-> serial wiring, the cases it refuses) and telemetry packet sets
//! (table order and the dictionary's use/omit checks), state machines and
//! state machine instances, hand-written components in a topology, and
//! instance init specifiers. The generated code itself is compiled and run
//! by `fprime-fpp-demo`.

use fprime_fpp::Session;
use fprime_fpp::analysis;
use fprime_fpp::codegen::{self, Options};
use std::path::Path;

const FRAMEWORK: &str = r#"
module Fw {
  type Buffer
  type Time
  type TlmBuffer
  type ComBuffer
  port Tlm($id: FwChanIdType, ref timeTag: Fw.Time, ref val: TlmBuffer)
  port Time(ref $time: Fw.Time)
  port Com(ref data: ComBuffer, context: U32)
}
module Svc {
  port Sched(context: U32)
}
type FwIdType = U32
type FwChanIdType = FwIdType
type FwSizeType = U64
"#;

fn generate(src: &str) -> Result<String, String> {
    generate_with(src, |_| {})
}

fn generate_with(src: &str, f: impl FnOnce(&mut Options)) -> Result<String, String> {
    let mut session = Session::new();
    session
        .parse_str(Path::new("framework.fpp"), FRAMEWORK)
        .expect("framework parses");
    session
        .parse_str(Path::new("model.fpp"), src)
        .expect("model parses");
    let a = analysis::analyze(&session).map_err(|e| e.message)?;
    let mut options = Options {
        targets: vec!["model.fpp".into()],
        ..Default::default()
    };
    f(&mut options);
    codegen::generate(&a, &options).map_err(|e| e.message)
}

const COMPONENTS: &str = r#"
  port P(x: U32, s: string size 4)
  port B(ref b: Fw.Buffer)
  passive component Hub {
    sync input port si: [3] serial
    output port so: [3] serial
  }
  passive component User {
    output port po: P
    sync input port pi: P
    output port bo: B
    sync input port bi: B
    output port co: Fw.Com
    sync input port ci: Fw.Com
  }
  instance hub: Hub base id 0x100
  instance user: User base id 0x200
"#;

#[test]
fn typed_serial_connections_go_through_the_port_adapters() {
    let src = format!(
        "module M {{ {COMPONENTS}
          topology T {{
            instance hub
            instance user
            connections C {{
              user.po -> hub.si[0]
              hub.so[0] -> user.pi
              user.co -> hub.si[1]
              hub.so[1] -> user.ci
              hub.so[2] -> hub.si[2]
            }}
          }}
        }}"
    );
    let code = generate(&src).expect("generates");
    // A generated port is made serializable next to its trait.
    assert!(code.contains("::fprime_comp::serializable_port! { port: PPort; args { val x: u32, ref s: ::fprime_fw::FwString<4> } }"));
    // Not a port carrying an Fw.Buffer.
    assert!(!code.contains("port: BPort;"));
    // Typed -> serial and serial -> typed wiring, generated and framework
    // port types alike; serial -> serial is a plain connection.
    assert!(
        code.contains(
            ".po.connect_serial(<crate::M::Hub as self::HubComponent>::si(&self.hub, 0));"
        )
    );
    assert!(code.contains(
        ".so[0].connect_typed(<crate::M::User as self::UserComponent>::pi(&self.user, 0));"
    ));
    assert!(code.contains(".co.connect_serial("));
    assert!(code.contains(".so[1].connect_typed("));
    assert!(
        code.contains(
            ".so[2].connect_to(<crate::M::Hub as self::HubComponent>::si(&self.hub, 2));"
        )
    );
    // Serial ports in the component.
    assert!(
        code.contains("pub so: [::fprime_comp::OutputPort<dyn ::fprime_comp::SerializePort>; 3],")
    );
    assert!(code.contains("fn si_handler(&self, port_num: ::fprime_config::FwIndexType, buffer: &mut dyn ::fprime_fw::SerBufAny);"));
    assert!(code.contains("pub fn so_out(&self, port_num: ::fprime_config::FwIndexType, buffer: &mut dyn ::fprime_fw::SerBufAny) -> ::fprime_fw::SerializeStatus {"));
}

#[test]
fn a_buffer_port_cannot_cross_a_serial_connection() {
    let src = format!(
        "module M {{ {COMPONENTS}
          topology T {{
            instance hub
            instance user
            connections C {{ user.bo -> hub.si[0] }}
          }}
        }}"
    );
    let err = generate(&src).expect_err("refused");
    assert!(err.contains("cannot connect a serial port here"), "{err}");
    assert!(err.contains("Fw.Buffer"), "{err}");
}

const SENSORS: &str = r#"
  passive component S {
    time get port timeGetOut
    telemetry port tlmOut
    telemetry A: U32
    telemetry B: F32
    telemetry C: string size 6
  }
  instance s1: S base id 0x100
  instance s2: S base id 0x200
"#;

#[test]
fn packet_tables_are_ordered_like_the_reference() {
    let src = format!(
        "module M {{ {SENSORS}
          topology T {{
            instance s1
            instance s2
            telemetry packets P {{
              packet Zeta id 0 group 1 {{ s1.B
                s1.A }}
              packet Alpha id 1 group 2 {{ s2.A }}
            }} omit {{ s2.C
              s1.C
              s2.B }}
          }}
        }}"
    );
    let code = generate(&src).expect("generates");
    assert!(code.contains("pub mod T_PTlmPackets {"));
    // Packets by name, channels in declaration order.
    assert!(code.contains(
        "pub const PACKET_LIST: &[TlmPacketizerPacket<'static>] = &[Packets::Alpha, Packets::Zeta];"
    ));
    let zeta = code
        .find("pub const Zeta: [TlmPacketizerChannelEntry; 2]")
        .unwrap();
    let b = code[zeta..].find("M_s1_B").unwrap();
    let a = code[zeta..].find("M_s1_A").unwrap();
    assert!(b < a);
    // Omitted channels by qualified name: M.s1.C, M.s2.B, M.s2.C.
    let omitted = code.find("pub const OMITTED_CHANNELS").unwrap();
    let rest = &code[omitted..];
    let (p1, p2, p3) = (
        rest.find("ChannelIds::M_s1_C").unwrap(),
        rest.find("ChannelIds::M_s2_B").unwrap(),
        rest.find("ChannelIds::M_s2_C").unwrap(),
    );
    assert!(p1 < p2 && p2 < p3);
    assert!(code.contains("pub const M_s2_A: ::fprime_config::FwChanIdType = 0x200;"));
    assert!(code.contains("pub const Alpha: ::fprime_config::FwChanIdType = 2;"));
}

#[test]
fn packet_sets_are_checked_like_the_dictionary() {
    let set = |body: &str| {
        format!(
            "module M {{ {SENSORS}
              topology T {{
                instance s1
                instance s2
                telemetry packets P {{ {body} }}
              }}
            }}"
        )
    };
    let err = generate(&set("packet X group 0 { s1.A }")).expect_err("refused");
    assert!(
        err.contains(
            "telemetry channel M.s1.B is neither used nor marked as omitted in packet set P"
        ),
        "{err}"
    );
    let err = generate(&set(
        "packet X group 0 { s1.A\n s1.B\n s1.C\n s2.A\n s2.B\n s2.C\n s1.A }",
    ))
    .expect_err("refused");
    assert!(
        err.contains("telemetry channel M.s1.A is used more than once in packet set P"),
        "{err}"
    );
    let src = format!(
        "module M {{ {SENSORS}
          topology T {{
            instance s1
            instance s2
            telemetry packets P {{ packet X group 0 {{ s1.A\n s1.B\n s1.C\n s2.A\n s2.B\n s2.C }} }} omit {{ s1.A }}
          }}
        }}"
    );
    let err = generate(&src).expect_err("refused");
    assert!(
        err.contains("telemetry channel M.s1.A is both used and marked omitted in packet set P"),
        "{err}"
    );
}

/// Whitespace-insensitive containment.
fn has(code: &str, needle: &str) -> bool {
    let squash = |s: &str| s.split_whitespace().collect::<Vec<_>>().join(" ");
    squash(code).contains(&squash(needle))
}

const MACHINES: &str = r#"
  state machine M {
    action a: U32
    action b
    guard g: U32
    signal zz: U16
    signal aa: U32
    signal s: string size 4
    initial do { b } enter S
    state S {
      initial enter T
      entry do { b }
      state T {
        on zz enter C
        on aa if g do { a } enter C
        on s do { b }
      }
      choice C { if g do { a } enter T else enter U }
      state U
    }
  }
  state machine E
  queued component Q {
    sync input port tick: Svc.Sched
    state machine instance zeta: M priority 3 hook
    state machine instance alpha: E priority 1 drop
    state machine instance mid: M
  }
"#;

#[test]
fn state_machines_become_fpp_state_machine_invocations() {
    let code = generate_with(MACHINES, |o| {
        o.bindings
            .bind_type("E", "crate::Ext", codegen::ArgKind::Ref)
    })
    .unwrap();
    assert!(has(&code, "pub mod M { // ---- state machine M ----"));
    assert!(has(
        &code,
        "::fprime_comp::fpp_state_machine! { state machine M: u8 {"
    ));
    assert!(has(
        &code,
        "actions { a => action_a (val u32), b => action_b, }"
    ));
    assert!(has(&code, "guards { g => guard_g (val u32), }"));
    // Signals numbered by name; strings passed as `str`.
    assert!(has(
        &code,
        "aa = 1 => send_signal_aa (val u32) { S_T => if g do { a } enter S_C; }"
    ));
    assert!(has(
        &code,
        "s = 2 => send_signal_s (str ::fprime_fw::FwString<4>) { S_T => do { b }; }"
    ));
    assert!(has(
        &code,
        "zz = 3 => send_signal_zz (val u16) { S_T => enter S_C; }"
    ));
    assert!(has(&code, "initial do { b } enter S;"));
    // Leaves numbered by flattened name; composites carry their initial.
    assert!(has(&code, "state S { initial enter S_T; entry do { b } }"));
    assert!(has(&code, "state S_T = 1 in S {}"));
    assert!(has(&code, "state S_U = 2 in S {}"));
    // The choice is typed with the common type of U16 and U32.
    assert!(has(
        &code,
        "choice S_C in S (val u32) { if g do { a } enter S_T else enter S_U }"
    ));
    // The implicit State enum is the macro's, not emitted separately.
    assert!(!code.contains("pub enum State"));
}

#[test]
fn state_machine_instances_follow_the_generator_contract() {
    let code = generate_with(MACHINES, |o| {
        o.bindings
            .bind_type("E", "crate::Ext", codegen::ArgKind::Ref)
    })
    .unwrap();
    // External before internal, after every other message type.
    assert!(has(
        &code,
        "pub const MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL: ::fprime_config::FwEnumStoreType = 1;"
    ));
    assert!(has(
        &code,
        "pub const MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL: ::fprime_config::FwEnumStoreType = 2;"
    ));
    assert!(has(&code, "M::StateMachine::SIGNAL_MSG_SIZE"));
    assert!(has(&code, "::fprime_comp::sm::EXTERNAL_SIGNAL_MSG_SIZE"));
    // SmId: every instance sorted by name.
    assert!(has(
        &code,
        "impl QBase { /// Instance `alpha`. SM_ID_ALPHA, /// Instance `mid`. SM_ID_MID, /// Instance `zeta`. SM_ID_ZETA, }"
    ));
    // init: machines in SmId order, then the queue.
    assert!(has(
        &code,
        "<crate::Ext as ::fprime_comp::sm::ExternalStateMachine>::init(&fpp_base.alpha, self.__alpha_sm_parent(), QBase::SM_ID_ALPHA); fpp_base.mid.init(QBase::SM_ID_MID, &fpp_h); fpp_base.zeta.init(QBase::SM_ID_ZETA, &fpp_h); fpp_base.init(queue_depth);"
    ));
    // Senders with the instance's priority and policy (and hook).
    assert!(has(&code, "fn zeta_send_signal_s(&self, value: &str)"));
    assert!(has(
        &code,
        "QBase::SM_ID_ZETA, 3 as ::fprime_config::FwQueuePriorityType, ::fprime_comp::QueueFullPolicy::Hook).with_overflow_hook(&fpp_hook);"
    ));
    assert!(has(
        &code,
        "QBase::SM_ID_MID, 0 as ::fprime_config::FwQueuePriorityType, ::fprime_comp::QueueFullPolicy::Assert);"
    ));
    assert!(has(
        &code,
        "fn alpha_state_machine_invoke(&self, signal: <crate::Ext as ::fprime_comp::sm::ExternalStateMachine>::Signal, data: &::fprime_fw::SmSignalBuffer)"
    ));
    // Handlers: the machine's actions under its qualified name, once.
    assert_eq!(code.matches("fn M_action_a(").count(), 1);
    assert!(has(
        &code,
        "fn M_guard_g(&self, sm_id: ::fprime_config::FwEnumStoreType, signal: self::M::Signal, value: u32) -> bool;"
    ));
    assert!(has(
        &code,
        "fn zeta_state_machine_overflow_hook(&self, sm_id: ::fprime_config::FwEnumStoreType, signal: ::fprime_config::FwEnumStoreType, msg: &mut dyn ::fprime_fw::SerBufAny);"
    ));
    assert!(!code.contains("fn mid_state_machine_overflow_hook"));
    // The external parent comes from the impl macro.
    assert!(has(
        &code,
        "pub trait QComponent: QHandlers + QSmParents + Sized"
    ));
    assert!(has(
        &code,
        "fn __alpha_sm_parent(&self) -> &<crate::Ext as ::fprime_comp::sm::ExternalStateMachine>::Parent { self }"
    ));
    // Dispatch: internal ids assert, external ids are errors.
    assert!(has(
        &code,
        "QBase::SM_ID_MID => fpp_base.mid.dispatch_signal(&fpp_h, fpp_signal, fpp_msg),"
    ));
    assert!(has(
        &code,
        "_ => ::fprime_fw::fw_assert!(false, fpp_sm_id),"
    ));
    assert!(has(&code, "_ => ::fprime_comp::MsgDispatchStatus::Error,"));
}

#[test]
fn an_external_state_machine_needs_a_binding() {
    let err = generate(MACHINES).unwrap_err();
    assert!(
        err.contains("external state machine E has no Rust binding"),
        "{err}"
    );
}

#[test]
fn bound_components_and_init_specifiers_in_a_topology() {
    let src = r#"
      module Fpp { module ToCpp { enum Phases {
        configConstants, configObjects, instances, initComponents, configComponents,
        regCommands, readParameters, loadParameters, startTasks, stopTasks, freeThreads,
        tearDownComponents, deinitComponents
      } } }
      module Svc {
        active component Hand {
          async input port Run: Svc.Sched
          output port PktSend: [2] Fw.Com
        }
      }
      passive component Sink {
        sync input port comIn: Fw.Com
        output port runOut: Svc.Sched
      }
      instance hand: Svc.Hand base id 0x100 queue size 4 {
        phase Fpp.ToCpp.Phases.configConstants """
        pub const LEVEL: u32 = 2;
        """
        phase Fpp.ToCpp.Phases.configComponents """
        self.hand.configure(T_ConfigConstants::hand::LEVEL);
        """
        phase Fpp.ToCpp.Phases.stopTasks "self.hand.stop_now();"
      }
      instance sink: Sink base id 0x200 type "crate::Sink" {
        phase Fpp.ToCpp.Phases.instances "crate::Sink::special()"
      }
      topology T {
        instance hand
        instance sink
        connections C {
          sink.runOut -> hand.Run
          hand.PktSend[1] -> sink.comIn
        }
      }
    "#;
    let code = generate_with(src, |o| {
        o.bindings.bind_component("Svc.Hand", "::hand::Hand")
    })
    .unwrap();
    // The hand-written component is not generated ...
    assert!(!code.contains("pub struct HandBase"));
    // ... and is used through its own conventions.
    assert!(has(&code, "pub hand: ::std::sync::Arc<::hand::Hand>,"));
    assert!(has(&code, "hand: ::hand::Hand::new(\"hand\"),"));
    assert!(has(
        &code,
        "self.hand.active.queued.base.set_id_base(0x100);"
    ));
    assert!(has(&code, "self.hand.init(4);"));
    assert!(has(
        &code,
        ".run_out.connect_to(::hand::Hand::run_in(&self.hand, 0));"
    ));
    assert!(has(&code, "self.hand.pkt_send[1].connect_to("));
    // Init specifiers: instances, configComponents, configConstants, and a
    // replaced default (stopTasks).
    assert!(has(&code, "sink: { crate::Sink::special() },"));
    assert!(has(
        &code,
        "pub fn config_components(&self) { // hand (init specifier) { self.hand.configure(T_ConfigConstants::hand::LEVEL); } }"
    ));
    assert!(has(
        &code,
        "pub mod T_ConfigConstants { /// Instance `hand`. pub mod hand { use super::super::*; pub const LEVEL: u32 = 2; } }"
    ));
    assert!(has(
        &code,
        "pub fn exit_tasks(&self) { // hand (init specifier) { self.hand.stop_now(); } }"
    ));
    assert!(has(
        &code,
        "pub fn join_tasks(&self) { let _ = self.hand.active.join(); }"
    ));
    assert!(has(
        &code,
        "pub fn setup(&self) { self.init(); self.config_components(); self.set_id_bases(); self.connect(); self.reg_commands(); self.read_parameters(); self.load_parameters(); self.start_tasks(); }"
    ));
}
