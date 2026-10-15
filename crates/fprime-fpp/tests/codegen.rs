//! The Rust back end on small inline models: serial ports (adapters,
//! typed <-> serial wiring, the cases it refuses) and telemetry packet sets
//! (table order and the dictionary's use/omit checks). The generated code
//! itself is compiled and run by `fprime-fpp-demo`.

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
type FwIdType = U32
type FwChanIdType = FwIdType
type FwSizeType = U64
"#;

fn generate(src: &str) -> Result<String, String> {
    let mut session = Session::new();
    session
        .parse_str(Path::new("framework.fpp"), FRAMEWORK)
        .expect("framework parses");
    session
        .parse_str(Path::new("model.fpp"), src)
        .expect("model parses");
    let a = analysis::analyze(&session).map_err(|e| e.message)?;
    let options = Options {
        targets: vec!["model.fpp".into()],
        ..Default::default()
    };
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
