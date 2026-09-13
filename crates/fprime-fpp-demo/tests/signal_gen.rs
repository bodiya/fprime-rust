//! The upstream `Ref.SignalGen` model, generated from its unmodified FPP:
//! the dictionary the reference compiler assigns, and that the generated
//! base constructs and registers.

use fprime_fpp_demo::generated::Ref::{self, SignalGenBase};

#[test]
fn upstream_signal_gen_generates_with_the_reference_dictionary() {
    // Commands.fppi: SETTINGS, TOGGLE, SKIP, DP in declaration order.
    assert_eq!(SignalGenBase::OPCODE_SETTINGS, 0);
    assert_eq!(SignalGenBase::OPCODE_TOGGLE, 1);
    assert_eq!(SignalGenBase::OPCODE_SKIP, 2);
    assert_eq!(SignalGenBase::OPCODE_DP, 3);
    assert_eq!(SignalGenBase::EVENTID_SETTINGS_CHANGED, 0);
    assert_eq!(SignalGenBase::CHANID_TYPE, 0);
    assert_eq!(SignalGenBase::CONTAINER_ID_DATA_CONTAINER, 0);
    assert_eq!(SignalGenBase::CONTAINER_PRIORITY_DATA_CONTAINER, 10);
    assert_eq!(SignalGenBase::RECORD_ID_DATA_RECORD, 0);
    // The nested enum lives in the component's module.
    assert_eq!(Ref::SignalGen::DpReqType::IMMEDIATE.as_repr(), 0i32);
    let base = SignalGenBase::new("sg");
    base.set_id_base(0x100);
    assert_eq!(base.id_base(), 0x100);
    assert!(!base.product_get_out.is_connected());
    // SignalInfo = type (I32 repr: 4) + history (4 f32) + pairHistory (4 * 2 f32).
    assert_eq!(Ref::SignalInfo::SERIALIZED_SIZE, 4 + 16 + 32);
}
