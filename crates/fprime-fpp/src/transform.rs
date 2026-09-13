//! AST transforms applied after parsing and before analysis.
//!
//! The only one is the reference compiler's *state enum*: every state
//! machine with a member sequence implicitly contains `enum State` whose
//! constants are `__FPRIME_UNINITIALIZED` followed by one constant per
//! leaf state, named by the state's qualified name with `.` replaced by
//! `_`, in lexical order. Its representation type is the smallest unsigned
//! integer type that holds one plus the number of leaf states.

use crate::ast::*;
use crate::error::Loc;

/// Add the implicit `State` enum to every state machine in a translation
/// unit (module-level and component-level).
pub fn add_state_enums(tu: &mut TransUnit, next_id: &mut NodeId) {
    for m in &mut tu.members {
        module_member(&mut m.node, next_id);
    }
}

fn module_member(m: &mut ModuleMemberNode, next_id: &mut NodeId) {
    match m {
        ModuleMemberNode::DefModule(node) => {
            for mm in &mut node.data.members {
                module_member(&mut mm.node, next_id);
            }
        }
        ModuleMemberNode::DefComponent(node) => {
            for cm in &mut node.data.members {
                if let ComponentMemberNode::DefStateMachine(sm) = &mut cm.node {
                    state_machine(sm, next_id);
                }
            }
        }
        ModuleMemberNode::DefStateMachine(sm) => state_machine(sm, next_id),
        _ => {}
    }
}

fn state_machine(sm: &mut Node<DefStateMachine>, next_id: &mut NodeId) {
    let Some(members) = &mut sm.data.members else {
        return;
    };
    if members
        .iter()
        .any(|m| matches!(&m.node, StateMachineMemberNode::DefEnum(e) if e.data.name == "State"))
    {
        return;
    }
    let mut leaves: Vec<String> = Vec::new();
    for m in members.iter() {
        if let StateMachineMemberNode::DefState(s) = &m.node {
            collect_leaves(s, &mut Vec::new(), &mut leaves);
        }
    }
    let loc = sm.loc.clone();
    let mut mk = |data: DefEnumConstant, loc: &Loc| -> Annotated<Node<DefEnumConstant>> {
        let id = *next_id;
        *next_id += 1;
        Annotated {
            pre: Vec::new(),
            node: Node {
                id,
                loc: loc.clone(),
                data,
            },
            post: Vec::new(),
        }
    };
    let mut constants = vec![mk(
        DefEnumConstant {
            name: "__FPRIME_UNINITIALIZED".into(),
            value: None,
        },
        &loc,
    )];
    for leaf in &leaves {
        constants.push(mk(
            DefEnumConstant {
                name: leaf.clone(),
                value: None,
            },
            &loc,
        ));
    }
    let n = leaves.len() as u64 + 1;
    let repr = if n <= u64::from(u8::MAX) {
        IntKind::U8
    } else if n <= u64::from(u16::MAX) {
        IntKind::U16
    } else {
        IntKind::U32
    };
    let tn_id = *next_id;
    *next_id += 1;
    let enum_id = *next_id;
    *next_id += 1;
    let def = DefEnum {
        name: "State".into(),
        type_name: Some(Node {
            id: tn_id,
            loc: loc.clone(),
            data: TypeName::Int(repr),
        }),
        constants,
        default: None,
        is_dictionary: false,
    };
    members.push(Annotated {
        pre: vec!["The state enum: the leaf states of the state machine".into()],
        node: StateMachineMemberNode::DefEnum(Node {
            id: enum_id,
            loc,
            data: def,
        }),
        post: Vec::new(),
    });
}

fn collect_leaves(state: &Node<DefState>, path: &mut Vec<String>, out: &mut Vec<String>) {
    path.push(state.data.name.clone());
    let children: Vec<&Node<DefState>> = state
        .data
        .members
        .iter()
        .filter_map(|m| match &m.node {
            StateMemberNode::DefState(s) => Some(s),
            _ => None,
        })
        .collect();
    if children.is_empty() {
        out.push(path.join("_"));
    } else {
        for c in children {
            collect_leaves(c, path, out);
        }
    }
    path.pop();
}
