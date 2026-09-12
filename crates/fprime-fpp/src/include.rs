//! Include resolution: `include "file"` specifiers are replaced by the
//! members of the included file, parsed with locations that chain back to
//! the specifier. Cycles are an error, and an include is resolved relative
//! to the directory of the file containing it (reference behavior).

use crate::ast::*;
use crate::error::{Diagnostic, Loc, Phase, Result};
use crate::lexer;
use crate::parser;
use std::path::{Path, PathBuf};
use std::rc::Rc;

/// Parse `text` (the contents of `path`) and splice every `include`.
pub fn parse_with_includes(
    path: &Path,
    text: &str,
    including: Option<Rc<Loc>>,
    next_id: &mut NodeId,
) -> Result<TransUnit> {
    let file = Rc::new(path.to_path_buf());
    let toks = lexer::lex(Rc::clone(&file), text, including)?;
    let mut tu = parser::parse_trans_unit(toks, next_id)?;
    let mut stack = vec![canonical(path)];
    tu.members = resolve_module_members(tu.members, path, &mut stack, next_id)?;
    Ok(tu)
}

fn canonical(path: &Path) -> PathBuf {
    std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf())
}

/// Read and parse an included file, returning its members.
fn load_included<T>(
    spec: &Node<SpecInclude>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
    parse_members: impl FnOnce(&mut Vec<lexer::Token>, &mut NodeId) -> Result<Vec<T>>,
    resolve: impl FnOnce(Vec<T>, &Path, &mut Vec<PathBuf>, &mut NodeId) -> Result<Vec<T>>,
) -> Result<Vec<T>> {
    let dir = from.parent().unwrap_or_else(|| Path::new("."));
    let target = dir.join(&spec.data.file.data);
    let canon = canonical(&target);
    if stack.contains(&canon) {
        return Err(Diagnostic::new(
            Phase::Include,
            spec.loc.clone(),
            format!("include cycle at {}", target.display()),
        ));
    }
    let text = std::fs::read_to_string(&target).map_err(|e| {
        Diagnostic::new(
            Phase::Include,
            spec.loc.clone(),
            format!("cannot open file {}: {e}", target.display()),
        )
    })?;
    let including = Some(Rc::new(spec.loc.clone()));
    let mut toks = lexer::lex(Rc::new(target.clone()), &text, including)?;
    let members = parse_members(&mut toks, next_id)?;
    stack.push(canon);
    let members = resolve(members, &target, stack, next_id)?;
    stack.pop();
    Ok(members)
}

fn resolve_module_members(
    members: Vec<ModuleMember>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
) -> Result<Vec<ModuleMember>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        match m.node {
            ModuleMemberNode::SpecInclude(spec) => {
                let included = load_included(
                    &spec,
                    from,
                    stack,
                    next_id,
                    |toks, id| {
                        let toks = std::mem::take(toks);
                        Ok(parser::parse_trans_unit(toks, id)?.members)
                    },
                    resolve_module_members,
                )?;
                out.extend(included);
            }
            ModuleMemberNode::DefModule(mut node) => {
                node.data.members =
                    resolve_module_members(node.data.members, from, stack, next_id)?;
                out.push(Annotated {
                    pre: m.pre,
                    node: ModuleMemberNode::DefModule(node),
                    post: m.post,
                });
            }
            ModuleMemberNode::DefComponent(mut node) => {
                node.data.members =
                    resolve_component_members(node.data.members, from, stack, next_id)?;
                out.push(Annotated {
                    pre: m.pre,
                    node: ModuleMemberNode::DefComponent(node),
                    post: m.post,
                });
            }
            ModuleMemberNode::DefTopology(mut node) => {
                node.data.members =
                    resolve_topology_members(node.data.members, from, stack, next_id)?;
                out.push(Annotated {
                    pre: m.pre,
                    node: ModuleMemberNode::DefTopology(node),
                    post: m.post,
                });
            }
            ModuleMemberNode::DefStateMachine(mut node) => {
                if let Some(members) = node.data.members.take() {
                    node.data.members = Some(resolve_sm_members(members, from, stack, next_id)?);
                }
                out.push(Annotated {
                    pre: m.pre,
                    node: ModuleMemberNode::DefStateMachine(node),
                    post: m.post,
                });
            }
            other => out.push(Annotated {
                pre: m.pre,
                node: other,
                post: m.post,
            }),
        }
    }
    Ok(out)
}

fn resolve_component_members(
    members: Vec<ComponentMember>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
) -> Result<Vec<ComponentMember>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        match m.node {
            ComponentMemberNode::SpecInclude(spec) => {
                let included = load_included(
                    &spec,
                    from,
                    stack,
                    next_id,
                    |toks, id| parser::parse_component_members(std::mem::take(toks), id),
                    resolve_component_members,
                )?;
                out.extend(included);
            }
            ComponentMemberNode::DefStateMachine(mut node) => {
                if let Some(members) = node.data.members.take() {
                    node.data.members = Some(resolve_sm_members(members, from, stack, next_id)?);
                }
                out.push(Annotated {
                    pre: m.pre,
                    node: ComponentMemberNode::DefStateMachine(node),
                    post: m.post,
                });
            }
            other => out.push(Annotated {
                pre: m.pre,
                node: other,
                post: m.post,
            }),
        }
    }
    Ok(out)
}

fn resolve_topology_members(
    members: Vec<TopologyMember>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
) -> Result<Vec<TopologyMember>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        match m.node {
            TopologyMemberNode::SpecInclude(spec) => {
                let included = load_included(
                    &spec,
                    from,
                    stack,
                    next_id,
                    |toks, id| parser::parse_topology_members(std::mem::take(toks), id),
                    resolve_topology_members,
                )?;
                out.extend(included);
            }
            TopologyMemberNode::SpecTlmPacketSet(mut node) => {
                node.data.members =
                    resolve_packet_set_members(node.data.members, from, stack, next_id)?;
                out.push(Annotated {
                    pre: m.pre,
                    node: TopologyMemberNode::SpecTlmPacketSet(node),
                    post: m.post,
                });
            }
            other => out.push(Annotated {
                pre: m.pre,
                node: other,
                post: m.post,
            }),
        }
    }
    Ok(out)
}

fn resolve_packet_set_members(
    members: Vec<TlmPacketSetMember>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
) -> Result<Vec<TlmPacketSetMember>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        match m.node {
            TlmPacketSetMemberNode::SpecInclude(spec) => {
                let included = load_included(
                    &spec,
                    from,
                    stack,
                    next_id,
                    |toks, id| parser::parse_tlm_packet_set_members(std::mem::take(toks), id),
                    resolve_packet_set_members,
                )?;
                out.extend(included);
            }
            TlmPacketSetMemberNode::SpecTlmPacket(mut node) => {
                node.data.members =
                    resolve_packet_members(node.data.members, from, stack, next_id)?;
                out.push(Annotated {
                    pre: m.pre,
                    node: TlmPacketSetMemberNode::SpecTlmPacket(node),
                    post: m.post,
                });
            }
        }
    }
    Ok(out)
}

fn resolve_packet_members(
    members: Vec<TlmPacketMember>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
) -> Result<Vec<TlmPacketMember>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        match m {
            TlmPacketMember::SpecInclude(spec) => {
                let included = load_included(
                    &spec,
                    from,
                    stack,
                    next_id,
                    |toks, id| parser::parse_tlm_packet_members(std::mem::take(toks), id),
                    resolve_packet_members,
                )?;
                out.extend(included);
            }
            other => out.push(other),
        }
    }
    Ok(out)
}

fn resolve_sm_members(
    members: Vec<StateMachineMember>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
) -> Result<Vec<StateMachineMember>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        match m.node {
            StateMachineMemberNode::SpecInclude(spec) => {
                let included = load_included(
                    &spec,
                    from,
                    stack,
                    next_id,
                    |toks, id| parser::parse_state_machine_members(std::mem::take(toks), id),
                    resolve_sm_members,
                )?;
                out.extend(included);
            }
            StateMachineMemberNode::DefState(mut node) => {
                node.data.members = resolve_state_members(node.data.members, from, stack, next_id)?;
                out.push(Annotated {
                    pre: m.pre,
                    node: StateMachineMemberNode::DefState(node),
                    post: m.post,
                });
            }
            other => out.push(Annotated {
                pre: m.pre,
                node: other,
                post: m.post,
            }),
        }
    }
    Ok(out)
}

fn resolve_state_members(
    members: Vec<StateMember>,
    from: &Path,
    stack: &mut Vec<PathBuf>,
    next_id: &mut NodeId,
) -> Result<Vec<StateMember>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        match m.node {
            StateMemberNode::SpecInclude(spec) => {
                let included = load_included(
                    &spec,
                    from,
                    stack,
                    next_id,
                    |toks, id| parser::parse_state_members(std::mem::take(toks), id),
                    resolve_state_members,
                )?;
                out.extend(included);
            }
            StateMemberNode::DefState(mut node) => {
                node.data.members = resolve_state_members(node.data.members, from, stack, next_id)?;
                out.push(Annotated {
                    pre: m.pre,
                    node: StateMemberNode::DefState(node),
                    post: m.post,
                });
            }
            other => out.push(Annotated {
                pre: m.pre,
                node: other,
                post: m.post,
            }),
        }
    }
    Ok(out)
}
