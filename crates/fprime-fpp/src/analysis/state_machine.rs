//! State machine semantics: the reference compiler's
//! `CheckStateMachineSemantics` for an internal state machine definition
//! (`EnterStateMachineSymbols`, `CheckStateMachineUses`,
//! `CheckInitialTransitions`, `CheckSignalUses`, `CheckTransitionGraph`,
//! `CheckTypedElements`).
//!
//! The result is a [`StateMachineModel`]: the actions, guards and signals
//! with their types, every state (flattened, with its parent) and choice,
//! every transition with its target resolved, and the type FPP infers for
//! each choice. External state machines (no members) have no model.
//!
//! Names follow the reference's state machine scoping: actions, guards and
//! signals live in the machine's outer scope; each state opens a scope for
//! its substates and choices, and a transition target `A.B` is resolved
//! by looking `A` up from the innermost enclosing state outwards, then
//! selecting `B` inside it.

use super::{Analysis, Def, SymId, Type};
use crate::ast::*;
use crate::error::{Diagnostic, Loc, Result};
use std::collections::{HashMap, HashSet};

/// An action, guard or signal: name and optional type.
#[derive(Debug, Clone)]
pub struct SmTyped {
    /// Name.
    pub name: String,
    /// The type, if the element is typed.
    pub ty: Option<Type>,
    /// Annotation lines.
    pub docs: Vec<String>,
    /// Definition location.
    pub loc: Loc,
}

/// A transition target: an index into [`StateMachineModel::states`] or
/// [`StateMachineModel::choices`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SmTarget {
    /// A state.
    State(usize),
    /// A choice.
    Choice(usize),
}

/// `do { a, .. } enter T`.
#[derive(Debug, Clone)]
pub struct SmTransition {
    /// Action names, in order.
    pub actions: Vec<String>,
    /// The target.
    pub target: SmTarget,
}

/// `on s [if g] [do { .. }] [enter T]` in a state.
#[derive(Debug, Clone)]
pub struct SmStateTransition {
    /// The signal.
    pub signal: String,
    /// The guard, if any.
    pub guard: Option<String>,
    /// Action names, in order.
    pub actions: Vec<String>,
    /// The target; `None` for an internal transition (`on s do { .. }`).
    pub target: Option<SmTarget>,
    /// Location of the specifier.
    pub loc: Loc,
}

/// A state.
#[derive(Debug, Clone)]
pub struct SmState {
    /// Unqualified name.
    pub name: String,
    /// Qualified name inside the machine (`["S1", "S2"]` for `S1.S2`).
    pub path: Vec<String>,
    /// The enclosing state.
    pub parent: Option<usize>,
    /// Direct substates, in definition order.
    pub substates: Vec<usize>,
    /// The initial transition (states with substates).
    pub initial: Option<SmTransition>,
    /// Entry actions.
    pub entry: Vec<String>,
    /// Exit actions.
    pub exit: Vec<String>,
    /// Transitions, in definition order.
    pub transitions: Vec<SmStateTransition>,
    /// Annotation lines.
    pub docs: Vec<String>,
    /// Definition location.
    pub loc: Loc,
}

impl SmState {
    /// Whether the state is a leaf (no substates).
    pub fn is_leaf(&self) -> bool {
        self.substates.is_empty()
    }

    /// The qualified name with `.` replaced by `_` (the C++ `State`
    /// enumerator and the `fpp_state_machine!` state name).
    pub fn flat_name(&self) -> String {
        self.path.join("_")
    }
}

/// A choice.
#[derive(Debug, Clone)]
pub struct SmChoice {
    /// Unqualified name.
    pub name: String,
    /// Qualified name inside the machine.
    pub path: Vec<String>,
    /// The enclosing state.
    pub parent: Option<usize>,
    /// The guard.
    pub guard: String,
    /// The `if` branch.
    pub if_transition: SmTransition,
    /// The `else` branch.
    pub else_transition: SmTransition,
    /// The type FPP infers for the choice (the common type of the
    /// transitions into it), if any.
    pub ty: Option<Type>,
    /// Annotation lines.
    pub docs: Vec<String>,
    /// Definition location.
    pub loc: Loc,
}

impl SmChoice {
    /// The qualified name with `.` replaced by `_`.
    pub fn flat_name(&self) -> String {
        self.path.join("_")
    }
}

/// A checked internal state machine.
#[derive(Debug, Clone)]
pub struct StateMachineModel {
    /// The state machine symbol.
    pub sym: SymId,
    /// Actions, in definition order.
    pub actions: Vec<SmTyped>,
    /// Guards, in definition order.
    pub guards: Vec<SmTyped>,
    /// Signals, in definition order.
    pub signals: Vec<SmTyped>,
    /// The machine's initial transition.
    pub initial: SmTransition,
    /// Every state, in definition order (a parent before its substates).
    pub states: Vec<SmState>,
    /// Every choice, in definition order.
    pub choices: Vec<SmChoice>,
}

impl StateMachineModel {
    /// The leaf states sorted by flattened name, with their `State` enum
    /// discriminants (from 1, as `fpp-to-cpp` numbers them).
    pub fn leaf_numbers(&self) -> HashMap<usize, usize> {
        let mut leaves: Vec<(String, usize)> = self
            .states
            .iter()
            .enumerate()
            .filter(|(_, s)| s.is_leaf())
            .map(|(i, s)| (s.flat_name(), i))
            .collect();
        leaves.sort();
        leaves
            .into_iter()
            .enumerate()
            .map(|(n, (_, i))| (i, n + 1))
            .collect()
    }

    /// The signals sorted by name, with their `Signal` enum discriminants
    /// (from 1).
    pub fn signals_by_name(&self) -> Vec<(usize, &SmTyped)> {
        let mut s: Vec<&SmTyped> = self.signals.iter().collect();
        s.sort_by(|a, b| a.name.cmp(&b.name));
        s.into_iter().enumerate().map(|(i, t)| (i + 1, t)).collect()
    }

    /// A signal by name.
    pub fn signal(&self, name: &str) -> Option<&SmTyped> {
        self.signals.iter().find(|s| s.name == name)
    }

    /// The flattened name of a target.
    pub fn target_name(&self, t: SmTarget) -> String {
        match t {
            SmTarget::State(i) => self.states[i].flat_name(),
            SmTarget::Choice(i) => self.choices[i].flat_name(),
        }
    }
}

/// A typed element of the transition graph (`StateMachineTypedElement`):
/// what carries a value into a transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum TypedElement {
    /// An initial transition (untyped).
    Initial,
    /// A transition of state `.0`, number `.1` (typed by its signal).
    StateTransition(usize, usize),
    /// A choice (typed by inference).
    Choice(usize),
}

/// Where a name is looked up: the machine's outer scope or a state's.
type ScopeKey = Option<usize>;

/// Builder state while checking one machine.
struct Builder<'a, 'b> {
    a: &'b mut Analysis<'a>,
    sym: SymId,
    stack: Vec<super::ScopeId>,
    actions: Vec<SmTyped>,
    guards: Vec<SmTyped>,
    signals: Vec<SmTyped>,
    states: Vec<SmState>,
    choices: Vec<SmChoice>,
    /// AST of each state.
    state_nodes: Vec<&'a Node<DefState>>,
    /// AST of each choice.
    choice_nodes: Vec<&'a Node<DefChoice>>,
    /// Names of states and choices per scope.
    scope: HashMap<(ScopeKey, String), (SmTarget, Loc)>,
}

impl<'a> Analysis<'a> {
    /// Check an internal state machine definition and record its model
    /// (does nothing for an external machine).
    pub fn resolve_state_machine(&mut self, sym: SymId) -> Result<()> {
        if self.state_machines.contains_key(&sym) {
            return Ok(());
        }
        let Def::StateMachine(node) = self.symbols.sym(sym).def else {
            unreachable!("not a state machine")
        };
        let Some(members) = &node.data.members else {
            return Ok(());
        };
        let mut stack = self.symbols.sym(sym).def_stack.clone();
        stack.push(self.symbols.sym(sym).scope.expect("state machine scope"));
        let mut b = Builder {
            a: self,
            sym,
            stack,
            actions: Vec::new(),
            guards: Vec::new(),
            signals: Vec::new(),
            states: Vec::new(),
            choices: Vec::new(),
            state_nodes: Vec::new(),
            choice_nodes: Vec::new(),
            scope: HashMap::new(),
        };
        let model = b.build(node, members)?;
        self.state_machines.insert(sym, model);
        Ok(())
    }
}

impl<'a> Builder<'a, '_> {
    fn build(
        &mut self,
        node: &'a Node<DefStateMachine>,
        members: &'a [StateMachineMember],
    ) -> Result<StateMachineModel> {
        // EnterStateMachineSymbols.
        let mut initials: Vec<&'a Node<SpecInitialTransition>> = Vec::new();
        for m in members {
            let docs: Vec<String> = m.doc_lines().cloned().collect();
            match &m.node {
                StateMachineMemberNode::DefAction(n) => {
                    let ty = self.opt_type(&n.data.type_name)?;
                    Self::enter_typed(&mut self.actions, "action", &n.data.name, ty, docs, &n.loc)?;
                }
                StateMachineMemberNode::DefGuard(n) => {
                    let ty = self.opt_type(&n.data.type_name)?;
                    Self::enter_typed(&mut self.guards, "guard", &n.data.name, ty, docs, &n.loc)?;
                }
                StateMachineMemberNode::DefSignal(n) => {
                    let ty = self.opt_type(&n.data.type_name)?;
                    Self::enter_typed(&mut self.signals, "signal", &n.data.name, ty, docs, &n.loc)?;
                }
                StateMachineMemberNode::DefState(n) => {
                    self.enter_state(n, None, &[], docs)?;
                }
                StateMachineMemberNode::DefChoice(n) => {
                    self.enter_choice(n, None, &[], docs)?;
                }
                StateMachineMemberNode::SpecInitialTransition(n) => initials.push(n),
                // Types and constants are ordinary definitions, resolved
                // with the rest of the model.
                StateMachineMemberNode::DefAbsType(_)
                | StateMachineMemberNode::DefAliasType(_)
                | StateMachineMemberNode::DefArray(_)
                | StateMachineMemberNode::DefConstant(_)
                | StateMachineMemberNode::DefEnum(_)
                | StateMachineMemberNode::DefStruct(_) => {}
                StateMachineMemberNode::SpecInclude(_) => unreachable!("includes are spliced"),
            }
        }

        // CheckStateMachineUses + CheckInitialTransitions (the machine).
        let initial = match initials.as_slice() {
            [one] => self.transition(&one.data.transition, None)?,
            [] => {
                return Err(Diagnostic::semantic(
                    node.loc.clone(),
                    "state machine must have initial transition",
                ));
            }
            more => {
                return Err(Diagnostic::semantic(
                    more[1].loc.clone(),
                    format!(
                        "state machine has {} initial transitions; only one is allowed",
                        more.len()
                    ),
                ));
            }
        };
        self.check_initial_dest(initial.target, None, &initials[0].loc)?;
        for i in 0..self.states.len() {
            self.resolve_state(i)?;
        }
        for i in 0..self.choices.len() {
            self.resolve_choice(i)?;
        }

        // CheckTransitionGraph: reachability, then choice cycles.
        self.check_reachability(initial.target)?;
        self.check_choice_cycles()?;

        // CheckTypedElements.
        let mut model = StateMachineModel {
            sym: self.sym,
            actions: std::mem::take(&mut self.actions),
            guards: std::mem::take(&mut self.guards),
            signals: std::mem::take(&mut self.signals),
            initial,
            states: std::mem::take(&mut self.states),
            choices: std::mem::take(&mut self.choices),
        };
        let types = choice_types(self.a, &model)?;
        for (i, ty) in types.into_iter().enumerate() {
            model.choices[i].ty = ty;
        }
        check_call_sites(self.a, &model)?;
        Ok(model)
    }

    fn opt_type(&mut self, tn: &Option<Node<TypeName>>) -> Result<Option<Type>> {
        match tn {
            Some(tn) => Ok(Some(self.a.resolve_type_name(&self.stack, tn)?)),
            None => Ok(None),
        }
    }

    fn enter_typed(
        list: &mut Vec<SmTyped>,
        kind: &str,
        name: &str,
        ty: Option<Type>,
        docs: Vec<String>,
        loc: &Loc,
    ) -> Result<()> {
        if let Some(prev) = list.iter().find(|x| x.name == name) {
            return Err(Diagnostic::semantic(
                loc.clone(),
                format!("redefinition of {kind} {name}"),
            )
            .with_note(prev.loc.clone(), "previous definition is here"));
        }
        list.push(SmTyped {
            name: name.to_string(),
            ty,
            docs,
            loc: loc.clone(),
        });
        Ok(())
    }

    fn enter_name(&mut self, scope: ScopeKey, name: &str, t: SmTarget, loc: &Loc) -> Result<()> {
        if let Some((_, prev)) = self.scope.get(&(scope, name.to_string())) {
            return Err(Diagnostic::semantic(
                loc.clone(),
                format!("redefinition of state or choice {name}"),
            )
            .with_note(prev.clone(), "previous definition is here"));
        }
        self.scope
            .insert((scope, name.to_string()), (t, loc.clone()));
        Ok(())
    }

    fn enter_state(
        &mut self,
        n: &'a Node<DefState>,
        parent: Option<usize>,
        prefix: &[String],
        docs: Vec<String>,
    ) -> Result<()> {
        let idx = self.states.len();
        self.enter_name(parent, &n.data.name, SmTarget::State(idx), &n.loc)?;
        let mut path = prefix.to_vec();
        path.push(n.data.name.clone());
        self.states.push(SmState {
            name: n.data.name.clone(),
            path: path.clone(),
            parent,
            substates: Vec::new(),
            initial: None,
            entry: Vec::new(),
            exit: Vec::new(),
            transitions: Vec::new(),
            docs,
            loc: n.loc.clone(),
        });
        self.state_nodes.push(n);
        if let Some(p) = parent {
            self.states[p].substates.push(idx);
        }
        for m in &n.data.members {
            let docs: Vec<String> = m.doc_lines().cloned().collect();
            match &m.node {
                StateMemberNode::DefState(s) => self.enter_state(s, Some(idx), &path, docs)?,
                StateMemberNode::DefChoice(c) => self.enter_choice(c, Some(idx), &path, docs)?,
                StateMemberNode::SpecInclude(_) => unreachable!("includes are spliced"),
                _ => {}
            }
        }
        Ok(())
    }

    fn enter_choice(
        &mut self,
        n: &'a Node<DefChoice>,
        parent: Option<usize>,
        prefix: &[String],
        docs: Vec<String>,
    ) -> Result<()> {
        let idx = self.choices.len();
        self.enter_name(parent, &n.data.name, SmTarget::Choice(idx), &n.loc)?;
        let mut path = prefix.to_vec();
        path.push(n.data.name.clone());
        // The guard and transitions are resolved once every name is known.
        let placeholder = SmTransition {
            actions: Vec::new(),
            target: SmTarget::Choice(idx),
        };
        self.choices.push(SmChoice {
            name: n.data.name.clone(),
            path,
            parent,
            guard: String::new(),
            if_transition: placeholder.clone(),
            else_transition: placeholder,
            ty: None,
            docs,
            loc: n.loc.clone(),
        });
        self.choice_nodes.push(n);
        Ok(())
    }

    /// Resolve a state or choice use from `scope` outwards.
    fn resolve_target(&self, qi: &Node<QualIdent>, scope: ScopeKey) -> Result<SmTarget> {
        let first = &qi.data.parts[0];
        let mut cur = scope;
        let mut found = None;
        loop {
            if let Some((t, _)) = self.scope.get(&(cur, first.data.clone())) {
                found = Some(*t);
                break;
            }
            match cur {
                Some(s) => cur = self.states[s].parent,
                None => break,
            }
        }
        let mut t = found.ok_or_else(|| {
            Diagnostic::semantic(
                first.loc.clone(),
                format!("undefined state or choice symbol {}", first.data),
            )
        })?;
        for part in &qi.data.parts[1..] {
            let SmTarget::State(s) = t else {
                return Err(Diagnostic::semantic(
                    part.loc.clone(),
                    format!(
                        "a choice is not a scope; cannot select {} from it",
                        part.data
                    ),
                ));
            };
            t = self
                .scope
                .get(&(Some(s), part.data.clone()))
                .map(|(t, _)| *t)
                .ok_or_else(|| {
                    Diagnostic::semantic(
                        part.loc.clone(),
                        format!(
                            "undefined state or choice symbol {} in state {}",
                            part.data,
                            self.states[s].path.join(".")
                        ),
                    )
                })?;
        }
        Ok(t)
    }

    fn check_use(&self, list: &[SmTyped], kind: &str, n: &Node<Ident>) -> Result<String> {
        if list.iter().any(|x| x.name == n.data) {
            Ok(n.data.clone())
        } else {
            Err(Diagnostic::semantic(
                n.loc.clone(),
                format!("undefined {kind} symbol {}", n.data),
            ))
        }
    }

    fn actions(&self, list: &[Node<Ident>]) -> Result<Vec<String>> {
        list.iter()
            .map(|a| self.check_use(&self.actions, "action", a))
            .collect()
    }

    fn transition(&self, t: &Node<TransitionExpr>, scope: ScopeKey) -> Result<SmTransition> {
        Ok(SmTransition {
            actions: self.actions(&t.data.actions)?,
            target: self.resolve_target(&t.data.target, scope)?,
        })
    }

    fn resolve_state(&mut self, idx: usize) -> Result<()> {
        let n = self.state_nodes[idx];
        let scope = Some(idx);
        let mut initials: Vec<(&Node<SpecInitialTransition>, SmTransition)> = Vec::new();
        let mut entry = Vec::new();
        let mut exit = Vec::new();
        let mut transitions: Vec<SmStateTransition> = Vec::new();
        let mut seen_signals: HashMap<String, Loc> = HashMap::new();
        for m in &n.data.members {
            match &m.node {
                StateMemberNode::SpecInitialTransition(i) => {
                    let t = self.transition(&i.data.transition, scope)?;
                    initials.push((i, t));
                }
                StateMemberNode::SpecStateEntry(e) => entry.extend(self.actions(&e.data.actions)?),
                StateMemberNode::SpecStateExit(e) => exit.extend(self.actions(&e.data.actions)?),
                StateMemberNode::SpecStateTransition(st) => {
                    let d = &st.data;
                    let signal = self.check_use(&self.signals, "signal", &d.signal)?;
                    // CheckSignalUses: one transition per signal and state.
                    if let Some(prev) = seen_signals.get(&signal) {
                        return Err(Diagnostic::semantic(
                            d.signal.loc.clone(),
                            format!("duplicate use of signal {signal} in state {}", n.data.name),
                        )
                        .with_note(prev.clone(), "previous use is here"));
                    }
                    seen_signals.insert(signal.clone(), d.signal.loc.clone());
                    let guard = match &d.guard {
                        Some(g) => Some(self.check_use(&self.guards, "guard", g)?),
                        None => None,
                    };
                    let (actions, target) = match &d.transition_or_do {
                        TransitionOrDo::Transition(t) => {
                            let t = self.transition(t, scope)?;
                            (t.actions, Some(t.target))
                        }
                        TransitionOrDo::Do(actions) => (self.actions(actions)?, None),
                    };
                    transitions.push(SmStateTransition {
                        signal,
                        guard,
                        actions,
                        target,
                        loc: st.loc.clone(),
                    });
                }
                _ => {}
            }
        }
        // CheckInitialTransitions (a state).
        let has_substates = !self.states[idx].substates.is_empty();
        let initial = match (has_substates, initials.len()) {
            (false, 0) => None,
            (_, 1) => {
                let (node, t) = initials.pop().expect("one");
                self.check_initial_dest(t.target, Some(idx), &node.loc)?;
                Some(t)
            }
            (_, 0) => {
                return Err(Diagnostic::semantic(
                    n.loc.clone(),
                    "state with substates must have initial transition",
                ));
            }
            (_, k) => {
                return Err(Diagnostic::semantic(
                    n.loc.clone(),
                    format!(
                        "state with substates has {k} initial transitions; only one is allowed"
                    ),
                ));
            }
        };
        let s = &mut self.states[idx];
        s.initial = initial;
        s.entry = entry;
        s.exit = exit;
        s.transitions = transitions;
        Ok(())
    }

    fn resolve_choice(&mut self, idx: usize) -> Result<()> {
        let n = self.choice_nodes[idx];
        let scope = self.choices[idx].parent;
        let guard = self.check_use(&self.guards, "guard", &n.data.guard)?;
        let if_t = self.transition(&n.data.if_transition, scope)?;
        let else_t = self.transition(&n.data.else_transition, scope)?;
        let c = &mut self.choices[idx];
        c.guard = guard;
        c.if_transition = if_t;
        c.else_transition = else_t;
        Ok(())
    }

    /// The enclosing state of a state or choice.
    fn parent_of(&self, t: SmTarget) -> Option<usize> {
        match t {
            SmTarget::State(s) => self.states[s].parent,
            SmTarget::Choice(c) => self.choices[c].parent,
        }
    }

    /// The choice transitions of choice `c` (its guard, transitions and
    /// targets are resolved on demand here, since initial transitions are
    /// checked before every choice is resolved).
    fn choice_targets(&self, c: usize) -> Result<[SmTarget; 2]> {
        let n = self.choice_nodes[c];
        let scope = self.choices[c].parent;
        Ok([
            self.resolve_target(&n.data.if_transition.data.target, scope)?,
            self.resolve_target(&n.data.else_transition.data.target, scope)?,
        ])
    }

    /// An initial transition of `parent` (the machine if `None`) must go
    /// to a state or choice defined in `parent`, and so must every choice
    /// transition it leads through.
    fn check_initial_dest(&self, dest: SmTarget, parent: Option<usize>, loc: &Loc) -> Result<()> {
        let mut visited: HashSet<SmTarget> = HashSet::new();
        let mut work = vec![dest];
        while let Some(t) = work.pop() {
            if !visited.insert(t) {
                continue;
            }
            if self.parent_of(t) != parent {
                let msg = match parent {
                    Some(_) => {
                        "initial transition of state must go to state or choice defined in the same state"
                    }
                    None => {
                        "initial transition of state machine may not go to a state or choice defined in a substate"
                    }
                };
                let tloc = match t {
                    SmTarget::State(s) => self.states[s].loc.clone(),
                    SmTarget::Choice(c) => self.choices[c].loc.clone(),
                };
                return Err(Diagnostic::semantic(loc.clone(), msg)
                    .with_note(tloc, "transition path goes here"));
            }
            if let SmTarget::Choice(c) = t {
                work.extend(self.choice_targets(c)?);
            }
        }
        Ok(())
    }

    /// The arcs of the transition graph out of a node.
    fn arcs(&self, t: SmTarget) -> Vec<SmTarget> {
        match t {
            SmTarget::State(s) => {
                let st = &self.states[s];
                st.initial
                    .iter()
                    .map(|i| i.target)
                    .chain(st.transitions.iter().filter_map(|t| t.target))
                    .collect()
            }
            SmTarget::Choice(c) => vec![
                self.choices[c].if_transition.target,
                self.choices[c].else_transition.target,
            ],
        }
    }

    /// CheckTGReachability: every state and choice is reachable from the
    /// initial transition.
    fn check_reachability(&self, initial: SmTarget) -> Result<()> {
        let mut visited: HashSet<SmTarget> = HashSet::new();
        let mut work = vec![initial];
        while let Some(t) = work.pop() {
            if visited.insert(t) {
                work.extend(self.arcs(t));
            }
        }
        for (i, s) in self.states.iter().enumerate() {
            if !visited.contains(&SmTarget::State(i)) {
                return Err(Diagnostic::semantic(
                    s.loc.clone(),
                    format!("state {} is unreachable", s.path.join(".")),
                ));
            }
        }
        for (i, c) in self.choices.iter().enumerate() {
            if !visited.contains(&SmTarget::Choice(i)) {
                return Err(Diagnostic::semantic(
                    c.loc.clone(),
                    format!("choice {} is unreachable", c.path.join(".")),
                ));
            }
        }
        Ok(())
    }

    /// CheckChoiceCycles: no cycle of choice-to-choice transitions.
    fn check_choice_cycles(&self) -> Result<()> {
        fn visit(b: &Builder<'_, '_>, c: usize, path: &mut Vec<usize>) -> Result<()> {
            if path.contains(&c) {
                let names: Vec<String> = path
                    .iter()
                    .chain(std::iter::once(&c))
                    .map(|i| b.choices[*i].path.join("."))
                    .collect();
                return Err(Diagnostic::semantic(
                    b.choices[c].loc.clone(),
                    format!("encountered a choice cycle: {}", names.join(" -> ")),
                ));
            }
            path.push(c);
            for t in b.arcs(SmTarget::Choice(c)) {
                if let SmTarget::Choice(next) = t {
                    visit(b, next, path)?;
                }
            }
            path.pop();
            Ok(())
        }
        for c in 0..self.choices.len() {
            visit(self, c, &mut Vec::new())?;
        }
        Ok(())
    }
}

// -- typed elements --------------------------------------------------------------------

/// `TypeOption.commonType`: `None` if the types have no common type.
fn common_type_option(
    a: &Analysis<'_>,
    t1: &Option<Type>,
    t2: &Option<Type>,
) -> Option<Option<Type>> {
    match (t1, t2) {
        (Some(x), Some(y)) => {
            if a.same_type(x, y) {
                return Some(Some(x.clone()));
            }
            match (a.underlying(x), a.underlying(y)) {
                (Type::Int(i1), Type::Int(i2)) => {
                    if i1.signed() != i2.signed() {
                        None
                    } else if i2.bits() > i1.bits() {
                        Some(t2.clone())
                    } else {
                        Some(t1.clone())
                    }
                }
                (Type::Float(f1), Type::Float(f2)) => {
                    let bits = |f: FloatKind| if f == FloatKind::F64 { 64 } else { 32 };
                    if bits(f2) > bits(f1) {
                        Some(t2.clone())
                    } else {
                        Some(t1.clone())
                    }
                }
                (Type::String(_), Type::String(_)) => Some(Some(Type::String(None))),
                _ => None,
            }
        }
        _ => Some(None),
    }
}

/// `TypeOption.isConvertibleTo`.
fn convertible(a: &Analysis<'_>, from: &Option<Type>, to: &Option<Type>) -> bool {
    match (from, to) {
        (_, None) => true,
        (None, Some(_)) => false,
        (Some(x), Some(y)) => {
            if a.same_type(x, y) {
                return true;
            }
            match (a.underlying(x), a.underlying(y)) {
                (Type::Int(i1), Type::Int(i2)) => {
                    i1.signed() == i2.signed() && i2.bits() >= i1.bits()
                }
                (Type::Float(f1), Type::Float(f2)) => f2 == FloatKind::F64 || f1 == FloatKind::F32,
                (Type::String(_), Type::String(_)) => true,
                _ => false,
            }
        }
    }
}

fn show(a: &Analysis<'_>, t: &Option<Type>) -> String {
    match t {
        None => "None".into(),
        Some(t) => a.describe_type(t),
    }
}

/// The type option of a typed element, choices given.
fn element_type(
    m: &StateMachineModel,
    choice_types: &[Option<Type>],
    te: TypedElement,
) -> Option<Type> {
    match te {
        TypedElement::Initial => None,
        TypedElement::StateTransition(s, t) => m
            .signal(&m.states[s].transitions[t].signal)
            .and_then(|s| s.ty.clone()),
        TypedElement::Choice(c) => choice_types[c].clone(),
    }
}

/// ComputeTypeOptionMap: each choice's type is the common type of the
/// typed elements of the transitions into it (none: untyped).
fn choice_types(a: &Analysis<'_>, m: &StateMachineModel) -> Result<Vec<Option<Type>>> {
    // Incoming arcs of each choice, as typed elements.
    let mut incoming: Vec<Vec<TypedElement>> = vec![Vec::new(); m.choices.len()];
    if let SmTarget::Choice(c) = m.initial.target {
        incoming[c].push(TypedElement::Initial);
    }
    for (si, s) in m.states.iter().enumerate() {
        if let Some(SmTarget::Choice(c)) = s.initial.as_ref().map(|i| i.target) {
            incoming[c].push(TypedElement::Initial);
        }
        for (ti, t) in s.transitions.iter().enumerate() {
            if let Some(SmTarget::Choice(c)) = t.target {
                incoming[c].push(TypedElement::StateTransition(si, ti));
            }
        }
    }
    for (ci, c) in m.choices.iter().enumerate() {
        for t in [c.if_transition.target, c.else_transition.target] {
            if let SmTarget::Choice(next) = t {
                incoming[next].push(TypedElement::Choice(ci));
            }
        }
    }
    // Choices form no cycles, so a depth-first evaluation terminates.
    fn eval(
        a: &Analysis<'_>,
        m: &StateMachineModel,
        incoming: &[Vec<TypedElement>],
        done: &mut Vec<Option<Option<Type>>>,
        c: usize,
    ) -> Result<Option<Type>> {
        if let Some(t) = &done[c] {
            return Ok(t.clone());
        }
        let mut acc: Option<(TypedElement, Option<Type>)> = None;
        for te in &incoming[c] {
            if let TypedElement::Choice(p) = te {
                eval(a, m, incoming, done, *p)?;
            }
            let known: Vec<Option<Type>> = done.iter().map(|d| d.clone().unwrap_or(None)).collect();
            let to = element_type(m, &known, *te);
            acc = Some(match acc {
                None => (*te, to),
                Some((_, prev)) => match common_type_option(a, &prev, &to) {
                    Some(t) => (*te, t),
                    None => {
                        return Err(Diagnostic::semantic(
                            m.choices[c].loc.clone(),
                            format!(
                                "type mismatch at choice {}: {} and {}",
                                m.choices[c].path.join("."),
                                show(a, &prev),
                                show(a, &to)
                            ),
                        ));
                    }
                },
            });
        }
        let t = acc.and_then(|(_, t)| t);
        done[c] = Some(t.clone());
        Ok(t)
    }
    let mut done: Vec<Option<Option<Type>>> = vec![None; m.choices.len()];
    let mut out = Vec::new();
    for c in 0..m.choices.len() {
        out.push(eval(a, m, &incoming, &mut done, c)?);
    }
    Ok(out)
}

/// CheckActionAndGuardTypes: the value of each typed element converts to
/// the type of every action and guard it reaches.
fn check_call_sites(a: &Analysis<'_>, m: &StateMachineModel) -> Result<()> {
    let choice_types: Vec<Option<Type>> = m.choices.iter().map(|c| c.ty.clone()).collect();
    let check = |te: TypedElement,
                 kind: &str,
                 site_kind: &str,
                 names: &[String],
                 loc: &Loc|
     -> Result<()> {
        let te_to = element_type(m, &choice_types, te);
        for n in names {
            let list = if site_kind == "action" {
                &m.actions
            } else {
                &m.guards
            };
            let site_to = list
                .iter()
                .find(|x| &x.name == n)
                .and_then(|x| x.ty.clone());
            if !convertible(a, &te_to, &site_to) {
                return Err(Diagnostic::semantic(
                    loc.clone(),
                    format!(
                        "type mismatch at {site_kind} {n}: {kind} has type {}, {site_kind} has type {}",
                        show(a, &te_to),
                        show(a, &site_to)
                    ),
                ));
            }
        }
        Ok(())
    };
    let sym_loc = a.symbols.sym(m.sym).loc.clone();
    check(
        TypedElement::Initial,
        "initial transition",
        "action",
        &m.initial.actions,
        &sym_loc,
    )?;
    for (si, s) in m.states.iter().enumerate() {
        if let Some(i) = &s.initial {
            check(
                TypedElement::Initial,
                "initial transition",
                "action",
                &i.actions,
                &s.loc,
            )?;
        }
        check(
            TypedElement::Initial,
            "state entry",
            "action",
            &s.entry,
            &s.loc,
        )?;
        check(
            TypedElement::Initial,
            "state exit",
            "action",
            &s.exit,
            &s.loc,
        )?;
        for (ti, t) in s.transitions.iter().enumerate() {
            let te = TypedElement::StateTransition(si, ti);
            if let Some(g) = &t.guard {
                check(
                    te,
                    "state transition",
                    "guard",
                    std::slice::from_ref(g),
                    &t.loc,
                )?;
            }
            check(te, "state transition", "action", &t.actions, &t.loc)?;
        }
    }
    for (ci, c) in m.choices.iter().enumerate() {
        let te = TypedElement::Choice(ci);
        check(
            te,
            "choice",
            "guard",
            std::slice::from_ref(&c.guard),
            &c.loc,
        )?;
        check(te, "choice", "action", &c.if_transition.actions, &c.loc)?;
        check(te, "choice", "action", &c.else_transition.actions, &c.loc)?;
    }
    Ok(())
}
