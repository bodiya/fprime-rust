//! The FPP parser — a recursive-descent port of the reference compiler's
//! `Parser.scala`, production for production.
//!
//! The element-sequence rules carry the language's punctuation model: an
//! element is terminated by the sequence's punctuation (`;` or `,`), by an
//! end of line, or by a post-annotation, and the last element may be
//! unterminated. Pre-annotations attach to the following element.

use crate::ast::*;
use crate::error::{Diagnostic, Loc, Phase, Result};
use crate::lexer::{Tok, Token};

/// Parse one translation unit from its token stream.
///
/// `next_id` is the first node id to hand out; the returned value is the id
/// after the last one used, so multiple files parsed in sequence get
/// disjoint ids.
pub fn parse_trans_unit(tokens: Vec<Token>, next_id: &mut NodeId) -> Result<TransUnit> {
    let mut p = Parser {
        toks: tokens,
        pos: 0,
        next_id: *next_id,
    };
    let members = p.module_members()?;
    if p.peek() != &Tok::Eof {
        return Err(p.err_here("unexpected token"));
    }
    *next_id = p.next_id;
    Ok(TransUnit { members })
}

/// Parse an included file's contents as component members.
pub fn parse_component_members(
    tokens: Vec<Token>,
    next_id: &mut NodeId,
) -> Result<Vec<ComponentMember>> {
    parse_fragment(tokens, next_id, |p| p.component_members())
}

/// Parse an included file's contents as topology members.
pub fn parse_topology_members(
    tokens: Vec<Token>,
    next_id: &mut NodeId,
) -> Result<Vec<TopologyMember>> {
    parse_fragment(tokens, next_id, |p| {
        p.annotated_seq(Punct::Semi, |p| p.topology_member_node())
    })
}

/// Parse an included file's contents as telemetry packet set members.
pub fn parse_tlm_packet_set_members(
    tokens: Vec<Token>,
    next_id: &mut NodeId,
) -> Result<Vec<TlmPacketSetMember>> {
    parse_fragment(tokens, next_id, |p| p.tlm_packet_set_members())
}

/// Parse an included file's contents as telemetry packet members.
pub fn parse_tlm_packet_members(
    tokens: Vec<Token>,
    next_id: &mut NodeId,
) -> Result<Vec<TlmPacketMember>> {
    parse_fragment(tokens, next_id, |p| {
        p.skip_eols();
        p.tlm_packet_members()
    })
}

/// Parse an included file's contents as state machine members.
pub fn parse_state_machine_members(
    tokens: Vec<Token>,
    next_id: &mut NodeId,
) -> Result<Vec<StateMachineMember>> {
    parse_fragment(tokens, next_id, |p| {
        p.annotated_seq(Punct::Semi, |p| p.state_machine_member_node())
    })
}

/// Parse an included file's contents as state members.
pub fn parse_state_members(tokens: Vec<Token>, next_id: &mut NodeId) -> Result<Vec<StateMember>> {
    parse_fragment(tokens, next_id, |p| {
        p.annotated_seq(Punct::Semi, |p| p.state_member_node())
    })
}

fn parse_fragment<T>(
    tokens: Vec<Token>,
    next_id: &mut NodeId,
    f: impl FnOnce(&mut Parser) -> Result<T>,
) -> Result<T> {
    let mut p = Parser {
        toks: tokens,
        pos: 0,
        next_id: *next_id,
    };
    let out = f(&mut p)?;
    p.skip_eols();
    if p.peek() != &Tok::Eof {
        return Err(p.err_here("unexpected token"));
    }
    *next_id = p.next_id;
    Ok(out)
}

struct Parser {
    toks: Vec<Token>,
    pos: usize,
    next_id: NodeId,
}

/// Which punctuation terminates elements of a sequence.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Punct {
    Semi,
    Comma,
}

impl Punct {
    fn tok(self) -> Tok {
        match self {
            Punct::Semi => Tok::Semi,
            Punct::Comma => Tok::Comma,
        }
    }
}

impl Parser {
    // -- token access ---------------------------------------------------------

    fn peek(&self) -> &Tok {
        &self.toks[self.pos.min(self.toks.len() - 1)].tok
    }

    fn peek_at(&self, n: usize) -> &Tok {
        &self.toks[(self.pos + n).min(self.toks.len() - 1)].tok
    }

    fn loc(&self) -> Loc {
        self.toks[self.pos.min(self.toks.len() - 1)].loc.clone()
    }

    fn advance(&mut self) -> Tok {
        let t = self.peek().clone();
        if self.pos < self.toks.len() - 1 {
            self.pos += 1;
        }
        t
    }

    fn at(&self, t: &Tok) -> bool {
        self.peek() == t
    }

    fn accept(&mut self, t: &Tok) -> bool {
        if self.at(t) {
            self.advance();
            true
        } else {
            false
        }
    }

    fn expect(&mut self, t: &Tok) -> Result<()> {
        if self.accept(t) {
            Ok(())
        } else {
            Err(self.err_here(format!("{} expected", t.describe())))
        }
    }

    fn err_here(&self, msg: impl Into<String>) -> Diagnostic {
        let msg = msg.into();
        Diagnostic::new(
            Phase::Syntax,
            self.loc(),
            format!("{msg}, found {}", self.peek().describe()),
        )
    }

    fn node<T>(&mut self, loc: Loc, data: T) -> Node<T> {
        let id = self.next_id;
        self.next_id += 1;
        Node { id, loc, data }
    }

    /// Run `f` and wrap its result in a node located at the current token.
    fn with_node<T>(&mut self, f: impl FnOnce(&mut Self) -> Result<T>) -> Result<Node<T>> {
        let loc = self.loc();
        let data = f(self)?;
        Ok(self.node(loc, data))
    }

    fn ident(&mut self) -> Result<Ident> {
        match self.peek() {
            Tok::Ident(s) => {
                let s = s.clone();
                self.advance();
                Ok(s)
            }
            _ => Err(self.err_here("identifier expected")),
        }
    }

    fn ident_node(&mut self) -> Result<Node<Ident>> {
        self.with_node(|p| p.ident())
    }

    fn literal_string(&mut self) -> Result<String> {
        match self.peek() {
            Tok::LitString(s) => {
                let s = s.clone();
                self.advance();
                Ok(s)
            }
            _ => Err(self.err_here("string literal expected")),
        }
    }

    fn literal_string_node(&mut self) -> Result<Node<String>> {
        self.with_node(|p| p.literal_string())
    }

    fn skip_eols(&mut self) {
        while self.accept(&Tok::Eol) {}
    }

    fn is_closer(&self) -> bool {
        matches!(
            self.peek(),
            Tok::RBrace | Tok::RParen | Tok::RBracket | Tok::Eof
        )
    }

    // -- sequences -----------------------------------------------------------------

    /// `annotatedElementSequence`: elements separated by `punct` or end of
    /// line, each with pre/post annotations.
    fn annotated_seq<T>(
        &mut self,
        punct: Punct,
        mut elt: impl FnMut(&mut Self) -> Result<T>,
    ) -> Result<Vec<Annotated<T>>> {
        let mut out = Vec::new();
        self.skip_eols();
        loop {
            if self.is_closer() {
                return Ok(out);
            }
            let mut pre = Vec::new();
            while let Tok::PreAnnotation(s) = self.peek() {
                pre.push(s.clone());
                self.advance();
            }
            let node = elt(self)?;
            let terminated = self.accept(&punct.tok())
                || self.accept(&Tok::Eol)
                || matches!(self.peek(), Tok::PostAnnotation(_));
            let mut post = Vec::new();
            while let Tok::PostAnnotation(s) = self.peek() {
                post.push(s.clone());
                self.advance();
            }
            out.push(Annotated { pre, node, post });
            if !terminated {
                // An unterminated element must be the last one.
                return Ok(out);
            }
        }
    }

    /// `elementSequence`: `repsep(elt, sep | eol) <~ opt(sep | eol)`.
    fn element_seq<T>(
        &mut self,
        punct: Punct,
        mut elt: impl FnMut(&mut Self) -> Result<T>,
    ) -> Result<Vec<T>> {
        let mut out = Vec::new();
        loop {
            if self.is_closer() {
                return Ok(out);
            }
            out.push(elt(self)?);
            if !(self.accept(&punct.tok()) || self.accept(&Tok::Eol)) {
                return Ok(out);
            }
        }
    }

    fn braced<T>(&mut self, f: impl FnOnce(&mut Self) -> Result<T>) -> Result<T> {
        self.expect(&Tok::LBrace)?;
        let v = f(self)?;
        self.expect(&Tok::RBrace)?;
        Ok(v)
    }

    // -- module level ----------------------------------------------------------------

    fn module_members(&mut self) -> Result<Vec<ModuleMember>> {
        self.annotated_seq(Punct::Semi, |p| p.module_member_node())
    }

    fn module_member_node(&mut self) -> Result<ModuleMemberNode> {
        Ok(match (self.peek(), self.peek_at(1), self.peek_at(2)) {
            (Tok::Dictionary, Tok::Type, _) | (Tok::Type, _, Tok::Equals) => {
                ModuleMemberNode::DefAliasType(self.with_node(|p| p.def_alias_type())?)
            }
            (Tok::Type, _, _) => {
                ModuleMemberNode::DefAbsType(self.with_node(|p| p.def_abs_type())?)
            }
            (Tok::Array, _, _) | (Tok::Dictionary, Tok::Array, _) => {
                ModuleMemberNode::DefArray(self.with_node(|p| p.def_array())?)
            }
            (Tok::Active | Tok::Passive | Tok::Queued, _, _) => {
                ModuleMemberNode::DefComponent(self.with_node(|p| p.def_component())?)
            }
            (Tok::Interface, _, _) => {
                ModuleMemberNode::DefInterface(self.with_node(|p| p.def_interface())?)
            }
            (Tok::Instance, _, _) => ModuleMemberNode::DefComponentInstance(
                self.with_node(|p| p.def_component_instance())?,
            ),
            (Tok::Constant, _, _) | (Tok::Dictionary, Tok::Constant, _) => {
                ModuleMemberNode::DefConstant(self.with_node(|p| p.def_constant())?)
            }
            (Tok::Enum, _, _) | (Tok::Dictionary, Tok::Enum, _) => {
                ModuleMemberNode::DefEnum(self.with_node(|p| p.def_enum())?)
            }
            (Tok::Module, _, _) => ModuleMemberNode::DefModule(self.with_node(|p| p.def_module())?),
            (Tok::Port, _, _) => ModuleMemberNode::DefPort(self.with_node(|p| p.def_port())?),
            (Tok::State, _, _) => {
                ModuleMemberNode::DefStateMachine(self.with_node(|p| p.def_state_machine())?)
            }
            (Tok::Struct, _, _) | (Tok::Dictionary, Tok::Struct, _) => {
                ModuleMemberNode::DefStruct(self.with_node(|p| p.def_struct())?)
            }
            (Tok::System, _, _) => ModuleMemberNode::DefSystem(self.with_node(|p| p.def_system())?),
            (Tok::Topology | Tok::Deployment, _, _) => {
                ModuleMemberNode::DefTopology(self.with_node(|p| p.def_topology())?)
            }
            (Tok::Include, _, _) => {
                ModuleMemberNode::SpecInclude(self.with_node(|p| p.spec_include())?)
            }
            (Tok::Locate, _, _) => ModuleMemberNode::SpecLoc(self.with_node(|p| p.spec_loc())?),
            _ => return Err(self.err_here("module member expected")),
        })
    }

    fn def_abs_type(&mut self) -> Result<DefAbsType> {
        self.expect(&Tok::Type)?;
        Ok(DefAbsType {
            name: self.ident()?,
        })
    }

    fn def_alias_type(&mut self) -> Result<DefAliasType> {
        let is_dictionary = self.accept(&Tok::Dictionary);
        self.expect(&Tok::Type)?;
        let name = self.ident()?;
        self.expect(&Tok::Equals)?;
        let type_name = self.type_name_node()?;
        Ok(DefAliasType {
            name,
            type_name,
            is_dictionary,
        })
    }

    fn def_array(&mut self) -> Result<DefArray> {
        let is_dictionary = self.accept(&Tok::Dictionary);
        self.expect(&Tok::Array)?;
        let name = self.ident()?;
        self.expect(&Tok::Equals)?;
        let size = self.index()?;
        let elt_type = self.type_name_node()?;
        let default = if self.accept(&Tok::Default) {
            Some(self.expr()?)
        } else {
            None
        };
        let format = if self.accept(&Tok::Format) {
            Some(self.literal_string_node()?)
        } else {
            None
        };
        Ok(DefArray {
            name,
            size,
            elt_type,
            default,
            format,
            is_dictionary,
        })
    }

    fn def_component(&mut self) -> Result<DefComponent> {
        let kind = match self.advance() {
            Tok::Active => ComponentKind::Active,
            Tok::Passive => ComponentKind::Passive,
            Tok::Queued => ComponentKind::Queued,
            _ => return Err(self.err_here("component kind expected")),
        };
        self.expect(&Tok::Component)?;
        let name = self.ident()?;
        let members = self.braced(|p| p.component_members())?;
        Ok(DefComponent {
            kind,
            name,
            members,
        })
    }

    fn def_component_instance(&mut self) -> Result<DefComponentInstance> {
        self.expect(&Tok::Instance)?;
        let name = self.ident()?;
        self.expect(&Tok::Colon)?;
        let component = self.qual_ident_node()?;
        self.expect(&Tok::Base)?;
        self.expect(&Tok::Id)?;
        let base_id = self.expr()?;
        let impl_type = if self.accept(&Tok::Type) {
            Some(self.literal_string_node()?)
        } else {
            None
        };
        let file = if self.accept(&Tok::At) {
            Some(self.literal_string_node()?)
        } else {
            None
        };
        let queue_size = if self.at(&Tok::Queue) {
            self.advance();
            self.expect(&Tok::Size)?;
            Some(self.expr()?)
        } else {
            None
        };
        let stack_size = if self.at(&Tok::Stack) {
            self.advance();
            self.expect(&Tok::Size)?;
            Some(self.expr()?)
        } else {
            None
        };
        let priority = if self.accept(&Tok::Priority) {
            Some(self.expr()?)
        } else {
            None
        };
        let cpu = if self.accept(&Tok::Cpu) {
            Some(self.expr()?)
        } else {
            None
        };
        let init_specs = if self.at(&Tok::LBrace) {
            self.braced(|p| p.annotated_seq(Punct::Semi, |p| p.with_node(|p| p.spec_init())))?
        } else {
            Vec::new()
        };
        Ok(DefComponentInstance {
            name,
            component,
            base_id,
            impl_type,
            file,
            queue_size,
            stack_size,
            priority,
            cpu,
            init_specs,
        })
    }

    fn spec_init(&mut self) -> Result<SpecInit> {
        self.expect(&Tok::Phase)?;
        let phase = self.expr()?;
        let code = self.literal_string()?;
        Ok(SpecInit { phase, code })
    }

    fn def_interface(&mut self) -> Result<DefInterface> {
        self.expect(&Tok::Interface)?;
        let name = self.ident()?;
        let members =
            self.braced(|p| p.annotated_seq(Punct::Semi, |p| p.interface_member_node()))?;
        Ok(DefInterface { name, members })
    }

    fn interface_member_node(&mut self) -> Result<InterfaceMemberNode> {
        Ok(match self.peek() {
            Tok::Import => {
                InterfaceMemberNode::SpecImportInterface(self.with_node(|p| p.spec_import())?)
            }
            _ => InterfaceMemberNode::SpecPortInstance(self.with_node(|p| p.spec_port_instance())?),
        })
    }

    fn def_constant(&mut self) -> Result<DefConstant> {
        let is_dictionary = self.accept(&Tok::Dictionary);
        self.expect(&Tok::Constant)?;
        let name = self.ident()?;
        self.expect(&Tok::Equals)?;
        let value = self.expr()?;
        Ok(DefConstant {
            name,
            value,
            is_dictionary,
        })
    }

    fn def_enum(&mut self) -> Result<DefEnum> {
        let is_dictionary = self.accept(&Tok::Dictionary);
        self.expect(&Tok::Enum)?;
        let name = self.ident()?;
        let type_name = if self.accept(&Tok::Colon) {
            Some(self.type_name_node()?)
        } else {
            None
        };
        let constants = self.braced(|p| {
            p.annotated_seq(Punct::Comma, |p| p.with_node(|p| p.def_enum_constant()))
        })?;
        let default = if self.accept(&Tok::Default) {
            Some(self.expr()?)
        } else {
            None
        };
        Ok(DefEnum {
            name,
            type_name,
            constants,
            default,
            is_dictionary,
        })
    }

    fn def_enum_constant(&mut self) -> Result<DefEnumConstant> {
        let name = self.ident()?;
        let value = if self.accept(&Tok::Equals) {
            Some(self.expr()?)
        } else {
            None
        };
        Ok(DefEnumConstant { name, value })
    }

    fn def_module(&mut self) -> Result<DefModule> {
        self.expect(&Tok::Module)?;
        let name = self.ident()?;
        let members = self.braced(|p| p.module_members())?;
        Ok(DefModule { name, members })
    }

    fn def_port(&mut self) -> Result<DefPort> {
        self.expect(&Tok::Port)?;
        let name = self.ident()?;
        let params = self.formal_param_list()?;
        let return_type = if self.accept(&Tok::RArrow) {
            Some(self.type_name_node()?)
        } else {
            None
        };
        Ok(DefPort {
            name,
            params,
            return_type,
        })
    }

    fn def_state_machine(&mut self) -> Result<DefStateMachine> {
        self.expect(&Tok::State)?;
        self.expect(&Tok::Machine)?;
        let name = self.ident()?;
        let members = if self.at(&Tok::LBrace) {
            Some(self.braced(|p| p.annotated_seq(Punct::Semi, |p| p.state_machine_member_node()))?)
        } else {
            None
        };
        Ok(DefStateMachine { name, members })
    }

    fn state_machine_member_node(&mut self) -> Result<StateMachineMemberNode> {
        Ok(match (self.peek(), self.peek_at(1), self.peek_at(2)) {
            (Tok::Dictionary, Tok::Type, _) | (Tok::Type, _, Tok::Equals) => {
                StateMachineMemberNode::DefAliasType(self.with_node(|p| p.def_alias_type())?)
            }
            (Tok::Type, _, _) => {
                StateMachineMemberNode::DefAbsType(self.with_node(|p| p.def_abs_type())?)
            }
            (Tok::Action, _, _) => {
                StateMachineMemberNode::DefAction(self.with_node(|p| p.def_action())?)
            }
            (Tok::Array, _, _) | (Tok::Dictionary, Tok::Array, _) => {
                StateMachineMemberNode::DefArray(self.with_node(|p| p.def_array())?)
            }
            (Tok::Choice, _, _) => {
                StateMachineMemberNode::DefChoice(self.with_node(|p| p.def_choice())?)
            }
            (Tok::Constant, _, _) | (Tok::Dictionary, Tok::Constant, _) => {
                StateMachineMemberNode::DefConstant(self.with_node(|p| p.def_constant())?)
            }
            (Tok::Enum, _, _) | (Tok::Dictionary, Tok::Enum, _) => {
                StateMachineMemberNode::DefEnum(self.with_node(|p| p.def_enum())?)
            }
            (Tok::Guard, _, _) => {
                StateMachineMemberNode::DefGuard(self.with_node(|p| p.def_guard())?)
            }
            (Tok::Signal, _, _) => {
                StateMachineMemberNode::DefSignal(self.with_node(|p| p.def_signal())?)
            }
            (Tok::State, _, _) => {
                StateMachineMemberNode::DefState(self.with_node(|p| p.def_state())?)
            }
            (Tok::Struct, _, _) | (Tok::Dictionary, Tok::Struct, _) => {
                StateMachineMemberNode::DefStruct(self.with_node(|p| p.def_struct())?)
            }
            (Tok::Include, _, _) => {
                StateMachineMemberNode::SpecInclude(self.with_node(|p| p.spec_include())?)
            }
            (Tok::Initial, _, _) => StateMachineMemberNode::SpecInitialTransition(
                self.with_node(|p| p.spec_initial_transition())?,
            ),
            _ => return Err(self.err_here("state machine member expected")),
        })
    }

    fn def_action(&mut self) -> Result<DefAction> {
        self.expect(&Tok::Action)?;
        let name = self.ident()?;
        let type_name = if self.accept(&Tok::Colon) {
            Some(self.type_name_node()?)
        } else {
            None
        };
        Ok(DefAction { name, type_name })
    }

    fn def_guard(&mut self) -> Result<DefGuard> {
        self.expect(&Tok::Guard)?;
        let name = self.ident()?;
        let type_name = if self.accept(&Tok::Colon) {
            Some(self.type_name_node()?)
        } else {
            None
        };
        Ok(DefGuard { name, type_name })
    }

    fn def_signal(&mut self) -> Result<DefSignal> {
        self.expect(&Tok::Signal)?;
        let name = self.ident()?;
        let type_name = if self.accept(&Tok::Colon) {
            Some(self.type_name_node()?)
        } else {
            None
        };
        Ok(DefSignal { name, type_name })
    }

    fn def_choice(&mut self) -> Result<DefChoice> {
        self.expect(&Tok::Choice)?;
        let name = self.ident()?;
        self.expect(&Tok::LBrace)?;
        self.expect(&Tok::If)?;
        let guard = self.ident_node()?;
        let if_transition = self.with_node(|p| p.transition_expr())?;
        self.expect(&Tok::Else)?;
        let else_transition = self.with_node(|p| p.transition_expr())?;
        self.expect(&Tok::RBrace)?;
        Ok(DefChoice {
            name,
            guard,
            if_transition,
            else_transition,
        })
    }

    fn def_state(&mut self) -> Result<DefState> {
        self.expect(&Tok::State)?;
        let name = self.ident()?;
        let members = if self.at(&Tok::LBrace) {
            self.braced(|p| p.annotated_seq(Punct::Semi, |p| p.state_member_node()))?
        } else {
            Vec::new()
        };
        Ok(DefState { name, members })
    }

    fn state_member_node(&mut self) -> Result<StateMemberNode> {
        Ok(match self.peek() {
            Tok::Choice => StateMemberNode::DefChoice(self.with_node(|p| p.def_choice())?),
            Tok::State => StateMemberNode::DefState(self.with_node(|p| p.def_state())?),
            Tok::Initial => StateMemberNode::SpecInitialTransition(
                self.with_node(|p| p.spec_initial_transition())?,
            ),
            Tok::Entry => StateMemberNode::SpecStateEntry(self.with_node(|p| {
                p.expect(&Tok::Entry)?;
                Ok(SpecStateEntry {
                    actions: p.do_expr()?,
                })
            })?),
            Tok::Exit => StateMemberNode::SpecStateExit(self.with_node(|p| {
                p.expect(&Tok::Exit)?;
                Ok(SpecStateExit {
                    actions: p.do_expr()?,
                })
            })?),
            Tok::Include => StateMemberNode::SpecInclude(self.with_node(|p| p.spec_include())?),
            Tok::On => {
                StateMemberNode::SpecStateTransition(self.with_node(|p| p.spec_state_transition())?)
            }
            _ => return Err(self.err_here("state member expected")),
        })
    }

    fn spec_initial_transition(&mut self) -> Result<SpecInitialTransition> {
        self.expect(&Tok::Initial)?;
        let transition = self.with_node(|p| p.transition_expr())?;
        Ok(SpecInitialTransition { transition })
    }

    fn spec_state_transition(&mut self) -> Result<SpecStateTransition> {
        self.expect(&Tok::On)?;
        let signal = self.ident_node()?;
        let guard = if self.accept(&Tok::If) {
            Some(self.ident_node()?)
        } else {
            None
        };
        let transition_or_do = if self.at(&Tok::Do) && !self.do_starts_transition() {
            TransitionOrDo::Do(self.do_expr()?)
        } else {
            TransitionOrDo::Transition(self.with_node(|p| p.transition_expr())?)
        };
        Ok(SpecStateTransition {
            signal,
            guard,
            transition_or_do,
        })
    }

    /// After `on s [if g]`, a `do { .. }` is a transition only if `enter`
    /// follows the closing brace.
    fn do_starts_transition(&self) -> bool {
        // Scan to the matching `}` and check for `enter`.
        let mut i = self.pos + 1;
        let mut depth = 0usize;
        while i < self.toks.len() {
            match &self.toks[i].tok {
                Tok::LBrace => depth += 1,
                Tok::RBrace => {
                    depth -= 1;
                    if depth == 0 {
                        return matches!(self.toks.get(i + 1).map(|t| &t.tok), Some(Tok::Enter));
                    }
                }
                Tok::Eof => return false,
                _ => {}
            }
            i += 1;
        }
        false
    }

    fn do_expr(&mut self) -> Result<Vec<Node<Ident>>> {
        self.expect(&Tok::Do)?;
        self.braced(|p| p.element_seq(Punct::Comma, |p| p.ident_node()))
    }

    fn transition_expr(&mut self) -> Result<TransitionExpr> {
        let actions = if self.at(&Tok::Do) {
            self.do_expr()?
        } else {
            Vec::new()
        };
        self.expect(&Tok::Enter)?;
        let target = self.qual_ident_node()?;
        Ok(TransitionExpr { actions, target })
    }

    fn def_struct(&mut self) -> Result<DefStruct> {
        let is_dictionary = self.accept(&Tok::Dictionary);
        self.expect(&Tok::Struct)?;
        let name = self.ident()?;
        let members = self.braced(|p| {
            p.annotated_seq(Punct::Comma, |p| p.with_node(|p| p.struct_type_member()))
        })?;
        let default = if self.accept(&Tok::Default) {
            Some(self.expr()?)
        } else {
            None
        };
        Ok(DefStruct {
            name,
            members,
            default,
            is_dictionary,
        })
    }

    fn struct_type_member(&mut self) -> Result<StructTypeMember> {
        let name = self.ident()?;
        self.expect(&Tok::Colon)?;
        let size = if self.at(&Tok::LBracket) {
            Some(self.index()?)
        } else {
            None
        };
        let type_name = self.type_name_node()?;
        let format = if self.accept(&Tok::Format) {
            Some(self.literal_string_node()?)
        } else {
            None
        };
        Ok(StructTypeMember {
            name,
            size,
            type_name,
            format,
        })
    }

    fn def_system(&mut self) -> Result<DefSystem> {
        self.expect(&Tok::System)?;
        let name = self.ident()?;
        self.expect(&Tok::Colon)?;
        let topology = self.qual_ident_node()?;
        Ok(DefSystem { name, topology })
    }

    fn def_topology(&mut self) -> Result<DefTopology> {
        let is_deployment = self.accept(&Tok::Deployment);
        self.expect(&Tok::Topology)?;
        let name = self.ident()?;
        let implements = if self.accept(&Tok::Implements) {
            self.element_seq(Punct::Comma, |p| p.qual_ident_node())?
        } else {
            Vec::new()
        };
        let members =
            self.braced(|p| p.annotated_seq(Punct::Semi, |p| p.topology_member_node()))?;
        Ok(DefTopology {
            is_deployment,
            name,
            members,
            implements,
        })
    }

    fn topology_member_node(&mut self) -> Result<TopologyMemberNode> {
        Ok(match (self.peek(), self.peek_at(1)) {
            (Tok::Instance | Tok::Import, _) => {
                TopologyMemberNode::SpecInstance(self.with_node(|p| p.spec_instance())?)
            }
            (Tok::Connections, _) => {
                TopologyMemberNode::SpecConnectionGraph(self.with_node(|p| p.direct_graph())?)
            }
            (Tok::Telemetry, Tok::Packets) => {
                TopologyMemberNode::SpecTlmPacketSet(self.with_node(|p| p.spec_tlm_packet_set())?)
            }
            (
                Tok::Command
                | Tok::Event
                | Tok::Health
                | Tok::Param
                | Tok::Telemetry
                | Tok::Text
                | Tok::Time,
                _,
            ) => TopologyMemberNode::SpecConnectionGraph(self.with_node(|p| p.pattern_graph())?),
            (Tok::Include, _) => {
                TopologyMemberNode::SpecInclude(self.with_node(|p| p.spec_include())?)
            }
            (Tok::Port, _) => {
                TopologyMemberNode::SpecTopPort(self.with_node(|p| p.spec_top_port())?)
            }
            _ => return Err(self.err_here("topology member expected")),
        })
    }

    fn spec_instance(&mut self) -> Result<SpecInstance> {
        if !(self.accept(&Tok::Instance) || self.accept(&Tok::Import)) {
            return Err(self.err_here("keyword `instance` or `import` expected"));
        }
        Ok(SpecInstance {
            instance: self.qual_ident_node()?,
        })
    }

    fn direct_graph(&mut self) -> Result<SpecConnectionGraph> {
        self.expect(&Tok::Connections)?;
        let name = self.ident()?;
        let connections = self.braced(|p| p.element_seq(Punct::Comma, |p| p.connection()))?;
        Ok(SpecConnectionGraph::Direct { name, connections })
    }

    fn connection(&mut self) -> Result<Connection> {
        let is_unmatched = self.accept(&Tok::Unmatched);
        let from_port = self.with_node(|p| p.port_instance_identifier())?;
        let from_index = if self.at(&Tok::LBracket) {
            Some(self.index()?)
        } else {
            None
        };
        self.expect(&Tok::RArrow)?;
        let to_port = self.with_node(|p| p.port_instance_identifier())?;
        let to_index = if self.at(&Tok::LBracket) {
            Some(self.index()?)
        } else {
            None
        };
        Ok(Connection {
            is_unmatched,
            from_port,
            from_index,
            to_port,
            to_index,
        })
    }

    fn pattern_graph(&mut self) -> Result<SpecConnectionGraph> {
        let kind = match self.advance() {
            Tok::Command => PatternKind::Command,
            Tok::Event => PatternKind::Event,
            Tok::Health => PatternKind::Health,
            Tok::Param => PatternKind::Param,
            Tok::Telemetry => PatternKind::Telemetry,
            Tok::Text => {
                self.expect(&Tok::Event)?;
                PatternKind::TextEvent
            }
            Tok::Time => PatternKind::Time,
            _ => return Err(self.err_here("connection graph expected")),
        };
        self.expect(&Tok::Connections)?;
        self.expect(&Tok::Instance)?;
        let source = self.qual_ident_node()?;
        let targets = if self.at(&Tok::LBrace) {
            self.braced(|p| p.element_seq(Punct::Comma, |p| p.qual_ident_node()))?
        } else {
            Vec::new()
        };
        Ok(SpecConnectionGraph::Pattern {
            kind,
            source,
            targets,
        })
    }

    fn spec_top_port(&mut self) -> Result<SpecTopPort> {
        self.expect(&Tok::Port)?;
        let name = self.ident()?;
        self.expect(&Tok::Equals)?;
        let underlying_port = self.with_node(|p| p.port_instance_identifier())?;
        Ok(SpecTopPort {
            name,
            underlying_port,
        })
    }

    fn spec_tlm_packet_set(&mut self) -> Result<SpecTlmPacketSet> {
        self.expect(&Tok::Telemetry)?;
        self.expect(&Tok::Packets)?;
        let name = self.ident()?;
        let members = self.braced(|p| p.tlm_packet_set_members())?;
        let omitted = if self.accept(&Tok::Omit) {
            self.braced(|p| {
                p.element_seq(Punct::Comma, |p| {
                    p.with_node(|p| p.tlm_channel_identifier())
                })
            })?
        } else {
            Vec::new()
        };
        Ok(SpecTlmPacketSet {
            name,
            members,
            omitted,
        })
    }

    fn spec_tlm_packet(&mut self) -> Result<SpecTlmPacket> {
        self.expect(&Tok::Packet)?;
        let name = self.ident()?;
        let id = if self.accept(&Tok::Id) {
            Some(self.expr()?)
        } else {
            None
        };
        self.expect(&Tok::Group)?;
        let group = self.expr()?;
        let members = self.braced(|p| p.tlm_packet_members())?;
        Ok(SpecTlmPacket {
            name,
            id,
            group,
            members,
        })
    }

    fn tlm_packet_set_members(&mut self) -> Result<Vec<TlmPacketSetMember>> {
        self.annotated_seq(Punct::Comma, |p| {
            Ok(match p.peek() {
                Tok::Include => {
                    TlmPacketSetMemberNode::SpecInclude(p.with_node(|p| p.spec_include())?)
                }
                _ => TlmPacketSetMemberNode::SpecTlmPacket(p.with_node(|p| p.spec_tlm_packet())?),
            })
        })
    }

    fn tlm_packet_members(&mut self) -> Result<Vec<TlmPacketMember>> {
        self.element_seq(Punct::Comma, |p| {
            Ok(match p.peek() {
                Tok::Include => TlmPacketMember::SpecInclude(p.with_node(|p| p.spec_include())?),
                _ => TlmPacketMember::TlmChannelIdentifier(
                    p.with_node(|p| p.tlm_channel_identifier())?,
                ),
            })
        })
    }

    fn spec_include(&mut self) -> Result<SpecInclude> {
        self.expect(&Tok::Include)?;
        Ok(SpecInclude {
            file: self.literal_string_node()?,
        })
    }

    fn spec_loc(&mut self) -> Result<SpecLoc> {
        self.expect(&Tok::Locate)?;
        let is_dictionary = self.accept(&Tok::Dictionary);
        let kind = match self.advance() {
            Tok::Constant => LocKind::Constant,
            Tok::Type => LocKind::Type,
            Tok::Component if !is_dictionary => LocKind::Component,
            Tok::Instance if !is_dictionary => LocKind::Instance,
            Tok::Port if !is_dictionary => LocKind::Port,
            Tok::State if !is_dictionary => {
                self.expect(&Tok::Machine)?;
                LocKind::StateMachine
            }
            Tok::System if !is_dictionary => LocKind::System,
            Tok::Interface if !is_dictionary => LocKind::Interface,
            _ => return Err(self.err_here("dictionary specifier or location kind expected")),
        };
        let symbol = self.qual_ident_node()?;
        self.expect(&Tok::At)?;
        let file = self.literal_string_node()?;
        Ok(SpecLoc {
            kind,
            symbol,
            file,
            is_dictionary,
        })
    }

    fn spec_import(&mut self) -> Result<SpecImport> {
        self.expect(&Tok::Import)?;
        Ok(SpecImport {
            sym: self.qual_ident_node()?,
        })
    }

    // -- components ------------------------------------------------------------------

    fn component_members(&mut self) -> Result<Vec<ComponentMember>> {
        self.annotated_seq(Punct::Semi, |p| p.component_member_node())
    }

    fn component_member_node(&mut self) -> Result<ComponentMemberNode> {
        use ComponentMemberNode as M;
        Ok(match (self.peek(), self.peek_at(1), self.peek_at(2)) {
            (Tok::Dictionary, Tok::Type, _) | (Tok::Type, _, Tok::Equals) => {
                M::DefAliasType(self.with_node(|p| p.def_alias_type())?)
            }
            (Tok::Type, _, _) => M::DefAbsType(self.with_node(|p| p.def_abs_type())?),
            (Tok::Array, _, _) | (Tok::Dictionary, Tok::Array, _) => {
                M::DefArray(self.with_node(|p| p.def_array())?)
            }
            (Tok::Constant, _, _) | (Tok::Dictionary, Tok::Constant, _) => {
                M::DefConstant(self.with_node(|p| p.def_constant())?)
            }
            (Tok::Enum, _, _) | (Tok::Dictionary, Tok::Enum, _) => {
                M::DefEnum(self.with_node(|p| p.def_enum())?)
            }
            (Tok::Struct, _, _) | (Tok::Dictionary, Tok::Struct, _) => {
                M::DefStruct(self.with_node(|p| p.def_struct())?)
            }
            (Tok::State, Tok::Machine, Tok::Instance) => {
                M::SpecStateMachineInstance(self.with_node(|p| p.spec_state_machine_instance())?)
            }
            (Tok::State, _, _) => M::DefStateMachine(self.with_node(|p| p.def_state_machine())?),
            (
                Tok::Async | Tok::Guarded | Tok::Sync,
                Tok::Command,
                Tok::Recv | Tok::Reg | Tok::Resp,
            ) => M::SpecPortInstance(self.with_node(|p| p.spec_port_instance())?),
            (Tok::Async | Tok::Guarded | Tok::Sync, Tok::Command, _) => {
                M::SpecCommand(self.with_node(|p| p.spec_command())?)
            }
            (Tok::Product, Tok::Container, _) => {
                M::SpecContainer(self.with_node(|p| p.spec_container())?)
            }
            (Tok::Product, Tok::Record, _) => M::SpecRecord(self.with_node(|p| p.spec_record())?),
            (Tok::Event, Tok::Port, _) => {
                M::SpecPortInstance(self.with_node(|p| p.spec_port_instance())?)
            }
            (Tok::Event, _, _) => M::SpecEvent(self.with_node(|p| p.spec_event())?),
            (Tok::Include, _, _) => M::SpecInclude(self.with_node(|p| p.spec_include())?),
            (Tok::Internal, _, _) => {
                M::SpecInternalPort(self.with_node(|p| p.spec_internal_port())?)
            }
            (Tok::Match, _, _) => M::SpecPortMatching(self.with_node(|p| p.spec_port_matching())?),
            (Tok::Param, Tok::Get | Tok::Set, _) => {
                M::SpecPortInstance(self.with_node(|p| p.spec_port_instance())?)
            }
            (Tok::Param | Tok::External, _, _) => M::SpecParam(self.with_node(|p| p.spec_param())?),
            (Tok::Telemetry, Tok::Port, _) => {
                M::SpecPortInstance(self.with_node(|p| p.spec_port_instance())?)
            }
            (Tok::Telemetry, _, _) => M::SpecTlmChannel(self.with_node(|p| p.spec_tlm_channel())?),
            (Tok::Import, _, _) => M::SpecImportInterface(self.with_node(|p| p.spec_import())?),
            (
                Tok::Async
                | Tok::Guarded
                | Tok::Sync
                | Tok::Output
                | Tok::Command
                | Tok::Product
                | Tok::Text
                | Tok::Time,
                _,
                _,
            ) => M::SpecPortInstance(self.with_node(|p| p.spec_port_instance())?),
            _ => return Err(self.err_here("component member expected")),
        })
    }

    fn spec_command(&mut self) -> Result<SpecCommand> {
        let kind = match self.advance() {
            Tok::Async => CommandKind::Async,
            Tok::Guarded => CommandKind::Guarded,
            Tok::Sync => CommandKind::Sync,
            _ => return Err(self.err_here("command kind expected")),
        };
        self.expect(&Tok::Command)?;
        let name = self.ident()?;
        let params = self.formal_param_list()?;
        let opcode = if self.accept(&Tok::Opcode) {
            Some(self.expr()?)
        } else {
            None
        };
        let priority = if self.accept(&Tok::Priority) {
            Some(self.expr()?)
        } else {
            None
        };
        let queue_full = self.opt_queue_full_node()?;
        Ok(SpecCommand {
            kind,
            name,
            params,
            opcode,
            priority,
            queue_full,
        })
    }

    fn queue_full(&mut self) -> Result<QueueFull> {
        Ok(match self.advance() {
            Tok::Assert => QueueFull::Assert,
            Tok::Block => QueueFull::Block,
            Tok::Drop => QueueFull::Drop,
            Tok::Hook => QueueFull::Hook,
            _ => return Err(self.err_here("queue full expected")),
        })
    }

    fn at_queue_full(&self) -> bool {
        matches!(
            self.peek(),
            Tok::Assert | Tok::Block | Tok::Drop | Tok::Hook
        )
    }

    fn opt_queue_full_node(&mut self) -> Result<Option<Node<QueueFull>>> {
        if self.at_queue_full() {
            Ok(Some(self.with_node(|p| p.queue_full())?))
        } else {
            Ok(None)
        }
    }

    fn opt_queue_full(&mut self) -> Result<Option<QueueFull>> {
        if self.at_queue_full() {
            Ok(Some(self.queue_full()?))
        } else {
            Ok(None)
        }
    }

    fn spec_container(&mut self) -> Result<SpecContainer> {
        self.expect(&Tok::Product)?;
        self.expect(&Tok::Container)?;
        let name = self.ident()?;
        let id = if self.accept(&Tok::Id) {
            Some(self.expr()?)
        } else {
            None
        };
        let default_priority = if self.at(&Tok::Default) {
            self.advance();
            self.expect(&Tok::Priority)?;
            Some(self.expr()?)
        } else {
            None
        };
        Ok(SpecContainer {
            name,
            id,
            default_priority,
        })
    }

    fn spec_record(&mut self) -> Result<SpecRecord> {
        self.expect(&Tok::Product)?;
        self.expect(&Tok::Record)?;
        let name = self.ident()?;
        self.expect(&Tok::Colon)?;
        let record_type = self.type_name_node()?;
        let is_array = self.accept(&Tok::Array);
        let id = if self.accept(&Tok::Id) {
            Some(self.expr()?)
        } else {
            None
        };
        Ok(SpecRecord {
            name,
            record_type,
            is_array,
            id,
        })
    }

    fn spec_event(&mut self) -> Result<SpecEvent> {
        self.expect(&Tok::Event)?;
        let name = self.ident()?;
        let params = self.formal_param_list()?;
        self.expect(&Tok::Severity)?;
        let severity = match self.advance() {
            Tok::Activity => match self.advance() {
                Tok::High => Severity::ActivityHigh,
                Tok::Low => Severity::ActivityLow,
                _ => return Err(self.err_here("severity level expected")),
            },
            Tok::Command => Severity::Command,
            Tok::Diagnostic => Severity::Diagnostic,
            Tok::Fatal => Severity::Fatal,
            Tok::Warning => match self.advance() {
                Tok::High => Severity::WarningHigh,
                Tok::Low => Severity::WarningLow,
                _ => return Err(self.err_here("severity level expected")),
            },
            _ => return Err(self.err_here("severity level expected")),
        };
        let id = if self.accept(&Tok::Id) {
            Some(self.expr()?)
        } else {
            None
        };
        self.expect(&Tok::Format)?;
        let format = self.literal_string_node()?;
        let throttle = if self.at(&Tok::Throttle) {
            Some(self.with_node(|p| {
                p.expect(&Tok::Throttle)?;
                let count = p.expr()?;
                let every = if p.accept(&Tok::Every) {
                    Some(p.expr()?)
                } else {
                    None
                };
                Ok(EventThrottle { count, every })
            })?)
        } else {
            None
        };
        Ok(SpecEvent {
            name,
            params,
            severity,
            id,
            format,
            throttle,
        })
    }

    fn spec_internal_port(&mut self) -> Result<SpecInternalPort> {
        self.expect(&Tok::Internal)?;
        self.expect(&Tok::Port)?;
        let name = self.ident()?;
        let params = self.formal_param_list()?;
        let priority = if self.accept(&Tok::Priority) {
            Some(self.expr()?)
        } else {
            None
        };
        let queue_full = self.opt_queue_full()?;
        Ok(SpecInternalPort {
            name,
            params,
            priority,
            queue_full,
        })
    }

    fn spec_param(&mut self) -> Result<SpecParam> {
        let is_external = self.accept(&Tok::External);
        self.expect(&Tok::Param)?;
        let name = self.ident()?;
        self.expect(&Tok::Colon)?;
        let type_name = self.type_name_node()?;
        let default = if self.accept(&Tok::Default) {
            Some(self.expr()?)
        } else {
            None
        };
        let id = if self.accept(&Tok::Id) {
            Some(self.expr()?)
        } else {
            None
        };
        let set_opcode = if self.at(&Tok::Set) {
            self.advance();
            self.expect(&Tok::Opcode)?;
            Some(self.expr()?)
        } else {
            None
        };
        let save_opcode = if self.at(&Tok::Save) {
            self.advance();
            self.expect(&Tok::Opcode)?;
            Some(self.expr()?)
        } else {
            None
        };
        Ok(SpecParam {
            name,
            type_name,
            default,
            id,
            set_opcode,
            save_opcode,
            is_external,
        })
    }

    fn spec_port_instance(&mut self) -> Result<SpecPortInstance> {
        // General: (async|guarded|sync) input port | output port
        let general_kind = match (self.peek(), self.peek_at(1)) {
            (Tok::Async, Tok::Input) => Some(GeneralPortKind::AsyncInput),
            (Tok::Guarded, Tok::Input) => Some(GeneralPortKind::GuardedInput),
            (Tok::Sync, Tok::Input) => Some(GeneralPortKind::SyncInput),
            (Tok::Output, _) => Some(GeneralPortKind::Output),
            _ => None,
        };
        if let Some(kind) = general_kind {
            self.advance();
            if kind != GeneralPortKind::Output {
                self.advance();
            }
            self.expect(&Tok::Port)?;
            let name = self.ident()?;
            self.expect(&Tok::Colon)?;
            let size = if self.at(&Tok::LBracket) {
                Some(self.index()?)
            } else {
                None
            };
            let port = if self.accept(&Tok::Serial) {
                None
            } else {
                Some(self.qual_ident_node()?)
            };
            let priority = if self.accept(&Tok::Priority) {
                Some(self.expr()?)
            } else {
                None
            };
            let queue_full = self.opt_queue_full_node()?;
            return Ok(SpecPortInstance::General {
                kind,
                name,
                size,
                port,
                priority,
                queue_full,
            });
        }
        let input_kind = match self.peek() {
            Tok::Async => Some(SpecialInputKind::Async),
            Tok::Guarded => Some(SpecialInputKind::Guarded),
            Tok::Sync => Some(SpecialInputKind::Sync),
            _ => None,
        };
        if input_kind.is_some() {
            self.advance();
        }
        let kind = match self.advance() {
            Tok::Command => match self.advance() {
                Tok::Recv => SpecialPortKind::CommandRecv,
                Tok::Reg => SpecialPortKind::CommandReg,
                Tok::Resp => SpecialPortKind::CommandResp,
                _ => return Err(self.err_here("special port kind expected")),
            },
            Tok::Event => SpecialPortKind::Event,
            Tok::Param => match self.advance() {
                Tok::Get => SpecialPortKind::ParamGet,
                Tok::Set => SpecialPortKind::ParamSet,
                _ => return Err(self.err_here("special port kind expected")),
            },
            Tok::Product => match self.advance() {
                Tok::Get => SpecialPortKind::ProductGet,
                Tok::Recv => SpecialPortKind::ProductRecv,
                Tok::Request => SpecialPortKind::ProductRequest,
                Tok::Send => SpecialPortKind::ProductSend,
                _ => return Err(self.err_here("special port kind expected")),
            },
            Tok::Telemetry => SpecialPortKind::Telemetry,
            Tok::Text => {
                self.expect(&Tok::Event)?;
                SpecialPortKind::TextEvent
            }
            Tok::Time => {
                self.expect(&Tok::Get)?;
                SpecialPortKind::TimeGet
            }
            _ => return Err(self.err_here("port instance specifier expected")),
        };
        self.expect(&Tok::Port)?;
        let name = self.ident()?;
        let priority = if self.accept(&Tok::Priority) {
            Some(self.expr()?)
        } else {
            None
        };
        let queue_full = self.opt_queue_full_node()?;
        Ok(SpecPortInstance::Special {
            input_kind,
            kind,
            name,
            priority,
            queue_full,
        })
    }

    fn spec_port_matching(&mut self) -> Result<SpecPortMatching> {
        self.expect(&Tok::Match)?;
        let port1 = self.ident_node()?;
        self.expect(&Tok::With)?;
        let port2 = self.ident_node()?;
        Ok(SpecPortMatching { port1, port2 })
    }

    fn spec_state_machine_instance(&mut self) -> Result<SpecStateMachineInstance> {
        self.expect(&Tok::State)?;
        self.expect(&Tok::Machine)?;
        self.expect(&Tok::Instance)?;
        let name = self.ident()?;
        self.expect(&Tok::Colon)?;
        let state_machine = self.qual_ident_node()?;
        let priority = if self.accept(&Tok::Priority) {
            Some(self.expr()?)
        } else {
            None
        };
        let queue_full = self.opt_queue_full()?;
        Ok(SpecStateMachineInstance {
            name,
            state_machine,
            priority,
            queue_full,
        })
    }

    fn spec_tlm_channel(&mut self) -> Result<SpecTlmChannel> {
        self.expect(&Tok::Telemetry)?;
        let name = self.ident()?;
        self.expect(&Tok::Colon)?;
        let type_name = self.type_name_node()?;
        let id = if self.accept(&Tok::Id) {
            Some(self.expr()?)
        } else {
            None
        };
        let update = if self.accept(&Tok::Update) {
            Some(match self.advance() {
                Tok::Always => TlmUpdate::Always,
                Tok::On => {
                    self.expect(&Tok::Change)?;
                    TlmUpdate::OnChange
                }
                _ => return Err(self.err_here("update kind expected")),
            })
        } else {
            None
        };
        let format = if self.accept(&Tok::Format) {
            Some(self.literal_string_node()?)
        } else {
            None
        };
        let low = self.limit_seq(&Tok::Low)?;
        let high = self.limit_seq(&Tok::High)?;
        Ok(SpecTlmChannel {
            name,
            type_name,
            id,
            update,
            format,
            low,
            high,
        })
    }

    fn limit_seq(&mut self, kw: &Tok) -> Result<Vec<(Node<LimitKind>, Node<Expr>)>> {
        if !self.accept(kw) {
            return Ok(Vec::new());
        }
        self.braced(|p| {
            p.element_seq(Punct::Comma, |p| {
                let kind = p.with_node(|p| {
                    Ok(match p.advance() {
                        Tok::Orange => LimitKind::Orange,
                        Tok::Red => LimitKind::Red,
                        Tok::Yellow => LimitKind::Yellow,
                        _ => return Err(p.err_here("limit kind expected")),
                    })
                })?;
                let e = p.expr()?;
                Ok((kind, e))
            })
        })
    }

    // -- shared pieces ------------------------------------------------------------------

    fn formal_param_list(&mut self) -> Result<FormalParamList> {
        if !self.accept(&Tok::LParen) {
            return Ok(Vec::new());
        }
        let params = self.annotated_seq(Punct::Comma, |p| {
            p.with_node(|p| {
                let kind = if p.accept(&Tok::Ref) {
                    FormalParamKind::Ref
                } else {
                    FormalParamKind::Value
                };
                let name = p.ident()?;
                p.expect(&Tok::Colon)?;
                let type_name = p.type_name_node()?;
                Ok(FormalParam {
                    kind,
                    name,
                    type_name,
                })
            })
        })?;
        self.expect(&Tok::RParen)?;
        Ok(params)
    }

    fn index(&mut self) -> Result<Node<Expr>> {
        self.expect(&Tok::LBracket)?;
        let e = self.expr()?;
        self.expect(&Tok::RBracket)?;
        Ok(e)
    }

    fn port_instance_identifier(&mut self) -> Result<PortInstanceIdentifier> {
        let loc = self.loc();
        let first = self.ident_node()?;
        self.expect(&Tok::Dot)?;
        let mut parts = vec![first];
        parts.push(self.ident_node()?);
        while self.accept(&Tok::Dot) {
            parts.push(self.ident_node()?);
        }
        let port_name = parts.pop().expect("at least two parts");
        let interface_instance = self.node(loc, QualIdent { parts });
        Ok(PortInstanceIdentifier {
            interface_instance,
            port_name,
        })
    }

    fn tlm_channel_identifier(&mut self) -> Result<TlmChannelIdentifier> {
        let pii = self.port_instance_identifier()?;
        Ok(TlmChannelIdentifier {
            component_instance: pii.interface_instance,
            channel_name: pii.port_name,
        })
    }

    fn qual_ident(&mut self) -> Result<QualIdent> {
        let mut parts = vec![self.ident_node()?];
        while self.at(&Tok::Dot) && matches!(self.peek_at(1), Tok::Ident(_)) {
            self.advance();
            parts.push(self.ident_node()?);
        }
        Ok(QualIdent { parts })
    }

    fn qual_ident_node(&mut self) -> Result<Node<QualIdent>> {
        self.with_node(|p| p.qual_ident())
    }

    fn type_name(&mut self) -> Result<TypeName> {
        Ok(match self.peek() {
            Tok::Bool => {
                self.advance();
                TypeName::Bool
            }
            Tok::String => {
                self.advance();
                let size = if self.accept(&Tok::Size) {
                    Some(Box::new(self.expr()?))
                } else {
                    None
                };
                TypeName::String(size)
            }
            Tok::F32 => {
                self.advance();
                TypeName::Float(FloatKind::F32)
            }
            Tok::F64 => {
                self.advance();
                TypeName::Float(FloatKind::F64)
            }
            Tok::I8 | Tok::I16 | Tok::I32 | Tok::I64 | Tok::U8 | Tok::U16 | Tok::U32 | Tok::U64 => {
                let k = match self.advance() {
                    Tok::I8 => IntKind::I8,
                    Tok::I16 => IntKind::I16,
                    Tok::I32 => IntKind::I32,
                    Tok::I64 => IntKind::I64,
                    Tok::U8 => IntKind::U8,
                    Tok::U16 => IntKind::U16,
                    Tok::U32 => IntKind::U32,
                    _ => IntKind::U64,
                };
                TypeName::Int(k)
            }
            Tok::Ident(_) => TypeName::QualIdent(self.qual_ident_node()?),
            _ => return Err(self.err_here("type name expected")),
        })
    }

    fn type_name_node(&mut self) -> Result<Node<TypeName>> {
        self.with_node(|p| p.type_name())
    }

    // -- expressions --------------------------------------------------------------------

    fn expr(&mut self) -> Result<Node<Expr>> {
        // shift < add/sub < mul/div < unary minus < postfix < primary
        let mut e = self.add_sub_expr()?;
        while matches!(self.peek(), Tok::LShift | Tok::RShift) {
            let loc = self.loc();
            let op = if self.advance() == Tok::LShift {
                Binop::LShift
            } else {
                Binop::RShift
            };
            let rhs = self.add_sub_expr()?;
            e = self.node(loc, Expr::Binop(Box::new(e), op, Box::new(rhs)));
        }
        Ok(e)
    }

    fn add_sub_expr(&mut self) -> Result<Node<Expr>> {
        let mut e = self.mul_div_expr()?;
        while matches!(self.peek(), Tok::Plus | Tok::Minus) {
            let loc = self.loc();
            let op = if self.advance() == Tok::Plus {
                Binop::Add
            } else {
                Binop::Sub
            };
            let rhs = self.mul_div_expr()?;
            e = self.node(loc, Expr::Binop(Box::new(e), op, Box::new(rhs)));
        }
        Ok(e)
    }

    fn mul_div_expr(&mut self) -> Result<Node<Expr>> {
        let mut e = self.unary_expr()?;
        while matches!(self.peek(), Tok::Star | Tok::Slash) {
            let loc = self.loc();
            let op = if self.advance() == Tok::Star {
                Binop::Mul
            } else {
                Binop::Div
            };
            let rhs = self.unary_expr()?;
            e = self.node(loc, Expr::Binop(Box::new(e), op, Box::new(rhs)));
        }
        Ok(e)
    }

    fn unary_expr(&mut self) -> Result<Node<Expr>> {
        if self.at(&Tok::Minus) {
            let loc = self.loc();
            self.advance();
            let e = self.postfix_expr()?;
            return Ok(self.node(loc, Expr::Unop(Unop::Minus, Box::new(e))));
        }
        self.postfix_expr()
    }

    fn postfix_expr(&mut self) -> Result<Node<Expr>> {
        let mut e = self.primary_expr()?;
        loop {
            if self.at(&Tok::Dot) {
                let loc = e.loc.clone();
                self.advance();
                let id = self.ident_node()?;
                e = self.node(loc, Expr::Dot(Box::new(e), id));
            } else if self.at(&Tok::LBracket) {
                let loc = e.loc.clone();
                let index = self.index()?;
                e = self.node(loc, Expr::ArraySubscript(Box::new(e), Box::new(index)));
            } else {
                return Ok(e);
            }
        }
    }

    fn primary_expr(&mut self) -> Result<Node<Expr>> {
        let loc = self.loc();
        let data = match self.peek().clone() {
            Tok::LBracket => {
                self.advance();
                let elts = self.element_seq(Punct::Comma, |p| p.expr())?;
                self.expect(&Tok::RBracket)?;
                Expr::Array(elts)
            }
            Tok::False => {
                self.advance();
                Expr::LiteralBool(false)
            }
            Tok::True => {
                self.advance();
                Expr::LiteralBool(true)
            }
            Tok::LitFloat(s) => {
                self.advance();
                Expr::LiteralFloat(s)
            }
            Tok::LitInt(s) => {
                self.advance();
                Expr::LiteralInt(s)
            }
            Tok::LitString(s) => {
                self.advance();
                Expr::LiteralString(s)
            }
            Tok::Ident(s) => {
                self.advance();
                Expr::Ident(s)
            }
            Tok::LParen => {
                self.advance();
                let e = self.expr()?;
                self.expect(&Tok::RParen)?;
                Expr::Paren(Box::new(e))
            }
            Tok::Sizeof => {
                self.advance();
                self.expect(&Tok::LParen)?;
                let t = self.type_name_node()?;
                self.expect(&Tok::RParen)?;
                Expr::SizeOf(Box::new(t))
            }
            Tok::LBrace => {
                self.advance();
                let members = self.element_seq(Punct::Comma, |p| {
                    p.with_node(|p| {
                        let name = p.ident()?;
                        p.expect(&Tok::Equals)?;
                        let value = p.expr()?;
                        Ok(StructMember { name, value })
                    })
                })?;
                self.expect(&Tok::RBrace)?;
                Expr::Struct(members)
            }
            _ => return Err(self.err_here("expression expected")),
        };
        Ok(self.node(loc, data))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use std::rc::Rc;

    fn parse(src: &str) -> Result<TransUnit> {
        let toks = crate::lexer::lex(Rc::new(PathBuf::from("t.fpp")), src, None)?;
        let mut id = 0;
        parse_trans_unit(toks, &mut id)
    }

    #[test]
    fn expression_precedence() {
        let tu = parse("constant x = 1 + 2 * 3 - -4 * 5 + 6 << 1").unwrap();
        let ModuleMemberNode::DefConstant(c) = &tu.members[0].node else {
            panic!()
        };
        // ((1 + (2*3)) - ((-4)*5)) + 6) << 1
        let Expr::Binop(l, Binop::LShift, _) = &c.data.value.data else {
            panic!("shift outermost")
        };
        let Expr::Binop(_, Binop::Add, _) = &l.data else {
            panic!("add")
        };
    }

    #[test]
    fn annotations_attach_to_elements() {
        let tu = parse("@ pre\nconstant a = 1 @< post\n@ b\nconstant b = 2").unwrap();
        assert_eq!(tu.members[0].pre, vec!["pre"]);
        assert_eq!(tu.members[0].post, vec!["post"]);
        assert_eq!(tu.members[1].pre, vec!["b"]);
    }

    #[test]
    fn component_member_disambiguation() {
        let src = r#"
            active component C {
              async input port p1: [2] P priority 1 drop
              output port p2: P
              sync input port s: serial
              async command CMD(a: U32) opcode 0x10
              product container D id 0 default priority 5
              product record R: U8 array id 1
              event E(x: string size 40) severity warning high format "x {}" throttle 3
              event port eventOut
              telemetry T: U32 update on change format "{d}" low { red 0 } high { yellow 1, red 2 }
              telemetry port tlmOut
              param P: F32 default 1.0
              external param Q: U32 id 4 set opcode 9 save opcode 10
              param get port prmGet
              time get port timeGet
              text event port textOut
              command recv port cmdIn
              async product recv port prIn priority 3 block
              internal port ip(x: U32) priority 2 hook
              state machine instance sm: S
              state machine SM
              match p1 with p2
              import I
            }"#;
        let tu = parse(src).unwrap();
        let ModuleMemberNode::DefComponent(c) = &tu.members[0].node else {
            panic!()
        };
        assert_eq!(c.data.members.len(), 22);
    }

    #[test]
    fn topology_members() {
        let src = r#"
            topology T {
              instance a
              import Other
              connections C { a.p[0] -> b.q, unmatched a.r -> b.s[1] }
              command connections instance a
              text event connections instance b { c, d }
              telemetry packets P { packet X id 1 group 2 { a.c } } omit { b.d }
              port x = a.p
            }"#;
        let tu = parse(src).unwrap();
        let ModuleMemberNode::DefTopology(t) = &tu.members[0].node else {
            panic!()
        };
        assert_eq!(t.data.members.len(), 7);
    }

    #[test]
    fn state_machines() {
        let src = r#"
            state machine M {
              signal s
              action a: U32
              guard g
              initial enter S1
              state S1 {
                entry do { a }
                on s if g do { a } enter S2
                on s do { a }
                exit do { a }
              }
              state S2
              choice c { if g enter S1 else do { a } enter S2 }
            }"#;
        let tu = parse(src).unwrap();
        let ModuleMemberNode::DefStateMachine(m) = &tu.members[0].node else {
            panic!()
        };
        assert_eq!(m.data.members.as_ref().unwrap().len(), 7);
    }

    #[test]
    fn errors_report_locations() {
        let err = parse("module M {\n  constant = 1\n}").unwrap_err();
        assert_eq!(err.phase, Phase::Syntax);
        assert_eq!((err.loc.line, err.loc.col), (2, 12));
    }
}
