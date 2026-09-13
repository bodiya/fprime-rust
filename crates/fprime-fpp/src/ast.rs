//! The FPP abstract syntax tree — a faithful mirror of the reference
//! compiler's `Ast.scala`, so that every construct the language has is
//! representable (including the ones the Rust back end does not generate
//! code for, such as state machines and telemetry packet sets).
//!
//! Every syntactic element is wrapped in a [`Node`] carrying a location and
//! a unique id; the analysis keeps its per-node results (types, values,
//! resolved symbols) in side tables keyed by that id.

// Member enums wrap whole definitions of very different sizes; boxing the
// large ones would only add indirection to a tree that is walked, not
// stored in bulk.
#![allow(clippy::large_enum_variant)]

use crate::error::Loc;
use std::fmt;

/// Unique id of an AST node within one front-end run.
pub type NodeId = u32;

/// An AST node: data plus location plus id.
#[derive(Debug, Clone)]
pub struct Node<T> {
    /// Unique id.
    pub id: NodeId,
    /// Source location of the first token.
    pub loc: Loc,
    /// The payload.
    pub data: T,
}

/// An element with its pre (`@`) and post (`@<`) annotations.
#[derive(Debug, Clone)]
pub struct Annotated<T> {
    /// `@ ...` lines before the element.
    pub pre: Vec<String>,
    /// The element.
    pub node: T,
    /// `@< ...` lines after the element.
    pub post: Vec<String>,
}

impl<T> Annotated<T> {
    /// All annotation lines, pre then post.
    pub fn doc_lines(&self) -> impl Iterator<Item = &String> {
        self.pre.iter().chain(self.post.iter())
    }
}

/// An identifier.
pub type Ident = String;

/// A possibly-qualified identifier `a.b.c`, kept as its parts.
#[derive(Debug, Clone)]
pub struct QualIdent {
    /// The identifier parts, at least one.
    pub parts: Vec<Node<Ident>>,
}

impl QualIdent {
    /// The unqualified (last) name.
    pub fn name(&self) -> &str {
        &self.parts[self.parts.len() - 1].data
    }

    /// The qualifier parts (all but the last).
    pub fn qualifier(&self) -> &[Node<Ident>] {
        &self.parts[..self.parts.len() - 1]
    }

    /// The parts as plain strings.
    pub fn idents(&self) -> Vec<&str> {
        self.parts.iter().map(|p| p.data.as_str()).collect()
    }
}

impl fmt::Display for QualIdent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut first = true;
        for p in &self.parts {
            if !first {
                f.write_str(".")?;
            }
            first = false;
            f.write_str(&p.data)?;
        }
        Ok(())
    }
}

/// A translation unit: the members of one source file.
#[derive(Debug, Clone)]
pub struct TransUnit {
    /// Top-level members (same as module members).
    pub members: Vec<ModuleMember>,
}

/// Binary operators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Binop {
    Add,
    Div,
    Mul,
    Sub,
    LShift,
    RShift,
}

impl fmt::Display for Binop {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Binop::Add => "+",
            Binop::Div => "/",
            Binop::Mul => "*",
            Binop::Sub => "-",
            Binop::LShift => "<<",
            Binop::RShift => ">>",
        })
    }
}

/// Unary operators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unop {
    Minus,
}

/// Component kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentKind {
    Active,
    Passive,
    Queued,
}

impl fmt::Display for ComponentKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ComponentKind::Active => "active",
            ComponentKind::Passive => "passive",
            ComponentKind::Queued => "queued",
        })
    }
}

/// A component member.
pub type ComponentMember = Annotated<ComponentMemberNode>;

/// The kinds of component member.
#[derive(Debug, Clone)]
pub enum ComponentMemberNode {
    DefAbsType(Node<DefAbsType>),
    DefAliasType(Node<DefAliasType>),
    DefArray(Node<DefArray>),
    DefConstant(Node<DefConstant>),
    DefEnum(Node<DefEnum>),
    DefStateMachine(Node<DefStateMachine>),
    DefStruct(Node<DefStruct>),
    SpecCommand(Node<SpecCommand>),
    SpecContainer(Node<SpecContainer>),
    SpecEvent(Node<SpecEvent>),
    SpecInclude(Node<SpecInclude>),
    SpecInternalPort(Node<SpecInternalPort>),
    SpecParam(Node<SpecParam>),
    SpecPortInstance(Node<SpecPortInstance>),
    SpecPortMatching(Node<SpecPortMatching>),
    SpecRecord(Node<SpecRecord>),
    SpecStateMachineInstance(Node<SpecStateMachineInstance>),
    SpecTlmChannel(Node<SpecTlmChannel>),
    SpecImportInterface(Node<SpecImport>),
}

/// `type T`
#[derive(Debug, Clone)]
pub struct DefAbsType {
    pub name: Ident,
}

/// `type T = U`
#[derive(Debug, Clone)]
pub struct DefAliasType {
    pub name: Ident,
    pub type_name: Node<TypeName>,
    pub is_dictionary: bool,
}

/// `array A = [n] T default e format "s"`
#[derive(Debug, Clone)]
pub struct DefArray {
    pub name: Ident,
    pub size: Node<Expr>,
    pub elt_type: Node<TypeName>,
    pub default: Option<Node<Expr>>,
    pub format: Option<Node<String>>,
    pub is_dictionary: bool,
}

/// `kind component C { ... }`
#[derive(Debug, Clone)]
pub struct DefComponent {
    pub kind: ComponentKind,
    pub name: Ident,
    pub members: Vec<ComponentMember>,
}

/// `instance i: C base id e ...`
#[derive(Debug, Clone)]
pub struct DefComponentInstance {
    pub name: Ident,
    pub component: Node<QualIdent>,
    pub base_id: Node<Expr>,
    pub impl_type: Option<Node<String>>,
    pub file: Option<Node<String>>,
    pub queue_size: Option<Node<Expr>>,
    pub stack_size: Option<Node<Expr>>,
    pub priority: Option<Node<Expr>>,
    pub cpu: Option<Node<Expr>>,
    pub init_specs: Vec<Annotated<Node<SpecInit>>>,
}

/// `constant c = e`
#[derive(Debug, Clone)]
pub struct DefConstant {
    pub name: Ident,
    pub value: Node<Expr>,
    pub is_dictionary: bool,
}

/// `enum E : T { ... } default e`
#[derive(Debug, Clone)]
pub struct DefEnum {
    pub name: Ident,
    pub type_name: Option<Node<TypeName>>,
    pub constants: Vec<Annotated<Node<DefEnumConstant>>>,
    pub default: Option<Node<Expr>>,
    pub is_dictionary: bool,
}

/// An enum constant, with optional explicit value.
#[derive(Debug, Clone)]
pub struct DefEnumConstant {
    pub name: Ident,
    pub value: Option<Node<Expr>>,
}

/// `module M { ... }`
#[derive(Debug, Clone)]
pub struct DefModule {
    pub name: Ident,
    pub members: Vec<ModuleMember>,
}

/// A module (or translation unit) member.
pub type ModuleMember = Annotated<ModuleMemberNode>;

/// The kinds of module member.
#[derive(Debug, Clone)]
pub enum ModuleMemberNode {
    DefAbsType(Node<DefAbsType>),
    DefAliasType(Node<DefAliasType>),
    DefArray(Node<DefArray>),
    DefComponent(Node<DefComponent>),
    DefComponentInstance(Node<DefComponentInstance>),
    DefConstant(Node<DefConstant>),
    DefEnum(Node<DefEnum>),
    DefInterface(Node<DefInterface>),
    DefModule(Node<DefModule>),
    DefPort(Node<DefPort>),
    DefStateMachine(Node<DefStateMachine>),
    DefStruct(Node<DefStruct>),
    DefSystem(Node<DefSystem>),
    DefTopology(Node<DefTopology>),
    SpecInclude(Node<SpecInclude>),
    SpecLoc(Node<SpecLoc>),
}

/// `port P(params) -> T`
#[derive(Debug, Clone)]
pub struct DefPort {
    pub name: Ident,
    pub params: FormalParamList,
    pub return_type: Option<Node<TypeName>>,
}

/// `state machine S { ... }` (members absent for an external machine).
#[derive(Debug, Clone)]
pub struct DefStateMachine {
    pub name: Ident,
    pub members: Option<Vec<StateMachineMember>>,
}

/// A state machine member.
pub type StateMachineMember = Annotated<StateMachineMemberNode>;

/// The kinds of state machine member.
#[derive(Debug, Clone)]
pub enum StateMachineMemberNode {
    DefAbsType(Node<DefAbsType>),
    DefAction(Node<DefAction>),
    DefAliasType(Node<DefAliasType>),
    DefArray(Node<DefArray>),
    DefChoice(Node<DefChoice>),
    DefConstant(Node<DefConstant>),
    DefEnum(Node<DefEnum>),
    DefGuard(Node<DefGuard>),
    DefSignal(Node<DefSignal>),
    DefState(Node<DefState>),
    DefStruct(Node<DefStruct>),
    SpecInclude(Node<SpecInclude>),
    SpecInitialTransition(Node<SpecInitialTransition>),
}

/// `action a: T`
#[derive(Debug, Clone)]
pub struct DefAction {
    pub name: Ident,
    pub type_name: Option<Node<TypeName>>,
}

/// `choice c { if g e1 else e2 }`
#[derive(Debug, Clone)]
pub struct DefChoice {
    pub name: Ident,
    pub guard: Node<Ident>,
    pub if_transition: Node<TransitionExpr>,
    pub else_transition: Node<TransitionExpr>,
}

/// `guard g: T`
#[derive(Debug, Clone)]
pub struct DefGuard {
    pub name: Ident,
    pub type_name: Option<Node<TypeName>>,
}

/// `do { a, b } enter S`
#[derive(Debug, Clone)]
pub struct TransitionExpr {
    pub actions: Vec<Node<Ident>>,
    pub target: Node<QualIdent>,
}

/// `signal s: T`
#[derive(Debug, Clone)]
pub struct DefSignal {
    pub name: Ident,
    pub type_name: Option<Node<TypeName>>,
}

/// `state S { ... }`
#[derive(Debug, Clone)]
pub struct DefState {
    pub name: Ident,
    pub members: Vec<StateMember>,
}

/// A state member.
pub type StateMember = Annotated<StateMemberNode>;

/// The kinds of state member.
#[derive(Debug, Clone)]
pub enum StateMemberNode {
    DefChoice(Node<DefChoice>),
    DefState(Node<DefState>),
    SpecInclude(Node<SpecInclude>),
    SpecInitialTransition(Node<SpecInitialTransition>),
    SpecStateEntry(Node<SpecStateEntry>),
    SpecStateExit(Node<SpecStateExit>),
    SpecStateTransition(Node<SpecStateTransition>),
}

/// `initial enter S`
#[derive(Debug, Clone)]
pub struct SpecInitialTransition {
    pub transition: Node<TransitionExpr>,
}

/// `entry do { ... }`
#[derive(Debug, Clone)]
pub struct SpecStateEntry {
    pub actions: Vec<Node<Ident>>,
}

/// `exit do { ... }`
#[derive(Debug, Clone)]
pub struct SpecStateExit {
    pub actions: Vec<Node<Ident>>,
}

/// `on s if g enter T` / `on s do { ... }`
#[derive(Debug, Clone)]
pub struct SpecStateTransition {
    pub signal: Node<Ident>,
    pub guard: Option<Node<Ident>>,
    pub transition_or_do: TransitionOrDo,
}

/// The right-hand side of a transition specifier.
#[derive(Debug, Clone)]
pub enum TransitionOrDo {
    Transition(Node<TransitionExpr>),
    Do(Vec<Node<Ident>>),
}

/// An interface member.
pub type InterfaceMember = Annotated<InterfaceMemberNode>;

/// The kinds of interface member.
#[derive(Debug, Clone)]
pub enum InterfaceMemberNode {
    SpecPortInstance(Node<SpecPortInstance>),
    SpecImportInterface(Node<SpecImport>),
}

/// `interface I { ... }`
#[derive(Debug, Clone)]
pub struct DefInterface {
    pub name: Ident,
    pub members: Vec<InterfaceMember>,
}

/// `struct S { ... } default e`
#[derive(Debug, Clone)]
pub struct DefStruct {
    pub name: Ident,
    pub members: Vec<Annotated<Node<StructTypeMember>>>,
    pub default: Option<Node<Expr>>,
    pub is_dictionary: bool,
}

/// Expressions.
#[derive(Debug, Clone)]
pub enum Expr {
    Array(Vec<Node<Expr>>),
    ArraySubscript(Box<Node<Expr>>, Box<Node<Expr>>),
    Binop(Box<Node<Expr>>, Binop, Box<Node<Expr>>),
    Dot(Box<Node<Expr>>, Node<Ident>),
    Ident(Ident),
    LiteralBool(bool),
    LiteralFloat(String),
    LiteralInt(String),
    LiteralString(String),
    Paren(Box<Node<Expr>>),
    SizeOf(Box<Node<TypeName>>),
    Struct(Vec<Node<StructMember>>),
    Unop(Unop, Box<Node<Expr>>),
}

/// `topology T implements I { ... }`
#[derive(Debug, Clone)]
pub struct DefTopology {
    pub is_deployment: bool,
    pub name: Ident,
    pub members: Vec<TopologyMember>,
    pub implements: Vec<Node<QualIdent>>,
}

/// A topology member.
pub type TopologyMember = Annotated<TopologyMemberNode>;

/// The kinds of topology member.
#[derive(Debug, Clone)]
pub enum TopologyMemberNode {
    SpecInstance(Node<SpecInstance>),
    SpecConnectionGraph(Node<SpecConnectionGraph>),
    SpecInclude(Node<SpecInclude>),
    SpecTopPort(Node<SpecTopPort>),
    SpecTlmPacketSet(Node<SpecTlmPacketSet>),
}

/// `system S: T`
#[derive(Debug, Clone)]
pub struct DefSystem {
    pub name: Ident,
    pub topology: Node<QualIdent>,
}

/// A formal parameter of a port or command or event or internal port.
#[derive(Debug, Clone)]
pub struct FormalParam {
    pub kind: FormalParamKind,
    pub name: Ident,
    pub type_name: Node<TypeName>,
}

/// Formal parameter passing kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormalParamKind {
    Ref,
    Value,
}

/// A formal parameter list.
pub type FormalParamList = Vec<Annotated<Node<FormalParam>>>;

/// `instance.port`
#[derive(Debug, Clone)]
pub struct PortInstanceIdentifier {
    pub interface_instance: Node<QualIdent>,
    pub port_name: Node<Ident>,
}

impl fmt::Display for PortInstanceIdentifier {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}.{}",
            self.interface_instance.data, self.port_name.data
        )
    }
}

/// Queue-full behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueFull {
    Assert,
    Block,
    Drop,
    Hook,
}

/// Command kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandKind {
    Async,
    Guarded,
    Sync,
}

/// `kind command C(params) opcode e priority e queueFull`
#[derive(Debug, Clone)]
pub struct SpecCommand {
    pub kind: CommandKind,
    pub name: Ident,
    pub params: FormalParamList,
    pub opcode: Option<Node<Expr>>,
    pub priority: Option<Node<Expr>>,
    pub queue_full: Option<Node<QueueFull>>,
}

/// `instance i` / `import T` inside a topology.
#[derive(Debug, Clone)]
pub struct SpecInstance {
    pub instance: Node<QualIdent>,
}

/// Connection graph specifiers.
#[derive(Debug, Clone)]
pub enum SpecConnectionGraph {
    Direct {
        name: Ident,
        connections: Vec<Connection>,
    },
    Pattern {
        kind: PatternKind,
        source: Node<QualIdent>,
        targets: Vec<Node<QualIdent>>,
    },
}

/// Pattern graph kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatternKind {
    Command,
    Event,
    Health,
    Param,
    Telemetry,
    TextEvent,
    Time,
}

impl fmt::Display for PatternKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            PatternKind::Command => "command",
            PatternKind::Event => "event",
            PatternKind::Health => "health",
            PatternKind::Param => "param",
            PatternKind::Telemetry => "telemetry",
            PatternKind::TextEvent => "text event",
            PatternKind::Time => "time",
        })
    }
}

/// `[unmatched] a.p[i] -> b.q[j]`
#[derive(Debug, Clone)]
pub struct Connection {
    pub is_unmatched: bool,
    pub from_port: Node<PortInstanceIdentifier>,
    pub from_index: Option<Node<Expr>>,
    pub to_port: Node<PortInstanceIdentifier>,
    pub to_index: Option<Node<Expr>>,
}

/// `product container C id e default priority e`
#[derive(Debug, Clone)]
pub struct SpecContainer {
    pub name: Ident,
    pub id: Option<Node<Expr>>,
    pub default_priority: Option<Node<Expr>>,
}

/// `throttle n every e`
#[derive(Debug, Clone)]
pub struct EventThrottle {
    pub count: Node<Expr>,
    pub every: Option<Node<Expr>>,
}

/// Event severities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    ActivityHigh,
    ActivityLow,
    Command,
    Diagnostic,
    Fatal,
    WarningHigh,
    WarningLow,
}

impl fmt::Display for Severity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Severity::ActivityHigh => "activity high",
            Severity::ActivityLow => "activity low",
            Severity::Command => "command",
            Severity::Diagnostic => "diagnostic",
            Severity::Fatal => "fatal",
            Severity::WarningHigh => "warning high",
            Severity::WarningLow => "warning low",
        })
    }
}

/// `event E(params) severity s id e format "s" throttle n`
#[derive(Debug, Clone)]
pub struct SpecEvent {
    pub name: Ident,
    pub params: FormalParamList,
    pub severity: Severity,
    pub id: Option<Node<Expr>>,
    pub format: Node<String>,
    pub throttle: Option<Node<EventThrottle>>,
}

/// `include "file"`
#[derive(Debug, Clone)]
pub struct SpecInclude {
    pub file: Node<String>,
}

/// `phase e "code"`
#[derive(Debug, Clone)]
pub struct SpecInit {
    pub phase: Node<Expr>,
    pub code: String,
}

/// `internal port P(params) priority e queueFull`
#[derive(Debug, Clone)]
pub struct SpecInternalPort {
    pub name: Ident,
    pub params: FormalParamList,
    pub priority: Option<Node<Expr>>,
    pub queue_full: Option<QueueFull>,
}

/// Location specifier kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocKind {
    Component,
    Instance,
    Constant,
    Port,
    StateMachine,
    System,
    Type,
    Interface,
}

/// `locate kind S at "file"`
#[derive(Debug, Clone)]
pub struct SpecLoc {
    pub kind: LocKind,
    pub symbol: Node<QualIdent>,
    pub file: Node<String>,
    pub is_dictionary: bool,
}

/// `[external] param P: T default e id e set opcode e save opcode e`
#[derive(Debug, Clone)]
pub struct SpecParam {
    pub name: Ident,
    pub type_name: Node<TypeName>,
    pub default: Option<Node<Expr>>,
    pub id: Option<Node<Expr>>,
    pub set_opcode: Option<Node<Expr>>,
    pub save_opcode: Option<Node<Expr>>,
    pub is_external: bool,
}

/// Port instance specifiers.
#[derive(Debug, Clone)]
pub enum SpecPortInstance {
    /// A general (typed or serial) port instance.
    General {
        kind: GeneralPortKind,
        name: Ident,
        size: Option<Node<Expr>>,
        /// `None` for `serial` ports.
        port: Option<Node<QualIdent>>,
        priority: Option<Node<Expr>>,
        queue_full: Option<Node<QueueFull>>,
    },
    /// A special (framework-role) port instance.
    Special {
        input_kind: Option<SpecialInputKind>,
        kind: SpecialPortKind,
        name: Ident,
        priority: Option<Node<Expr>>,
        queue_full: Option<Node<QueueFull>>,
    },
}

impl SpecPortInstance {
    /// The port instance name.
    pub fn name(&self) -> &str {
        match self {
            SpecPortInstance::General { name, .. } | SpecPortInstance::Special { name, .. } => name,
        }
    }
}

/// General port instance kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeneralPortKind {
    AsyncInput,
    GuardedInput,
    Output,
    SyncInput,
}

impl fmt::Display for GeneralPortKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            GeneralPortKind::AsyncInput => "async input",
            GeneralPortKind::GuardedInput => "guarded input",
            GeneralPortKind::Output => "output",
            GeneralPortKind::SyncInput => "sync input",
        })
    }
}

/// Special port input kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpecialInputKind {
    Async,
    Guarded,
    Sync,
}

/// Special port kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SpecialPortKind {
    CommandRecv,
    CommandReg,
    CommandResp,
    Event,
    ParamGet,
    ParamSet,
    ProductGet,
    ProductRecv,
    ProductRequest,
    ProductSend,
    Telemetry,
    TextEvent,
    TimeGet,
}

impl fmt::Display for SpecialPortKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SpecialPortKind::CommandRecv => "command recv",
            SpecialPortKind::CommandReg => "command reg",
            SpecialPortKind::CommandResp => "command resp",
            SpecialPortKind::Event => "event",
            SpecialPortKind::ParamGet => "param get",
            SpecialPortKind::ParamSet => "param set",
            SpecialPortKind::ProductGet => "product get",
            SpecialPortKind::ProductRecv => "product recv",
            SpecialPortKind::ProductRequest => "product request",
            SpecialPortKind::ProductSend => "product send",
            SpecialPortKind::Telemetry => "telemetry",
            SpecialPortKind::TextEvent => "text event",
            SpecialPortKind::TimeGet => "time get",
        })
    }
}

/// `match p1 with p2`
#[derive(Debug, Clone)]
pub struct SpecPortMatching {
    pub port1: Node<Ident>,
    pub port2: Node<Ident>,
}

/// `product record R: T [array] id e`
#[derive(Debug, Clone)]
pub struct SpecRecord {
    pub name: Ident,
    pub record_type: Node<TypeName>,
    pub is_array: bool,
    pub id: Option<Node<Expr>>,
}

/// `state machine instance s: S priority e queueFull`
#[derive(Debug, Clone)]
pub struct SpecStateMachineInstance {
    pub name: Ident,
    pub state_machine: Node<QualIdent>,
    pub priority: Option<Node<Expr>>,
    pub queue_full: Option<QueueFull>,
}

/// Telemetry update kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TlmUpdate {
    Always,
    OnChange,
}

/// Telemetry limit kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitKind {
    Red,
    Orange,
    Yellow,
}

/// `telemetry C: T id e update u format "s" low { .. } high { .. }`
#[derive(Debug, Clone)]
pub struct SpecTlmChannel {
    pub name: Ident,
    pub type_name: Node<TypeName>,
    pub id: Option<Node<Expr>>,
    pub update: Option<TlmUpdate>,
    pub format: Option<Node<String>>,
    pub low: Vec<(Node<LimitKind>, Node<Expr>)>,
    pub high: Vec<(Node<LimitKind>, Node<Expr>)>,
}

/// `packet P id e group e { ... }`
#[derive(Debug, Clone)]
pub struct SpecTlmPacket {
    pub name: Ident,
    pub id: Option<Node<Expr>>,
    pub group: Node<Expr>,
    pub members: Vec<TlmPacketMember>,
}

/// `telemetry packets P { ... } omit { ... }`
#[derive(Debug, Clone)]
pub struct SpecTlmPacketSet {
    pub name: Ident,
    pub members: Vec<TlmPacketSetMember>,
    pub omitted: Vec<Node<TlmChannelIdentifier>>,
}

/// `port p = i.q`
#[derive(Debug, Clone)]
pub struct SpecTopPort {
    pub name: Ident,
    pub underlying_port: Node<PortInstanceIdentifier>,
}

/// `import I`
#[derive(Debug, Clone)]
pub struct SpecImport {
    pub sym: Node<QualIdent>,
}

/// `name = value` inside a struct expression.
#[derive(Debug, Clone)]
pub struct StructMember {
    pub name: Ident,
    pub value: Node<Expr>,
}

/// A struct type member `name: [n] T format "s"`.
#[derive(Debug, Clone)]
pub struct StructTypeMember {
    pub name: Ident,
    pub size: Option<Node<Expr>>,
    pub type_name: Node<TypeName>,
    pub format: Option<Node<String>>,
}

/// `instance.channel`
#[derive(Debug, Clone)]
pub struct TlmChannelIdentifier {
    pub component_instance: Node<QualIdent>,
    pub channel_name: Node<Ident>,
}

/// A telemetry packet member.
#[derive(Debug, Clone)]
pub enum TlmPacketMember {
    SpecInclude(Node<SpecInclude>),
    TlmChannelIdentifier(Node<TlmChannelIdentifier>),
}

/// A telemetry packet set member.
pub type TlmPacketSetMember = Annotated<TlmPacketSetMemberNode>;

/// The kinds of telemetry packet set member.
#[derive(Debug, Clone)]
pub enum TlmPacketSetMemberNode {
    SpecInclude(Node<SpecInclude>),
    SpecTlmPacket(Node<SpecTlmPacket>),
}

/// Float types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FloatKind {
    F32,
    F64,
}

/// Integer types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IntKind {
    I8,
    I16,
    I32,
    I64,
    U8,
    U16,
    U32,
    U64,
}

impl IntKind {
    /// Bit width.
    pub fn bits(self) -> u32 {
        match self {
            IntKind::I8 | IntKind::U8 => 8,
            IntKind::I16 | IntKind::U16 => 16,
            IntKind::I32 | IntKind::U32 => 32,
            IntKind::I64 | IntKind::U64 => 64,
        }
    }

    /// Whether the type is signed.
    pub fn signed(self) -> bool {
        matches!(
            self,
            IntKind::I8 | IntKind::I16 | IntKind::I32 | IntKind::I64
        )
    }

    /// The FPP spelling.
    pub fn fpp_name(self) -> &'static str {
        match self {
            IntKind::I8 => "I8",
            IntKind::I16 => "I16",
            IntKind::I32 => "I32",
            IntKind::I64 => "I64",
            IntKind::U8 => "U8",
            IntKind::U16 => "U16",
            IntKind::U32 => "U32",
            IntKind::U64 => "U64",
        }
    }

    /// The Rust spelling.
    pub fn rust_name(self) -> &'static str {
        match self {
            IntKind::I8 => "i8",
            IntKind::I16 => "i16",
            IntKind::I32 => "i32",
            IntKind::I64 => "i64",
            IntKind::U8 => "u8",
            IntKind::U16 => "u16",
            IntKind::U32 => "u32",
            IntKind::U64 => "u64",
        }
    }
}

/// Type names.
#[derive(Debug, Clone)]
pub enum TypeName {
    Float(FloatKind),
    Int(IntKind),
    QualIdent(Node<QualIdent>),
    Bool,
    String(Option<Box<Node<Expr>>>),
}
