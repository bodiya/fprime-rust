//! Emission of a component's state machine instances: the C++
//! autocoder's `SmId`, `<m>_sendSignal_<s>`, `<m>_stateMachineInvoke`,
//! `<m>_getState`, the overflow hooks, the `smDispatch` arms of
//! `doDispatch` and the state machine part of `init`, written as the
//! contract in `docs/api-notes.md` ("fprime-comp: state machines")
//! specifies and `fprime-comp/tests/sm_instance.rs` / `sm_external.rs`
//! show by hand.
//!
//! For component `C` with instances:
//!
//! - `CBase` holds each instance as a field (`M::StateMachine` for an
//!   internal machine, the bound Rust type for an external one), the
//!   `SM_ID_<M>` constants (`component_sm_ids!`, every instance sorted by
//!   name) and `<m>_get_state()`;
//! - `CHandlers` declares the actions and guards of every internal machine
//!   type as `<fq>_action_<a>` / `<fq>_guard_<g>` (`fq` the machine's
//!   qualified name with `.` replaced by `_`), and
//!   `<m>_state_machine_overflow_hook` for `hook` instances;
//! - `CComponent` provides `init` (machines in `SmId` order, then the
//!   queue), `<m>_send_signal_<s>` (internal) and
//!   `<m>_state_machine_invoke` (external), and the dispatch arms;
//! - `CSmHandlers<'_, H>` adapts an implementation to each machine's
//!   `Handlers` trait (one impl per machine type), and, with external
//!   instances, `CSmParents` hands each external machine its parent
//!   interface (implemented by `impl_c_component!`, where the concrete
//!   type is known and `&Self` coerces to the machine's `Parent`).

use super::Generator;
use super::names::{ident, snake, snake_ident};
use crate::analysis::{ComponentModel, StateMachineInstanceDef, SymId};
use crate::ast::{ComponentKind, QueueFull};
use crate::error::{Diagnostic, Result};

/// The names of the generated component items.
pub(super) struct SmNames<'n> {
    /// Component name.
    pub name: &'n str,
    /// `CBase`.
    pub base: &'n str,
    /// `CHandlers`.
    pub handlers: &'n str,
    /// Component kind.
    pub kind: ComponentKind,
}

impl Generator<'_, '_> {
    /// Whether an instance's machine is internal.
    pub(super) fn sm_is_internal(&self, sm: SymId) -> bool {
        self.a.state_machines.contains_key(&sm)
    }

    /// The instances sorted by name (the `SmId` order).
    pub(super) fn sm_instances_sorted(m: &ComponentModel) -> Vec<StateMachineInstanceDef> {
        let mut v = m.state_machine_instances.clone();
        v.sort_by(|a, b| a.name.cmp(&b.name));
        v
    }

    /// `SM_ID_<NAME>`.
    pub(super) fn sm_id_const(name: &str) -> String {
        format!("SM_ID_{}", snake(name).to_ascii_uppercase())
    }

    /// The distinct internal machine types of a component, in order of
    /// first use.
    fn sm_internal_types(&self, m: &ComponentModel) -> Vec<SymId> {
        let mut out: Vec<SymId> = Vec::new();
        for i in &m.state_machine_instances {
            if self.sm_is_internal(i.state_machine) && !out.contains(&i.state_machine) {
                out.push(i.state_machine);
            }
        }
        out
    }

    /// The Rust type of an external machine (its binding).
    fn sm_external_type(&self, i: &StateMachineInstanceDef) -> Result<String> {
        let s = self.a.symbols.sym(i.state_machine);
        match self.opts.bindings.types.get(&s.qualified_name()) {
            Some(b) => Ok(b.path.clone()),
            None => Err(Diagnostic::codegen(
                i.loc.clone(),
                format!(
                    "external state machine {} has no Rust binding; pass --bind-type {}=<type implementing fprime_comp::sm::ExternalStateMachine>",
                    s.qualified_name(),
                    s.qualified_name()
                ),
            )),
        }
    }

    /// The field type of an instance.
    fn sm_field_type(&self, i: &StateMachineInstanceDef) -> Result<String> {
        if self.sm_is_internal(i.state_machine) {
            Ok(self.sm_item(i.state_machine, "StateMachine"))
        } else {
            self.sm_external_type(i)
        }
    }

    /// `<fq>`: the machine's qualified name with `.` replaced by `_`.
    fn sm_fq(&self, sm: SymId) -> String {
        self.a.symbols.sym(sm).qualified.join("_")
    }

    /// The queue-message size expression of the state machine messages
    /// (one entry per internal machine type, one for external ones).
    pub(super) fn sm_msg_sizes(&self, m: &ComponentModel, internal: bool) -> Vec<String> {
        if internal {
            self.sm_internal_types(m)
                .into_iter()
                .map(|s| format!("{}::SIGNAL_MSG_SIZE", self.sm_item(s, "StateMachine")))
                .collect()
        } else {
            vec!["::fprime_comp::sm::EXTERNAL_SIGNAL_MSG_SIZE".into()]
        }
    }

    /// Base struct fields.
    pub(super) fn emit_sm_fields(&mut self, m: &ComponentModel) -> Result<()> {
        for i in &m.state_machine_instances {
            let t = self.sm_field_type(i)?;
            self.doc(&i.docs);
            self.line(&format!(
                "/// State machine instance `{}` (C++ `m_stateMachine_{}`).",
                i.name, i.name
            ));
            self.line(&format!("pub {}: {t},", snake_ident(&i.name)));
        }
        Ok(())
    }

    /// Base field initializers in `new`.
    pub(super) fn emit_sm_field_inits(&mut self, m: &ComponentModel) -> Result<()> {
        for i in &m.state_machine_instances {
            let t = self.sm_field_type(i)?;
            if self.sm_is_internal(i.state_machine) {
                self.line(&format!("{}: {t}::new(),", snake_ident(&i.name)));
            } else {
                self.line(&format!(
                    "{}: <{t} as ::core::default::Default>::default(),",
                    snake_ident(&i.name)
                ));
            }
        }
        Ok(())
    }

    /// `component_sm_ids!` for the base.
    pub(super) fn emit_sm_ids(&mut self, m: &ComponentModel, base: &str) {
        if m.state_machine_instances.is_empty() {
            return;
        }
        self.line("");
        self.line("::fprime_comp::component_sm_ids! {");
        self.line(
            "    /// State machine ids (C++ `SmId`): every instance, sorted by name, from 0.",
        );
        self.line(&format!("    impl {base} {{"));
        for i in Self::sm_instances_sorted(m) {
            self.line(&format!("        /// Instance `{}`.", i.name));
            self.line(&format!("        {},", Self::sm_id_const(&i.name)));
        }
        self.line("    }");
        self.line("}");
    }

    /// `<m>_get_state()` on the base.
    pub(super) fn emit_sm_base_methods(&mut self, m: &ComponentModel) {
        for i in &m.state_machine_instances {
            if !self.sm_is_internal(i.state_machine) {
                continue;
            }
            let state = self.sm_item(i.state_machine, "State");
            self.line(&format!(
                "/// The current state of instance `{}` (C++ `{}_getState`).",
                i.name, i.name
            ));
            self.line(&format!(
                "pub fn {}_get_state(&self) -> {state} {{",
                snake(&i.name)
            ));
            self.line(&format!("    self.{}.get_state()", snake_ident(&i.name)));
            self.line("}");
        }
    }

    /// Handler trait members: actions, guards, overflow hooks.
    pub(super) fn emit_sm_handler_decls(&mut self, m: &ComponentModel) -> Result<()> {
        for sm in self.sm_internal_types(m) {
            let model = self.sm_model(sm).expect("internal").clone();
            let fq = self.sm_fq(sm);
            let signal = self.sm_item(sm, "Signal");
            let qn = self.a.symbols.sym(sm).qualified_name();
            for a in &model.actions {
                self.doc(&a.docs);
                self.line(&format!(
                    "/// Action `{}` of state machine `{qn}` (C++ `{fq}_action_{}`).",
                    a.name, a.name
                ));
                let v = match &a.ty {
                    Some(t) => format!(", value: {}", self.sm_param_type(t)?),
                    None => String::new(),
                };
                self.line(&format!(
                    "fn {fq}_action_{}(&self, sm_id: ::fprime_config::FwEnumStoreType, signal: {signal}{v});",
                    a.name
                ));
            }
            for g in &model.guards {
                self.doc(&g.docs);
                self.line(&format!(
                    "/// Guard `{}` of state machine `{qn}` (C++ `{fq}_guard_{}`).",
                    g.name, g.name
                ));
                let v = match &g.ty {
                    Some(t) => format!(", value: {}", self.sm_param_type(t)?),
                    None => String::new(),
                };
                self.line(&format!(
                    "fn {fq}_guard_{}(&self, sm_id: ::fprime_config::FwEnumStoreType, signal: {signal}{v}) -> bool;",
                    g.name
                ));
            }
        }
        for i in &m.state_machine_instances {
            if i.queue_full != QueueFull::Hook {
                continue;
            }
            if self.sm_is_internal(i.state_machine) {
                self.line(&format!("/// Called on the sender's thread when the queue is full for `hook` instance `{}` (C++ `{}_stateMachineOverflowHook`): the message's cursor is at the signal data.", i.name, i.name));
                self.line(&format!(
                    "fn {}_state_machine_overflow_hook(&self, sm_id: ::fprime_config::FwEnumStoreType, signal: ::fprime_config::FwEnumStoreType, msg: &mut dyn ::fprime_fw::SerBufAny);",
                    snake(&i.name)
                ));
            } else {
                self.line(&format!("/// Called on the sender's thread when the queue is full for `hook` instance `{}` (C++ `{}_stateMachineOverflowHook`).", i.name, i.name));
                self.line(&format!(
                    "fn {}_state_machine_overflow_hook(&self, signal: ::fprime_config::FwEnumStoreType, data: &::fprime_fw::SmSignalBuffer);",
                    snake(&i.name)
                ));
            }
        }
        Ok(())
    }

    /// `CSmHandlers` and, with external instances, `CSmParents`.
    pub(super) fn emit_sm_adapters(&mut self, m: &ComponentModel, n: &SmNames<'_>) -> Result<()> {
        let internal = self.sm_internal_types(m);
        if !internal.is_empty() {
            let adapter = format!("{}SmHandlers", n.name);
            self.line("");
            self.line(&format!("/// Adapts an implementation of `{}` to the `Handlers` trait of each of its internal state machines: `<fq>_action_<a>` and `<fq>_guard_<g>` of the handler trait.", n.handlers));
            self.line(&format!("pub struct {adapter}<'a, H: ?Sized>(pub &'a H);"));
            for sm in internal {
                let model = self.sm_model(sm).expect("internal").clone();
                let fq = self.sm_fq(sm);
                let signal = self.sm_item(sm, "Signal");
                let tr = self.sm_item(sm, "Handlers");
                self.line(&format!(
                    "impl<H: {} + ?Sized> {tr} for {adapter}<'_, H> {{",
                    n.handlers
                ));
                self.indent();
                for a in &model.actions {
                    let (v, arg) = match &a.ty {
                        Some(t) => (format!(", value: {}", self.sm_param_type(t)?), ", value"),
                        None => (String::new(), ""),
                    };
                    self.line(&format!("fn action_{}(&self, sm_id: ::fprime_config::FwEnumStoreType, signal: {signal}{v}) {{", a.name));
                    self.line(&format!(
                        "    self.0.{fq}_action_{}(sm_id, signal{arg})",
                        a.name
                    ));
                    self.line("}");
                }
                for g in &model.guards {
                    let (v, arg) = match &g.ty {
                        Some(t) => (format!(", value: {}", self.sm_param_type(t)?), ", value"),
                        None => (String::new(), ""),
                    };
                    self.line(&format!("fn guard_{}(&self, sm_id: ::fprime_config::FwEnumStoreType, signal: {signal}{v}) -> bool {{", g.name));
                    self.line(&format!(
                        "    self.0.{fq}_guard_{}(sm_id, signal{arg})",
                        g.name
                    ));
                    self.line("}");
                }
                self.dedent();
                self.line("}");
            }
        }
        let externals: Vec<StateMachineInstanceDef> = m
            .state_machine_instances
            .iter()
            .filter(|i| !self.sm_is_internal(i.state_machine))
            .cloned()
            .collect();
        if !externals.is_empty() {
            self.line("");
            self.line(&format!("/// The parent interface each external state machine of `{}` calls back into (C++: the component base inherits `<M>_Interface`). Implemented by `impl_{}_component!`.", n.name, snake(n.name)));
            self.line(&format!("pub trait {}SmParents {{", n.name));
            self.indent();
            for i in &externals {
                let t = self.sm_external_type(i)?;
                self.line(&format!(
                    "/// The parent of external instance `{}`.",
                    i.name
                ));
                self.line(&format!(
                    "fn __{}_sm_parent(&self) -> &<{t} as ::fprime_comp::sm::ExternalStateMachine>::Parent;",
                    snake(&i.name)
                ));
            }
            self.dedent();
            self.line("}");
        }
        Ok(())
    }

    /// Whether the component has external instances (its component trait
    /// then requires `CSmParents`).
    pub(super) fn sm_has_external(&self, m: &ComponentModel) -> bool {
        m.state_machine_instances
            .iter()
            .any(|i| !self.sm_is_internal(i.state_machine))
    }

    /// `impl CSmParents for $t` inside the impl macro, with the absolute
    /// path prefix of the generated module (`prefix::`).
    pub(super) fn emit_sm_parents_impl(
        &mut self,
        m: &ComponentModel,
        name: &str,
        prefix: &str,
    ) -> Result<()> {
        if !self.sm_has_external(m) {
            return Ok(());
        }
        self.line(&format!("impl {prefix}{name}SmParents for $t {{"));
        for i in &m.state_machine_instances {
            if self.sm_is_internal(i.state_machine) {
                continue;
            }
            let t = self.sm_external_type(i)?;
            self.line(&format!(
                "    fn __{}_sm_parent(&self) -> &<{t} as ::fprime_comp::sm::ExternalStateMachine>::Parent {{ self }}",
                snake(&i.name)
            ));
        }
        self.line("}");
        Ok(())
    }

    /// Component trait members: `init`, the senders.
    pub(super) fn emit_sm_component_fns(
        &mut self,
        m: &ComponentModel,
        n: &SmNames<'_>,
        serial_async: bool,
    ) -> Result<()> {
        let base = n.base;
        let q = match n.kind {
            ComponentKind::Queued => "queued",
            _ => "active.queued",
        };
        // init
        let (params, args) = if serial_async {
            (
                "queue_depth: ::fprime_config::FwSizeType, msg_size: ::fprime_config::FwSizeType",
                "queue_depth, msg_size",
            )
        } else {
            ("queue_depth: ::fprime_config::FwSizeType", "queue_depth")
        };
        self.line("/// Initialize the component (C++ `init`): every state machine instance in `SmId` order (running its initial transition), then the queue — so an initial transition cannot send signals through the queue.");
        self.line(&format!("fn init(&self, {params}) {{"));
        self.indent();
        self.line("let fpp_base = self.base();");
        if !self.sm_internal_types(m).is_empty() {
            self.line(&format!("let fpp_h = {}SmHandlers(self);", n.name));
        }
        for i in Self::sm_instances_sorted(m) {
            let id = format!("{base}::{}", Self::sm_id_const(&i.name));
            let f = snake_ident(&i.name);
            if self.sm_is_internal(i.state_machine) {
                self.line(&format!("fpp_base.{f}.init({id}, &fpp_h);"));
            } else {
                let t = self.sm_external_type(&i)?;
                self.line(&format!(
                    "<{t} as ::fprime_comp::sm::ExternalStateMachine>::init(&fpp_base.{f}, self.__{}_sm_parent(), {id});",
                    snake(&i.name)
                ));
            }
        }
        self.line(&format!("fpp_base.init({args});"));
        self.dedent();
        self.line("}");

        for i in &m.state_machine_instances {
            let id = format!("{base}::{}", Self::sm_id_const(&i.name));
            let policy = match i.queue_full {
                QueueFull::Assert => "Assert",
                QueueFull::Block => "Block",
                QueueFull::Drop => "Drop",
                QueueFull::Hook => "Hook",
            };
            let prio = i.priority.unwrap_or(0);
            let hook = i.queue_full == QueueFull::Hook;
            let plain = snake(&i.name);
            if self.sm_is_internal(i.state_machine) {
                let model = self.sm_model(i.state_machine).expect("internal").clone();
                let sender = self.sm_item(i.state_machine, "Sender");
                for sig in &model.signals {
                    let (v, arg) = match &sig.ty {
                        Some(t) => (format!(", value: {}", self.sm_param_type(t)?), "value"),
                        None => (String::new(), ""),
                    };
                    self.doc(&sig.docs);
                    self.line(&format!(
                        "/// Send signal `{}` to instance `{}` through the queue (C++ `{}_sendSignal_{}`).",
                        sig.name, i.name, i.name, sig.name
                    ));
                    self.line(&format!("fn {plain}_send_signal_{}(&self{v}) {{", sig.name));
                    self.indent();
                    self.line("let fpp_base = self.base();");
                    if hook {
                        self.line(&format!("let fpp_hook = |fpp_sm_id: ::fprime_config::FwEnumStoreType, fpp_signal: ::fprime_config::FwEnumStoreType, fpp_msg: &mut dyn ::fprime_fw::SerBufAny| self.{plain}_state_machine_overflow_hook(fpp_sm_id, fpp_signal, fpp_msg);"));
                    }
                    self.line(&format!(
                        "let fpp_sender = ::fprime_comp::sm::SmSender::<'_, {{ {base}::MSG_SIZE }}>::new(&fpp_base.{q}, {base}::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL, {id}, {prio} as ::fprime_config::FwQueuePriorityType, ::fprime_comp::QueueFullPolicy::{policy}){};",
                        if hook { ".with_overflow_hook(&fpp_hook)" } else { "" }
                    ));
                    self.line(&format!(
                        "{sender}(fpp_sender).send_signal_{}({arg});",
                        sig.name
                    ));
                    self.dedent();
                    self.line("}");
                }
            } else {
                let t = self.sm_external_type(i)?;
                self.line(&format!(
                    "/// Send a signal with its data buffer to external instance `{}` through the queue (C++ `{}_stateMachineInvoke`).",
                    i.name, i.name
                ));
                self.line(&format!(
                    "fn {plain}_state_machine_invoke(&self, signal: <{t} as ::fprime_comp::sm::ExternalStateMachine>::Signal, data: &::fprime_fw::SmSignalBuffer) {{"
                ));
                self.indent();
                self.line("let fpp_base = self.base();");
                if hook {
                    self.line(&format!("let fpp_hook = |_fpp_sm_id: ::fprime_config::FwEnumStoreType, fpp_signal: ::fprime_config::FwEnumStoreType, fpp_data: &::fprime_fw::SmSignalBuffer| self.{plain}_state_machine_overflow_hook(fpp_signal, fpp_data);"));
                }
                self.line(&format!(
                    "::fprime_comp::sm::ExternalSmSender::<'_, {{ {base}::MSG_SIZE }}>::new(&fpp_base.{q}, {base}::MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL, {id}, {prio} as ::fprime_config::FwQueuePriorityType, ::fprime_comp::QueueFullPolicy::{policy}){}.invoke(::core::convert::Into::into(signal), data);",
                    if hook { ".with_overflow_hook(&fpp_hook)" } else { "" }
                ));
                self.dedent();
                self.line("}");
            }
        }
        Ok(())
    }

    /// The body of the `INTERNAL_STATE_MACHINE_SIGNAL` dispatch arm (C++
    /// `smDispatch`).
    pub(super) fn emit_sm_dispatch_internal(&mut self, m: &ComponentModel, n: &SmNames<'_>) {
        let base = n.base;
        self.line("let _ = port_num;");
        self.line("let (fpp_sm_id, fpp_signal) = ::fprime_comp::sm::read_signal_header(fpp_msg);");
        self.line(&format!("let fpp_h = {}SmHandlers(self);", n.name));
        self.line("match fpp_sm_id {");
        for i in &m.state_machine_instances {
            if self.sm_is_internal(i.state_machine) {
                self.line(&format!(
                    "    {base}::{} => fpp_base.{}.dispatch_signal(&fpp_h, fpp_signal, fpp_msg),",
                    Self::sm_id_const(&i.name),
                    snake_ident(&i.name)
                ));
            }
        }
        self.line("    // C++ parity: an unknown state machine id asserts.");
        self.line("    _ => ::fprime_fw::fw_assert!(false, fpp_sm_id),");
        self.line("}");
    }

    /// The body of the `EXTERNAL_STATE_MACHINE_SIGNAL` dispatch arm.
    pub(super) fn emit_sm_dispatch_external(
        &mut self,
        m: &ComponentModel,
        n: &SmNames<'_>,
    ) -> Result<()> {
        let base = n.base;
        self.line("let _ = port_num;");
        self.line("let mut fpp_data = ::fprime_fw::SmSignalBuffer::new();");
        self.line("let (fpp_sm_id, fpp_signal) = ::fprime_comp::sm::read_external_signal(fpp_msg, &mut fpp_data);");
        self.line("let fpp_status = match fpp_sm_id {");
        for i in &m.state_machine_instances {
            if self.sm_is_internal(i.state_machine) {
                continue;
            }
            let t = self.sm_external_type(i)?;
            self.line(&format!(
                "    {base}::{} => <{t} as ::fprime_comp::sm::ExternalStateMachine>::dispatch(&fpp_base.{}, self.__{}_sm_parent(), fpp_sm_id, fpp_signal, &fpp_data),",
                Self::sm_id_const(&i.name),
                snake_ident(&i.name),
                snake(&i.name)
            ));
        }
        self.line("    // C++ parity: an unknown external state machine id is a dispatch error.");
        self.line("    _ => ::fprime_comp::MsgDispatchStatus::Error,");
        self.line("};");
        self.line("if fpp_status != ::fprime_comp::MsgDispatchStatus::Ok { return fpp_status; }");
        let _ = ident;
        Ok(())
    }
}
