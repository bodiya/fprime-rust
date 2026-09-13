//! The JSON dictionary back end (`fpp-to-dict`): for every deployment
//! topology, the ground dictionary that F Prime's ground data system
//! reads (`dictionarySpecVersion` 1.0.0, the format of
//! `DictionaryJsonEncoder.scala`).
//!
//! A dictionary lists the type definitions and constants a topology's
//! instances depend on (the deep closure of the types and constants named
//! by their commands, events, telemetry channels, parameters, records and
//! containers, the implied uses of dictionary generation, and every
//! `dictionary`-marked definition), then the commands, parameters,
//! events, telemetry channels, records and containers of every instance
//! keyed by global id (`base id + local id`), and the telemetry packet
//! sets of the topology.

use crate::analysis::symbols::{Def, NameGroup, SymId};
use crate::analysis::uses::Uses;
use crate::analysis::{
    Analysis, CommandDef, ParamDef, TlmChannelRef, TlmPacketModel, TlmPacketSetModel, Type,
    TypeDef, Value,
};
use crate::ast::*;
use crate::error::{Diagnostic, Loc, Result};
use crate::json::Json;
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};

/// The dictionary format version this back end writes.
pub const DICTIONARY_SPEC_VERSION: &str = "1.0.0";

/// The serialized size of `bool` reported in dictionaries.
const BOOL_SIZE: i128 = 8;

/// The types every dictionary depends on (`ImpliedUse.getTopologyTypes`).
const IMPLIED_TYPES: &[&[&str]] = &[
    &["Fw", "DpCfg", "ProcType"],
    &["Fw", "DpState"],
    &["FwChanIdType"],
    &["FwDpIdType"],
    &["FwDpPriorityType"],
    &["FwEventIdType"],
    &["FwOpcodeType"],
    &["FwPacketDescriptorType"],
    &["FwSizeType"],
    &["FwSizeStoreType"],
    &["FwTimeBaseStoreType"],
    &["FwTimeContextStoreType"],
    &["FwTlmPacketizeIdType"],
];

/// The constants every dictionary depends on.
const IMPLIED_CONSTANTS: &[&[&str]] = &[
    &["Fw", "DpCfg", "CONTAINER_USER_DATA_SIZE"],
    &["FW_FIXED_LENGTH_STRING_SIZE"],
];

/// Dictionary generator options.
#[derive(Debug, Clone)]
pub struct DictOptions {
    /// `metadata.projectVersion`.
    pub project_version: String,
    /// `metadata.frameworkVersion`.
    pub framework_version: String,
    /// `metadata.libraryVersions`.
    pub library_versions: Vec<String>,
    /// Files whose deployment topologies and systems get dictionaries
    /// (everything else is imported for resolution only). Empty means all
    /// files.
    pub targets: Vec<PathBuf>,
}

impl Default for DictOptions {
    fn default() -> Self {
        Self {
            project_version: "[no value specified]".into(),
            framework_version: "[no value specified]".into(),
            library_versions: Vec::new(),
            targets: Vec::new(),
        }
    }
}

/// One generated dictionary.
#[derive(Debug, Clone)]
pub struct DictFile {
    /// `<Topology>TopologyDictionary.json` or `<System>SystemDictionary.json`.
    pub name: String,
    /// The dictionary.
    pub json: Json,
}

/// Generate the dictionaries of every deployment topology and system in
/// the target files.
pub fn generate_dictionaries(a: &Analysis<'_>, opts: &DictOptions) -> Result<Vec<DictFile>> {
    let mut out: Vec<DictFile> = Vec::new();
    let mut seen: HashMap<String, Loc> = HashMap::new();
    for &sym in &a.order {
        let s = a.symbols.sym(sym);
        if !is_target(&opts.targets, &s.loc) {
            continue;
        }
        let (name, top) = match s.def {
            Def::Topology(node) if node.data.is_deployment => {
                (format!("{}TopologyDictionary.json", node.data.name), sym)
            }
            Def::System(node) => {
                let top = a.symbols.resolve(
                    &s.def_stack,
                    NameGroup::PortInterfaceInstance,
                    &node.data.topology.data,
                )?;
                match a.symbols.sym(top).def {
                    Def::Topology(t) if t.data.is_deployment => {}
                    _ => {
                        return Err(Diagnostic::semantic(
                            node.data.topology.loc.clone(),
                            format!(
                                "{} is not a deployment topology",
                                a.symbols.sym(top).qualified_name()
                            ),
                        ));
                    }
                }
                (
                    format!("{}SystemDictionary.json", s.qualified.join("_")),
                    top,
                )
            }
            _ => continue,
        };
        if let Some(prev) = seen.get(&name) {
            return Err(
                Diagnostic::codegen(s.loc.clone(), format!("duplicate JSON file {name}"))
                    .with_note(prev.clone(), "previous file would be generated here"),
            );
        }
        seen.insert(name.clone(), s.loc.clone());
        let json = Dictionary::new(a, opts, top)?.build()?;
        out.push(DictFile { name, json });
    }
    Ok(out)
}

fn is_target(targets: &[PathBuf], loc: &Loc) -> bool {
    if targets.is_empty() {
        return true;
    }
    let mut loc = loc;
    while let Some(inc) = &loc.including {
        loc = inc;
    }
    let file = canonical(&loc.file);
    targets.iter().any(|t| canonical(t) == file)
}

fn canonical(p: &Path) -> PathBuf {
    std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf())
}

/// A dictionary under construction.
struct Dictionary<'d, 'a> {
    a: &'d Analysis<'a>,
    opts: &'d DictOptions,
    /// The deployment topology.
    top: SymId,
    /// `FW_FIXED_LENGTH_STRING_SIZE`: the size of unsized strings.
    default_string_size: i128,
}

/// A dictionary entry of one instance.
struct Entry<'m, T> {
    instance: SymId,
    item: &'m T,
}

impl<'d, 'a> Dictionary<'d, 'a> {
    fn new(a: &'d Analysis<'a>, opts: &'d DictOptions, top: SymId) -> Result<Self> {
        let loc = a.symbols.sym(top).loc.clone();
        let size_sym = implied(a, NameGroup::Value, &["FW_FIXED_LENGTH_STRING_SIZE"], &loc)?;
        let default_string_size =
            a.values
                .get(&size_sym)
                .and_then(Value::as_int)
                .ok_or_else(|| {
                    Diagnostic::semantic(
                        a.symbols.sym(size_sym).loc.clone(),
                        "FW_FIXED_LENGTH_STRING_SIZE must be an integer constant",
                    )
                })?;
        Ok(Self {
            a,
            opts,
            top,
            default_string_size,
        })
    }

    fn instances(&self) -> &[SymId] {
        &self.a.topologies[&self.top].instances
    }

    fn instance_name(&self, inst: SymId) -> String {
        self.a.symbols.sym(inst).qualified_name()
    }

    /// Build the dictionary JSON.
    fn build(&self) -> Result<Json> {
        let top_loc = self.a.symbols.sym(self.top).loc.clone();
        let used = self.used_symbols(&top_loc)?;

        let mut metadata = Json::obj();
        metadata.push(
            "deploymentName",
            Json::Str(self.a.symbols.sym(self.top).qualified_name()),
        );
        metadata.push(
            "projectVersion",
            Json::Str(self.opts.project_version.clone()),
        );
        metadata.push(
            "frameworkVersion",
            Json::Str(self.opts.framework_version.clone()),
        );
        metadata.push(
            "libraryVersions",
            Json::Arr(
                self.opts
                    .library_versions
                    .iter()
                    .map(|v| Json::Str(v.clone()))
                    .collect(),
            ),
        );
        metadata.push(
            "dictionarySpecVersion",
            Json::Str(DICTIONARY_SPEC_VERSION.into()),
        );

        // Type definitions and constants, by qualified name.
        let mut type_defs: BTreeMap<String, Json> = BTreeMap::new();
        let mut constants: BTreeMap<String, Json> = BTreeMap::new();
        for &sym in &used {
            let s = self.a.symbols.sym(sym);
            match s.def {
                Def::AliasType(_) | Def::Array(_) | Def::Enum(_) | Def::Struct(_) => {
                    type_defs.insert(s.qualified_name(), self.type_def_json(sym)?);
                }
                Def::Constant(_) => {
                    if let Some(j) = self.constant_json(sym)? {
                        constants.insert(s.qualified_name(), j);
                    }
                }
                _ => {}
            }
        }

        // Entries keyed by global id.
        let mut commands: BTreeMap<u64, Entry<'_, CommandDef>> = BTreeMap::new();
        let mut params = BTreeMap::new();
        let mut events = BTreeMap::new();
        let mut channels = BTreeMap::new();
        let mut records = BTreeMap::new();
        let mut containers = BTreeMap::new();
        for &inst in self.instances() {
            let im = self.a.instance(inst);
            let comp = self.a.component_of_instance(inst);
            let base = im.base_id;
            self.add_entries(&mut commands, inst, base, &comp.commands, |c| {
                (c.opcode, &c.loc)
            })?;
            self.add_entries(&mut params, inst, base, &comp.params, |p| (p.id, &p.loc))?;
            self.add_entries(&mut events, inst, base, &comp.events, |e| (e.id, &e.loc))?;
            self.add_entries(&mut channels, inst, base, &comp.tlm_channels, |c| {
                (c.id, &c.loc)
            })?;
            self.add_entries(&mut records, inst, base, &comp.records, |r| (r.id, &r.loc))?;
            self.add_entries(&mut containers, inst, base, &comp.containers, |c| {
                (c.id, &c.loc)
            })?;
        }

        let mut commands_json = Vec::new();
        for (id, e) in &commands {
            commands_json.push(self.command_json(*id, e)?);
        }
        let mut params_json = Vec::new();
        for (id, e) in &params {
            let p = e.item;
            let mut j = Json::obj();
            j.push("name", Json::Str(self.entry_name(e.instance, &p.name)));
            j.push("type", self.type_json(&p.ty)?);
            j.push("id", Json::Int(*id as i128));
            if let Some(d) = &p.default {
                j.push("default", self.value_json(d)?);
            }
            j.push_opt("annotation", annotation(&p.docs));
            params_json.push(j);
        }
        let mut events_json = Vec::new();
        for (id, e) in &events {
            let ev = e.item;
            let mut j = Json::obj();
            j.push("name", Json::Str(self.entry_name(e.instance, &ev.name)));
            j.push("severity", Json::Str(severity_name(ev.severity).into()));
            j.push("formalParams", self.params_json(&ev.params)?);
            j.push("id", Json::Int(*id as i128));
            j.push("format", Json::Str(ev.format.to_string()));
            j.push_opt("annotation", annotation(&ev.docs));
            if let Some(count) = ev.throttle {
                let mut t = Json::obj();
                t.push("count", Json::Int(count as i128));
                match ev.every {
                    Some((secs, usecs)) => {
                        let mut every = Json::obj();
                        every.push("seconds", Json::Int(secs as i128));
                        every.push("useconds", Json::Int(usecs as i128));
                        t.push("every", every);
                    }
                    None => t.push("every", Json::Null),
                }
                j.push("throttle", t);
            }
            events_json.push(j);
        }
        let mut channels_json = Vec::new();
        for (id, e) in &channels {
            let c = e.item;
            let mut j = Json::obj();
            j.push("name", Json::Str(self.entry_name(e.instance, &c.name)));
            j.push("type", self.type_json(&c.ty)?);
            j.push("id", Json::Int(*id as i128));
            j.push(
                "telemetryUpdate",
                Json::Str(
                    match c.update {
                        TlmUpdate::Always => "always",
                        TlmUpdate::OnChange => "on change",
                    }
                    .into(),
                ),
            );
            if let Some(f) = &c.format {
                j.push("format", Json::Str(f.to_string()));
            }
            j.push_opt("annotation", annotation(&c.docs));
            if !c.low.is_empty() || !c.high.is_empty() {
                let mut limits = Json::obj();
                for (key, list) in [("low", &c.low), ("high", &c.high)] {
                    if list.is_empty() {
                        continue;
                    }
                    let mut l = Json::obj();
                    for (kind, v) in list {
                        l.push(limit_name(*kind), self.value_json(v)?);
                    }
                    limits.push(key, l);
                }
                j.push("limits", limits);
            }
            channels_json.push(j);
        }
        let mut records_json = Vec::new();
        for (id, e) in &records {
            let r = e.item;
            let mut j = Json::obj();
            j.push("name", Json::Str(self.entry_name(e.instance, &r.name)));
            j.push("type", self.type_json(&r.ty)?);
            j.push("array", Json::Bool(r.is_array));
            j.push("id", Json::Int(*id as i128));
            j.push_opt("annotation", annotation(&r.docs));
            records_json.push(j);
        }
        let mut containers_json = Vec::new();
        for (id, e) in &containers {
            let c = e.item;
            let mut j = Json::obj();
            j.push("name", Json::Str(self.entry_name(e.instance, &c.name)));
            j.push("id", Json::Int(*id as i128));
            if let Some(p) = c.default_priority {
                j.push("defaultPriority", Json::Int(p as i128));
            }
            j.push_opt("annotation", annotation(&c.docs));
            containers_json.push(j);
        }

        let packet_sets = self.packet_sets_json(&channels)?;

        let mut d = Json::obj();
        d.push("metadata", metadata);
        d.push(
            "typeDefinitions",
            Json::Arr(type_defs.into_values().collect()),
        );
        d.push("constants", Json::Arr(constants.into_values().collect()));
        d.push("commands", Json::Arr(commands_json));
        d.push("parameters", Json::Arr(params_json));
        d.push("events", Json::Arr(events_json));
        d.push("telemetryChannels", Json::Arr(channels_json));
        d.push("records", Json::Arr(records_json));
        d.push("containers", Json::Arr(containers_json));
        d.push("telemetryPacketSets", Json::Arr(packet_sets));
        Ok(d)
    }

    /// Add one kind of specifier of an instance to an id-keyed map,
    /// rejecting global-id collisions.
    fn add_entries<'m, T>(
        &self,
        map: &mut BTreeMap<u64, Entry<'m, T>>,
        instance: SymId,
        base: u64,
        items: &'m [T],
        key: impl Fn(&T) -> (u64, &Loc),
    ) -> Result<()> {
        for item in items {
            let (local, loc) = key(item);
            let id = base + local;
            if let Some(prev) = map.get(&id) {
                let (_, prev_loc) = key(prev.item);
                return Err(Diagnostic::semantic(
                    loc.clone(),
                    format!(
                        "duplicate dictionary id {id:#x} (instance {} and instance {})",
                        self.instance_name(instance),
                        self.instance_name(prev.instance)
                    ),
                )
                .with_note(prev_loc.clone(), "previous occurrence is here"));
            }
            map.insert(id, Entry { instance, item });
        }
        Ok(())
    }

    fn entry_name(&self, inst: SymId, name: &str) -> String {
        format!("{}.{}", self.instance_name(inst), name)
    }

    // -- used symbols ---------------------------------------------------------------

    /// The closure of type and constant definitions the dictionary needs.
    fn used_symbols(&self, top_loc: &Loc) -> Result<Vec<SymId>> {
        let a = self.a;
        let mut uses = Uses::new(a);
        for &inst in self.instances() {
            uses.component_specifiers(a.instance(inst).component)?;
        }
        for parts in IMPLIED_TYPES {
            uses.found
                .insert(implied(a, NameGroup::Type, parts, top_loc)?);
        }
        for parts in IMPLIED_CONSTANTS {
            uses.found
                .insert(implied(a, NameGroup::Value, parts, top_loc)?);
        }
        for (sym, s) in a.symbols.symbols.iter().enumerate() {
            let is_dict = match s.def {
                Def::AliasType(n) => n.data.is_dictionary,
                Def::Array(n) => n.data.is_dictionary,
                Def::Enum(n) => n.data.is_dictionary,
                Def::Struct(n) => n.data.is_dictionary,
                Def::Constant(n) => n.data.is_dictionary,
                _ => false,
            };
            if !is_dict {
                continue;
            }
            match s.def {
                Def::Constant(_) => {
                    let v = a.values.get(&sym).ok_or_else(|| {
                        Diagnostic::semantic(s.loc.clone(), "dictionary constant is unresolved")
                    })?;
                    if !matches!(
                        v,
                        Value::Int(..)
                            | Value::Float(..)
                            | Value::Bool(_)
                            | Value::Str(_)
                            | Value::Enum(..)
                    ) {
                        return Err(Diagnostic::semantic(
                            s.loc.clone(),
                            "dictionary constant must have a numeric, Boolean, string, or enum type",
                        ));
                    }
                }
                _ => {
                    let ty = match s.def {
                        Def::AliasType(_) => Type::Alias(sym),
                        Def::Array(_) => Type::Array(sym),
                        Def::Enum(_) => Type::Enum(sym),
                        _ => Type::Struct(sym),
                    };
                    if let Some(culprit) = self.non_displayable(&ty) {
                        return Err(Diagnostic::semantic(
                            s.loc.clone(),
                            "dictionary type is not displayable",
                        )
                        .with_note(
                            a.symbols.sym(culprit).loc.clone(),
                            format!(
                                "because {} is not displayable",
                                a.symbols.sym(culprit).qualified_name()
                            ),
                        ));
                    }
                }
            }
            uses.found.insert(sym);
        }
        uses.resolve_deep()?;
        Ok(uses.found.into_iter().collect())
    }

    /// The abstract type that makes `t` non-displayable, if any.
    fn non_displayable(&self, t: &Type) -> Option<SymId> {
        match t {
            Type::Int(_) | Type::Float(_) | Type::Bool | Type::String(_) | Type::Integer => None,
            Type::Abs(s) => Some(*s),
            Type::Alias(s) => match self.a.type_defs.get(s) {
                Some(TypeDef::Alias(inner)) => self.non_displayable(inner),
                _ => None,
            },
            Type::Enum(_) => None,
            Type::Array(s) => match self.a.type_defs.get(s) {
                Some(TypeDef::Array { elt, .. }) => self.non_displayable(elt),
                _ => None,
            },
            Type::Struct(s) => match self.a.type_defs.get(s) {
                Some(TypeDef::Struct { members, .. }) => {
                    members.iter().find_map(|m| self.non_displayable(&m.ty))
                }
                _ => None,
            },
            Type::AnonArray(_, elt) => self.non_displayable(elt),
            Type::AnonStruct(ms) => ms.iter().find_map(|(_, t)| self.non_displayable(t)),
        }
    }

    // -- types and values -----------------------------------------------------------

    /// A type reference (`typeAsJson`).
    fn type_json(&self, t: &Type) -> Result<Json> {
        let mut j = Json::obj();
        match t {
            Type::Int(k) => {
                j.push("name", Json::Str(k.fpp_name().into()));
                j.push("kind", Json::Str("integer".into()));
                j.push("size", Json::Int(i128::from(k.bits())));
                j.push("signed", Json::Bool(k.signed()));
            }
            Type::Integer => {
                j.push("name", Json::Str("U64".into()));
                j.push("kind", Json::Str("integer".into()));
                j.push("size", Json::Int(64));
                j.push("signed", Json::Bool(false));
            }
            Type::Float(k) => {
                let (name, size) = match k {
                    FloatKind::F32 => ("F32", 32),
                    FloatKind::F64 => ("F64", 64),
                };
                j.push("name", Json::Str(name.into()));
                j.push("kind", Json::Str("float".into()));
                j.push("size", Json::Int(size));
            }
            Type::Bool => {
                j.push("name", Json::Str("bool".into()));
                j.push("kind", Json::Str("bool".into()));
                j.push("size", Json::Int(BOOL_SIZE));
            }
            Type::String(size) => {
                j.push("name", Json::Str("string".into()));
                j.push("kind", Json::Str("string".into()));
                j.push(
                    "size",
                    Json::Int(size.map_or(self.default_string_size, i128::from)),
                );
            }
            Type::Alias(s) | Type::Enum(s) | Type::Array(s) | Type::Struct(s) => {
                j.push("name", Json::Str(self.a.symbols.sym(*s).qualified_name()));
                j.push("kind", Json::Str("qualifiedIdentifier".into()));
            }
            Type::Abs(s) => {
                let sym = self.a.symbols.sym(*s);
                return Err(Diagnostic::codegen(
                    sym.loc.clone(),
                    format!(
                        "abstract type {} is not displayable and cannot appear in a dictionary",
                        sym.qualified_name()
                    ),
                ));
            }
            Type::AnonArray(..) | Type::AnonStruct(_) => {
                return Err(Diagnostic::codegen(
                    self.a.symbols.sym(self.top).loc.clone(),
                    format!("type {t} cannot appear in a dictionary"),
                ));
            }
        }
        Ok(j)
    }

    /// A value (`valueAsJson`): enum constants by qualified name.
    fn value_json(&self, v: &Value) -> Result<Json> {
        Ok(match v {
            Value::Int(i, _) => Json::Int(*i),
            Value::Float(f, _) => Json::Float(*f),
            Value::Bool(b) => Json::Bool(*b),
            Value::Str(s) => Json::Str(s.clone()),
            Value::Enum(e, name, _) => Json::Str(format!(
                "{}.{}",
                self.a.symbols.sym(*e).qualified_name(),
                name
            )),
            Value::Array(_, elts) => {
                let mut out = Vec::new();
                for e in elts {
                    out.push(self.value_json(e)?);
                }
                Json::Arr(out)
            }
            Value::Struct(_, members) => {
                let mut out = Json::obj();
                for (n, m) in members {
                    out.push(n, self.value_json(m)?);
                }
                out
            }
            Value::Abs(s) => {
                let sym = self.a.symbols.sym(*s);
                return Err(Diagnostic::codegen(
                    sym.loc.clone(),
                    format!(
                        "value of abstract type {} cannot appear in a dictionary",
                        sym.qualified_name()
                    ),
                ));
            }
        })
    }

    /// A type definition entry.
    fn type_def_json(&self, sym: SymId) -> Result<Json> {
        let s = self.a.symbols.sym(sym);
        let def = self.a.type_defs.get(&sym).ok_or_else(|| {
            Diagnostic::codegen(
                s.loc.clone(),
                format!("type {} is unresolved", s.qualified_name()),
            )
        })?;
        let mut j = Json::obj();
        match def {
            TypeDef::Alias(target) => {
                j.push("kind", Json::Str("alias".into()));
                j.push("qualifiedName", Json::Str(s.qualified_name()));
                j.push("type", self.type_json(target)?);
                j.push(
                    "underlyingType",
                    self.type_json(&self.a.underlying(target))?,
                );
            }
            TypeDef::Enum {
                repr,
                constants,
                default,
            } => {
                j.push("kind", Json::Str("enum".into()));
                j.push("qualifiedName", Json::Str(s.qualified_name()));
                j.push("representationType", self.type_json(&Type::Int(*repr))?);
                let mut cs = Vec::new();
                for (name, value, csym) in constants {
                    let mut c = Json::obj();
                    c.push("name", Json::Str(name.clone()));
                    c.push("value", Json::Int(*value));
                    c.push_opt("annotation", annotation(&self.a.symbols.sym(*csym).docs));
                    cs.push(c);
                }
                j.push("enumeratedConstants", Json::Arr(cs));
                j.push(
                    "default",
                    Json::Str(format!("{}.{}", s.qualified_name(), default)),
                );
            }
            TypeDef::Array {
                size,
                elt,
                default,
                format,
            } => {
                j.push("kind", Json::Str("array".into()));
                j.push("qualifiedName", Json::Str(s.qualified_name()));
                j.push("size", Json::Int(i128::from(*size)));
                j.push("elementType", self.type_json(elt)?);
                j.push(
                    "default",
                    match default {
                        Value::Array(_, elts) => {
                            let mut out = Vec::new();
                            for e in elts {
                                out.push(self.value_json(e)?);
                            }
                            Json::Arr(out)
                        }
                        other => self.value_json(other)?,
                    },
                );
                if let Some(f) = format {
                    j.push("format", Json::Str(f.clone()));
                }
            }
            TypeDef::Struct { members, default } => {
                j.push("kind", Json::Str("struct".into()));
                j.push("qualifiedName", Json::Str(s.qualified_name()));
                let mut ms = Json::obj();
                for (index, m) in members.iter().enumerate() {
                    let (ty, size) = match &m.ty {
                        Type::AnonArray(Some(n), elt) => (elt.as_ref(), Some(*n)),
                        other => (other, None),
                    };
                    let mut mj = Json::obj();
                    mj.push("type", self.type_json(ty)?);
                    mj.push("index", Json::Int(index as i128));
                    if let Some(n) = size {
                        mj.push("size", Json::Int(i128::from(n)));
                    }
                    if let Some(f) = &m.format {
                        mj.push("format", Json::Str(f.clone()));
                    }
                    mj.push_opt("annotation", annotation(&m.docs));
                    ms.push(&m.name, mj);
                }
                j.push("members", ms);
                j.push("default", self.value_json(default)?);
            }
            TypeDef::Abs => {
                return Err(Diagnostic::codegen(
                    s.loc.clone(),
                    format!(
                        "abstract type {} cannot appear in a dictionary",
                        s.qualified_name()
                    ),
                ));
            }
        }
        j.push_opt("annotation", annotation(&s.docs));
        Ok(j)
    }

    /// A constant entry, or `None` when the constant's type is not
    /// representable (anonymous arrays and structs, abstract values).
    fn constant_json(&self, sym: SymId) -> Result<Option<Json>> {
        let s = self.a.symbols.sym(sym);
        let Some(v) = self.a.values.get(&sym) else {
            return Ok(None);
        };
        let ty = match v {
            Value::Int(i, None) => Type::Int(if *i < 0 { IntKind::I64 } else { IntKind::U64 }),
            Value::Int(_, Some(k)) => Type::Int(*k),
            Value::Float(_, k) => Type::Float(*k),
            Value::Bool(_) => Type::Bool,
            Value::Str(_) => Type::String(None),
            Value::Enum(e, ..) => Type::Enum(*e),
            Value::Array(Some(t), _) => Type::Array(*t),
            Value::Struct(Some(t), _) => Type::Struct(*t),
            Value::Array(None, _) | Value::Struct(None, _) | Value::Abs(_) => return Ok(None),
        };
        if self.non_displayable(&ty).is_some() {
            return Ok(None);
        }
        let mut j = Json::obj();
        j.push("kind", Json::Str("constant".into()));
        j.push("qualifiedName", Json::Str(s.qualified_name()));
        j.push("type", self.type_json(&ty)?);
        j.push("value", self.value_json(v)?);
        j.push_opt("annotation", annotation(&s.docs));
        Ok(Some(j))
    }

    // -- entries --------------------------------------------------------------------

    fn params_json(&self, params: &[ParamDef]) -> Result<Json> {
        let mut out = Vec::new();
        for p in params {
            let mut j = Json::obj();
            j.push("name", Json::Str(p.name.clone()));
            j.push("type", self.type_json(&p.ty)?);
            j.push("ref", Json::Bool(p.kind == FormalParamKind::Ref));
            j.push_opt("annotation", annotation(&p.docs));
            out.push(j);
        }
        Ok(Json::Arr(out))
    }

    fn command_json(&self, opcode: u64, e: &Entry<'_, CommandDef>) -> Result<Json> {
        let c = e.item;
        let comp = self.a.component_of_instance(e.instance);
        let mut j = Json::obj();
        match c.param_cmd {
            Some((index, is_set)) => {
                // The implicit parameter commands: named `<PARAM>_PRM_SET`
                // and `<PARAM>_PRM_SAVE` (upper case) in dictionaries,
                // annotated with the parameter's annotation.
                let p = &comp.params[index];
                let suffix = if is_set { "PRM_SET" } else { "PRM_SAVE" };
                j.push(
                    "name",
                    Json::Str(
                        self.entry_name(e.instance, &format!("{}_{suffix}", p.name.to_uppercase())),
                    ),
                );
                j.push(
                    "commandKind",
                    Json::Str(if is_set { "set" } else { "save" }.into()),
                );
                j.push("opcode", Json::Int(opcode as i128));
                let params = if is_set {
                    let mut v = Json::obj();
                    v.push("name", Json::Str("val".into()));
                    v.push("type", self.type_json(&p.ty)?);
                    v.push("ref", Json::Bool(false));
                    Json::Arr(vec![v])
                } else {
                    Json::Arr(Vec::new())
                };
                j.push("formalParams", params);
                j.push_opt("annotation", annotation(&p.docs));
            }
            None => {
                j.push("name", Json::Str(self.entry_name(e.instance, &c.name)));
                let kind = match c.kind {
                    CommandKind::Async => "async",
                    CommandKind::Guarded => "guarded",
                    CommandKind::Sync => "sync",
                };
                j.push("commandKind", Json::Str(kind.into()));
                j.push("opcode", Json::Int(opcode as i128));
                j.push("formalParams", self.params_json(&c.params)?);
                if c.kind == CommandKind::Async {
                    if let Some(p) = c.priority {
                        j.push("priority", Json::Int(p));
                    }
                    j.push(
                        "queueFullBehavior",
                        Json::Str(
                            match c.queue_full {
                                QueueFull::Assert => "assert",
                                QueueFull::Block => "block",
                                QueueFull::Drop => "drop",
                                QueueFull::Hook => "hook",
                            }
                            .into(),
                        ),
                    );
                }
                j.push_opt("annotation", annotation(&c.docs));
            }
        }
        Ok(j)
    }

    // -- telemetry packet sets ------------------------------------------------------

    /// The topology's own telemetry packet sets. Every channel of the
    /// dictionary must be in exactly one of a packet or the omitted list.
    fn packet_sets_json(
        &self,
        channels: &BTreeMap<u64, Entry<'_, crate::analysis::TlmChannelDef>>,
    ) -> Result<Vec<Json>> {
        let a = self.a;
        let channel_name = |id: u64| -> String {
            let e = &channels[&id];
            self.entry_name(e.instance, &e.item.name)
        };
        let channel_id = |r: &TlmChannelRef| -> u64 {
            let comp = a.component_of_instance(r.instance);
            let local = comp
                .tlm_channels
                .iter()
                .find(|c| c.name == r.channel)
                .expect("channel resolved by analysis")
                .id;
            a.instance(r.instance).base_id + local
        };
        let mut sets: Vec<Json> = Vec::new();
        let mut models: Vec<&TlmPacketSetModel> =
            a.topologies[&self.top].packet_sets.iter().collect();
        models.sort_by(|x, y| x.name.cmp(&y.name));
        for set in models {
            let mut used: HashMap<u64, Loc> = HashMap::new();
            let mut packets: Vec<(u64, &TlmPacketModel, Vec<u64>)> = Vec::new();
            for p in &set.packets {
                let mut members = Vec::new();
                for r in &p.members {
                    let cid = channel_id(r);
                    if let Some(prev) = used.get(&cid) {
                        return Err(Diagnostic::semantic(
                            r.loc.clone(),
                            format!(
                                "telemetry channel {} is used more than once in packet set {}",
                                channel_name(cid),
                                set.name
                            ),
                        )
                        .with_note(prev.clone(), "previous use is here"));
                    }
                    used.insert(cid, r.loc.clone());
                    members.push(cid);
                }
                packets.push((p.id, p, members));
            }
            packets.sort_by_key(|(id, ..)| *id);
            let mut omitted: BTreeMap<u64, Loc> = BTreeMap::new();
            for r in &set.omitted {
                omitted.insert(channel_id(r), r.loc.clone());
            }
            for (&cid, e) in channels {
                let is_used = used.contains_key(&cid);
                let is_omitted = omitted.contains_key(&cid);
                if is_used && is_omitted {
                    return Err(Diagnostic::semantic(
                        omitted[&cid].clone(),
                        format!(
                            "telemetry channel {} is both used and marked omitted in packet set {}",
                            channel_name(cid),
                            set.name
                        ),
                    )
                    .with_note(used[&cid].clone(), "used here"));
                }
                if !is_used && !is_omitted {
                    return Err(Diagnostic::semantic(
                        set.loc.clone(),
                        format!(
                            "telemetry channel {} is neither used nor marked as omitted in packet set {}",
                            channel_name(cid),
                            set.name
                        ),
                    )
                    .with_note(e.item.loc.clone(), "telemetry channel is specified here"));
                }
            }
            let mut members = Vec::new();
            for (id, p, chans) in &packets {
                let mut pj = Json::obj();
                pj.push("name", Json::Str(p.name.clone()));
                pj.push("id", Json::Int(*id as i128));
                pj.push("group", Json::Int(p.group as i128));
                pj.push(
                    "members",
                    Json::Arr(chans.iter().map(|c| Json::Str(channel_name(*c))).collect()),
                );
                members.push(pj);
            }
            let mut omitted_names: Vec<String> = omitted.keys().map(|c| channel_name(*c)).collect();
            omitted_names.sort();
            let mut sj = Json::obj();
            sj.push("name", Json::Str(set.name.clone()));
            sj.push("members", Json::Arr(members));
            sj.push(
                "omitted",
                Json::Arr(omitted_names.into_iter().map(Json::Str).collect()),
            );
            sets.push(sj);
        }
        Ok(sets)
    }
}

/// Resolve an implied use by absolute name, or report it as the reference
/// compiler does.
fn implied(a: &Analysis<'_>, group: NameGroup, parts: &[&str], loc: &Loc) -> Result<SymId> {
    a.symbols.resolve_absolute(group, parts).ok_or_else(|| {
        let name = parts.join(".");
        Diagnostic::semantic(loc.clone(), format!("symbol {name} is not defined"))
            .with_note(
                loc.clone(),
                format!(
                    "looking for a {} here; the symbol {name} has an implied use when constructing a dictionary",
                    match group {
                        NameGroup::Type => "type",
                        _ => "constant",
                    }
                ),
            )
    })
}

fn annotation(docs: &[String]) -> Option<Json> {
    if docs.is_empty() {
        None
    } else {
        Some(Json::Str(docs.join("\n")))
    }
}

fn severity_name(s: Severity) -> &'static str {
    match s {
        Severity::ActivityHigh => "ACTIVITY_HI",
        Severity::ActivityLow => "ACTIVITY_LO",
        Severity::Command => "COMMAND",
        Severity::Diagnostic => "DIAGNOSTIC",
        Severity::Fatal => "FATAL",
        Severity::WarningHigh => "WARNING_HI",
        Severity::WarningLow => "WARNING_LO",
    }
}

fn limit_name(k: LimitKind) -> &'static str {
    match k {
        LimitKind::Red => "red",
        LimitKind::Orange => "orange",
        LimitKind::Yellow => "yellow",
    }
}
