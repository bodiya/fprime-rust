//! Emission of telemetry packet sets: the Rust equivalent of the C++
//! autocoder's `<Topology>_<Set>TlmPacketsAc` (`TlmPacketSetCppWriter`).
//!
//! For packet set `S` of topology `T` the back end emits, next to
//! `TTopology`, a module `T_STlmPackets` holding the packet table that
//! `Svc::TlmPacketizer::set_packet_list` consumes:
//!
//! - `PACKET_LIST: &[TlmPacketizerPacket]` — C++ `packetList`: one entry
//!   per packet, **in packet-name order** (as the reference writer sorts
//!   them), each listing its channels in declaration order as
//!   `(global id, maximum serialized size)` with its id and group;
//! - `OMITTED_CHANNELS: &[TlmPacketizerChannelEntry]` — C++
//!   `omittedChannels` (its entry list), in channel-name order;
//! - the C++ file-local tables as nested modules (`ChannelIds`,
//!   `ChannelSizes`, `PacketIds`, `PacketGroups`, `PacketDataSizes`,
//!   `PacketEntryArrays`, `Packets`, `SizeBounds`) with the C++
//!   `static_assert`s as compile-time assertions: the packet header fits a
//!   com buffer, every packet's data fits what is left, and there are at
//!   most `MAX_PACKETIZER_PACKETS` packets.
//!
//! Like the C++ autocoder, the generated topology does not call
//! `set_packet_list` itself: upstream deployments do it in instance init
//! code (`phase Fpp.ToCpp.Phases.configComponents`), passing
//! `<T>_<S>TlmPackets::packetList`, `::omittedChannels` and a start level.
//!
//! The set is checked as the dictionary back end checks it: a channel is
//! used at most once, and every channel of the topology is either used or
//! omitted, never both.

use super::Generator;
use super::names::ident;
use crate::analysis::{SymId, TlmChannelDef, TlmChannelRef, TlmPacketModel, TlmPacketSetModel};
use crate::error::{Diagnostic, Loc, Result};
use std::collections::{BTreeMap, HashMap};

/// A channel of the topology, keyed by global id.
struct Channel<'m> {
    instance: SymId,
    def: &'m TlmChannelDef,
}

impl Generator<'_, '_> {
    /// The C++ identifier of a channel: its qualified name, `_`-joined.
    fn channel_ident(&self, instance: SymId, channel: &str) -> String {
        format!(
            "{}_{}",
            self.a
                .symbols
                .sym(instance)
                .qualified_name()
                .replace('.', "_"),
            channel
        )
    }

    /// Emit the packet-set modules of topology `top`.
    pub(super) fn emit_packet_sets(&mut self, top: SymId) -> Result<()> {
        let a = self.a;
        let t = &a.topologies[&top];
        if t.packet_sets.is_empty() {
            return Ok(());
        }
        // Every channel of every instance, by global id.
        let mut channels: BTreeMap<u64, Channel<'_>> = BTreeMap::new();
        for &inst in &t.instances {
            let base = a.instance(inst).base_id;
            for def in &a.component_of_instance(inst).tlm_channels {
                let id = base + def.id;
                if let Some(prev) = channels.get(&id) {
                    return Err(Diagnostic::semantic(
                        def.loc.clone(),
                        format!(
                            "duplicate dictionary id {id:#x} (instance {} and instance {})",
                            self.a.symbols.sym(inst).qualified_name(),
                            self.a.symbols.sym(prev.instance).qualified_name()
                        ),
                    )
                    .with_note(prev.def.loc.clone(), "previous occurrence is here"));
                }
                channels.insert(
                    id,
                    Channel {
                        instance: inst,
                        def,
                    },
                );
            }
        }
        let top_name = a.symbols.sym(top).name.clone();
        for set in &t.packet_sets {
            self.emit_packet_set(&top_name, set, &channels)?;
        }
        Ok(())
    }

    fn channel_id(&self, r: &TlmChannelRef) -> u64 {
        let local = self
            .a
            .component_of_instance(r.instance)
            .tlm_channels
            .iter()
            .find(|c| c.name == r.channel)
            .expect("channel resolved by analysis")
            .id;
        self.a.instance(r.instance).base_id + local
    }

    fn emit_packet_set(
        &mut self,
        top: &str,
        set: &TlmPacketSetModel,
        channels: &BTreeMap<u64, Channel<'_>>,
    ) -> Result<()> {
        let name_of = |g: &Self, id: u64| -> String {
            let c = &channels[&id];
            format!(
                "{}.{}",
                g.a.symbols.sym(c.instance).qualified_name(),
                c.def.name
            )
        };
        // The dictionary's checks (`TlmPacketSet` in the reference).
        let mut used: HashMap<u64, Loc> = HashMap::new();
        let mut packets: Vec<(&TlmPacketModel, Vec<u64>)> = Vec::new();
        for p in &set.packets {
            let mut members = Vec::new();
            for r in &p.members {
                let cid = self.channel_id(r);
                if let Some(prev) = used.get(&cid) {
                    return Err(Diagnostic::semantic(
                        r.loc.clone(),
                        format!(
                            "telemetry channel {} is used more than once in packet set {}",
                            name_of(self, cid),
                            set.name
                        ),
                    )
                    .with_note(prev.clone(), "previous use is here"));
                }
                used.insert(cid, r.loc.clone());
                members.push(cid);
            }
            packets.push((p, members));
        }
        let mut omitted: BTreeMap<u64, Loc> = BTreeMap::new();
        for r in &set.omitted {
            omitted.insert(self.channel_id(r), r.loc.clone());
        }
        for (&cid, c) in channels {
            match (used.get(&cid), omitted.get(&cid)) {
                (Some(u), Some(o)) => {
                    return Err(Diagnostic::semantic(
                        o.clone(),
                        format!(
                            "telemetry channel {} is both used and marked omitted in packet set {}",
                            name_of(self, cid),
                            set.name
                        ),
                    )
                    .with_note(u.clone(), "used here"));
                }
                (None, None) => {
                    return Err(Diagnostic::semantic(
                        set.loc.clone(),
                        format!(
                            "telemetry channel {} is neither used nor marked as omitted in packet set {}",
                            name_of(self, cid),
                            set.name
                        ),
                    )
                    .with_note(c.def.loc.clone(), "telemetry channel is specified here"));
                }
                _ => {}
            }
        }
        // C++ orders packets by name and the channel tables and omitted
        // channels by qualified channel name.
        packets.sort_by(|a, b| a.0.name.cmp(&b.0.name));
        let mut by_name: Vec<(String, u64)> =
            channels.keys().map(|&id| (name_of(self, id), id)).collect();
        by_name.sort();
        let mut omitted_sorted: Vec<(String, u64)> =
            omitted.keys().map(|&id| (name_of(self, id), id)).collect();
        omitted_sorted.sort();
        let ident_of = |g: &Self, id: u64| -> String {
            let c = &channels[&id];
            g.channel_ident(c.instance, &c.def.name)
        };

        let pk = self.opts.packetizer_path.clone();
        let module = format!("{top}_{}TlmPackets", set.name);
        self.line("");
        self.line(&format!(
            "// ---- telemetry packet set {}.{} ----",
            top, set.name
        ));
        self.line("");
        self.line(&format!("/// Telemetry packet set `{}` of topology `{top}` (C++ `{module}`): the packet table `TlmPacketizer::set_packet_list` consumes.", set.name));
        self.line(
            "#[allow(non_snake_case, non_camel_case_types, non_upper_case_globals, dead_code)]",
        );
        self.line(&format!("pub mod {module} {{"));
        self.indent();
        self.line(&format!(
            "use {pk}::{{TlmPacketizerChannelEntry, TlmPacketizerPacket}};"
        ));
        // SizeBounds
        if !packets.is_empty() {
            self.line("");
            self.line("/// Packet size bounds.");
            self.line("pub mod SizeBounds {");
            self.line(
                "    /// The size of a packet header: time tag, packet id and packet descriptor.",
            );
            self.line("    pub const PACKET_HEADER_SIZE: ::fprime_config::FwSizeType = (<::fprime_fw::Time as ::fprime_fw::FppSized>::SERIALIZED_SIZE + ::std::mem::size_of::<::fprime_config::FwTlmPacketizeIdType>() + ::std::mem::size_of::<::fprime_config::FwPacketDescriptorType>()) as ::fprime_config::FwSizeType;");
            self.line("    const _: () = assert!(PACKET_HEADER_SIZE <= ::fprime_config::FW_COM_BUFFER_MAX_SIZE as ::fprime_config::FwSizeType, \"packet header must fit in com buffer\");");
            self.line("    /// The maximum data size of a packet in a com buffer.");
            self.line("    pub const PACKET_MAX_DATA_SIZE: ::fprime_config::FwSizeType = ::fprime_config::FW_COM_BUFFER_MAX_SIZE as ::fprime_config::FwSizeType - PACKET_HEADER_SIZE;");
            self.line("}");
        }
        // ChannelIds / ChannelSizes
        self.line("");
        self.line("/// The global identifier of every channel of the topology.");
        self.line("pub mod ChannelIds {");
        for (name, id) in &by_name {
            self.line(&format!("    /// The identifier for channel `{name}`."));
            self.line(&format!(
                "    pub const {}: ::fprime_config::FwChanIdType = {id:#x};",
                ident_of(self, *id)
            ));
        }
        self.line("}");
        self.line("");
        self.line("/// The maximum serialized size of every channel of the topology.");
        self.line("pub mod ChannelSizes {");
        // Type paths are relative to the module they are written in.
        self.module.push(module.clone());
        self.module.push("ChannelSizes".into());
        let sizes: Result<Vec<String>> = by_name
            .iter()
            .map(|(_, id)| self.size_expr(&channels[id].def.ty))
            .collect();
        self.module.pop();
        self.module.pop();
        for ((name, id), size) in by_name.iter().zip(sizes?) {
            self.line(&format!("    /// The serialized size of channel `{name}`."));
            self.line(&format!(
                "    pub const {}: ::fprime_config::FwSizeType = ({size}) as ::fprime_config::FwSizeType;",
                ident_of(self, *id)
            ));
        }
        self.line("}");
        // PacketIds / PacketGroups / PacketDataSizes / PacketEntryArrays
        self.line("");
        self.line("/// Packet identifiers.");
        self.line("pub mod PacketIds {");
        for (p, _) in &packets {
            self.line(&format!("    /// The identifier for packet `{}`.", p.name));
            self.line(&format!(
                "    pub const {}: ::fprime_config::FwTlmPacketizeIdType = {:#x};",
                ident(&p.name),
                p.id
            ));
        }
        self.line("}");
        self.line("");
        self.line("/// Packet groups (the packetizer's levels).");
        self.line("pub mod PacketGroups {");
        for (p, _) in &packets {
            self.line(&format!("    /// The group for packet `{}`.", p.name));
            self.line(&format!(
                "    pub const {}: ::fprime_config::FwChanIdType = {};",
                ident(&p.name),
                p.group
            ));
        }
        self.line("}");
        self.line("");
        self.line("/// Packet data sizes: the sum of the packet's channel sizes.");
        self.line("pub mod PacketDataSizes {");
        for (p, members) in &packets {
            let sum = if members.is_empty() {
                "0".to_string()
            } else {
                members
                    .iter()
                    .map(|id| format!("super::ChannelSizes::{}", ident_of(self, *id)))
                    .collect::<Vec<_>>()
                    .join(" + ")
            };
            let n = ident(&p.name);
            self.line(&format!("    /// The data size for packet `{}`.", p.name));
            self.line(&format!(
                "    pub const {n}: ::fprime_config::FwSizeType = {sum};"
            ));
            self.line(&format!("    const _: () = assert!({n} <= super::SizeBounds::PACKET_MAX_DATA_SIZE, \"packet data must fit in max data size\");"));
        }
        self.line("}");
        self.line("");
        self.line("/// The channel entries of each packet, in declaration order.");
        self.line("pub mod PacketEntryArrays {");
        self.line("    use super::TlmPacketizerChannelEntry;");
        for (p, members) in &packets {
            self.line(&format!(
                "    /// The channel entries for packet `{}`.",
                p.name
            ));
            self.line(&format!(
                "    pub const {}: [TlmPacketizerChannelEntry; {}] = [",
                ident(&p.name),
                members.len()
            ));
            for id in members {
                let c = ident_of(self, *id);
                self.line(&format!("        TlmPacketizerChannelEntry::new(super::ChannelIds::{c}, super::ChannelSizes::{c}),"));
            }
            self.line("    ];");
        }
        self.line("}");
        self.line("");
        self.line("/// The packets.");
        self.line("pub mod Packets {");
        self.line("    use super::TlmPacketizerPacket;");
        self.line("    /// The number of packets.");
        self.line(&format!(
            "    pub const NUM_PACKETS: usize = {};",
            packets.len()
        ));
        self.line(&format!("    const _: () = assert!(NUM_PACKETS <= {pk}::MAX_PACKETIZER_PACKETS, \"number of packets must be less than or equal to the maximum\");"));
        for (p, _) in &packets {
            let n = ident(&p.name);
            self.line(&format!(
                "    /// Packet `{}` (id {:#x}, group {}).",
                p.name, p.id, p.group
            ));
            self.line(&format!("    pub const {n}: TlmPacketizerPacket<'static> = TlmPacketizerPacket::new(&super::PacketEntryArrays::{n}, super::PacketIds::{n}, super::PacketGroups::{n});"));
        }
        self.line("}");
        // The extern variables.
        self.line("");
        self.line("/// The list of packets, in packet-name order (C++ `packetList`).");
        let list: Vec<String> = packets
            .iter()
            .map(|(p, _)| format!("Packets::{}", ident(&p.name)))
            .collect();
        self.line(&format!(
            "pub const PACKET_LIST: &[TlmPacketizerPacket<'static>] = &[{}];",
            list.join(", ")
        ));
        self.line("/// The omitted channels, in channel-name order (the entries of C++ `omittedChannels`).");
        self.line("pub const OMITTED_CHANNELS: &[TlmPacketizerChannelEntry] = &[");
        for (_, id) in &omitted_sorted {
            let c = ident_of(self, *id);
            self.line(&format!(
                "    TlmPacketizerChannelEntry::new(ChannelIds::{c}, ChannelSizes::{c}),"
            ));
        }
        self.line("];");
        self.dedent();
        self.line("}");
        Ok(())
    }
}
