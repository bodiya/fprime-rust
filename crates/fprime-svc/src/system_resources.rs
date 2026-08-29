//! # Svc::SystemResources — CPU / memory / disk telemetry (passive)
//!
//! C++ sources: `Svc/SystemResources/SystemResources.{cpp,hpp,fpp}` plus the
//! OSAL sampling back ends `Os/Linux/Cpu.cpp`, `Os/Linux/Memory.cpp` and
//! `Os/Posix/FileSystem.cpp`.
//! Analysis: `docs/cpp-analysis/svc-misc.md` (SystemResources section).
//!
//! The component is PASSIVE with one GUARDED `run` (`Svc.Sched`) input and
//! one GUARDED `ENABLE` command; sampling therefore happens on the rate
//! group's thread under the single component mutex.
//!
//! Sampling order per tick is the C++ order: `Cpu()`, `Mem()`, `PhysMem()`.
//!
//! ## Sampling seam
//!
//! `fprime-os` does not port `Os::Cpu` / `Os::Memory` (see
//! `docs/api-notes.md`, fprime-os deviation 5), so the sampling lives behind
//! the [`ResourceSampler`] trait implemented here:
//!
//! - [`ProcSampler`] (the default) reads `/proc/stat` and `/proc/meminfo`
//!   with plain file I/O — a direct port of the Linux `Os::Cpu::getTicks` /
//!   `Os::Memory::getUsage` implementations (note: `MemFree`, matching
//!   `sysinfo`'s `freeram`, NOT `MemAvailable`). On a system without
//!   `/proc` it reports [`GenericStatus::Error`], which makes the component
//!   skip those channels exactly like the C++ error path.
//! - free space is delegated to `fprime_os::filesystem::get_free_space`,
//!   which is `NotSupported` in this workspace (statvfs has no
//!   zero-dependency std equivalent), so `NON_VOLATILE_*` are skipped —
//!   the same behavior the C++ code has on VxWorks. A project supplies a
//!   real implementation through the OSAL seam or a custom sampler.
//! - tests inject a deterministic sampler with
//!   [`SystemResources::with_sampler`].

use fprime_comp::{CmdGlue, CmdPort, EventGlue, PassiveBase, PortRef, SchedPort, TlmGlue};
use fprime_config::{FwChanIdType, FwIndexType, FwOpcodeType, FwSizeType};
use fprime_fw::{CmdArgBuffer, CmdResponse, SerBuf, fpp_enum, fw_assert};
use fprime_os::filesystem;
use std::sync::{Arc, Mutex};

/// C++ `SystemResources::CPU_COUNT` — maximum number of cores reported as
/// telemetry.
pub const CPU_COUNT: usize = 16;

/// Hard-coded free-space path (C++ `PhysMem()` passes `"/"` literally).
pub const NON_VOLATILE_PATH: &str = "/";

fpp_enum! {
    /// FPP `enum SystemResourceEnabled : U8` — the `ENABLE` command argument.
    pub enum SystemResourceEnabled : u8 {
        /// Stop sampling and writing telemetry.
        Disabled = 0,
        /// Sample and write telemetry on every `run` tick.
        Enabled = 1,
    }
    default Disabled
}

/// Port of `Os::Generic::Status` (the CPU/memory sampling status).
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GenericStatus {
    /// Sample taken.
    OpOk = 0,
    /// Sample unavailable; the caller skips the affected channels.
    Error = 1,
}

/// Port of `Os::Generic::UsedTotal` (aliased in C++ as `Os::Cpu::Ticks` and
/// `Os::Memory::Usage`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct UsedTotal {
    /// Used ticks / used bytes.
    pub used: FwSizeType,
    /// Total ticks / total bytes.
    pub total: FwSizeType,
}

/// The `Os::Cpu` / `Os::Memory` / `Os::FileSystem::getFreeSpace` seam used by
/// [`SystemResources`]. Implementors must be cheap enough for a rate-group
/// tick and must never block.
pub trait ResourceSampler: Send + Sync {
    /// C++ `Os::Cpu::getCount`. `Error` makes the component report zero
    /// cores (constructor parity).
    fn cpu_count(&self, count: &mut FwSizeType) -> GenericStatus;

    /// C++ `Os::Cpu::getTicks(ticks, index)`.
    fn cpu_ticks(&self, ticks: &mut UsedTotal, cpu_index: FwSizeType) -> GenericStatus;

    /// C++ `Os::Memory::getUsage` — byte-valued totals.
    fn memory_usage(&self, usage: &mut UsedTotal) -> GenericStatus;

    /// C++ `Os::FileSystem::getFreeSpace(path, total, free)`.
    fn free_space(
        &self,
        path: &str,
        total: &mut FwSizeType,
        free: &mut FwSizeType,
    ) -> filesystem::Status;
}

/// Default sampler: `/proc/stat` + `/proc/meminfo` + the OSAL free-space
/// call. See the module header for the divergences.
#[derive(Debug, Clone, Copy, Default)]
pub struct ProcSampler;

/// Number of tick fields parsed out of a `/proc/stat` `cpu` line
/// (C++ `MAX_CPU_TICK_TYPES`).
const MAX_CPU_TICK_TYPES: usize = 8;
/// Index of the CPU number field in a `/proc/stat` line.
const PROC_CPU_NUMBER: usize = 0;
/// Index of the USER field.
const PROC_CPU_USER: usize = 1;
/// Index of the NICE field.
const PROC_CPU_NICE: usize = 2;
/// Index of the SYSTEM field.
const PROC_CPU_SYSTEM: usize = 3;
/// Index of the IDLE field.
const PROC_CPU_IDLE: usize = 4;

impl ProcSampler {
    /// Parse one `/proc/stat` line into the 8 leading tick fields.
    ///
    /// C++ parity (`Os/Linux/Cpu.cpp getCpuData`): the line must start with
    /// `"cpu"`, the first field after that prefix is the CPU index and must
    /// equal `cpu_index`, and every one of the 8 fields must parse.
    fn parse_cpu_line(
        line: &str,
        cpu_index: FwSizeType,
    ) -> Option<[FwSizeType; MAX_CPU_TICK_TYPES]> {
        let rest = line.strip_prefix("cpu")?;
        let mut data = [0 as FwSizeType; MAX_CPU_TICK_TYPES];
        let mut fields = rest.split_whitespace();
        for (i, slot) in data.iter_mut().enumerate() {
            let token = fields.next()?;
            *slot = token.parse::<FwSizeType>().ok()?;
            if i == PROC_CPU_NUMBER && *slot != cpu_index {
                return None;
            }
        }
        Some(data)
    }

    /// Count the per-core `cpuN` lines of `/proc/stat`.
    ///
    /// The C++ code calls `sysconf(_SC_NPROCESSORS_ONLN)`; counting the
    /// `/proc/stat` lines keeps `cpu_count` consistent with what
    /// [`Self::cpu_ticks`] can actually read (see the analysis's
    /// "Rust feasibility" note).
    fn count_from_proc() -> Option<FwSizeType> {
        let stat = std::fs::read_to_string("/proc/stat").ok()?;
        let mut count: FwSizeType = 0;
        for line in stat.lines() {
            match line.strip_prefix("cpu") {
                // The aggregate "cpu " line is not a core.
                Some(rest) if rest.starts_with(|c: char| c.is_ascii_digit()) => count += 1,
                Some(_) => {}
                None => break,
            }
        }
        Some(count)
    }

    /// Read a `kB`-valued `/proc/meminfo` field.
    fn meminfo_field(meminfo: &str, key: &str) -> Option<FwSizeType> {
        for line in meminfo.lines() {
            if let Some(rest) = line.strip_prefix(key) {
                let value = rest.trim_start_matches(':').split_whitespace().next()?;
                return value.parse::<FwSizeType>().ok();
            }
        }
        None
    }
}

impl ResourceSampler for ProcSampler {
    fn cpu_count(&self, count: &mut FwSizeType) -> GenericStatus {
        let cpus = Self::count_from_proc().or_else(|| {
            // Non-Linux fallback (the analysis's recommended substitute for
            // sysconf(_SC_NPROCESSORS_ONLN)).
            std::thread::available_parallelism()
                .ok()
                .map(|n| n.get() as FwSizeType)
        });
        match cpus {
            Some(n) if n > 0 => {
                *count = n;
                GenericStatus::OpOk
            }
            _ => {
                *count = 0;
                GenericStatus::Error
            }
        }
    }

    fn cpu_ticks(&self, ticks: &mut UsedTotal, cpu_index: FwSizeType) -> GenericStatus {
        // C++ parity: _getTicks re-checks the count and rejects an index
        // beyond it before touching the file.
        let mut count: FwSizeType = 0;
        if self.cpu_count(&mut count) != GenericStatus::OpOk || cpu_index >= count {
            return GenericStatus::Error;
        }
        let Ok(stat) = std::fs::read_to_string("/proc/stat") else {
            return GenericStatus::Error;
        };
        // The file starts with the aggregate cpu line, then one per core.
        let Some(line) = stat.lines().nth(cpu_index as usize + 1) else {
            return GenericStatus::Error;
        };
        match Self::parse_cpu_line(line, cpu_index) {
            Some(data) => {
                ticks.used = data[PROC_CPU_USER]
                    .wrapping_add(data[PROC_CPU_NICE])
                    .wrapping_add(data[PROC_CPU_SYSTEM]);
                ticks.total = ticks.used.wrapping_add(data[PROC_CPU_IDLE]);
                GenericStatus::OpOk
            }
            None => GenericStatus::Error,
        }
    }

    fn memory_usage(&self, usage: &mut UsedTotal) -> GenericStatus {
        let Ok(meminfo) = std::fs::read_to_string("/proc/meminfo") else {
            return GenericStatus::Error;
        };
        // sysinfo(2) parity: freeram, not MemAvailable.
        let (Some(total_kb), Some(free_kb)) = (
            Self::meminfo_field(&meminfo, "MemTotal"),
            Self::meminfo_field(&meminfo, "MemFree"),
        ) else {
            return GenericStatus::Error;
        };
        // C++ overflow guard: report 1/1 and ERROR rather than wrapping.
        let (Some(total), Some(used)) = (
            total_kb.checked_mul(1024),
            total_kb.saturating_sub(free_kb).checked_mul(1024),
        ) else {
            usage.total = 1;
            usage.used = 1;
            return GenericStatus::Error;
        };
        usage.total = total;
        usage.used = used;
        GenericStatus::OpOk
    }

    fn free_space(
        &self,
        path: &str,
        total: &mut FwSizeType,
        free: &mut FwSizeType,
    ) -> filesystem::Status {
        filesystem::get_free_space(path, total, free)
    }
}

/// Mutable component state; the `Mutex` is the C++ guarded-port mutex shared
/// by `run` and the `ENABLE` command.
struct SysResState {
    /// C++ `m_enable` (constructed `true`).
    enable: bool,
    /// C++ `m_cpu_count`, already clamped to [`CPU_COUNT`].
    cpu_count: usize,
    /// C++ `m_cpu`.
    cpu: [UsedTotal; CPU_COUNT],
    /// C++ `m_cpu_prev`.
    cpu_prev: [UsedTotal; CPU_COUNT],
    /// C++ `m_mem`.
    mem: UsedTotal,
}

/// One telemetry write produced by a sampling pass: `(channel id, value)`.
#[derive(Debug, Clone, Copy)]
enum TlmWrite {
    /// An `F32` channel (`CPU`, `CPU_00`..`CPU_15`).
    Cpu(FwChanIdType, f32),
    /// A `U64` channel (memory / non-volatile).
    Size(FwChanIdType, u64),
}

/// Maximum writes one sampling pass can produce: 16 per-core channels, the
/// CPU average, two memory channels and two non-volatile channels.
const MAX_TLM_WRITES: usize = CPU_COUNT + 5;

/// Fixed-capacity write list: a sampling pass allocates nothing (the
/// steady-state no-heap rule in `CONVENTIONS.md`).
struct TlmWrites {
    items: [Option<TlmWrite>; MAX_TLM_WRITES],
    count: usize,
}

impl TlmWrites {
    const fn new() -> Self {
        Self {
            items: [None; MAX_TLM_WRITES],
            count: 0,
        }
    }

    fn push(&mut self, write: TlmWrite) {
        // The capacity is the worst case, so overflow would be a coding
        // error, not a runtime condition.
        fw_assert!(self.count < MAX_TLM_WRITES, self.count as i32);
        self.items[self.count] = Some(write);
        self.count += 1;
    }

    fn iter(&self) -> impl Iterator<Item = TlmWrite> + '_ {
        self.items[..self.count].iter().flatten().copied()
    }
}

/// `Svc::SystemResources`.
pub struct SystemResources {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Command registration + response ports (`CmdReg`, `CmdStatus`).
    pub cmd: CmdGlue,
    /// Event ports (`Log`, `LogText`) and the `Time` port. No events are
    /// declared by the FPP model; the time port stamps telemetry.
    pub evt: EventGlue,
    /// Telemetry port (`Tlm`).
    pub tlm: TlmGlue,
    /// The sampling back end (see [`ResourceSampler`]).
    sampler: Box<dyn ResourceSampler>,
    state: Mutex<SysResState>,
}

impl SystemResources {
    /// Command `ENABLE(enable: SystemResourceEnabled)` — opcode 0, guarded.
    pub const OPCODE_ENABLE: FwOpcodeType = 0;

    /// Telemetry `MEMORY_TOTAL: U64` — "{} KB".
    pub const CHANID_MEMORY_TOTAL: FwChanIdType = 0;
    /// Telemetry `MEMORY_USED: U64` — "{} KB".
    pub const CHANID_MEMORY_USED: FwChanIdType = 1;
    /// Telemetry `NON_VOLATILE_TOTAL: U64` — "{} KB".
    pub const CHANID_NON_VOLATILE_TOTAL: FwChanIdType = 2;
    /// Telemetry `NON_VOLATILE_FREE: U64` — "{} KB".
    pub const CHANID_NON_VOLATILE_FREE: FwChanIdType = 3;
    /// Telemetry `CPU: F32` — the average over the sampled cores.
    pub const CHANID_CPU: FwChanIdType = 4;
    /// Telemetry `CPU_00: F32`; `CPU_nn` is `CHANID_CPU_00 + nn`.
    pub const CHANID_CPU_00: FwChanIdType = 5;

    /// Construct with the default `/proc`-based sampler.
    pub fn new(name: &str) -> Arc<Self> {
        Self::with_sampler(name, Box::new(ProcSampler))
    }

    /// Construct with an injected sampler (used by tests and by projects
    /// supplying their own OSAL back end).
    ///
    /// C++ constructor parity: the core count is read ONCE here, an `Error`
    /// status yields zero cores, and the count is clamped to [`CPU_COUNT`].
    pub fn with_sampler(name: &str, sampler: Box<dyn ResourceSampler>) -> Arc<Self> {
        let mut cpu_count: FwSizeType = 0;
        if sampler.cpu_count(&mut cpu_count) == GenericStatus::Error {
            cpu_count = 0;
        }
        let cpu_count = std::cmp::min(cpu_count as usize, CPU_COUNT);
        Arc::new(Self {
            base: PassiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            sampler,
            state: Mutex::new(SysResState {
                enable: true,
                cpu_count,
                cpu: [UsedTotal::default(); CPU_COUNT],
                cpu_prev: [UsedTotal::default(); CPU_COUNT],
                mem: UsedTotal::default(),
            }),
        })
    }

    fn id_base(&self) -> u32 {
        self.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_ENABLE]);
    }

    /// Number of cores this component will sample (clamped to
    /// [`CPU_COUNT`]).
    #[must_use]
    pub fn cpu_count(&self) -> usize {
        self.state.lock().unwrap().cpu_count
    }

    /// Whether telemetry sampling is enabled (C++ `m_enable`).
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.state.lock().unwrap().enable
    }

    // -- Input-port factories ------------------------------------------------

    /// `run` — GUARDED `Svc.Sched` input array of size 1.
    pub fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `CmdDisp` — the command input (`ENABLE` is a GUARDED command and so
    /// runs on the dispatcher's thread under the component mutex).
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ------------------------------------------------------------

    /// `run_handler` — GUARDED: runs on the caller's (rate group's) thread.
    ///
    /// Divergence (documented in `CONVENTIONS.md` terms): sampling happens
    /// under the component mutex exactly as in C++, but the telemetry port
    /// invocations are made after the lock is released. The write ORDER is
    /// unchanged, which is all that is observable.
    fn run_handler(&self, _port_num: FwIndexType, _tick_time_hz: u32) {
        let mut writes = TlmWrites::new();
        {
            let mut state = self.state.lock().unwrap();
            if !state.enable {
                return;
            }
            self.sample_cpu(&mut state, &mut writes);
            self.sample_mem(&mut state, &mut writes);
            self.sample_phys_mem(&mut writes);
        }

        let time_tag = self.evt.time_get();
        let id_base = self.id_base();
        for write in writes.iter() {
            match write {
                TlmWrite::Cpu(id, value) => self.tlm.tlm_write(id_base, id, &value, time_tag),
                TlmWrite::Size(id, value) => self.tlm.tlm_write(id_base, id, &value, time_tag),
            }
        }
    }

    /// C++ `compCpuUtil`: 100% when the total tick delta is zero (fast
    /// sampling) and when the division produces NaN.
    fn comp_cpu_util(current: UsedTotal, previous: UsedTotal) -> f32 {
        let mut util = 100.0f32;
        let total_delta = current.total.wrapping_sub(previous.total);
        if total_delta != 0 {
            let used_delta = current.used.wrapping_sub(previous.used);
            util = (used_delta as f32 / total_delta as f32) * 100.0;
            if util.is_nan() {
                util = 100.0;
            }
        }
        util
    }

    /// C++ `Cpu()`.
    fn sample_cpu(&self, state: &mut SysResState, writes: &mut TlmWrites) {
        let mut count: u32 = 0;
        let mut cpu_avg = 0.0f32;
        for i in 0..std::cmp::min(state.cpu_count, CPU_COUNT) {
            let mut ticks = UsedTotal::default();
            if self.sampler.cpu_ticks(&mut ticks, i as FwSizeType) != GenericStatus::OpOk {
                continue;
            }
            state.cpu[i] = ticks;
            // Counter went backwards (e.g. a counter reset): store the new
            // sample and skip telemetry for this core this cycle.
            if state.cpu[i].used < state.cpu_prev[i].used
                || state.cpu[i].total < state.cpu_prev[i].total
            {
                state.cpu_prev[i] = state.cpu[i];
                continue;
            }
            let cpu_util = Self::comp_cpu_util(state.cpu[i], state.cpu_prev[i]);
            cpu_avg += cpu_util;
            writes.push(TlmWrite::Cpu(
                Self::CHANID_CPU_00 + i as FwChanIdType,
                cpu_util,
            ));
            state.cpu_prev[i] = state.cpu[i];
            count += 1;
        }
        cpu_avg = if count == 0 {
            0.0
        } else {
            cpu_avg / count as f32
        };
        writes.push(TlmWrite::Cpu(Self::CHANID_CPU, cpu_avg));
    }

    /// C++ `Mem()` — bytes divided down to KiB with integer division.
    fn sample_mem(&self, state: &mut SysResState, writes: &mut TlmWrites) {
        let mut usage = UsedTotal::default();
        if self.sampler.memory_usage(&mut usage) == GenericStatus::OpOk {
            state.mem = usage;
            writes.push(TlmWrite::Size(
                Self::CHANID_MEMORY_TOTAL,
                usage.total / 1024,
            ));
            writes.push(TlmWrite::Size(Self::CHANID_MEMORY_USED, usage.used / 1024));
        }
    }

    /// C++ `PhysMem()` — hard-coded `"/"`, FREE written BEFORE TOTAL.
    fn sample_phys_mem(&self, writes: &mut TlmWrites) {
        let mut total: FwSizeType = 0;
        let mut free: FwSizeType = 0;
        if self
            .sampler
            .free_space(NON_VOLATILE_PATH, &mut total, &mut free)
            == filesystem::Status::OpOk
        {
            writes.push(TlmWrite::Size(Self::CHANID_NON_VOLATILE_FREE, free / 1024));
            writes.push(TlmWrite::Size(
                Self::CHANID_NON_VOLATILE_TOTAL,
                total / 1024,
            ));
        }
    }

    /// `ENABLE_cmdHandler` — GUARDED, always answers `Ok`.
    fn enable_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut raw: u8 = 0;
        if !args.deserialize_u8_be(&mut raw).is_ok() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        // C++ parity (FW_CMD_CHECK_RESIDUAL).
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        // Workspace discipline: an undeclared enum value is a
        // ValidationError (stock C++ generated code fails at deserialize
        // with FORMAT_ERROR).
        let Ok(enable) = SystemResourceEnabled::try_from(raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        {
            let mut state = self.state.lock().unwrap();
            state.enable = enable == SystemResourceEnabled::Enabled;
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }
}

impl SchedPort for SystemResources {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        self.run_handler(port_num, context);
    }
}

impl CmdPort for SystemResources {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_ENABLE => self.enable_cmd_handler(op_code, cmd_seq, args),
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, TimePort, TlmPort};
    use fprime_fw::{Endianness, Serialize, Time, TimeBase, TlmBuffer};

    const ID_BASE: u32 = 0x2A00;

    /// Deterministic sampler: every reading is scripted.
    #[derive(Default)]
    struct FakeSampler {
        count: FwSizeType,
        count_status: Option<GenericStatus>,
        /// Per-core scripted `(status, used, total)` sequences; each
        /// `cpu_ticks` call pops the next entry for that core.
        ticks: Mutex<Vec<Vec<(GenericStatus, u64, u64)>>>,
        mem: Option<(u64, u64)>,
        disk: Option<(u64, u64)>,
        /// Paths passed to `free_space`, in call order.
        disk_paths: Mutex<Vec<String>>,
    }

    /// Shared handle so a test can inspect what the component asked for.
    impl ResourceSampler for Arc<FakeSampler> {
        fn cpu_count(&self, count: &mut FwSizeType) -> GenericStatus {
            (**self).cpu_count(count)
        }
        fn cpu_ticks(&self, ticks: &mut UsedTotal, cpu_index: FwSizeType) -> GenericStatus {
            (**self).cpu_ticks(ticks, cpu_index)
        }
        fn memory_usage(&self, usage: &mut UsedTotal) -> GenericStatus {
            (**self).memory_usage(usage)
        }
        fn free_space(
            &self,
            path: &str,
            total: &mut FwSizeType,
            free: &mut FwSizeType,
        ) -> filesystem::Status {
            (**self).free_space(path, total, free)
        }
    }

    impl ResourceSampler for FakeSampler {
        fn cpu_count(&self, count: &mut FwSizeType) -> GenericStatus {
            *count = self.count;
            self.count_status.unwrap_or(GenericStatus::OpOk)
        }

        fn cpu_ticks(&self, ticks: &mut UsedTotal, cpu_index: FwSizeType) -> GenericStatus {
            let mut scripts = self.ticks.lock().unwrap();
            let Some(script) = scripts.get_mut(cpu_index as usize) else {
                return GenericStatus::Error;
            };
            if script.is_empty() {
                return GenericStatus::Error;
            }
            let (status, used, total) = script.remove(0);
            ticks.used = used;
            ticks.total = total;
            status
        }

        fn memory_usage(&self, usage: &mut UsedTotal) -> GenericStatus {
            match self.mem {
                Some((used, total)) => {
                    usage.used = used;
                    usage.total = total;
                    GenericStatus::OpOk
                }
                None => GenericStatus::Error,
            }
        }

        fn free_space(
            &self,
            path: &str,
            total: &mut FwSizeType,
            free: &mut FwSizeType,
        ) -> filesystem::Status {
            self.disk_paths.lock().unwrap().push(path.to_string());
            match self.disk {
                Some((t, f)) => {
                    *total = t;
                    *free = f;
                    filesystem::Status::OpOk
                }
                None => filesystem::Status::NotSupported,
            }
        }
    }

    #[derive(Default)]
    struct Ground {
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    }

    impl TlmPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm.lock().unwrap().push((id, val.as_slice().to_vec()));
        }
    }

    impl CmdRegPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.responses
                .lock()
                .unwrap()
                .push((op_code, cmd_seq, response));
        }
    }

    struct TimeStub;
    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 7, 8);
        }
    }

    fn build(sampler: FakeSampler) -> (Arc<SystemResources>, Arc<Ground>) {
        let (comp, ground, _sampler) = build_shared(sampler);
        (comp, ground)
    }

    fn build_shared(sampler: FakeSampler) -> (Arc<SystemResources>, Arc<Ground>, Arc<FakeSampler>) {
        let ground = Arc::new(Ground::default());
        let sampler = Arc::new(sampler);
        let comp = SystemResources::with_sampler("sysRes", Box::new(sampler.clone()));
        comp.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        (comp, ground, sampler)
    }

    fn tick(comp: &Arc<SystemResources>) {
        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 1);
    }

    fn f32_bytes(v: f32) -> Vec<u8> {
        let mut buf = fprime_fw::LinearBuffer::<8>::new();
        assert!(v.serialize_to(&mut buf, Endianness::Big).is_ok());
        buf.as_slice().to_vec()
    }

    fn u64_bytes(v: u64) -> Vec<u8> {
        v.to_be_bytes().to_vec()
    }

    fn send_cmd(comp: &Arc<SystemResources>, op_code: FwOpcodeType, seq: u32, bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(bytes).is_ok());
        let port = comp.cmd_in(0);
        port.target.invoke(port.port_num, op_code, seq, &mut args);
    }

    #[test]
    fn full_sampling_pass_writes_every_channel_in_cpp_order() {
        let sampler = FakeSampler {
            count: 2,
            ticks: Mutex::new(vec![
                vec![(GenericStatus::OpOk, 0, 0), (GenericStatus::OpOk, 50, 100)],
                vec![(GenericStatus::OpOk, 0, 0), (GenericStatus::OpOk, 25, 100)],
            ]),
            mem: Some((2048, 8192)),
            disk: Some((40960, 10240)),
            ..FakeSampler::default()
        };
        let (comp, ground) = build(sampler);
        assert_eq!(comp.cpu_count(), 2);

        // First tick establishes the baseline (prev = 0,0 -> delta used/total).
        tick(&comp);
        ground.tlm.lock().unwrap().clear();
        // Second tick: cpu0 = 50/100 = 50%, cpu1 = 25/100 = 25%, avg 37.5%.
        tick(&comp);

        let tlm = ground.tlm.lock().unwrap();
        let expected: Vec<(FwChanIdType, Vec<u8>)> = vec![
            (ID_BASE + SystemResources::CHANID_CPU_00, f32_bytes(50.0)),
            (
                ID_BASE + SystemResources::CHANID_CPU_00 + 1,
                f32_bytes(25.0),
            ),
            (ID_BASE + SystemResources::CHANID_CPU, f32_bytes(37.5)),
            (ID_BASE + SystemResources::CHANID_MEMORY_TOTAL, u64_bytes(8)),
            (ID_BASE + SystemResources::CHANID_MEMORY_USED, u64_bytes(2)),
            (
                ID_BASE + SystemResources::CHANID_NON_VOLATILE_FREE,
                u64_bytes(10),
            ),
            (
                ID_BASE + SystemResources::CHANID_NON_VOLATILE_TOTAL,
                u64_bytes(40),
            ),
        ];
        assert_eq!(*tlm, expected);
    }

    #[test]
    fn free_space_uses_the_hardcoded_root_path() {
        let sampler = FakeSampler {
            count: 0,
            disk: Some((1024, 512)),
            ..FakeSampler::default()
        };
        let (comp, _ground, sampler) = build_shared(sampler);
        tick(&comp);
        tick(&comp);
        // Gotcha: the free-space path is hard-coded "/" in PhysMem().
        assert_eq!(*sampler.disk_paths.lock().unwrap(), vec!["/", "/"]);
    }

    #[test]
    fn zero_total_delta_reports_one_hundred_percent() {
        // Gotcha: compCpuUtil returns 100.0 when the total tick delta is 0.
        assert_eq!(
            SystemResources::comp_cpu_util(
                UsedTotal { used: 5, total: 9 },
                UsedTotal { used: 5, total: 9 }
            ),
            100.0
        );
    }

    #[test]
    fn backwards_counter_skips_the_sample_but_stores_it() {
        // Gotcha: a counter that went backwards skips telemetry for that core
        // this cycle (and the average then counts zero cores).
        let sampler = FakeSampler {
            count: 1,
            ticks: Mutex::new(vec![vec![
                (GenericStatus::OpOk, 100, 200),
                (GenericStatus::OpOk, 10, 20),
                (GenericStatus::OpOk, 15, 40),
            ]]),
            ..FakeSampler::default()
        };
        let (comp, ground) = build(sampler);
        tick(&comp); // baseline, emits CPU_00 + CPU
        ground.tlm.lock().unwrap().clear();
        tick(&comp); // backwards -> only the CPU average (0.0, count == 0)
        {
            let tlm = ground.tlm.lock().unwrap();
            assert_eq!(
                *tlm,
                vec![(ID_BASE + SystemResources::CHANID_CPU, f32_bytes(0.0))]
            );
        }
        ground.tlm.lock().unwrap().clear();
        tick(&comp); // resumes from the stored sample: (15-10)/(40-20) = 25%
        let tlm = ground.tlm.lock().unwrap();
        assert_eq!(
            *tlm,
            vec![
                (ID_BASE + SystemResources::CHANID_CPU_00, f32_bytes(25.0)),
                (ID_BASE + SystemResources::CHANID_CPU, f32_bytes(25.0)),
            ]
        );
    }

    #[test]
    fn sampler_errors_skip_only_their_own_channels() {
        // CPU read fails, memory unavailable, free space NotSupported:
        // only the CPU average (0.0) is written.
        let sampler = FakeSampler {
            count: 1,
            ticks: Mutex::new(vec![vec![(GenericStatus::Error, 0, 0)]]),
            ..FakeSampler::default()
        };
        let (comp, ground) = build(sampler);
        tick(&comp);
        let tlm = ground.tlm.lock().unwrap();
        assert_eq!(
            *tlm,
            vec![(ID_BASE + SystemResources::CHANID_CPU, f32_bytes(0.0))]
        );
    }

    #[test]
    fn cpu_count_is_clamped_and_zeroed_on_error() {
        let clamped = SystemResources::with_sampler(
            "clamped",
            Box::new(FakeSampler {
                count: 64,
                ..FakeSampler::default()
            }),
        );
        assert_eq!(clamped.cpu_count(), CPU_COUNT);

        let errored = SystemResources::with_sampler(
            "errored",
            Box::new(FakeSampler {
                count: 8,
                count_status: Some(GenericStatus::Error),
                ..FakeSampler::default()
            }),
        );
        assert_eq!(errored.cpu_count(), 0);
    }

    #[test]
    fn enable_command_gates_sampling_and_always_responds_ok() {
        let sampler = FakeSampler {
            count: 0,
            mem: Some((1024, 4096)),
            ..FakeSampler::default()
        };
        let (comp, ground) = build(sampler);
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![ID_BASE + SystemResources::OPCODE_ENABLE]
        );
        assert!(comp.is_enabled());

        let opcode = ID_BASE + SystemResources::OPCODE_ENABLE;
        send_cmd(&comp, opcode, 1, &[0x00]); // DISABLED
        assert!(!comp.is_enabled());
        tick(&comp);
        assert!(ground.tlm.lock().unwrap().is_empty(), "disabled: no writes");

        send_cmd(&comp, opcode, 2, &[0x01]); // ENABLED
        assert!(comp.is_enabled());
        tick(&comp);
        assert!(!ground.tlm.lock().unwrap().is_empty());

        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 1, CmdResponse::Ok), (opcode, 2, CmdResponse::Ok)]
        );
    }

    #[test]
    fn enable_command_status_branches() {
        let (comp, ground) = build(FakeSampler::default());
        let opcode = ID_BASE + SystemResources::OPCODE_ENABLE;
        send_cmd(&comp, opcode, 1, &[]); // short args
        send_cmd(&comp, opcode, 2, &[0x01, 0x02]); // residual byte
        send_cmd(&comp, opcode, 3, &[0x07]); // undeclared enum value
        send_cmd(&comp, ID_BASE + 0x55, 4, &[]); // unknown opcode
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (opcode, 1, CmdResponse::FormatError),
                (opcode, 2, CmdResponse::FormatError),
                (opcode, 3, CmdResponse::ValidationError),
                (ID_BASE + 0x55, 4, CmdResponse::InvalidOpcode),
            ]
        );
    }

    #[test]
    fn proc_sampler_parses_a_stat_line() {
        let data = ProcSampler::parse_cpu_line("cpu0 1 2 3 4 5 6 7 8", 0).unwrap();
        assert_eq!(data, [0, 1, 2, 3, 4, 5, 6, 7]);
        // Field 0 must equal the requested index.
        assert!(ProcSampler::parse_cpu_line("cpu1 1 2 3 4 5 6 7 8", 0).is_none());
        // A line outside the cpu section is rejected.
        assert!(ProcSampler::parse_cpu_line("intr 1 2 3 4 5 6 7 8", 0).is_none());
        // Too few fields.
        assert!(ProcSampler::parse_cpu_line("cpu0 1 2 3", 0).is_none());
    }

    #[test]
    fn proc_sampler_parses_meminfo_fields() {
        let meminfo = "MemTotal:       16316000 kB\nMemFree:         2624936 kB\n";
        assert_eq!(
            ProcSampler::meminfo_field(meminfo, "MemTotal"),
            Some(16_316_000)
        );
        assert_eq!(
            ProcSampler::meminfo_field(meminfo, "MemFree"),
            Some(2_624_936)
        );
        assert_eq!(ProcSampler::meminfo_field(meminfo, "MemAvailable"), None);
    }

    #[test]
    fn telemetry_ids_match_the_fpp_dictionary() {
        assert_eq!(SystemResources::CHANID_MEMORY_TOTAL, 0);
        assert_eq!(SystemResources::CHANID_MEMORY_USED, 1);
        assert_eq!(SystemResources::CHANID_NON_VOLATILE_TOTAL, 2);
        assert_eq!(SystemResources::CHANID_NON_VOLATILE_FREE, 3);
        assert_eq!(SystemResources::CHANID_CPU, 4);
        assert_eq!(SystemResources::CHANID_CPU_00, 5);
        // CPU_15 is the last declared channel.
        assert_eq!(
            SystemResources::CHANID_CPU_00 + (CPU_COUNT as FwChanIdType - 1),
            20
        );
        assert_eq!(SystemResources::OPCODE_ENABLE, 0);
        assert_eq!(SystemResourceEnabled::Enabled.as_repr(), 1);
        assert_eq!(SystemResourceEnabled::SERIALIZED_SIZE, 1);
    }
}
