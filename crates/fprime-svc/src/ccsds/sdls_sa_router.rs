//! # SdlsSaRouter — port of `Svc::Ccsds::SdlsSaRouter` (passive)
//!
//! C++ sources: `Svc/Ccsds/SdlsSaRouter/SdlsSaRouter.{cpp,hpp,fpp}`, the
//! `SdlsSaRouterCfg.fpp` config module (in `fprime_config::sdls`) and its
//! `docs/sdd.md`.
//!
//! Routes SDLS encryption/decryption requests to downstream crypto
//! components by security association: the SA index arriving on `dataIn`
//! is looked up in an SA-to-port map (the compile-time `SdlsCfg.SaMap`,
//! replaceable per instance with [`SdlsSaRouter::configure`]) and the
//! request goes out on `saDataOut[port]`. An unmapped SA passes
//! `UNKNOWN_SA` forward on `dataOut` with the untouched buffer; a mapped
//! port that is out of range or unconnected passes `UNKNOWN_PORT`.
//!
//! Processed data from a crypto component (`saDataIn[port]`) goes upstream
//! on `dataOut`, and the originating port is remembered in an
//! outstanding-buffer table keyed by the buffer's allocation context, so
//! the ownership return on `dataReturnIn` is routed back to
//! `saDataReturnOut[port]` — or upstream on `bufferReturnOut` for buffers
//! the router itself forwarded on a routing error. A full table raises
//! `TrackingTableFull` and drops the data; an untracked return raises
//! `UntrackedBufferReturned` and goes upstream.
//!
//! `dataIn` is guarded; `dataReturnIn`, `saDataIn` and `saBufferReturnIn`
//! are sync because they are called back on the caller's thread while the
//! guard is held. The outstanding table has its own mutex, held per map
//! operation (C++ `m_outstandingLock`).
//!
//! The two `Fw::ArrayMap`s become fixed-capacity arrays with the same
//! insert semantics (an existing key is overwritten; a new key fails when
//! full).

use crate::ccsds::types::{CcsdsSdlsDataPort, CcsdsSdlsEncryptionPort, SaMapEntry, SdlsStatus};
use fprime_comp::{ComDataWithContextPort, EventGlue, OutputPort, PassiveBase, PortRef};
use fprime_config::sdls::{
    SA_MAP, SA_ROUTER_MAP_ENTRY_COUNT, SA_ROUTER_MAX_OUTSTANDING_BUFFERS, SA_ROUTER_PORT_COUNT,
};
use fprime_config::{FwEventIdType, FwIndexType};
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{Buffer, FrameContext, LogSeverity, SerializeStatus, fw_assert};

/// `TrackingTableFull()` — WARNING_HI.
pub const EVENTID_TRACKING_TABLE_FULL: FwEventIdType = 0;
/// `UntrackedBufferReturned()` — WARNING_HI.
pub const EVENTID_UNTRACKED_BUFFER_RETURNED: FwEventIdType = 1;

/// Sentinel port index marking buffers the router forwarded itself on a
/// routing error (`ROUTER_ERROR_PORT`).
pub const ROUTER_ERROR_PORT: FwIndexType = -1;

/// The SA-to-port map type (`SdlsCfg.SaMap`).
pub type SaMap = [SaMapEntry; SA_ROUTER_MAP_ENTRY_COUNT];

/// The compile-time default map (`SdlsCfg.SaMap` defaults).
pub fn default_sa_map() -> SaMap {
    SA_MAP.map(|(sa, port)| SaMapEntry::new(sa, port))
}

/// A fixed-capacity array map with `Fw::ArrayMap` insert semantics.
#[derive(Debug)]
struct ArrayMap<K: Copy + PartialEq, V: Copy, const N: usize> {
    entries: [Option<(K, V)>; N],
}

impl<K: Copy + PartialEq, V: Copy, const N: usize> ArrayMap<K, V, N> {
    fn new() -> Self {
        Self { entries: [None; N] }
    }
    fn clear(&mut self) {
        self.entries = [None; N];
    }
    fn find(&self, key: K) -> Option<V> {
        self.entries
            .iter()
            .flatten()
            .find(|(k, _)| *k == key)
            .map(|(_, v)| *v)
    }
    /// Overwrites an existing key; otherwise fills a free slot; false when
    /// full.
    fn insert(&mut self, key: K, value: V) -> bool {
        if let Some(entry) = self.entries.iter_mut().flatten().find(|(k, _)| *k == key) {
            entry.1 = value;
            return true;
        }
        match self.entries.iter_mut().find(|e| e.is_none()) {
            Some(slot) => {
                *slot = Some((key, value));
                true
            }
            None => false,
        }
    }
    fn remove(&mut self, key: K) {
        if let Some(slot) = self
            .entries
            .iter_mut()
            .find(|e| matches!(e, Some((k, _)) if *k == key))
        {
            *slot = None;
        }
    }
    fn len(&self) -> usize {
        self.entries.iter().flatten().count()
    }
}

/// `Svc::Ccsds::SdlsSaRouter` — SA-indexed SDLS request router.
pub struct SdlsSaRouter {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// `dataOut` — status + processed data upstream.
    pub data_out: OutputPort<dyn CcsdsSdlsDataPort>,
    /// `bufferReturnOut` — incoming IV/data buffers back for deallocation.
    pub buffer_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `saDataOut[n]` — requests to the mapped crypto component.
    pub sa_data_out: [OutputPort<dyn CcsdsSdlsEncryptionPort>; SA_ROUTER_PORT_COUNT],
    /// `saDataReturnOut[n]` — processed buffers back to their crypto
    /// component.
    pub sa_data_return_out: [OutputPort<dyn ComDataWithContextPort>; SA_ROUTER_PORT_COUNT],
    sa_map: Mutex<ArrayMap<u16, FwIndexType, SA_ROUTER_MAP_ENTRY_COUNT>>,
    outstanding: Mutex<ArrayMap<u32, FwIndexType, SA_ROUTER_MAX_OUTSTANDING_BUFFERS>>,
    guard: Mutex<()>,
}

impl SdlsSaRouter {
    /// Construct the component with the compile-time default map loaded.
    pub fn new(name: &str) -> Arc<Self> {
        let comp = Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            data_out: OutputPort::new(),
            buffer_return_out: OutputPort::new(),
            sa_data_out: core::array::from_fn(|_| OutputPort::new()),
            sa_data_return_out: core::array::from_fn(|_| OutputPort::new()),
            sa_map: Mutex::new(ArrayMap::new()),
            outstanding: Mutex::new(ArrayMap::new()),
            guard: Mutex::new(()),
        });
        comp.configure(&default_sa_map());
        comp
    }

    /// Replace this instance's SA-to-port table (topology setup, before any
    /// frame is routed). Each direction needs its own table: an SA is
    /// simplex (CCSDS 355.0-B-2 2.3.1.1).
    pub fn configure(&self, sa_map: &SaMap) {
        let mut map = self.sa_map.lock();
        map.clear();
        for (i, entry) in sa_map.iter().enumerate() {
            let inserted = map.insert(entry.security_association_index, entry.port_index);
            fw_assert!(inserted, i as i32);
        }
    }

    /// Number of outstanding (tracked) processed buffers.
    pub fn outstanding_count(&self) -> usize {
        self.outstanding.lock().len()
    }

    // -- Input ports ------------------------------------------------------------

    /// `dataIn` — GUARDED: the SA index and buffer to route.
    pub fn data_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn CcsdsSdlsEncryptionPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `saDataIn[n]` — SYNC: a crypto component's result.
    pub fn sa_data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CcsdsSdlsDataPort> {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < SA_ROUTER_PORT_COUNT,
            i32::from(port_num)
        );
        PortRef::new(self.clone(), port_num)
    }

    /// `dataReturnIn` — SYNC: ownership of a buffer sent on `dataOut`.
    pub fn data_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(
            Arc::new(DataReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `saBufferReturnIn[n]` — SYNC: an IV/data buffer back from a crypto
    /// component.
    pub fn sa_buffer_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ComDataWithContextPort> {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < SA_ROUTER_PORT_COUNT,
            i32::from(port_num)
        );
        PortRef::new(
            Arc::new(SaBufferReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    // -- Handlers ---------------------------------------------------------------

    fn data_in_handler(
        &self,
        _port_num: FwIndexType,
        security_association_index: u16,
        data: Buffer,
        context: &FrameContext,
    ) {
        let _guard = self.guard.lock();
        let found = self.sa_map.lock().find(security_association_index);
        let mut error_status = SdlsStatus::UnknownSa;
        if let Some(output_port) = found {
            if output_port >= 0 && (output_port as usize) < SA_ROUTER_PORT_COUNT {
                if let Some(p) = self.sa_data_out[output_port as usize].try_get() {
                    p.target
                        .invoke(p.port_num, security_association_index, data, context);
                    return;
                }
            }
            error_status = SdlsStatus::UnknownPort;
        }
        // Routing failed: pass the status forward with the untouched buffer,
        // tracked so its return goes back upstream.
        let inserted = self
            .outstanding
            .lock()
            .insert(data.context(), ROUTER_ERROR_PORT);
        if !inserted {
            self.log_tracking_table_full();
            self.buffer_return_out(data, context);
            return;
        }
        let p = self.data_out.get();
        p.target.invoke(p.port_num, error_status, data, context);
    }

    fn data_return_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        let found = {
            let mut table = self.outstanding.lock();
            let found = table.find(data.context());
            if found.is_some() {
                table.remove(data.context());
            }
            found
        };
        match found {
            None => {
                // Untracked (e.g. lost to a context-key collision).
                self.evt.log_event(
                    self.base.get_id_base(),
                    EVENTID_UNTRACKED_BUFFER_RETURNED,
                    LogSeverity::WarningHi,
                    "SA router received an untracked buffer return",
                    |_buf| SerializeStatus::Ok,
                );
                self.buffer_return_out(data, context);
            }
            Some(ROUTER_ERROR_PORT) => self.buffer_return_out(data, context),
            Some(port) => {
                let p = self.sa_data_return_out[port as usize].get();
                p.target.invoke(p.port_num, data, context);
            }
        }
    }

    fn sa_buffer_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        context: &FrameContext,
    ) {
        self.buffer_return_out(data, context);
    }

    fn sa_data_in_handler(
        &self,
        port_num: FwIndexType,
        status: SdlsStatus,
        data: Buffer,
        context: &FrameContext,
    ) {
        let inserted = self.outstanding.lock().insert(data.context(), port_num);
        if !inserted {
            // Table full: drop the data, ownership back to the component.
            self.log_tracking_table_full();
            let p = self.sa_data_return_out[port_num as usize].get();
            p.target.invoke(p.port_num, data, context);
            return;
        }
        let p = self.data_out.get();
        p.target.invoke(p.port_num, status, data, context);
    }

    fn buffer_return_out(&self, data: Buffer, context: &FrameContext) {
        let p = self.buffer_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    fn log_tracking_table_full(&self) {
        self.evt.log_event(
            self.base.get_id_base(),
            EVENTID_TRACKING_TABLE_FULL,
            LogSeverity::WarningHi,
            "SA router tracking table full: data dropped and buffer returned",
            |_buf| SerializeStatus::Ok,
        );
    }
}

impl CcsdsSdlsEncryptionPort for SdlsSaRouter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        security_association_index: u16,
        data: Buffer,
        context: &FrameContext,
    ) {
        self.data_in_handler(port_num, security_association_index, data, context);
    }
}

impl CcsdsSdlsDataPort for SdlsSaRouter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        status: SdlsStatus,
        data: Buffer,
        context: &FrameContext,
    ) {
        self.sa_data_in_handler(port_num, status, data, context);
    }
}

/// Adapter for `dataReturnIn`.
struct DataReturnInAdapter {
    comp: Arc<SdlsSaRouter>,
}

impl ComDataWithContextPort for DataReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_return_in_handler(port_num, data, context);
    }
}

/// Adapter for `saBufferReturnIn[n]`.
struct SaBufferReturnInAdapter {
    comp: Arc<SdlsSaRouter>,
}

impl ComDataWithContextPort for SaBufferReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp
            .sa_buffer_return_in_handler(port_num, data, context);
    }
}

// ---------------------------------------------------------------------------
// Tests — port of SdlsSaRouterTester (+ Rules/Route.cpp, Rules/DataFlow.cpp)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::LogPort;
    use fprime_config::sdls::{SA_ROUTER_PORT_PLAINTEXT, SA_ROUTER_PORT_UNCONNECTED};
    use fprime_fw::{LogBuffer, Time};
    use std::collections::HashMap;
    use std::sync::Mutex as StdMutex;
    use std::sync::Weak;

    const TEST_BUFFER_SIZE: usize = 16;

    /// One downstream port of the harness (records which index it is).
    struct SaPort {
        index: FwIndexType,
        h: Arc<Harness>,
    }

    impl CcsdsSdlsEncryptionPort for SaPort {
        fn invoke(&self, _p: FwIndexType, sa: u16, data: Buffer, context: &FrameContext) {
            self.h
                .sa_data_out
                .lock()
                .unwrap()
                .push((self.index, sa, data.context(), *context));
            if *self.h.loopback.lock().unwrap() {
                let router = self.h.router.lock().unwrap().upgrade().unwrap();
                let p = router.sa_data_in(self.index);
                p.target
                    .invoke(p.port_num, SdlsStatus::Success, data, context);
            }
        }
    }

    impl ComDataWithContextPort for SaPort {
        fn invoke(&self, _p: FwIndexType, data: Buffer, context: &FrameContext) {
            self.h
                .sa_data_return_out
                .lock()
                .unwrap()
                .push((self.index, data.context()));
            if *self.h.loopback.lock().unwrap() {
                let router = self.h.router.lock().unwrap().upgrade().unwrap();
                let p = router.sa_buffer_return_in(self.index);
                p.target.invoke(p.port_num, data, context);
            }
        }
    }

    #[derive(Default)]
    struct Harness {
        sa_data_out: StdMutex<Vec<(FwIndexType, u16, u32, FrameContext)>>,
        sa_data_return_out: StdMutex<Vec<(FwIndexType, u32)>>,
        data_out: StdMutex<Vec<(SdlsStatus, u32)>>,
        buffer_return_out: StdMutex<Vec<u32>>,
        events: StdMutex<Vec<FwEventIdType>>,
        loopback: StdMutex<bool>,
        router: StdMutex<Weak<SdlsSaRouter>>,
    }

    impl CcsdsSdlsDataPort for Harness {
        fn invoke(&self, _p: FwIndexType, status: SdlsStatus, data: Buffer, _c: &FrameContext) {
            self.data_out.lock().unwrap().push((status, data.context()));
        }
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, _p: FwIndexType, data: Buffer, _c: &FrameContext) {
            self.buffer_return_out.lock().unwrap().push(data.context());
        }
    }

    impl LogPort for Harness {
        fn invoke(
            &self,
            _p: FwIndexType,
            id: FwEventIdType,
            _t: &mut Time,
            _s: LogSeverity,
            _a: &mut LogBuffer,
        ) {
            self.events.lock().unwrap().push(id);
        }
    }

    impl Harness {
        fn clear(&self) {
            self.sa_data_out.lock().unwrap().clear();
            self.sa_data_return_out.lock().unwrap().clear();
            self.data_out.lock().unwrap().clear();
            self.buffer_return_out.lock().unwrap().clear();
            self.events.lock().unwrap().clear();
        }
    }

    /// The tester wiring: every saDataOut connected except UNCONNECTED.
    fn build() -> (Arc<SdlsSaRouter>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        let r = SdlsSaRouter::new("SdlsSaRouter");
        *h.router.lock().unwrap() = Arc::downgrade(&r);
        r.data_out.connect(h.clone(), 0);
        r.buffer_return_out.connect(h.clone(), 0);
        r.evt.log_out.connect(h.clone(), 0);
        for i in 0..SA_ROUTER_PORT_COUNT {
            let port = Arc::new(SaPort {
                index: i as FwIndexType,
                h: h.clone(),
            });
            if i as FwIndexType != SA_ROUTER_PORT_UNCONNECTED {
                r.sa_data_out[i].connect(port.clone(), 0);
            }
            r.sa_data_return_out[i].connect(port, 0);
        }
        (r, h)
    }

    fn pool_buffer(index: u32) -> Buffer {
        let mut b = Buffer::allocate(TEST_BUFFER_SIZE);
        b.set_context(index);
        b
    }

    fn data_in(r: &Arc<SdlsSaRouter>, sa: u16, b: Buffer) {
        let p = r.data_in(0);
        p.target.invoke(p.port_num, sa, b, &FrameContext::default());
    }

    fn sa_data_in(r: &Arc<SdlsSaRouter>, port: FwIndexType, status: SdlsStatus, b: Buffer) {
        let p = r.sa_data_in(port);
        p.target
            .invoke(p.port_num, status, b, &FrameContext::default());
    }

    fn data_return_in(r: &Arc<SdlsSaRouter>, b: Buffer) {
        let p = r.data_return_in(0);
        p.target.invoke(p.port_num, b, &FrameContext::default());
    }

    fn is_mapped(sa: u16) -> bool {
        SA_MAP.iter().any(|(s, _)| *s == sa)
    }

    #[test]
    fn default_map_matches_config() {
        let map = default_sa_map();
        assert_eq!(map[0], SaMapEntry::new(1, SA_ROUTER_PORT_PLAINTEXT));
        assert_eq!(map[1], SaMapEntry::new(0, SA_ROUTER_PORT_UNCONNECTED));
    }

    /// Route.KnownSa.
    #[test]
    fn route_known_sa() {
        let (r, h) = build();
        data_in(&r, 1, pool_buffer(0));
        let out = h.sa_data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(
            (out[0].0, out[0].1, out[0].2),
            (SA_ROUTER_PORT_PLAINTEXT, 1, 0)
        );
        assert!(h.data_out.lock().unwrap().is_empty());
    }

    /// Route.UnknownSa and Route.UnknownPort.
    #[test]
    fn route_errors() {
        let (r, h) = build();
        data_in(&r, 100, pool_buffer(0));
        assert!(h.sa_data_out.lock().unwrap().is_empty());
        assert_eq!(
            *h.data_out.lock().unwrap(),
            vec![(SdlsStatus::UnknownSa, 0)]
        );
        h.clear();
        // SA 0 maps to the unconnected port.
        data_in(&r, 0, pool_buffer(1));
        assert!(h.sa_data_out.lock().unwrap().is_empty());
        assert_eq!(
            *h.data_out.lock().unwrap(),
            vec![(SdlsStatus::UnknownPort, 1)]
        );
        // Both error buffers return upstream when handed back.
        h.clear();
        data_return_in(&r, pool_buffer(0));
        data_return_in(&r, pool_buffer(1));
        assert_eq!(*h.buffer_return_out.lock().unwrap(), vec![0, 1]);
        assert!(h.sa_data_return_out.lock().unwrap().is_empty());
        assert!(h.events.lock().unwrap().is_empty());
    }

    /// A mapped port index out of range is UNKNOWN_PORT too.
    #[test]
    fn out_of_range_port_is_unknown_port() {
        let (r, h) = build();
        r.configure(&[SaMapEntry::new(9, 5), SaMapEntry::new(10, -2)]);
        data_in(&r, 9, pool_buffer(0));
        data_in(&r, 10, pool_buffer(1));
        data_in(&r, 1, pool_buffer(2));
        assert_eq!(
            *h.data_out.lock().unwrap(),
            vec![
                (SdlsStatus::UnknownPort, 0),
                (SdlsStatus::UnknownPort, 1),
                (SdlsStatus::UnknownSa, 2)
            ]
        );
    }

    /// DataFlow.ProcessedData + ProcessedDataReturn.
    #[test]
    fn processed_data_and_return() {
        let (r, h) = build();
        sa_data_in(&r, 1, SdlsStatus::DecryptionFailure, pool_buffer(2));
        assert_eq!(
            *h.data_out.lock().unwrap(),
            vec![(SdlsStatus::DecryptionFailure, 2)]
        );
        h.clear();
        // The return is keyed by context, so a window shift is irrelevant.
        let mut b = pool_buffer(2);
        b.advance(3);
        data_return_in(&r, b);
        assert_eq!(*h.sa_data_return_out.lock().unwrap(), vec![(1, 2)]);
        assert!(h.buffer_return_out.lock().unwrap().is_empty());
        assert_eq!(r.outstanding_count(), 0);
    }

    /// DataFlow.BufferReturn.
    #[test]
    fn buffer_return() {
        let (r, h) = build();
        for port in 0..SA_ROUTER_PORT_COUNT as FwIndexType {
            let p = r.sa_buffer_return_in(port);
            p.target
                .invoke(p.port_num, pool_buffer(7), &FrameContext::default());
        }
        assert_eq!(*h.buffer_return_out.lock().unwrap(), vec![7, 7]);
    }

    fn fill_outstanding_table(r: &Arc<SdlsSaRouter>) {
        for i in 0..SA_ROUTER_MAX_OUTSTANDING_BUFFERS as u32 {
            sa_data_in(r, 0, SdlsStatus::Success, pool_buffer(i));
        }
        assert_eq!(r.outstanding_count(), SA_ROUTER_MAX_OUTSTANDING_BUFFERS);
    }

    #[test]
    fn table_full_data_in() {
        let (r, h) = build();
        fill_outstanding_table(&r);
        h.clear();
        data_in(
            &r,
            100,
            pool_buffer(SA_ROUTER_MAX_OUTSTANDING_BUFFERS as u32 + 1),
        );
        assert!(h.data_out.lock().unwrap().is_empty());
        assert_eq!(*h.buffer_return_out.lock().unwrap(), vec![5]);
        assert_eq!(*h.events.lock().unwrap(), vec![EVENTID_TRACKING_TABLE_FULL]);
    }

    #[test]
    fn untracked_data_return() {
        let (r, h) = build();
        data_return_in(&r, pool_buffer(5));
        assert!(h.sa_data_return_out.lock().unwrap().is_empty());
        assert_eq!(*h.buffer_return_out.lock().unwrap(), vec![5]);
        assert_eq!(
            *h.events.lock().unwrap(),
            vec![EVENTID_UNTRACKED_BUFFER_RETURNED]
        );
    }

    #[test]
    fn table_full_sa_data_in() {
        let (r, h) = build();
        fill_outstanding_table(&r);
        h.clear();
        sa_data_in(&r, 1, SdlsStatus::Success, pool_buffer(5));
        assert!(h.data_out.lock().unwrap().is_empty());
        assert_eq!(*h.sa_data_return_out.lock().unwrap(), vec![(1, 5)]);
        assert_eq!(*h.events.lock().unwrap(), vec![EVENTID_TRACKING_TABLE_FULL]);
    }

    /// The crypto component calls back synchronously while the guard is
    /// held (SVC-CCSDS-SDLS-SA-ROUTER-008).
    #[test]
    fn synchronous_crypto_loopback() {
        let (r, h) = build();
        *h.loopback.lock().unwrap() = true;
        data_in(&r, 1, pool_buffer(3));
        assert_eq!(h.sa_data_out.lock().unwrap().len(), 1);
        assert_eq!(*h.data_out.lock().unwrap(), vec![(SdlsStatus::Success, 3)]);
        h.clear();
        data_return_in(&r, pool_buffer(3));
        assert_eq!(
            *h.sa_data_return_out.lock().unwrap(),
            vec![(SA_ROUTER_PORT_PLAINTEXT, 3)]
        );
        assert_eq!(*h.buffer_return_out.lock().unwrap(), vec![3]);
    }

    /// `configure` replaces the table per instance.
    #[test]
    fn configure_replaces_the_map() {
        let (r, h) = build();
        r.configure(&[SaMapEntry::new(7, 1), SaMapEntry::new(8, 0)]);
        data_in(&r, 7, pool_buffer(0));
        data_in(&r, 1, pool_buffer(1));
        assert_eq!(h.sa_data_out.lock().unwrap().len(), 1);
        assert_eq!(
            *h.data_out.lock().unwrap(),
            vec![(SdlsStatus::UnknownSa, 1)]
        );
    }

    /// RandomizedTesting: all rules interleaved against a shadow table.
    #[test]
    fn randomized_testing() {
        let (r, h) = build();
        let mut rng: u64 = 0x5DEE_CE66;
        let mut pick = |lo: u64, hi: u64| {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            lo + rng % (hi - lo + 1)
        };
        // shadow: pool index -> originating port
        let mut shadow: HashMap<u32, FwIndexType> = HashMap::new();
        let free = |shadow: &HashMap<u32, FwIndexType>| {
            (0..SA_ROUTER_MAX_OUTSTANDING_BUFFERS as u32).find(|i| !shadow.contains_key(i))
        };
        let mut steps = 0;
        while steps < 10_000 {
            h.clear();
            match pick(0, 5) {
                0 => {
                    // Route.KnownSa (SA 1 -> PLAINTEXT)
                    data_in(&r, 1, pool_buffer(100));
                    assert_eq!(h.sa_data_out.lock().unwrap()[0].0, SA_ROUTER_PORT_PLAINTEXT);
                    assert!(h.data_out.lock().unwrap().is_empty());
                }
                1 | 2 => {
                    // Route.UnknownSa / Route.UnknownPort
                    let Some(index) = free(&shadow) else { continue };
                    let (sa, status) = if pick(0, 1) == 0 {
                        let mut sa = pick(0, 0xFFFF) as u16;
                        while is_mapped(sa) {
                            sa = sa.wrapping_add(1);
                        }
                        (sa, SdlsStatus::UnknownSa)
                    } else {
                        (0, SdlsStatus::UnknownPort)
                    };
                    data_in(&r, sa, pool_buffer(index));
                    assert_eq!(*h.data_out.lock().unwrap(), vec![(status, index)]);
                    shadow.insert(index, ROUTER_ERROR_PORT);
                }
                3 => {
                    // DataFlow.ProcessedData
                    let Some(index) = free(&shadow) else { continue };
                    let port = pick(0, SA_ROUTER_PORT_COUNT as u64 - 1) as FwIndexType;
                    let status = if pick(0, 1) == 0 {
                        SdlsStatus::Success
                    } else {
                        SdlsStatus::DecryptionFailure
                    };
                    sa_data_in(&r, port, status, pool_buffer(index));
                    assert_eq!(*h.data_out.lock().unwrap(), vec![(status, index)]);
                    shadow.insert(index, port);
                }
                4 => {
                    // DataFlow.ProcessedDataReturn
                    let Some((&index, &port)) = shadow.iter().next() else {
                        continue;
                    };
                    data_return_in(&r, pool_buffer(index));
                    if port == ROUTER_ERROR_PORT {
                        assert!(h.sa_data_return_out.lock().unwrap().is_empty());
                        assert_eq!(*h.buffer_return_out.lock().unwrap(), vec![index]);
                    } else {
                        assert_eq!(*h.sa_data_return_out.lock().unwrap(), vec![(port, index)]);
                    }
                    shadow.remove(&index);
                }
                _ => {
                    // DataFlow.BufferReturn
                    let port = pick(0, SA_ROUTER_PORT_COUNT as u64 - 1) as FwIndexType;
                    let p = r.sa_buffer_return_in(port);
                    p.target
                        .invoke(p.port_num, pool_buffer(200), &FrameContext::default());
                    assert_eq!(*h.buffer_return_out.lock().unwrap(), vec![200]);
                }
            }
            assert!(h.events.lock().unwrap().is_empty());
            assert_eq!(r.outstanding_count(), shadow.len());
            steps += 1;
        }
    }
}
