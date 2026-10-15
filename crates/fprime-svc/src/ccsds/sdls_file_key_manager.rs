//! # SdlsFileKeyManager — port of `Svc::Ccsds::SdlsFileKeyManager` (passive)
//!
//! C++ sources: `Svc/Ccsds/SdlsFileKeyManager/SdlsFileKeyManager.{cpp,hpp,fpp}`
//! and its `docs/sdd.md`.
//!
//! Supplies SDLS keys read from a file (`Svc.Ccsds.SdlsKeyInterface`).
//! [`configure`](SdlsFileKeyManager::configure) sets the path and the key
//! length (`(0, MAX_SDLS_KEY_SIZE]`). Each guarded `keyGet` opens the file,
//! reads exactly the key length into the caller's [`SdlsKeyBuffer`] and
//! returns `SUCCESS`; an open error, read error or short read zeroizes the
//! buffer, resets its length to 0, raises `KeyReadFailed(status, bytesRead,
//! expectedSize)` and returns `KEY_ERROR`. A request before configuration
//! asserts.
//!
//! This implementation ignores the requested security association index
//! and serves the same key for every SA (as `SdlsKeyInterface` requires such
//! implementations to state): SAs served this way share one key and so
//! share its IV budget.

use crate::ccsds::types::{SdlsKeyBuffer, SdlsKeyPort, SdlsStatus};
use fprime_comp::{EventGlue, PassiveBase, PortRef};
use fprime_config::sdls::MAX_SDLS_KEY_SIZE;
use fprime_config::{FwEventIdType, FwIndexType, FwSizeType};
use fprime_fw::{FileNameString, LogSeverity, SerBuf, SerBufAny, fw_assert, fw_try};
use fprime_os::file::{File, Mode, Status as FileStatus, WaitType};
use std::sync::{Arc, Mutex};

/// `KeyReadFailed(status: I32, bytesRead: FwSizeType, expectedSize:
/// FwSizeType)` — WARNING_HI.
pub const EVENTID_KEY_READ_FAILED: FwEventIdType = 0;

struct KeyState {
    path: FileNameString,
    key_size: FwSizeType,
    configured: bool,
}

/// `Svc::Ccsds::SdlsFileKeyManager` — SDLS keys from a file.
pub struct SdlsFileKeyManager {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// Configuration; also the `keyGet` guard.
    state: Mutex<KeyState>,
}

impl SdlsFileKeyManager {
    /// Construct the component (unconfigured).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            state: Mutex::new(KeyState {
                path: FileNameString::new(),
                key_size: 0,
                configured: false,
            }),
        })
    }

    /// C++ `configure(path, keySize)`: asserts `keySize` in
    /// `(0, MAX_SDLS_KEY_SIZE]` and that the path is not truncated.
    pub fn configure(&self, path: &str, key_size: FwSizeType) {
        fw_assert!(key_size > 0, key_size as i32);
        fw_assert!(key_size as usize <= MAX_SDLS_KEY_SIZE, key_size as i32);
        let mut st = self.state.lock().unwrap();
        // Reject paths that would be silently truncated.
        fw_assert!(
            path.len() <= FileNameString::max_length(),
            path.len() as i32
        );
        st.path = FileNameString::from(path);
        st.key_size = key_size;
        st.configured = true;
    }

    /// `keyGet` — GUARDED `Svc.Ccsds.SdlsKey`.
    pub fn key_get(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SdlsKeyPort> {
        PortRef::new(self.clone(), port_num)
    }

    fn key_get_handler(
        &self,
        _port_num: FwIndexType,
        _sa: u16,
        key: &mut SdlsKeyBuffer,
    ) -> SdlsStatus {
        let st = self.state.lock().unwrap();
        fw_assert!(st.configured);
        // One key file: the SA does not select among keys.
        let mut file = File::new();
        let open_status = file.open(st.path.as_str().unwrap_or(""), Mode::OpenRead);
        if open_status != FileStatus::OpOk {
            self.log_key_read_failed(open_status as i32, 0, st.key_size);
            Self::zeroize(key);
            return SdlsStatus::KeyError;
        }
        let mut read_size: FwSizeType = st.key_size;
        let read_status = {
            let dest = &mut key.bytes_mut()[..st.key_size as usize];
            file.read(dest, &mut read_size, WaitType::Wait)
        };
        file.close();
        if read_status != FileStatus::OpOk || read_size != st.key_size {
            self.log_key_read_failed(read_status as i32, read_size, st.key_size);
            // Zeroize partially read bytes so they cannot leak.
            Self::zeroize(key);
            return SdlsStatus::KeyError;
        }
        let status = key.set_buff_len(st.key_size as usize);
        fw_assert!(status.is_ok(), status as i32);
        SdlsStatus::Success
    }

    /// Zeroize any stale key bytes and reset the length.
    fn zeroize(key: &mut SdlsKeyBuffer) {
        key.bytes_mut().fill(0);
        let status = key.set_buff_len(0);
        fw_assert!(status.is_ok(), status as i32);
    }

    fn log_key_read_failed(&self, status: i32, bytes_read: FwSizeType, expected: FwSizeType) {
        self.evt.log_event(
            self.base.get_id_base(),
            EVENTID_KEY_READ_FAILED,
            LogSeverity::WarningHi,
            &format!(
                "Failed to read key file: status {status}, read {bytes_read} of {expected} bytes"
            ),
            |buf| {
                fw_try!(buf.serialize_i32_be(status));
                fw_try!(buf.serialize_u64_be(bytes_read));
                buf.serialize_u64_be(expected)
            },
        );
    }
}

impl SdlsKeyPort for SdlsFileKeyManager {
    fn invoke(
        &self,
        port_num: FwIndexType,
        security_association_index: u16,
        key: &mut SdlsKeyBuffer,
    ) -> SdlsStatus {
        self.key_get_handler(port_num, security_association_index, key)
    }
}

// ---------------------------------------------------------------------------
// Tests — port of SdlsFileKeyManagerTester
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::LogPort;
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    const TEST_SA_INDEX: u16 = 1;

    #[derive(Default)]
    struct Harness {
        events: StdMutex<Vec<(FwEventIdType, Vec<u8>)>>,
    }

    impl LogPort for Harness {
        fn invoke(
            &self,
            _p: FwIndexType,
            id: FwEventIdType,
            _t: &mut Time,
            _s: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, args.as_slice().to_vec()));
        }
    }

    fn build() -> (Arc<SdlsFileKeyManager>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        let c = SdlsFileKeyManager::new("SdlsFileKeyManager");
        c.evt.log_out.connect(h.clone(), 0);
        (c, h)
    }

    fn temp_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("fprime_sdls_key_{}_{name}", std::process::id()))
    }

    fn key_get(c: &Arc<SdlsFileKeyManager>, key: &mut SdlsKeyBuffer) -> SdlsStatus {
        let p = c.key_get(0);
        p.target.invoke(p.port_num, TEST_SA_INDEX, key)
    }

    fn event_args(status: i32, read: u64, expected: u64) -> Vec<u8> {
        let mut v = status.to_be_bytes().to_vec();
        v.extend_from_slice(&read.to_be_bytes());
        v.extend_from_slice(&expected.to_be_bytes());
        v
    }

    #[test]
    fn nominal_read() {
        for key_size in [1usize, 16, 32] {
            let (c, h) = build();
            let path = temp_path(&format!("nominal{key_size}"));
            let data: Vec<u8> = (0..key_size as u8)
                .map(|b| b.wrapping_mul(29) ^ 0x6B)
                .collect();
            std::fs::write(&path, &data).unwrap();
            c.configure(path.to_str().unwrap(), key_size as FwSizeType);
            let mut key = SdlsKeyBuffer::new();
            assert_eq!(key_get(&c, &mut key), SdlsStatus::Success);
            assert_eq!(key.get_size(), key_size);
            assert_eq!(key.as_slice(), &data[..]);
            assert!(h.events.lock().unwrap().is_empty());
            std::fs::remove_file(&path).unwrap();
        }
    }

    /// A file longer than the key: exactly the key length is read.
    #[test]
    fn longer_file_reads_the_key_length() {
        let (c, _h) = build();
        let path = temp_path("long");
        std::fs::write(&path, [0xAAu8; 40]).unwrap();
        c.configure(path.to_str().unwrap(), 32);
        let mut key = SdlsKeyBuffer::new();
        assert_eq!(key_get(&c, &mut key), SdlsStatus::Success);
        assert_eq!(key.as_slice(), &[0xAA; 32]);
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn missing_file() {
        let (c, h) = build();
        let key_size = 24;
        c.configure(temp_path("missing").to_str().unwrap(), key_size);
        // A reused buffer holding a previously read key.
        let mut key = SdlsKeyBuffer::new();
        key.bytes_mut().fill(0xAB);
        assert!(key.set_buff_len(key_size as usize).is_ok());
        assert_eq!(key_get(&c, &mut key), SdlsStatus::KeyError);
        assert_eq!(key.get_size(), 0);
        assert!(key.bytes().iter().all(|&b| b == 0));
        assert_eq!(
            *h.events.lock().unwrap(),
            vec![(
                EVENTID_KEY_READ_FAILED,
                event_args(FileStatus::DoesntExist as i32, 0, key_size)
            )]
        );
    }

    #[test]
    fn short_file() {
        let (c, h) = build();
        let path = temp_path("short");
        std::fs::write(&path, [0x11u8; 5]).unwrap();
        c.configure(path.to_str().unwrap(), 16);
        let mut key = SdlsKeyBuffer::new();
        assert_eq!(key_get(&c, &mut key), SdlsStatus::KeyError);
        assert_eq!(key.get_size(), 0);
        assert!(key.bytes().iter().all(|&b| b == 0));
        assert_eq!(
            *h.events.lock().unwrap(),
            vec![(EVENTID_KEY_READ_FAILED, event_args(0, 5, 16))]
        );
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn unconfigured_request_asserts() {
        let (c, _h) = build();
        let mut key = SdlsKeyBuffer::new();
        let _ = key_get(&c, &mut key);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn oversized_key_length_asserts() {
        let (c, _h) = build();
        c.configure("key.bin", MAX_SDLS_KEY_SIZE as FwSizeType + 1);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn zero_key_length_asserts() {
        let (c, _h) = build();
        c.configure("key.bin", 0);
    }
}
