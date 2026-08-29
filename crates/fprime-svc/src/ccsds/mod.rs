//! CCSDS communications stack (`Svc/Ccsds`): Space Packet framing/deframing,
//! TM/TC transfer frames, and APID sequence management.
//!
//! Implemented by the CCSDS wave; see `docs/cpp-analysis/ccsds.md`.

pub mod apid_manager;
pub mod crc16;
pub mod space_packet_deframer;
pub mod space_packet_framer;
pub mod tc_deframer;
pub mod tm_framer;
pub mod types;
