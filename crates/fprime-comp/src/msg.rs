//! The async queue-message envelope — byte-exact with the C++ generated
//! code (`docs/cpp-analysis/fw-comp.md` "Async port/command/internal-
//! interface queue message", `fpp-autocoder.md` "Component queue message").
//!
//! ```text
//! [msg_type: FwEnumStoreType = i32 BE]   // 0 reserved for EXIT
//! [port_num: FwIndexType    = i16 BE]    // absent for the EXIT message
//! [args serialized in declaration order, big-endian]
//! ```
//!
//! The EXIT message is exactly 4 bytes `[i32 0]` — no `port_num` follows
//! (C++ `ActiveComponentBase::exit()` sizes its buffer `sizeof(int)`), so
//! dispatch must check the exit sentinel BEFORE reading `port_num`.

use fprime_config::{FwEnumStoreType, FwIndexType};
use fprime_fw::{SerBuf, SerBufAny, SerializeStatus, fw_try};

/// The exit sentinel (`Fw::ActiveComponentBase::ACTIVE_COMPONENT_EXIT`).
/// Component message-type discriminants start at 1.
pub const EXIT_MSG_TYPE: FwEnumStoreType = 0;

/// Size of the EXIT message on the queue: just the i32 sentinel
/// (C++ parity: `sizeof(ACTIVE_COMPONENT_EXIT)` == `sizeof(int)` == 4).
pub const EXIT_MSG_SIZE: usize = size_of::<FwEnumStoreType>();

/// The EXIT message bytes.
pub const EXIT_MSG_BYTES: [u8; EXIT_MSG_SIZE] = [0, 0, 0, 0];

/// Envelope header size for non-EXIT messages: i32 msg_type + i16 port_num.
pub const ENVELOPE_HEADER_SIZE: usize = size_of::<FwEnumStoreType>() + size_of::<FwIndexType>();

/// Serialize the `[msg_type][port_num]` envelope header (big-endian). The
/// caller then serializes the port arguments in declaration order.
pub fn write_envelope_header(
    buf: &mut dyn SerBufAny,
    msg_type: FwEnumStoreType,
    port_num: FwIndexType,
) -> SerializeStatus {
    fw_try!(buf.serialize_i32_be(msg_type));
    buf.serialize_i16_be(port_num)
}

/// Serialize the 4-byte EXIT message.
pub fn write_exit(buf: &mut dyn SerBufAny) -> SerializeStatus {
    buf.serialize_i32_be(EXIT_MSG_TYPE)
}

/// Deserialize the leading `msg_type` (done by the dispatch loop before the
/// exit check).
pub fn read_msg_type(buf: &mut dyn SerBufAny, msg_type: &mut FwEnumStoreType) -> SerializeStatus {
    buf.deserialize_i32_be(msg_type)
}

/// Deserialize the `port_num` that follows `msg_type` on non-EXIT messages
/// (done by the component's `dispatch_message`).
pub fn read_port_num(buf: &mut dyn SerBufAny, port_num: &mut FwIndexType) -> SerializeStatus {
    buf.deserialize_i16_be(port_num)
}

/// Per-port queue-full policy (FPP `assert` / `drop` / `block` / `hook`
/// qualifiers on async inputs; `docs/cpp-analysis/fpp-autocoder.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum QueueFullPolicy {
    /// Default: `fw_assert` on any non-OK send status (queue overflow is a
    /// crash, not a drop — C++ parity).
    #[default]
    Assert,
    /// On `Full`, increment the component's `msgs_dropped` counter and
    /// discard the message silently.
    Drop,
    /// Blocking send — waits for queue space.
    Block,
    /// On `Full`, return `Full` to the adapter so it can invoke its
    /// overflow hook with the original (pre-serialization) arguments.
    Hook,
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::LinearBuffer;

    #[test]
    fn envelope_header_is_i32_be_then_i16_be() {
        let mut buf = LinearBuffer::<16>::new();
        let status = write_envelope_header(&mut buf, 1, 2);
        assert!(status.is_ok());
        assert_eq!(buf.as_slice(), &[0x00, 0x00, 0x00, 0x01, 0x00, 0x02]);
        assert_eq!(buf.as_slice().len(), ENVELOPE_HEADER_SIZE);
    }

    #[test]
    fn envelope_header_encodes_negative_port_num_as_i16() {
        // FwIndexType is SIGNED i16 (-1 = unset) — sign must survive.
        let mut buf = LinearBuffer::<16>::new();
        let status = write_envelope_header(&mut buf, 0x0102_0304, -1);
        assert!(status.is_ok());
        assert_eq!(buf.as_slice(), &[0x01, 0x02, 0x03, 0x04, 0xFF, 0xFF]);
    }

    #[test]
    fn exit_message_is_exactly_four_zero_bytes() {
        let mut buf = LinearBuffer::<16>::new();
        let status = write_exit(&mut buf);
        assert!(status.is_ok());
        assert_eq!(buf.as_slice(), &EXIT_MSG_BYTES);
        assert_eq!(buf.as_slice().len(), 4);
    }

    #[test]
    fn header_round_trips() {
        let mut buf = LinearBuffer::<16>::new();
        let status = write_envelope_header(&mut buf, 7, -1);
        assert!(status.is_ok());
        let mut msg_type = 0;
        let mut port_num = 0;
        assert!(read_msg_type(&mut buf, &mut msg_type).is_ok());
        assert!(read_port_num(&mut buf, &mut port_num).is_ok());
        assert_eq!(msg_type, 7);
        assert_eq!(port_num, -1);
    }

    #[test]
    fn write_header_reports_no_room() {
        let mut buf = LinearBuffer::<3>::new();
        let status = write_envelope_header(&mut buf, 1, 0);
        assert_eq!(status, SerializeStatus::NoRoomLeft);
    }
}
