//! Port of `ExtLibs/Mavlink/MAVLinkMessage.cs`

use std::sync::{LazyLock, OnceLock};

use super::mavlink::*;
use super::mavlinkheader::{get_header_length, get_target_system_field, read_system_id, MAVLINK_SUPPORTED_IFLAGS};
use super::mavlinkutil::{get_message_info, MavlinkUtil};
use crate::dotnet::{DateTime, Exception};

/// `MAVLink.MAVLinkMessage`. C# shares instances by reference; Rust callers hold them in an
/// `Arc` where the C# code keeps a message beyond the call that produced it.
pub struct MAVLinkMessage {
    _buffer: Option<Vec<u8>>,
    pub rxtime: DateTime,
    header: u8,
    payloadlength: u8,
    incompat_flags: u8,
    compat_flags: u8,
    seq: u8,
    sysid: u32,
    compid: u8,
    headerlength: i32,
    target_system: Option<u32>,
    msgid: u32,
    crc16: u16,
    sig: Option<Vec<u8>>,
    /// `_data` behind `_locker`: built once, on first access.
    _data: OnceLock<Option<Box<dyn MavlinkData>>>,
}

static INVALID: LazyLock<MAVLinkMessage> = LazyLock::new(MAVLinkMessage::new);

impl std::fmt::Debug for MAVLinkMessage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.to_cs_string())
    }
}

impl Default for MAVLinkMessage {
    fn default() -> Self {
        Self::new()
    }
}

impl MAVLinkMessage {
    /// `public static readonly MAVLinkMessage Invalid`
    pub fn invalid() -> &'static MAVLinkMessage {
        &INVALID
    }

    /// `public MAVLinkMessage()`: no buffer, `rxtime = DateTime.MinValue`.
    pub fn new() -> Self {
        MAVLinkMessage {
            _buffer: None,
            rxtime: DateTime::MIN_VALUE,
            header: 0,
            payloadlength: 0,
            incompat_flags: 0,
            compat_flags: 0,
            seq: 0,
            sysid: 0,
            compid: 0,
            headerlength: 0,
            target_system: None,
            msgid: 0,
            crc16: 0,
            sig: None,
            _data: OnceLock::new(),
        }
    }

    /// `public MAVLinkMessage(byte[] buffer)`: received now.
    pub fn from_buffer(buffer: Vec<u8>) -> Result<Self, Exception> {
        Self::from_buffer_at(buffer, DateTime::utc_now())
    }

    /// `public MAVLinkMessage(byte[] buffer, DateTime rxTime)`
    pub fn from_buffer_at(buffer: Vec<u8>, rx_time: DateTime) -> Result<Self, Exception> {
        let mut m = Self::new();
        m.set_buffer(Some(buffer))?;
        m.rxtime = rx_time;
        Ok(m)
    }

    pub fn buffer(&self) -> Option<&[u8]> {
        self._buffer.as_deref()
    }

    /// The `buffer` setter: stores the bytes, then parses them (and throws on a bad frame).
    pub fn set_buffer(&mut self, value: Option<Vec<u8>>) -> Result<(), Exception> {
        self._buffer = value;
        let buf = self._buffer.take();
        let r = self.process_buffer(buf.as_deref());
        self._buffer = buf;
        r
    }

    pub fn header(&self) -> u8 {
        self.header
    }
    pub fn payloadlength(&self) -> u8 {
        self.payloadlength
    }
    pub fn incompat_flags(&self) -> u8 {
        self.incompat_flags
    }
    pub fn compat_flags(&self) -> u8 {
        self.compat_flags
    }
    pub fn seq(&self) -> u8 {
        self.seq
    }
    pub fn sysid(&self) -> u32 {
        self.sysid
    }
    pub fn compid(&self) -> u8 {
        self.compid
    }
    pub fn headerlength(&self) -> i32 {
        self.headerlength
    }
    pub fn target_system(&self) -> Option<u32> {
        self.target_system
    }
    pub fn msgid(&self) -> u32 {
        self.msgid
    }
    pub fn crc16(&self) -> u16 {
        self.crc16
    }
    pub fn sig(&self) -> Option<&[u8]> {
        self.sig.as_deref()
    }

    /// `GetTargetSystem()`: a wide header target takes precedence over the legacy payload byte.
    pub fn get_target_system(&self) -> Option<u32> {
        if self.target_system.is_some() {
            return self.target_system;
        }
        let r#type = get_message_info(MAVLINK_MESSAGE_INFOS.get(), self.msgid).r#type?;
        let field = get_target_system_field(r#type)?;
        self.data()?.get_field(field.name)?.to_u32()
    }

    /// `GetTargetComponent()`
    pub fn get_target_component(&self) -> Option<u8> {
        let r#type = get_message_info(MAVLINK_MESSAGE_INFOS.get(), self.msgid).r#type?;
        r#type.get_field("target_component")?;
        match self.data()?.get_field("target_component")? {
            FieldValue::Byte(b) => Some(b),
            v => v.to_f64().map(|x| x as u8),
        }
    }

    /// `IsTargetedTo(system, component)`
    pub fn is_targeted_to(&self, system: u32, component: u8) -> bool {
        let target_system = self.get_target_system();
        let target_component = self.get_target_component();
        (target_system.is_none() || target_system == Some(0) || target_system == Some(system))
            && (target_component.is_none() || target_component == Some(0) || target_component == Some(component))
    }

    pub fn ismavlink2(&self) -> bool {
        match &self._buffer {
            Some(b) if !b.is_empty() => b[0] == MAVLINK_STX,
            _ => false,
        }
    }

    /// `msgtypename`: the message name, empty (C# null) for an unknown id.
    pub fn msgtypename(&self) -> &'static str {
        get_message_info(MAVLINK_MESSAGE_INFOS.get(), self.msgid).name
    }

    /// `data`: the payload decoded into its generated struct, built once. `None` for an
    /// unknown message id.
    pub fn data(&self) -> Option<&dyn MavlinkData> {
        self._data
            .get_or_init(|| {
                let typeinfo = get_message_info(MAVLINK_MESSAGE_INFOS.get(), self.msgid);
                let t = typeinfo.r#type?;
                if self.payloadlength == 0 {
                    return Some((t.deserialize)(&[]));
                }
                let buffer = self._buffer.as_deref()?;
                let start = if self.ismavlink2() { self.headerlength as u8 } else { 6 };
                Some(MavlinkUtil::byte_array_to_structure_gc_dyn(buffer, t, start, self.payloadlength))
            })
            .as_deref()
    }

    /// `ToStructure<T>()`: C# casts, so a type mismatch throws InvalidCastException; here it is
    /// a panic with the same meaning. Use `data()?.downcast_ref` to test the type instead.
    pub fn to_structure<T: Clone + 'static>(&self) -> T {
        self.data()
            .and_then(|d| d.downcast_ref::<T>())
            .cloned()
            .unwrap_or_else(|| panic!("InvalidCastException: {} is not {}", self.msgtypename(), std::any::type_name::<T>()))
    }

    pub fn sig_linkid(&self) -> u8 {
        self.sig.as_ref().map(|s| s[0]).unwrap_or(0)
    }

    pub fn sig_timestamp(&self) -> u64 {
        match &self.sig {
            Some(sig) => {
                let mut temp = [0u8; 8];
                temp[..6].copy_from_slice(&sig[1..7]);
                u64::from_le_bytes(temp)
            }
            None => 0,
        }
    }

    /// `Length`: the frame length, 0 without a buffer.
    pub fn length(&self) -> i32 {
        self._buffer.as_ref().map(|b| b.len() as i32).unwrap_or(0)
    }

    /// `internal void processBuffer(byte[] buffer)`
    pub(crate) fn process_buffer(&mut self, buffer: Option<&[u8]>) -> Result<(), Exception> {
        self._data = OnceLock::new();
        self.sig = None;
        self.target_system = None;
        self.incompat_flags = 0;
        self.compat_flags = 0;
        let buffer = match buffer {
            Some(b) if b.len() >= 2 => b,
            _ => return Err(Exception::InvalidData("Truncated MAVLink frame".into())),
        };
        self.header = buffer[0];
        self.payloadlength = buffer[1];
        if self.header == MAVLINK_STX {
            if buffer.len() < MAVLINK_NUM_HEADER_BYTES as usize {
                return Err(Exception::InvalidData("Truncated MAVLink header".into()));
            }
            self.incompat_flags = buffer[2];
            self.compat_flags = buffer[3];
            if (self.incompat_flags & !MAVLINK_SUPPORTED_IFLAGS) != 0 {
                return Err(Exception::InvalidData("Unsupported MAVLink incompatible flags".into()));
            }
            self.headerlength = get_header_length(self.incompat_flags);
        } else if self.header == MAVLINK_STX_MAVLINK1 {
            self.headerlength = 6;
        } else {
            return Err(Exception::InvalidData("Invalid MAVLink magic".into()));
        }

        let signature_length =
            if (self.incompat_flags & MAVLINK_IFLAG_SIGNED) != 0 { MAVLINK_SIGNATURE_BLOCK_LEN as i32 } else { 0 };
        if buffer.len() as i32 != self.headerlength + self.payloadlength as i32 + 2 + signature_length {
            return Err(Exception::InvalidData("Invalid MAVLink frame length".into()));
        }
        if self.header == MAVLINK_STX {
            self.seq = buffer[4];
            let wide = (self.incompat_flags & MAVLINK_IFLAG_SYSID32) != 0;
            let mut offset = 5usize;
            self.sysid = read_system_id(buffer, &mut offset, wide);
            self.compid = buffer[offset];
            offset += 1;
            self.msgid = buffer[offset] as u32 | (buffer[offset + 1] as u32) << 8 | (buffer[offset + 2] as u32) << 16;
            offset += 3;
            if (self.incompat_flags & MAVLINK_IFLAG_TARGET32) != 0 {
                self.target_system = Some(read_system_id(buffer, &mut offset, true));
            }
        } else {
            self.seq = buffer[2];
            self.sysid = buffer[3] as u32;
            self.compid = buffer[4];
            self.msgid = buffer[5] as u32;
        }
        let crc_offset = (self.headerlength + self.payloadlength as i32) as usize;
        self.crc16 = buffer[crc_offset] as u16 | (buffer[crc_offset + 1] as u16) << 8;
        if signature_length != 0 {
            self.sig = Some(buffer[crc_offset + 2..crc_offset + 2 + signature_length as usize].to_vec());
        }
        Ok(())
    }

    /// `ToString()`: `String.Format("{5},{4},{0},{1},{2},{3}", sysid, compid, msgid, msgtypename, ismavlink2, rxtime)`
    pub fn to_cs_string(&self) -> String {
        format!(
            "{},{},{},{},{},{}",
            self.rxtime,
            if self.ismavlink2() { "True" } else { "False" },
            self.sysid,
            self.compid,
            self.msgid,
            self.msgtypename()
        )
    }
}

impl std::fmt::Display for MAVLinkMessage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.to_cs_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extlibs::mavlink::mavlinkparse::MavlinkParse;

    #[test]
    fn parses_wide_header_frames() {
        let mut p = MavlinkParse::new(false);
        let cmd = mavlink_command_long_t { target_system: 1, target_component: 1, command: 400, param1: 1.0, ..Default::default() };
        let pkt = p.generate_mavlink_packet20(MAVLINK_MSG_ID::COMMAND_LONG, &cmd, false, 70000, 190, -1, Some(300), 1).unwrap();
        let m = MAVLinkMessage::from_buffer(pkt).unwrap();
        assert_eq!(m.sysid(), 70000);
        assert_eq!(m.target_system(), Some(300));
        assert_eq!(m.get_target_system(), Some(300));
        assert_eq!(m.get_target_component(), Some(1));
        assert!(m.is_targeted_to(300, 1));
        assert!(!m.is_targeted_to(255, 1));
        let c: mavlink_command_long_t = m.to_structure();
        assert_eq!(c.command, 400);
        assert_eq!(c.target_system, 255, "payload target is clamped to 255 for wide targets");
        assert_eq!(m.msgtypename(), "COMMAND_LONG");
    }

    #[test]
    fn rejects_bad_frames() {
        assert!(MAVLinkMessage::from_buffer(vec![0xFD]).is_err());
        assert!(MAVLinkMessage::from_buffer(vec![0x55, 0, 0, 0, 0, 0, 0, 0]).is_err());
        assert!(MAVLinkMessage::from_buffer(vec![0xFD, 0, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0]).is_err());
        assert_eq!(MAVLinkMessage::invalid().length(), 0);
        assert_eq!(MAVLinkMessage::invalid().msgtypename(), "HEARTBEAT", "msgid 0 is HEARTBEAT, as in C#");
    }
}
