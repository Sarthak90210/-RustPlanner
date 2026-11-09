//! Port of `ExtLibs/Mavlink/MavlinkParse.cs`
//!
//! The `Units`, `Description` and `hasLocation` attribute classes are declared here. The
//! generator attaches them through `FieldInfo` and `EnumEntry`; `FieldInfo::attributes` and
//! `EnumEntry::attributes` give them back as these types, like `GetCustomAttributes`.

use sha2::{Digest, Sha256};

use super::mavlink::*;
use super::mavlinkcrc::MavlinkCRC;
use super::mavlinkheader::{get_header_length, set_payload_target, write_header, MAVLINK_SUPPORTED_IFLAGS};
use super::mavlinkmessage::MAVLinkMessage;
use super::mavlinkutil::{get_message_info, MavlinkUtil};
use crate::dotnet::{DateTime, DateTimeKind, Exception, Stream};

/// `MAVLink.Units` attribute
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Units {
    pub unit: String,
}

impl Units {
    pub fn new(unit: &str) -> Self {
        Units { unit: unit.to_string() }
    }
}

/// `MAVLink.Description` attribute
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Description {
    pub text: String,
}

impl Description {
    pub fn new(desc: &str) -> Self {
        Description { text: desc.to_string() }
    }
}

/// `MAVLink.hasLocation` attribute
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct hasLocation;

/// One custom attribute, as `GetCustomAttributes(false)` returns them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Attribute {
    Units(Units),
    Description(Description),
    HasLocation(hasLocation),
    Obsolete,
}

impl FieldInfo {
    /// `field.GetCustomAttributes(false)`: `[Units]` then `[Description]`, as generated.
    pub fn attributes(&self) -> Vec<Attribute> {
        vec![Attribute::Units(Units::new(self.units)), Attribute::Description(Description::new(self.description))]
    }
}

impl EnumEntry {
    /// The member's custom attributes in the generator's order.
    pub fn attributes(&self) -> Vec<Attribute> {
        let mut v = vec![];
        if self.has_location {
            v.push(Attribute::HasLocation(hasLocation));
        }
        if self.obsolete {
            v.push(Attribute::Obsolete);
        }
        v.push(Attribute::Description(Description::new(self.description)));
        v
    }
}

/// `GetUnit(fieldname, packetype = null, name = "", msgid = UInt32.MaxValue)`. Every failure
/// path in C# ends in the swallowed exception and `""`.
pub fn get_unit(fieldname: &str, packetype: Option<&MessageType>, name: &str, msgid: u32) -> String {
    let mut msginfo: Option<&message_info> = None;
    if let Some(pt) = packetype {
        match MAVLINK_MESSAGE_INFOS.get().iter().find(|a| a.r#type.is_some_and(|t| t == pt)) {
            Some(m) => msginfo = Some(m),
            None => return String::new(),
        }
    }
    if msgid != u32::MAX {
        match MAVLINK_MESSAGE_INFOS.get().iter().find(|a| a.msgid == msgid) {
            Some(m) => msginfo = Some(m),
            None => return String::new(),
        }
    }
    if !name.is_empty() {
        match MAVLINK_MESSAGE_INFOS.get().iter().find(|a| a.name == name) {
            Some(m) => msginfo = Some(m),
            None => return String::new(),
        }
    }
    // A default message_info has a null name and type: the C# dereference throws, caught as "".
    let Some(msginfo) = msginfo else { return String::new() };
    let Some(t) = msginfo.r#type else { return String::new() };
    let Some(typeofthing) = t.get_field(fieldname) else { return String::new() };
    let attrib = typeofthing.attributes();
    if !attrib.is_empty() {
        return attrib
            .iter()
            .find_map(|a| if let Attribute::Units(u) = a { Some(u.unit.clone()) } else { None })
            .unwrap_or_default();
    }
    String::new()
}

/// `MAVLink.MavlinkParse`
#[derive(Debug, Default)]
pub struct MavlinkParse {
    pub packetcount: i32,
    pub bad_crc: i32,
    pub bad_length: i32,
    pub has_timestamp: bool,
    pub sendlinkid: u8,
    pub lasttimestamp: u64,
    pub signing_key: Option<Vec<u8>>,
}

impl MavlinkParse {
    pub fn new(has_timestamp: bool) -> Self {
        MavlinkParse { has_timestamp, ..Default::default() }
    }

    /// `ReadWithTimeout(Stream, buffer, offset, count)`
    pub fn read_with_timeout(base_stream: &mut dyn Stream, buffer: &mut [u8], offset: usize, count: usize) -> Result<(), Exception> {
        let mut timeout = 500i32;
        if base_stream.can_seek() {
            timeout = 0;
            if base_stream.position()? + count as i64 > base_stream.length()? {
                return Err(Exception::EndOfStream("End of data".into()));
            }
        }
        if base_stream.can_timeout() {
            timeout = base_stream.read_timeout();
            if timeout == -1 {
                timeout = 60000;
            }
        }
        let mut to = DateTime::utc_now().add_milliseconds(timeout as f64)?;
        let mut toread = count;
        let mut pos = offset;
        loop {
            let read = base_stream.read(&mut buffer[pos..pos + toread])?;
            toread -= read;
            pos += read;
            if read > 0 {
                to = DateTime::utc_now().add_milliseconds(timeout as f64)?;
            }
            if toread == 0 {
                break;
            }
            if read > 0 {
                continue;
            }
            if base_stream.can_seek() {
                return Err(Exception::EndOfStream("End of data".into()));
            }
            if DateTime::utc_now() > to {
                return Err(Exception::Timeout("Timeout waiting for data".into()));
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        Ok(())
    }

    /// `ReadPacket(Stream)`: `Ok(None)` is the C# `null` (no header, short read, bad flags or CRC).
    pub fn read_packet(&mut self, base_stream: &mut dyn Stream) -> Result<Option<MAVLinkMessage>, Exception> {
        let mut buffer = vec![0u8; MAVLINK_MAX_PACKET_LEN as usize];
        let mut packettime = DateTime::MIN_VALUE;

        if self.has_timestamp {
            let mut datearray = [0u8; 8];
            Self::read_with_timeout(base_stream, &mut datearray, 0, 8)?;
            datearray.reverse();
            let mut date1 = DateTime::new(1970, 1, 1, 0, 0, 0, DateTimeKind::Utc)?;
            let dateint = u64::from_le_bytes(datearray);
            if (dateint / 1000 / 1000 / 60 / 60) < 9999999 {
                date1 = date1.add_milliseconds((dateint / 1000) as f64)?;
                packettime = date1.to_local_time();
            }
        }

        let mut readcount = 0;
        while readcount <= MAVLINK_MAX_PACKET_LEN {
            Self::read_with_timeout(base_stream, &mut buffer, 0, 1)?;
            if buffer[0] == MAVLINK_STX || buffer[0] == MAVLINK_STX_MAVLINK1 {
                break;
            }
            readcount += 1;
        }
        if readcount >= MAVLINK_MAX_PACKET_LEN {
            return Ok(None);
        }

        let headerlength =
            if buffer[0] == MAVLINK_STX { MAVLINK_CORE_HEADER_LEN } else { MAVLINK_CORE_HEADER_MAVLINK1_LEN } as usize;
        let headerlengthstx = headerlength + 1;

        match Self::read_with_timeout(base_stream, &mut buffer, 1, headerlength) {
            Err(Exception::EndOfStream(_)) => return Ok(None),
            r => r?,
        }

        let lengthtoread: usize = if buffer[0] == MAVLINK_STX {
            let mut l = buffer[1] as usize + get_header_length(buffer[2]) as usize;
            if (buffer[2] & MAVLINK_IFLAG_SIGNED) > 0 {
                l += MAVLINK_SIGNATURE_BLOCK_LEN as usize;
            }
            l
        } else {
            buffer[1] as usize + headerlengthstx + 2 - 2
        };

        match Self::read_with_timeout(base_stream, &mut buffer, headerlengthstx, lengthtoread - (headerlengthstx - 2)) {
            Err(Exception::EndOfStream(_)) => return Ok(None),
            r => r?,
        }

        buffer.resize(lengthtoread + 2, 0);

        if buffer[0] == MAVLINK_STX && (buffer[2] & !MAVLINK_SUPPORTED_IFLAGS) != 0 {
            return Ok(None);
        }
        let message = MAVLinkMessage::from_buffer_at(buffer, packettime)?;

        let buf = message.buffer().unwrap_or(&[]);
        let mut crc = MavlinkCRC::crc_calculate(buf, message.headerlength() + message.payloadlength() as i32);
        if message.header() == MAVLINK_STX || message.header() == MAVLINK_STX_MAVLINK1 {
            crc = MavlinkCRC::crc_accumulate(get_message_info(MAVLINK_MESSAGE_INFOS.get(), message.msgid()).crc, crc);
        }
        if (message.crc16() >> 8) != (crc >> 8) || (message.crc16() & 0xff) != (crc & 0xff) {
            self.bad_crc += 1;
            return Ok(None);
        }
        Ok(Some(message))
    }

    /// `GenerateMAVLinkPacket10(messageType, indata, sysid = 255, compid = MAV_COMP_ID_MISSIONPLANNER, sequence = -1)`
    pub fn generate_mavlink_packet10(
        &mut self,
        message_type: MAVLINK_MSG_ID,
        indata: &dyn MavlinkData,
        sysid: u32,
        compid: u8,
        sequence: i32,
    ) -> Result<Vec<u8>, Exception> {
        if sysid > u8::MAX as u32 {
            return Err(Exception::ArgumentOutOfRange("MAVLink 1 cannot represent 32-bit system IDs\r\nParameter name: sysid".into()));
        }
        let data = MavlinkUtil::structure_to_byte_array(indata);
        let mut packet = vec![0u8; data.len() + 6 + 2];
        packet[0] = MAVLINK_STX_MAVLINK1;
        packet[1] = data.len() as u8;
        packet[2] = self.packetcount as u8;
        if sequence != -1 {
            packet[2] = sequence as u8;
        }
        self.packetcount += 1;
        packet[3] = sysid as u8;
        packet[4] = compid;
        packet[5] = message_type.0 as u8;
        let mut i = 6;
        for b in &data {
            packet[i] = *b;
            i += 1;
        }
        let mut checksum = MavlinkCRC::crc_calculate(&packet, packet[1] as i32 + 6);
        checksum = MavlinkCRC::crc_accumulate(get_message_info(MAVLINK_MESSAGE_INFOS.get(), message_type.0 as u32).crc, checksum);
        packet[i] = (checksum & 0xFF) as u8;
        i += 1;
        packet[i] = (checksum >> 8) as u8;
        Ok(packet)
    }

    /// `GenerateMAVLinkPacket20(messageType, indata, sign = false, sysid = 255,
    /// compid = MAV_COMP_ID_MISSIONPLANNER, sequence = -1, targetSystem = null, targetComponent = 0)`
    #[allow(clippy::too_many_arguments)]
    pub fn generate_mavlink_packet20(
        &mut self,
        message_type: MAVLINK_MSG_ID,
        indata: &dyn MavlinkData,
        sign: bool,
        sysid: u32,
        compid: u8,
        sequence: i32,
        target_system: Option<u32>,
        mut target_component: u8,
    ) -> Result<Vec<u8>, Exception> {
        let mut data = MavlinkUtil::structure_to_byte_array(indata);

        if indata.message_type().get_field("target_component").is_none() {
            target_component = 0;
        }
        if let Some(ts) = target_system {
            if !set_payload_target(indata, &mut data, ts, target_component) {
                return Err(Exception::Argument("Message has no target system field\r\nParameter name: targetSystem".into()));
            }
        }
        MavlinkUtil::trim_payload(&mut data);

        let extra = if sign { MAVLINK_SIGNATURE_BLOCK_LEN as usize } else { 0 };
        let mut flags = if sign { MAVLINK_IFLAG_SIGNED } else { 0 };
        if sysid > 255 {
            flags |= MAVLINK_IFLAG_SYSID32;
        }
        if target_system.is_some_and(|t| t > 255) {
            flags |= MAVLINK_IFLAG_TARGET32;
        }
        let header_length = get_header_length(flags) as usize;
        let mut packet = vec![0u8; data.len() + header_length + 2 + extra];
        let mut i = write_header(
            &mut packet,
            data.len() as u8,
            flags,
            if sequence == -1 { self.packetcount as u8 } else { sequence as u8 },
            sysid,
            compid,
            message_type.0 as u32,
            target_system.unwrap_or(0),
        )? as usize;
        self.packetcount += 1;

        for b in &data {
            packet[i] = *b;
            i += 1;
        }

        let mut checksum = MavlinkCRC::crc_calculate(&packet, (data.len() + header_length) as i32);
        checksum = MavlinkCRC::crc_accumulate(get_message_info(MAVLINK_MESSAGE_INFOS.get(), message_type.0 as u32).crc, checksum);
        packet[i] = (checksum & 0xFF) as u8;
        i += 1;
        packet[i] = (checksum >> 8) as u8;
        i += 1;

        if sign {
            // signature = sha256_48(secret_key + header + payload + CRC + link-ID + timestamp)
            let since = DateTime::utc_now() - DateTime::new(2015, 1, 1, 0, 0, 0, DateTimeKind::Unspecified)?;
            let mut timestamp = (since.total_milliseconds() * 100.0) as u64;
            if timestamp == self.lasttimestamp {
                timestamp += 1;
            }
            self.lasttimestamp = timestamp;
            let timebytes = timestamp.to_le_bytes();
            let mut sig = [0u8; 7];
            sig[0] = self.sendlinkid;
            sig[1..7].copy_from_slice(&timebytes[..6]);

            if self.signing_key.as_ref().is_none_or(|k| k.len() != 32) {
                self.signing_key = Some(vec![0u8; 32]);
            }
            let mut h = Sha256::new();
            h.update(self.signing_key.as_ref().unwrap());
            h.update(&packet[..i]);
            h.update(sig);
            let ctx = h.finalize();
            for b in sig {
                packet[i] = b;
                i += 1;
            }
            for b in &ctx[..6] {
                packet[i] = *b;
                i += 1;
            }
        }
        Ok(packet)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dotnet::MemoryStream;

    #[test]
    fn v1_and_v2_round_trip_through_read_packet() {
        let mut p = MavlinkParse::new(false);
        let hb = mavlink_heartbeat_t { r#type: 6, autopilot: 8, mavlink_version: 3, ..Default::default() };
        let mut bytes = vec![1u8, 2, 3]; // garbage before the first STX
        bytes.extend(p.generate_mavlink_packet10(MAVLINK_MSG_ID::HEARTBEAT, &hb, 255, 190, -1).unwrap());
        bytes.extend(p.generate_mavlink_packet20(MAVLINK_MSG_ID::HEARTBEAT, &hb, true, 255, 190, -1, None, 0).unwrap());
        let mut s = MemoryStream::new(bytes);
        let m1 = p.read_packet(&mut s).unwrap().unwrap();
        assert!(!m1.ismavlink2());
        assert_eq!(m1.seq(), 0);
        let m2 = p.read_packet(&mut s).unwrap().unwrap();
        assert!(m2.ismavlink2());
        assert_eq!(m2.seq(), 1);
        assert_eq!(m2.sig().unwrap().len(), 13);
        assert_eq!(m2.to_structure::<mavlink_heartbeat_t>(), hb);
        assert!(matches!(p.read_packet(&mut s), Err(Exception::EndOfStream(_))));
    }

    #[test]
    fn bad_crc_counts_and_returns_null() {
        let mut p = MavlinkParse::new(false);
        let hb = mavlink_heartbeat_t::default();
        let mut pkt = p.generate_mavlink_packet20(MAVLINK_MSG_ID::HEARTBEAT, &hb, false, 255, 190, -1, None, 0).unwrap();
        let n = pkt.len();
        pkt[n - 1] ^= 0xff;
        let mut s = MemoryStream::new(pkt);
        assert!(p.read_packet(&mut s).unwrap().is_none());
        assert_eq!(p.bad_crc, 1);
    }

    #[test]
    fn units_lookup() {
        assert_eq!(get_unit("roll", None, "ATTITUDE", u32::MAX), "[rad]");
        assert_eq!(get_unit("roll", None, "NOPE", u32::MAX), "");
        assert_eq!(get_unit("lat", None, "", 33), "[degE7]");
        assert_eq!(get_unit("lat", None, "", u32::MAX), "");
    }
}
