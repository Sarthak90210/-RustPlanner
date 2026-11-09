//! Port of `ExtLibs/ArduPilot/missionpck.cs`
//!
//! The `mission.dat` / `fence.dat` / `rally.dat` files ArduPilot serves over MAVFTP
//! (https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Filesystem/AP_Filesystem_Mission.cpp).

use crate::dotnet::{bit_converter, Exception};
use crate::extlibs::mavlink::{
    get_message_info, mavlink_mission_item_int_t, CsEnum, MavlinkUtil, MAVLINK_MESSAGE_INFOS, MAVLINK_MSG_ID, MAV_MISSION_TYPE,
};

const MISSION_MAGIC: u16 = 0x763d;

/// `internal struct header`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct Header {
    pub magic: u16,
    /// MAV_MISSION_TYPE_*
    pub data_type: u16,
    /// optional features
    pub options: u16,
    /// first WP num, 0 for full upload
    pub start: u16,
    pub num_items: u16,
}

impl Header {
    /// `StructureToByteArray(hdr)`
    fn to_bytes(self) -> Vec<u8> {
        let mut v = Vec::with_capacity(10);
        for x in [self.magic, self.data_type, self.options, self.start, self.num_items] {
            v.extend_from_slice(&x.to_le_bytes());
        }
        v
    }
}

/// `public static class missionpck`
#[allow(non_camel_case_types)]
pub struct missionpck;

impl missionpck {
    /// `unpack(byte[] data)`
    pub fn unpack(data: &[u8]) -> Result<(Vec<mavlink_mission_item_int_t>, MAV_MISSION_TYPE, u16), Exception> {
        let mut data_pointer = 0i32;

        let mut header = Header::default();
        header.magic = bit_converter::to_u16(data, data_pointer)?;
        data_pointer += 2;
        header.data_type = bit_converter::to_u16(data, data_pointer)?;
        data_pointer += 2;
        header.options = bit_converter::to_u16(data, data_pointer)?;
        data_pointer += 2;
        header.start = bit_converter::to_u16(data, data_pointer)?;
        data_pointer += 2;
        header.num_items = bit_converter::to_u16(data, data_pointer)?;
        data_pointer += 2;

        if header.magic != MISSION_MAGIC {
            return Err(Exception::Other(format!("Invalid Magic {}", header.magic)));
        }

        let msginfo = get_message_info(MAVLINK_MESSAGE_INFOS.get(), MAVLINK_MSG_ID::MISSION_ITEM_INT.0 as u32);

        let msgs = &data[data_pointer as usize..];

        let item_size = msginfo.length as u8 as usize;
        let msgcount = (msgs.len() / item_size) as u16;

        if msgcount != header.num_items {
            return Err(Exception::Other(format!("Bad Item count {} vs {}", header.num_items, msgcount)));
        }

        let mut list = Vec::with_capacity(header.num_items as usize);
        for a in 0..msgcount as usize {
            let msg = &msgs[a * item_size..a * item_size + item_size];
            let missionitem: mavlink_mission_item_int_t = MavlinkUtil::byte_array_to_structure(msg, 0);
            list.push(missionitem);
        }

        Ok((list, MAV_MISSION_TYPE::from_i64(header.data_type as i64), header.start))
    }

    /// `pack(List<mavlink_mission_item_int_t> wps, MAV_MISSION_TYPE type, ushort start = 0)`
    pub fn pack(wps: &[mavlink_mission_item_int_t], r#type: MAV_MISSION_TYPE, start: u16) -> Vec<u8> {
        let hdr = Header { data_type: r#type.value_i64() as u16, magic: MISSION_MAGIC, num_items: wps.len() as u16, start, options: 0 };
        let mut ans = hdr.to_bytes();
        for wp in wps {
            ans.extend(MavlinkUtil::structure_to_byte_array_t(wp));
        }
        ans
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_unpack_round_trip() {
        let wp = mavlink_mission_item_int_t { seq: 3, x: 123456789, y: -987654321, z: 50.0, command: 16, ..Default::default() };
        let b = missionpck::pack(&[wp, wp], MAV_MISSION_TYPE::FENCE, 0);
        assert_eq!(&b[..2], &[0x3d, 0x76]);
        assert_eq!(b.len(), 10 + 2 * mavlink_mission_item_int_t::WIRE_LENGTH);
        let (items, t, start) = missionpck::unpack(&b).unwrap();
        assert_eq!(items, vec![wp, wp]);
        assert_eq!(t, MAV_MISSION_TYPE::FENCE);
        assert_eq!(start, 0);
        let mut bad = b.clone();
        bad[0] = 0;
        assert!(matches!(missionpck::unpack(&bad), Err(Exception::Other(m)) if m.starts_with("Invalid Magic")));
        assert!(matches!(missionpck::unpack(&b[..b.len() - 1]), Err(Exception::Other(m)) if m == "Bad Item count 2 vs 1"));
        assert!(matches!(missionpck::unpack(&b[..5]), Err(Exception::Argument(_))));
    }
}
