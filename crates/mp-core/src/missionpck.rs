//! missionpck: AP_Filesystem_Mission's `@MISSION/mission.dat`, `fence.dat` and `rally.dat`.
//! A 10 byte header (magic 0x763d, mission type, options, start, count) followed by packed
//! MISSION_ITEM_INT structs (38 bytes, MAVLink field order).

use crate::link::Link;
use crate::mavlink::dialects::ardupilotmega::*;
use crate::mission::Locationwp;
use num_traits::FromPrimitive;

const MAGIC: u16 = 0x763d;
const ITEM_LEN: usize = 38;

pub fn path(mission_type: u8) -> &'static str {
    match mission_type {
        1 => "@MISSION/fence.dat",
        2 => "@MISSION/rally.dat",
        _ => "@MISSION/mission.dat",
    }
}

fn mtype(t: u8) -> MavMissionType {
    match t {
        1 => MavMissionType::MAV_MISSION_TYPE_FENCE,
        2 => MavMissionType::MAV_MISSION_TYPE_RALLY,
        _ => MavMissionType::MAV_MISSION_TYPE_MISSION,
    }
}

fn encode(m: &MISSION_ITEM_INT_DATA, out: &mut Vec<u8>) {
    for f in [m.param1, m.param2, m.param3, m.param4] {
        out.extend(f.to_le_bytes());
    }
    out.extend(m.x.to_le_bytes());
    out.extend(m.y.to_le_bytes());
    out.extend(m.z.to_le_bytes());
    out.extend(m.seq.to_le_bytes());
    out.extend((m.command as u16).to_le_bytes());
    out.extend([m.target_system, m.target_component, m.frame as u8, m.current, m.autocontinue, m.mission_type as u8]);
}

fn decode(b: &[u8]) -> Option<MISSION_ITEM_INT_DATA> {
    let f = |i: usize| f32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    let i32_at = |i: usize| i32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
    Some(MISSION_ITEM_INT_DATA {
        param1: f(0),
        param2: f(4),
        param3: f(8),
        param4: f(12),
        x: i32_at(16),
        y: i32_at(20),
        z: f(24),
        seq: u16_at(28),
        command: MavCmd::from_u16(u16_at(30))?,
        target_system: b[32],
        target_component: b[33],
        frame: MavFrame::from_u8(b[34]).unwrap_or(MavFrame::MAV_FRAME_GLOBAL),
        current: b[35],
        autocontinue: b[36],
        mission_type: MavMissionType::from_u8(b[37]).unwrap_or(MavMissionType::MAV_MISSION_TYPE_MISSION),
    })
}

/// missionpck.pack
pub fn pack(items: &[Locationwp], mission_type: u8, start: u16) -> Vec<u8> {
    let mut out = Vec::with_capacity(10 + items.len() * ITEM_LEN);
    for v in [MAGIC, mission_type as u16, 0, start, items.len() as u16] {
        out.extend(v.to_le_bytes());
    }
    for (i, w) in items.iter().enumerate() {
        if let Some(m) = w.to_mission_item_int(i as u16, 0, 0, mtype(mission_type)) {
            encode(&m, &mut out);
        }
    }
    out
}

/// missionpck.unpack: (items, mission type, start)
pub fn unpack(data: &[u8]) -> Result<(Vec<Locationwp>, u8, u16), String> {
    if data.len() < 10 {
        return Err("short file".into());
    }
    let u = |i: usize| u16::from_le_bytes([data[i], data[i + 1]]);
    if u(0) != MAGIC {
        return Err(format!("Invalid Magic {}", u(0)));
    }
    let n = u(8) as usize;
    let body = &data[10..];
    if body.len() / ITEM_LEN != n {
        return Err(format!("Bad Item count {n} vs {}", body.len() / ITEM_LEN));
    }
    let items = body.chunks_exact(ITEM_LEN).filter_map(decode).map(|m| Locationwp::from_mission_item_int(&m)).collect();
    Ok((items, u(2) as u8, u(6)))
}

/// Download a mission list over MAVFtp.
pub fn ftp_read(link: &Link, mission_type: u8) -> Result<Vec<Locationwp>, String> {
    read_with(&mut crate::ftp::Ftp::new(link), mission_type)
}

/// Upload a mission list over MAVFtp.
pub fn ftp_write(link: &Link, mission_type: u8, items: &[Locationwp]) -> Result<(), String> {
    write_with(&mut crate::ftp::Ftp::new(link), mission_type, items)
}

pub fn read_with(ftp: &mut crate::ftp::Ftp, mission_type: u8) -> Result<Vec<Locationwp>, String> {
    let data = ftp.get(path(mission_type), |_, _| {})?;
    unpack(&data).map(|r| r.0)
}

pub fn write_with(ftp: &mut crate::ftp::Ftp, mission_type: u8, items: &[Locationwp]) -> Result<(), String> {
    ftp.put(path(mission_type), &pack(items, mission_type, 0), |_, _| {})
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let items = vec![Locationwp::waypoint(-35.36, 149.16, 0.0, 0), Locationwp::waypoint(-35.361, 149.161, 50.0, 3), Locationwp { id: 20, ..Default::default() }];
        let b = pack(&items, 0, 0);
        assert_eq!(b.len(), 10 + 3 * 38);
        let (back, t, s) = unpack(&b).unwrap();
        assert_eq!((t, s, back.len()), (0, 0, 3));
        assert!((back[1].lat - -35.361).abs() < 1e-6 && back[1].alt == 50.0 && back[1].frame == 3);
        assert_eq!(back[2].id, 20);
        assert!(unpack(&[0u8; 12]).is_err());
    }
}
