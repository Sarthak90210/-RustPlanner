//! FlightPlanner.saveWPsFast: the "Write Fast" upload. MISSION_COUNT (setWPTotal), then the
//! items streamed without waiting for each request; every ten items it waits up to 1.1 s for the
//! vehicle's MISSION_REQUEST to catch up, resumes from the requested item, and on
//! MAV_MISSION_ERROR asks for a partial re-upload. It finishes with a MISSION_ACK (setWPACK).

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::Receiver;
use mavlink::dialects::ardupilotmega::*;
use num_traits::FromPrimitive;

use std::sync::Arc;

use crossbeam_channel::Sender;
use parking_lot::RwLock;

use crate::link::{LinkCommand, LinkShared, Progress, GCS_COMPID, GCS_SYSID};
use crate::mission::Locationwp;

const MAV_CMD_LAST: u16 = 95;
const DO_SET_HOME: u16 = 179;
const DO_DIGICAM_CONFIGURE: u16 = 202;
const DO_DIGICAM_CONTROL: u16 = 203;

/// What the vehicle has said so far (the three SubscribeToPacketType handlers).
struct Replies {
    ack: Receiver<MavMessage>,
    req: Receiver<MavMessage>,
    req_int: Receiver<MavMessage>,
    result: MavMissionResult,
    reqno: u16,
    /// a MISSION_REQUEST(_INT) for seq 0 or 1 arrived (setWPTotal's answer)
    started: bool,
}

impl Replies {
    fn drain(&mut self) {
        // "check this gcs sent it"
        let ours = |s: u8, c: u8| s == GCS_SYSID && c == GCS_COMPID;
        while let Ok(m) = self.ack.try_recv() {
            if let MavMessage::MISSION_ACK(a) = m {
                if ours(a.target_system, a.target_component) {
                    self.result = a.mavtype;
                    self.started = true;
                }
            }
        }
        for rx in [&self.req, &self.req_int] {
            while let Ok(m) = rx.try_recv() {
                let (seq, ts, tc) = match m {
                    MavMessage::MISSION_REQUEST(r) => (r.seq, r.target_system, r.target_component),
                    MavMessage::MISSION_REQUEST_INT(r) => (r.seq, r.target_system, r.target_component),
                    _ => continue,
                };
                if ours(ts, tc) {
                    self.reqno = seq;
                    if seq <= 1 {
                        self.started = true;
                    }
                }
            }
        }
    }
}

fn result_name(r: MavMissionResult) -> String {
    format!("{r:?}")
}

/// The MISSION_ITEM_INT saveWPsFast builds for `loc` at `seq` with the running `frame`.
pub fn item(loc: &Locationwp, seq: u16, frame: &mut u8, altmode: u8, sys: u8, comp: u8) -> Option<MISSION_ITEM_INT_DATA> {
    // the frame only follows the alt mode for navigation commands and DO_SET_HOME
    if loc.id < MAV_CMD_LAST || loc.id == DO_SET_HOME {
        *frame = match altmode {
            10 => 10,
            0 => 0,
            _ => 3,
        };
    }
    let (x, y) = if loc.id == DO_DIGICAM_CONTROL || loc.id == DO_DIGICAM_CONFIGURE {
        (loc.lat as i32, loc.lng as i32)
    } else {
        ((loc.lat * 1.0e7) as i32, (loc.lng * 1.0e7) as i32)
    };
    Some(MISSION_ITEM_INT_DATA {
        param1: loc.p1,
        param2: loc.p2,
        param3: loc.p3,
        param4: loc.p4,
        x,
        y,
        z: loc.alt,
        seq,
        // an id the dialect lacks goes out raw (see raw_item)
        command: MavCmd::from_u16(loc.id).unwrap_or(MavCmd::MAV_CMD_NAV_WAYPOINT),
        target_system: sys,
        target_component: comp,
        frame: MavFrame::from_u8(*frame).unwrap_or(MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT),
        current: 0,
        autocontinue: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    })
}

/// MAVLink's X.25 checksum (crc_accumulate).
fn crc_x25(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for b in bytes {
        let mut tmp = *b ^ (crc & 0xFF) as u8;
        tmp ^= tmp << 4;
        crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    }
    crc
}

/// A MISSION_ITEM_INT frame carrying `command` as written, for ids the dialect has no name for
/// (MAVLink.mavlink_mission_item_int_t.command is a plain ushort).
pub fn raw_item(item: &MISSION_ITEM_INT_DATA, command: u16) -> Vec<u8> {
    let mut buf = Vec::new();
    let hdr = mavlink::MavHeader { system_id: crate::link::GCS_SYSID as u32, component_id: crate::link::GCS_COMPID, sequence: 0 };
    let _ = mavlink::write_versioned_msg(&mut buf, mavlink::MavlinkVersion::V2, hdr, &MavMessage::MISSION_ITEM_INT(item.clone()));
    // payload at 10; command is the u16 after param1-4, x, y, z and seq
    let len = buf[1] as usize;
    if len >= 32 {
        buf[10 + 30..10 + 32].copy_from_slice(&command.to_le_bytes());
        let mut crc_in = buf[1..10 + len].to_vec();
        crc_in.push(38); // MISSION_ITEM_INT CRC_EXTRA
        let crc = crc_x25(&crc_in);
        buf[10 + len..10 + len + 2].copy_from_slice(&crc.to_le_bytes());
    }
    buf
}

fn count(total: u16, sys: u8, comp: u8) -> LinkCommand {
    LinkCommand::Send(Box::new(MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
        count: total,
        target_system: sys,
        target_component: comp,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        opaque_id: 0,
    })))
}

/// Uploads `commandlist` (home first). `rows` is the grid's row count for the progress
/// percentage; `altmode` is CMB_altmode's frame. Ok(false) means the user cancelled.
pub fn write_fast(tx: Sender<LinkCommand>, shared: Arc<RwLock<LinkShared>>, commandlist: &[Locationwp], rows: usize, altmode: u8, cancel: &AtomicBool) -> Result<bool, String> {
    struct L {
        tx: Sender<LinkCommand>,
        shared: Arc<RwLock<LinkShared>>,
    }
    impl L {
        fn send(&self, c: LinkCommand) {
            let _ = self.tx.send(c);
        }
    }
    let link = &L { tx, shared };
    let (sys, comp) = {
        let s = link.shared.read();
        (s.cs.sysid, s.cs.compid)
    };
    let sub = |msgid: u32| {
        let (tx, rx) = crossbeam_channel::unbounded();
        link.send(LinkCommand::SubscribeFrom { msgid, sysid: sys, compid: comp, tx });
        rx
    };
    let mut r = Replies { ack: sub(47), req: sub(40), req_int: sub(51), result: MavMissionResult::MAV_MISSION_ACCEPTED, reqno: 0, started: false };
    let progress = |pct: usize, label: String| link.shared.write().progress = Some(Progress { label, done: pct, total: 100 });
    let total = commandlist.len() as u16;

    progress(0, "Set total wps ".into());
    // setWPTotal: up to three resends 700 ms apart until the vehicle asks for item 0 or 1
    let mut retries = 3;
    link.send(count(total, sys, comp));
    let mut start = Instant::now();
    loop {
        r.drain();
        if r.started {
            break;
        }
        if start.elapsed() > Duration::from_millis(700) {
            if retries == 0 {
                return Err("Timeout on read - setWPTotal".into());
            }
            retries -= 1;
            link.send(count(total, sys, comp));
            start = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    let mut frame = 3u8;
    let mut a = 0usize;
    while a < commandlist.len() {
        if a.is_multiple_of(10) && a != 0 {
            let start = Instant::now();
            loop {
                // the progress dialog's Cancel clears the shared progress
                if cancel.load(Ordering::Relaxed) || link.shared.read().progress.is_none() {
                    link.send(count(0, sys, comp));
                    return Ok(false);
                }
                r.drain();
                if r.reqno as usize == a {
                    break;
                }
                if start.elapsed() > Duration::from_millis(1100) {
                    // do the next ten from the requested item
                    a = r.reqno as usize;
                    break;
                }
                if r.result == MavMissionResult::MAV_MISSION_INVALID_SEQUENCE {
                    std::thread::sleep(Duration::from_millis(500));
                }
                if r.result == MavMissionResult::MAV_MISSION_ERROR {
                    // resend for a partial upload
                    link.send(LinkCommand::Send(Box::new(MavMessage::MISSION_WRITE_PARTIAL_LIST(MISSION_WRITE_PARTIAL_LIST_DATA {
                        start_index: r.reqno as i16,
                        end_index: total as i16,
                        target_system: sys,
                        target_component: comp,
                        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
                    }))));
                    a = r.reqno as usize;
                    break;
                }
                if r.result == MavMissionResult::MAV_MISSION_NO_SPACE {
                    return Err("Upload failed, please reduce the number of wp's".into());
                }
                if r.result == MavMissionResult::MAV_MISSION_INVALID {
                    return Err(format!("Upload failed, mission was rejected byt the Mav,\n item had a bad option wp# {a} {}", result_name(r.result)));
                }
                if r.result != MavMissionResult::MAV_MISSION_ACCEPTED {
                    return Err(format!("Upload wps failed {} {}", r.reqno, result_name(r.result)));
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        if let Some(req) = item(&commandlist[a], a as u16, &mut frame, altmode, sys, comp) {
            if rows > 0 && link.shared.read().progress.is_some() {
                progress(a * 100 / rows, format!("Setting WP {a}"));
            }
            if MavCmd::from_u16(commandlist[a].id).is_some() {
                link.send(LinkCommand::Send(Box::new(MavMessage::MISSION_ITEM_INT(req))));
            } else {
                link.send(LinkCommand::SendRaw(raw_item(&req, commandlist[a].id)));
            }
        }
        a += 1;
    }
    // setWPACK, then getHomePositionAsync
    link.send(LinkCommand::Send(Box::new(MavMessage::MISSION_ACK(MISSION_ACK_DATA {
        target_system: sys,
        target_component: comp,
        mavtype: MavMissionResult::MAV_MISSION_ACCEPTED,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        opaque_id: 0,
    }))));
    link.send(LinkCommand::CommandNoAck { command: 512, params: [242.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
    link.shared.write().progress = None;
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_command_ids_go_out_raw_with_a_valid_checksum() {
        let mut frame = 3;
        let wp = Locationwp { id: 65001, p1: 1.0, lat: -35.0, lng: 149.0, alt: 10.0, ..Default::default() };
        assert!(MavCmd::from_u16(65001).is_none(), "an id the dialect lacks");
        let item = item(&wp, 4, &mut frame, 3, 1, 1).unwrap();
        let bytes = raw_item(&item, 65001);
        assert_eq!(u16::from_le_bytes([bytes[40], bytes[41]]), 65001);
        // re-encoding a known command reproduces the crate's own frame, checksum included
        let mut own = Vec::new();
        let hdr = mavlink::MavHeader { system_id: 255, component_id: 190, sequence: 0 };
        mavlink::write_versioned_msg(&mut own, mavlink::MavlinkVersion::V2, hdr, &MavMessage::MISSION_ITEM_INT(item.clone())).unwrap();
        assert_eq!(raw_item(&item, 16), own);
        assert_ne!(bytes, own);
    }

    #[test]
    fn frames_follow_the_alt_mode_for_nav_commands_only() {
        let mut frame = 3;
        let wp = Locationwp { id: 16, lat: -35.1234567, lng: 149.7654321, alt: 50.0, ..Default::default() };
        let i = item(&wp, 1, &mut frame, 10, 1, 1).unwrap();
        assert_eq!((i.frame, i.x, i.y), (MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT, -351234567, 1497654321));
        // a DO command keeps the frame of the item before it
        let servo = Locationwp { id: 183, p1: 9.0, ..Default::default() };
        assert_eq!(item(&servo, 2, &mut frame, 0, 1, 1).unwrap().frame, MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT);
        // DO_SET_HOME follows the mode; digicam lat/lng are sent unscaled
        assert_eq!(item(&Locationwp { id: 179, ..Default::default() }, 3, &mut frame, 0, 1, 1).unwrap().frame, MavFrame::MAV_FRAME_GLOBAL);
        let cam = Locationwp { id: 203, lat: 1.0, lng: 2.0, ..Default::default() };
        let c = item(&cam, 4, &mut frame, 3, 1, 1).unwrap();
        assert_eq!((c.x, c.y, c.autocontinue, c.current), (1, 2, 1, 0));
    }
}
