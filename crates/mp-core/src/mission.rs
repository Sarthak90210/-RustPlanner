//! The app's mission rows, as a layer over the file-by-file ports: reading goes through
//! `WaypointFile.ReadWaypointFile` and `.plan` files through `MissionFile`
//! (`extlibs::utilities::missionfile`), with rows converted to and from the ported
//! `Locationwp`. The "QGC WPL 110" writer is `FlightPlanner.savewaypoints`'s.

use std::fmt::Write as _;

use crate::extlibs::utilities::locationwp::Locationwp as PortLocationwp;
use crate::extlibs::utilities::missionfile::{MissionFile, WaypointFile};
use crate::mavcmd;

/// Mirrors MissionPlanner.Utilities.Locationwp (the struct as used in ArduPilot).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Locationwp {
    pub id: u16,
    pub p1: f32,
    pub p2: f32,
    pub p3: f32,
    pub p4: f32,
    pub lat: f64,
    pub lng: f64,
    pub alt: f32,
    pub frame: u8,
    pub seq: u16,
}

impl Default for Locationwp {
    fn default() -> Self {
        Self {
            id: 0,
            p1: 0.0,
            p2: 0.0,
            p3: 0.0,
            p4: 0.0,
            lat: 0.0,
            lng: 0.0,
            alt: 0.0,
            frame: 0,
            seq: 0,
        }
    }
}

/// MAV_FRAME values that Mission Planner exposes through the "altmode" combo.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AltMode {
    Absolute = 0,
    Relative = 3,
    Terrain = 10,
}

impl AltMode {
    pub const ALL: [AltMode; 3] = [AltMode::Relative, AltMode::Absolute, AltMode::Terrain];

    pub fn from_frame(frame: u8) -> Self {
        match frame {
            0 | 5 => AltMode::Absolute,
            10 | 11 => AltMode::Terrain,
            _ => AltMode::Relative,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            AltMode::Absolute => "Absolute",
            AltMode::Relative => "Relative",
            AltMode::Terrain => "Terrain",
        }
    }
}

/// FlightPlanner's `enum altmode { Relative = GLOBAL_RELATIVE_ALT, Absolute = GLOBAL,
/// Terrain = GLOBAL_TERRAIN_ALT }` as EnumTranslator reflects over it (no attributes).
impl crate::extlibs::utilities::enumtranslator::EnumReflect for AltMode {
    fn fields() -> Vec<crate::extlibs::utilities::enumtranslator::EnumFieldInfo> {
        AltMode::ALL
            .iter()
            .map(|m| crate::extlibs::utilities::enumtranslator::EnumFieldInfo {
                name: m.label().to_string(),
                value: *m as i64,
                display_text: None,
                private: None,
            })
            .collect()
    }

    fn to_cs_string(value: i64) -> String {
        match AltMode::ALL.iter().find(|m| **m as i64 == value) {
            Some(m) => m.label().to_string(),
            None => value.to_string(),
        }
    }
}

impl Locationwp {
    pub fn waypoint(lat: f64, lng: f64, alt: f32, frame: u8) -> Self {
        Self {
            id: mavcmd::WAYPOINT,
            lat,
            lng,
            alt,
            frame,
            ..Default::default()
        }
    }

    /// True if the command carries a position (see `Locationwp.isLocationCommand`).
    pub fn has_location(&self) -> bool {
        mavcmd::is_location_command(self.id)
    }

    pub fn to_mission_item_int(
        &self,
        seq: u16,
        target_system: u8,
        target_component: u8,
        mission_type: mavlink::dialects::ardupilotmega::MavMissionType,
    ) -> Option<mavlink::dialects::ardupilotmega::MISSION_ITEM_INT_DATA> {
        use mavlink::dialects::ardupilotmega::*;
        use num_traits::FromPrimitive;
        let command = MavCmd::from_u16(self.id)?;
        let frame =
            MavFrame::from_u8(self.frame).unwrap_or(MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT);
        let (x, y) = if self.has_location() {
            (
                (self.lat * 1e7).round() as i32,
                (self.lng * 1e7).round() as i32,
            )
        } else {
            (self.lat as i32, self.lng as i32)
        };
        Some(MISSION_ITEM_INT_DATA {
            param1: self.p1,
            param2: self.p2,
            param3: self.p3,
            param4: self.p4,
            x,
            y,
            z: self.alt,
            seq,
            command,
            target_system,
            target_component,
            frame,
            current: 0,
            autocontinue: 1,
            mission_type,
        })
    }

    pub fn from_mission_item_int(
        m: &mavlink::dialects::ardupilotmega::MISSION_ITEM_INT_DATA,
    ) -> Self {
        let id = m.command as u16;
        let (lat, lng) = if mavcmd::is_location_command(id) {
            (m.x as f64 * 1e-7, m.y as f64 * 1e-7)
        } else {
            (m.x as f64, m.y as f64)
        };
        Self {
            id,
            p1: m.param1,
            p2: m.param2,
            p3: m.param3,
            p4: m.param4,
            lat,
            lng,
            alt: m.z,
            frame: m.frame as u8,
            seq: m.seq,
        }
    }

    pub fn from_mission_item(m: &mavlink::dialects::ardupilotmega::MISSION_ITEM_DATA) -> Self {
        Self {
            id: m.command as u16,
            p1: m.param1,
            p2: m.param2,
            p3: m.param3,
            p4: m.param4,
            lat: m.x as f64,
            lng: m.y as f64,
            alt: m.z,
            frame: m.frame as u8,
            seq: m.seq,
        }
    }
}

/// The app's mission row to the ported `MissionPlanner.Utilities.Locationwp`.
pub fn to_port(w: &Locationwp) -> PortLocationwp {
    PortLocationwp {
        seq: w.seq,
        frame: w.frame,
        tag: None,
        id: w.id,
        p1: w.p1,
        p2: w.p2,
        p3: w.p3,
        p4: w.p4,
        lat: w.lat,
        lng: w.lng,
        alt: w.alt,
    }
}

/// The ported `Locationwp` back to the app's row (the `Tag` is dropped).
pub fn from_port(w: &PortLocationwp) -> Locationwp {
    Locationwp { id: w.id, p1: w.p1, p2: w.p2, p3: w.p3, p4: w.p4, lat: w.lat, lng: w.lng, alt: w.alt, frame: w.frame, seq: w.seq }
}

#[derive(Debug, thiserror::Error)]
pub enum WaypointFileError {
    /// The first line has no "QGC WPL" header (`ReadWaypointFile` shows this and returns an
    /// empty list; callers that fall back to other formats need to tell it apart).
    #[error("Invalid Waypoint file")]
    InvalidHeader,
}

/// `WaypointFile.ReadWaypointFile` over text already read: index 0 is home, and a file whose
/// first item is not index 0 gets a blank home inserted. A bad line is skipped after `show`
/// gets "Line invalid\n<line>", where the C# shows a `CustomMessageBox`.
pub fn read_waypoint_file(text: &str, show: &mut dyn FnMut(&str)) -> Result<Vec<Locationwp>, WaypointFileError> {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut bad_header = false;
    let list = WaypointFile::read_waypoint_lines(crate::dotnet::io::split_lines(text), &mut |m| {
        if m == "Invalid Waypoint file" {
            bad_header = true;
        } else {
            show(m);
        }
        Ok(())
    })
    .unwrap_or_default();
    if bad_header {
        return Err(WaypointFileError::InvalidHeader);
    }
    Ok(list.iter().map(from_port).collect())
}

/// The "QGC WPL 110" branch of `FlightPlanner.savewaypoints`: `StreamWriter.WriteLine` line
/// ends, each grid cell's text (`float.ToString()` / `double.ToString()`) re-read with
/// `double.Parse` and written with `ToString("0.00000000", en-US)` (alt "0.000000"). Home
/// with empty TXT_home boxes (0,0 here) fails `double.Parse` and writes the fallback line.
pub fn write_waypoint_file(home: &Locationwp, items: &[Locationwp]) -> String {
    use crate::dotnet::{format_fixed, format_general, INVARIANT_CULTURE};
    let nl = crate::dotnet::io::NEW_LINE;
    let en = &INVARIANT_CULTURE;
    // the grid cell's text and back: float cells hold G7, double cells G15
    let cell_f = |v: f32| format_general(v as f64, 7, en).parse::<f64>().unwrap_or(v as f64);
    let cell_d = |v: f64| format_general(v, 15, en).parse::<f64>().unwrap_or(v);
    let f8 = |v: f64| format_fixed(v, 1, 8, en);

    let mut s = format!("QGC WPL 110{nl}");
    if home.lat == 0.0 && home.lng == 0.0 {
        let _ = write!(s, "0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1{nl}");
    } else {
        let _ = write!(
            s,
            "0\t1\t0\t16\t0\t0\t0\t0\t{}\t{}\t{}\t1{nl}",
            format_fixed(cell_d(home.lat), 1, 7, en),
            format_fixed(cell_d(home.lng), 1, 7, en),
            format_fixed(cell_f(home.alt), 1, 6, en)
        );
    }
    for (a, wp) in items.iter().enumerate() {
        let _ = write!(
            s,
            "{}\t0\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t1{nl}",
            a + 1,
            wp.frame,
            wp.id,
            f8(cell_f(wp.p1)),
            f8(cell_f(wp.p2)),
            f8(cell_f(wp.p3)),
            f8(cell_f(wp.p4)),
            f8(cell_d(wp.lat)),
            f8(cell_d(wp.lng)),
            format_fixed(cell_f(wp.alt), 1, 6, en)
        );
    }
    s
}

/// `MissionFile.WriteFile(file, MissionFile.ConvertFromLocationwps(list, frame))` as
/// `FlightPlanner.savewaypoints` calls it for a .plan file, returning the text: home first,
/// its lat/lng/alt as plannedHomePosition, every other item with the chosen frame.
pub fn write_mission_json(home: &Locationwp, items: &[Locationwp], frame: u8) -> String {
    // FlightPlanner.GetCommandList builds new Locationwps from the grid rows: _seq is never set,
    // so every doJumpId is written as 0
    let list: Vec<PortLocationwp> = std::iter::once(home).chain(items).map(|w| PortLocationwp { seq: 0, ..to_port(w) }).collect();
    MissionFile::serialize_indented(&MissionFile::ConvertFromLocationwps(&list, Some(frame)))
}

/// `MissionFile.ReadFile` then `ConvertToLocationwps` over text already read; the exception's
/// message where the C# throws.
pub fn read_mission_json(text: &str) -> Result<Vec<Locationwp>, String> {
    let root = MissionFile::parse_text(text).map_err(|e| e.message().to_string())?;
    let Some(root) = root else { return Err("Object reference not set to an instance of an object.".into()) };
    MissionFile::ConvertToLocationwps(&root).map(|l| l.iter().map(from_port).collect()).map_err(|e| e.message().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mission_json_matches_missionfile_writefile() {
        let home = Locationwp { lat: -35.0, lng: 149.0, alt: 584.5, ..Default::default() };
        let wp = Locationwp { id: 16, lat: -35.1, lng: 149.1, alt: 50.0, frame: 0, seq: 0, ..Default::default() };
        let j = write_mission_json(&home, &[wp], 3);
        let v: serde_json::Value = serde_json::from_str(&j).unwrap();
        assert_eq!(v["groundStation"], "MissionPlanner");
        assert_eq!(v["version"], 1);
        assert!(v["fileType"].is_null() && v["geoFence"].is_null() && v["rallyPoints"].is_null());
        assert_eq!(v["mission"]["plannedHomePosition"], serde_json::json!([-35.0, 149.0, 584.5]));
        let item = &v["mission"]["items"][0];
        assert_eq!((item["command"].as_u64(), item["frame"].as_u64(), item["autoContinue"].as_bool()), (Some(16), Some(3), Some(false)));
        assert!(item["type"].is_null());
        assert!(j.starts_with(&format!("{{{}  \"fileType\": null,", crate::dotnet::io::NEW_LINE)), "{j}");
        // and back through MissionFile.ReadFile: only "SimpleItem" items survive ConvertToLocationwps
        assert_eq!(read_mission_json(&j).unwrap().len(), 1);
        assert!(read_mission_json("[]").is_err());
    }

    #[test]
    fn bad_lines_are_reported_and_skipped_like_readwaypointfile() {
        let text = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t1.0\t2.0\t30\t1\n1\t0\t3\tNOPE\t0\t0\t0\t0\t1\t2\t3\t1\n2\t0\t3\t16\t0\t0\t0\t0\t3.0\t4.0\t30\t1\n";
        let mut shown = vec![];
        let parsed = read_waypoint_file(text, &mut |m| shown.push(m.to_string())).unwrap();
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[1].lat, 3.0);
        assert_eq!(shown, vec!["Line invalid\n1\t0\t3\tNOPE\t0\t0\t0\t0\t1\t2\t3\t1".to_string()]);
        let mut shown = vec![];
        assert!(matches!(read_waypoint_file("hello", &mut |m| shown.push(m.to_string())), Err(WaypointFileError::InvalidHeader)));
        assert!(shown.is_empty());
    }

    #[test]
    fn wpl_roundtrip() {
        let home = Locationwp {
            lat: -35.363261,
            lng: 149.16523,
            alt: 584.0,
            ..Default::default()
        };
        let items = vec![
            Locationwp {
                id: 22,
                alt: 20.0,
                frame: 3,
                ..Default::default()
            },
            Locationwp::waypoint(-35.36, 149.166, 50.0, 3),
            Locationwp {
                id: 20,
                frame: 3,
                ..Default::default()
            },
        ];
        let text = write_waypoint_file(&home, &items);
        let parsed = read_waypoint_file(&text, &mut |m| panic!("{m}")).unwrap();
        assert_eq!(parsed.len(), 4);
        assert!((parsed[0].lat - home.lat).abs() < 1e-7);
        assert_eq!(parsed[1].id, 22);
        assert_eq!(parsed[2].alt, 50.0);
        assert_eq!(parsed[3].id, 20);
    }

    #[test]
    fn missing_home_inserted() {
        let text = "QGC WPL 110\n1\t0\t3\t16\t0\t0\t0\t0\t1.0\t2.0\t30\t1\n";
        let parsed = read_waypoint_file(text, &mut |m| panic!("{m}")).unwrap();
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[1].lat, 1.0);
    }
}
