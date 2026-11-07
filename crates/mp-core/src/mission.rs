//! Mission model: port of `Locationwp`, `WaypointFile` and the "QGC WPL 110" writer from
//! `FlightPlanner.cs`.

use std::fmt::Write as _;

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

#[derive(Debug, thiserror::Error)]
pub enum WaypointFileError {
    #[error("Invalid Waypoint file")]
    InvalidHeader,
    #[error("Line invalid\n{0}")]
    InvalidLine(String),
}

/// Port of `WaypointFile.ReadWaypointFile`. Index 0 is always home; if the file does not start
/// with seq 0, a blank home is inserted, exactly like Mission Planner.
pub fn read_waypoint_file(text: &str) -> Result<Vec<Locationwp>, WaypointFileError> {
    let mut lines = text.lines();
    let header = lines.next().unwrap_or("");
    if !header.contains("QGC WPL") {
        return Err(WaypointFileError::InvalidHeader);
    }
    let mut cmds = Vec::new();
    let mut wp_count = 0;
    for line in lines {
        if line.starts_with('#') {
            continue;
        }
        let items: Vec<&str> = line
            .split(|c| c == '\t' || c == ' ' || c == ',')
            .filter(|s| !s.is_empty())
            .collect();
        if items.len() <= 9 {
            continue;
        }
        let parse = || -> Option<Locationwp> {
            let mut id: u16 = items[3].parse().ok()?;
            if id == 99 {
                id = 0;
            }
            Some(Locationwp {
                frame: items[2].parse::<i32>().ok()? as u8,
                id,
                p1: items[4].parse().ok()?,
                p2: items[5].parse::<f64>().ok()? as f32,
                p3: items[6].parse::<f64>().ok()? as f32,
                p4: items[7].parse::<f64>().ok()? as f32,
                lat: items[8].parse().ok()?,
                lng: items[9].parse().ok()?,
                alt: items[10].parse::<f64>().ok()? as f32,
                seq: 0,
            })
        };
        if wp_count == 0 && items[0] != "0" {
            cmds.push(Locationwp::default());
        }
        let wp = parse().ok_or_else(|| WaypointFileError::InvalidLine(line.to_string()))?;
        cmds.push(wp);
        wp_count += 1;
    }
    for (i, c) in cmds.iter_mut().enumerate() {
        c.seq = i as u16;
    }
    Ok(cmds)
}

/// Port of the "QGC WPL 110" writer in `FlightPlanner.savewaypoints`. `items[0]` is home.
pub fn write_waypoint_file(home: &Locationwp, items: &[Locationwp]) -> String {
    let mut s = String::from("QGC WPL 110\n");
    let _ = writeln!(
        s,
        "0\t1\t0\t16\t0\t0\t0\t0\t{:.7}\t{:.7}\t{:.6}\t1",
        home.lat, home.lng, home.alt
    );
    for (a, wp) in items.iter().enumerate() {
        let _ = writeln!(
            s,
            "{}\t0\t{}\t{}\t{:.8}\t{:.8}\t{:.8}\t{:.8}\t{:.8}\t{:.8}\t{:.6}\t1",
            a + 1,
            wp.frame,
            wp.id,
            wp.p1,
            wp.p2,
            wp.p3,
            wp.p4,
            wp.lat,
            wp.lng,
            wp.alt
        );
    }
    s
}

/// MissionFile.Item as Json.NET writes it (every property, nulls included).
#[derive(serde::Serialize)]
#[allow(non_snake_case)]
struct MissionFileItem {
    autoContinue: bool,
    command: u16,
    doJumpId: u16,
    frame: u8,
    params: [f64; 7],
    r#type: Option<String>,
    TransectStyleComplexItem: Option<()>,
    angle: Option<i32>,
    complexItemType: Option<String>,
    entryLocation: Option<i32>,
    flyAlternateTransects: Option<bool>,
    polygon: Option<()>,
    splitConcavePolygons: Option<bool>,
    version: Option<i32>,
}

#[derive(serde::Serialize)]
#[allow(non_snake_case)]
struct MissionFileMission {
    cruiseSpeed: i32,
    firmwareType: i32,
    hoverSpeed: i32,
    items: Vec<MissionFileItem>,
    plannedHomePosition: Option<Vec<f64>>,
    vehicleType: i32,
    version: i32,
}

#[derive(serde::Serialize)]
#[allow(non_snake_case)]
struct MissionFileRoot {
    fileType: Option<String>,
    geoFence: Option<()>,
    groundStation: String,
    mission: MissionFileMission,
    rallyPoints: Option<()>,
    version: i32,
}

/// MissionFile.ConvertFromLocationwps(list with home first, frame) written with
/// Formatting.Indented: the home's lat/lng/alt as plannedHomePosition, every other item with
/// the chosen frame.
pub fn write_mission_json(home: &Locationwp, items: &[Locationwp], frame: u8) -> String {
    let item = |w: &Locationwp, frame: u8| MissionFileItem {
        autoContinue: false,
        command: w.id,
        doJumpId: w.seq,
        frame,
        params: [w.p1 as f64, w.p2 as f64, w.p3 as f64, w.p4 as f64, w.lat, w.lng, w.alt as f64],
        r#type: None,
        TransectStyleComplexItem: None,
        angle: None,
        complexItemType: None,
        entryLocation: None,
        flyAlternateTransects: None,
        polygon: None,
        splitConcavePolygons: None,
        version: None,
    };
    let root = MissionFileRoot {
        fileType: None,
        geoFence: None,
        groundStation: "MissionPlanner".into(),
        mission: MissionFileMission {
            cruiseSpeed: 0,
            firmwareType: 0,
            hoverSpeed: 0,
            items: items.iter().map(|w| item(w, frame)).collect(),
            plannedHomePosition: Some(vec![home.lat, home.lng, home.alt as f64]),
            vehicleType: 0,
            version: 0,
        },
        rallyPoints: None,
        version: 1,
    };
    serde_json::to_string_pretty(&root).unwrap_or_default()
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
        assert!(j.starts_with("{\n  \"fileType\": null,"));
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
        let parsed = read_waypoint_file(&text).unwrap();
        assert_eq!(parsed.len(), 4);
        assert!((parsed[0].lat - home.lat).abs() < 1e-7);
        assert_eq!(parsed[1].id, 22);
        assert_eq!(parsed[2].alt, 50.0);
        assert_eq!(parsed[3].id, 20);
    }

    #[test]
    fn missing_home_inserted() {
        let text = "QGC WPL 110\n1\t0\t3\t16\t0\t0\t0\t0\t1.0\t2.0\t30\t1\n";
        let parsed = read_waypoint_file(text).unwrap();
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[1].lat, 1.0);
    }
}
