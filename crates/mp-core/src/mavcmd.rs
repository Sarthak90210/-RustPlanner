//! MAV_CMD naming and the `mavcmd.xml` parameter-label table used by the Flight Plan grid.

use std::collections::HashMap;
use std::sync::OnceLock;

use mavlink::dialects::ardupilotmega::MavCmd;
use num_traits::FromPrimitive;

use crate::firmware::Firmware;

pub const WAYPOINT: u16 = 16;
pub const LOITER_UNLIM: u16 = 17;
pub const LOITER_TURNS: u16 = 18;
pub const LOITER_TIME: u16 = 19;
pub const RETURN_TO_LAUNCH: u16 = 20;
pub const LAND: u16 = 21;
pub const TAKEOFF: u16 = 22;
pub const SPLINE_WAYPOINT: u16 = 82;
pub const DO_JUMP: u16 = 177;
pub const DO_SET_ROI: u16 = 201;

const MAVCMD_XML: &str = include_str!("../assets/mavcmd.xml");

/// Mission Planner's C# enum strips `MAV_CMD_` and `NAV_`, giving `WAYPOINT`, `DO_JUMP`, ...
pub fn cmd_name(id: u16) -> String {
    match MavCmd::from_u16(id) {
        Some(c) => {
            let full = format!("{c:?}");
            let s = full.strip_prefix("MAV_CMD_").unwrap_or(&full);
            s.strip_prefix("NAV_").unwrap_or(s).to_string()
        }
        None => "UNKNOWN".to_string(),
    }
}

pub fn cmd_id(name: &str) -> Option<u16> {
    static MAP: OnceLock<HashMap<String, u16>> = OnceLock::new();
    let map = MAP.get_or_init(|| {
        let mut m = HashMap::new();
        for id in 0..=u16::MAX {
            if MavCmd::from_u16(id).is_some() {
                m.insert(cmd_name(id), id);
            }
        }
        m
    });
    map.get(name).copied()
}

/// Commands whose x/y are lat/lng (`hasLocation="true"` in the MAVLink XML).
/// Unknown commands are treated as location commands, as Mission Planner does.
pub fn is_location_command(id: u16) -> bool {
    const LOC: &[u16] = &[
        16, 17, 18, 19, 21, 22, 23, 24, 25, 31, 32, 80, 81, 82, 84, 85, 94, 179, 189, 192, 195,
        201, 5000, 5001, 5002, 5003, 5004, 5100, 30001, 30002,
    ];
    if MavCmd::from_u16(id).is_none() {
        return true;
    }
    LOC.contains(&id)
}

#[derive(Debug, Clone, Default)]
pub struct CmdParamLabels {
    pub name: String,
    /// P1, P2, P3, P4, X(Lat), Y(Long), Z(Alt) column headers. Empty means unused.
    pub labels: [String; 7],
    /// The `unitType` attribute of each slot: "alt", "speed", "dist" or empty.
    pub units: [String; 7],
}

fn section_for(fw: Firmware) -> &'static str {
    match fw {
        Firmware::ArduPlane => "APM",
        Firmware::ArduRover => "APRover",
        _ => "AC2",
    }
}

fn table() -> &'static HashMap<&'static str, Vec<CmdParamLabels>> {
    static TABLE: OnceLock<HashMap<&'static str, Vec<CmdParamLabels>>> = OnceLock::new();
    TABLE.get_or_init(|| {
        let mut out = HashMap::new();
        let doc = match roxmltree::Document::parse(MAVCMD_XML) {
            Ok(d) => d,
            Err(e) => {
                log::error!("mavcmd.xml parse: {e}");
                return out;
            }
        };
        for section in doc.root_element().children().filter(|n| n.is_element()) {
            let key: &'static str = match section.tag_name().name() {
                "AC2" => "AC2",
                "APM" => "APM",
                "APRover" => "APRover",
                _ => continue,
            };
            let mut cmds = Vec::new();
            for cmd in section.children().filter(|n| n.is_element()) {
                let mut labels: [String; 7] = Default::default();
                let mut units: [String; 7] = Default::default();
                for p in cmd.children().filter(|n| n.is_element()) {
                    let idx = match p.tag_name().name() {
                        "P1" => 0,
                        "P2" => 1,
                        "P3" => 2,
                        "P4" => 3,
                        "X" => 4,
                        "Y" => 5,
                        "Z" => 6,
                        _ => continue,
                    };
                    labels[idx] = p.text().unwrap_or("").trim().to_string();
                    units[idx] = p.attribute("unitType").unwrap_or("").to_string();
                    // legacy mavcmd.xml without unitType: "Alt" in Z is an altitude
                    if units[idx].is_empty() && idx == 6 && labels[idx] == "Alt" {
                        units[idx] = "alt".into();
                    }
                }
                cmds.push(CmdParamLabels {
                    name: cmd.tag_name().name().to_string(),
                    labels,
                    units,
                });
            }
            out.insert(key, cmds);
        }
        out
    })
}

/// The command list shown in the Flight Plan "Command" combo for this vehicle type.
pub fn commands_for(fw: Firmware) -> &'static [CmdParamLabels] {
    table()
        .get(section_for(fw))
        .map(|v| v.as_slice())
        .unwrap_or(&[])
}

pub fn labels_for(fw: Firmware, id: u16) -> Option<&'static CmdParamLabels> {
    let name = cmd_name(id);
    commands_for(fw).iter().find(|c| c.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names() {
        assert_eq!(cmd_name(16), "WAYPOINT");
        assert_eq!(cmd_name(177), "DO_JUMP");
        assert_eq!(cmd_name(112), "CONDITION_DELAY");
        assert_eq!(cmd_id("LOITER_TURNS"), Some(18));
    }

    #[test]
    fn xml_labels() {
        let l = labels_for(Firmware::ArduCopter2, 16).unwrap();
        assert_eq!(l.labels[0], "Delay");
        assert_eq!(l.labels[4], "Lat");
        assert!(commands_for(Firmware::ArduPlane).len() > 10);
    }
}
