//! Firmware detection and flight-mode tables (`Common.getModesList`).

use mavlink::dialects::ardupilotmega::{MavAutopilot, MavType};

use crate::param_meta;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Firmware {
    #[default]
    ArduCopter2,
    ArduPlane,
    ArduRover,
    ArduSub,
    ArduTracker,
    Px4,
    Other,
}

impl Firmware {
    pub fn from_heartbeat(autopilot: MavAutopilot, mavtype: MavType) -> Self {
        use MavType::*;
        if autopilot == MavAutopilot::MAV_AUTOPILOT_PX4 {
            return Firmware::Px4;
        }
        match mavtype {
            MAV_TYPE_QUADROTOR | MAV_TYPE_HEXAROTOR | MAV_TYPE_OCTOROTOR | MAV_TYPE_TRICOPTER
            | MAV_TYPE_HELICOPTER | MAV_TYPE_COAXIAL | MAV_TYPE_DODECAROTOR
            | MAV_TYPE_DECAROTOR => Firmware::ArduCopter2,
            MAV_TYPE_FIXED_WING
            | MAV_TYPE_VTOL_TAILSITTER_DUOROTOR
            | MAV_TYPE_VTOL_TAILSITTER_QUADROTOR
            | MAV_TYPE_VTOL_TILTROTOR
            | MAV_TYPE_VTOL_FIXEDROTOR
            | MAV_TYPE_VTOL_TAILSITTER
            | MAV_TYPE_VTOL_TILTWING
            | MAV_TYPE_VTOL_RESERVED5 => Firmware::ArduPlane,
            MAV_TYPE_GROUND_ROVER | MAV_TYPE_SURFACE_BOAT => Firmware::ArduRover,
            MAV_TYPE_SUBMARINE => Firmware::ArduSub,
            MAV_TYPE_ANTENNA_TRACKER => Firmware::ArduTracker,
            _ => Firmware::Other,
        }
    }

    /// Section name in ParameterMetaData.xml.
    pub fn meta_key(&self) -> &'static str {
        match self {
            Firmware::ArduCopter2 | Firmware::Other | Firmware::Px4 => "ArduCopter2",
            Firmware::ArduPlane => "ArduPlane",
            Firmware::ArduRover => "ArduRover",
            Firmware::ArduSub => "ArduSub",
            Firmware::ArduTracker => "ArduTracker",
        }
    }

    /// Name of the flight-mode parameter for each switch position.
    pub fn mode_param_prefix(&self) -> &'static str {
        match self {
            Firmware::ArduRover => "MODE",
            _ => "FLTMODE",
        }
    }

    pub fn modes(&self) -> Vec<(u32, String)> {
        match self {
            Firmware::ArduTracker => vec![
                (0, "MANUAL".into()),
                (1, "STOP".into()),
                (2, "SCAN".into()),
                (3, "SERVO_TEST".into()),
                (10, "AUTO".into()),
                (16, "INITIALISING".into()),
            ],
            Firmware::Px4 => vec![
                (1 << 16, "Manual".into()),
                (5 << 16, "Acro".into()),
                (7 << 16, "Stabalized".into()),
                (8 << 16, "Rattitude".into()),
                (2 << 16, "Altitude Control".into()),
                (3 << 16, "Position Control".into()),
                (6 << 16, "Offboard Control".into()),
                ((4 << 16) + (1 << 24), "Auto: Ready".into()),
                ((4 << 16) + (2 << 24), "Auto: Takeoff".into()),
                ((4 << 16) + (3 << 24), "Loiter".into()),
                ((4 << 16) + (4 << 24), "Auto".into()),
                ((4 << 16) + (5 << 24), "RTL".into()),
                ((4 << 16) + (6 << 24), "Auto: Landing".into()),
            ],
            Firmware::ArduSub => vec![
                (0, "Stabilize".into()),
                (1, "Acro".into()),
                (2, "Depth Hold".into()),
                (3, "Auto".into()),
                (4, "Guided".into()),
                (7, "Circle".into()),
                (9, "Surface".into()),
                (16, "Position Hold".into()),
                (19, "Manual".into()),
            ],
            _ => {
                let param = format!("{}1", self.mode_param_prefix());
                let mut v: Vec<(u32, String)> = param_meta::options(*self, &param)
                    .into_iter()
                    .map(|(k, s)| (k as u32, s))
                    .collect();
                if *self == Firmware::ArduPlane {
                    v.push((16, "INITIALISING".into()));
                }
                v
            }
        }
    }

    pub fn mode_name(&self, custom_mode: u32) -> String {
        self.modes()
            .into_iter()
            .find(|(k, _)| *k == custom_mode)
            .map(|(_, s)| s)
            .unwrap_or_else(|| format!("Mode({custom_mode})"))
    }
}
