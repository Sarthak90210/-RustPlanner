//! A small built-in simulated ArduCopter that speaks MAVLink over UDP.
//!
//! It stands in for SITL so the ground station can be exercised end to end (connect, params,
//! missions, modes, arming, guided, auto) on any platform, including mobile.

use std::collections::BTreeMap;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::*;
use mavlink::{MavHeader, MavlinkVersion};
use num_traits::FromPrimitive;

use crate::geo::PointLatLngAlt;
use crate::mavcmd;
use crate::mission::Locationwp;

/// ArduPilot's default SITL home (Canberra Model Aircraft Club).
pub const SITL_HOME: PointLatLngAlt = PointLatLngAlt::new(-35.363261, 149.165230, 584.0);

const MODE_STABILIZE: u32 = 0;
const MODE_AUTO: u32 = 3;
const MODE_GUIDED: u32 = 4;
const MODE_LOITER: u32 = 5;
const MODE_RTL: u32 = 6;
const MODE_LAND: u32 = 9;

pub struct SimHandle {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SimHandle {
    pub fn stop(mut self) {
        self.shutdown();
    }
    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Start a simulated vehicle that sends to `127.0.0.1:gcs_port` (like SITL's `--out udp:...:14550`).
pub fn spawn(gcs_port: u16) -> io::Result<SimHandle> {
    let sock = UdpSocket::bind("127.0.0.1:0")?;
    sock.set_read_timeout(Some(Duration::from_millis(10)))?;
    let gcs: SocketAddr = format!("127.0.0.1:{gcs_port}")
        .parse()
        .map_err(io::Error::other)?;
    let stop = Arc::new(AtomicBool::new(false));
    let st = stop.clone();
    let thread = std::thread::Builder::new()
        .name("sim".into())
        .spawn(move || {
            let mut v = Vehicle::new(sock, gcs);
            v.run(&st);
        })?;
    Ok(SimHandle {
        stop,
        thread: Some(thread),
    })
}

struct Vehicle {
    sock: UdpSocket,
    gcs: SocketAddr,
    seq: u8,
    boot: Instant,
    params: BTreeMap<String, f32>,
    mission: Vec<Locationwp>,
    upload: Option<(u16, Vec<Locationwp>)>,
    armed: bool,
    mode: u32,
    pos: PointLatLngAlt, // alt = relative metres
    yaw: f32,
    roll: f32,
    pitch: f32,
    gs: f32,
    climb: f32,
    target: Option<PointLatLngAlt>,
    current_wp: u16,
    battery: f32,
    log: Vec<String>,
}

impl Vehicle {
    fn new(sock: UdpSocket, gcs: SocketAddr) -> Self {
        let mut params = BTreeMap::new();
        for (n, v) in [
            ("SYSID_THISMAV", 1.0),
            ("FRAME_CLASS", 1.0),
            ("FRAME_TYPE", 1.0),
            ("ANGLE_MAX", 4500.0),
            ("WPNAV_SPEED", 1000.0),
            ("WPNAV_SPEED_UP", 250.0),
            ("WPNAV_SPEED_DN", 150.0),
            ("WPNAV_RADIUS", 200.0),
            ("RTL_ALT", 1500.0),
            ("PILOT_SPEED_UP", 250.0),
            ("FLTMODE_CH", 5.0),
            ("FLTMODE1", 0.0),
            ("FLTMODE2", 2.0),
            ("FLTMODE3", 5.0),
            ("FLTMODE4", 3.0),
            ("FLTMODE5", 6.0),
            ("FLTMODE6", 9.0),
            ("ATC_RAT_RLL_P", 0.135),
            ("ATC_RAT_RLL_I", 0.135),
            ("ATC_RAT_RLL_D", 0.0036),
            ("ATC_RAT_PIT_P", 0.135),
            ("ATC_RAT_PIT_I", 0.135),
            ("ATC_RAT_PIT_D", 0.0036),
            ("ATC_RAT_YAW_P", 0.18),
            ("ATC_RAT_YAW_I", 0.018),
            ("BATT_MONITOR", 4.0),
            ("BATT_CAPACITY", 3300.0),
            ("FENCE_ENABLE", 0.0),
            ("ARMING_CHECK", 1.0),
            ("SERIAL0_BAUD", 115.0),
            ("SERIAL1_PROTOCOL", 2.0),
            ("RC1_MIN", 1100.0),
            ("RC1_MAX", 1900.0),
            ("RC1_TRIM", 1500.0),
            ("RC3_MIN", 1100.0),
            ("RC3_MAX", 1900.0),
            ("COMPASS_USE", 1.0),
            ("GPS_TYPE", 1.0),
        ] {
            params.insert(n.to_string(), v);
        }
        Self {
            sock,
            gcs,
            seq: 0,
            boot: Instant::now(),
            params,
            mission: vec![Locationwp {
                lat: SITL_HOME.lat,
                lng: SITL_HOME.lng,
                alt: SITL_HOME.alt as f32,
                id: 16,
                ..Default::default()
            }],
            upload: None,
            armed: false,
            mode: MODE_STABILIZE,
            pos: PointLatLngAlt::new(SITL_HOME.lat, SITL_HOME.lng, 0.0),
            yaw: 0.0,
            roll: 0.0,
            pitch: 0.0,
            gs: 0.0,
            climb: 0.0,
            target: None,
            current_wp: 0,
            battery: 100.0,
            log: Vec::new(),
        }
    }

    fn send(&mut self, msg: MavMessage) {
        let hdr = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: self.seq,
        };
        self.seq = self.seq.wrapping_add(1);
        let mut buf = Vec::with_capacity(300);
        if mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, hdr, &msg).is_ok() {
            let _ = self.sock.send_to(&buf, self.gcs);
        }
    }

    fn text(&mut self, sev: MavSeverity, s: &str) {
        self.log.push(s.to_string());
        self.send(MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity: sev,
            text: s.into(),
            id: 0,
            chunk_seq: 0,
        }));
    }

    fn run(&mut self, stop: &AtomicBool) {
        let mut buf = vec![0u8; 2048];
        let mut last_fast = Instant::now();
        let mut last_slow = Instant::now();
        self.text(
            MavSeverity::MAV_SEVERITY_INFO,
            "ArduCopter V4.6.0 (rust-sim)",
        );
        while !stop.load(Ordering::Relaxed) {
            if let Ok((n, _)) = self.sock.recv_from(&mut buf) {
                let mut r = mavlink::MavlinkReader::new(&buf[..n]);
                while let Ok((_, msg)) = r.read_any_message::<MavMessage>() {
                    self.handle(msg);
                }
            }
            let now = Instant::now();
            let dt = now.duration_since(last_fast).as_secs_f32();
            if dt >= 0.1 {
                last_fast = now;
                self.physics(dt);
                self.fast_telemetry();
            }
            if now.duration_since(last_slow) >= Duration::from_secs(1) {
                last_slow = now;
                self.slow_telemetry();
            }
        }
    }

    fn handle(&mut self, msg: MavMessage) {
        match msg {
            MavMessage::PARAM_REQUEST_LIST(_) => {
                let all: Vec<(String, f32)> =
                    self.params.iter().map(|(k, v)| (k.clone(), *v)).collect();
                let count = all.len() as u16;
                for (i, (k, v)) in all.into_iter().enumerate() {
                    self.send_param(&k, v, i as u16, count);
                }
            }
            MavMessage::PARAM_REQUEST_READ(r) => {
                let count = self.params.len() as u16;
                let entry = if r.param_index >= 0 {
                    self.params
                        .iter()
                        .nth(r.param_index as usize)
                        .map(|(k, v)| (k.clone(), *v, r.param_index as u16))
                } else {
                    let name = name16(&r.param_id);
                    self.params
                        .iter()
                        .position(|(k, _)| *k == name)
                        .map(|i| (name.clone(), self.params[&name], i as u16))
                };
                if let Some((k, v, i)) = entry {
                    self.send_param(&k, v, i, count);
                }
            }
            MavMessage::PARAM_SET(p) => {
                let name = name16(&p.param_id);
                if let Some(idx) = self.params.keys().position(|k| *k == name) {
                    self.params.insert(name.clone(), p.param_value);
                    let count = self.params.len() as u16;
                    self.send_param(&name, p.param_value, idx as u16, count);
                }
            }
            MavMessage::MISSION_REQUEST_LIST(r) => {
                let count = if r.mission_type == MavMissionType::MAV_MISSION_TYPE_MISSION {
                    self.mission.len() as u16
                } else {
                    0
                };
                self.send(MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
                    count,
                    target_system: 255,
                    target_component: 190,
                    mission_type: r.mission_type,
                    opaque_id: 0,
                }));
            }
            MavMessage::MISSION_REQUEST_INT(r) => {
                if let Some(wp) = self.mission.get(r.seq as usize).copied() {
                    if let Some(item) = wp.to_mission_item_int(r.seq, 255, 190, r.mission_type) {
                        self.send(MavMessage::MISSION_ITEM_INT(item));
                    }
                }
            }
            MavMessage::MISSION_COUNT(c) => {
                if c.mission_type != MavMissionType::MAV_MISSION_TYPE_MISSION {
                    self.mission_ack(c.mission_type, MavMissionResult::MAV_MISSION_UNSUPPORTED);
                } else if c.count == 0 {
                    self.mission.truncate(1);
                    self.mission_ack(c.mission_type, MavMissionResult::MAV_MISSION_ACCEPTED);
                } else {
                    self.upload = Some((c.count, Vec::new()));
                    self.request_upload_item(0);
                }
            }
            MavMessage::MISSION_ITEM_INT(m) => {
                if m.current == 2 {
                    // guided "fly to here"
                    let wp = Locationwp::from_mission_item_int(&m);
                    self.target = Some(PointLatLngAlt::new(wp.lat, wp.lng, wp.alt as f64));
                    self.mission_ack(m.mission_type, MavMissionResult::MAV_MISSION_ACCEPTED);
                    return;
                }
                let mut done = None;
                let mut next = None;
                if let Some((count, items)) = self.upload.as_mut() {
                    if m.seq as usize == items.len() {
                        items.push(Locationwp::from_mission_item_int(&m));
                        if items.len() == *count as usize {
                            done = Some(std::mem::take(items));
                        } else {
                            next = Some(items.len() as u16);
                        }
                    }
                }
                if let Some(n) = next {
                    self.request_upload_item(n);
                }
                if let Some(items) = done {
                    self.upload = None;
                    self.mission = items;
                    self.mission_ack(m.mission_type, MavMissionResult::MAV_MISSION_ACCEPTED);
                    let n = self.mission.len();
                    self.text(
                        MavSeverity::MAV_SEVERITY_INFO,
                        &format!("Mission: {n} items received"),
                    );
                }
            }
            MavMessage::MISSION_SET_CURRENT(s) => {
                self.current_wp = s.seq;
                self.target = None;
            }
            MavMessage::SET_MODE(s) => self.set_mode(s.custom_mode),
            MavMessage::COMMAND_LONG(c) => self.command(c),
            _ => {}
        }
    }

    fn request_upload_item(&mut self, seq: u16) {
        self.send(MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            seq,
            target_system: 255,
            target_component: 190,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        }));
    }

    fn mission_ack(&mut self, mt: MavMissionType, r: MavMissionResult) {
        self.send(MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: 255,
            target_component: 190,
            mavtype: r,
            mission_type: mt,
            opaque_id: 0,
        }));
    }

    fn send_param(&mut self, name: &str, value: f32, index: u16, count: u16) {
        self.send(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value: value,
            param_count: count,
            param_index: index,
            param_id: name.into(),
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        }));
    }

    fn set_mode(&mut self, mode: u32) {
        self.mode = mode;
        self.target = None;
        if mode == MODE_AUTO {
            self.current_wp = 1;
        }
        if mode == MODE_RTL {
            self.target = Some(PointLatLngAlt::new(
                SITL_HOME.lat,
                SITL_HOME.lng,
                self.params["RTL_ALT"] as f64 / 100.0,
            ));
        }
    }

    fn ack(&mut self, command: MavCmd, result: MavResult) {
        self.send(MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: 255,
            target_component: 190,
        }));
    }

    fn command(&mut self, c: COMMAND_LONG_DATA) {
        use MavCmd::*;
        let res = match c.command {
            MAV_CMD_COMPONENT_ARM_DISARM => {
                let arm = c.param1 >= 0.5;
                if !arm && self.pos.alt > 0.5 && c.param2 != 21196.0 {
                    self.text(
                        MavSeverity::MAV_SEVERITY_CRITICAL,
                        "Disarm: vehicle is flying",
                    );
                    MavResult::MAV_RESULT_DENIED
                } else {
                    self.armed = arm;
                    self.text(
                        MavSeverity::MAV_SEVERITY_INFO,
                        if arm {
                            "Arming motors"
                        } else {
                            "Disarming motors"
                        },
                    );
                    MavResult::MAV_RESULT_ACCEPTED
                }
            }
            MAV_CMD_DO_SET_MODE => {
                self.set_mode(c.param2 as u32);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_NAV_TAKEOFF => {
                if !self.armed || self.mode != MODE_GUIDED {
                    MavResult::MAV_RESULT_FAILED
                } else {
                    self.target = Some(PointLatLngAlt::new(
                        self.pos.lat,
                        self.pos.lng,
                        c.param7 as f64,
                    ));
                    MavResult::MAV_RESULT_ACCEPTED
                }
            }
            MAV_CMD_NAV_RETURN_TO_LAUNCH => {
                self.set_mode(MODE_RTL);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_NAV_LAND => {
                self.set_mode(MODE_LAND);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_MISSION_START => {
                self.set_mode(MODE_AUTO);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_REQUEST_MESSAGE => {
                match c.param1 as u32 {
                    242 => self.home_position(),
                    148 => self.send(MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
                        flight_sw_version: (4 << 24) | (6 << 16),
                        ..Default::default()
                    })),
                    _ => {}
                }
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_PREFLIGHT_CALIBRATION
            | MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            | MAV_CMD_DO_SET_SERVO
            | MAV_CMD_DO_REPEAT_SERVO
            | MAV_CMD_DO_SET_RELAY => MavResult::MAV_RESULT_ACCEPTED,
            _ => MavResult::MAV_RESULT_UNSUPPORTED,
        };
        self.ack(c.command, res);
    }

    fn home_position(&mut self) {
        self.send(MavMessage::HOME_POSITION(HOME_POSITION_DATA {
            latitude: (SITL_HOME.lat * 1e7) as i32,
            longitude: (SITL_HOME.lng * 1e7) as i32,
            altitude: (SITL_HOME.alt * 1000.0) as i32,
            q: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        }));
    }

    /// Simple kinematic model: fly toward the target at WPNAV_SPEED, climb at PILOT_SPEED_UP.
    fn physics(&mut self, dt: f32) {
        if self.armed {
            self.battery = (self.battery - dt * 0.02).max(0.0);
        }
        if self.armed && self.mode == MODE_AUTO && self.target.is_none() {
            self.next_auto_target();
        }
        if self.mode == MODE_LAND && self.armed {
            self.target = Some(PointLatLngAlt::new(self.pos.lat, self.pos.lng, 0.0));
        }

        let speed = self.params["WPNAV_SPEED"] / 100.0;
        let vz_max = self.params["WPNAV_SPEED_UP"] / 100.0;
        let (mut gs, mut climb, mut roll, mut pitch) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        if let (true, Some(t)) = (self.armed, self.target) {
            let here = PointLatLngAlt::new(self.pos.lat, self.pos.lng, 0.0);
            let d = here.distance(&t) as f32;
            if d > 0.5 {
                let brg = here.bearing(&t) as f32;
                let mut dy = brg - self.yaw;
                if dy > 180.0 {
                    dy -= 360.0;
                }
                if dy < -180.0 {
                    dy += 360.0;
                }
                self.yaw = (self.yaw + dy.clamp(-90.0 * dt, 90.0 * dt) + 360.0) % 360.0;
                roll = (dy * 0.3).clamp(-20.0, 20.0);
                gs = speed.min(d / 2.0 + 0.5);
                pitch = -(gs / speed) * 12.0;
                let np = here.newpos(brg as f64, (gs * dt) as f64);
                self.pos.lat = np.lat;
                self.pos.lng = np.lng;
            }
            let dz = t.alt as f32 - self.pos.alt as f32;
            climb = dz.clamp(-vz_max * dt, vz_max * dt) / dt.max(1e-3);
            self.pos.alt = (self.pos.alt + (climb * dt) as f64).max(0.0);

            let wp_radius = self.params["WPNAV_RADIUS"] / 100.0;
            if d <= wp_radius.max(1.0) && dz.abs() < 0.5 {
                if self.mode == MODE_AUTO {
                    self.text(
                        MavSeverity::MAV_SEVERITY_INFO,
                        &format!("Reached command #{}", self.current_wp),
                    );
                    self.current_wp += 1;
                    self.target = None;
                } else if self.mode == MODE_RTL {
                    self.set_mode(MODE_LAND);
                }
            }
        }
        if self.armed && self.pos.alt < 0.05 && matches!(self.mode, MODE_LAND) {
            self.armed = false;
            self.text(MavSeverity::MAV_SEVERITY_INFO, "Land complete, disarming");
            self.set_mode(MODE_LOITER);
        }
        self.gs = gs;
        self.climb = climb;
        self.roll += (roll - self.roll) * 0.3;
        self.pitch += (pitch - self.pitch) * 0.3;
    }

    fn next_auto_target(&mut self) {
        loop {
            let Some(wp) = self.mission.get(self.current_wp as usize).copied() else {
                self.text(
                    MavSeverity::MAV_SEVERITY_INFO,
                    "Mission complete, changing mode to RTL",
                );
                self.set_mode(MODE_RTL);
                return;
            };
            match wp.id {
                mavcmd::TAKEOFF => {
                    self.target = Some(PointLatLngAlt::new(
                        self.pos.lat,
                        self.pos.lng,
                        wp.alt as f64,
                    ));
                    return;
                }
                mavcmd::WAYPOINT
                | mavcmd::SPLINE_WAYPOINT
                | mavcmd::LOITER_TIME
                | mavcmd::LOITER_TURNS
                | mavcmd::LOITER_UNLIM => {
                    let (lat, lng) = if wp.lat == 0.0 && wp.lng == 0.0 {
                        (self.pos.lat, self.pos.lng)
                    } else {
                        (wp.lat, wp.lng)
                    };
                    self.target = Some(PointLatLngAlt::new(lat, lng, wp.alt as f64));
                    return;
                }
                mavcmd::LAND => {
                    self.set_mode(MODE_LAND);
                    return;
                }
                mavcmd::RETURN_TO_LAUNCH => {
                    self.set_mode(MODE_RTL);
                    return;
                }
                mavcmd::DO_JUMP => {
                    self.current_wp = wp.p1 as u16;
                }
                _ => self.current_wp += 1,
            }
        }
    }

    fn fast_telemetry(&mut self) {
        let t = self.boot.elapsed().as_millis() as u32;
        self.send(MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: t,
            roll: self.roll.to_radians(),
            pitch: self.pitch.to_radians(),
            yaw: if self.yaw > 180.0 {
                self.yaw - 360.0
            } else {
                self.yaw
            }
            .to_radians(),
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        }));
        let (vn, ve) = (
            self.gs * self.yaw.to_radians().cos(),
            self.gs * self.yaw.to_radians().sin(),
        );
        self.send(MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            time_boot_ms: t,
            lat: (self.pos.lat * 1e7) as i32,
            lon: (self.pos.lng * 1e7) as i32,
            alt: ((SITL_HOME.alt + self.pos.alt) * 1000.0) as i32,
            relative_alt: (self.pos.alt * 1000.0) as i32,
            vx: (vn * 100.0) as i16,
            vy: (ve * 100.0) as i16,
            vz: (-self.climb * 100.0) as i16,
            hdg: (self.yaw * 100.0) as u16,
        }));
        self.send(MavMessage::VFR_HUD(VFR_HUD_DATA {
            airspeed: self.gs,
            groundspeed: self.gs,
            alt: (SITL_HOME.alt + self.pos.alt) as f32,
            climb: self.climb,
            heading: self.yaw as i16,
            throttle: if self.armed {
                45 + (self.climb * 5.0) as u16
            } else {
                0
            },
        }));
        if let Some(tgt) = self.target {
            let here = PointLatLngAlt::new(self.pos.lat, self.pos.lng, 0.0);
            self.send(MavMessage::NAV_CONTROLLER_OUTPUT(
                NAV_CONTROLLER_OUTPUT_DATA {
                    nav_roll: self.roll,
                    nav_pitch: self.pitch,
                    alt_error: tgt.alt as f32 - self.pos.alt as f32,
                    aspd_error: 0.0,
                    xtrack_error: 0.0,
                    nav_bearing: here.bearing(&tgt) as i16,
                    target_bearing: here.bearing(&tgt) as i16,
                    wp_dist: here.distance(&tgt) as u16,
                },
            ));
        }
    }

    fn slow_telemetry(&mut self) {
        let mut base = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED;
        if self.armed {
            base |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }
        self.send(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: self.mode,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: base,
            system_status: if self.armed {
                MavState::MAV_STATE_ACTIVE
            } else {
                MavState::MAV_STATE_STANDBY
            },
            mavlink_version: 3,
        }));
        let sensors = MavSysStatusSensor::from_bits_truncate(0x0020_FC2F);
        self.send(MavMessage::SYS_STATUS(SYS_STATUS_DATA {
            onboard_control_sensors_present: sensors,
            onboard_control_sensors_enabled: sensors,
            onboard_control_sensors_health: sensors,
            load: 250,
            voltage_battery: (11_100.0 + self.battery * 15.0) as u16,
            current_battery: if self.armed { 1250 } else { 50 },
            battery_remaining: self.battery as i8,
            ..Default::default()
        }));
        self.send(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: self.boot.elapsed().as_micros() as u64,
            lat: (self.pos.lat * 1e7) as i32,
            lon: (self.pos.lng * 1e7) as i32,
            alt: ((SITL_HOME.alt + self.pos.alt) * 1000.0) as i32,
            eph: 121,
            epv: 200,
            vel: (self.gs * 100.0) as u16,
            cog: (self.yaw * 100.0) as u16,
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            satellites_visible: 10,
            ..Default::default()
        }));
        self.send(MavMessage::MISSION_CURRENT(MISSION_CURRENT_DATA {
            seq: self.current_wp,
            ..Default::default()
        }));
        let thr = if self.armed { 1500 } else { 1000 };
        self.send(MavMessage::RC_CHANNELS(RC_CHANNELS_DATA {
            time_boot_ms: self.boot.elapsed().as_millis() as u32,
            chancount: 8,
            chan1_raw: 1500,
            chan2_raw: 1500,
            chan3_raw: 1000,
            chan4_raw: 1500,
            chan5_raw: 1100 + (self.mode.min(6) as u16) * 100,
            chan6_raw: 1500,
            chan7_raw: 1000,
            chan8_raw: 1000,
            rssi: 255,
            ..Default::default()
        }));
        self.send(MavMessage::SERVO_OUTPUT_RAW(SERVO_OUTPUT_RAW_DATA {
            servo1_raw: thr,
            servo2_raw: thr,
            servo3_raw: thr,
            servo4_raw: thr,
            ..Default::default()
        }));
        self.send(MavMessage::VIBRATION(VIBRATION_DATA {
            vibration_x: 3.1,
            vibration_y: 2.8,
            vibration_z: 4.5,
            ..Default::default()
        }));
        self.home_position();
        let _ = MavCmd::from_u16(0);
    }
}

fn name16(id: &mavlink::types::CharArray<16>) -> String {
    let b: &[u8] = &id[..];
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).to_string()
}
