//! Port of `CurrentState` (the `cs` object): live telemetry decoded from MAVLink.

use std::time::Instant;

use mavlink::dialects::ardupilotmega::*;

use crate::firmware::Firmware;
use crate::geo::PointLatLngAlt;

#[derive(Debug, Clone)]
pub struct StatusMessage {
    pub time: String,
    pub severity: u8,
    pub text: String,
}

/// Field names intentionally match Mission Planner's `CurrentState` properties so that the
/// Quick / Status views read the same.
#[derive(Debug, Clone)]
pub struct CurrentState {
    pub firmware: Firmware,
    pub sysid: u8,
    pub compid: u8,
    pub mavtype: u32,

    // attitude
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,

    // position
    pub lat: f64,
    pub lng: f64,
    pub alt: f32,
    pub altasl: f32,
    pub groundcourse: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,

    // gps
    pub gpsstatus: u8,
    pub satcount: u8,
    pub gpshdop: f32,
    pub gpsvdop: f32,
    pub gpsalt: f32,

    // vfr hud
    pub airspeed: f32,
    pub groundspeed: f32,
    pub verticalspeed: f32,
    pub throttle: f32,

    // nav
    pub nav_roll: f32,
    pub nav_pitch: f32,
    pub nav_bearing: f32,
    pub target_bearing: f32,
    pub wp_dist: f32,
    pub alt_error: f32,
    pub aspd_error: f32,
    pub xtrack_error: f32,
    pub wpno: u16,

    // status
    pub armed: bool,
    pub custom_mode: u32,
    pub mode: String,
    pub base_mode: u8,
    pub system_status: u8,
    pub load: f32,
    pub battery_voltage: f32,
    pub current: f32,
    pub battery_remaining: i32,
    pub battery_usedmah: f32,
    pub sensors_present: u32,
    pub sensors_enabled: u32,
    pub sensors_health: u32,

    // radio
    pub ch_in: [u16; 18],
    pub ch_out: [u16; 16],
    pub rssi: u8,
    pub remrssi: u8,
    pub noise: u8,
    pub remnoise: u8,
    pub txbuf: u8,
    pub rxerrors: u16,

    // ekf / vibe
    pub ekfstatus: f32,
    pub ekfflags: u16,
    pub vibex: f32,
    pub vibey: f32,
    pub vibez: f32,
    pub clip0: u32,
    pub clip1: u32,
    pub clip2: u32,

    // misc
    pub rangefinder: f32,
    pub wind_dir: f32,
    pub wind_vel: f32,
    pub press_temp: f32,
    pub home: Option<PointLatLngAlt>,
    pub time_in_air: f32,
    pub dist_traveled: f32,
    pub time_since_boot: f32,
    pub message_high: String,
    pub message_high_severity: u8,
    pub linkqualitygcs: f32,
    pub packets_received: u64,
    pub packets_lost: u64,

    pub track: Vec<(f64, f64)>,

    last_pos_update: Option<Instant>,
    last_seq: Option<u8>,
}

impl Default for CurrentState {
    fn default() -> Self {
        Self {
            firmware: Firmware::default(),
            sysid: 0,
            compid: 0,
            mavtype: 0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
            lat: 0.0,
            lng: 0.0,
            alt: 0.0,
            altasl: 0.0,
            groundcourse: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            gpsstatus: 0,
            satcount: 0,
            gpshdop: 0.0,
            gpsvdop: 0.0,
            gpsalt: 0.0,
            airspeed: 0.0,
            groundspeed: 0.0,
            verticalspeed: 0.0,
            throttle: 0.0,
            nav_roll: 0.0,
            nav_pitch: 0.0,
            nav_bearing: 0.0,
            target_bearing: 0.0,
            wp_dist: 0.0,
            alt_error: 0.0,
            aspd_error: 0.0,
            xtrack_error: 0.0,
            wpno: 0,
            armed: false,
            custom_mode: 0,
            mode: "Unknown".into(),
            base_mode: 0,
            system_status: 0,
            load: 0.0,
            battery_voltage: 0.0,
            current: 0.0,
            battery_remaining: 0,
            battery_usedmah: 0.0,
            sensors_present: 0,
            sensors_enabled: 0,
            sensors_health: 0,
            ch_in: [0; 18],
            ch_out: [0; 16],
            rssi: 0,
            remrssi: 0,
            noise: 0,
            remnoise: 0,
            txbuf: 0,
            rxerrors: 0,
            ekfstatus: 0.0,
            ekfflags: 0,
            vibex: 0.0,
            vibey: 0.0,
            vibez: 0.0,
            clip0: 0,
            clip1: 0,
            clip2: 0,
            rangefinder: 0.0,
            wind_dir: 0.0,
            wind_vel: 0.0,
            press_temp: 0.0,
            home: None,
            time_in_air: 0.0,
            dist_traveled: 0.0,
            time_since_boot: 0.0,
            message_high: String::new(),
            message_high_severity: 7,
            linkqualitygcs: 0.0,
            packets_received: 0,
            packets_lost: 0,
            track: Vec::new(),
            last_pos_update: None,
            last_seq: None,
        }
    }
}

impl CurrentState {
    pub fn location(&self) -> PointLatLngAlt {
        PointLatLngAlt::new(self.lat, self.lng, self.alt as f64)
    }

    #[allow(non_snake_case)]
    pub fn DistToHome(&self) -> f32 {
        match self.home {
            Some(h) if !h.is_zero() && !(self.lat == 0.0 && self.lng == 0.0) => {
                h.distance(&self.location()) as f32
            }
            _ => 0.0,
        }
    }

    pub fn gps_fix_name(&self) -> &'static str {
        match self.gpsstatus {
            0 => "No GPS",
            1 => "No Fix",
            2 => "2D Fix",
            3 => "3D Fix",
            4 => "3D dgps",
            5 => "3D rtk float",
            6 => "3D rtk fixed",
            7 => "Static",
            8 => "PPP",
            _ => "Unknown",
        }
    }

    /// Track packet loss from the MAVLink sequence number (`linkqualitygcs`).
    pub fn note_packet(&mut self, seq: u8) {
        if let Some(last) = self.last_seq {
            let expected = last.wrapping_add(1);
            if seq != expected {
                self.packets_lost += seq.wrapping_sub(expected) as u64;
            }
        }
        self.last_seq = Some(seq);
        self.packets_received += 1;
        let total = (self.packets_received + self.packets_lost).max(1);
        self.linkqualitygcs = (self.packets_received as f32 / total as f32) * 100.0;
    }

    /// Every numeric field, as listed in Mission Planner's Status tab.
    pub fn fields(&self) -> Vec<(&'static str, f64)> {
        let mut v: Vec<(&'static str, f64)> = vec![
            ("roll", self.roll as f64),
            ("pitch", self.pitch as f64),
            ("yaw", self.yaw as f64),
            ("lat", self.lat),
            ("lng", self.lng),
            ("alt", self.alt as f64),
            ("altasl", self.altasl as f64),
            ("groundcourse", self.groundcourse as f64),
            ("gpsstatus", self.gpsstatus as f64),
            ("satcount", self.satcount as f64),
            ("gpshdop", self.gpshdop as f64),
            ("gpsvdop", self.gpsvdop as f64),
            ("gpsalt", self.gpsalt as f64),
            ("airspeed", self.airspeed as f64),
            ("groundspeed", self.groundspeed as f64),
            ("verticalspeed", self.verticalspeed as f64),
            ("throttle", self.throttle as f64),
            ("nav_roll", self.nav_roll as f64),
            ("nav_pitch", self.nav_pitch as f64),
            ("nav_bearing", self.nav_bearing as f64),
            ("target_bearing", self.target_bearing as f64),
            ("wp_dist", self.wp_dist as f64),
            ("alt_error", self.alt_error as f64),
            ("aspd_error", self.aspd_error as f64),
            ("xtrack_error", self.xtrack_error as f64),
            ("wpno", self.wpno as f64),
            ("armed", self.armed as u8 as f64),
            ("battery_voltage", self.battery_voltage as f64),
            ("current", self.current as f64),
            ("battery_remaining", self.battery_remaining as f64),
            ("battery_usedmah", self.battery_usedmah as f64),
            ("load", self.load as f64),
            ("rssi", self.rssi as f64),
            ("remrssi", self.remrssi as f64),
            ("noise", self.noise as f64),
            ("remnoise", self.remnoise as f64),
            ("txbuf", self.txbuf as f64),
            ("rxerrors", self.rxerrors as f64),
            ("ekfstatus", self.ekfstatus as f64),
            ("vibex", self.vibex as f64),
            ("vibey", self.vibey as f64),
            ("vibez", self.vibez as f64),
            ("clip0", self.clip0 as f64),
            ("clip1", self.clip1 as f64),
            ("clip2", self.clip2 as f64),
            ("rangefinder1", self.rangefinder as f64),
            ("wind_dir", self.wind_dir as f64),
            ("wind_vel", self.wind_vel as f64),
            ("press_temp", self.press_temp as f64),
            ("DistToHome", self.DistToHome() as f64),
            ("timeInAir", self.time_in_air as f64),
            ("distTraveled", self.dist_traveled as f64),
            ("timeSinceBoot", self.time_since_boot as f64),
            ("linkqualitygcs", self.linkqualitygcs as f64),
            ("packets_received", self.packets_received as f64),
            ("packets_lost", self.packets_lost as f64),
            ("sysid", self.sysid as f64),
            ("compid", self.compid as f64),
        ];
        const CH_IN: [&str; 18] = [
            "ch1in", "ch2in", "ch3in", "ch4in", "ch5in", "ch6in", "ch7in", "ch8in", "ch9in",
            "ch10in", "ch11in", "ch12in", "ch13in", "ch14in", "ch15in", "ch16in", "ch17in",
            "ch18in",
        ];
        const CH_OUT: [&str; 16] = [
            "ch1out", "ch2out", "ch3out", "ch4out", "ch5out", "ch6out", "ch7out", "ch8out",
            "ch9out", "ch10out", "ch11out", "ch12out", "ch13out", "ch14out", "ch15out", "ch16out",
        ];
        for (i, n) in CH_IN.iter().enumerate() {
            v.push((n, self.ch_in[i] as f64));
        }
        for (i, n) in CH_OUT.iter().enumerate() {
            v.push((n, self.ch_out[i] as f64));
        }
        v
    }

    pub fn field(&self, name: &str) -> Option<f64> {
        self.fields()
            .into_iter()
            .find(|(n, _)| *n == name)
            .map(|(_, v)| v)
    }

    /// Port of `CurrentState.UpdateCurrentSettings` message handling.
    pub fn update(&mut self, msg: &MavMessage) {
        match msg {
            MavMessage::HEARTBEAT(hb) => {
                self.firmware = Firmware::from_heartbeat(hb.autopilot, hb.mavtype);
                self.mavtype = hb.mavtype as u32;
                self.base_mode = hb.base_mode.bits();
                self.armed = hb
                    .base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                self.custom_mode = hb.custom_mode;
                self.system_status = hb.system_status as u8;
                self.mode = self.firmware.mode_name(hb.custom_mode);
            }
            MavMessage::SYS_STATUS(s) => {
                self.load = s.load as f32 / 10.0;
                self.battery_voltage = s.voltage_battery as f32 / 1000.0;
                self.current = if s.current_battery < 0 {
                    0.0
                } else {
                    s.current_battery as f32 / 100.0
                };
                self.battery_remaining = s.battery_remaining as i32;
                self.sensors_present = s.onboard_control_sensors_present.bits();
                self.sensors_enabled = s.onboard_control_sensors_enabled.bits();
                self.sensors_health = s.onboard_control_sensors_health.bits();
            }
            MavMessage::ATTITUDE(a) => {
                self.roll = a.roll.to_degrees();
                self.pitch = a.pitch.to_degrees();
                self.yaw = (a.yaw.to_degrees() + 360.0) % 360.0;
                self.rollspeed = a.rollspeed.to_degrees();
                self.pitchspeed = a.pitchspeed.to_degrees();
                self.yawspeed = a.yawspeed.to_degrees();
                self.time_since_boot = a.time_boot_ms as f32 / 1000.0;
            }
            MavMessage::GLOBAL_POSITION_INT(g) => {
                let lat = g.lat as f64 / 1e7;
                let lng = g.lon as f64 / 1e7;
                if lat != 0.0 || lng != 0.0 {
                    if self.lat != 0.0 || self.lng != 0.0 {
                        let d = PointLatLngAlt::new(self.lat, self.lng, 0.0)
                            .distance(&PointLatLngAlt::new(lat, lng, 0.0));
                        if self.armed && d < 1000.0 {
                            self.dist_traveled += d as f32;
                        }
                    }
                    self.lat = lat;
                    self.lng = lng;
                    let push = self
                        .track
                        .last()
                        .map(|&(a, b)| (a - lat).abs() > 1e-6 || (b - lng).abs() > 1e-6)
                        .unwrap_or(true);
                    if push {
                        self.track.push((lat, lng));
                        if self.track.len() > 5000 {
                            self.track.drain(0..1000);
                        }
                    }
                }
                self.alt = g.relative_alt as f32 / 1000.0;
                self.altasl = g.alt as f32 / 1000.0;
                self.vx = g.vx as f32 / 100.0;
                self.vy = g.vy as f32 / 100.0;
                self.vz = g.vz as f32 / 100.0;
                let now = Instant::now();
                if let Some(t) = self.last_pos_update {
                    if self.armed && (self.groundspeed > 3.0 || self.alt > 1.0) {
                        self.time_in_air += now.duration_since(t).as_secs_f32();
                    }
                }
                self.last_pos_update = Some(now);
            }
            MavMessage::GPS_RAW_INT(g) => {
                self.gpsstatus = g.fix_type as u8;
                self.satcount = g.satellites_visible;
                self.gpshdop = if g.eph == u16::MAX {
                    99.99
                } else {
                    g.eph as f32 / 100.0
                };
                self.gpsvdop = if g.epv == u16::MAX {
                    99.99
                } else {
                    g.epv as f32 / 100.0
                };
                self.gpsalt = g.alt as f32 / 1000.0;
                if g.cog != u16::MAX {
                    self.groundcourse = g.cog as f32 / 100.0;
                }
            }
            MavMessage::VFR_HUD(v) => {
                self.airspeed = v.airspeed;
                self.groundspeed = v.groundspeed;
                self.verticalspeed = v.climb;
                self.throttle = v.throttle as f32;
            }
            MavMessage::NAV_CONTROLLER_OUTPUT(n) => {
                self.nav_roll = n.nav_roll;
                self.nav_pitch = n.nav_pitch;
                self.nav_bearing = n.nav_bearing as f32;
                self.target_bearing = n.target_bearing as f32;
                self.wp_dist = n.wp_dist as f32;
                self.alt_error = n.alt_error;
                self.aspd_error = n.aspd_error / 100.0;
                self.xtrack_error = n.xtrack_error;
            }
            MavMessage::MISSION_CURRENT(m) => self.wpno = m.seq,
            MavMessage::RC_CHANNELS(r) => {
                let c = [
                    r.chan1_raw,
                    r.chan2_raw,
                    r.chan3_raw,
                    r.chan4_raw,
                    r.chan5_raw,
                    r.chan6_raw,
                    r.chan7_raw,
                    r.chan8_raw,
                    r.chan9_raw,
                    r.chan10_raw,
                    r.chan11_raw,
                    r.chan12_raw,
                    r.chan13_raw,
                    r.chan14_raw,
                    r.chan15_raw,
                    r.chan16_raw,
                    r.chan17_raw,
                    r.chan18_raw,
                ];
                self.ch_in = c;
                if r.rssi != 255 {
                    self.rssi = r.rssi;
                }
            }
            MavMessage::RC_CHANNELS_RAW(r) => {
                let c = [
                    r.chan1_raw,
                    r.chan2_raw,
                    r.chan3_raw,
                    r.chan4_raw,
                    r.chan5_raw,
                    r.chan6_raw,
                    r.chan7_raw,
                    r.chan8_raw,
                ];
                self.ch_in[..8].copy_from_slice(&c);
            }
            MavMessage::SERVO_OUTPUT_RAW(s) => {
                let c = [
                    s.servo1_raw,
                    s.servo2_raw,
                    s.servo3_raw,
                    s.servo4_raw,
                    s.servo5_raw,
                    s.servo6_raw,
                    s.servo7_raw,
                    s.servo8_raw,
                    s.servo9_raw,
                    s.servo10_raw,
                    s.servo11_raw,
                    s.servo12_raw,
                    s.servo13_raw,
                    s.servo14_raw,
                    s.servo15_raw,
                    s.servo16_raw,
                ];
                self.ch_out = c;
            }
            MavMessage::RADIO_STATUS(r) => {
                self.rssi = r.rssi;
                self.remrssi = r.remrssi;
                self.noise = r.noise;
                self.remnoise = r.remnoise;
                self.txbuf = r.txbuf;
                self.rxerrors = r.rxerrors;
            }
            MavMessage::HOME_POSITION(h) => {
                self.home = Some(PointLatLngAlt::new(
                    h.latitude as f64 / 1e7,
                    h.longitude as f64 / 1e7,
                    h.altitude as f64 / 1000.0,
                ));
            }
            MavMessage::VIBRATION(v) => {
                self.vibex = v.vibration_x;
                self.vibey = v.vibration_y;
                self.vibez = v.vibration_z;
                self.clip0 = v.clipping_0;
                self.clip1 = v.clipping_1;
                self.clip2 = v.clipping_2;
            }
            MavMessage::EKF_STATUS_REPORT(e) => {
                self.ekfflags = e.flags.bits();
                self.ekfstatus = e
                    .velocity_variance
                    .max(e.pos_horiz_variance)
                    .max(e.pos_vert_variance)
                    .max(e.compass_variance)
                    .max(e.terrain_alt_variance);
            }
            MavMessage::BATTERY_STATUS(b) if b.id == 0 => {
                if b.current_consumed >= 0 {
                    self.battery_usedmah = b.current_consumed as f32;
                }
            }
            MavMessage::RANGEFINDER(r) => self.rangefinder = r.distance,
            MavMessage::WIND(w) => {
                self.wind_dir = (w.direction + 360.0) % 360.0;
                self.wind_vel = w.speed;
            }
            MavMessage::DISTANCE_SENSOR(d) => self.rangefinder = d.current_distance as f32 / 100.0,
            MavMessage::SCALED_PRESSURE(p) => self.press_temp = p.temperature as f32 / 100.0,
            MavMessage::STATUSTEXT(s) => {
                let sev = s.severity as u8;
                if sev <= 4 {
                    self.message_high = s
                        .text
                        .to_str()
                        .unwrap_or("")
                        .trim_end_matches('\0')
                        .to_string();
                    self.message_high_severity = sev;
                }
            }
            _ => {}
        }
    }
}
