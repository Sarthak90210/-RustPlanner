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
    pub ekf_velv: f32,
    pub ekf_posh: f32,
    pub ekf_posv: f32,
    pub ekf_compass: f32,
    pub ekf_terrain: f32,
    pub vibex: f32,
    pub vibey: f32,
    pub vibez: f32,
    pub clip0: u32,
    pub clip1: u32,
    pub clip2: u32,

    // misc
    pub rangefinder: f32,
    /// Set Home Alt: subtracted from alt (CurrentState.altoffsethome)
    pub altoffsethome: f32,
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub mx: f32,
    pub my: f32,
    pub mz: f32,
    pub wind_dir: f32,
    pub wind_vel: f32,
    pub press_temp: f32,
    pub home: Option<PointLatLngAlt>,
    pub time_in_air: f32,
    pub dist_traveled: f32,
    pub time_since_boot: f32,
    /// `_messagehigh`: read through `message_high()`, written through `set_message_high`
    message_high_text: String,
    /// `_messageHighTime`
    message_high_time: Option<Instant>,
    pub message_high_severity: u8,
    pub linkqualitygcs: f32,
    pub packets_received: u64,
    pub packets_lost: u64,

    pub track: Vec<(f64, f64)>,
    pub camerapoints: Vec<(f64, f64, f32)>,
    pub extra: std::collections::BTreeMap<String, f64>,
    pub version_string: String,
    pub uid2: String,

    last_pos_update: Option<Instant>,
    /// the alt setter's lastalt / oldalt and gotVFR
    last_alt: Option<Instant>,
    old_alt: f32,
    got_vfr: bool,
    last_seq: Option<u8>,
    rangefinder_alt_index: i32,
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
            ekf_velv: 0.0,
            ekf_posh: 0.0,
            ekf_posv: 0.0,
            ekf_compass: 0.0,
            ekf_terrain: 0.0,
            vibex: 0.0,
            vibey: 0.0,
            vibez: 0.0,
            clip0: 0,
            clip1: 0,
            clip2: 0,
            rangefinder: 0.0,
            altoffsethome: 0.0,
            ax: 0.0,
            ay: 0.0,
            az: 0.0,
            gx: 0.0,
            gy: 0.0,
            gz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
            wind_dir: 0.0,
            wind_vel: 0.0,
            press_temp: 0.0,
            home: None,
            time_in_air: 0.0,
            dist_traveled: 0.0,
            time_since_boot: 0.0,
            message_high_text: String::new(),
            message_high_time: None,
            message_high_severity: 7,
            linkqualitygcs: 0.0,
            packets_received: 0,
            packets_lost: 0,
            track: Vec::new(),
            camerapoints: Vec::new(),
            extra: default_extra(),
            version_string: String::new(),
            uid2: String::new(),
            last_pos_update: None,
            last_alt: None,
            old_alt: 0.0,
            got_vfr: false,
            last_seq: None,
            rangefinder_alt_index: i32::MAX,
        }
    }
}

/// `(byte)Settings.Instance.GetInt32("severity", 4)`: the highest STATUSTEXT severity shown
/// as `messageHigh`; the app keeps it in step with the setting.
pub static STATUSTEXT_SEVERITY: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(4);

#[path = "state_properties.rs"]
pub mod properties;

/// The [DisplayText] of a CurrentState property, untranslated ("Altitude (alt)").
pub fn display_text(name: &str) -> Option<&'static str> {
    properties::DISPLAY.iter().find(|(n, ..)| *n == name).and_then(|(_, t, _)| *t)
}

/// The [DisplayFieldName] translation key of a CurrentState property ("alt.Field").
pub fn field_key(name: &str) -> Option<&'static str> {
    properties::DISPLAY.iter().find(|(n, ..)| *n == name).and_then(|(_, _, k)| *k)
}
use properties::{PropertyKind, PROPERTIES, AUTO_DEFAULTS};

fn default_extra() -> std::collections::BTreeMap<String, f64> {
    let mut values: std::collections::BTreeMap<_, _> = AUTO_DEFAULTS.iter().map(|(name, value)| ((*name).into(), *value)).collect();
    // ResetInternals overrides the language default for voltageflag.
    values.insert("voltageflag".into(), 1.0);
    values
}

impl CurrentState {
    /// `messageHigh` getter: "" once 10 s have passed since it was set; characters outside
    /// 0x20..0x7E are removed (`TrimUnPrintable`).
    pub fn message_high(&self) -> String {
        match self.message_high_time {
            Some(t) if t.elapsed().as_secs_f64() <= 10.0 => self.message_high_text.chars().filter(|c| (' '..='~').contains(c)).collect(),
            _ => String::new(),
        }
    }

    /// `messageHigh` setter: ignores null/"" and the text already showing; otherwise stamps
    /// the time and sets the severity to EMERGENCY.
    pub fn set_message_high(&mut self, value: &str) {
        if value.is_empty() {
            return;
        }
        // check against get
        if self.message_high() == value {
            return;
        }
        log::info!("messageHigh {value}");
        self.message_high_time = Some(Instant::now());
        self.message_high_text = value.to_string();
        // MAVLink.MAV_SEVERITY.EMERGENCY
        self.message_high_severity = 0;
    }

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




    /// Public properties only, matching GetItemList's type filter and natural sort.
    pub fn get_item_list(&self, alpha: bool, numbers_only: bool) -> Vec<&'static str> {
        let mut names: Vec<_> = PROPERTIES.iter()
            .filter(|(_, kind)| !numbers_only || *kind == PropertyKind::Number)
            .map(|(name, _)| *name).collect();
        if alpha { names.sort_by(|a, b| property_order(a, b)); }
        names
    }

    /// Values keep their original types. Unported computed getters remain null.
    pub fn property(&self, name: &str) -> serde_json::Value {
        use serde_json::json;
        match name {
            "mode" => json!(self.mode),
            "armed" => json!(self.armed),
            "messageHigh" => json!(self.message_high()),
            "HomeLocation" => self.home.map(|h| json!({"Lat":h.lat,"Lng":h.lng,"Alt":h.alt})).unwrap_or(json!({"Lat":0,"Lng":0,"Alt":0})),
            _ => match self.field(name) {
                Some(v) if PROPERTIES.iter().any(|(n,k)| *n == name && *k == PropertyKind::Bool) => json!(v != 0.0),
                Some(v) => json!(v),
                None => serde_json::Value::Null,
            }
        }
    }

    /// Fast property lookup matching CurrentState properties.
    pub fn field(&self, name: &str) -> Option<f64> {
        let val = match name {
            "roll" => self.roll as f64,
            "pitch" => self.pitch as f64,
            "yaw" => self.yaw as f64,
            "rollspeed" => self.rollspeed as f64,
            "pitchspeed" => self.pitchspeed as f64,
            "yawspeed" => self.yawspeed as f64,
            "lat" => self.lat,
            "lng" => self.lng,
            "alt" => self.alt as f64,
            "altasl" => self.altasl as f64,
            "altoffsethome" => self.altoffsethome as f64,
            "groundcourse" => self.groundcourse as f64,
            "gpsstatus" => self.gpsstatus as f64,
            "satcount" => self.satcount as f64,
            "gpshdop" => self.gpshdop as f64,
            "gpsvdop" => self.gpsvdop as f64,
            "gpsalt" => self.gpsalt as f64,
            "airspeed" => self.airspeed as f64,
            "groundspeed" => self.groundspeed as f64,
            "verticalspeed" => self.verticalspeed as f64,
            "climbrate" => self.extra.get(name).copied().unwrap_or(self.verticalspeed as f64),
            "throttle" => self.throttle as f64,
            "nav_roll" => self.nav_roll as f64,
            "nav_pitch" => self.nav_pitch as f64,
            "nav_bearing" => self.nav_bearing as f64,
            "target_bearing" => self.target_bearing as f64,
            "wp_dist" => self.wp_dist as f64,
            "alt_error" => self.alt_error as f64,
            "aspd_error" => self.aspd_error as f64,
            "xtrack_error" => self.xtrack_error as f64,
            "wpno" => self.wpno as f64,
            "armed" => if self.armed { 1.0 } else { 0.0 },
            "battery_voltage" => self.battery_voltage as f64,
            "current" => self.current as f64,
            "battery_remaining" => self.battery_remaining as f64,
            "battery_usedmah" => self.battery_usedmah as f64,
            "load" => self.load as f64,
            "rssi" => self.rssi as f64,
            "remrssi" => self.remrssi as f64,
            "noise" => self.noise as f64,
            "remnoise" => self.remnoise as f64,
            "txbuf" | "txbuffer" => self.txbuf as f64,
            "rxerrors" => self.rxerrors as f64,
            "ekfstatus" => self.ekfstatus as f64,
            "ekfflags" => self.ekfflags as f64,
            "ekfvelv" | "ekf_velv" => self.ekf_velv as f64,
            "ekfposhor" | "ekf_posh" => self.ekf_posh as f64,
            "ekfposvert" | "ekf_posv" => self.ekf_posv as f64,
            "ekfcompv" | "ekf_compass" => self.ekf_compass as f64,
            "ekfteralt" | "ekf_terrain" => self.ekf_terrain as f64,
            "vibex" => self.vibex as f64,
            "vibey" => self.vibey as f64,
            "vibez" => self.vibez as f64,
            "clip0" | "vibeclip0" => self.clip0 as f64,
            "clip1" | "vibeclip1" => self.clip1 as f64,
            "clip2" | "vibeclip2" => self.clip2 as f64,
            "sonarrange" => self.rangefinder as f64,
            "ax" => self.ax as f64,
            "ay" => self.ay as f64,
            "az" => self.az as f64,
            "gx" => self.gx as f64,
            "gy" => self.gy as f64,
            "gz" => self.gz as f64,
            "mx" => self.mx as f64,
            "my" => self.my as f64,
            "mz" => self.mz as f64,
            "accelsq" => self.magnitude("ax", "ay", "az") / 1000.0,
            "accelsq2" => self.magnitude("ax2", "ay2", "az2") / 1000.0,
            "accelsq3" => self.magnitude("ax3", "ay3", "az3") / 1000.0,
            "gyrosq" => self.magnitude("gx", "gy", "gz"),
            "gyrosq2" => self.magnitude("gx2", "gy2", "gz2"),
            "gyrosq3" => self.magnitude("gx3", "gy3", "gz3"),
            "magfield" => self.magnitude("mx", "my", "mz"),
            "magfield2" => self.magnitude("mx2", "my2", "mz2"),
            "magfield3" => self.magnitude("mx3", "my3", "mz3"),
            "watts" => self.battery_voltage as f64 * self.current as f64,
            "vlen" => self.magnitude("vx", "vy", "vz"),
            "verticalspeed_fpm" => self.vz as f64 * -3.28084 * 60.0,
            "timeInAirMinSec" => (self.time_in_air / 60.0).trunc() as f64 + (self.time_in_air % 60.0) as f64 / 100.0,
            "wind_dir" => self.wind_dir as f64,
            "wind_vel" => self.wind_vel as f64,
            "press_temp" => self.press_temp as f64,
            "DistToHome" => self.DistToHome() as f64,
            "timeInAir" => self.time_in_air as f64,
            "distTraveled" => self.dist_traveled as f64,
            "timeSinceBoot" => self.time_since_boot as f64,
            "linkqualitygcs" => self.linkqualitygcs as f64,
            "packets_received" => self.packets_received as f64,
            "packets_lost" => self.packets_lost as f64,
            "sysid" => self.sysid as f64,
            "compid" => self.compid as f64,
            "HomeAlt" => self.home.map(|h| h.alt as f64).unwrap_or(0.0),
            "vx" => self.vx as f64,
            "vy" => self.vy as f64,
            "vz" => self.vz as f64,
            "custom_mode" => self.custom_mode as f64,
            "base_mode" => self.base_mode as f64,
            "system_status" => self.system_status as f64,
            "capabilities" => return self.extra.get(name).copied(),
            "accelerometer" => if self.sensors_health & 1 != 0 { 1.0 } else { 0.0 },
            "gyro" => if self.sensors_health & 2 != 0 { 1.0 } else { 0.0 },
            "compass" => if self.sensors_health & 4 != 0 { 1.0 } else { 0.0 },
            "barometer" => if self.sensors_health & 8 != 0 { 1.0 } else { 0.0 },
            "gps" => if self.gpsstatus >= 3 { 1.0 } else { 0.0 },
            "battery" => if self.sensors_health & (1 << 10) != 0 { 1.0 } else { 0.0 },
            "failsafe" => self.extra.get(name).copied().unwrap_or(if self.system_status == 5 { 1.0 } else { 0.0 }),
            s if s.starts_with("ch") && s.ends_with("in") => {
                if let Ok(idx) = s[2..s.len() - 2].parse::<usize>() {
                    if idx >= 1 && idx <= 18 {
                        return Some(self.ch_in[idx - 1] as f64);
                    }
                }
                return self.extra.get(name).copied();
            }
            s if s.starts_with("ch") && s.ends_with("out") => {
                if let Ok(idx) = s[2..s.len() - 3].parse::<usize>() {
                    if idx >= 1 && idx <= 16 {
                        return Some(self.ch_out[idx - 1] as f64);
                    }
                }
                return self.extra.get(name).copied();
            }
            _ => return self.extra.get(name).copied(),
        };
        Some(val)
    }

    /// Numeric and boolean properties supported by the telemetry decoder.
    /// Text and object properties belong in Status/JSON, not numeric plots.
    pub fn fields(&self) -> Vec<(&'static str, f64)> {
        self.get_item_list(true, false).into_iter()
            .filter(|name| PROPERTIES.iter().any(|(n,k)| n == name && matches!(k, PropertyKind::Number | PropertyKind::Bool)))
            .filter_map(|name| self.field(name).map(|value| (name, value))).collect()
    }

    /// The alt setter: verticalspeed (and climbrate before any VFR_HUD) from altitude changes
    /// sampled at least 0.2 s apart.
    fn set_alt(&mut self, alt: f32, now: Instant) {
        self.alt = alt;
        let elapsed = self.last_alt.map(|t| now.saturating_duration_since(t).as_secs_f32());
        if elapsed.is_none_or(|e| e >= 0.2) && self.old_alt != alt {
            // the first sample divides by the time since DateTime.MinValue: zero
            let mut vs = match elapsed {
                Some(e) if e > 0.0 => (alt - self.old_alt) / e,
                _ => 0.0,
            };
            if vs.is_infinite() {
                vs = 0.0;
            }
            if !self.got_vfr {
                self.extra.insert("climbrate".into(), vs as f64);
            }
            self.verticalspeed = vs;
            self.last_alt = Some(now);
            self.old_alt = alt;
        }
    }

    fn magnitude(&self, x: &str, y: &str, z: &str) -> f64 {
        [x, y, z].into_iter().map(|name| self.field(name).unwrap_or(0.0).powi(2)).sum::<f64>().sqrt()
    }

    fn high_latency_mode(&mut self, mode: u32) {
        if self.firmware.modes().iter().any(|(number, _)| *number == mode) {
            self.custom_mode = mode;
            self.mode = self.firmware.mode_name(mode);
        }
    }

    /// Port of `CurrentState.UpdateCurrentSettings` message handling.
    fn esc(&mut self, first: usize, voltage: &[u16; 4], current: &[u16; 4], rpm: &[u16; 4], temperature: &[u8; 4]) {
        for i in 0..4 {
            for (suffix, value) in [("volt", voltage[i] as f64 / 100.0), ("curr", current[i] as f64 / 100.0), ("rpm", rpm[i] as f64), ("temp", temperature[i] as f64)] {
                self.extra.insert(format!("esc{}_{suffix}", first + i), value);
            }
        }
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
                self.extra.insert("failsafe".into(), f64::from(self.system_status == 5));
                self.mode = self.firmware.mode_name(hb.custom_mode);
            }
            MavMessage::HIGH_LATENCY(h) => {
                self.extra.insert("landed".into(), f64::from(h.landed_state as u8 == 1));
                if h.base_mode.contains(MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) { self.high_latency_mode(h.custom_mode); }
                self.roll = h.roll as f32 / 100.0;
                self.pitch = h.pitch as f32 / 100.0;
                self.yaw = h.heading as f32 / 100.0;
                self.extra.insert("ch3percent".into(), h.throttle as f64);
                self.lat = h.latitude as f64 / 1e7;
                self.lng = h.longitude as f64 / 1e7;
                self.altasl = h.altitude_amsl as f32;
                self.set_alt(self.altasl - self.home.map(|p| p.alt as f32).unwrap_or(0.0), Instant::now());
                self.alt_error = h.altitude_sp as f32 - self.alt;
                let old = self.extra.get("targetalt").copied().unwrap_or(0.0);
                self.extra.insert("targetalt".into(), old * 0.5 + ((self.alt + self.alt_error).round_ties_even() as f64) * 0.5);
                self.airspeed = h.airspeed as f32;
                self.extra.insert("targetairspeed".into(), h.airspeed_sp as f64);
                self.groundspeed = h.groundspeed as f32;
                self.extra.insert("climbrate".into(), h.climb_rate as f64);
                self.satcount = h.gps_nsat;
                self.gpsstatus = h.gps_fix_type as u8;
                self.battery_remaining = h.battery_remaining as i32;
                self.press_temp = h.temperature as f32;
                self.extra.insert("airspeed1_temp".into(), h.temperature_air as f64);
                self.extra.insert("failsafe".into(), f64::from(h.failsafe > 0));
                self.wpno = h.wp_num as u16;
                self.wp_dist = h.wp_distance as f32;
            }
            MavMessage::HIGH_LATENCY2(h) => {
                self.firmware = Firmware::from_heartbeat(h.autopilot, h.mavtype);
                self.mavtype = h.mavtype as u32;
                self.high_latency_mode(h.custom_mode as u32);
                self.lat = h.latitude as f64 / 1e7;
                self.lng = h.longitude as f64 / 1e7;
                self.altasl = h.altitude as f32;
                self.set_alt(self.altasl - self.home.map(|p| p.alt as f32).unwrap_or(0.0), Instant::now());
                self.alt_error = h.target_altitude as f32 - self.alt;
                self.extra.insert("targetalt".into(), h.target_altitude as f64);
                self.wp_dist = h.target_distance as f32 * 10.0;
                self.wpno = h.wp_num;
                // Preserve CurrentState's currentbit == 1 check: only GPS adds text.
                if h.failure_flags.bits() & 1 == 1 { self.set_message_high("ERROR GPS "); }
                self.yaw = h.heading as f32 * 2.0;
                self.target_bearing = h.target_heading as f32 * 2.0;
                self.extra.insert("ch3percent".into(), h.throttle as f64);
                self.airspeed = h.airspeed as f32 / 5.0;
                self.extra.insert("targetairspeed".into(), h.airspeed_sp as f64 / 5.0);
                self.groundspeed = h.groundspeed as f32 / 5.0;
                self.wind_vel = h.windspeed as f32 / 5.0;
                self.wind_dir = h.wind_heading as f32 * 2.0;
                self.gpshdop = h.eph as f32;
                self.extra.insert("airspeed1_temp".into(), h.temperature_air as f64);
                // This is the C# display factor, including its existing unit discrepancy.
                self.extra.insert("climbrate".into(), h.climb_rate as f64 * 10.0);
                self.battery_remaining = h.battery as i32;
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
                self.set_alt(g.relative_alt as f32 / 1000.0 - self.altoffsethome, Instant::now());
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
                self.throttle = v.throttle as f32;
                // ch3percent = vfr.throttle, negative with reverse thrust present/enabled/healthy
                const REVERSE_MOTOR: u32 = 1 << 23;
                let reverse = self.sensors_present & self.sensors_enabled & self.sensors_health & REVERSE_MOTOR != 0;
                let ch3 = v.throttle as f64;
                self.extra.insert("ch3percent".into(), if reverse && ch3 > 0.0 { -ch3 } else { ch3 });
                // "This comes from the EKF": climbrate, while verticalspeed stays the alt setter's
                self.extra.insert("climbrate".into(), v.climb as f64);
                self.got_vfr = true;
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
                self.ekf_velv = e.velocity_variance;
                self.ekf_posh = e.pos_horiz_variance;
                self.ekf_posv = e.pos_vert_variance;
                self.ekf_compass = e.compass_variance;
                self.ekf_terrain = e.terrain_alt_variance;
                self.ekfstatus = e
                    .velocity_variance
                    .max(e.pos_horiz_variance)
                    .max(e.pos_vert_variance)
                    .max(e.compass_variance)
                    .max(e.terrain_alt_variance);
            }
            MavMessage::BATTERY_STATUS(b) if b.id < 9 => {
                let volts = b.voltages.iter().chain(&b.voltages_ext).filter(|v| **v != u16::MAX).map(|v| *v as f64 / 1000.0).sum::<f64>();
                let suffix = if b.id == 0 { String::new() } else { (b.id + 1).to_string() };
                for (name, value) in [
                    ("battery_voltage", volts), ("current", b.current_battery as f64 / 100.0),
                    ("battery_usedmah", b.current_consumed as f64), ("battery_remaining", b.battery_remaining as f64),
                    ("battery_remainmin", b.time_remaining as f64 / 60.0),
                ] { self.extra.insert(format!("{name}{suffix}"), value); }
                if b.temperature != i16::MAX { self.extra.insert(format!("battery_temp{suffix}"), b.temperature as f64 / 100.0); }
                if b.id == 0 {
                    self.battery_voltage = volts as f32;
                    self.current = b.current_battery as f32 / 100.0;
                    self.battery_usedmah = b.current_consumed as f32;
                    self.battery_remaining = b.battery_remaining as i32;
                    if b.voltages[0] != u16::MAX {
                        for (i, v) in b.voltages.iter().chain(&b.voltages_ext).enumerate() {
                            self.extra.insert(format!("battery_cell{}", i + 1), if *v == u16::MAX { 0.0 } else { *v as f64 / 1000.0 });
                        }
                    }
                }
            }
            MavMessage::RANGEFINDER(r) => {
                self.rangefinder = r.distance;
                self.extra.insert("sonarvoltage".into(), r.voltage as f64);
                self.rangefinder_alt_index = -1;
            }
            MavMessage::RAW_IMU(r) => {
                self.ax = r.xacc as f32;
                self.ay = r.yacc as f32;
                self.az = r.zacc as f32;
                self.gx = r.xgyro as f32;
                self.gy = r.ygyro as f32;
                self.gz = r.zgyro as f32;
                self.mx = r.xmag as f32;
                self.my = r.ymag as f32;
                self.mz = r.zmag as f32;
            }
            MavMessage::WIND(w) => {
                self.wind_dir = (w.direction + 360.0) % 360.0;
                self.wind_vel = w.speed;
            }
            MavMessage::POWER_STATUS(p) => {
                self.extra.insert("boardvoltage".into(), p.Vcc as f64);
                self.extra.insert("servovoltage".into(), p.Vservo as f64);
                self.extra.insert("voltageflag".into(), p.flags.bits() as f64);
            }
            MavMessage::GENERATOR_STATUS(g) => {
                for (n,v) in [("gen_status", g.status.bits() as f64), ("gen_speed", g.generator_speed as f64), ("gen_current", g.load_current as f64), ("gen_voltage", g.bus_voltage as f64), ("gen_runtime", g.runtime as f64), ("gen_maint_time", g.time_until_maintenance as f64)] { self.extra.insert(n.into(), v); }
            }
            MavMessage::HWSTATUS(h) => {
                self.extra.insert("hwvoltage".into(), h.Vcc as f64 / 1000.0);
                self.extra.insert("i2cerrors".into(), h.I2Cerr as f64);
            }
            MavMessage::MCU_STATUS(m) if m.id == 0 => {
                for (n,v) in [("mcutemp", m.MCU_temperature as f64 / 100.0), ("mcuvoltage", m.MCU_voltage as f64 / 1000.0), ("mcuminvolt", m.MCU_voltage_min as f64 / 1000.0), ("mcumaxvolt", m.MCU_voltage_max as f64 / 1000.0)] { self.extra.insert(n.into(), v); }
            }
            MavMessage::MEMINFO(m) => {
                self.extra.insert("freemem".into(), if m.freemem32 > 0 { m.freemem32 as f64 } else { m.freemem as f64 });
                self.extra.insert("brklevel".into(), m.brkval as f64);
            }
            MavMessage::RPM(r) => {
                self.extra.insert("rpm1".into(), r.rpm1 as f64);
                self.extra.insert("rpm2".into(), r.rpm2 as f64);
            }
            MavMessage::AOA_SSA(a) => {
                self.extra.insert("AOA".into(), a.AOA as f64);
                self.extra.insert("SSA".into(), a.SSA as f64);
            }
            MavMessage::FENCE_STATUS(f) => {
                self.extra.insert("fenceb_count".into(), f.breach_count as f64);
                self.extra.insert("fenceb_status".into(), f.breach_status as f64);
                self.extra.insert("fenceb_type".into(), f.breach_type as u8 as f64);
                if f.breach_status != 0 { self.set_message_high(&format!("Fence Breach {}", format!("{:?}", f.breach_type).trim_start_matches("FENCE_BREACH_"))); }
            }
            MavMessage::EFI_STATUS(e) if e.ecu_index == 0.0 => {
                for (n,v) in [("efi_baro", e.barometric_pressure), ("efi_headtemp", e.cylinder_head_temperature), ("efi_load", e.engine_load), ("efi_health", e.health as f32), ("efi_exhasttemp", e.exhaust_gas_temperature), ("efi_intaketemp", e.intake_manifold_temperature), ("efi_rpm", e.rpm), ("efi_fuelflow", e.fuel_flow), ("efi_fuelconsumed", e.fuel_consumed), ("efi_fuelpressure", if e.fuel_pressure == 0.0 { -1.0 } else { e.fuel_pressure })] { self.extra.insert(n.into(), v as f64); }
            }
            MavMessage::ESC_TELEMETRY_1_TO_4(e) => self.esc(1, &e.voltage, &e.current, &e.rpm, &e.temperature),
            MavMessage::ESC_TELEMETRY_5_TO_8(e) => self.esc(5, &e.voltage, &e.current, &e.rpm, &e.temperature),
            MavMessage::ESC_TELEMETRY_9_TO_12(e) => self.esc(9, &e.voltage, &e.current, &e.rpm, &e.temperature),
            MavMessage::ESC_TELEMETRY_13_TO_16(e) => self.esc(13, &e.voltage, &e.current, &e.rpm, &e.temperature),
            MavMessage::AUTOPILOT_VERSION(v) => { self.extra.insert("capabilities".into(), v.capabilities.bits() as f64); }
            MavMessage::LOCAL_POSITION_NED(p) => {
                self.extra.insert("posn".into(), p.x as f64);
                self.extra.insert("pose".into(), p.y as f64);
                self.extra.insert("posd".into(), p.z as f64);
            }
            MavMessage::PID_TUNING(p) => {
                for (name, val) in [("pidaxis", p.axis as u8 as f64), ("piddesired", p.desired as f64), ("pidachieved", p.achieved as f64), ("pidff", p.FF as f64), ("pidP", p.P as f64), ("pidI", p.I as f64), ("pidD", p.D as f64), ("pidSRate", p.SRate as f64), ("pidPDmod", p.PDmod as f64)] {
                    self.extra.insert(name.into(), val);
                }
            }
            MavMessage::DISTANCE_SENSOR(d) => {
                if d.id < 10 { self.extra.insert(format!("rangefinder{}", d.id + 1), d.current_distance as f64); }
                let down = d.orientation == MavSensorOrientation::MAV_SENSOR_ROTATION_PITCH_270;
                if down && (d.id as i32) < self.rangefinder_alt_index { self.rangefinder_alt_index = d.id as i32; }
                if d.id as i32 == self.rangefinder_alt_index {
                    if down { self.rangefinder = (d.current_distance / 100) as f32; }
                    else { self.rangefinder_alt_index = i32::MAX; }
                }
            }
            MavMessage::SCALED_PRESSURE(p) => {
                self.press_temp = p.temperature as f32;
                self.extra.insert("press_abs".into(), p.press_abs as f64);
                self.extra.insert("airspeed1_temp".into(), p.temperature_press_diff as f64);
            }
            MavMessage::SCALED_PRESSURE2(p) => {
                self.extra.insert("press_abs2".into(), p.press_abs as f64);
                self.extra.insert("press_temp2".into(), p.temperature as f64);
                self.extra.insert("airspeed2_temp".into(), p.temperature_press_diff as f64);
            }
            MavMessage::AHRS2(a) => {
                self.extra.insert("ahrs2_lat".into(), a.lat as f64 / 1e7);
                self.extra.insert("ahrs2_lng".into(), a.lng as f64 / 1e7);
                self.extra.insert("ahrs2_alt".into(), a.altitude as f64);
                self.extra.insert("ahrs2_roll".into(), a.roll.to_degrees() as f64);
                self.extra.insert("ahrs2_pitch".into(), a.pitch.to_degrees() as f64);
                self.extra.insert("ahrs2_yaw".into(), (a.yaw.to_degrees() as f64 + 360.0) % 360.0);
            }
            MavMessage::SCALED_IMU2(s) => {
                self.extra.insert("ax2".into(), s.xacc as f64);
                self.extra.insert("ay2".into(), s.yacc as f64);
                self.extra.insert("az2".into(), s.zacc as f64);
                self.extra.insert("gx2".into(), s.xgyro as f64);
                self.extra.insert("gy2".into(), s.ygyro as f64);
                self.extra.insert("gz2".into(), s.zgyro as f64);
                self.extra.insert("mx2".into(), s.xmag as f64);
                self.extra.insert("my2".into(), s.ymag as f64);
                self.extra.insert("mz2".into(), s.zmag as f64);
                self.extra.insert("imu2_temp".into(), s.temperature as f64 / 100.0);
            }
            MavMessage::SCALED_IMU3(s) => {
                self.extra.insert("ax3".into(), s.xacc as f64);
                self.extra.insert("ay3".into(), s.yacc as f64);
                self.extra.insert("az3".into(), s.zacc as f64);
                self.extra.insert("gx3".into(), s.xgyro as f64);
                self.extra.insert("gy3".into(), s.ygyro as f64);
                self.extra.insert("gz3".into(), s.zgyro as f64);
                self.extra.insert("mx3".into(), s.xmag as f64);
                self.extra.insert("my3".into(), s.ymag as f64);
                self.extra.insert("mz3".into(), s.zmag as f64);
                self.extra.insert("imu3_temp".into(), s.temperature as f64 / 100.0);
            }
            MavMessage::GPS2_RAW(g) => {
                self.extra.insert("gpsstatus2".into(), g.fix_type as u8 as f64);
                self.extra.insert("satcount2".into(), g.satellites_visible as f64);
                self.extra.insert("gpshdop2".into(), g.eph as f64 / 100.0);
                self.extra.insert("altasl2".into(), g.alt as f64 / 1000.0);
                self.extra.insert("groundspeed2".into(), g.vel as f64 / 100.0);
                self.extra.insert("groundcourse2".into(), g.cog as f64 / 100.0);
                self.extra.insert("lat2".into(), g.lat as f64 / 1e7);
                self.extra.insert("lng2".into(), g.lon as f64 / 1e7);
            }
            MavMessage::TERRAIN_REPORT(t) => {
                self.extra.insert("ter_alt".into(), t.terrain_height as f64);
                self.extra.insert("ter_curalt".into(), t.current_height as f64);
                self.extra.insert("ter_space".into(), t.spacing as f64);
                self.extra.insert("ter_load".into(), t.loaded as f64);
                self.extra.insert("ter_pend".into(), t.pending as f64);
            }
            MavMessage::OPTICAL_FLOW(o) => {
                self.extra.insert("opt_x".into(), o.flow_x as f64);
                self.extra.insert("opt_y".into(), o.flow_y as f64);
                self.extra.insert("opt_qua".into(), o.quality as f64);
                self.extra.insert("opt_m_x".into(), o.flow_comp_m_x as f64);
                self.extra.insert("opt_m_y".into(), o.flow_comp_m_y as f64);
            }
            MavMessage::STATUSTEXT(s) => {
                // MAVLinkInterface: printit when sev <= (byte)Settings.GetInt32("severity", 4)
                // (mavsev = sev) or for Tuning:/PreArm:/Arm: text (mavsev stays EMERGENCY)
                let sev = s.severity as u8;
                let text = s.text.to_str().unwrap_or("");
                let text = match text.find('\0') {
                    Some(i) => &text[..i],
                    None => text,
                };
                let mut printit = false;
                let mut mavsev = 0u8;
                if sev <= STATUSTEXT_SEVERITY.load(std::sync::atomic::Ordering::Relaxed) {
                    printit = true;
                    mavsev = sev;
                }
                if text.starts_with("Tuning:") || text.starts_with("PreArm:") || text.starts_with("Arm:") {
                    printit = true;
                }
                if printit {
                    self.set_message_high(text);
                    self.message_high_severity = mavsev;
                }
            }
            MavMessage::CAMERA_FEEDBACK(c) => {
                let lat = c.lat as f64 * 1e-7;
                let lng = c.lng as f64 * 1e-7;
                let alt = c.alt_msl;
                self.camerapoints.push((lat, lng, alt));
            }
            _ => {}
        }
        // DATA's route consumes position from high-latency messages too.
        if (self.lat != 0.0 || self.lng != 0.0) && self.track.last().map(|&(a,b)| (a-self.lat).abs() > 1e-6 || (b-self.lng).abs() > 1e-6).unwrap_or(true) {
            self.track.push((self.lat, self.lng));
            if self.track.len() > 5000 { self.track.drain(0..1000); }
        }
    }
}

/// Port of CurrentState.StringCompareTo, including natural channel/cell ordering.
fn property_order(a: &str, b: &str) -> std::cmp::Ordering {
    let (aa, bb) = (a.as_bytes(), b.as_bytes());
    let (mut i, mut j) = (0, 0);
    while i < aa.len() && j < bb.len() {
        let order = if aa[i].is_ascii_digit() && bb[j].is_ascii_digit() {
            let (mut av, mut bv) = (0u64, 0u64);
            while i < aa.len() && aa[i].is_ascii_digit() { av = av * 10 + (aa[i] - b'0') as u64; i += 1; }
            while j < bb.len() && bb[j].is_ascii_digit() { bv = bv * 10 + (bb[j] - b'0') as u64; j += 1; }
            av.cmp(&bv)
        } else {
            let order = aa[i].to_ascii_lowercase().cmp(&bb[j].to_ascii_lowercase());
            i += 1; j += 1; order
        };
        if !order.is_eq() { return order; }
    }
    aa.len().cmp(&bb.len())
}

#[cfg(test)]
mod property_tests {
    use super::*;
    #[test]
    fn imu_magnitudes_have_source_defaults_and_follow_each_sensor() {
        let mut cs = CurrentState::default();
        for name in ["accelsq", "accelsq2", "accelsq3", "gyrosq", "gyrosq2", "gyrosq3", "magfield", "magfield2", "magfield3"] { assert_eq!(cs.field(name), Some(0.0)); }
        cs.update(&MavMessage::RAW_IMU(RAW_IMU_DATA { xacc: 3000, yacc: 4000, xgyro: 3, ygyro: 4, xmag: 300, ymag: 400, ..Default::default() }));
        cs.update(&MavMessage::SCALED_IMU2(SCALED_IMU2_DATA { xacc: -6000, yacc: 8000, xgyro: 6, ygyro: 8, xmag: 600, ymag: 800, ..Default::default() }));
        cs.update(&MavMessage::SCALED_IMU3(SCALED_IMU3_DATA { xacc: 5000, yacc: 12000, xgyro: 5, ygyro: 12, xmag: 500, ymag: 1200, ..Default::default() }));
        for (name, value) in [("accelsq",5.0),("accelsq2",10.0),("accelsq3",13.0),("gyrosq",5.0),("gyrosq2",10.0),("gyrosq3",13.0),("magfield",500.0),("magfield2",1000.0),("magfield3",1300.0)] {
            assert_eq!(cs.field(name), Some(value)); assert_eq!(cs.property(name), serde_json::json!(value));
            assert!(cs.fields().iter().any(|(n,v)| *n == name && *v == value));
        }
        // Source auto-properties remain selectable even before their first packet.
        assert!(cs.fields().iter().any(|(n,v)| *n == "gen_speed" && *v == 0.0));
    }

    #[test]
    fn legacy_high_latency_matches_current_state_conversions_and_mode_gate() {
        let mut cs = CurrentState { firmware: Firmware::ArduPlane, home: Some(PointLatLngAlt::new(-35.0,149.0,500.0)), ..Default::default() };
        let h = HIGH_LATENCY_DATA { base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode: 15, landed_state: MavLandedState::MAV_LANDED_STATE_ON_GROUND, roll: -1234, pitch: 345, heading: 12345, throttle: 65, latitude: -351000000, longitude: 1492000000, altitude_amsl: 600, altitude_sp: 120, airspeed: 25, airspeed_sp: 30, groundspeed: 20, climb_rate: -2, gps_nsat: 12, gps_fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX, battery_remaining: 75, temperature: 24, temperature_air: 26, failsafe: 1, wp_num: 8, wp_distance: 400, ..Default::default() };
        cs.update(&MavMessage::HIGH_LATENCY(h.clone()));
        assert_eq!(cs.mode, "Guided"); assert_eq!((cs.altasl,cs.alt,cs.alt_error), (600.0,100.0,20.0));
        assert!((cs.roll + 12.34).abs() < 0.001); assert_eq!(cs.field("ch3percent"), Some(65.0));
        assert_eq!(cs.field("climbrate"), Some(-2.0)); assert_eq!(cs.field("targetairspeed"), Some(30.0));
        assert_eq!(cs.field("targetalt"), Some(60.0)); assert_eq!(cs.property("landed"), true);
        assert_eq!(cs.property("failsafe"), true); assert_eq!(cs.track, [(-35.1,149.2)]);
        let mut no_mode = h; no_mode.custom_mode = 10; no_mode.base_mode = MavModeFlag::empty();
        cs.update(&MavMessage::HIGH_LATENCY(no_mode)); assert_eq!(cs.mode, "Guided");
    }

    #[test]
    fn high_latency2_preserves_source_scales_and_unknown_modes() {
        let mut cs = CurrentState { home: Some(PointLatLngAlt::new(-35.0,149.0,500.0)), ..Default::default() };
        let mut h = HIGH_LATENCY2_DATA { mavtype: MavType::MAV_TYPE_FIXED_WING, autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA, custom_mode: 15, latitude: -351000000, longitude: 1492000000, altitude: 600, target_altitude: 120, target_distance: 40, wp_num: 8, heading: 90, target_heading: 100, throttle: 65, airspeed: 125, airspeed_sp: 150, groundspeed: 100, windspeed: 20, wind_heading: 45, eph: 7, temperature_air: 26, climb_rate: -2, battery: 75, ..Default::default() };
        cs.update(&MavMessage::HIGH_LATENCY2(h.clone()));
        assert_eq!(cs.firmware, Firmware::ArduPlane); assert_eq!(cs.mode, "Guided");
        assert_eq!((cs.alt,cs.alt_error,cs.wp_dist), (100.0,20.0,400.0));
        assert_eq!((cs.yaw,cs.target_bearing,cs.airspeed,cs.groundspeed,cs.wind_vel,cs.wind_dir), (180.0,200.0,25.0,20.0,4.0,90.0));
        assert_eq!(cs.field("targetalt"), Some(120.0)); assert_eq!(cs.field("targetairspeed"), Some(30.0));
        assert_eq!(cs.field("climbrate"), Some(-20.0)); // source's factor is retained
        h.custom_mode = 999; cs.update(&MavMessage::HIGH_LATENCY2(h)); assert_eq!(cs.mode, "Guided");
    }
    #[test]
    fn engine_hardware_and_all_sixteen_esc_values_follow_reference_scaling() {
        let mut cs = CurrentState::default();
        cs.update(&MavMessage::HWSTATUS(HWSTATUS_DATA { Vcc: 5123, I2Cerr: 7 }));
        assert_eq!(cs.field("hwvoltage"), Some(5.123));
        cs.update(&MavMessage::MCU_STATUS(MCU_STATUS_DATA { id: 0, MCU_temperature: 3650, MCU_voltage: 3300, MCU_voltage_min: 3200, MCU_voltage_max: 3400 }));
        assert_eq!(cs.field("mcutemp"), Some(36.5));
        assert_eq!(cs.field("mcuminvolt"), Some(3.2));
        cs.update(&MavMessage::MCU_STATUS(MCU_STATUS_DATA { id: 1, ..Default::default() }));
        assert_eq!(cs.field("mcutemp"), Some(36.5));
        cs.update(&MavMessage::MEMINFO(MEMINFO_DATA { brkval: 12, freemem: 32, freemem32: 70000 }));
        assert_eq!(cs.field("freemem"), Some(70000.0));
        cs.update(&MavMessage::MEMINFO(MEMINFO_DATA { freemem: 32, ..Default::default() }));
        assert_eq!(cs.field("freemem"), Some(32.0));
        cs.update(&MavMessage::GENERATOR_STATUS(GENERATOR_STATUS_DATA { generator_speed: 2400, load_current: 12.5, bus_voltage: 48.0, runtime: 12345, time_until_maintenance: -60, ..Default::default() }));
        assert_eq!(cs.field("gen_speed"), Some(2400.0));
        assert_eq!(cs.field("gen_maint_time"), Some(-60.0));
        cs.update(&MavMessage::EFI_STATUS(EFI_STATUS_DATA { ecu_index: 0.0, rpm: 3000.0, fuel_pressure: 0.0, ..Default::default() }));
        assert_eq!(cs.field("efi_fuelpressure"), Some(-1.0));
        cs.update(&MavMessage::EFI_STATUS(EFI_STATUS_DATA { ecu_index: 1.0, ..Default::default() }));
        assert_eq!(cs.field("efi_rpm"), Some(3000.0));
        let voltage = [1200, 1210, 1220, 1230];
        let current = [100, 200, 300, 400];
        let rpm = [1000, 2000, 3000, 4000];
        let temperature = [30, 31, 32, 33];
        for msg in [
            MavMessage::ESC_TELEMETRY_1_TO_4(ESC_TELEMETRY_1_TO_4_DATA { voltage, current, rpm, temperature, ..Default::default() }),
            MavMessage::ESC_TELEMETRY_5_TO_8(ESC_TELEMETRY_5_TO_8_DATA { voltage, current, rpm, temperature, ..Default::default() }),
            MavMessage::ESC_TELEMETRY_9_TO_12(ESC_TELEMETRY_9_TO_12_DATA { voltage, current, rpm, temperature, ..Default::default() }),
            MavMessage::ESC_TELEMETRY_13_TO_16(ESC_TELEMETRY_13_TO_16_DATA { voltage, current, rpm, temperature, ..Default::default() }),
        ] { cs.update(&msg); }
        for n in 1..=16 {
            let i = (n - 1) % 4;
            for (suffix, value) in [("volt", voltage[i] as f64 / 100.0), ("curr", current[i] as f64 / 100.0), ("rpm", rpm[i] as f64), ("temp", temperature[i] as f64)] {
                let name = format!("esc{n}_{suffix}");
                assert_eq!(cs.field(&name), Some(value));
                assert!(cs.fields().iter().any(|(k,v)| *k == name && *v == value));
                assert_eq!(cs.property(&name), serde_json::json!(value));
            }
        }
    }
    #[test]
    fn registry_filters_types_sorts_naturally_and_preserves_values() {
        let cs = CurrentState { mode: "Guided".into(), armed: true, ..Default::default() };
        let names = cs.get_item_list(true, false);
        assert!(names.iter().position(|n| *n == "ch2in").unwrap() < names.iter().position(|n| *n == "ch10in").unwrap());
        assert!(!names.contains(&"accelerometer")); // nested sensor helper, not a CurrentState property
        assert!(!names.contains(&"Mavlink_Sensors"));
        let numbers = cs.get_item_list(false, true);
        assert!(!numbers.contains(&"mode") && !numbers.contains(&"armed") && !numbers.contains(&"HomeLocation"));
        assert_eq!(cs.property("mode"), "Guided");
        assert_eq!(cs.property("armed"), true);
        assert_eq!(cs.property("gen_speed"), 0.0);
        assert_eq!(cs.field("gen_speed"), Some(0.0));
        assert_eq!(cs.field("voltageflag"), Some(1.0));
        assert!(cs.property("TrackerLocation").is_null());
    }
    #[test]
    fn sensor_and_battery_packets_use_original_names_units_and_precedence() {
        let mut cs = CurrentState::default();
        cs.update(&MavMessage::DISTANCE_SENSOR(DISTANCE_SENSOR_DATA { id: 1, current_distance: 367, orientation: MavSensorOrientation::MAV_SENSOR_ROTATION_PITCH_270, ..Default::default() }));
        assert_eq!(cs.field("rangefinder2"), Some(367.0));
        assert_eq!(cs.field("sonarrange"), Some(3.0));
        cs.update(&MavMessage::RANGEFINDER(RANGEFINDER_DATA { distance: 4.5, voltage: 2.0 }));
        cs.update(&MavMessage::DISTANCE_SENSOR(DISTANCE_SENSOR_DATA { current_distance: 900, orientation: MavSensorOrientation::MAV_SENSOR_ROTATION_PITCH_270, ..Default::default() }));
        assert_eq!(cs.field("sonarrange"), Some(4.5));
        cs.update(&MavMessage::SCALED_PRESSURE2(SCALED_PRESSURE2_DATA { temperature: 2345, press_abs: 1013.0, ..Default::default() }));
        assert_eq!(cs.field("press_temp2"), Some(2345.0));
        cs.update(&MavMessage::TERRAIN_REPORT(TERRAIN_REPORT_DATA { spacing: 100, ..Default::default() }));
        assert_eq!(cs.field("ter_space"), Some(100.0));
        let mut volts = [u16::MAX; 10]; volts[0] = 4000; volts[1] = 4100;
        cs.update(&MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA { id: 1, voltages: volts, voltages_ext: [u16::MAX; 4], current_battery: 1234, temperature: 2500, ..Default::default() }));
        assert!((cs.field("battery_voltage2").unwrap() - 8.1).abs() < 0.001);
        assert_eq!(cs.field("current2"), Some(12.34));
        assert_eq!(cs.field("battery_temp2"), Some(25.0));
        assert_eq!(cs.field("capabilities"), Some(0.0));
    }

    #[test]
    fn verticalspeed_comes_from_altitude_changes_and_vfr_only_sets_climbrate() {
        let mut cs = CurrentState::default();
        let t0 = Instant::now();
        cs.set_alt(10.0, t0);
        assert_eq!(cs.verticalspeed, 0.0, "first sample");
        cs.set_alt(11.0, t0 + std::time::Duration::from_millis(100));
        assert_eq!(cs.verticalspeed, 0.0, "under 0.2 s");
        cs.set_alt(12.0, t0 + std::time::Duration::from_millis(500));
        assert!((cs.verticalspeed - 4.0).abs() < 1e-3, "{}", cs.verticalspeed);
        assert!((cs.field("climbrate").unwrap() - 4.0).abs() < 1e-3);
        cs.update(&MavMessage::VFR_HUD(VFR_HUD_DATA { climb: -1.5, throttle: 40, ..Default::default() }));
        assert_eq!(cs.field("climbrate"), Some(-1.5));
        assert!((cs.verticalspeed - 4.0).abs() < 1e-3, "VFR climb does not set verticalspeed");
        assert_eq!(cs.field("ch3percent"), Some(40.0));
        cs.set_alt(14.0, t0 + std::time::Duration::from_millis(1500));
        assert_eq!(cs.field("climbrate"), Some(-1.5), "gotVFR keeps climbrate");
        assert!((cs.verticalspeed - 2.0).abs() < 1e-3);
        cs.sensors_present = 1 << 23;
        cs.sensors_enabled = 1 << 23;
        cs.sensors_health = 1 << 23;
        cs.update(&MavMessage::VFR_HUD(VFR_HUD_DATA { throttle: 30, ..Default::default() }));
        assert_eq!(cs.field("ch3percent"), Some(-30.0), "reverse thrust");
    }
}
