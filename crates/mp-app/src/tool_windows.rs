//! The tools behind the Ctrl+F "temp" form (`temp.cs`) and Setup > Advanced
//! (`ConfigAdvanced.cs`): Warning Manager, NMEA / CoT / MicroDrone output, MAVLink mirror,
//! Follow Me, Moving Base, Proximity, MAVLink signing, FFT / Spectrogram, Message Interval,
//! log utilities and the vehicle maintenance commands.

use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use egui::{Color32, Pos2, Rect, Shape, Stroke, Vec2};
use mp_core::link::{Link, LinkCommand};
use mp_core::mavlink::dialects::ardupilotmega::MavMessage;
use mp_core::state::CurrentState;

use crate::app::Toasts;
use crate::theme;

// ------------------------------------------------------------------ file pickers

#[cfg(not(any(target_os = "android", target_os = "ios")))]
fn pick(title: &str, filter: &str, exts: &[&str]) -> Option<PathBuf> {
    rfd::FileDialog::new().set_title(title).add_filter(filter, exts).set_directory(crate::platform::data_dir().join("logs")).pick_file()
}

#[cfg(any(target_os = "android", target_os = "ios"))]
fn pick(_title: &str, _filter: &str, exts: &[&str]) -> Option<PathBuf> {
    crate::platform::list_files_in(&crate::platform::data_dir().join("logs"), exts).last().cloned()
}

#[cfg(not(any(target_os = "android", target_os = "ios")))]
fn pick_dir(title: &str) -> Option<PathBuf> {
    rfd::FileDialog::new().set_title(title).set_directory(crate::platform::data_dir().join("logs")).pick_folder()
}

#[cfg(any(target_os = "android", target_os = "ios"))]
fn pick_dir(_title: &str) -> Option<PathBuf> {
    Some(crate::platform::data_dir().join("logs"))
}

// ------------------------------------------------------------------ NMEA helpers

fn nmea_checksum(body: &str) -> String {
    let c = body.bytes().fold(0u8, |a, b| a ^ b);
    format!("${body}*{c:02X}\r\n")
}

fn nmea_deg(v: f64, lat: bool) -> (String, char) {
    let a = v.abs();
    let d = a.trunc();
    let m = (a - d) * 60.0;
    let s = if lat { format!("{:02}{:07.4}", d as u32, m) } else { format!("{:03}{:07.4}", d as u32, m) };
    let h = if lat {
        if v >= 0.0 { 'N' } else { 'S' }
    } else if v >= 0.0 {
        'E'
    } else {
        'W'
    };
    (s, h)
}

/// SerialOutputNMEA: GPGGA + GPRMC for the vehicle position.
fn nmea_sentences(cs: &CurrentState) -> String {
    let now = chrono_like_utc();
    let (la, lh) = nmea_deg(cs.lat, true);
    let (lo, oh) = nmea_deg(cs.lng, false);
    let gga = nmea_checksum(&format!("GPGGA,{now},{la},{lh},{lo},{oh},1,{:02},{:.1},{:.2},M,0.0,M,,", cs.satcount, cs.gpshdop, cs.altasl));
    let rmc = nmea_checksum(&format!("GPRMC,{now},A,{la},{lh},{lo},{oh},{:.2},{:.2},010100,,", cs.groundspeed * 1.943_844, cs.groundcourse));
    format!("{gga}{rmc}")
}

fn chrono_like_utc() -> String {
    let s = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64();
    let day = s % 86400.0;
    format!("{:02}{:02}{:05.2}", (day / 3600.0) as u32, ((day % 3600.0) / 60.0) as u32, day % 60.0)
}

/// Parse a $xxGGA sentence into (lat, lng, alt).
fn parse_gga(line: &str) -> Option<(f64, f64, f64)> {
    let f: Vec<&str> = line.trim().split(',').collect();
    if f.len() < 10 || !f[0].ends_with("GGA") {
        return None;
    }
    let conv = |v: &str, deg_len: usize| -> Option<f64> {
        if v.len() <= deg_len {
            return None;
        }
        let d: f64 = v[..deg_len].parse().ok()?;
        let m: f64 = v[deg_len..].parse().ok()?;
        Some(d + m / 60.0)
    };
    let mut lat = conv(f[2], 2)?;
    let mut lng = conv(f[4], 3)?;
    if f[3] == "S" {
        lat = -lat;
    }
    if f[5] == "W" {
        lng = -lng;
    }
    let alt = f[9].parse().unwrap_or(0.0);
    Some((lat, lng, alt))
}

/// Cursor-on-Target event XML (TAK) for the vehicle.
fn cot_xml(cs: &CurrentState) -> String {
    let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs();
    let iso = |secs: u64| {
        let days = secs / 86400;
        let (y, m, d) = civil_from_days(days as i64);
        let r = secs % 86400;
        format!("{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z", r / 3600, (r % 3600) / 60, r % 60)
    };
    format!(
        "<?xml version=\"1.0\" standalone=\"yes\"?><event version=\"2.0\" uid=\"MP-{}\" type=\"a-f-A-M-H-Q\" time=\"{}\" start=\"{}\" stale=\"{}\" how=\"m-g\"><point lat=\"{:.7}\" lon=\"{:.7}\" hae=\"{:.1}\" ce=\"9999999\" le=\"9999999\"/><detail><track course=\"{:.1}\" speed=\"{:.1}\"/></detail></event>",
        cs.sysid,
        iso(t),
        iso(t),
        iso(t + 5),
        cs.lat,
        cs.lng,
        cs.altasl,
        cs.groundcourse,
        cs.groundspeed
    )
}

fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + if m <= 2 { 1 } else { 0 }, m, d)
}

// ------------------------------------------------------------------ FFT

fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        for i in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (s, c) = (ang * k as f64).sin_cos();
                let (ur, ui) = (re[i + k], im[i + k]);
                let (vr, vi) = (re[i + k + len / 2] * c - im[i + k + len / 2] * s, re[i + k + len / 2] * s + im[i + k + len / 2] * c);
                re[i + k] = ur + vr;
                im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr;
                im[i + k + len / 2] = ui - vi;
            }
        }
        len <<= 1;
    }
}

/// Magnitude spectra of successive Hann-windowed blocks; returns (sample rate, blocks).
fn spectra(series: &[(f64, f64)], n: usize) -> (f64, Vec<Vec<f64>>) {
    if series.len() < n + 1 {
        return (0.0, Vec::new());
    }
    let dt = (series[series.len() - 1].0 - series[0].0) / (series.len() - 1) as f64;
    let rate = if dt > 0.0 { 1.0 / dt } else { 0.0 };
    let mut out = Vec::new();
    for chunk in series.chunks_exact(n) {
        let mean = chunk.iter().map(|p| p.1).sum::<f64>() / n as f64;
        let mut re: Vec<f64> = chunk.iter().enumerate().map(|(i, p)| (p.1 - mean) * (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / n as f64).cos())).collect();
        let mut im = vec![0.0; n];
        fft(&mut re, &mut im);
        out.push((0..n / 2).map(|i| (re[i] * re[i] + im[i] * im[i]).sqrt() / n as f64).collect());
    }
    (rate, out)
}

// ------------------------------------------------------------------ state

#[derive(Clone, Copy, PartialEq, Eq)]
enum OutKind {
    Nmea,
    Cot,
    MicroDrone,
}

struct Output {
    kind: OutKind,
    target: String,
    sock: Option<UdpSocket>,
    last: Instant,
}

#[derive(Clone)]
struct Warning {
    field: String,
    op: usize,
    value: f64,
    text: String,
    repeat: f64,
    last: Option<Instant>,
}

const OPS: [&str; 5] = ["<", ">", "==", "<=", ">="];

enum Prompt {
    MsgInterval { name: String, rate: String },
    Hex(String),
    HwId(String),
    Split(PathBuf, String),
    Takeoff(String),
}

struct Confirm {
    text: String,
    then: Box<dyn FnOnce(&Link)>,
}

#[derive(Default)]
pub struct Tools {
    prompt: Option<Prompt>,
    confirm: Option<Confirm>,
    text: Option<(String, String)>,
    outputs: Vec<Output>,
    out_window: Option<OutKind>,
    out_target: String,
    warnings: Vec<Warning>,
    warn_open: bool,
    warn_new: Option<Warning>,
    mirror_open: bool,
    mirror_target: String,
    mirror_write: bool,
    mirror_on: bool,
    follow: Option<(UdpSocket, Instant, bool)>,
    follow_open: Option<bool>,
    follow_port: String,
    last_fix: Option<(f64, f64, f64)>,
    prox_open: bool,
    sign_open: bool,
    sign_pass: String,
    fft: Option<(String, f64, Vec<(String, Vec<f64>)>)>,
    spect: Option<(String, f64, Vec<Vec<f64>>)>,
}

impl Tools {
    /// Run a temp/Advanced tool by its button text; false if this build has no handler.
    pub fn activate(&mut self, name: &str, link: Option<&Link>, toasts: &mut Toasts) -> bool {
        let need_link = |toasts: &mut Toasts| -> bool {
            if link.is_none() {
                toasts.error("Please connect first");
                return false;
            }
            true
        };
        match name {
            "Warning Manager" => self.warn_open = true,
            "NMEA" => self.open_output(OutKind::Nmea, "127.0.0.1:10110"),
            "CoT" => self.open_output(OutKind::Cot, "239.2.3.1:6969"),
            "MicroDrone" => self.open_output(OutKind::MicroDrone, "127.0.0.1:10111"),
            "Mavlink" | "Mavlink Mirror" => {
                self.mirror_open = true;
                if self.mirror_target.is_empty() {
                    self.mirror_target = "127.0.0.1:14550".into();
                }
            }
            "Follow Me" => self.open_follow(true),
            "Moving Base" => self.open_follow(false),
            "Proximity" => self.prox_open = true,
            "signing" | "Mavlink Signing" => self.sign_open = true,
            "FFT" => self.load_fft(false, toasts),
            "Spectrogram" => self.load_fft(true, toasts),
            "Message Interval" => self.prompt = Some(Prompt::MsgInterval { name: "ATTITUDE".into(), rate: "10".into() }),
            "hex Mavlink decode" | "parse packet bytes" => self.prompt = Some(Prompt::Hex("fd0500001a0c1".into())),
            "decode HWIDs" => self.prompt = Some(Prompt::HwId(String::new())),
            "Split DFLog" => {
                if let Some(p) = pick("Split DFLog", "Binary Log", &["bin", "BIN"]) {
                    self.prompt = Some(Prompt::Split(p, "2".into()));
                }
            }
            "arm and takeoff" => {
                if need_link(toasts) {
                    self.prompt = Some(Prompt::Takeoff("10".into()));
                }
            }
            "Toggle Safety Switch" => {
                if need_link(toasts) {
                    let custom = if link.map(|l| l.shared.read().cs.armed).unwrap_or(false) { 0 } else { 1 };
                    self.confirm = Some(Confirm {
                        text: "Are you sure?".into(),
                        then: Box::new(move |l: &Link| {
                            use mp_core::mavlink::dialects::ardupilotmega::{MavModeFlag, SET_MODE_DATA};
                            let ts = l.shared.read().cs.sysid;
                            l.send(LinkCommand::Send(Box::new(MavMessage::SET_MODE(SET_MODE_DATA { custom_mode: custom, target_system: ts, base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED }))));
                        }),
                    });
                }
            }
            "Force Accel Cal" => {
                if need_link(toasts) {
                    link.unwrap().send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 76.0, 0.0, 0.0] });
                }
            }
            "Force Compass Cal" => {
                if need_link(toasts) {
                    link.unwrap().send(LinkCommand::CommandLong { command: 241, params: [0.0, 76.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                }
            }
            "DFU Mode" => {
                if need_link(toasts) {
                    link.unwrap().send(LinkCommand::CommandLong { command: 246, params: [99.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                }
            }
            "Bootloader Upgrade" => {
                if need_link(toasts) {
                    self.confirm = Some(Confirm {
                        text: "Are you sure you want to upgrade the bootloader? This can brick your board".into(),
                        then: Box::new(|l: &Link| l.send(LinkCommand::CommandLong { command: 42650, params: [0.0, 0.0, 0.0, 0.0, 290876.0, 0.0, 0.0] })),
                    });
                }
            }
            "Lockup MAV" => {
                if need_link(toasts) {
                    self.confirm = Some(Confirm {
                        text: "Lockup the autopilot??? this can cause a CRASH!!!!!!".into(),
                        then: Box::new(|l: &Link| l.send(LinkCommand::CommandLong { command: 246, params: [42.0, 24.0, 71.0, 93.0, 0.0, 0.0, 0.0] })),
                    });
                }
            }
            "Param Restore" => {
                if need_link(toasts) {
                    if let Some(p) = pick("Param Restore", "Param List", &["param", "parm", "txt"]) {
                        let text = std::fs::read_to_string(&p).unwrap_or_default();
                        let list = mp_core::paramfile::load(&text);
                        let n = list.len();
                        for (k, v) in list {
                            link.unwrap().send(LinkCommand::SetParam { name: k, value: v });
                        }
                        self.text = Some(("Param Restore".into(), format!("Wrote {n} parameters from {}", p.display())));
                    }
                }
            }
            "extract gps_inject" => self.extract_rtcm(toasts),
            "Anon Log" => self.anon_log(toasts),
            "DashWare" => self.dashware(toasts),
            "Sort TLogs" | "ReSort All logs" => self.sort_tlogs(name == "ReSort All logs", toasts),
            "mag calb log" => self.mag_log(toasts),
            "APJ Tool" => {
                if let Some(p) = pick("APJ Tool", "ArduPilot firmware", &["apj"]) {
                    match std::fs::read_to_string(&p).map_err(|e| e.to_string()).and_then(|t| mp_core::fwupload::parse_apj(&t)) {
                        Ok(a) => self.text = Some(("APJ Tool".into(), format!("{}\nboard_id: {}\nimage size: {} bytes\nimage crc: 0x{:08X}", a.summary, a.board_id, a.image.len(), mp_core::fwupload::image_crc(&a.image, 0)))),
                        Err(e) => toasts.error(e),
                    }
                }
            }
            "Age Map Data" | "Clear Custom Maps" => {
                let dir = crate::platform::tile_cache_dir();
                let cutoff = if name == "Age Map Data" { Some(Duration::from_secs(30 * 86400)) } else { None };
                let n = remove_tiles(&dir, cutoff);
                self.text = Some((name.into(), format!("Removed {n} map tiles from {}", dir.display())));
            }
            "structtest" => {
                let t = Instant::now();
                let data = mp_core::dataflash::sample_log();
                let mut n = 0;
                for _ in 0..20 {
                    n += mp_core::dataflash::parse(&data).records.len();
                }
                let el = t.elapsed();
                self.text = Some(("structtest".into(), format!("{n} records decoded in {:.1} ms ({:.0} records/s)", el.as_secs_f64() * 1e3, n as f64 / el.as_secs_f64())));
            }
            "Geo ref images" => return false,
            "driver clean" => self.text = Some(("driver clean".into(), "No Mission Planner drivers are installed by this build; nothing to remove.".into())),
            "Param gen" => {
                let dir = crate::platform::data_dir();
                let r = crate::firmware_ui::get("https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.xml").and_then(|b| std::fs::write(dir.join("ParameterMetaData.xml"), b).map_err(|e| e.to_string()));
                match r {
                    Ok(()) => self.text = Some(("Param gen".into(), format!("Parameter metadata saved to {}", dir.join("ParameterMetaData.xml").display()))),
                    Err(e) => toasts.error(format!("Param gen failed\n{e}")),
                }
            }
            _ => return false,
        }
        true
    }

    fn open_output(&mut self, kind: OutKind, def: &str) {
        self.out_window = Some(kind);
        self.out_target = self.outputs.iter().find(|o| o.kind == kind).map(|o| o.target.clone()).unwrap_or_else(|| def.to_string());
    }

    fn open_follow(&mut self, follow: bool) {
        self.follow_open = Some(follow);
        if self.follow_port.is_empty() {
            self.follow_port = "10110".into();
        }
    }

    fn load_fft(&mut self, spect: bool, toasts: &mut Toasts) {
        let Some(p) = crate::log_browse::pick_bin() else { return };
        let Ok(data) = std::fs::read(&p) else {
            toasts.error("Failed to read log");
            return;
        };
        let log = mp_core::dataflash::parse(&data);
        let name = p.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
        if spect {
            let s = log.series("IMU", "AccX");
            let (rate, blocks) = spectra(&s, 256);
            if blocks.is_empty() {
                toasts.error("Not enough IMU data in this log");
                return;
            }
            self.spect = Some((name, rate, blocks));
        } else {
            let mut out = Vec::new();
            let mut rate = 0.0;
            for f in ["AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ"] {
                let s = log.series("IMU", f);
                let (r, blocks) = spectra(&s, 1024.min(s.len().next_power_of_two() / 2).max(64));
                if blocks.is_empty() {
                    continue;
                }
                rate = r;
                let n = blocks[0].len();
                let avg: Vec<f64> = (0..n).map(|i| blocks.iter().map(|b| b[i]).sum::<f64>() / blocks.len() as f64).collect();
                out.push((f.to_string(), avg));
            }
            if out.is_empty() {
                toasts.error("Not enough IMU data in this log");
                return;
            }
            self.fft = Some((name, rate, out));
        }
    }

    fn extract_rtcm(&mut self, toasts: &mut Toasts) {
        let Some(p) = pick("extract gps_inject", "Telemetry Log", &["tlog"]) else { return };
        match mp_core::tlog::read(&p) {
            Ok(recs) => {
                let mut out = Vec::new();
                for r in recs {
                    match &r.msg {
                        MavMessage::GPS_RTCM_DATA(d) => out.extend_from_slice(&d.data[..d.len as usize]),
                        MavMessage::GPS_INJECT_DATA(d) => out.extend_from_slice(&d.data[..d.len as usize]),
                        _ => {}
                    }
                }
                let to = p.with_extension("rtcm");
                match std::fs::write(&to, &out) {
                    Ok(()) => self.text = Some(("extract gps_inject".into(), format!("Wrote {} bytes to {}", out.len(), to.display()))),
                    Err(e) => toasts.error(e.to_string()),
                }
            }
            Err(e) => toasts.error(e.to_string()),
        }
    }

    /// LogAnon: shift every position in a tlog by a random offset.
    fn anon_log(&mut self, toasts: &mut Toasts) {
        let Some(p) = pick("Anon Log", "Telemetry Log", &["tlog"]) else { return };
        let Ok(recs) = mp_core::tlog::read(&p) else {
            toasts.error("Failed to read log");
            return;
        };
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().subsec_nanos();
        let (dlat, dlng) = (((seed % 1000) as i32 - 500) * 10_000, (((seed / 1000) % 1000) as i32 - 500) * 10_000);
        let mut buf = Vec::new();
        for r in recs {
            let mut m = r.msg.clone();
            match &mut m {
                MavMessage::GLOBAL_POSITION_INT(d) => {
                    d.lat += dlat;
                    d.lon += dlng;
                }
                MavMessage::GPS_RAW_INT(d) => {
                    d.lat += dlat;
                    d.lon += dlng;
                }
                MavMessage::HOME_POSITION(d) => {
                    d.latitude += dlat;
                    d.longitude += dlng;
                }
                MavMessage::MISSION_ITEM_INT(d) => {
                    if d.x != 0 || d.y != 0 {
                        d.x += dlat;
                        d.y += dlng;
                    }
                }
                _ => {}
            }
            buf.extend_from_slice(&r.time_us.to_be_bytes());
            let _ = mp_core::mavlink::write_versioned_msg(&mut buf, mp_core::mavlink::MavlinkVersion::V2, r.header, &m);
        }
        let to = p.with_file_name(format!("{}-anon.tlog", p.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default()));
        match std::fs::write(&to, buf) {
            Ok(()) => self.text = Some(("Anon Log".into(), format!("Saved {}", to.display()))),
            Err(e) => toasts.error(e.to_string()),
        }
    }

    fn dashware(&mut self, toasts: &mut Toasts) {
        let Some(p) = pick("DashWare", "Telemetry Log", &["tlog"]) else { return };
        let Ok(recs) = mp_core::tlog::read(&p) else {
            toasts.error("Failed to read log");
            return;
        };
        let mut cs = CurrentState::default();
        let names: Vec<&str> = cs.fields().iter().map(|(n, _)| *n).collect();
        let mut out = format!("time,{}\n", names.join(","));
        let mut last = 0u64;
        for r in recs {
            cs.update(&r.msg);
            if r.time_us.saturating_sub(last) >= 200_000 {
                last = r.time_us;
                let vals: Vec<String> = cs.fields().iter().map(|(_, v)| format!("{v}")).collect();
                out.push_str(&format!("{},{}\n", r.time_us as f64 / 1e6, vals.join(",")));
            }
        }
        let to = p.with_extension("csv");
        match std::fs::write(&to, out) {
            Ok(()) => self.text = Some(("DashWare".into(), format!("Created {}", to.display()))),
            Err(e) => toasts.error(e.to_string()),
        }
    }

    /// LogSort: move tlogs into <vehicle type>/<sysid>/ folders from their first heartbeat.
    fn sort_tlogs(&mut self, all: bool, toasts: &mut Toasts) {
        let dir = if all { Some(crate::platform::data_dir().join("logs")) } else { pick_dir("Sort TLogs") };
        let Some(dir) = dir else { return };
        let mut moved = 0;
        for f in crate::platform::list_files_in(&dir, &["tlog"]) {
            let Ok(recs) = mp_core::tlog::read(&f) else { continue };
            let Some((sys, ty)) = recs.iter().find_map(|r| match &r.msg {
                MavMessage::HEARTBEAT(h) if r.header.system_id != 255 => Some((r.header.system_id, format!("{:?}", h.mavtype).trim_start_matches("MAV_TYPE_").to_string())),
                _ => None,
            }) else {
                continue;
            };
            let to_dir = dir.join(ty).join(sys.to_string());
            if std::fs::create_dir_all(&to_dir).is_ok() {
                if let Some(name) = f.file_name() {
                    if std::fs::rename(&f, to_dir.join(name)).is_ok() {
                        moved += 1;
                    }
                }
            }
        }
        if moved == 0 {
            toasts.error("No tlogs with a vehicle heartbeat were found");
        }
        self.text = Some(("Sort TLogs".into(), format!("Moved {moved} tlogs under {}", dir.display())));
    }

    /// MagCalib.ProcessLog: hard-iron offsets from the MAG min/max of a log.
    fn mag_log(&mut self, toasts: &mut Toasts) {
        let Some(p) = crate::log_browse::pick_bin() else { return };
        let Ok(data) = std::fs::read(&p) else { return };
        let log = mp_core::dataflash::parse(&data);
        let mut out = String::new();
        for m in ["MAG", "MAG2", "MAG3"] {
            let axes: Vec<Vec<(f64, f64)>> = ["MagX", "MagY", "MagZ"].iter().map(|f| log.series(m, f)).collect();
            if axes.iter().any(|a| a.is_empty()) {
                continue;
            }
            let ofs: Vec<f64> = axes.iter().map(|a| {
                let (lo, hi) = a.iter().fold((f64::MAX, f64::MIN), |(l, h), p| (l.min(p.1), h.max(p.1)));
                -(lo + hi) / 2.0
            }).collect();
            out.push_str(&format!("{m}: offsets {:.0}, {:.0}, {:.0}\n", ofs[0], ofs[1], ofs[2]));
        }
        if out.is_empty() {
            toasts.error("No MAG data in this log");
        } else {
            self.text = Some(("mag calb log".into(), out));
        }
    }

    /// Per-frame: run outputs, follow-me, warnings and draw the tool windows.
    pub fn ui(&mut self, ctx: &egui::Context, link: Option<&Link>, toasts: &mut Toasts) {
        let cs = link.map(|l| l.shared.read().cs.clone()).unwrap_or_default();
        self.tick(link, &cs, toasts);
        self.prompt_ui(ctx, link, toasts);
        if let Some(c) = self.confirm.take() {
            let mut answer = None;
            egui::Modal::new(egui::Id::new("tool_confirm")).show(ctx, |ui| {
                ui.set_width(300.0);
                theme::label(ui, &c.text);
                ui.horizontal(|ui| {
                    if theme::button(ui, "Yes").clicked() {
                        answer = Some(true);
                    }
                    if theme::button(ui, "No").clicked() {
                        answer = Some(false);
                    }
                });
            });
            match answer {
                Some(true) => {
                    if let Some(l) = link {
                        (c.then)(l);
                    }
                }
                Some(false) => {}
                None => self.confirm = Some(c),
            }
        }
        if let Some((title, text)) = &self.text {
            let mut open = true;
            egui::Window::new(title.as_str()).id(egui::Id::new("tool_text")).open(&mut open).collapsible(false).show(ctx, |ui| {
                egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                    ui.label(egui::RichText::new(text.as_str()).monospace().color(Color32::WHITE));
                });
            });
            if !open {
                self.text = None;
            }
        }
        self.output_ui(ctx);
        self.warn_ui(ctx, &cs);
        self.mirror_ui(ctx, link);
        self.follow_ui(ctx);
        self.prox_ui(ctx, link);
        self.sign_ui(ctx, link, toasts);
        self.fft_ui(ctx);
    }

    fn tick(&mut self, link: Option<&Link>, cs: &CurrentState, toasts: &mut Toasts) {
        for o in &mut self.outputs {
            if o.last.elapsed() < Duration::from_millis(500) {
                continue;
            }
            o.last = Instant::now();
            if let Some(s) = &o.sock {
                let payload = match o.kind {
                    OutKind::Nmea => nmea_sentences(cs),
                    OutKind::Cot => cot_xml(cs),
                    OutKind::MicroDrone => format!("$MD,{:.7},{:.7},{:.1},{:.1},{:.1},{:.1}\r\n", cs.lat, cs.lng, cs.alt, cs.yaw, cs.groundspeed, cs.battery_voltage),
                };
                let _ = s.send_to(payload.as_bytes(), o.target.as_str());
            }
        }
        if let Some((sock, last, follow)) = &mut self.follow {
            let mut b = [0u8; 1024];
            while let Ok((n, _)) = sock.recv_from(&mut b) {
                for line in String::from_utf8_lossy(&b[..n]).lines() {
                    if let Some(fix) = parse_gga(line) {
                        self.last_fix = Some(fix);
                    }
                }
            }
            if *follow && last.elapsed() > Duration::from_secs(1) {
                *last = Instant::now();
                if let (Some((lat, lng, _)), Some(l)) = (self.last_fix, link) {
                    if cs.mode.eq_ignore_ascii_case("guided") {
                        l.send(LinkCommand::Guided { lat, lng, alt: cs.alt.max(10.0), frame: 3 });
                    }
                }
            }
        }
        for w in &mut self.warnings {
            let Some(v) = cs.field(&w.field) else { continue };
            let hit = match w.op {
                0 => v < w.value,
                1 => v > w.value,
                2 => (v - w.value).abs() < 1e-6,
                3 => v <= w.value,
                _ => v >= w.value,
            };
            if hit && w.last.map(|t| t.elapsed().as_secs_f64() > w.repeat).unwrap_or(true) {
                w.last = Some(Instant::now());
                toasts.info(format!("{} ({} {} {})", w.text, w.field, OPS[w.op], w.value));
            }
        }
    }

    fn prompt_ui(&mut self, ctx: &egui::Context, link: Option<&Link>, toasts: &mut Toasts) {
        let Some(pr) = self.prompt.as_mut() else { return };
        let mut done = false;
        let mut ok = false;
        let title = match pr {
            Prompt::MsgInterval { .. } => "Message Interval",
            Prompt::Hex(_) => "hex Mavlink decode",
            Prompt::HwId(_) => "hwid",
            Prompt::Split(..) => "Split DFLog",
            Prompt::Takeoff(_) => "arm and takeoff",
        };
        let seen: Vec<&'static str> = link.map(|l| l.shared.read().inspector.values().map(|s| s.name).collect()).unwrap_or_default();
        egui::Window::new(title).collapsible(false).resizable(false).show(ctx, |ui| {
            match pr {
                Prompt::MsgInterval { name, rate } => {
                    ui.horizontal(|ui| {
                        egui::ComboBox::from_id_salt("mi_msg").width(200.0).selected_text(name.as_str()).show_ui(ui, |ui| {
                            let mut all = seen.clone();
                            all.sort();
                            all.dedup();
                            for n in all {
                                ui.selectable_value(name, n.to_string(), n);
                            }
                        });
                        ui.add(egui::TextEdit::singleline(rate).desired_width(50.0));
                        theme::label(ui, "Hz");
                    });
                }
                Prompt::Hex(s) => {
                    theme::label(ui, "enter the hex value 'fd0500001a0c1'");
                    ui.add(egui::TextEdit::multiline(s).desired_width(360.0).desired_rows(3));
                }
                Prompt::HwId(s) => {
                    theme::label(ui, "Enter the ID number");
                    ui.text_edit_singleline(s);
                }
                Prompt::Split(_, s) => {
                    theme::label(ui, "Split into how many pieces?");
                    ui.text_edit_singleline(s);
                }
                Prompt::Takeoff(s) => {
                    theme::label(ui, "Takeoff altitude (m)");
                    ui.text_edit_singleline(s);
                }
            }
            ui.horizontal(|ui| {
                if theme::button(ui, "OK").clicked() {
                    ok = true;
                    done = true;
                }
                if theme::button(ui, "Cancel").clicked() {
                    done = true;
                }
            });
        });
        if !done {
            return;
        }
        let pr = self.prompt.take().unwrap();
        if !ok {
            return;
        }
        match pr {
            Prompt::MsgInterval { name, rate } => {
                let (Some(l), Ok(r)) = (link, rate.trim().parse::<f32>()) else { return };
                let Some(id) = <MavMessage as mp_core::mavlink::Message>::message_id_from_name(&name) else {
                    toasts.error(format!("Unknown message {name}"));
                    return;
                };
                let interval = if r <= 0.0 { -1.0 } else { 1e6 / r };
                l.send(LinkCommand::CommandLong { command: 511, params: [id as f32, interval, 0.0, 0.0, 0.0, 0.0, 0.0] });
            }
            Prompt::Hex(s) => {
                let clean: String = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
                let bytes: Vec<u8> = (0..clean.len() / 2).filter_map(|i| u8::from_str_radix(&clean[i * 2..i * 2 + 2], 16).ok()).collect();
                let mut r = mp_core::mavlink::MavlinkReader::new(&bytes[..]);
                let mut out = String::new();
                while let Ok((h, m)) = r.read_any_message::<MavMessage>() {
                    out.push_str(&format!("{h:?}\n{m:#?}\n"));
                }
                if out.is_empty() {
                    out = format!("Could not decode a MAVLink packet from {} bytes", bytes.len());
                }
                self.text = Some(("hex Mavlink decode".into(), out));
            }
            Prompt::HwId(s) => {
                let Ok(raw) = s.trim().parse::<u32>() else {
                    toasts.error("Invalid number");
                    return;
                };
                let d = mp_core::devid::decode(raw);
                self.text = Some((
                    "hwid".into(),
                    format!(
                        "bus_type: {}\nbus: {}\naddress: {} (0x{:02X})\ndevtype: {} (0x{:02X})\n  compass: {}\n  imu: {}\n  baro: {}\n  airspeed: {}",
                        d.bus_type,
                        d.bus,
                        d.address,
                        d.address,
                        d.devtype,
                        d.devtype,
                        mp_core::devid::compass_devtype(d.devtype),
                        mp_core::devid::imu_devtype(d.devtype),
                        mp_core::devid::baro_devtype(d.devtype),
                        mp_core::devid::airspeed_devtype(d.devtype)
                    ),
                ));
            }
            Prompt::Split(path, s) => {
                let Ok(n) = s.trim().parse::<usize>() else { return };
                match split_bin(&path, n.max(1)) {
                    Ok(files) => self.text = Some(("Split DFLog".into(), files.iter().map(|f| f.display().to_string()).collect::<Vec<_>>().join("\n"))),
                    Err(e) => toasts.error(e),
                }
            }
            Prompt::Takeoff(s) => {
                let (Some(l), Ok(alt)) = (link, s.trim().parse::<f32>()) else { return };
                let fw = l.shared.read().cs.firmware;
                if let Some((num, _)) = fw.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case("guided")) {
                    l.send(LinkCommand::SetMode(num));
                }
                l.send(LinkCommand::Arm { arm: true, force: false });
                l.send(LinkCommand::CommandLong { command: 22, params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt] });
            }
        }
    }

    fn output_ui(&mut self, ctx: &egui::Context) {
        let Some(kind) = self.out_window else { return };
        let title = match kind {
            OutKind::Nmea => "NMEA Output",
            OutKind::Cot => "Cursor on Target",
            OutKind::MicroDrone => "MicroDrone Output",
        };
        let mut open = true;
        let running = self.outputs.iter().any(|o| o.kind == kind);
        let mut toggle = false;
        egui::Window::new(title).open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            theme::label(ui, "UDP destination (host:port)");
            ui.add_enabled(!running, egui::TextEdit::singleline(&mut self.out_target));
            if theme::button(ui, if running { "Stop" } else { "Start" }).clicked() {
                toggle = true;
            }
        });
        if toggle {
            if running {
                self.outputs.retain(|o| o.kind != kind);
            } else {
                let sock = UdpSocket::bind("0.0.0.0:0").ok();
                if let Some(s) = &sock {
                    let _ = s.set_broadcast(true);
                }
                self.outputs.push(Output { kind, target: self.out_target.clone(), sock, last: Instant::now() - Duration::from_secs(1) });
            }
        }
        if !open {
            self.out_window = None;
        }
    }

    fn warn_ui(&mut self, ctx: &egui::Context, cs: &CurrentState) {
        if !self.warn_open {
            return;
        }
        let mut open = true;
        let fields: Vec<&'static str> = cs.fields().into_iter().map(|(n, _)| n).collect();
        let mut remove = None;
        egui::Window::new("Warning Manager").open(&mut open).collapsible(false).default_size([520.0, 300.0]).show(ctx, |ui| {
            egui::Grid::new("warnings").num_columns(6).striped(true).show(ui, |ui| {
                for h in ["Field", "Op", "Value", "Repeat (s)", "Say", ""] {
                    theme::label(ui, h);
                }
                ui.end_row();
                for (i, w) in self.warnings.iter().enumerate() {
                    theme::label(ui, &w.field);
                    theme::label(ui, OPS[w.op]);
                    theme::label(ui, w.value.to_string());
                    theme::label(ui, w.repeat.to_string());
                    theme::label(ui, &w.text);
                    if theme::button(ui, "Delete").clicked() {
                        remove = Some(i);
                    }
                    ui.end_row();
                }
            });
            ui.separator();
            let nw = self.warn_new.get_or_insert_with(|| Warning { field: "alt".into(), op: 0, value: 10.0, text: "Altitude low".into(), repeat: 10.0, last: None });
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt("wm_field").width(110.0).selected_text(nw.field.as_str()).show_ui(ui, |ui| {
                    for f in &fields {
                        ui.selectable_value(&mut nw.field, f.to_string(), *f);
                    }
                });
                egui::ComboBox::from_id_salt("wm_op").width(40.0).selected_text(OPS[nw.op]).show_ui(ui, |ui| {
                    for (k, o) in OPS.iter().enumerate() {
                        ui.selectable_value(&mut nw.op, k, *o);
                    }
                });
                ui.add(egui::DragValue::new(&mut nw.value).speed(0.1));
                ui.add(egui::DragValue::new(&mut nw.repeat).range(1.0..=3600.0));
                ui.add(egui::TextEdit::singleline(&mut nw.text).desired_width(120.0));
            });
            if theme::button(ui, "Add").clicked() {
                if let Some(w) = self.warn_new.clone() {
                    self.warnings.push(w);
                }
            }
        });
        if let Some(i) = remove {
            self.warnings.remove(i);
        }
        if !open {
            self.warn_open = false;
        }
    }

    fn mirror_ui(&mut self, ctx: &egui::Context, link: Option<&Link>) {
        if !self.mirror_open {
            return;
        }
        let mut open = true;
        egui::Window::new("Mavlink Mirror").open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            theme::label(ui, "UDP destination (host:port)");
            ui.add_enabled(!self.mirror_on, egui::TextEdit::singleline(&mut self.mirror_target));
            ui.add_enabled(!self.mirror_on, egui::Checkbox::new(&mut self.mirror_write, "Write access"));
            if theme::button(ui, if self.mirror_on { "Stop" } else { "Start" }).clicked() {
                match link {
                    Some(l) => {
                        self.mirror_on = !self.mirror_on;
                        l.send(LinkCommand::Mirror { target: self.mirror_on.then(|| self.mirror_target.clone()), write: self.mirror_write });
                    }
                    None => {
                        theme::label(ui, "Please connect first");
                    }
                }
            }
        });
        if !open {
            self.mirror_open = false;
        }
    }

    fn follow_ui(&mut self, ctx: &egui::Context) {
        let Some(follow) = self.follow_open else { return };
        let mut open = true;
        let running = self.follow.is_some();
        let mut toggle = false;
        egui::Window::new(if follow { "Follow Me" } else { "Moving Base" }).open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            theme::label(ui, "NMEA GPS input: UDP port");
            ui.add_enabled(!running, egui::TextEdit::singleline(&mut self.follow_port));
            if follow {
                theme::label(ui, "The vehicle must be in Guided mode; it is sent your position every second.");
            }
            match self.last_fix {
                Some((la, lo, al)) => theme::label(ui, format!("Last fix: {la:.7} {lo:.7} {al:.1}m")),
                None => theme::label(ui, "No fix"),
            };
            if theme::button(ui, if running { "Stop" } else { "Start" }).clicked() {
                toggle = true;
            }
        });
        if toggle {
            if running {
                self.follow = None;
            } else if let Ok(s) = UdpSocket::bind(format!("0.0.0.0:{}", self.follow_port.trim())) {
                let _ = s.set_nonblocking(true);
                self.follow = Some((s, Instant::now(), follow));
            }
        }
        if !open {
            self.follow_open = None;
        }
    }

    /// ProximityControl: 360-degree view of DISTANCE_SENSOR readings.
    fn prox_ui(&mut self, ctx: &egui::Context, link: Option<&Link>) {
        if !self.prox_open {
            return;
        }
        let readings: Vec<(f32, f32)> = link
            .map(|l| {
                l.shared
                    .read()
                    .inspector
                    .values()
                    .filter_map(|s| match &s.msg {
                        MavMessage::DISTANCE_SENSOR(d) if s.last.elapsed() < Duration::from_secs(3) => Some((d.orientation as u8 as f32 * 45.0, d.current_distance as f32 / 100.0)),
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default();
        let mut open = true;
        egui::Window::new("Proximity").open(&mut open).collapsible(false).default_size([400.0, 420.0]).show(ctx, |ui| {
            let (rect, _) = ui.allocate_exact_size(Vec2::splat(380.0), egui::Sense::hover());
            let p = ui.painter();
            p.rect_filled(rect, 0.0, Color32::BLACK);
            let c = rect.center();
            let max = 10.0f32;
            for k in 1..=4 {
                let r = 180.0 * k as f32 / 4.0;
                p.circle_stroke(c, r, Stroke::new(1.0, Color32::from_gray(80)));
                p.text(c + Vec2::new(2.0, -r), egui::Align2::LEFT_BOTTOM, format!("{:.1}m", max * k as f32 / 4.0), egui::FontId::proportional(10.0), Color32::GRAY);
            }
            p.add(Shape::convex_polygon(vec![c + Vec2::new(0.0, -12.0), c + Vec2::new(8.0, 8.0), c + Vec2::new(-8.0, 8.0)], Color32::WHITE, Stroke::NONE));
            for (ang, d) in &readings {
                let r = (d / max).min(1.0) * 180.0;
                let a0 = (ang - 22.5 - 90.0).to_radians();
                let a1 = (ang + 22.5 - 90.0).to_radians();
                let pts: Vec<Pos2> = (0..=8).map(|i| {
                    let a = a0 + (a1 - a0) * i as f32 / 8.0;
                    c + Vec2::new(a.cos(), a.sin()) * r
                }).collect();
                let col = if *d < 2.0 { Color32::RED } else if *d < 5.0 { Color32::YELLOW } else { Color32::GREEN };
                p.add(Shape::line(pts, Stroke::new(3.0, col)));
            }
            if readings.is_empty() {
                p.text(Pos2::new(c.x, rect.bottom() - 16.0), egui::Align2::CENTER_CENTER, "No proximity data", egui::FontId::proportional(12.0), Color32::WHITE);
            }
        });
        if !open {
            self.prox_open = false;
        }
    }

    /// AuthKeys: MAVLink2 signing key from a passphrase (SHA-256), sent with SETUP_SIGNING.
    fn sign_ui(&mut self, ctx: &egui::Context, link: Option<&Link>, toasts: &mut Toasts) {
        if !self.sign_open {
            return;
        }
        let mut open = true;
        egui::Window::new("Mavlink Signing").open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            theme::label(ui, "Passphrase");
            ui.add(egui::TextEdit::singleline(&mut self.sign_pass).password(true));
            ui.horizontal(|ui| {
                let use_key = theme::button(ui, "Use Key").clicked();
                let disable = theme::button(ui, "Disable").clicked();
                if disable {
                    self.sign_pass.clear();
                }
                if use_key || disable {
                    let Some(l) = link else {
                        toasts.error("Please connect first");
                        return;
                    };
                    use mp_core::mavlink::dialects::ardupilotmega::SETUP_SIGNING_DATA;
                    use sha2::Digest;
                    let key: [u8; 32] = if self.sign_pass.is_empty() { [0; 32] } else { sha2::Sha256::digest(self.sign_pass.as_bytes()).into() };
                    let us = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_micros() as u64;
                    // MAVLink signing timestamps count 10us units since 2015-01-01
                    let ts = us.saturating_sub(1_420_070_400_000_000) / 10;
                    let (sys, comp) = {
                        let s = l.shared.read();
                        (s.cs.sysid, s.cs.compid)
                    };
                    l.send(LinkCommand::Send(Box::new(MavMessage::SETUP_SIGNING(SETUP_SIGNING_DATA { initial_timestamp: ts, target_system: sys, target_component: comp, secret_key: key }))));
                    toasts.info(if self.sign_pass.is_empty() { "Signing disabled" } else { "Signing key sent" });
                }
            });
        });
        if !open {
            self.sign_open = false;
        }
    }

    fn fft_ui(&mut self, ctx: &egui::Context) {
        let colors = [Color32::RED, Color32::GREEN, Color32::from_rgb(0, 0xbf, 0xff), Color32::from_rgb(0xff, 0xa5, 0), Color32::YELLOW, Color32::from_rgb(0xff, 0, 0xff)];
        if let Some((name, rate, series)) = &self.fft {
            let mut open = true;
            egui::Window::new(format!("FFT {name}")).id(egui::Id::new("fftui")).open(&mut open).default_size([700.0, 420.0]).show(ctx, |ui| {
                let size = ui.available_size().max(Vec2::new(500.0, 320.0));
                let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
                let p = ui.painter();
                p.rect_filled(rect, 0.0, Color32::from_rgb(0x1f, 0x1f, 0x20));
                let chart = Rect::from_min_max(rect.min + Vec2::new(50.0, 10.0), rect.max - Vec2::new(10.0, 30.0));
                let hi = series.iter().flat_map(|(_, s)| s.iter().skip(1)).cloned().fold(1e-9, f64::max);
                let nyq = rate / 2.0;
                for k in 0..=5 {
                    let x = chart.left() + chart.width() * k as f32 / 5.0;
                    p.text(Pos2::new(x, chart.bottom() + 3.0), egui::Align2::CENTER_TOP, format!("{:.0}", nyq * k as f64 / 5.0), egui::FontId::proportional(10.0), Color32::WHITE);
                }
                p.text(Pos2::new(chart.center().x, rect.bottom() - 2.0), egui::Align2::CENTER_BOTTOM, "Hz", egui::FontId::proportional(10.0), Color32::WHITE);
                for (i, (n, s)) in series.iter().enumerate() {
                    let pts: Vec<Pos2> = s.iter().enumerate().skip(1).map(|(k, v)| Pos2::new(chart.left() + chart.width() * k as f32 / s.len() as f32, chart.bottom() - (v / hi) as f32 * chart.height())).collect();
                    p.add(Shape::line(pts, Stroke::new(1.0, colors[i % colors.len()])));
                    p.text(chart.left_top() + Vec2::new(10.0 + i as f32 * 60.0, 2.0), egui::Align2::LEFT_TOP, n, egui::FontId::proportional(11.0), colors[i % colors.len()]);
                }
            });
            if !open {
                self.fft = None;
            }
        }
        if let Some((name, rate, blocks)) = &self.spect {
            let mut open = true;
            egui::Window::new(format!("Spectrogram {name}")).id(egui::Id::new("spectui")).open(&mut open).default_size([700.0, 420.0]).show(ctx, |ui| {
                let size = ui.available_size().max(Vec2::new(500.0, 320.0));
                let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
                let p = ui.painter();
                let chart = Rect::from_min_max(rect.min + Vec2::new(40.0, 6.0), rect.max - Vec2::new(6.0, 24.0));
                let hi = blocks.iter().flat_map(|b| b.iter().skip(1)).cloned().fold(1e-9, f64::max).ln();
                let lo = hi - 8.0;
                let (w, h) = (chart.width() / blocks.len() as f32, chart.height() / blocks[0].len() as f32);
                for (x, b) in blocks.iter().enumerate() {
                    for (y, v) in b.iter().enumerate() {
                        let t = (((v.max(1e-12).ln() - lo) / (hi - lo)).clamp(0.0, 1.0)) as f32;
                        let col = Color32::from_rgb((255.0 * t) as u8, (255.0 * (1.0 - (2.0 * t - 1.0).abs())) as u8, (255.0 * (1.0 - t)) as u8);
                        p.rect_filled(Rect::from_min_size(Pos2::new(chart.left() + x as f32 * w, chart.bottom() - (y + 1) as f32 * h), Vec2::new(w.max(1.0), h.max(1.0))), 0.0, col);
                    }
                }
                p.text(chart.left_top() - Vec2::new(4.0, 0.0), egui::Align2::RIGHT_TOP, format!("{:.0}Hz", rate / 2.0), egui::FontId::proportional(10.0), Color32::WHITE);
                p.text(chart.left_bottom() - Vec2::new(4.0, 0.0), egui::Align2::RIGHT_BOTTOM, "0", egui::FontId::proportional(10.0), Color32::WHITE);
                p.text(Pos2::new(chart.center().x, rect.bottom() - 2.0), egui::Align2::CENTER_BOTTOM, "time (IMU AccX)", egui::FontId::proportional(10.0), Color32::WHITE);
            });
            if !open {
                self.spect = None;
            }
        }
    }
}

fn remove_tiles(dir: &Path, older: Option<Duration>) -> usize {
    let mut n = 0;
    let Ok(rd) = std::fs::read_dir(dir) else { return 0 };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            n += remove_tiles(&p, older);
            continue;
        }
        let old = match older {
            Some(age) => e.metadata().and_then(|m| m.modified()).map(|t| t.elapsed().unwrap_or_default() > age).unwrap_or(false),
            None => true,
        };
        if old && std::fs::remove_file(&p).is_ok() {
            n += 1;
        }
    }
    n
}

/// LogSplit: cut a .bin into `n` parts at message boundaries; each part keeps the FMT header.
fn split_bin(path: &Path, n: usize) -> Result<Vec<PathBuf>, String> {
    let data = std::fs::read(path).map_err(|e| e.to_string())?;
    let fmt_len = data.windows(3).enumerate().filter(|(_, w)| w[0] == 0xA3 && w[1] == 0x95 && w[2] == 0x80).map(|(i, _)| i + 89).max().unwrap_or(0).min(data.len());
    let header = &data[..fmt_len];
    let body = &data[fmt_len..];
    let per = body.len().div_ceil(n);
    let mut out = Vec::new();
    let mut start = 0;
    for i in 0..n {
        let mut end = ((i + 1) * per).min(body.len());
        while end < body.len() && !(body[end] == 0xA3 && body.get(end + 1) == Some(&0x95)) {
            end += 1;
        }
        if start >= end {
            break;
        }
        let to = path.with_file_name(format!("{}-{}.bin", path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default(), i + 1));
        let mut buf = header.to_vec();
        buf.extend_from_slice(&body[start..end]);
        std::fs::write(&to, buf).map_err(|e| e.to_string())?;
        out.push(to);
        start = end;
    }
    Ok(out)
}

