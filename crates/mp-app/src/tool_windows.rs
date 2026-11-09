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

fn pick(title: &str, filter: &str, exts: &[&str]) -> Option<PathBuf> {
    let logs_dir = crate::platform::data_dir().join("logs");
    crate::platform::pick_file_in(title, filter, exts, Some(&logs_dir))
}

fn pick_dir(title: &str) -> Option<PathBuf> {
    let logs_dir = crate::platform::data_dir().join("logs");
    crate::platform::pick_folder_in(title, Some(&logs_dir))
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

/// The forward FFT through the port of `fft.cs` (FFT2.init(log2 n) + run); `n` is a power
/// of two.
fn fft(re: &mut [f64], im: &mut [f64]) {
    let mut f = mp_core::extlibs::utilities::FFT2::new();
    f.init(re.len().trailing_zeros());
    if let Err(e) = f.run(re, im, false) {
        log::error!("{e}");
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

/// `CustomWarning.defaultsrc = comPort.MAV.cs`: the vehicle state as of the last frame.
struct CsSource(std::sync::Arc<std::sync::RwLock<mp_core::state::CurrentState>>);

impl mp_core::extlibs::utilities::warnings::PropertySource for CsSource {
    fn property_names(&self) -> Vec<String> {
        mp_core::state::properties::PROPERTIES.iter().map(|(n, _)| n.to_string()).collect()
    }
    fn property_value(&self, name: &str) -> Option<mp_core::extlibs::utilities::warnings::PropertyValue> {
        use mp_core::extlibs::utilities::warnings::PropertyValue;
        use mp_core::state::properties::PropertyKind;
        let (_, kind) = mp_core::state::properties::PROPERTIES.iter().find(|(n, _)| *n == name)?;
        let cs = self.0.read().ok()?;
        Some(match kind {
            PropertyKind::Number | PropertyKind::Bool => cs.field(name).map(PropertyValue::Number).unwrap_or(PropertyValue::Null),
            PropertyKind::Text => PropertyValue::Text(String::new()),
            PropertyKind::Object => PropertyValue::Object,
        })
    }
}

/// MainV2's `speechEngine` as WarningEngine's ISpeech.
struct AppSpeech;

impl mp_core::extlibs::utilities::ispeech::ISpeech for AppSpeech {
    fn speechEnable(&self) -> bool {
        true
    }
    fn set_speechEnable(&self, _value: bool) {}
    fn is_ready(&self) -> bool {
        true
    }
    fn speak_async(&self, text: &str) {
        crate::speech::say(text.to_string());
    }
    fn speak_async_cancel_all(&self) {}
}

static CS_SNAPSHOT: std::sync::OnceLock<std::sync::Arc<std::sync::RwLock<mp_core::state::CurrentState>>> = std::sync::OnceLock::new();
/// WarningMessage texts waiting to become `cs.messageHigh` on the next frame
static MESSAGE_HIGH: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());

/// MainV2: `CustomWarning.defaultsrc = comPort.MAV.cs;
/// WarningEngine.Start(speechEnable ? speechEngine : null);
/// WarningEngine.WarningMessage += (sender, s) => MAV.cs.messageHigh = s;`
pub fn start_warning_engine(planner: &std::collections::BTreeMap<String, String>) {
    use mp_core::extlibs::utilities::warnings::{CustomWarning, WarningEngine};
    let snap = CS_SNAPSHOT.get_or_init(Default::default).clone();
    CustomWarning::set_defaultsrc(Some(std::sync::Arc::new(CsSource(snap))));
    let speech_enable = planner.get("speechenable").is_some_and(|v| v.eq_ignore_ascii_case("true"));
    WarningEngine::Start(if speech_enable { Some(std::sync::Arc::new(AppSpeech)) } else { None });
    WarningEngine::add_WarningMessage(std::sync::Arc::new(|s: &str| MESSAGE_HIGH.lock().unwrap().push(s.to_string())));
}

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

pub struct ProximityView {
    radius_cm: f32,
    vehicle_cm: f32,
    quad: Option<egui::TextureHandle>,
    focus_on_open: bool,
}

impl Default for ProximityView {
    fn default() -> Self { Self { radius_cm: 500.0, vehicle_cm: 80.0, quad: None, focus_on_open: true } }
}

impl ProximityView {
    pub fn with_dimensions(radius_cm: f32, vehicle_cm: f32) -> Option<Self> {
        if !radius_cm.is_finite() || !vehicle_cm.is_finite() || radius_cm < 1.0 || vehicle_cm < 1.0 { return None; }
        Some(Self { radius_cm, vehicle_cm, ..Self::default() })
    }

    /// Production radar content painter, also used by the deterministic capture example.
    pub fn paint(&mut self, ui: &egui::Ui, rect: Rect, readings: &[mp_core::proximity::Reading], is_copter: bool) {
        let p = ui.painter().with_clip_rect(rect);
        p.rect_filled(rect, 0.0, theme::bg());
        let c = rect.center();
        let max = self.radius_cm / 100.0;
        let scale = rect.width().min(rect.height()) / (2.0 * (max + 0.5));
        let dim_gray = Color32::from_gray(105);
        let green = Color32::from_rgb(0, 128, 0); // System.Drawing.Brushes.Green
        // SystemFonts.DefaultFont + 2 points at 96 DPI. GDI synthesizes the
        // Microsoft Sans Serif bold face; the second pass approximates that weight.
        // Font rasterization is still a known GDI/egui visual-parity limitation.
        let text = |position: Pos2, label: String, color: Color32| {
            let font = crate::fonts::ui_id(10.25 * 96.0 / 72.0);
            let position = position + Vec2::new(1.0, 1.5);
            p.text(position, egui::Align2::LEFT_TOP, &label, font.clone(), color);
            p.text(position + Vec2::new(0.5, 0.0), egui::Align2::LEFT_TOP, label, font, color);
        };
        // Reference Temp_Paint draws the vehicle underneath rings/spokes.
        if is_copter {
            let quad = self.quad.get_or_insert_with(|| {
                let image = image::load_from_memory(include_bytes!("../assets/proximity_quad.png")).expect("bundled quad icon").to_rgba8();
                ui.ctx().load_texture("proximity_quad", egui::ColorImage::from_rgba_unmultiplied([image.width() as usize, image.height() as usize], image.as_raw()), egui::TextureOptions::LINEAR)
            });
            p.image(quad.id(), Rect::from_center_size(c, Vec2::splat(self.vehicle_cm / 100.0 * scale)), Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
        }
        for k in 1..=(self.radius_cm / 50.0) as usize {
            let meters = k as f32 * 0.5;
            let r = meters * scale;
            p.circle_stroke(c, r, Stroke::new(1.0, dim_gray));
            text(c + Vec2::new(5.0, -r), format!("{meters:.1}m"), green);
        }
        for angle in (0..360).step_by(45) {
            let rad = (angle as f32 - 90.0).to_radians();
            let end = c + Vec2::new(rad.cos(), rad.sin()) * max * scale;
            text(end + Vec2::new(-10.0, 5.0), angle.to_string(), dim_gray);
            p.line_segment([c, end], Stroke::new(1.0, dim_gray));
        }
        for reading in readings {
            let r = reading.distance_m.min(max) * scale;
            let a0 = (reading.angle_deg - reading.width_deg / 2.0 - 90.0).to_radians();
            let a1 = (reading.angle_deg + reading.width_deg / 2.0 - 90.0).to_radians();
            // Keep chords below two physical pixels for wide obstacle-array arcs.
            let segments = (((a1 - a0).abs() * r * ui.ctx().pixels_per_point() / 2.0).ceil() as usize).clamp(8, 4096);
            let pts: Vec<Pos2> = (0..=segments).map(|i| {
                let a = a0 + (a1 - a0) * i as f32 / segments as f32;
                c + Vec2::new(a.cos(), a.sin()) * r
            }).collect();
            if reading.orientation != 100 {
                let angle = (reading.angle_deg - 90.0).to_radians();
                let offset = match (reading.angle_deg / 45.0).round() as u8 {
                    0 => Vec2::new(-10.0, 5.0), 1 => Vec2::new(-40.0, 5.0),
                    2 => Vec2::new(-40.0, 0.0), 3 => Vec2::new(-40.0, -5.0),
                    4 => Vec2::new(-10.0, -15.0), 5 => Vec2::new(5.0, -5.0),
                    6 => Vec2::new(5.0, 0.0), _ => Vec2::new(5.0, 5.0),
                };
                text(c + Vec2::new(angle.cos(), angle.sin()) * r + offset, format!("{:.1}m", reading.distance_m), green);
            }
            let col = if reading.orientation == 100 { Color32::YELLOW } else { Color32::RED };
            p.add(Shape::line(pts, Stroke::new(3.0, col)));
        }
    }

    fn key(&mut self, c: char) {
        match c {
            '+' => self.radius_cm -= 50.0,
            '-' => self.radius_cm += 50.0,
            ']' => self.vehicle_cm += 1.0,
            '[' => self.vehicle_cm -= 1.0,
            _ => {}
        }
        if self.radius_cm < 1.0 { self.radius_cm = 50.0; }
        if self.vehicle_cm < 1.0 { self.vehicle_cm = 1.0; }
    }
}

enum BinaryTool { Fft(bool), MagCal, Split }

#[derive(Default)]
pub struct Tools {
    prompt: Option<Prompt>,
    binary_request: Option<(BinaryTool, crate::platform::FilePicker)>,
    confirm: Option<Confirm>,
    text: Option<(String, String)>,
    outputs: Vec<Output>,
    out_window: Option<OutKind>,
    out_target: String,
    /// `new WarningsManager().Show()`: each click opens another form
    warning_managers: Vec<crate::mp::warnings::WarningsManager>,
    warning_manager_count: u64,
    mirror_open: bool,
    mirror_target: String,
    mirror_write: bool,
    mirror_on: bool,
    follow: Option<(UdpSocket, Instant, bool)>,
    follow_open: Option<bool>,
    follow_port: String,
    last_fix: Option<(f64, f64, f64)>,
    prox_open: bool,
    prox_view: ProximityView,
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
            "Warning Manager" => {
                self.warning_manager_count += 1;
                match crate::mp::warnings::WarningsManager::new(self.warning_manager_count) {
                    Ok(m) => self.warning_managers.push(m),
                    Err(e) => toasts.error(e.to_string()),
                }
            }
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
            "Proximity" => { self.prox_open = true; self.prox_view.focus_on_open = true; },
            "signing" | "Mavlink Signing" => self.sign_open = true,
            "FFT" => self.load_fft(false, toasts),
            "Spectrogram" => self.load_fft(true, toasts),
            "Message Interval" => self.prompt = Some(Prompt::MsgInterval { name: "ATTITUDE".into(), rate: "10".into() }),
            "hex Mavlink decode" | "parse packet bytes" => self.prompt = Some(Prompt::Hex("fd0500001a0c1".into())),
            "decode HWIDs" => self.prompt = Some(Prompt::HwId(String::new())),
            "Split DFLog" => {
                self.binary_request = Some((BinaryTool::Split, crate::platform::binary_log_picker()));
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
                    link.unwrap().send(LinkCommand::CommandLong { command: 246, params: [42.0, 24.0, 71.0, 99.0, 0.0, 0.0, 0.0] });
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
                        let list = match mp_core::paramfile::read(&p) { Ok(list) => list, Err(e) => { toasts.error(e.to_string()); return true; } };
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

    fn load_fft(&mut self, spect: bool, _toasts: &mut Toasts) {
        self.binary_request = Some((BinaryTool::Fft(spect), crate::platform::binary_log_picker()));
    }

    fn load_fft_path(&mut self, spect: bool, p: PathBuf, toasts: &mut Toasts) {
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
    fn mag_log(&mut self, _toasts: &mut Toasts) {
        self.binary_request = Some((BinaryTool::MagCal, crate::platform::binary_log_picker()));
    }

    fn mag_log_path(&mut self, p: PathBuf, toasts: &mut Toasts) {
        let data = match std::fs::read(&p) { Ok(data) => data, Err(e) => { toasts.error(e.to_string()); return; } };
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
        if let Some((action, path)) = crate::platform::poll_file_action(&mut self.binary_request, ctx) {
            match action {
                BinaryTool::Fft(spect) => self.load_fft_path(spect, path, toasts),
                BinaryTool::MagCal => self.mag_log_path(path, toasts),
                BinaryTool::Split => self.prompt = Some(Prompt::Split(path, "2".into())),
            }
        }
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
        self.warn_ui(ctx, &cs, toasts);
        self.mirror_ui(ctx, link);
        self.follow_ui(ctx);
        self.prox_ui(ctx, link);
        self.sign_ui(ctx, link, toasts);
        self.fft_ui(ctx);
    }

    fn tick(&mut self, link: Option<&Link>, cs: &CurrentState, _toasts: &mut Toasts) {
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
        // WarningEngine reads defaultsrc; its WarningMessage sets messageHigh
        if let Some(snap) = CS_SNAPSHOT.get() {
            if let Ok(mut s) = snap.write() {
                *s = cs.clone();
            }
        }
        let pending: Vec<String> = std::mem::take(&mut *MESSAGE_HIGH.lock().unwrap());
        if let Some(l) = link {
            for m in &pending {
                l.shared.write().cs.set_message_high(m);
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
                let sock = UdpSocket::bind((mp_core::comms::listen_ip(), 0)).ok();
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

    /// ConfigAdvanced's "Warning Manager": every open `WarningsManager` form.
    fn warn_ui(&mut self, ctx: &egui::Context, _cs: &CurrentState, toasts: &mut Toasts) {
        self.warning_managers.retain_mut(|m| m.show(ctx, toasts));
    }

    fn mirror_ui(&mut self, ctx: &egui::Context, link: Option<&Link>) {
        if !self.mirror_open {
            return;
        }
        let mut open = true;
        egui::Window::new(crate::i18n::t("Mavlink Mirror")).open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            theme::label(ui, "UDP destination (host:port)");
            ui.add_enabled(!self.mirror_on, egui::TextEdit::singleline(&mut self.mirror_target));
            ui.add_enabled(!self.mirror_on, egui::Checkbox::new(&mut self.mirror_write, crate::i18n::t("Write access")));
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
            } else if let Ok(s) = UdpSocket::bind(format!("{}:{}", mp_core::comms::listen_ip(), self.follow_port.trim())) {
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
        let readings = link.map(|l| {
            let state = l.shared.read();
            state.proximity.readings(state.cs.sysid, state.cs.compid, Instant::now())
        }).unwrap_or_default();
        let mut open = true;
        let title = format!("Radius(+/-): {}m MAV size([/]): {}m", self.prox_view.radius_cm / 100.0, self.prox_view.vehicle_cm / 100.0);
        egui::Window::new(title).id(egui::Id::new("proximity_window")).open(&mut open).collapsible(false).default_size([430.0, 391.0]).show(ctx, |ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size().max(Vec2::splat(120.0)), egui::Sense::hover());
            let response = ui.interact(rect, egui::Id::new("proximity_keyboard"), egui::Sense::click());
            if response.clicked() || self.prox_view.focus_on_open {
                response.request_focus();
                self.prox_view.focus_on_open = false;
            }
            if response.has_focus() {
                for event in ui.input(|i| i.events.clone()) {
                    if let egui::Event::Text(text) = event { for c in text.chars() { self.prox_view.key(c); } }
                }
            }
            let is_copter = link.is_some_and(|l| l.shared.read().cs.firmware == mp_core::firmware::Firmware::ArduCopter2);
            self.prox_view.paint(ui, rect, &readings, is_copter);
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
        egui::Window::new(crate::i18n::t("Mavlink Signing")).open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
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
                p.text(Pos2::new(chart.center().x, rect.bottom() - 2.0), egui::Align2::CENTER_BOTTOM, crate::i18n::t("Hz"), egui::FontId::proportional(10.0), Color32::WHITE);
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
                p.text(Pos2::new(chart.center().x, rect.bottom() - 2.0), egui::Align2::CENTER_BOTTOM, crate::i18n::t("time (IMU AccX)"), egui::FontId::proportional(10.0), Color32::WHITE);
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

#[cfg(test)]
mod proximity_view_tests {
    use super::*;

    #[test]
    fn reference_range_and_vehicle_size_controls_preserve_lower_bounds() {
        let mut view = ProximityView::default();
        assert_eq!((view.radius_cm, view.vehicle_cm), (500.0, 80.0));
        for c in "+]".chars() { view.key(c); }
        assert_eq!((view.radius_cm, view.vehicle_cm), (450.0, 81.0));
        for c in "-[".chars() { view.key(c); }
        assert_eq!((view.radius_cm, view.vehicle_cm), (500.0, 80.0));
        for _ in 0..1000 { view.key('+'); view.key('['); }
        assert_eq!((view.radius_cm, view.vehicle_cm), (50.0, 1.0));
    }

    #[test]
    fn radar_window_draws_at_small_and_large_viewports() {
        for size in [Vec2::new(320.0, 480.0), Vec2::new(1200.0, 900.0)] {
            let ctx = egui::Context::default();
            let mut tools = Tools::default();
            tools.prox_open = true;
            let mut frame = ctx.run_ui(egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, size)), ..Default::default() }, |ui| tools.prox_ui(ui.ctx(), None));
            assert!(!frame.shapes.is_empty());
            frame.textures_delta.clear();
        }
    }

    #[test]
    fn radar_click_focus_routes_real_text_events_to_controls() {
        let ctx = egui::Context::default();
        let mut tools = Tools::default();
        tools.prox_open = true;
        let input = || egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 800.0))), ..Default::default() };
        let mut frame = ctx.run_ui(input(), |ui| tools.prox_ui(ui.ctx(), None));
        frame.textures_delta.clear();
        for _ in 0..3 {
            let mut frame = ctx.run_ui(input(), |ui| tools.prox_ui(ui.ctx(), None));
            frame.textures_delta.clear();
        }
        let center = ctx.read_response(egui::Id::new("proximity_keyboard")).unwrap().rect.center();
        for pressed in [true, false] {
            let mut event = input();
            event.events = vec![egui::Event::PointerMoved(center), egui::Event::PointerButton { pos: center, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE }];
            let mut frame = ctx.run_ui(event, |ui| tools.prox_ui(ui.ctx(), None));
            frame.textures_delta.clear();
        }
        let mut event = input();
        event.events = vec![egui::Event::Text("+]".into())];
        let mut frame = ctx.run_ui(event, |ui| tools.prox_ui(ui.ctx(), None));
        frame.textures_delta.clear();
        assert_eq!((tools.prox_view.radius_cm, tools.prox_view.vehicle_cm), (450.0, 81.0));
        let mut other_text = String::new();
        let mut frame = ctx.run_ui(input(), |ui| {
            tools.prox_ui(ui.ctx(), None);
            ui.text_edit_singleline(&mut other_text).request_focus();
        });
        frame.textures_delta.clear();
        let mut event = input();
        event.events = vec![egui::Event::Text("+]".into())];
        let mut frame = ctx.run_ui(event, |ui| {
            tools.prox_ui(ui.ctx(), None);
            ui.text_edit_singleline(&mut other_text);
        });
        frame.textures_delta.clear();
        assert_eq!(other_text, "+]");
        assert_eq!((tools.prox_view.radius_cm, tools.prox_view.vehicle_cm), (450.0, 81.0), "radar shortcuts must not consume other input focus");
        tools.prox_open = false;
        let mut frame = ctx.run_ui(input(), |ui| tools.prox_ui(ui.ctx(), None));
        frame.textures_delta.clear();
        tools.activate("Proximity", None, &mut Toasts::default());
        let mut frame = ctx.run_ui(input(), |ui| tools.prox_ui(ui.ctx(), None));
        frame.textures_delta.clear();
        let mut event = input();
        event.events = vec![egui::Event::Text("-[".into())];
        let mut frame = ctx.run_ui(event, |ui| tools.prox_ui(ui.ctx(), None));
        frame.textures_delta.clear();
        assert_eq!((tools.prox_view.radius_cm, tools.prox_view.vehicle_cm), (500.0, 80.0), "opening radar restores keyboard controls without a content click");
    }
}

