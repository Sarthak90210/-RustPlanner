//! GimbalVideoControl (Controls/GimbalVideoControl.cs) and its placement in Flight Data
//! (FlightData.cs:6666-6845): a GStreamer camera stream with gimbal and camera commands on the
//! context menu, the keyboard and the mouse, GimbalControlSettingsForm key bindings, and the
//! Full Sized / Mini / Pop Out layouts.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use egui::{Color32, Pos2, Rect, Sense, Stroke, Vec2};
use mp_core::gimbal::{self, Camera, Cmd, GimbalManager};
use mp_core::link::LinkCommand;
use serde::{Deserialize, Serialize};

use crate::app::AppCtx;
use crate::theme;

/// System.Windows.Forms.Keys / MouseButtons values, as GimbalControlPreferences stores them.
pub mod keys {
    pub const NONE: u32 = 0;
    pub const SHIFT: u32 = 0x10000;
    pub const CONTROL: u32 = 0x20000;
    pub const ALT: u32 = 0x40000;
    pub const MODIFIERS: u32 = SHIFT | CONTROL | ALT;
    pub const SHIFT_KEY: u32 = 16;
    pub const CONTROL_KEY: u32 = 17;
    pub const MENU: u32 = 18;
    pub const MOUSE_LEFT: u32 = 0x100000;
    pub const MOUSE_RIGHT: u32 = 0x200000;
    pub const MOUSE_MIDDLE: u32 = 0x400000;
    pub const MOUSE_X1: u32 = 0x800000;
    pub const MOUSE_X2: u32 = 0x1000000;
}

/// Keys code for an egui key.
pub fn win_key(k: egui::Key) -> Option<u32> {
    use egui::Key::*;
    let name = k.name();
    if name.len() == 1 && name.as_bytes()[0].is_ascii_uppercase() {
        return Some(name.as_bytes()[0] as u32);
    }
    Some(match k {
        Num0 => 48, Num1 => 49, Num2 => 50, Num3 => 51, Num4 => 52, Num5 => 53, Num6 => 54, Num7 => 55, Num8 => 56, Num9 => 57,
        F1 => 112, F2 => 113, F3 => 114, F4 => 115, F5 => 116, F6 => 117, F7 => 118, F8 => 119, F9 => 120, F10 => 121, F11 => 122, F12 => 123,
        ArrowLeft => 37, ArrowUp => 38, ArrowRight => 39, ArrowDown => 40,
        Space => 32, Enter => 13, Tab => 9, Backspace => 8, Escape => 27, Insert => 45, Delete => 46, Home => 36, End => 35, PageUp => 33, PageDown => 34,
        Minus => 189, Plus | Equals => 187, Comma => 188, Period => 190, Slash => 191, Semicolon => 186, OpenBracket => 219, CloseBracket => 221, Backslash => 220, Quote => 222, Backtick => 192,
        _ => return None,
    })
}

/// Keys.ToString() for a key code without modifiers.
fn key_name(code: u32) -> String {
    match code {
        65..=90 => (code as u8 as char).to_string(),
        48..=57 => format!("D{}", code - 48),
        112..=123 => format!("F{}", code - 111),
        37 => "Left".into(), 38 => "Up".into(), 39 => "Right".into(), 40 => "Down".into(),
        32 => "Space".into(), 13 => "Return".into(), 9 => "Tab".into(), 8 => "Back".into(), 27 => "Escape".into(),
        45 => "Insert".into(), 46 => "Delete".into(), 36 => "Home".into(), 35 => "End".into(), 33 => "PageUp".into(), 34 => "Next".into(),
        189 => "OemMinus".into(), 187 => "Oemplus".into(), 188 => "Oemcomma".into(), 190 => "OemPeriod".into(), 191 => "OemQuestion".into(),
        186 => "OemSemicolon".into(), 219 => "OemOpenBrackets".into(), 221 => "OemCloseBrackets".into(), 220 => "OemPipe".into(), 222 => "OemQuotes".into(), 192 => "Oemtilde".into(),
        16 => "ShiftKey".into(), 17 => "ControlKey".into(), 18 => "Menu".into(),
        c => c.to_string(),
    }
}

/// KeyBindingButton.GetModifiersString
fn modifiers_string(k: u32) -> String {
    let mut s = String::new();
    if k & keys::SHIFT != 0 {
        s += "Shift + ";
    }
    if k & keys::CONTROL != 0 {
        s += "Ctrl + ";
    }
    if k & keys::ALT != 0 {
        s += "Alt + ";
    }
    s
}

/// KeyBindingButton.GetKeyString
pub fn key_string(k: u32, modifiers_only: bool) -> String {
    let m = modifiers_string(k);
    if modifiers_only {
        return m;
    }
    let key = k & !keys::MODIFIERS;
    if key == keys::NONE {
        return "None".into();
    }
    m + &key_name(key)
}

/// ClickBindingButton.GetClickBindingString
pub fn click_string(b: (u32, u32)) -> String {
    let button = match b.1 {
        keys::MOUSE_LEFT => "Left",
        keys::MOUSE_RIGHT => "Right",
        keys::MOUSE_MIDDLE => "Middle",
        keys::MOUSE_X1 => "XButton1",
        keys::MOUSE_X2 => "XButton2",
        _ => return "None".into(),
    };
    format!("{}{button} Click", modifiers_string(b.0))
}

fn modifier_bits(m: egui::Modifiers) -> u32 {
    (if m.shift { keys::SHIFT } else { 0 }) | (if m.ctrl || m.command { keys::CONTROL } else { 0 }) | (if m.alt { keys::ALT } else { 0 })
}

fn mouse_bits(b: egui::PointerButton) -> u32 {
    match b {
        egui::PointerButton::Primary => keys::MOUSE_LEFT,
        egui::PointerButton::Secondary => keys::MOUSE_RIGHT,
        egui::PointerButton::Middle => keys::MOUSE_MIDDLE,
        egui::PointerButton::Extra1 => keys::MOUSE_X1,
        egui::PointerButton::Extra2 => keys::MOUSE_X2,
    }
}

/// A ValueTuple<Keys, MouseButtons> as Json.NET writes it.
#[derive(Clone, Copy, PartialEq, Debug, Serialize, Deserialize)]
pub struct Click {
    #[serde(rename = "Item1")]
    pub modifiers: u32,
    #[serde(rename = "Item2")]
    pub button: u32,
}

impl Click {
    const fn new(modifiers: u32, button: u32) -> Self {
        Self { modifiers, button }
    }
}

/// GimbalControlSettings, stored as JSON in Settings "GimbalControlPreferences".
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase", default)]
pub struct Prefs {
    pub slew_left: u32,
    pub slew_right: u32,
    pub slew_up: u32,
    pub slew_down: u32,
    pub zoom_in: u32,
    pub zoom_out: u32,
    pub slew_fast_modifier: u32,
    pub slew_slow_modifier: u32,
    pub take_picture: u32,
    pub toggle_recording: u32,
    pub start_recording: u32,
    pub stop_recording: u32,
    pub toggle_lock_follow: u32,
    pub set_lock: u32,
    pub set_follow: u32,
    pub retract: u32,
    pub neutral: u32,
    pub point_down: u32,
    pub home: u32,
    pub move_camera_to_mouse_location: Click,
    #[serde(rename = "MoveCameraPOIToMouseLocation")]
    pub move_camera_poi_to_mouse_location: Click,
    pub track_object_under_mouse: Click,
    pub slew_speed_slow: f64,
    pub slew_speed_normal: f64,
    pub slew_speed_fast: f64,
    pub zoom_speed: f64,
    #[serde(rename = "CameraHFOV")]
    pub camera_hfov: f64,
    #[serde(rename = "CameraVFOV")]
    pub camera_vfov: f64,
    #[serde(rename = "UseFOVReportedByCamera")]
    pub use_fov_reported_by_camera: bool,
    pub default_locked_mode: bool,
}

impl Default for Prefs {
    fn default() -> Self {
        Self {
            slew_left: 'A' as u32,
            slew_right: 'D' as u32,
            slew_up: 'W' as u32,
            slew_down: 'S' as u32,
            zoom_in: 'E' as u32,
            zoom_out: 'Q' as u32,
            slew_slow_modifier: keys::CONTROL,
            slew_fast_modifier: keys::SHIFT,
            take_picture: keys::ALT | 'F' as u32,
            toggle_recording: keys::ALT | 'R' as u32,
            start_recording: keys::NONE,
            stop_recording: keys::NONE,
            toggle_lock_follow: 'L' as u32,
            set_lock: keys::NONE,
            set_follow: keys::NONE,
            retract: keys::NONE,
            neutral: 'N' as u32,
            point_down: keys::NONE,
            home: 'H' as u32,
            move_camera_to_mouse_location: Click::new(keys::NONE, keys::MOUSE_LEFT),
            move_camera_poi_to_mouse_location: Click::new(keys::CONTROL, keys::MOUSE_LEFT),
            track_object_under_mouse: Click::new(keys::ALT, keys::MOUSE_LEFT),
            slew_speed_slow: 1.0,
            slew_speed_normal: 5.0,
            slew_speed_fast: 25.0,
            zoom_speed: 1.0,
            camera_hfov: 40.0,
            camera_vfov: 30.0,
            use_fov_reported_by_camera: true,
            default_locked_mode: false,
        }
    }
}

/// The settings form's rows, in GimbalControlSettings property order.
enum Row {
    Key(&'static str, fn(&mut Prefs) -> &mut u32),
    Modifier(&'static str, fn(&mut Prefs) -> &mut u32),
    Click(&'static str, fn(&mut Prefs) -> &mut Click),
    Decimal(&'static str, fn(&mut Prefs) -> &mut f64, f64, f64, f64, usize),
    Check(&'static str, fn(&mut Prefs) -> &mut bool),
}

const ROWS: &[Row] = &[
    Row::Key("Slew Left", |p| &mut p.slew_left),
    Row::Key("Slew Right", |p| &mut p.slew_right),
    Row::Key("Slew Up", |p| &mut p.slew_up),
    Row::Key("Slew Down", |p| &mut p.slew_down),
    Row::Key("Zoom In", |p| &mut p.zoom_in),
    Row::Key("Zoom Out", |p| &mut p.zoom_out),
    Row::Modifier("Slew Fast Modifier", |p| &mut p.slew_fast_modifier),
    Row::Modifier("Slew Slow Modifier", |p| &mut p.slew_slow_modifier),
    Row::Key("Take Picture", |p| &mut p.take_picture),
    Row::Key("Toggle Recording", |p| &mut p.toggle_recording),
    Row::Key("Start Recording", |p| &mut p.start_recording),
    Row::Key("Stop Recording", |p| &mut p.stop_recording),
    Row::Key("Toggle Lock/Follow", |p| &mut p.toggle_lock_follow),
    Row::Key("Set Lock", |p| &mut p.set_lock),
    Row::Key("Set Follow", |p| &mut p.set_follow),
    Row::Key("Retract", |p| &mut p.retract),
    Row::Key("Neutral", |p| &mut p.neutral),
    Row::Key("Point Down", |p| &mut p.point_down),
    Row::Key("Home", |p| &mut p.home),
    Row::Click("Click Pan/Tilt", |p| &mut p.move_camera_to_mouse_location),
    Row::Click("Click Point-of-Interest", |p| &mut p.move_camera_poi_to_mouse_location),
    Row::Click("Click Track Object", |p| &mut p.track_object_under_mouse),
    Row::Decimal("Slew Speed Slow (deg/s)", |p| &mut p.slew_speed_slow, 0.1, 360.0, 1.0, 1),
    Row::Decimal("Slew Speed Normal (deg/s)", |p| &mut p.slew_speed_normal, 0.1, 360.0, 1.0, 1),
    Row::Decimal("Slew Speed Fast (deg/s)", |p| &mut p.slew_speed_fast, 0.1, 360.0, 1.0, 1),
    Row::Decimal("Zoom Speed (unitless)", |p| &mut p.zoom_speed, 0.01, 1.0, 0.1, 2),
    Row::Decimal("Camera Horizontal FOV (deg)", |p| &mut p.camera_hfov, 0.01, 180.0, 1.0, 2),
    Row::Decimal("Camera Vertical FOV (deg)", |p| &mut p.camera_vfov, 0.01, 180.0, 1.0, 2),
    Row::Check("Use FOV Reported by Camera", |p| &mut p.use_fov_reported_by_camera),
    Row::Check("Default Locked Mode", |p| &mut p.default_locked_mode),
];

impl Prefs {
    /// loadPreferences: invalid JSON falls back to the defaults.
    pub fn load(planner: &std::collections::BTreeMap<String, String>) -> Self {
        planner
            .get("GimbalControlPreferences")
            .filter(|j| !j.is_empty())
            .and_then(|j| match serde_json::from_str(j) {
                Ok(p) => Some(p),
                Err(e) => {
                    log::error!("Invalid GimbalControlPreferences, reverting to default {e}");
                    None
                }
            })
            .unwrap_or_default()
    }

    fn press_keys(&self) -> [u32; 11] {
        [self.take_picture, self.toggle_recording, self.start_recording, self.stop_recording, self.toggle_lock_follow, self.set_lock, self.set_follow, self.retract, self.neutral, self.point_down, self.home]
    }

    fn mod_key(m: u32) -> u32 {
        match m {
            keys::SHIFT => keys::SHIFT_KEY,
            keys::CONTROL => keys::CONTROL_KEY,
            keys::ALT => keys::MENU,
            _ => keys::NONE,
        }
    }

    fn hold_keys(&self) -> [u32; 8] {
        [self.slew_left, self.slew_right, self.slew_up, self.slew_down, self.zoom_in, self.zoom_out, Self::mod_key(self.slew_fast_modifier), Self::mod_key(self.slew_slow_modifier)]
    }

    /// HandleHeldKeys: (pitch rate, yaw rate, zoom rate) for the held keys and modifiers.
    pub fn rates(&self, held: &BTreeSet<u32>, modifiers: u32) -> (f32, f32, f32) {
        let mut pitch = 0.0f32;
        let mut yaw = 0.0f32;
        if held.contains(&self.slew_down) {
            pitch -= 1.0;
        }
        if held.contains(&self.slew_up) {
            pitch += 1.0;
        }
        if held.contains(&self.slew_left) {
            yaw -= 1.0;
        }
        if held.contains(&self.slew_right) {
            yaw += 1.0;
        }
        let speed = if modifiers == self.slew_fast_modifier {
            self.slew_speed_fast
        } else if modifiers == self.slew_slow_modifier {
            self.slew_speed_slow
        } else {
            self.slew_speed_normal
        } as f32;
        let mut zoom = 0.0f32;
        if held.contains(&self.zoom_in) {
            zoom += 1.0;
        }
        if held.contains(&self.zoom_out) {
            zoom -= 1.0;
        }
        (pitch * speed, yaw * speed, zoom * self.zoom_speed as f32)
    }

    /// GetClashes: the labels of other rows of the same kind already using `binding`.
    fn clashes(&mut self, row: usize, binding: Binding) -> Vec<&'static str> {
        let mut out = Vec::new();
        for (i, r) in ROWS.iter().enumerate() {
            if i == row {
                continue;
            }
            match (r, binding) {
                (Row::Key(l, f) | Row::Modifier(l, f), Binding::Key(k)) if *f(self) == k => out.push(*l),
                (Row::Click(l, f), Binding::Click(c)) if *f(self) == c => out.push(*l),
                _ => {}
            }
        }
        out
    }

    fn set(&mut self, row: usize, binding: Binding) {
        match (&ROWS[row], binding) {
            (Row::Key(_, f) | Row::Modifier(_, f), Binding::Key(k)) => *f(self) = k,
            (Row::Click(_, f), Binding::Click(c)) => *f(self) = c,
            _ => {}
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Binding {
    Key(u32),
    Click(Click),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum GimbalPlacement {
    #[default]
    Closed,
    FullSized,
    Mini,
    PopOut,
}

/// GimbalControlSettingsForm
struct SettingsForm {
    prefs: Prefs,
    /// the binding button waiting for a key or click
    listening: Option<usize>,
    /// "Key Binding Clash" Yes/No: row, binding, message
    clash: Option<(usize, Binding, String)>,
}

/// VideoStreamSelector
struct Selector {
    streams: Vec<gimbal::VideoStream>,
    names: Vec<String>,
    sel: usize,
    raw: String,
}

type Download = (Arc<Mutex<(i32, String)>>, Arc<AtomicBool>, Arc<AtomicBool>);

pub struct GimbalVideo {
    pub placement: GimbalPlacement,
    /// gimbalVideoShowMiniMap (CheckOnClick, starts checked)
    pub show_mini_map: bool,
    prefs: Prefs,
    yaw_lock: bool,
    is_recording: bool,
    gst: Option<crate::mjpeg::Mjpeg>,
    /// initializeGStreamer succeeded: the stream and AutoConnectTimer run
    gst_ready: bool,
    auto_connect_at: Option<Instant>,
    settings_camera_address: Option<(u8, u8)>,
    held: BTreeSet<u32>,
    last_modifiers: u32,
    previous: (f32, f32, f32),
    drag_start: Option<(f64, f64)>,
    drag_end: Option<(f64, f64)>,
    last_mouse_move: Option<Instant>,
    /// mouseMapMarker: where the pointer's image point meets the ground, for the FD map
    pub mouse_marker: Option<(f64, f64)>,
    gst_prompt: bool,
    download: Option<Download>,
    selector: Option<Selector>,
    settings: Option<SettingsForm>,
}

impl Default for GimbalVideo {
    fn default() -> Self {
        Self {
            placement: GimbalPlacement::Closed,
            show_mini_map: true,
            prefs: Prefs::default(),
            yaw_lock: false,
            is_recording: false,
            gst: None,
            gst_ready: false,
            auto_connect_at: None,
            settings_camera_address: None,
            held: BTreeSet::new(),
            last_modifiers: 0,
            previous: (0.0, 0.0, 0.0),
            drag_start: None,
            drag_end: None,
            last_mouse_move: None,
            mouse_marker: None,
            gst_prompt: false,
            download: None,
            selector: None,
            settings: None,
        }
    }
}

fn send(app: &AppCtx, cmd: Option<Cmd>) {
    if let (Some(l), Some((command, params))) = (app.link, cmd) {
        l.send(LinkCommand::CommandLong { command, params });
    }
}

fn gimbal_manager(app: &AppCtx) -> Option<(GimbalManager, f64)> {
    app.link.map(|l| {
        let s = l.shared.read();
        (s.gimbal.clone(), s.cs.yaw as f64)
    })
}

impl GimbalVideo {
    /// The selected camera with UITimer's preference overrides.
    fn camera(&self, app: &AppCtx) -> Option<Camera> {
        let l = app.link?;
        let mut c = l.shared.read().camera.clone();
        c.hfov_setting = self.prefs.camera_hfov as f32;
        c.vfov_setting = self.prefs.camera_vfov as f32;
        c.use_fov_status = self.prefs.use_fov_reported_by_camera;
        Some(c)
    }

    /// The `gimbalVideoControl` getter: a new control the first time it is shown after a Close.
    fn construct(&mut self, app: &mut AppCtx) {
        self.prefs = Prefs::load(&app.settings.planner);
        self.yaw_lock = self.prefs.default_locked_mode;
        self.held.clear();
        self.previous = (0.0, 0.0, 0.0);
        if crate::gstreamer::look_for_gstreamer(&crate::platform::data_dir()).is_some() {
            self.gst_ready = true;
            self.auto_connect_at = Some(Instant::now() + Duration::from_secs(1));
        } else {
            // "This feature requires GStreamer. Would you like to download and install it now?"
            self.gst_prompt = true;
        }
    }

    pub fn set_placement(&mut self, p: GimbalPlacement, app: &mut AppCtx, _ctx: &egui::Context) {
        if self.placement == GimbalPlacement::Closed && p != GimbalPlacement::Closed {
            self.construct(app);
        }
        if p == GimbalPlacement::Closed {
            self.stop();
        }
        self.placement = p;
    }

    pub fn stop(&mut self) {
        self.gst = None;
        self.gst_ready = false;
        self.auto_connect_at = None;
        self.mouse_marker = None;
        self.gst_prompt = false;
        self.download = None;
        self.selector = None;
        self.settings = None;
    }

    fn start(&mut self, pipeline: &str, ctx: &egui::Context) {
        if let Some(lib) = crate::gstreamer::look_for_gstreamer(&crate::platform::data_dir()) {
            self.gst = Some(crate::mjpeg::Mjpeg::gstreamer(lib, pipeline, ctx));
        }
    }

    /// AutoConnectTimerCallback
    fn auto_connect(&mut self, app: &mut AppCtx, ctx: &egui::Context) {
        let Some(at) = self.auto_connect_at else { return };
        if Instant::now() < at {
            ctx.request_repaint_after(at - Instant::now());
            return;
        }
        self.auto_connect_at = None;
        let streams: Vec<gimbal::VideoStream> = app.link.map(|l| l.shared.read().video_streams.values().cloned().collect()).unwrap_or_default();
        if streams.is_empty() {
            // "Requesting camera information..." and try again in a second
            if let Some(l) = app.link {
                for (command, params) in Camera::request_information() {
                    l.send(LinkCommand::CommandNoAck { command, params });
                }
            }
            self.auto_connect_at = Some(Instant::now() + Duration::from_secs(1));
            return;
        }
        let previous = app.settings.planner.get("gimbal_video_stream").cloned().unwrap_or_default();
        let stream = match streams.iter().find(|s| s.uri == previous) {
            Some(s) => s.clone(),
            None => {
                let first = streams[0].clone();
                app.settings.planner.insert("gimbal_video_stream".into(), first.uri.clone());
                first
            }
        };
        self.settings_camera_address = Some((stream.sysid, stream.compid));
        self.start(&gimbal::gstreamer_pipeline(&stream), ctx);
    }

    fn set_yaw_lock(&mut self, locked: bool, app: &AppCtx) {
        self.yaw_lock = locked;
        let (p, y, _) = self.previous;
        send(app, gimbal_manager(app).and_then(|(g, _)| g.set_rates(p, y, self.yaw_lock, 0)));
    }

    fn take_picture(&self, app: &AppCtx) {
        if let Some(l) = app.link {
            let (command, params) = l.shared.write().camera.take_picture();
            l.send(LinkCommand::CommandLong { command, params });
        }
    }

    fn set_recording(&mut self, start: bool, app: &AppCtx) {
        self.is_recording = start;
        send(app, Some(if start { Camera::start_recording() } else { Camera::stop_recording() }));
    }

    fn point_down(&self, app: &AppCtx) {
        send(app, gimbal_manager(app).and_then(|(g, _)| g.set_angles(-90.0, 0.0, false, 0)));
    }

    /// Home: ROI at the home location
    fn point_home(&self, app: &AppCtx) {
        let Some(l) = app.link else { return };
        let (g, home) = {
            let s = l.shared.read();
            (s.gimbal.clone(), s.cs.home)
        };
        let h = home.unwrap_or_default();
        if let Some(c) = g.roi_location(h.lat, h.lng, h.alt) {
            l.send(LinkCommand::CommandInt(c));
        }
    }

    /// HandleKeyPress
    fn key_press(&mut self, key: u32, app: &AppCtx) {
        let p = self.prefs.clone();
        if key == p.take_picture {
            self.take_picture(app);
        }
        if key == p.toggle_recording {
            self.set_recording(!self.is_recording, app);
        }
        if key == p.start_recording {
            self.set_recording(true, app);
        }
        if key == p.stop_recording {
            self.set_recording(false, app);
        }
        if key == p.toggle_lock_follow {
            self.set_yaw_lock(!self.yaw_lock, app);
        }
        if key == p.set_lock {
            self.set_yaw_lock(true, app);
        }
        if key == p.set_follow {
            self.set_yaw_lock(false, app);
        }
        if key == p.retract {
            send(app, gimbal_manager(app).and_then(|(g, _)| g.retract()));
        }
        if key == p.neutral {
            send(app, gimbal_manager(app).and_then(|(g, _)| g.neutral()));
        }
        if key == p.point_down {
            self.point_down(app);
        }
        if key == p.home {
            self.point_home(app);
        }
    }

    /// HandleHeldKeys: rate and zoom commands when they change.
    fn held_keys(&mut self, modifiers: u32, app: &AppCtx) {
        let (pitch, yaw, zoom) = self.prefs.rates(&self.held, modifiers);
        if pitch != self.previous.0 || yaw != self.previous.1 {
            self.previous.0 = pitch;
            self.previous.1 = yaw;
            send(app, gimbal_manager(app).and_then(|(g, _)| g.set_rates(pitch, yaw, self.yaw_lock, 0)));
        }
        if zoom != self.previous.2 {
            self.previous.2 = zoom;
            send(app, Some(Camera::zoom(zoom, gimbal::ZOOM_TYPE_CONTINUOUS)));
        }
    }

    /// PreFilterMessage: bound keys while the control's window has focus and no text box does.
    fn keyboard(&mut self, ctx: &egui::Context, app: &AppCtx) {
        if self.settings.as_ref().is_some_and(|s| s.listening.is_some()) || ctx.egui_wants_keyboard_input() || !ctx.input(|i| i.focused) {
            if !self.held.is_empty() {
                self.held.clear();
                self.held_keys(self.last_modifiers, app);
            }
            return;
        }
        let hold = self.prefs.hold_keys();
        let press = self.prefs.press_keys();
        let events = ctx.input(|i| i.events.clone());
        let modifiers = modifier_bits(ctx.input(|i| i.modifiers));
        let mut changed = false;
        // the modifier keys themselves are bound hold keys (mod2key)
        for (bit, key) in [(keys::SHIFT, keys::SHIFT_KEY), (keys::CONTROL, keys::CONTROL_KEY), (keys::ALT, keys::MENU)] {
            if (modifiers & bit != 0) != (self.last_modifiers & bit != 0) && hold.contains(&key) {
                if modifiers & bit != 0 {
                    self.held.insert(key);
                } else {
                    self.held.remove(&key);
                }
                changed = true;
            }
        }
        self.last_modifiers = modifiers;
        for e in events {
            let egui::Event::Key { key, pressed, repeat, modifiers: m, .. } = e else { continue };
            let Some(code) = win_key(key) else { continue };
            if pressed {
                if repeat {
                    continue;
                }
                if hold.contains(&code) {
                    self.held.insert(code);
                    changed = true;
                } else if press.contains(&(code | modifier_bits(m))) && code | modifier_bits(m) != keys::NONE {
                    self.key_press(code | modifier_bits(m), app);
                }
            } else if self.held.remove(&code) || hold.contains(&code) {
                changed = true;
            }
        }
        if changed {
            self.held_keys(modifiers, app);
        }
    }

    /// Renders the VideoBox in `rect` (PictureBoxSizeMode.Zoom) with its mouse handling and
    /// context menu.
    pub fn render_video_box(&mut self, ui: &mut egui::Ui, rect: Rect, app: &mut AppCtx) -> egui::Response {
        let resp = ui.allocate_rect(rect, Sense::click_and_drag());
        let tex = self.gst.as_mut().and_then(|v| v.texture(ui.ctx()));
        if let Some(e) = self.gst.as_ref().and_then(|v| v.error.lock().unwrap().take()) {
            log::error!("{e}");
            self.gst = None;
        }
        // the no_video picture is 640x480 like the designer's VideoBox
        let [iw, ih] = self.gst.as_ref().and_then(|g| g.size()).filter(|_| tex.is_some()).map_or([640.0, 480.0], |s| [s[0] as f32, s[1] as f32]);
        let scale = (rect.width() / iw).min(rect.height() / ih);
        let fit = Rect::from_center_size(rect.center(), Vec2::new(iw * scale, ih * scale));
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::BLACK);
        match tex {
            Some(t) => {
                painter.image(t, fit, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
            }
            None => {
                painter.rect_filled(fit, 0.0, Color32::from_rgb(20, 20, 20));
                painter.text(fit.center(), egui::Align2::CENTER_CENTER, crate::i18n::t("NO VIDEO"), crate::fonts::ui_id((fit.height() / 8.0).max(10.0)), Color32::from_gray(120));
            }
        }
        let to_image = |p: Pos2| gimbal::image_position((p.x - rect.left()) as f64, (p.y - rect.top()) as f64, rect.width() as f64, rect.height() as f64, iw as f64, ih as f64);
        let to_screen = |x: f32, y: f32| Pos2::new(fit.left() + x * fit.width(), fit.top() + y * fit.height());

        // RenderFrame: the drag rectangle, else the camera's own tracking target
        let camera = self.camera(app);
        if let (Some(s), Some(e)) = (self.drag_start, self.drag_end) {
            let a = to_screen(((s.0.min(e.0) + 1.0) / 2.0) as f32, ((s.1.min(e.1) + 1.0) / 2.0) as f32);
            let size = Vec2::new(((s.0 - e.0).abs() / 2.0) as f32 * fit.width(), ((s.1 - e.1).abs() / 2.0) as f32 * fit.height());
            painter.rect_stroke(Rect::from_min_size(a, size), 0.0, Stroke::new(1.0, Color32::RED), egui::StrokeKind::Middle);
        } else if let Some(t) = camera.as_ref().map(|c| c.tracking).filter(|t| t.status == 1 && t.target_data & 2 == 0 && t.target_data & 4 != 0) {
            if t.mode == 1 && !t.point.0.is_nan() && !t.point.1.is_nan() {
                let r = if t.radius.is_nan() { 10.0 } else { t.radius } / 2.0;
                painter.circle_stroke(to_screen(t.point.0, t.point.1), r * scale, Stroke::new(1.0, Color32::RED));
            } else if t.mode == 2 && ![t.rec_top.0, t.rec_top.1, t.rec_bottom.0, t.rec_bottom.1].iter().any(|v| v.is_nan()) {
                let r = Rect::from_two_pos(to_screen(t.rec_top.0, t.rec_top.1), to_screen(t.rec_bottom.0, t.rec_bottom.1));
                painter.rect_stroke(r, 0.0, Stroke::new(1.0, Color32::RED), egui::StrokeKind::Middle);
            }
        }

        let modifiers = modifier_bits(ui.input(|i| i.modifiers));
        if let Some(pos) = resp.hover_pos() {
            // VideoBox_MouseMove: the ground marker at most every 100 ms
            if self.last_mouse_move.is_none_or(|t| t.elapsed() > Duration::from_millis(100)) {
                self.last_mouse_move = Some(Instant::now());
                self.mouse_marker = to_image(pos).and_then(|(x, y)| camera.as_ref()?.image_point_location(x, y)).map(|p| (p.lat, p.lng));
            }
            let down = [egui::PointerButton::Primary, egui::PointerButton::Secondary, egui::PointerButton::Middle].into_iter().find(|b| ui.input(|i| i.pointer.button_down(*b)));
            if down.is_some_and(|b| Click::new(modifiers, mouse_bits(b)) == self.prefs.track_object_under_mouse) {
                if self.drag_start.is_none() {
                    self.drag_start = to_image(pos);
                }
                self.drag_end = to_image(pos);
            } else if down.is_none() || ui.input(|i| i.pointer.is_moving()) {
                // releasing keeps the rectangle for the click that follows
                if down.is_some() {
                    self.drag_start = None;
                    self.drag_end = None;
                }
            }
            // VideoBox_Click (on release over the box)
            for b in [egui::PointerButton::Primary, egui::PointerButton::Secondary, egui::PointerButton::Middle, egui::PointerButton::Extra1, egui::PointerButton::Extra2] {
                if ui.input(|i| i.pointer.button_released(b)) {
                    if let Some(point) = to_image(pos) {
                        self.click(Click::new(modifiers, mouse_bits(b)), point, app);
                    }
                    self.drag_start = None;
                    self.drag_end = None;
                }
            }
        } else if self.mouse_marker.is_some() || self.drag_start.is_some() {
            // VideoBox_MouseLeave
            self.mouse_marker = None;
            self.drag_start = None;
            self.drag_end = None;
        }

        self.context_menu(&resp, app, camera.as_ref());
        resp
    }

    fn click(&mut self, binding: Click, point: (f64, f64), app: &AppCtx) {
        let p = &self.prefs;
        let camera = self.camera(app);
        if binding == p.move_camera_to_mouse_location {
            let Some((g, vehicle_yaw)) = gimbal_manager(app) else { return };
            let (Some(att), Some(c)) = (g.attitude(0, vehicle_yaw), camera) else { return };
            let q = att.mul(&c.image_point_rotation(point.0, point.1));
            send(app, g.set_attitude(&q, self.yaw_lock, vehicle_yaw, 0));
        } else if binding == p.move_camera_poi_to_mouse_location {
            let Some(loc) = camera.and_then(|c| c.image_point_location(point.0, point.1)) else { return };
            if let (Some(l), Some((g, _))) = (app.link, gimbal_manager(app)) {
                if let Some(c) = g.roi_location(loc.lat, loc.lng, loc.alt) {
                    l.send(LinkCommand::CommandInt(c));
                }
            }
        } else if binding == p.track_object_under_mouse {
            let Some(c) = camera else { return };
            if let Some(l) = app.link {
                let (command, params) = Camera::tracking_interval(5);
                l.send(LinkCommand::CommandNoAck { command, params });
            }
            let (x, y) = (point.0 as f32, point.1 as f32);
            send(app, match self.drag_start {
                Some(s) => c.track_rectangle(s.0 as f32, s.1 as f32, x, y),
                None => c.track_point(x, y),
            });
        }
    }

    fn context_menu(&mut self, resp: &egui::Response, app: &mut AppCtx, camera: Option<&Camera>) {
        let placement = self.placement;
        let mut next = None;
        let mut action = None;
        let can_image = camera.is_some_and(|c| c.can_capture_image());
        let can_video = camera.is_some_and(|c| c.can_capture_video());
        resp.context_menu(|ui| {
            let items: [(&str, bool, &'static str); 4] = [("Video Stream", true, "stream"), ("Retract", true, "retract"), ("Neutral", true, "neutral"), ("Point Down", true, "down")];
            let item = |ui: &mut egui::Ui, text: &str, enabled: bool| {
                let clicked = ui.add_enabled(enabled, egui::Button::new(crate::i18n::t(text))).clicked();
                if clicked {
                    ui.close();
                }
                clicked
            };
            for (i, (text, enabled, a)) in items.into_iter().enumerate() {
                if item(ui, text, enabled) {
                    action = Some(a);
                }
                if i == 0 {
                    ui.separator();
                }
            }
            if item(ui, "Point Home", true) {
                action = Some("home");
            }
            let mut locked = self.yaw_lock;
            if ui.checkbox(&mut locked, crate::i18n::t("Yaw Lock")).clicked() {
                action = Some("yawlock");
                ui.close();
            }
            ui.separator();
            for (text, enabled, a) in [("Take Picture", can_image, "picture"), ("Start Recording", can_video, "rec_start"), ("Stop Recording", can_video, "rec_stop")] {
                if item(ui, text, enabled) {
                    action = Some(a);
                }
            }
            ui.separator();
            if item(ui, "Shortcuts", true) {
                action = Some("shortcuts");
            }
            // CameraCustomSettingsForm (camera definition settings) is not available yet
            item(ui, "Custom Settings", false);
            // FlightData's items: Mini map (Full Sized), Swap with map, Close
            if placement == GimbalPlacement::FullSized {
                let mut mini = self.show_mini_map;
                if ui.checkbox(&mut mini, crate::i18n::t("Mini map")).clicked() {
                    self.show_mini_map = mini;
                }
            }
            let swap_visible = placement == GimbalPlacement::Mini || (placement == GimbalPlacement::FullSized && self.show_mini_map);
            if swap_visible && ui.button(crate::i18n::t("Swap with map")).clicked() {
                next = Some(if placement == GimbalPlacement::Mini { GimbalPlacement::FullSized } else { GimbalPlacement::Mini });
                ui.close();
            }
            if matches!(placement, GimbalPlacement::FullSized | GimbalPlacement::Mini) && ui.button(crate::i18n::t("Close")).clicked() {
                next = Some(GimbalPlacement::Closed);
                ui.close();
            }
        });
        match action {
            Some("stream") => self.open_selector(app),
            Some("retract") => send(app, gimbal_manager(app).and_then(|(g, _)| g.retract())),
            Some("neutral") => send(app, gimbal_manager(app).and_then(|(g, _)| g.neutral())),
            Some("down") => self.point_down(app),
            Some("home") => self.point_home(app),
            Some("yawlock") => self.set_yaw_lock(!self.yaw_lock, app),
            Some("picture") => self.take_picture(app),
            Some("rec_start") => self.set_recording(true, app),
            Some("rec_stop") => self.set_recording(false, app),
            Some("shortcuts") => self.settings = Some(SettingsForm { prefs: self.prefs.clone(), listening: None, clash: None }),
            _ => {}
        }
        if let Some(p) = next {
            let ctx = resp.ctx.clone();
            self.set_placement(p, app, &ctx);
        }
    }

    /// videoStreamToolStripMenuItem_Click
    fn open_selector(&mut self, app: &mut AppCtx) {
        if crate::gstreamer::look_for_gstreamer(&crate::platform::data_dir()).is_none() {
            self.start_download(app);
            return;
        }
        let streams: Vec<gimbal::VideoStream> = app.link.map(|l| l.shared.read().video_streams.values().cloned().collect()).unwrap_or_default();
        let names = streams.iter().map(|s| s.name.clone()).collect();
        let raw = streams.first().map(gimbal::gstreamer_pipeline).unwrap_or_default();
        self.selector = Some(Selector { streams, names, sel: 0, raw });
    }

    fn start_download(&mut self, _app: &mut AppCtx) {
        let status = Arc::new(Mutex::new((0, "Downloading..".to_string())));
        let done = Arc::new(AtomicBool::new(false));
        let cancel = Arc::new(AtomicBool::new(false));
        let (s, d, k) = (status.clone(), done.clone(), cancel.clone());
        let data = crate::platform::data_dir();
        std::thread::spawn(move || {
            let report = |p: i32, t: &str| *s.lock().unwrap() = (p, t.to_string());
            let _ = crate::gstreamer::download_cancellable(&data, &report, &k);
            d.store(true, Ordering::Relaxed);
        });
        self.download = Some((status, done, cancel));
    }

    /// Dialogs, the pop-out window, keyboard handling and the auto-connect timer.
    pub fn extra_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if self.placement == GimbalPlacement::Closed {
            return;
        }
        self.keyboard(ctx, app);
        if self.gst_ready {
            self.auto_connect(app, ctx);
        }
        if self.gst.is_some() {
            ctx.request_repaint_after(Duration::from_millis(33));
        }
        self.gst_prompt_ui(ctx, app);
        self.download_ui(ctx, app);
        self.selector_ui(ctx, app);
        self.settings_ui(ctx, app);

        if self.placement == GimbalPlacement::PopOut {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Gimbal Control"))
                .id(egui::Id::new("gimbal_popout_window"))
                .collapsible(false)
                .open(&mut open)
                .default_size(Vec2::new(600.0, 400.0))
                .pivot(egui::Align2::CENTER_CENTER)
                .frame(egui::Frame::window(&ctx.global_style()).inner_margin(0.0))
                .default_pos(ctx.content_rect().center())
                .show(ctx, |ui| {
                    let r = ui.available_rect_before_wrap();
                    self.render_video_box(ui, r, app);
                });
            if !open {
                // closing the form disposes the control
                self.set_placement(GimbalPlacement::Closed, app, ctx);
            }
        }
    }

    fn gst_prompt_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if !self.gst_prompt {
            return;
        }
        let mut answer = None;
        egui::Modal::new(egui::Id::new("gimbal_gst_prompt")).show(ctx, |ui| {
            ui.set_width(340.0);
            ui.label(egui::RichText::new(crate::i18n::t("GStreamer not found")).strong());
            theme::label(ui, "This feature requires GStreamer. Would you like to download and install it now?");
            ui.horizontal(|ui| {
                if theme::button(ui, "Yes").clicked() {
                    answer = Some(true);
                }
                if theme::button(ui, "No").clicked() {
                    answer = Some(false);
                }
            });
        });
        if let Some(yes) = answer {
            self.gst_prompt = false;
            if yes {
                self.start_download(app);
            }
        }
    }

    fn download_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some((status, done, cancel)) = self.download.clone() else { return };
        let (pct, text) = status.lock().unwrap().clone();
        egui::Window::new(crate::i18n::t("Progress")).collapsible(false).resizable(false).fixed_size(Vec2::new(370.0, 110.0)).show(ctx, |ui| {
            theme::label(ui, &text);
            theme::progress(ui, pct.max(0) as f32 / 100.0, Vec2::new(358.0, 23.0), "");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                if theme::button(ui, "Cancel").clicked() {
                    cancel.store(true, Ordering::Relaxed);
                }
            });
        });
        ctx.request_repaint_after(Duration::from_millis(100));
        if !done.load(Ordering::Relaxed) {
            return;
        }
        self.download = None;
        if crate::gstreamer::look_for_gstreamer(&crate::platform::data_dir()).is_some() {
            self.gst_ready = true;
            self.auto_connect_at = Some(Instant::now() + Duration::from_secs(1));
        } else if !cancel.load(Ordering::Relaxed) {
            let mut message = "GStreamer was not found after installation. Please install it manually.".to_string();
            if cfg!(windows) {
                message += "\n\nFor Windows, install the MinGW version from https://gstreamer.freedesktop.org/download/#windows";
            }
            app.toasts.error(message);
        }
    }

    fn selector_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some(sel) = self.selector.as_mut() else { return };
        let mut open = true;
        let mut launch = false;
        egui::Window::new("VideoStreamSelector")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .fixed_size(Vec2::new(430.0, 93.0))
            .show(ctx, |ui| {
                let r = ui.max_rect();
                let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(r.min + Vec2::new(x - 4.0, y - 4.0), Vec2::new(w, h));
                ui.put(at(12.0, 9.0, 92.0, 13.0), egui::Label::new(crate::i18n::t("Detected Streams")));
                ui.put(at(12.0, 36.0, 97.0, 13.0), egui::Label::new(crate::i18n::t("GStreamer Pipeline")));
                let selected = sel.names.get(sel.sel).cloned().unwrap_or_default();
                let mut pick = None;
                ui.put(at(115.0, 6.0, 303.0, 21.0), |ui: &mut egui::Ui| {
                    egui::ComboBox::from_id_salt("cmb_detectedstreams")
                        .width(303.0)
                        .selected_text(selected)
                        .show_ui(ui, |ui| {
                            for (i, n) in sel.names.iter().enumerate() {
                                if ui.selectable_label(i == sel.sel, n).clicked() {
                                    pick = Some(i);
                                }
                            }
                        })
                        .response
                });
                if let Some(i) = pick {
                    // cmb_detectedstreams_SelectedIndexChanged
                    sel.sel = i;
                    sel.raw = gimbal::gstreamer_pipeline(&sel.streams[i]);
                }
                ui.put(at(115.0, 33.0, 303.0, 20.0), egui::TextEdit::singleline(&mut sel.raw));
                if ui.put(at(343.0, 59.0, 75.0, 21.0), |ui: &mut egui::Ui| theme::button_sized(ui, "Connect", Vec2::new(75.0, 21.0))).clicked() {
                    launch = true;
                }
                ui.allocate_space(Vec2::new(422.0, 80.0));
            });
        if launch {
            // but_launch_Click: an empty pipeline just closes
            let sel = self.selector.take().unwrap();
            if !sel.raw.is_empty() {
                self.settings_camera_address = sel.streams.get(sel.sel).filter(|s| gimbal::gstreamer_pipeline(s) == sel.raw).map(|s| (s.sysid, s.compid));
                self.auto_connect_at = None;
                self.start(&sel.raw, ctx);
            }
            let _ = app;
        } else if !open {
            self.selector = None;
        }
    }

    /// GimbalControlSettingsForm (533x504): label | binding button | ❌ per preference row.
    fn settings_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some(form) = self.settings.as_mut() else { return };
        // a listening button takes the next key (KeyBindingButton) or click (ClickBindingButton)
        if let Some(row) = form.listening {
            let (events, modifiers) = ctx.input(|i| (i.events.clone(), modifier_bits(i.modifiers)));
            for e in events {
                match (&ROWS[row], e) {
                    (_, egui::Event::Key { key: egui::Key::Escape, pressed: true, .. }) => form.listening = None,
                    (Row::Key(..), egui::Event::Key { key, pressed: true, modifiers: m, .. }) => {
                        if let Some(k) = win_key(key) {
                            form.listening = None;
                            bind(form, row, Binding::Key(k | modifier_bits(m)));
                        }
                    }
                    (Row::Modifier(..), egui::Event::Key { key: egui::Key::Enter, pressed: true, .. }) => {
                        form.listening = None;
                        bind(form, row, Binding::Key(modifiers));
                    }
                    _ => {}
                }
            }
        }
        let mut result = None;
        egui::Window::new("GimbalVideoControlSettings")
            .collapsible(false)
            .resizable(false)
            .fixed_size(Vec2::new(533.0, 504.0))
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().max_height(466.0).show(ui, |ui| {
                    egui::Grid::new("gimbal_prefs").num_columns(3).spacing([6.0, 3.0]).show(ui, |ui| {
                        for (i, row) in ROWS.iter().enumerate() {
                            let listening = form.listening == Some(i);
                            match row {
                                Row::Key(l, f) | Row::Modifier(l, f) => {
                                    let modifiers_only = matches!(row, Row::Modifier(..));
                                    ui.add_sized([200.0, 20.0], egui::Label::new(crate::i18n::t(l)));
                                    let text = if listening {
                                        if modifiers_only { "Press modifier keys + Enter...".to_string() } else { "Press a key... (Esc to cancel)".to_string() }
                                    } else {
                                        key_string(*f(&mut form.prefs), modifiers_only)
                                    };
                                    if binding_button(ui, &text, listening).clicked() {
                                        form.listening = Some(i);
                                    }
                                    if theme::button_sized(ui, "❌", Vec2::new(23.0, 23.0)).clicked() {
                                        *f(&mut form.prefs) = keys::NONE;
                                    }
                                }
                                Row::Click(l, f) => {
                                    ui.add_sized([200.0, 20.0], egui::Label::new(crate::i18n::t(l)));
                                    let text = if listening { "Click a mouse button... (Esc to cancel)".to_string() } else { click_string((f(&mut form.prefs).modifiers, f(&mut form.prefs).button)) };
                                    let r = binding_button(ui, &text, listening);
                                    if listening {
                                        let m = modifier_bits(ui.input(|i| i.modifiers));
                                        let b = [egui::PointerButton::Primary, egui::PointerButton::Secondary, egui::PointerButton::Middle, egui::PointerButton::Extra1, egui::PointerButton::Extra2]
                                            .into_iter()
                                            .find(|b| r.hovered() && ui.input(|inp| inp.pointer.button_clicked(*b)));
                                        if let Some(b) = b {
                                            form.listening = None;
                                            bind(form, i, Binding::Click(Click::new(m, mouse_bits(b))));
                                        }
                                    } else if r.clicked() {
                                        form.listening = Some(i);
                                    }
                                    if theme::button_sized(ui, "❌", Vec2::new(23.0, 23.0)).clicked() {
                                        *f(&mut form.prefs) = Click::new(keys::NONE, 0);
                                    }
                                }
                                Row::Decimal(l, f, min, max, inc, dp) => {
                                    ui.add_sized([200.0, 20.0], egui::Label::new(crate::i18n::t(l)));
                                    let v = f(&mut form.prefs);
                                    // Minimum/Maximum widen to hold the stored value
                                    let (lo, hi) = (min.min(*v), max.max(*v));
                                    ui.add(egui::DragValue::new(v).range(lo..=hi).speed(*inc).fixed_decimals(*dp));
                                    ui.label("");
                                }
                                Row::Check(l, f) => {
                                    ui.add_sized([200.0, 20.0], egui::Label::new(crate::i18n::t(l)));
                                    ui.checkbox(f(&mut form.prefs), "");
                                    ui.label("");
                                }
                            }
                            ui.end_row();
                        }
                    });
                });
                ui.separator();
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if theme::button_sized(ui, "Cancel", Vec2::new(75.0, 23.0)).clicked() {
                        result = Some(false);
                    }
                    if theme::button_sized(ui, "Save", Vec2::new(75.0, 23.0)).clicked() {
                        result = Some(true);
                    }
                });
            });
        // "Key Binding Clash"
        if let Some((row, binding, msg)) = form.clash.clone() {
            let mut answer = None;
            egui::Modal::new(egui::Id::new("gimbal_binding_clash")).show(ctx, |ui| {
                ui.set_width(320.0);
                ui.label(egui::RichText::new(crate::i18n::t("Key Binding Clash")).strong());
                theme::label(ui, &msg);
                ui.horizontal(|ui| {
                    if theme::button(ui, "Yes").clicked() {
                        answer = Some(true);
                    }
                    if theme::button(ui, "No").clicked() {
                        answer = Some(false);
                    }
                });
            });
            if let Some(yes) = answer {
                form.clash = None;
                if yes {
                    form.prefs.set(row, binding);
                }
            }
        }
        match result {
            Some(true) => {
                let prefs = self.settings.take().unwrap().prefs;
                if let Ok(j) = serde_json::to_string(&prefs) {
                    app.settings.planner.insert("GimbalControlPreferences".into(), j);
                }
                self.prefs = Prefs::load(&app.settings.planner);
            }
            Some(false) => self.settings = None,
            None => {}
        }
    }
}

/// HandleKeyBindingChanged / HandleClickBindingChanged: None always applies; a clash asks first.
fn bind(form: &mut SettingsForm, row: usize, binding: Binding) {
    let none = matches!(binding, Binding::Key(keys::NONE)) || matches!(binding, Binding::Click(c) if c.button == 0);
    let clashes = if none { Vec::new() } else { form.prefs.clashes(row, binding) };
    if clashes.is_empty() {
        form.prefs.set(row, binding);
    } else {
        let mut msg = "This binding is already used by the following:\n\n".to_string();
        for c in clashes {
            msg += &format!("- {c}\n");
        }
        msg += "\nAre you sure you want to use this binding?";
        form.clash = Some((row, binding, msg));
    }
}

/// KeyBindingButton: the green MyButton, outlined red while listening.
fn binding_button(ui: &mut egui::Ui, text: &str, listening: bool) -> egui::Response {
    let r = theme::button_sized(ui, text, Vec2::new(250.0, 23.0));
    if listening {
        ui.painter().rect_stroke(r.rect, 2.0, Stroke::new(1.0, Color32::RED), egui::StrokeKind::Inside);
    }
    r
}

/// splitContainer1_Panel2_Resize: the mini video keeps the picture's aspect, 30% of the panel,
/// left of TRK_zoom (45 px) at the bottom.
pub fn mini_video_rect(panel: Rect, aspect: f32) -> Rect {
    let w = (panel.width() * 0.3).floor();
    let h = (panel.height() * 0.3).floor();
    let (w, h) = (w.min((h * aspect).floor()), h.min((w / aspect).floor()));
    Rect::from_min_size(Pos2::new(panel.right() - w - 45.0, panel.bottom() - h), Vec2::new(w, h))
}

/// The mini map: 30% of the panel in the bottom-right corner.
pub fn mini_map_rect(panel: Rect) -> Rect {
    let (w, h) = ((panel.width() * 0.3).floor(), (panel.height() * 0.3).floor());
    Rect::from_min_size(Pos2::new(panel.right() - w, panel.bottom() - h), Vec2::new(w, h))
}

/// MP_GIMBAL=full|mini|popout (screen captures): the placement to open once at start-up.
pub fn env_placement() -> Option<GimbalPlacement> {
    static TAKEN: AtomicBool = AtomicBool::new(false);
    if TAKEN.swap(true, Ordering::Relaxed) {
        return None;
    }
    match std::env::var("MP_GIMBAL").ok()?.as_str() {
        "full" => Some(GimbalPlacement::FullSized),
        "mini" => Some(GimbalPlacement::Mini),
        "popout" => Some(GimbalPlacement::PopOut),
        _ => None,
    }
}

impl GimbalVideo {
    /// The picture's aspect ratio (no_video is 4:3).
    pub fn aspect(&self) -> f32 {
        self.gst.as_ref().and_then(|g| g.size()).filter(|s| s[1] > 0).map_or(640.0 / 480.0, |s| s[0] as f32 / s[1] as f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preferences_round_trip_as_json_net_writes_them() {
        let p = Prefs::default();
        let j = serde_json::to_string(&p).unwrap();
        assert!(j.contains("\"SlewLeft\":65"));
        assert!(j.contains("\"TakePicture\":262214"), "Alt | F");
        assert!(j.contains("\"MoveCameraPOIToMouseLocation\":{\"Item1\":131072,\"Item2\":1048576}"));
        assert!(j.contains("\"CameraHFOV\":40.0"));
        let mut planner = std::collections::BTreeMap::new();
        planner.insert("GimbalControlPreferences".to_string(), j);
        assert_eq!(Prefs::load(&planner), p);
        planner.insert("GimbalControlPreferences".to_string(), "{not json".into());
        assert_eq!(Prefs::load(&planner), Prefs::default());
        // a partial object keeps the other defaults
        planner.insert("GimbalControlPreferences".to_string(), "{\"SlewLeft\":37}".into());
        assert_eq!(Prefs::load(&planner).slew_left, 37);
        assert_eq!(Prefs::load(&planner).home, 'H' as u32);
    }

    #[test]
    fn binding_texts_match_the_buttons() {
        assert_eq!(key_string('A' as u32, false), "A");
        assert_eq!(key_string(keys::ALT | 'F' as u32, false), "Alt + F");
        assert_eq!(key_string(keys::NONE, false), "None");
        assert_eq!(key_string(keys::SHIFT, true), "Shift + ");
        assert_eq!(key_string(keys::CONTROL | keys::SHIFT | 49, false), "Shift + Ctrl + D1");
        assert_eq!(click_string((keys::CONTROL, keys::MOUSE_LEFT)), "Ctrl + Left Click");
        assert_eq!(click_string((keys::ALT, 0)), "None");
    }

    #[test]
    fn held_keys_become_rates() {
        let p = Prefs::default();
        let held: BTreeSet<u32> = ['W' as u32, 'D' as u32].into_iter().collect();
        assert_eq!(p.rates(&held, 0), (5.0, 5.0, 0.0));
        assert_eq!(p.rates(&held, keys::SHIFT), (25.0, 25.0, 0.0));
        assert_eq!(p.rates(&held, keys::CONTROL), (1.0, 1.0, 0.0));
        assert_eq!(p.rates(&held, keys::CONTROL | keys::SHIFT), (5.0, 5.0, 0.0), "only an exact modifier match changes speed");
        let held: BTreeSet<u32> = ['A' as u32, 'S' as u32, 'D' as u32, 'Q' as u32].into_iter().collect();
        assert_eq!(p.rates(&held, 0), (-5.0, 0.0, -1.0));
        assert!(p.hold_keys().contains(&keys::SHIFT_KEY) && p.hold_keys().contains(&keys::CONTROL_KEY));
    }

    #[test]
    fn clashing_bindings_ask_before_applying() {
        let mut form = SettingsForm { prefs: Prefs::default(), listening: None, clash: None };
        // Slew Left (row 0) to N, which Neutral uses
        bind(&mut form, 0, Binding::Key('N' as u32));
        let (_, _, msg) = form.clash.clone().unwrap();
        assert!(msg.contains("- Neutral"));
        assert_eq!(form.prefs.slew_left, 'A' as u32, "unchanged until Yes");
        // a free key applies at once; None never clashes
        form.clash = None;
        bind(&mut form, 0, Binding::Key('Z' as u32));
        assert_eq!(form.prefs.slew_left, 'Z' as u32);
        bind(&mut form, 10, Binding::Key(keys::NONE));
        assert!(form.clash.is_none());
        // click bindings compare against the other click rows only
        bind(&mut form, 19, Binding::Click(Click::new(keys::ALT, keys::MOUSE_LEFT)));
        assert!(form.clash.as_ref().unwrap().2.contains("- Click Track Object"));
    }

    #[test]
    fn mini_layouts_follow_panel2_resize() {
        let panel = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 600.0));
        assert_eq!(mini_map_rect(panel), Rect::from_min_size(Pos2::new(700.0, 420.0), Vec2::new(300.0, 180.0)));
        // 16:9 video: height limits, width follows the aspect
        let r = mini_video_rect(panel, 16.0 / 9.0);
        assert_eq!(r.size(), Vec2::new(300.0, 168.0));
        assert_eq!(r.right(), 955.0);
        assert_eq!(r.bottom(), 600.0);
    }

    #[test]
    fn egui_keys_map_to_winforms_codes() {
        assert_eq!(win_key(egui::Key::A), Some(65));
        assert_eq!(win_key(egui::Key::Z), Some(90));
        assert_eq!(win_key(egui::Key::Num1), Some(49));
        assert_eq!(win_key(egui::Key::F5), Some(116));
        assert_eq!(win_key(egui::Key::ArrowUp), Some(38));
    }
}
