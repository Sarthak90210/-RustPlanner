//! Port of `GCSViews/FlightData` laid out from FlightData.Designer.cs / FlightData.resx:
//! MainH (left 300px: HUD over tabControlactions) | tableMap (map + 25px bottom panel).

use std::time::Instant;

use egui::{Color32, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Vec2};
use mp_core::firmware::Firmware;
use mp_core::link::{LinkCommand, LinkEvent, LinkStatus};
use mp_core::mavcmd;
use mp_core::mission::Locationwp;
use mp_core::state::CurrentState;

use crate::app::AppCtx;
use crate::{fonts, hud, map, theme};
use super::gimbal_video::{GimbalPlacement, GimbalVideo};

const TABS: [&str; 13] = [
    "Quick",
    "Actions",
    "Messages",
    "PreFlight",
    "Gauges",
    "Transponder",
    "Status",
    "Servo/Relay",
    "Aux Function",
    "Scripts",
    "Payload Control",
    "Telemetry Logs",
    "DataFlash Logs",
];

/// Index of tabActionsSimple (not in TABS: it shares the "Actions" text).
const SIMPLE_ACTIONS: usize = 13;

/// quickView1..6 numberColor from FlightData.Designer.cs
const QUICK_COLORS: [Color32; 6] = [
    Color32::from_rgb(209, 151, 248),
    Color32::from_rgb(254, 132, 46),
    Color32::from_rgb(255, 96, 91),
    Color32::from_rgb(0, 255, 83),
    Color32::from_rgb(254, 254, 86),
    Color32::from_rgb(0, 255, 252),
];

/// CMB_action items (MAV_CMD names offered by the Actions tab).
/// FlightData.actions enum (CMB_action items) and the command each one sends.
/// FlightData.actions, in enum order (CMB_action)
const ACTIONS: &[(&str,)] = &[
    ("Loiter_Unlim",),
    ("Return_To_Launch",),
    ("Preflight_Calibration",),
    ("Mission_Start",),
    ("Preflight_Reboot_Shutdown",),
    ("Trigger_Camera",),
    ("System_Time",),
    ("Battery_Reset",),
    ("ADSB_Out_Ident",),
    ("Scripting_cmd_stop_and_restart",),
    ("Scripting_cmd_stop",),
    ("HighLatency_Enable",),
    ("HighLatency_Disable",),
    ("Toggle_Safety_Switch",),
    ("Parachute_Release",),
    ("Engine_Start",),
    ("Engine_Stop",),
    ("Terminate_Flight",),
    ("Format_SD_Card",),
    ("Reboot_Mass_Storage",),
];

enum Prompt {
    /// CustomMessageBox YesNo
    Confirm { title: String, text: String, then: Confirmed },
    /// InputBox with one or more fields
    Input { title: String, fields: Vec<(String, String)>, then: Inputted },
    /// AltInputBox: altitude string + frame selection dropdown
    AltInput { title: String, prompt_text: String, alt: String, frame: u8, then: AltInputted },
}

#[derive(Clone, Copy)]
enum AltInputted {
    FlyToHere(f64, f64),
    FlyToHereAlt,
}

#[derive(Clone, Copy)]
enum Confirmed {
    DoAction(usize),
    Arm(bool),
    /// "Force Arm" / "Force Disarm" after the vehicle refused
    ForceArm(bool),
    /// Common.MessageShowAgain("Resume Mission", ...)
    ResumeWarn,
    /// "You are in failsafe, are you sure?"
    SetMode(u32),
    /// setHomeHereToolStripMenuItem_Click (OK / Cancel): lat, lng, terrain alt
    SetHome(f64, f64, f64),
}

/// BUT_resumemis_Click as a state machine driven by frames and link events.
#[derive(Clone)]
enum Resume {
    Download { wp: u16 },
    Upload { wp: u16, target_alt: f32, items: Vec<mp_core::mission::Locationwp> },
    Uploading { target_alt: f32 },
    Guided { target_alt: f32, next: Instant, tries: u32 },
    Arm { target_alt: f32, next: Instant, tries: u32 },
    Takeoff { target_alt: f32, next: Instant, tries: u32 },
    Auto { next: Instant, tries: u32 },
}

#[derive(Clone, Copy)]
enum Inputted {
    FlyToCoords,
    PointCameraCoords,
    /// pointCameraHereToolStripMenuItem_Click's target altitude
    PointCameraHere(f64, f64),
    /// takeOffToolStripMenuItem_Click's altitude
    TakeOff,
    AddPoi(f64, f64),
    SendMessage,
    JumpTag,
    ResumeAt,
}

/// CMB_mountmode.DataSource: the options of the first of these parameters with any.
fn mount_modes(fw: mp_core::firmware::Firmware) -> Vec<(i64, String)> {
    ["MNT1_DEFLT_MODE", "MNT_DEFLT_MODE", "MNT_MODE"]
        .iter()
        .filter_map(|n| mp_core::param_meta::get(fw, n))
        .map(|m| m.values.clone())
        .find(|v| !v.is_empty())
        .unwrap_or_default()
}

/// Strings.CommandFailed (the resource ends with a line break)
const COMMAND_FAILED: &str = "The Command failed to execute\n";

/// BUTactiondo_Click's command for a built-in action after its confirmation, or None where
/// Enum.Parse finds no MAV_CMD (the action then fails with "Command Failed").
fn action_command(name: &str, fw: mp_core::firmware::Firmware) -> Option<(u16, [f32; 7])> {
    let p = |a: f32, b: f32, c: f32| [a, b, c, 0.0, 0.0, 0.0, 0.0];
    Some(match name {
        // request gyro on Copter; baro / airspeed always
        "Preflight_Calibration" => (241, p(if fw == mp_core::firmware::Firmware::ArduCopter2 { 1.0 } else { 0.0 }, 0.0, 1.0)),
        "Terminate_Flight" => (185, p(1.0, 0.0, 0.0)),
        "Parachute_Release" => (208, p(2.0, 0.0, 0.0)),
        // doReboot()
        "Preflight_Reboot_Shutdown" => (246, p(1.0, 0.0, 0.0)),
        // ArduPilot REBOOT_TO_MASS_STORAGE action
        "Reboot_Mass_Storage" => (246, p(5.0, 0.0, 0.0)),
        "HighLatency_Enable" => (2600, p(1.0, 0.0, 0.0)),
        "HighLatency_Disable" => (2600, p(0.0, 0.0, 0.0)),
        "Engine_Start" => (223, p(1.0, 0.0, 0.0)),
        "Engine_Stop" => (223, p(0.0, 0.0, 0.0)),
        // batt 1, 100%
        "Battery_Reset" => (42651, p(255.0, 100.0, 0.0)),
        "Loiter_Unlim" => (17, p(0.0, 0.0, 0.0)),
        "Return_To_Launch" => (20, p(0.0, 0.0, 0.0)),
        "Mission_Start" => (300, p(0.0, 0.0, 0.0)),
        _ => return None,
    })
}

pub struct FlightDataView {
    tab: usize,
    auto_pan: bool,
    tuning: bool,
    mission: Vec<Locationwp>,
    mode_sel: Option<u32>,
    wp_sel: u16,
    action_sel: usize,
    mount_sel: usize,
    /// a Do Action command waiting for its ACK: (command, "Command Failed" suffix or "" when
    /// the original ignores the result; 203 retries as DIGICAM_CONTROL)
    pub(crate) action_ack: Option<(u16, String)>,
    change_speed: f32,
    change_alt: f32,
    loiter_rad: f32,
    quick_edit: Option<usize>,
    /// the quick tiles' QuickView controls (their fitted font size persists between paints)
    quick_views: Vec<crate::extlibs::controls::QuickView>,
    last_mode: String,
    mode_changed: Instant,
    last_armed: bool,
    /// doARM in flight: (arm, message count when sent, time sent)
    arm_pending: Option<(bool, usize, Instant)>,
    /// a Force Arm / Force Disarm waiting for its ACK
    force_pending: bool,
    /// Strings.ErrorRejectedByMAV after a refused forced arm
    arm_error: Option<String>,
    /// the vehicle refused: (arm, collected STATUSTEXT)
    arm_failed: Option<(bool, String)>,
    /// CurrentState.lastautowp: the last waypoint reached in AUTO
    last_auto_wp: i32,
    resume: Option<Resume>,
    armed_at: Instant,
    left_w: f32,
    prompt: Option<Prompt>,
    camera_overlap: bool,
    graph: crate::zedgraph::TuningGraph,
    tlog_name: String,
    hud_window: Option<hud::HudClick>,
    quick_count: Option<(String, String)>,
    tabs_customize: bool,
    pub(crate) video: Option<crate::mjpeg::Mjpeg>,
    /// FlightData.hudGStreamer: its own source, stopped by GStreamer Stop and a cancelled prompt
    gst: Option<crate::mjpeg::Mjpeg>,
    hud_prompt: Option<HudPrompt>,
    /// GStreamerUI.DownloadGStreamer in progress: (percent, status), finished, the pipeline to start
    gst_download: Option<(std::sync::Arc<std::sync::Mutex<(i32, String)>>, std::sync::Arc<std::sync::atomic::AtomicBool>, String, std::sync::Arc<std::sync::atomic::AtomicBool>)>,
    log_browse: crate::log_browse::LogBrowse,
    log_file_request: Option<(String, crate::platform::FilePicker)>,
    analysis: Option<(String, Vec<(String, String, String)>)>,
    raw_sensor: bool,
    extra: super::fd_tabs::ExtraTabs,
    checklist: super::checklist::CheckList,
    mavlog: super::mavlog::MavlinkLog,
    pub(crate) georef: crate::georef_ui::GeorefWindow,
    pub(crate) gimbal: GimbalVideo,
    logdl_open: bool,
    logdl_busy: bool,
    logdl_queue: Vec<(u16, u32)>,
    logdl_selected: std::collections::BTreeSet<u16>,
    logdl_output: Vec<String>,
}

impl Default for FlightDataView {
    fn default() -> Self {
        Self {
            tab: std::env::var("MP_FDTAB").ok().and_then(|v| v.parse().ok()).unwrap_or(0),
            auto_pan: true,
            tuning: false,
            mission: Vec::new(),
            mode_sel: None,
            wp_sel: 0,
            action_sel: 0,
            mount_sel: 0,
            action_ack: None,
            change_speed: 0.0,
            change_alt: 100.0,
            loiter_rad: 0.0,
            quick_edit: None,
            quick_views: Vec::new(),
            last_mode: String::new(),
            mode_changed: Instant::now(),
            last_armed: false,
            arm_pending: None,
            force_pending: false,
            arm_error: None,
            arm_failed: None,
            last_auto_wp: -1,
            resume: None,
            armed_at: Instant::now(),
            left_w: 300.0,
            prompt: None,
            camera_overlap: false,
            graph: Default::default(),
            tlog_name: String::new(),
            hud_window: None,
            quick_count: None,
            tabs_customize: false,
            video: None,
            gst: None,
            hud_prompt: None,
            gst_download: None,
            log_browse: Default::default(),
            log_file_request: None,
            analysis: None,
            raw_sensor: false,
            extra: Default::default(),
            checklist: Default::default(),
            mavlog: Default::default(),
            georef: Default::default(),
            gimbal: Default::default(),
            logdl_open: false,
            logdl_busy: false,
            logdl_queue: Vec::new(),
            logdl_selected: Default::default(),
            logdl_output: Vec::new(),
        }
    }
}

impl FlightDataView {
    /// temp form "logindex": open the log browser on a picked .bin.
    pub fn open_log_browse(&mut self) {
        self.log_browse.request_load();
    }

    /// The ACK of a Do Action command: Trigger_Camera falls back to DIGICAM_CONTROL, others
    /// report "Command Failed <CMD>".
    pub fn on_action_ack(&mut self, e: &LinkEvent, link: Option<&mp_core::link::Link>, toasts: &mut crate::app::Toasts) -> bool {
        let LinkEvent::CommandAck { command, ok, .. } = e else { return false };
        if *command == 400 && (self.arm_pending.is_some() || self.force_pending) {
            return true;
        }
        let Some((cmd, name)) = self.action_ack.clone().filter(|(c, _)| c == command) else { return false };
        self.action_ack = None;
        if !*ok {
            if cmd == 203 {
                if let Some(l) = link {
                    use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, DIGICAM_CONTROL_DATA};
                    let (sys, comp) = {
                        let s = l.shared.read();
                        (s.cs.sysid, s.cs.compid)
                    };
                    l.send(LinkCommand::Send(Box::new(MavMessage::DIGICAM_CONTROL(DIGICAM_CONTROL_DATA { target_system: sys, target_component: comp, shot: 1, ..Default::default() }))));
                }
            } else if !name.is_empty() {
                toasts.message(format!("{COMMAND_FAILED} {name}"), "Error");
            }
        }
        true
    }

    pub fn on_event(&mut self, e: &LinkEvent) {
        // the download threw ("Timeout getting waypoints"): the resume stops there
        if let LinkEvent::MissionDownloadFailed { mission_type: 0 } = e {
            if matches!(self.resume, Some(Resume::Download { .. })) {
                self.resume = None;
            }
        }
        if let LinkEvent::MissionDownloaded { mission_type: 0, items } = e {
            self.mission = items.clone();
            if let Some(Resume::Download { wp }) = self.resume.clone() {
                // keep home, takeoff and DO commands before the resume point; drop skipped NAV items
                let target_alt = items.get(wp as usize).map(|w| w.alt).unwrap_or(0.0);
                let cmds: Vec<_> = items
                    .iter()
                    .enumerate()
                    .filter(|(a, w)| !(*a < wp as usize && *a != 0 && ((w.id != 22 && w.id < 95) || w.id > 240)))
                    .map(|(_, w)| *w)
                    .collect();
                self.resume = Some(Resume::Upload { wp, target_alt, items: cmds });
            }
        }
        if let LinkEvent::MissionUploaded { mission_type: 0, result } = e {
            if let Some(Resume::Uploading { target_alt }) = self.resume.clone() {
                self.resume = match result {
                    Ok(()) => Some(Resume::Guided { target_alt, next: Instant::now(), tries: 0 }),
                    Err(_) => None,
                };
            }
        }
        if let LinkEvent::CommandAck { command: 400, ok, .. } = e {
            if std::mem::take(&mut self.force_pending) && !ok {
                self.arm_error = Some("Error: command rejected by MAV".into());
            }
            if let Some((arm, _, _)) = self.arm_pending {
                if !ok {
                    self.arm_failed = Some((arm, String::new()));
                } else {
                    self.arm_pending = None;
                }
            }
        }
    }

    /// The arm result and the Resume Mission steps, once per frame.
    fn actions_tick(&mut self, app: &mut AppCtx, cs: &CurrentState) {
        if cs.mode.eq_ignore_ascii_case("auto") && cs.wpno > 0 {
            self.last_auto_wp = cs.wpno as i32;
        }
        if let Some(e) = self.arm_error.take() {
            app.toasts.message(e, "Error");
        }
        // 8-second arm command timeout fallback: if vehicle does not ACK, prompt Force Arm
        if let Some((arm, _, sent_at)) = self.arm_pending {
            if sent_at.elapsed() >= std::time::Duration::from_secs(8) && self.arm_failed.is_none() {
                self.arm_failed = Some((arm, String::new()));
            }
        }
        if let (Some((arm, _, _)), Some((arm2, reasons))) = (self.arm_pending, self.arm_failed.as_mut()) {
            if arm == *arm2 && reasons.is_empty() {
                let start = self.arm_pending.map(|p| p.1).unwrap_or(0);
                // the STATUSTEXTs heard while arming, each with AppendLine's line break
                let msgs: String = app.link.map(|l| l.shared.read().messages.iter().skip(start).map(|m| format!("{}\n", m.text)).collect()).unwrap_or_default();
                *reasons = " ".into();
                let action = if arm { "Arm" } else { "Disarm" };
                self.prompt = Some(Prompt::Confirm {
                    title: "Error".into(),
                    text: format!("{action} failed.\n{msgs}\nForce {action} can bypass safety checks,\nwhich can lead to the vehicle crashing\nand causing serious injuries.\n\nDo you wish to Force {action}?"),
                    then: Confirmed::ForceArm(arm),
                });
                self.arm_pending = None;
                self.arm_failed = None;
            }
        }
        let Some(l) = app.link else {
            self.resume = None;
            return;
        };
        let find = |name: &str| cs.firmware.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case(name)).map(|(k, _)| k);
        let copter = cs.firmware == mp_core::firmware::Firmware::ArduCopter2;
        let fail = |toasts: &mut crate::app::Toasts| toasts.error("No response from the vehicle");
        self.resume = match self.resume.take() {
            Some(Resume::Upload { wp, target_alt, items }) => {
                l.send(LinkCommand::WriteMission { mission_type: 0, items });
                let _ = wp;
                Some(Resume::Uploading { target_alt })
            }
            Some(Resume::Guided { target_alt, next, tries }) if Instant::now() >= next => {
                if tries == 0 {
                    // set index back to 1
                    l.send(LinkCommand::SetCurrentWp(1));
                }
                if !copter {
                    Some(Resume::Auto { next: Instant::now(), tries: 0 })
                } else if cs.mode.eq_ignore_ascii_case("guided") {
                    Some(Resume::Arm { target_alt, next: Instant::now(), tries: 0 })
                } else if tries > 30 {
                    fail(app.toasts);
                    None
                } else {
                    if let Some(k) = find("Guided") {
                        l.send(LinkCommand::SetMode(k));
                    }
                    Some(Resume::Guided { target_alt, next: Instant::now() + std::time::Duration::from_secs(1), tries: tries + 1 })
                }
            }
            Some(Resume::Arm { target_alt, next, tries }) if Instant::now() >= next => {
                if cs.armed {
                    Some(Resume::Takeoff { target_alt, next: Instant::now(), tries: 0 })
                } else if tries > 30 {
                    fail(app.toasts);
                    None
                } else {
                    l.send(LinkCommand::Arm { arm: true, force: false });
                    Some(Resume::Arm { target_alt, next: Instant::now() + std::time::Duration::from_secs(1), tries: tries + 1 })
                }
            }
            Some(Resume::Takeoff { target_alt, next, tries }) if Instant::now() >= next => {
                if cs.alt >= target_alt - 2.0 {
                    Some(Resume::Auto { next: Instant::now(), tries: 0 })
                } else if tries > 40 {
                    fail(app.toasts);
                    None
                } else {
                    l.send(LinkCommand::CommandLong { command: 22, params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, target_alt] });
                    Some(Resume::Takeoff { target_alt, next: Instant::now() + std::time::Duration::from_secs(1), tries: tries + 1 })
                }
            }
            Some(Resume::Auto { next, tries }) if Instant::now() >= next => {
                if cs.mode.eq_ignore_ascii_case("auto") {
                    None
                } else if tries > 30 {
                    fail(app.toasts);
                    None
                } else {
                    if let Some(k) = find("Auto") {
                        l.send(LinkCommand::SetMode(k));
                    }
                    Some(Resume::Auto { next: Instant::now() + std::time::Duration::from_secs(1), tries: tries + 1 })
                }
            }
            other => other,
        };
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let cs = app.link.map(|l| l.shared.read().cs.clone()).unwrap_or_default();
        let connected = app.connected();
        if cs.mode != self.last_mode {
            self.last_mode = cs.mode.clone();
            self.mode_changed = Instant::now();
        }
        if cs.armed && !self.last_armed {
            self.armed_at = Instant::now();
        }
        self.last_armed = cs.armed;
        self.actions_tick(app, &cs);

        let narrow = ui.available_width() < 700.0;
        if narrow {
            // phone: HUD, map, tabs stacked
            let w = ui.available_width();
            let aspect = if app.settings.hud_wide { 0.5625 } else { 0.75 };
            self.hud_ui(ui, app, &cs, connected, Vec2::new(w, (w * aspect).round()));
            egui::Panel::bottom("fd_tabs")
                .resizable(true)
                .default_size(ui.available_height() * 0.45)
                .frame(egui::Frame::new().fill(theme::bg()))
                .show(ui, |ui| self.tabs_ui(ui, app, &cs));
            egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| self.map_area(ui, app, &cs));
            return;
        }

        // MainH: SplitterDistance = 300, FixedPanel = Panel1
        let r = egui::Panel::left("MainH")
            .resizable(true)
            .default_size(self.left_w)
            .size_range(200.0..=ui.available_width() * 0.7)
            .frame(egui::Frame::new().fill(theme::bg()))
            .show(ui, |ui| {
                let w = ui.available_width();
                // SubMainLeft: HUD keeps a 4:3 aspect
                let aspect = if app.settings.hud_wide { 0.5625 } else { 0.75 };
                let hud_h = (w * aspect).round().min(ui.available_height() - 120.0);
                if app.settings.hud_swap {
                    ui.allocate_ui(Vec2::new(w, hud_h), |ui| {
                        ui.set_min_size(Vec2::new(w, hud_h));
                        self.map_area(ui, app, &cs);
                    });
                } else {
                    self.hud_ui(ui, app, &cs, connected, Vec2::new(w, hud_h));
                }
                ui.add_space(4.0); // splitter
                self.tabs_ui(ui, app, &cs);
            });
        self.left_w = r.response.rect.width();
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(theme::bg()).inner_margin(egui::Margin { left: 4, right: 1, top: 4, bottom: 1 }))
            .show(ui, |ui| {
                if app.settings.hud_swap {
                    let size = ui.available_size();
                    self.hud_ui(ui, app, &cs, connected, size);
                } else {
                    self.map_area(ui, app, &cs);
                }
            });
    }

    fn hud_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState, connected: bool, size: Vec2) {
        // developer switch for captures: MP_GSTREAMER=1 starts the configured GStreamer source
        if self.gst.is_none() && self.gst_download.is_none() && std::env::var_os("MP_GSTREAMER").is_some() && !ui.ctx().data(|d| d.get_temp::<bool>(egui::Id::new("mp_gst_auto"))).unwrap_or(false) {
            ui.ctx().data_mut(|d| d.insert_temp(egui::Id::new("mp_gst_auto"), true));
            let url = app.settings.planner.get("gstreamer_url").cloned().unwrap_or_else(|| crate::gstreamer::DEFAULT_URL.to_string());
            self.start_gstreamer(&url, app, ui.ctx());
        }
        // both sources draw into the HUD background; the GStreamer frame wins while it runs
        let gst_frame = self.gst.as_mut().and_then(|v| v.texture(ui.ctx()));
        let video = gst_frame.or_else(|| self.video.as_mut().and_then(|v| v.texture(ui.ctx())));
        if let Some(e) = self.video.as_ref().and_then(|v| v.error.lock().unwrap().take()) {
            app.toasts.error(format!("Video source failed\n{e}"));
            self.video = None;
        }
        if let Some(e) = self.gst.as_ref().and_then(|v| v.error.lock().unwrap().take()) {
            app.toasts.error(e);
            self.gst = None;
        }
        let opts = app.settings.hud.clone();
        let (click, resp) = hud::show(
            ui,
            &hud::HudInput {
                cs,
                connected,
                mode_changed_secs: self.mode_changed.elapsed().as_secs_f64(),
                armed_secs: self.armed_at.elapsed().as_secs_f64(),
                opts: &opts,
                video,
            },
            size,
        );
        crate::httpserver::set_hud_region(resp.rect);
        resp.context_menu(|ui| self.hud_menu(ui, app));
        self.hud_prompt_ui(ui.ctx(), app, cs);
        if let Some(c) = click {
            self.hud_window = Some(c);
        }
        self.hud_windows(ui.ctx(), cs);
    }

    pub fn start_video(&mut self, url: &str, ctx: &egui::Context) {
        self.video = Some(crate::mjpeg::Mjpeg::start(url, ctx));
    }

    pub fn stop_video(&mut self) {
        self.video = None;
    }

    /// contextMenuStripHud
    fn hud_menu(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        ui.menu_button(crate::i18n::t("Video"), |ui| {
            let rec = self.video.as_ref().map(|v| v.recording()).unwrap_or(false);
            if ui.add_enabled(self.video.is_some() && !rec, egui::Button::new("Record Hud to AVI")).clicked() {
                if let Some(p) = crate::platform::save_file_as("Record Hud to AVI", "hud.avi", "AVI", &["avi"]) {
                    if let Some(v) = &self.video {
                        if let Err(e) = v.record(Some(p)) {
                            app.toasts.error(e.to_string());
                        }
                    }
                }
                ui.close();
            }
            if ui.add_enabled(rec, egui::Button::new("Stop Record")).clicked() {
                if let Some(v) = &self.video {
                    let _ = v.record(None);
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Set MJPEG source")).clicked() {
                self.hud_prompt = Some(HudPrompt::Mjpeg(app.settings.mjpeg_url.clone()));
                ui.close();
            }
            if ui.button(crate::i18n::t("Start Camera")).clicked() {
                if self.video.is_none() {
                    self.video = Some(crate::mjpeg::Mjpeg::start(&app.settings.mjpeg_url, ui.ctx()));
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Set GStreamer Source")).clicked() {
                let url = app.settings.planner.get("gstreamer_url").cloned().unwrap_or_else(|| crate::gstreamer::DEFAULT_URL.to_string());
                self.hud_prompt = Some(HudPrompt::GStreamer(url));
                ui.close();
            }
            if ui.button(crate::i18n::t("HereLink Video")).clicked() {
                let ip = app.settings.planner.get("herelinkip").cloned().unwrap_or_else(|| "192.168.43.1".to_string());
                self.hud_prompt = Some(HudPrompt::HereLink(ip));
                ui.close();
            }
            if ui.button(crate::i18n::t("GStreamer Stop")).clicked() {
                // hudGStreamer.Stop(): the MJPEG camera keeps running
                self.gst = None;
                ui.close();
            }
        });
        if ui.button(if app.settings.hud_wide { "Set Aspect Ratio (4:3)" } else { "Set Aspect Ratio" }).clicked() {
            app.settings.hud_wide = !app.settings.hud_wide;
            ui.close();
        }
        if ui.button(crate::i18n::t("User Items")).clicked() {
            self.hud_prompt = Some(HudPrompt::UserItems);
            ui.close();
        }
        if ui.checkbox(&mut app.settings.hud.russian, crate::i18n::t("Russian Hud")).clicked() {
            ui.close();
        }
        if ui.button(crate::i18n::t("Swap With Map")).clicked() {
            app.settings.hud_swap = !app.settings.hud_swap;
            ui.close();
        }
        if ui.button(crate::i18n::t("Ground Color")).clicked() {
            let c = app.settings.hud.ground.unwrap_or([0x9b, 0xb8, 0x24, 0x41, 0x4f, 0x07]);
            self.hud_prompt = Some(HudPrompt::Ground([c[0], c[1], c[2]], [c[3], c[4], c[5]]));
            ui.close();
        }
        if ui.button(crate::i18n::t("Battery Cell Voltage")).clicked() {
            self.hud_prompt = Some(HudPrompt::Cells(app.settings.hud.cells.to_string()));
            ui.close();
        }
        if ui.checkbox(&mut app.settings.hud.icons, crate::i18n::t("Show icons")).clicked() {
            ui.close();
        }
    }

    /// GStreamer.GstLaunch = LookForGstreamer(); download when missing, then hudGStreamer.Start.
    fn start_gstreamer(&mut self, pipeline: &str, app: &mut AppCtx, ctx: &egui::Context) {
        let data = crate::platform::data_dir();
        match crate::gstreamer::look_for_gstreamer(&data) {
            Some(lib) => {
                app.settings.planner.insert("gstlaunchexe".into(), lib.display().to_string());
                self.gst = Some(crate::mjpeg::Mjpeg::gstreamer(lib, pipeline, ctx));
            }
            None if self.gst_download.is_none() => {
                let status = std::sync::Arc::new(std::sync::Mutex::new((0, "Downloading..".to_string())));
                let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
                let cancel = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
                let (s, d, c, k) = (status.clone(), done.clone(), ctx.clone(), cancel.clone());
                std::thread::spawn(move || {
                    let report = |p: i32, t: &str| {
                        *s.lock().unwrap() = (p, t.to_string());
                        c.request_repaint();
                    };
                    let _ = crate::gstreamer::download_cancellable(&data, &report, &k);
                    d.store(true, std::sync::atomic::Ordering::Relaxed);
                    c.request_repaint();
                });
                self.gst_download = Some((status, done, pipeline.to_string(), cancel));
            }
            None => {}
        }
    }

    /// ProgressReporterDialogue for the GStreamer download.
    fn gst_download_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some((status, done, pipeline, cancel)) = self.gst_download.clone() else { return };
        let (pct, text) = status.lock().unwrap().clone();
        egui::Window::new(crate::i18n::t("Progress")).collapsible(false).resizable(false).fixed_size(Vec2::new(370.0, 110.0)).show(ctx, |ui| {
            theme::label(ui, &text);
            theme::progress(ui, pct.max(0) as f32 / 100.0, Vec2::new(358.0, 23.0), "");
            // ProgressReporterDialogue's Cancel: "User Request"
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                if theme::button(ui, "Cancel").clicked() {
                    cancel.store(true, std::sync::atomic::Ordering::Relaxed);
                }
            });
        });
        if done.load(std::sync::atomic::Ordering::Relaxed) {
            self.gst_download = None;
            if cancel.load(std::sync::atomic::Ordering::Relaxed) {
                return;
            }
            match crate::gstreamer::look_for_gstreamer(&crate::platform::data_dir()) {
                Some(_) => self.start_gstreamer(&pipeline, app, ctx),
                None => app.toasts.error(text),
            }
        }
    }

    fn hud_prompt_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx, cs: &CurrentState) {
        self.gst_download_ui(ctx, app);
        let Some(p) = self.hud_prompt.as_mut() else { return };
        let mut done = None;
        let title = match p {
            HudPrompt::Mjpeg(_) => "MJPEG source",
            HudPrompt::GStreamer(_) => "GStreamer url",
            HudPrompt::HereLink(_) => "herelink ip",
            HudPrompt::Cells(_) => "Battery Cell Voltage",
            HudPrompt::Ground(..) => "Ground Color",
            HudPrompt::UserItems => "User Items",
        };
        egui::Window::new(title).collapsible(false).resizable(false).show(ctx, |ui| {
            match p {
                HudPrompt::Mjpeg(u) => {
                    theme::label(ui, "Enter MJPEG url");
                    ui.add(egui::TextEdit::singleline(u).desired_width(320.0));
                }
                HudPrompt::GStreamer(u) => {
                    theme::label(ui, "Enter the source pipeline\nEnsure the final payload is ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink");
                    ui.add(egui::TextEdit::singleline(u).desired_width(420.0));
                }
                HudPrompt::HereLink(ip) => {
                    theme::label(ui, "Enter herelink ip address");
                    ui.add(egui::TextEdit::singleline(ip).desired_width(200.0));
                }
                HudPrompt::Cells(c) => {
                    theme::label(ui, "Number of cells (0 shows the pack voltage)");
                    ui.text_edit_singleline(c);
                }
                HudPrompt::Ground(a, b) => {
                    ui.horizontal(|ui| {
                        theme::label(ui, "Top");
                        ui.color_edit_button_srgb(a);
                        theme::label(ui, "Bottom");
                        ui.color_edit_button_srgb(b);
                    });
                }
                HudPrompt::UserItems => {
                    egui::ScrollArea::vertical().max_height(360.0).show(ui, |ui| {
                        egui::Grid::new("hud_items").num_columns(4).show(ui, |ui| {
                            for (k, (n, _)) in cs.fields().into_iter().enumerate() {
                                let mut on = app.settings.hud.items.iter().any(|s| s == n);
                                if ui.checkbox(&mut on, n).changed() {
                                    if on {
                                        app.settings.hud.items.push(n.to_string());
                                    } else {
                                        app.settings.hud.items.retain(|s| s != n);
                                    }
                                }
                                if k % 4 == 3 {
                                    ui.end_row();
                                }
                            }
                        });
                    });
                }
            }
            ui.horizontal(|ui| {
                if theme::button(ui, "OK").clicked() {
                    done = Some(true);
                }
                if theme::button(ui, "Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        let Some(ok) = done else { return };
        let p = self.hud_prompt.take().unwrap();
        if !ok {
            // InputBox.Show's result is ignored for HereLink: the address is saved and used
            match p {
                HudPrompt::HereLink(ip) => {
                    app.settings.planner.insert("herelinkip".into(), ip.clone());
                    self.start_gstreamer(&crate::gstreamer::herelink_url(&ip), app, ctx);
                }
                // setGStreamerSource: a cancelled prompt stops hudGStreamer
                HudPrompt::GStreamer(_) => self.gst = None,
                _ => {}
            }
            return;
        }
        match p {
            HudPrompt::Mjpeg(u) => {
                app.settings.mjpeg_url = u.clone();
                self.video = Some(crate::mjpeg::Mjpeg::start(&u, ctx));
            }
            HudPrompt::GStreamer(u) => {
                app.settings.planner.insert("gstreamer_url".into(), u.clone());
                self.start_gstreamer(&u, app, ctx);
            }
            HudPrompt::HereLink(ip) => {
                app.settings.planner.insert("herelinkip".into(), ip.clone());
                self.start_gstreamer(&crate::gstreamer::herelink_url(&ip), app, ctx);
            }
            HudPrompt::Cells(c) => app.settings.hud.cells = c.trim().parse().unwrap_or(0),
            HudPrompt::Ground(a, b) => app.settings.hud.ground = Some([a[0], a[1], a[2], b[0], b[1], b[2]]),
            HudPrompt::UserItems => {}
        }
    }

    /// EKF Status / Vibration / PreArm windows opened from the HUD.
    fn hud_windows(&mut self, ctx: &egui::Context, cs: &CurrentState) {
        let Some(which) = self.hud_window else { return };
        let mut open = true;
        let title = match which {
            hud::HudClick::Ekf => "EKF Status",
            hud::HudClick::Vibe => "Vibration",
            hud::HudClick::PreArm => "PreArm Checks",
        };
        egui::Window::new(title).open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| match which {
            hud::HudClick::Ekf => {
                for (n, v) in [("velocity_variance", cs.ekf_velv), ("pos_horiz_variance", cs.ekf_posh), ("pos_vert_variance", cs.ekf_posv), ("compass_variance", cs.ekf_compass), ("terrain_alt_variance", cs.ekf_terrain)] {
                    ui.horizontal(|ui| {
                        ui.add_sized(Vec2::new(130.0, 18.0), egui::Label::new(RichText::new(n).color(Color32::WHITE)));
                        let col = if v > 0.8 { Color32::RED } else if v > 0.5 { Color32::from_rgb(255, 165, 0) } else { theme::green() };
                        let (r, _) = ui.allocate_exact_size(Vec2::new(200.0, 16.0), Sense::hover());
                        ui.painter().rect_filled(r, 0.0, theme::control_bg());
                        ui.painter().rect_filled(Rect::from_min_size(r.min, Vec2::new(r.width() * v.clamp(0.0, 1.0), r.height())), 0.0, col);
                        theme::label(ui, format!("{v:.2}"));
                    });
                }
                ui.separator();
                let names = ["attitude", "velocity_horiz", "velocity_vert", "pos_horiz_rel", "pos_horiz_abs", "pos_vert_abs", "pos_vert_agl", "const_pos_mode", "pred_pos_horiz_rel", "pred_pos_horiz_abs", "uninitialized", "gps_glitch"];
                egui::Grid::new("ekfflags").num_columns(2).show(ui, |ui| {
                    for (i, n) in names.iter().enumerate() {
                        let on = cs.ekfflags & (1 << i) != 0;
                        theme::label(ui, *n);
                        ui.label(RichText::new(if on { "True" } else { "False" }).color(if on { theme::green() } else { Color32::GRAY }));
                        ui.end_row();
                    }
                });
            }
            hud::HudClick::Vibe => {
                for (n, v) in [("Vibe X", cs.vibex), ("Vibe Y", cs.vibey), ("Vibe Z", cs.vibez)] {
                    ui.horizontal(|ui| {
                        ui.add_sized(Vec2::new(60.0, 18.0), egui::Label::new(RichText::new(n).color(Color32::WHITE)));
                        let col = if v > 60.0 { Color32::RED } else if v > 30.0 { Color32::from_rgb(255, 165, 0) } else { theme::green() };
                        let (r, _) = ui.allocate_exact_size(Vec2::new(220.0, 16.0), Sense::hover());
                        ui.painter().rect_filled(r, 0.0, theme::control_bg());
                        ui.painter().rect_filled(Rect::from_min_size(r.min, Vec2::new(r.width() * (v / 90.0).clamp(0.0, 1.0), r.height())), 0.0, col);
                        theme::label(ui, format!("{v:.1}"));
                    });
                }
                theme::label(ui, format!("Clipping  Primary {}  Secondary {}  Tertiary {}", cs.clip0, cs.clip1, cs.clip2));
            }
            hud::HudClick::PreArm => {
                theme::label(ui, if cs.sensors_health & 0x1000_0000 != 0 { "Ready to Arm" } else { "Not Ready to Arm" });
            }
        });
        if !open {
            self.hud_window = None;
        }
    }

    // ------------------------------------------------------------------ tabControlactions

    fn tabs_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        // tabControlactions order (tabActionsSimple after Messages); the DisplayView layout and
        // Customize decide which are shown. self.tab is an index into TABS, SIMPLE_ACTIONS for
        // the simple Actions tab.
        let order: Vec<usize> = [0usize, 1, 2, SIMPLE_ACTIONS].into_iter().chain(3..TABS.len()).collect();
        let key = |i: usize| if i == SIMPLE_ACTIONS { "ActionsSimple" } else { TABS[i] };
        let pl = &app.settings.planner;
        let visible: Vec<usize> = order.into_iter().filter(|i| crate::display::fd_tab(pl, key(*i)) && !app.settings.fd_hidden_tabs.iter().any(|h| h == key(*i))).collect();
        let visible = if visible.is_empty() { vec![0] } else { visible };
        let labels: Vec<&str> = visible.iter().map(|i| if *i == SIMPLE_ACTIONS { "Actions" } else { TABS[*i] }).collect();
        let mut sel = visible.iter().position(|i| *i == self.tab).unwrap_or(0);
        let strip_top = ui.available_rect_before_wrap().min;
        let strip = if app.settings.fd_tabs_multiline {
            theme::tab_strip_multiline(ui, "tabControlactions", &labels, &mut sel)
        } else {
            theme::tab_strip(ui, "tabControlactions", &labels, &mut sel);
            Rect::from_min_max(strip_top, egui::pos2(ui.available_rect_before_wrap().right(), ui.available_rect_before_wrap().top()))
        };
        self.tab = visible[sel.min(visible.len() - 1)];
        // contextMenuStripactionstab: Customize / MultiLine
        ui.interact(strip, ui.scope_id().with("tabs_ctx"), Sense::click()).context_menu(|ui| {
            if ui.button(crate::i18n::t("Customize")).clicked() {
                self.tabs_customize = true;
                ui.close();
            }
            if ui.checkbox(&mut app.settings.fd_tabs_multiline, crate::i18n::t("MultiLine")).clicked() {
                ui.close();
            }
        });
        if self.tabs_customize {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Customize")).open(&mut open).collapsible(false).resizable(false).show(ui.ctx(), |ui| {
                for t in TABS {
                    let mut on = !app.settings.fd_hidden_tabs.iter().any(|h| h == t);
                    if ui.checkbox(&mut on, t).changed() {
                        if on {
                            app.settings.fd_hidden_tabs.retain(|h| h != t);
                        } else {
                            app.settings.fd_hidden_tabs.push(t.to_string());
                        }
                    }
                }
            });
            if !open {
                self.tabs_customize = false;
            }
        }
        let rect = ui.available_rect_before_wrap();
        ui.painter().rect_filled(rect, 0.0, theme::bg());
        ui.scope_builder(egui::UiBuilder::new().max_rect(rect.shrink(3.0)), |ui| {
            if self.tab == 6 {
                status_ui(ui, cs);
                return;
            }
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| match self.tab {
                0 => self.quick_ui(ui, app, cs),
                1 => self.actions_ui(ui, app, cs),
                SIMPLE_ACTIONS => self.simple_actions_ui(ui, app),
                2 => messages_ui(ui, app),
                3 => self.checklist.ui(ui, app, cs),
                4 => gauges_ui(ui, cs, app.assets),
                5 => self.extra.transponder(ui, app),
                6 => { status_ui(ui, cs); }
                7 => servo_ui(ui, app, cs),
                8 => self.extra.aux(ui, app),
                9 => self.extra.scripts(ui, app),
                10 => {
                    self.extra.payload(ui, app);
                    if self.extra.open_gimbal_video {
                        self.extra.open_gimbal_video = false;
                        self.gimbal.set_placement(GimbalPlacement::PopOut, app, ui.ctx());
                    }
                }
                11 => self.tlogs_ui(ui, app),
                _ => self.dflogs_ui(ui, app),
            });
        });
    }

    /// tabActionsSimple (Basic layout): Auto, Loiter and RTL.
    fn simple_actions_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.max_rect().min;
        for (y, text, mode) in [(20.0, "Auto", "Auto"), (49.0, "Loiter", "Loiter"), (78.0, "RTL", "RTL")] {
            let r = Rect::from_min_size(o + Vec2::new(150.0, y), Vec2::new(47.0, 23.0));
            if ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, text, r.size())).clicked() {
                // BUT_quickauto / quickmanual / quickrtl: setMode, "Command Failed" when it throws
                match app.link {
                    Some(l) => {
                        let fw = l.shared.read().cs.firmware;
                        if mode == "Loiter" && !matches!(fw, Firmware::ArduPlane | Firmware::ArduRover | Firmware::ArduCopter2) {
                            continue;
                        }
                        match fw.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case(mode)) {
                            Some((k, _)) => l.send(LinkCommand::SetMode(k)),
                            None => app.toasts.message(COMMAND_FAILED, "Error"),
                        }
                    }
                    None => app.toasts.message(COMMAND_FAILED, "Error"),
                }
            }
        }
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(287.0, 110.0)), Sense::hover());
    }

    fn quick_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        if app.settings.quick_undocked {
            theme::label(ui, "Quick view is undocked (right-click it to dock)");
            let mut open = true;
            egui::Window::new(crate::i18n::t("Quick")).open(&mut open).default_size([300.0, 240.0]).show(ui.ctx(), |ui| self.quick_grid(ui, app, cs));
            if !open {
                app.settings.quick_undocked = false;
            }
        } else {
            self.quick_grid(ui, app, cs);
        }
        self.quick_dialogs(ui.ctx(), app, cs);
        self.quick_ui_picker(ui, app, cs);
    }

    /// tableLayoutPanelQuick: quick_cols x quick_rows tiles, Dock = Fill.
    fn quick_grid(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        let (rows, cols) = (app.settings.quick_rows.clamp(1, 10), app.settings.quick_cols.clamp(1, 6));
        let defaults = ["alt", "groundspeed", "wp_dist", "yaw", "verticalspeed", "DistToHome"];
        while app.settings.quick_fields.len() < rows * cols {
            let k = app.settings.quick_fields.len();
            app.settings.quick_fields.push(defaults[k % defaults.len()].to_string());
        }
        let area = ui.available_rect_before_wrap();
        let h = area.height().max(50.0 * rows as f32) / rows as f32;
        let w = area.width() / cols as f32;
        for i in 0..rows * cols {
            let (col, row) = (i % cols, i / cols);
            let r = Rect::from_min_size(area.min + Vec2::new(col as f32 * w, row as f32 * h), Vec2::new(w, h)).shrink(3.0);
            let name = app.settings.quick_fields[i].clone();
            let val = crate::units::field(&name, cs.field(&name).unwrap_or(0.0)).0;
            let resp = ui.interact(r, ui.scope_id().with(("qv", i)), Sense::click());
            if self.quick_views.len() <= i {
                self.quick_views.resize_with(i + 1, Default::default);
            }
            let qv = &mut self.quick_views[i];
            qv.set_desc(&quick_label(&app.settings.planner, i, &name));
            qv.set_number(val);
            qv.numberColor = QUICK_COLORS[i % QUICK_COLORS.len()];
            qv.OnPaintSurface(ui.painter(), r, fonts::UI_SIZE * 72.0 / 96.0, Color32::WHITE);
            if resp.double_clicked() {
                if !crate::display::flag(&app.settings.planner, "lockQuickView") {
                    self.quick_edit = Some(i);
                }
            }
            // contextMenuStripQuickView
            resp.context_menu(|ui| {
                if ui.button(crate::i18n::t("Set View Count")).clicked() {
                    self.quick_count = Some((app.settings.quick_cols.to_string(), app.settings.quick_rows.to_string()));
                    ui.close();
                }
                if ui.button(if app.settings.quick_undocked { "Dock" } else { "Undock" }).clicked() {
                    app.settings.quick_undocked = !app.settings.quick_undocked;
                    ui.close();
                }
            });
        }
        ui.allocate_rect(Rect::from_min_size(area.min, Vec2::new(area.width(), h * rows as f32)), Sense::hover());
    }

    fn quick_dialogs(&mut self, ctx: &egui::Context, app: &mut AppCtx, cs: &CurrentState) {
        if let Some((c, r)) = self.quick_count.as_mut() {
            let mut done = None;
            egui::Window::new(crate::i18n::t("Set View Count")).collapsible(false).resizable(false).show(ctx, |ui| {
                theme::label(ui, "Enter number of columns to have.");
                ui.text_edit_singleline(c);
                theme::label(ui, "Enter number of rows to have.");
                ui.text_edit_singleline(r);
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() {
                        done = Some(true);
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        done = Some(false);
                    }
                });
            });
            if let Some(ok) = done {
                if ok {
                    if let (Ok(c), Ok(r)) = (c.trim().parse::<usize>(), r.trim().parse::<usize>()) {
                        if c * r >= 1 && c * r <= 30 {
                            app.settings.quick_cols = c;
                            app.settings.quick_rows = r;
                        }
                    }
                }
                self.quick_count = None;
            }
        }
        let _ = cs;
    }

    fn quick_ui_picker(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {

        if let Some(i) = self.quick_edit {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Display This")).open(&mut open).collapsible(false).show(ui.ctx(), |ui| {
                egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                    // the fields by GetFieldDesc, sorted by that description
                    let mut fields: Vec<(&str, String)> = cs.fields().into_iter().map(|(n, _)| (n, field_desc(n))).collect();
                    fields.sort_by(|a, b| a.1.to_lowercase().cmp(&b.1.to_lowercase()).then(a.1.cmp(&b.1)));
                    egui::Grid::new("qsel").num_columns(4).show(ui, |ui| {
                        for (k, (n, desc)) in fields.into_iter().enumerate() {
                            if ui.checkbox(&mut (app.settings.quick_fields[i] == n), desc).clicked() {
                                app.settings.quick_fields[i] = n.to_string();
                                // Settings["quickView" + f]
                                app.settings.planner.insert(format!("quickView{}", i + 1), n.to_string());
                                self.quick_edit = None;
                            }
                            if k % 4 == 3 {
                                ui.end_row();
                            }
                        }
                    });
                });
            });
            if !open {
                self.quick_edit = None;
            }
        }
    }

    fn actions_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        let link = app.link.filter(|l| l.status() == LinkStatus::Connected);
        let modes = cs.firmware.modes();
        // tableLayoutPanel1: anchored left/right, 5 columns of 20% and 5 rows of 36 px
        let origin = ui.available_rect_before_wrap().min;
        let cw = ui.available_width().max(287.0) / 5.0;
        let col_x = [0.0, cw, cw * 2.0, cw * 3.0, cw * 4.0];
        let col_w = [cw; 5];
        let row_y = [0.0, 36.0, 72.0, 108.0, 144.0];
        let cell = |c: usize, r: usize| Rect::from_min_size(origin + Vec2::new(col_x[c], row_y[r]), Vec2::new(col_w[c], 36.0)).shrink(3.0);
        let mut clicked: Option<&str> = None;
        let mut btn = |ui: &mut egui::Ui, c: usize, r: usize, text: &'static str| {
            let rr = cell(c, r);
            if ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, text, rr.size())).clicked() {
                clicked = Some(text);
            }
        };
        let rr = cell(0, 0);
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_action").wrap_mode(egui::TextWrapMode::Truncate)
                .width(rr.width() - 4.0)
                .selected_text(ACTIONS[self.action_sel].0)
                .show_ui(ui, |ui| {
                    for (i, a) in ACTIONS.iter().enumerate() {
                        ui.selectable_value(&mut self.action_sel, i, a.0);
                    }
                })
                .response
        });
        let rr = cell(0, 1);
        let wp_count = self.mission.len().max(1) as u16;
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_setwp").wrap_mode(egui::TextWrapMode::Truncate)
                .width(rr.width() - 4.0)
                .selected_text(self.wp_sel.to_string())
                .show_ui(ui, |ui| {
                    for i in 0..wp_count {
                        ui.selectable_value(&mut self.wp_sel, i, i.to_string());
                    }
                })
                .response
        });
        let sel_mode = self.mode_sel.unwrap_or(cs.custom_mode);
        let sel_name = modes.iter().find(|(k, _)| *k == sel_mode).map(|(_, n)| n.clone()).unwrap_or_default();
        let rr = cell(0, 2);
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_modes").wrap_mode(egui::TextWrapMode::Truncate)
                .width(rr.width() - 4.0)
                .selected_text(sel_name)
                .show_ui(ui, |ui| {
                    for (k, n) in &modes {
                        if ui.selectable_label(sel_mode == *k, n).clicked() {
                            self.mode_sel = Some(*k);
                        }
                    }
                })
                .response
        });
        let rr = cell(0, 3);
        let mounts = mount_modes(cs.firmware);
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_mountmode").wrap_mode(egui::TextWrapMode::Truncate)
                .width(rr.width() - 4.0)
                .selected_text(mounts.iter().find(|(k, _)| *k == self.mount_sel as i64).map(|(_, v)| v.clone()).unwrap_or_default())
                .show_ui(ui, |ui| {
                    for (k, m) in &mounts {
                        ui.selectable_value(&mut self.mount_sel, *k as usize, m);
                    }
                })
                .response
        });
        btn(ui, 1, 0, "Do Action");
        btn(ui, 1, 1, "Set WP");
        btn(ui, 1, 2, "Set Mode");
        btn(ui, 1, 3, "Set Mount");
        btn(ui, 2, 0, "Auto");
        btn(ui, 2, 1, "Loiter");
        btn(ui, 2, 2, "RTL");
        btn(ui, 2, 3, "Joystick");
        btn(ui, 2, 4, "Message");
        btn(ui, 3, 0, "Set Home Alt");
        btn(ui, 3, 1, "Restart Mission");
        btn(ui, 3, 2, "Raw Sensor View");
        btn(ui, 3, 3, "Arm/ Disarm");
        btn(ui, 3, 4, "Resume Mission");
        btn(ui, 4, 3, "Clear Track");
        btn(ui, 4, 4, "Abort Landing");
        // column 4 rows 0-2: ModifyandSet (numeric above button)
        for (r, (val, text)) in [(&mut self.change_speed, "Change Speed"), (&mut self.change_alt, "Change Alt"), (&mut self.loiter_rad, "Set Loiter Rad")]
            .into_iter()
            .enumerate()
        {
            let rr = cell(4, r);
            // ModifyandSet: FlowLayoutPanel (no margins) holding a 41x20 NumericUpDown and a
            // 68x23 MyButton; the button flows beside the number when the cell is wide enough
            // (112px) and wraps underneath it otherwise.
            let cellr = rr.expand(3.0);
            let num = Rect::from_min_size(cellr.min, Vec2::new(41.0, 20.0));
            let bot = if cellr.width() >= 109.0 {
                Rect::from_min_size(Pos2::new(cellr.left() + 41.0, cellr.top()), Vec2::new(68.0, 23.0))
            } else {
                Rect::from_min_size(Pos2::new(cellr.left(), cellr.top() + 20.0), Vec2::new(68.0, 23.0))
            };
            let mut v = *val as f64;
            if theme::numeric_updown(ui, num, &mut v, 0.0, 100000.0, 1.0, 1).changed() {
                *val = v as f32;
            }
            // the panel clips its children: a wrapped button shows only its top part
            let clip = cellr.intersect(ui.clip_rect());
            let resp = ui
                .scope_builder(egui::UiBuilder::new().max_rect(bot), |ui| {
                    ui.set_clip_rect(clip);
                    theme::button_sized(ui, text, bot.size())
                })
                .inner;
            if resp.clicked() && resp.interact_pointer_pos().map(|p| clip.contains(p)).unwrap_or(true) {
                clicked = Some(text);
            }
        }
        ui.allocate_rect(Rect::from_min_size(origin, Vec2::new(287.0, 181.0)), Sense::hover());

        let Some(text) = clicked else { return };
        // the handlers do nothing without an open port
        let Some(link) = link else { return };
        let find = |name: &str| modes.iter().find(|(_, n)| n.eq_ignore_ascii_case(name)).map(|(k, _)| *k);
        let fw = cs.firmware;
        match text {
            "Do Action" => {
                let name = ACTIONS[self.action_sel].0;
                // these run at once; the rest ask first
                match name {
                    "Format_SD_Card" => link.send(LinkCommand::CommandInt(mp_core::gimbal::CommandInt { command: 526, frame: 0, params: [1.0, 1.0, 0.0, 0.0], x: 0, y: 0, z: 0.0 })),
                    "Trigger_Camera" => {
                        // setDigicamControl: DIGICAM_CONTROL when the command is refused
                        self.action_ack = Some((203, String::new()));
                        link.send(LinkCommand::CommandLong { command: 203, params: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0] });
                    }
                    "Scripting_cmd_stop_and_restart" | "Scripting_cmd_stop" => {
                        let c = if name == "Scripting_cmd_stop" { 2.0 } else { 3.0 };
                        link.send(LinkCommand::CommandInt(mp_core::gimbal::CommandInt { command: 42701, frame: 0, params: [c, 0.0, 0.0, 0.0], x: 0, y: 0, z: 0.0 }));
                    }
                    "System_Time" => {
                        use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, SYSTEM_TIME_DATA};
                        let us = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_micros() as u64).unwrap_or(0);
                        link.send(LinkCommand::Send(Box::new(MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA { time_unix_usec: us, time_boot_ms: 0 }))));
                    }
                    _ => {
                        self.prompt = Some(Prompt::Confirm { title: "Action".into(), text: format!("Are you sure you want to do {name} ?"), then: Confirmed::DoAction(self.action_sel) });
                    }
                }
            }
            "Set WP" => link.send(LinkCommand::SetCurrentWp(self.wp_sel)),
            "Set Mode" => {
                // BUT_setmode_Click asks first while in failsafe
                if cs.field("failsafe").unwrap_or(0.0) != 0.0 {
                    self.prompt = Some(Prompt::Confirm { title: "Failsafe".into(), text: "You are in failsafe, are you sure?".into(), then: Confirmed::SetMode(sel_mode) });
                } else {
                    link.send(LinkCommand::SetMode(sel_mode));
                }
            }
            // BUT_quickmanual_Click only switches Plane, Ateryx (not a firmware here), Rover and Copter
            "Loiter" if !matches!(fw, Firmware::ArduPlane | Firmware::ArduRover | Firmware::ArduCopter2) => {}
            "Auto" | "Loiter" | "RTL" => {
                if let Some(k) = find(text) {
                    link.send(LinkCommand::SetMode(k));
                }
            }
            // BUT_ARM_Click: only disarming asks first
            "Arm/ Disarm" => {
                if cs.armed {
                    self.prompt = Some(Prompt::Confirm { title: "Disarm".into(), text: "Are you sure you want to Disarm".into(), then: Confirmed::Arm(false) });
                } else {
                    self.arm_pending = Some((true, link.shared.read().messages.len(), Instant::now()));
                    link.send(LinkCommand::Arm { arm: true, force: false });
                }
            }
            // BUTrestartmission_Click: setWPCurrent(0)
            "Restart Mission" => link.send(LinkCommand::SetCurrentWp(0)),
            "Resume Mission" => {
                self.prompt = Some(Prompt::Confirm { title: "Resume Mission".into(), text: "Warning this will reprogram your mission, arm and issue a takeoff command (copter)".into(), then: Confirmed::ResumeWarn });
            }
            "Clear Track" => {
                let mut s = link.shared.write();
                s.cs.track.clear();
                s.cs.camerapoints.clear();
            }
            "Change Speed" => link.send(LinkCommand::CommandLong { command: 178, params: [0.0, self.change_speed, 0.0, 0.0, 0.0, 0.0, 0.0] }),
            // modifyandSetAlt: (int)Value / multiplieralt
            "Change Alt" => link.send(LinkCommand::ChangeAltitude { alt: (self.change_alt.trunc()) / crate::units::alt(1.0) }),
            // modifyandSetLoiterRad: setParam(["LOITER_RAD", "WP_LOITER_RAD"], (int)Value / multiplierdist)
            "Set Loiter Rad" => {
                let _ = fw;
                let name = {
                    let s = link.shared.read();
                    ["LOITER_RAD", "WP_LOITER_RAD"].into_iter().find(|n| s.params.contains_key(*n))
                };
                // setParam(string[]) returns false when neither exists; the handler ignores it
                if let Some(n) = name {
                    link.send(LinkCommand::SetParam { name: n.into(), value: self.loiter_rad.trunc() / crate::units::dist(1.0) });
                }
            }
            "Abort Landing" => link.send(LinkCommand::CommandLong { command: 191, params: [0.0; 7] }),
            "Set Home Alt" => {
                // BUT_Homealt: altoffsethome = -HomeAlt / multiplieralt, so alt reads above sea level (toggle back)
                let mut s = link.shared.write();
                if s.cs.altoffsethome != 0.0 {
                    s.cs.altoffsethome = 0.0;
                } else {
                    let home_alt = s.cs.home.map(|h| h.alt as f32).unwrap_or(0.0);
                    s.cs.altoffsethome = -home_alt / crate::units::alt(1.0);
                }
            }
            "Raw Sensor View" => self.raw_sensor = true,
            "Message" => {
                self.prompt = Some(Prompt::Input {
                    title: "Enter Message".into(),
                    fields: vec![("Enter Message to be logged".into(), String::new())],
                    then: Inputted::SendMessage,
                })
            }
            // BUT_mountmode_Click: the MNT_MODE parameter, else DO_MOUNT_CONTROL with the mode in p7
            "Set Mount" => {
                if link.shared.read().params.contains_key("MNT_MODE") {
                    link.send(LinkCommand::SetParam { name: "MNT_MODE".into(), value: self.mount_sel as f32 });
                } else {
                    link.send(LinkCommand::CommandLong { command: 205, params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.mount_sel as f32] });
                }
            }
            "Joystick" => app.joystick.window_open = true,
            _ => {}
        }
    }

    // ------------------------------------------------------------------ tableMap

    fn map_area(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        // panel1 (25px) along the bottom: coords1, Tuning, Auto Pan
        egui::Panel::bottom("fd_panel1").exact_size(25.0).frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
            let o = ui.max_rect().min;
            let coords = Rect::from_min_size(o + Vec2::new(7.0, 2.0), Vec2::new(247.0, 21.0));
            ui.painter().rect_filled(coords, 0.0, theme::control_bg());
            ui.painter().rect_stroke(coords, 0.0, Stroke::new(1.0, Color32::from_rgb(0x7a, 0x7a, 0x7a)), StrokeKind::Inside);
            ui.painter().text(
                coords.left_center() + Vec2::new(4.0, 0.0),
                egui::Align2::LEFT_CENTER,
                format!("{:.6} {:.6}  {:.2}  GEO", cs.lat, cs.lng, cs.alt),
                fonts::ui_id(fonts::UI_SIZE),
                Color32::WHITE,
            );
            ui.put(Rect::from_min_size(o + Vec2::new(260.0, 3.0), Vec2::new(59.0, 17.0)), egui::Checkbox::new(&mut self.tuning, crate::i18n::t("Tuning")));
            ui.put(Rect::from_min_size(o + Vec2::new(325.0, 3.0), Vec2::new(75.0, 17.0)), egui::Checkbox::new(&mut self.auto_pan, crate::i18n::t("Auto Pan")));
        });

        egui::CentralPanel::default().frame(egui::Frame::new()).show(ui, |ui| {
            if self.tuning {
                // splitContainer1.Panel1: zg1 tuning graph
                let size = Vec2::new(ui.available_width(), ui.available_height() * 0.5);
                self.graph.ui(ui, cs, size);
                ui.add_space(4.0);
            }
            self.map_ui(ui, app, cs);
        });
    }

    fn map_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        let has_pos = cs.lat != 0.0 || cs.lng != 0.0;
        if self.auto_pan && has_pos {
            app.settings.fd_map.center_on(cs.lat, cs.lng);
        }
        let mission = self.mission.clone();
        let home = cs.home;
        let assets = app.assets;
        let zoom = app.settings.fd_map.zoom;
        let target = active_guided(app.link);
        let guided = if cs.mode.eq_ignore_ascii_case("Guided") && target.lat != 0.0 {
            Some((target.lat, target.lng))
        } else { None };
        let pois = app.settings.pois.clone();
        // the other vehicles on the link (MAVlist), drawn without heading lines
        let others: Vec<(u8, f64, f64, f32)> = app
            .link
            .map(|l| {
                let s = l.shared.read();
                s.mavs.iter().filter(|((sys, comp), m)| (*sys, *comp) != (s.cs.sysid, s.cs.compid) && m.lat != 0.0).map(|((sys, _), m)| (*sys, m.lat, m.lng, m.heading))
                    // vehicles on the other comports
                    .chain(app.extra_links.iter().flat_map(|l| l.shared.read().mavs.iter().filter(|(_, m)| m.lat != 0.0).map(|((sys, _), m)| (*sys, m.lat, m.lng, m.heading)).collect::<Vec<_>>()))
                    .collect()
            })
            .unwrap_or_default();
        // MainV2 ADSB: traffic reported by the vehicle (Planner "ADSB" option)
        let adsb: Vec<(String, f64, f64, f32, f32)> = if app.settings.planner.get("enableadsb").map(|v| v == "True").unwrap_or(false) {
            app.link.map(|l| l.shared.read().adsb.values().map(|v| (v.0.clone(), v.1, v.2, v.3, v.4)).collect()).unwrap_or_default()
        } else {
            Vec::new()
        };
        let pl = &app.settings.planner;
        let flag = |k: &str| pl.get(k).map(|v| v == "True").unwrap_or(true);
        let line_opts = map::LineOpts {
            heading: flag("GMapMarkerBase_DisplayHeading"),
            cog: flag("GMapMarkerBase_DisplayCOG"),
            target: flag("GMapMarkerBase_DisplayTarget"),
            nav_bearing: flag("GMapMarkerBase_DisplayNavBearing"),
            length: pl.get("GMapMarkerBase_length").and_then(|v| v.parse().ok()).unwrap_or(500.0),
        };
        let track_len: usize = pl.get("NUM_tracklength").and_then(|v| v.parse().ok()).unwrap_or(200);
        if let Some(p) = super::gimbal_video::env_placement() {
            self.gimbal.set_placement(p, app, ui.ctx());
        }
        if self.gimbal.placement == GimbalPlacement::FullSized {
            let mrect = ui.available_rect_before_wrap();
            self.gimbal.render_video_box(ui, mrect, app);
            if self.gimbal.show_mini_map {
                let mini_map_rect = super::gimbal_video::mini_map_rect(mrect);
                let mut mini_ui = ui.new_child(egui::UiBuilder::new().max_rect(mini_map_rect));
                map::show(&mut mini_ui, &mut app.settings.fd_map, app.tiles, |_ui, view, _resp, painter| {
                    if let Some(h) = home {
                        map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), view.to_screen(h.lat, h.lng));
                    }
                    if has_pos {
                        let p = view.to_screen(cs.lat, cs.lng);
                        map::draw_quad(painter, p, cs.yaw, cs.sysid.max(1));
                    }
                    false
                });
                ui.painter().rect_stroke(mini_map_rect, 0.0, Stroke::new(1.0, Color32::from_gray(80)), egui::StrokeKind::Inside);
            }
            self.gimbal.extra_ui(ui.ctx(), app);
            return;
        }
        let mut context_latlng = None;
        let gimbal_marker = self.gimbal.mouse_marker;
        let out = map::show(ui, &mut app.settings.fd_map, app.tiles, |_ui, view, resp, painter| {
            let pts: Vec<Pos2> = mission
                .iter()
                .skip(1)
                .filter(|w| w.has_location() && (w.lat != 0.0 || w.lng != 0.0))
                .map(|w| view.to_screen(w.lat, w.lng))
                .collect();
            if pts.len() > 1 {
                painter.add(egui::Shape::line(pts, Stroke::new(2.0, Color32::YELLOW)));
            }
            let hover = resp.hover_pos();
            for (i, w) in mission.iter().enumerate().skip(1) {
                if w.has_location() && (w.lat != 0.0 || w.lng != 0.0) {
                    let p = view.to_screen(w.lat, w.lng);
                    let hov = hover.map(|h| (h - p).length() < 16.0).unwrap_or(false);
                    map::draw_wp(painter, assets.marker_green.id(), assets.marker_shadow.id(), p, &i.to_string(), false, zoom, hov);
                }
            }
            if let Some(h) = home {
                map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), view.to_screen(h.lat, h.lng));
            }
            if let Some((lat, lng)) = guided {
                map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), view.to_screen(lat, lng));
            }
            // GimbalVideoControl's mouseMapMarker: GMarkerGoogle blue_small, bottom centre on the point
            if let Some((lat, lng)) = gimbal_marker {
                let p = view.to_screen(lat, lng);
                let sz = assets.marker_blue_small.size_vec2();
                painter.image(assets.marker_blue_small.id(), Rect::from_min_size(p - Vec2::new(sz.x / 2.0, sz.y), sz), Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
            }
            for (callsign, lat, lng, alt, hdg) in &adsb {
                // GMapMarkerADSBPlane: plane silhouette rotated to the heading, label below
                let p = view.to_screen(*lat, *lng);
                let rot = |x: f32, y: f32| {
                    let (s, c) = hdg.to_radians().sin_cos();
                    p + Vec2::new(x * c - y * s, x * s + y * c)
                };
                let gold = Color32::from_rgb(0xff, 0xd7, 0x00);
                for (w, (x0, y0, x1, y1)) in [(4.0, (0.0, -12.0, 0.0, 11.0)), (3.5, (-11.0, 1.0, 11.0, 1.0)), (3.0, (-5.0, 10.0, 5.0, 10.0))] {
                    painter.line_segment([rot(x0, y0), rot(x1, y1)], Stroke::new(w + 2.0, Color32::BLACK));
                    painter.line_segment([rot(x0, y0), rot(x1, y1)], Stroke::new(w, gold));
                }
                painter.text(p + Vec2::new(0.0, 14.0), egui::Align2::CENTER_TOP, format!("{callsign} {alt:.0}m"), fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
            }
            for (lat, lng, name) in &pois {
                let p = view.to_screen(*lat, *lng);
                map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), p);
                painter.text(p + Vec2::new(8.0, -20.0), egui::Align2::LEFT_TOP, name, fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
            }
            if cs.track.len() > 1 {
                let skip = cs.track.len().saturating_sub(track_len);
                let t: Vec<Pos2> = cs.track.iter().skip(skip).map(|&(a, b)| view.to_screen(a, b)).collect();
                painter.add(egui::Shape::line(t, Stroke::new(2.0, Color32::from_rgb(255, 0, 255))));
            }
            for (sys, lat, lng, hdg) in &others {
                map::draw_quad(painter, view.to_screen(*lat, *lng), *hdg, *sys);
            }
            if has_pos {
                let p = view.to_screen(cs.lat, cs.lng);
                map::draw_heading_lines(painter, p, cs.yaw, cs.groundcourse, cs.target_bearing, cs.nav_bearing, &line_opts);
                map::draw_quad(painter, p, cs.yaw, cs.sysid.max(1));
            }
            if resp.secondary_clicked() {
                if let Some(pos) = resp.interact_pointer_pos() {
                    context_latlng = Some(view.to_latlng(pos));
                }
            }
            false
        });
        let mrect = out.view.rect;
        crate::httpserver::set_map_region(mrect);
        if let Some(ll) = context_latlng {
            ui.ctx().data_mut(|d| d.insert_temp(egui::Id::unique("fd_ctx_ll"), ll));
        }
        if out.response.dragged() {
            self.auto_pan = false;
        }
        let link = app.link;
        let mut ctx_action: Option<&str> = None;
        out.response.context_menu(|ui| {
            // contextMenuStripMap, in FlightData.Designer.cs order
            for item in ["Fly To Here", "Fly To Here Alt", "Fly To Coords", "Add Poi", "Point Camera Here", "Point Camera Coords", "Trigger Camera NOW", "Flight Planner"] {
                if ui.button(item).clicked() {
                    ctx_action = Some(item);
                    ui.close();
                }
            }
            ui.menu_button(crate::i18n::t("Set Home Here"), |ui| {
                if ui.button(crate::i18n::t("Set Home Here")).clicked() {
                    ctx_action = Some("Set Home Here");
                    ui.close();
                }
                if ui.button(crate::i18n::t("Set EKF Origin Here")).clicked() {
                    ctx_action = Some("Set EKF Origin Here");
                    ui.close();
                }
            });
            if ui.button(crate::i18n::t("TakeOff")).clicked() {
                ctx_action = Some("TakeOff");
                ui.close();
            }
            if ui.checkbox(&mut self.camera_overlap, crate::i18n::t("Camera Overlap")).clicked() {
                ui.close();
            }
            if ui.button(crate::i18n::t("Jump To Tag")).clicked() {
                ctx_action = Some("Jump To Tag");
                ui.close();
            }
            ui.menu_button(crate::i18n::t("Gimbal Video"), |ui| {
                if ui.button(crate::i18n::t("Full Sized")).clicked() {
                    ctx_action = Some("Gimbal Full Sized");
                    ui.close();
                }
                if ui.button(crate::i18n::t("Mini")).clicked() {
                    ctx_action = Some("Gimbal Mini");
                    ui.close();
                }
                if ui.button(crate::i18n::t("Pop Out")).clicked() {
                    ctx_action = Some("Gimbal Pop Out");
                    ui.close();
                }
            });
            // PluginHost.FDMenuMap items, at MouseDownStart
            for (label, plugin, id) in crate::plugins::menu_items(mp_plugin::MENU_FD_MAP) {
                if ui.button(label).clicked() {
                    let ll: (f64, f64) = ui.ctx().data(|d| d.get_temp(egui::Id::unique("fd_ctx_ll"))).unwrap_or_default();
                    crate::plugins::click(plugin, id, ll.0, ll.1);
                    ui.close();
                }
            }
        });
        if let Some(a) = ctx_action {
            let ll: (f64, f64) = ui.ctx().data(|d| d.get_temp(egui::Id::unique("fd_ctx_ll"))).unwrap_or_default();
            let need_link = |app: &mut AppCtx| {
                if link.is_none() {
                    app.toasts.error("Please Connect First");
                }
                link
            };
            let guided_wp = active_guided(app.link);
            match a {
                "Fly To Here" => {
                    if let Some(l) = need_link(app) {
                        if guided_wp.alt == 0.0 {
                            let (alt, frame) = guided_prompt_values(cs.firmware, &app.settings.planner);
                            self.prompt = Some(Prompt::AltInput {
                                title: "Enter Alt".into(),
                                prompt_text: "Enter Guided Mode Alt".into(),
                                alt,
                                frame,
                                then: AltInputted::FlyToHere(ll.0, ll.1),
                            });
                        } else {
                            l.send(LinkCommand::Guided { lat: ll.0, lng: ll.1, alt: guided_wp.alt, frame: guided_wp.frame });
                        }
                    }
                }
                "Fly To Here Alt" => {
                    let (alt, frame) = guided_prompt_values(cs.firmware, &app.settings.planner);
                    self.prompt = Some(Prompt::AltInput {
                        title: "Enter Alt".into(),
                        prompt_text: "Enter Guided Mode Alt".into(),
                        alt,
                        frame,
                        then: AltInputted::FlyToHereAlt,
                    });
                }
                "Fly To Coords" => {
                    self.prompt = Some(Prompt::Input {
                        title: "Fly To Coords".into(),
                        fields: vec![("Lat".into(), format!("{:.7}", ll.0)), ("Lng".into(), format!("{:.7}", ll.1)), ("Alt".into(), format!("{}", crate::units::alt(guided_wp.alt)))],
                        then: Inputted::FlyToCoords,
                    })
                }
                "Add Poi" => self.prompt = Some(Prompt::Input { title: "POI".into(), fields: vec![("Enter ID".into(), String::new())], then: Inputted::AddPoi(ll.0, ll.1) }),
                "Point Camera Here" => {
                    if link.is_none() {
                        app.toasts.error("Please Connect First");
                    } else {
                        self.prompt = Some(Prompt::Input { title: "Enter Alt".into(), fields: vec![("Enter Target Alt (Relative to home)".into(), "0".into())], then: Inputted::PointCameraHere(ll.0, ll.1) });
                    }
                }
                "Point Camera Coords" => {
                    self.prompt = Some(Prompt::Input {
                        title: "Point Camera Coords".into(),
                        fields: vec![("Lat".into(), format!("{:.7}", ll.0)), ("Lng".into(), format!("{:.7}", ll.1)), ("Alt".into(), "0".into())],
                        then: Inputted::PointCameraCoords,
                    })
                }
                // setDigicamControl(true): DIGICAM_CONTROL when the command is refused
                "Trigger Camera NOW" => {
                    if let Some(l) = link {
                        self.action_ack = Some((203, String::new()));
                        l.send(LinkCommand::CommandLong { command: 203, params: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0] });
                    }
                }
                "Flight Planner" => app.settings.view = crate::app::View::FlightPlan,
                "Set Home Here" => {
                    if link.is_some() {
                        // srtm.getAltitude must be valid (or ocean)
                        match crate::srtm::altitude(ll.0, ll.1) {
                            None => app.toasts.message("No SRTM data for this area", "Error"),
                            Some(alt) => {
                                self.prompt = Some(Prompt::Confirm { title: "Are you sure?".into(), text: "This will reset the onboard home position (effects RTL etc). Are you Sure?".into(), then: Confirmed::SetHome(ll.0, ll.1, alt) });
                            }
                        }
                    }
                }
                "Set EKF Origin Here" => {
                    if let Some(l) = need_link(app) {
                        use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, SET_GPS_GLOBAL_ORIGIN_DATA};
                        l.send(LinkCommand::Send(Box::new(MavMessage::SET_GPS_GLOBAL_ORIGIN(SET_GPS_GLOBAL_ORIGIN_DATA {
                            latitude: (ll.0 * 1e7) as i32,
                            longitude: (ll.1 * 1e7) as i32,
                            altitude: 0,
                            target_system: cs.sysid,
                            time_usec: 0,
                        }))));
                    }
                }
                "TakeOff" => {
                    if link.is_some() {
                        let alt = app.settings.planner.get("takeoff_alt").cloned().unwrap_or_else(|| "5".into());
                        self.prompt = Some(Prompt::Input { title: "Enter Alt".into(), fields: vec![("Enter Takeoff Alt".into(), alt)], then: Inputted::TakeOff });
                    }
                }
                "Jump To Tag" => {
                    if need_link(app).is_some() {
                        self.prompt = Some(Prompt::Input { title: "Jump To Tag".into(), fields: vec![("Tag".into(), "1".into())], then: Inputted::JumpTag });
                    }
                }
                "Gimbal Full Sized" => self.gimbal.set_placement(GimbalPlacement::FullSized, app, ui.ctx()),
                "Gimbal Mini" => self.gimbal.set_placement(GimbalPlacement::Mini, app, ui.ctx()),
                "Gimbal Pop Out" => self.gimbal.set_placement(GimbalPlacement::PopOut, app, ui.ctx()),
                _ => {}
            }
        }
        self.prompt_ui(ui.ctx(), app);
        self.log_downloader(ui.ctx(), app);
        if let Some((action, path)) = crate::platform::poll_file_action(&mut self.log_file_request, ui.ctx()) {
            self.dflog_path(&action, path, app);
        }
        self.log_browse.ui(ui.ctx());
        self.analysis_window(ui.ctx());
        self.raw_sensor_window(ui.ctx(), cs);
        self.mavlog.ui(ui.ctx());
        self.georef.ui(ui.ctx(), app);

        // ---- overlays positioned as in splitContainer1.Panel2
        let p = ui.painter().with_clip_rect(mrect);
        let o = mrect.min;
        let (w, h) = (mrect.width(), mrect.height());
        wind_dir(&p, Rect::from_min_size(o, Vec2::splat(60.0)), cs.wind_dir, cs.wind_vel);
        distance_bar(&p, Rect::from_min_size(o + Vec2::new(66.0, 0.0), Vec2::new((w - 131.0).max(10.0), 22.0)), cs, &mission);
        p.text(o + Vec2::new(w - 40.0, 3.0), egui::Align2::LEFT_TOP, crate::i18n::t("Zoom"), fonts::ui_id(fonts::UI_SIZE), Color32::BLACK);
        let zl = Rect::from_min_size(o + Vec2::new(w - 44.0, 22.0), Vec2::new(42.0, 20.0));
        ui.put(zl, egui::DragValue::new(&mut app.settings.fd_map.zoom).speed(0.1).range(2.0..=22.0).max_decimals(1));
        let trk = Rect::from_min_size(o + Vec2::new(w - 45.0, 40.0), Vec2::new(45.0, (h - 54.0).max(40.0)));
        theme::vtrackbar(ui, trk, &mut app.settings.fd_map.zoom, 2.0, 22.0);
        let small = fonts::ui_id(fonts::UI_SIZE);
        p.text(o + Vec2::new(-1.0, h - 48.0), egui::Align2::LEFT_TOP, format!("hdop: {:.1}", cs.gpshdop), small.clone(), Color32::WHITE);
        p.text(o + Vec2::new(-1.0, h - 29.0), egui::Align2::LEFT_TOP, format!("Sats: {}", cs.satcount), small.clone(), Color32::WHITE);
        let ly = h - 30.0;
        p.text(o + Vec2::new(45.0, ly), egui::Align2::LEFT_TOP, crate::i18n::t("Current Heading"), small.clone(), Color32::RED);
        p.text(o + Vec2::new(135.0, ly), egui::Align2::LEFT_TOP, crate::i18n::t("Direct to current WP"), small.clone(), Color32::from_rgb(255, 128, 0));
        p.text(o + Vec2::new(245.0, ly), egui::Align2::LEFT_TOP, crate::i18n::t("Target Heading"), small.clone(), Color32::from_rgb(0, 128, 0));
        let gl = p.layout_no_wrap(crate::i18n::t("GPS Track (Black)"), small, Color32::WHITE);
        let gr = Rect::from_min_size(o + Vec2::new(332.0, ly), gl.size());
        p.rect_filled(gr, 0.0, Color32::BLACK);
        p.galley(gr.min, gl, Color32::WHITE);
        if self.gimbal.placement == GimbalPlacement::Mini {
            let mini_vid_rect = super::gimbal_video::mini_video_rect(mrect, self.gimbal.aspect());
            let mut vid_ui = ui.new_child(egui::UiBuilder::new().max_rect(mini_vid_rect));
            self.gimbal.render_video_box(&mut vid_ui, mini_vid_rect, app);
            ui.painter().rect_stroke(mini_vid_rect, 0.0, Stroke::new(1.0, Color32::from_gray(80)), egui::StrokeKind::Inside);
        }
        self.gimbal.extra_ui(ui.ctx(), app);
    }
}

impl FlightDataView {
    /// CustomMessageBox (Yes/No) and InputBox prompts.
    fn prompt_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some(prompt) = self.prompt.as_mut() else { return };
        let mut result: Option<bool> = None;
        let title = match prompt {
            Prompt::Confirm { title, .. } | Prompt::Input { title, .. } | Prompt::AltInput { title, .. } => title.clone(),
        };
        egui::Modal::new(egui::Id::unique("fd_prompt")).show(ctx, |ui| {
            ui.set_min_width(if matches!(prompt, Prompt::AltInput { .. }) { 376.0 } else { 280.0 });
            ui.label(RichText::new(&title).strong().color(Color32::WHITE));
            ui.add_space(6.0);
            let confirm = matches!(prompt, Prompt::Confirm { .. });
            match prompt {
                Prompt::Confirm { text, .. } => {
                    theme::label(ui, text.as_str());
                }
                Prompt::Input { fields, .. } => {
                    for (l, v) in fields.iter_mut() {
                        theme::label(ui, l.as_str());
                        ui.text_edit_singleline(v);
                    }
                }
                Prompt::AltInput { prompt_text, alt, frame, .. } => {
                    theme::label(ui, prompt_text.as_str());
                    ui.horizontal(|ui| {
                        let frame_label = match *frame {
                            0 => "Absolute",
                            10 => "Terrain",
                            _ => "Relative",
                        };
                        egui::ComboBox::from_id_salt("guided_frame_sel")
                            .width(70.0)
                            .selected_text(frame_label)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(frame, 0, "Absolute");
                                ui.selectable_value(frame, 3, "Relative");
                                ui.selectable_value(frame, 10, "Terrain");
                            });
                        ui.add_sized(Vec2::new(296.0, 20.0), egui::TextEdit::singleline(alt));
                    });
                }
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let force = match prompt {
                    Prompt::Confirm { then: Confirmed::ForceArm(arm), .. } => Some(if *arm { "Force Arm" } else { "Force Disarm" }),
                    // MessageBoxButtons.OKCancel
                    Prompt::Confirm { then: Confirmed::SetHome(..), .. } => Some("OK"),
                    _ => None,
                };
                let (yes, no) = match force {
                    // CustomMessageBox YesText / NoText
                    Some(f) => (f, "Cancel"),
                    None if confirm => ("Yes", "No"),
                    None => ("OK", "Cancel"),
                };
                if theme::button(ui, yes).clicked() {
                    result = Some(true);
                }
                if theme::button(ui, no).clicked() {
                    result = Some(false);
                }
            });
        });
        let is_force_arm = self.prompt.as_ref().map(|p| matches!(p, Prompt::Confirm { then: Confirmed::ForceArm(_), .. })).unwrap_or(false);
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) { result = Some(false); }
        else if !is_force_arm && ctx.input(|i| i.key_pressed(egui::Key::Enter)) { result = Some(true); }
        let Some(ok) = result else { return };
        let Some(prompt) = self.prompt.take() else { return };
        if !ok {
            return;
        }
        let link = app.link;
        let num = |f: &[(String, String)], i: usize| f.get(i).and_then(|(_, v)| v.trim().parse::<f64>().ok());
        match prompt {
            Prompt::Confirm { then, .. } => {
                let Some(l) = link else {
                    app.toasts.error("Please Connect First");
                    return;
                };
                match then {
                    Confirmed::DoAction(i) => {
                        let name = ACTIONS[i].0;
                        let fw = l.shared.read().cs.firmware;
                        if name == "Toggle_Safety_Switch" {
                            // setMode(SET_MODE { custom_mode = motor outputs enabled }, SAFETY_ARMED)
                            let (sys, comp, custom) = {
                                let s = l.shared.read();
                                let motor = s.cs.sensors_enabled & 0x8000 != 0 && s.cs.sensors_present != 0;
                                (s.cs.sysid, s.cs.compid, u32::from(motor))
                            };
                            if sys != 0 {
                                use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, MavModeFlag, SET_MODE_DATA};
                                l.send(LinkCommand::CommandNoAck { command: 176, params: [128.0, custom as f32, 0.0, 0.0, 0.0, 0.0, 0.0] });
                                for _ in 0..2 {
                                    l.send(LinkCommand::Send(Box::new(MavMessage::SET_MODE(SET_MODE_DATA { custom_mode: custom, target_system: sys, base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED }))));
                                }
                                let _ = comp;
                            }
                        } else if let Some((cmd, p)) = action_command(name, fw) {
                            // failures report "Command Failed <MAV_CMD>" (DO_PARACHUTE by its own check)
                            let silent = matches!(name, "Terminate_Flight" | "Preflight_Reboot_Shutdown" | "Reboot_Mass_Storage" | "HighLatency_Enable" | "HighLatency_Disable" | "Engine_Start" | "Engine_Stop");
                            self.action_ack = Some((cmd, if silent { String::new() } else { mp_core::mavcmd::cmd_name(cmd) }));
                            l.send(LinkCommand::CommandLong { command: cmd, params: p });
                        } else {
                            // Enum.Parse found no MAV_CMD
                            app.toasts.message(COMMAND_FAILED, "Error");
                        }
                    }
                    Confirmed::SetMode(m) => l.send(LinkCommand::SetMode(m)),
                    Confirmed::SetHome(lat, lng, alt) => {
                        l.send(LinkCommand::CommandInt(mp_core::gimbal::CommandInt { command: 179, frame: 0, params: [0.0; 4], x: (lat * 1e7) as i32, y: (lng * 1e7) as i32, z: alt as f32 }));
                        // getHomePositionAsync
                        l.send(LinkCommand::CommandNoAck { command: 512, params: [242.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    }
                    Confirmed::Arm(arm) => {
                        self.arm_pending = Some((arm, l.shared.read().messages.len(), Instant::now()));
                        l.send(LinkCommand::Arm { arm, force: false });
                    }
                    Confirmed::ForceArm(arm) => {
                        self.force_pending = true;
                        l.send(LinkCommand::Arm { arm, force: true });
                    }
                    Confirmed::ResumeWarn => {
                        let last = if self.last_auto_wp < 0 { 1 } else { self.last_auto_wp };
                        self.prompt = Some(Prompt::Input { title: "Resume at".into(), fields: vec![("Resume mission at waypoint#".into(), last.to_string())], then: Inputted::ResumeAt });
                    }
                }
            }
            Prompt::AltInput { alt, frame, then, .. } => {
                app.settings.planner.insert("guided_alt".into(), alt.trim().to_string());
                app.settings.planner.insert("guided_alt_frame".into(), frame.to_string());
                let Ok(parsed) = alt.trim().parse::<i32>() else {
                    app.toasts.error("Bad Alt");
                    return;
                };
                let alt = parsed as f32 / crate::units::alt(1.0);
                if let Some(l) = link {
                    let target = l.shared.write().set_guided_altitude(alt, frame);
                    match then {
                        AltInputted::FlyToHere(lat, lng) if alt != 0.0 => {
                            l.send(LinkCommand::Guided { lat, lng, alt, frame });
                        }
                        AltInputted::FlyToHereAlt if l.shared.read().cs.mode.eq_ignore_ascii_case("Guided") => {
                            l.send(LinkCommand::Guided { lat: target.lat, lng: target.lng, alt, frame });
                        }
                        _ => {}
                    }
                }
            }
            Prompt::Input { fields, then, .. } => match then {
                Inputted::FlyToCoords => {
                    if let (Some(l), Some(lat), Some(lng), Some(alt)) = (link, num(&fields, 0), num(&fields, 1), num(&fields, 2)) {
                        l.send(LinkCommand::Guided { lat, lng, alt: alt as f32 / crate::units::alt(1.0), frame: active_guided(link).frame });
                    }
                }
                Inputted::PointCameraCoords => {
                    if let (Some(l), Some(lat), Some(lng), Some(alt)) = (link, num(&fields, 0), num(&fields, 1), num(&fields, 2)) {
                        l.send(LinkCommand::CommandLong { command: mavcmd::DO_SET_ROI, params: [0.0, 0.0, 0.0, 0.0, lat as f32, lng as f32, alt as f32] });
                    }
                }
                Inputted::ResumeAt => {
                    if let (Some(l), Some(Ok(wp))) = (link, fields.first().map(|(_, t)| t.trim().parse::<u16>())) {
                        // get all: the current mission from the vehicle, then trim and re-upload
                        l.send(LinkCommand::ReadMission { mission_type: 0 });
                        self.resume = Some(Resume::Download { wp });
                    }
                }
                Inputted::PointCameraHere(lat, lng) => {
                    let text = fields.first().map(|(_, t)| t.trim().to_string()).unwrap_or_default();
                    match text.parse::<f32>() {
                        Err(_) => app.toasts.error("Bad Alt"),
                        Ok(_) if lat == 0.0 || lng == 0.0 => app.toasts.error("Bad Lat/Long"),
                        Ok(alt) => {
                            if let Some(l) = link {
                                l.send(LinkCommand::CommandInt(mp_core::gimbal::CommandInt { command: mavcmd::DO_SET_ROI, frame: 3, params: [0.0; 4], x: (lat * 1e7) as i32, y: (lng * 1e7) as i32, z: alt / crate::units::alt(1.0) }));
                            }
                        }
                    }
                }
                Inputted::TakeOff => {
                    let text = fields.first().map(|(_, t)| t.trim().to_string()).unwrap_or_default();
                    // float.Parse: an unreadable altitude stops here
                    if let (Some(l), Ok(alt)) = (link, text.parse::<f32>()) {
                        app.settings.planner.insert("takeoff_alt".into(), alt.to_string());
                        let guided = l.shared.read().cs.firmware.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case("GUIDED")).map(|(k, _)| k);
                        if let Some(m) = guided {
                            l.send(LinkCommand::SetMode(m));
                        }
                        l.send(LinkCommand::CommandLong { command: mavcmd::TAKEOFF, params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt] });
                    }
                }
                Inputted::JumpTag => {
                    if let (Some(l), Some(Ok(tag))) = (link, fields.first().map(|(_, t)| t.trim().parse::<f32>())) {
                        // MAV_CMD_DO_JUMP_TAG
                        l.send(LinkCommand::CommandLong { command: 601, params: [tag, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    }
                }
                Inputted::SendMessage => {
                    if let (Some(l), Some((_, text))) = (link, fields.first()) {
                        use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, MavSeverity, STATUSTEXT_DATA};
                        l.send(LinkCommand::Send(Box::new(MavMessage::STATUSTEXT(STATUSTEXT_DATA { severity: MavSeverity::MAV_SEVERITY_NOTICE, text: text.as_str().into(), id: 0, chunk_seq: 0 }))));
                    }
                }
                Inputted::AddPoi(lat, lng) => {
                    let name = fields.first().map(|(_, v)| v.clone()).unwrap_or_default();
                    app.settings.pois.push((lat, lng, name));
                }
            },
        }
    }
}

impl FlightDataView {
    /// tabTLogs: Load Log / Play/Pause / Tlog > Kml, LBL_logfn, tracklog, speed buttons.
    fn tlogs_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.available_rect_before_wrap().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let b = |ui: &mut egui::Ui, rr: Rect, t: &str| ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, t, rr.size())).clicked();
        let f = fonts::ui_id(fonts::UI_SIZE);
        let pb = app.link.filter(|l| l.is_playback());
        let info = pb.and_then(|l| l.shared.read().playback);
        if b(ui, r(3.0, 3.0, 80.0, 27.0), "Load Log") {
            let picked = if crate::platform::HAS_NATIVE_DIALOGS {
                rfd_pick_tlog()
            } else {
                crate::platform::list_files_in(&crate::platform::data_dir().join("logs"), &["tlog"]).last().cloned()
            };
            if let Some(path) = picked {
                self.tlog_name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                app.actions.push(crate::app::AppAction::PlayTlog(path));
            }
        }
        if b(ui, r(3.0, 36.0, 80.0, 27.0), "Play/Pause") {
            if let (Some(l), Some(i)) = (pb, info) {
                l.send(LinkCommand::Playback(mp_core::tlog::PlaybackCommand::Play(!i.playing)));
            }
        }
        if b(ui, r(3.0, 69.0, 80.0, 46.0), "Tlog > Kml or Graph") {
            self.mavlog.open = true;
        }
        ui.painter().with_clip_rect(r(94.0, 0.0, 193.0, 33.0)).text(o + Vec2::new(94.0, 10.0), egui::Align2::LEFT_TOP, &self.tlog_name, f.clone(), Color32::WHITE);
        // tracklog: seek bar
        let tr = r(94.0, 36.0, 157.0, 27.0);
        let mut pos = info.map(|i| i.position).unwrap_or(0.0);
        let resp = ui.put(tr, egui::Slider::new(&mut pos, 0.0..=1.0).show_value(false));
        if resp.drag_stopped() || resp.clicked() {
            if let Some(l) = pb {
                l.send(LinkCommand::Playback(mp_core::tlog::PlaybackCommand::Seek(pos)));
            }
        }
        for (x, y, w, t, sp) in [
            (6.0, 23.0, 23.0, "0.1", 0.1f32),
            (35.0, 23.0, 27.0, "0.25", 0.25),
            (68.0, 23.0, 23.0, "0.5", 0.5),
            (97.0, 23.0, 23.0, "1x", 1.0),
            (6.0, 59.0, 23.0, "2x", 2.0),
            (35.0, 59.0, 23.0, "5x", 5.0),
            (64.0, 59.0, 23.0, "10x", 10.0),
        ] {
            if b(ui, r(94.0 + x, 69.0 + y - 23.0, w, 30.0), t) {
                if let Some(l) = pb {
                    l.send(LinkCommand::Playback(mp_core::tlog::PlaybackCommand::Speed(sp)));
                }
            }
        }
        let p = ui.painter();
        p.text(o + Vec2::new(257.0, 40.0), egui::Align2::LEFT_TOP, format!("{:.2} %", pos * 100.0), f.clone(), Color32::WHITE);
        p.text(o + Vec2::new(257.0, 76.0), egui::Align2::LEFT_TOP, format!("x {:.1}", info.map(|i| i.speed).unwrap_or(1.0)), f, Color32::WHITE);
        ui.allocate_rect(r(0.0, 0.0, 287.0, 181.0), Sense::hover());
    }

    /// tablogbrowse: DataFlash log tools; the first button opens the Log Downloader.
    fn dflogs_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.available_rect_before_wrap().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let b = |ui: &mut egui::Ui, rr: Rect, t: &str| ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, t, rr.size())).clicked();
        if b(ui, r(3.0, 3.0, 110.0, 38.0), "Download DataFlash Log Via Mavlink") {
            match app.link.filter(|l| l.status() == LinkStatus::Connected && !l.is_playback()) {
                Some(l) => {
                    l.send(LinkCommand::LogList);
                    self.logdl_open = true;
                    self.logdl_output.push("Getting list of logs".into());
                }
                None => app.toasts.error("Please Connect First"),
            }
        }
        for (x, y, w, h, t) in [
            (119.0, 3.0, 87.0, 38.0, "Review a Log"),
            (235.0, 3.0, 87.0, 38.0, "Auto Analysis"),
            (3.0, 47.0, 110.0, 38.0, "Create KML + gpx"),
            (119.0, 47.0, 87.0, 38.0, "Convert .Bin to .Log"),
            (235.0, 47.0, 88.0, 38.0, "Create Matlab File"),
            (3.0, 91.0, 110.0, 40.0, "Geo Reference Images"),
        ] {
            if b(ui, r(x, y, w, h), t) {
                self.dflog_action(t, app);
            }
        }
        ui.allocate_rect(r(0.0, 0.0, 330.0, 135.0), Sense::hover());
    }

    /// DataFlash log tools on a picked .bin.
    fn dflog_action(&mut self, what: &str, _app: &mut AppCtx) {
        if what == "Geo Reference Images" {
            self.georef.open = true;
            return;
        }
        if what == "Review a Log" { self.log_browse.request_load(); return; }
        self.log_file_request = Some((what.into(), crate::platform::binary_log_picker()));
    }

    fn dflog_path(&mut self, what: &str, path: std::path::PathBuf, app: &mut AppCtx) {
        if what == "Convert .Bin to .Log" {
            // but_bintolog_Click: <dir>\<name>.log next to each log, through BinaryLog.ConvertBin;
            // nothing is shown on success
            let dir = path.parent().map(|d| d.to_string_lossy().to_string()).unwrap_or_default();
            let stem = path.file_stem().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
            let outfilename = format!("{dir}{}{stem}.log", std::path::MAIN_SEPARATOR);
            if let Err(e) = mp_core::extlibs::utilities::binarylog::BinaryLog::ConvertBin(&path.to_string_lossy(), &outfilename, None) {
                app.toasts.error(e.to_string());
            }
            return;
        }
        let data = match std::fs::read(&path) {
            Ok(d) => d,
            Err(e) => {
                app.toasts.error(e.to_string());
                return;
            }
        };
        let log = mp_core::dataflash::parse(&data);
        let stem = path.with_extension("");
        let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let res = match what {
            "Create KML + gpx" => {
                let (kml, gpx) = log.to_kml_gpx(&name);
                std::fs::write(stem.with_extension("kml"), kml).and_then(|_| std::fs::write(stem.with_extension("gpx"), gpx)).map(|_| format!("{}.kml / .gpx", stem.display()))
            }
            "Create Matlab File" => std::fs::write(stem.with_extension("mat"), log.to_mat_v4()).map(|_| format!("{}.mat", stem.display())),
            "Auto Analysis" => {
                self.analysis = Some((name.clone(), log.analysis()));
                return;
            }
            _ => return,
        };
        match res {
            Ok(p) => app.toasts.error(format!("Saved {p}")),
            Err(e) => app.toasts.error(e.to_string()),
        }
    }

    /// RAW_IMU / RC view (BUT_RAWSensor -> RAW_Sensor form).
    fn raw_sensor_window(&mut self, ctx: &egui::Context, cs: &CurrentState) {
        if !self.raw_sensor {
            return;
        }
        let mut open = true;
        egui::Window::new(crate::i18n::t("Raw Sensor View")).open(&mut open).collapsible(false).show(ctx, |ui| {
            egui::Grid::new("raw").num_columns(4).spacing(Vec2::new(18.0, 4.0)).show(ui, |ui| {
                for (a, b, c, n) in [("ax", "ay", "az", "Accel"), ("gx", "gy", "gz", "Gyro"), ("mx", "my", "mz", "Mag")] {
                    theme::label(ui, n);
                    for k in [a, b, c] {
                        theme::label(ui, format!("{k}: {:.0}", cs.field(k).unwrap_or(0.0)));
                    }
                    ui.end_row();
                }
            });
            ui.separator();
            egui::Grid::new("rawrc").num_columns(2).show(ui, |ui| {
                for i in 0..8 {
                    theme::label(ui, format!("ch{}in {}", i + 1, cs.ch_in[i]));
                    theme::label(ui, format!("ch{}out {}", i + 1, cs.ch_out[i]));
                    ui.end_row();
                }
            });
        });
        if !open {
            self.raw_sensor = false;
        }
    }

    /// LogAnalyzer results window.
    fn analysis_window(&mut self, ctx: &egui::Context) {
        let Some((name, rows)) = self.analysis.clone() else { return };
        let mut open = true;
        egui::Window::new(format!("Log Analysis - {name}")).open(&mut open).collapsible(false).show(ctx, |ui| {
            egui::Grid::new("analysis").num_columns(3).striped(true).show(ui, |ui| {
                for h in ["Test", "Status", "Message"] {
                    ui.label(RichText::new(h).strong().color(Color32::WHITE));
                }
                ui.end_row();
                for (t, st, m) in rows {
                    theme::label(ui, t);
                    let col = match st.as_str() {
                        "GOOD" => Color32::from_rgb(0, 200, 0),
                        "WARN" => Color32::from_rgb(255, 165, 0),
                        "FAIL" => Color32::RED,
                        _ => Color32::WHITE,
                    };
                    ui.label(RichText::new(st).color(col).strong());
                    theme::label(ui, m);
                    ui.end_row();
                }
            });
        });
        if !open {
            self.analysis = None;
        }
    }

    /// Log/LogDownloadMavLink "Log Downloader" form.
    fn log_downloader(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if !self.logdl_open {
            return;
        }
        let Some(link) = app.link.filter(|l| !l.is_playback()) else {
            self.logdl_open = false;
            return;
        };
        let (entries, prog) = {
            let s = link.shared.read();
            (s.log_entries.clone(), s.progress.clone())
        };
        // start the next queued download
        if !self.logdl_busy {
            if let Some((id, size)) = self.logdl_queue.pop() {
                let dir = crate::platform::data_dir().join("logs");
                let name = format!("{} {}.bin", chrono_like(entries.iter().find(|e| e.0 == id).map(|e| e.2).unwrap_or(0)), id);
                self.logdl_output.push(format!("Downloading log {id} ({size} bytes)"));
                link.send(LinkCommand::LogDownload { id, size, path: dir.join(name) });
                self.logdl_busy = true;
            }
        }
        let mut open = true;
        egui::Window::new(crate::i18n::t("Log Downloader")).open(&mut open).collapsible(false).default_size(Vec2::new(784.0, 412.0)).show(ctx, |ui| {
            ui.horizontal_top(|ui| {
                ui.vertical(|ui| {
                    ui.set_width(321.0);
                    theme::label(ui, "Log files:");
                    egui::ScrollArea::vertical().id_salt("logs").max_height(244.0).show(ui, |ui| {
                        for (id, size, t) in &entries {
                            let mut on = self.logdl_selected.contains(id);
                            if ui.checkbox(&mut on, format!("{id} {} {:.0} KB", chrono_like(*t), *size as f32 / 1024.0)).changed() {
                                if on {
                                    self.logdl_selected.insert(*id);
                                } else {
                                    self.logdl_selected.remove(id);
                                }
                            }
                        }
                    });
                    ui.add_space(8.0);
                    egui::Grid::new("logbtn").num_columns(2).spacing(Vec2::new(24.0, 6.0)).show(ui, |ui| {
                        if theme::button_sized(ui, "Download All Logs", Vec2::new(136.0, 23.0)).clicked() {
                            self.logdl_queue = entries.iter().rev().map(|e| (e.0, e.1)).collect();
                        }
                        if theme::button_sized(ui, "First Person KML", Vec2::new(136.0, 23.0)).clicked() {
                            self.dflog_action("Create KML + gpx", app);
                        }
                        ui.end_row();
                        if theme::button_sized(ui, "Download Selected Logs", Vec2::new(136.0, 23.0)).clicked() {
                            self.logdl_queue = entries.iter().rev().filter(|e| self.logdl_selected.contains(&e.0)).map(|e| (e.0, e.1)).collect();
                        }
                        if theme::button_sized(ui, "Create KML", Vec2::new(136.0, 23.0)).clicked() {
                            self.dflog_action("Create KML + gpx", app);
                        }
                        ui.end_row();
                        if theme::button_sized(ui, "Clear Logs", Vec2::new(136.0, 23.0)).clicked() {
                            link.send(LinkCommand::LogErase);
                            link.send(LinkCommand::LogList);
                            self.logdl_output.push("Erasing logs".into());
                        }
                        if theme::button_sized(ui, ".bin to .log", Vec2::new(136.0, 23.0)).clicked() {
                            self.dflog_action("Convert .Bin to .Log", app);
                        }
                        ui.end_row();
                    });
                    ui.add_space(8.0);
                    let (done, total) = prog.as_ref().map(|p| (p.done, p.total)).unwrap_or((0, 0));
                    ui.horizontal(|ui| {
                        theme::progress(ui, if total > 0 { done as f32 / total as f32 } else { 0.0 }, Vec2::new(276.0, 19.0), "");
                        theme::label(ui, done.to_string());
                    });
                });
                ui.vertical(|ui| {
                    theme::label(ui, "Output:");
                    let (rr, _) = ui.allocate_exact_size(Vec2::new(451.0, 356.0), Sense::hover());
                    ui.painter().rect_filled(rr, 0.0, theme::control_bg());
                    ui.scope_builder(egui::UiBuilder::new().max_rect(rr.shrink(3.0)), |ui| {
                        egui::ScrollArea::vertical().id_salt("logout").stick_to_bottom(true).show(ui, |ui| {
                            for l in &self.logdl_output {
                                theme::label(ui, l.as_str());
                            }
                        });
                    });
                    theme::label(ui, "NOTE: When posting support querys, please send the .bin file");
                });
            });
        });
        if !open {
            self.logdl_open = false;
        }
    }

    pub fn on_log_event(&mut self, e: &LinkEvent) {
        match e {
            LinkEvent::LogList(n) => self.logdl_output.push(format!("Got {n} logs")),
            LinkEvent::LogDownloaded { id, path, result } => {
                self.logdl_busy = false;
                match result {
                    Ok(()) => self.logdl_output.push(format!("Log {id} saved to {}", path.display())),
                    Err(e) => self.logdl_output.push(format!("Log {id} failed: {e}")),
                }
            }
            _ => {}
        }
    }
}

fn chrono_like(utc: u32) -> String {
    if utc == 0 {
        return "1-1-1980 0-00-00".into();
    }
    let days = utc as i64 / 86400;
    let secs = utc as i64 % 86400;
    // civil-from-days (Howard Hinnant)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02} {:02}-{:02}-{:02}", secs / 3600, (secs / 60) % 60, secs % 60)
}

fn rfd_pick_tlog() -> Option<std::path::PathBuf> {
    let logs_dir = crate::platform::data_dir().join("logs");
    crate::platform::pick_file_in("Telemetry Log", "Telemetry Log", &["tlog"], Some(&logs_dir))
}

// ------------------------------------------------------------------ QuickView

/// CurrentState.GetNameandUnit: the (L10NU-translated) DisplayText with its unit placeholder
/// filled in, or the name itself.
pub(crate) fn name_and_unit(name: &str) -> String {
    use crate::units::{alt_unit, dist_unit, speed_unit};
    let desc = match mp_core::state::display_text(name) {
        Some(text) => crate::i18n::l10nu(text).unwrap_or(text).to_string(),
        None => name.to_string(),
    };
    if desc.contains("(dist)") {
        desc.replace("(dist)", &format!("({})", dist_unit()))
    } else if desc.contains("(speed)") {
        desc.replace("(speed)", &format!("({})", speed_unit()))
    } else if desc.contains("(alt)") {
        desc.replace("(alt)", &format!("({})", alt_unit()))
    } else {
        desc
    }
}

/// CurrentState.GetFieldDesc: the translated DisplayFieldName with (fieldName), or the name.
pub(crate) fn field_desc(name: &str) -> String {
    mp_core::state::field_key(name).and_then(crate::i18n::l10nu).map(|t| t.replace("(fieldName)", name)).unwrap_or_else(|| name.to_string())
}

/// The designer's quickView1..6 desc before Settings "quickView<n>" exists.
const QUICK_DESIGNER: [&str; 6] = ["alt", "groundspeed", "wp_dist", "yaw", "verticalspeed", "DistToMAV"];

/// QV.desc: GetNameandUnit of the chosen field, else of the designer's predefined desc.
fn quick_label(planner: &std::collections::BTreeMap<String, String>, index: usize, name: &str) -> String {
    match planner.get(&format!("quickView{}", index + 1)) {
        Some(chosen) => name_and_unit(chosen),
        None => name_and_unit(QUICK_DESIGNER.get(index).copied().unwrap_or(name)),
    }
}

// ------------------------------------------------------------------ map overlays

/// `Controls/WindDir`: circle with the wind arrow and speed.
fn wind_dir(p: &egui::Painter, r: Rect, dir: f32, speed: f32) {
    let c = r.center();
    let rad = r.width() / 2.0 - 4.0;
    p.circle_stroke(c, rad, Stroke::new(1.0, Color32::BLACK));
    let a = (dir + 180.0 - 90.0).to_radians();
    let d = Vec2::new(a.cos(), a.sin()) * rad;
    p.arrow(c - d, d * 2.0, Stroke::new(2.0, Color32::BLACK));
    p.text(r.center_bottom() - Vec2::new(0.0, 2.0), egui::Align2::CENTER_BOTTOM, format!("{speed:.1}"), fonts::ui_id(9.0), Color32::BLACK);
}

/// `Controls/DistanceBar`: route waypoints with the vehicle's progress marker.
fn distance_bar(p: &egui::Painter, r: Rect, cs: &CurrentState, mission: &[Locationwp]) {
    let n = mission.iter().skip(1).filter(|w| w.has_location() && (w.lat != 0.0 || w.lng != 0.0)).count();
    if n < 2 {
        return;
    }
    let bar = Rect::from_center_size(r.center(), Vec2::new(r.width(), 8.0));
    p.rect_filled(bar, 0.0, Color32::from_black_alpha(120));
    p.rect_stroke(bar, 0.0, Stroke::new(1.0, Color32::WHITE), StrokeKind::Inside);
    for i in 0..n {
        let x = bar.left() + bar.width() * i as f32 / (n - 1) as f32;
        p.line_segment([Pos2::new(x, bar.top() - 2.0), Pos2::new(x, bar.bottom() + 2.0)], Stroke::new(1.0, Color32::WHITE));
    }
    let cur = (cs.wpno as usize).saturating_sub(1).min(n - 1);
    let x = bar.left() + bar.width() * cur as f32 / (n - 1) as f32;
    p.circle_filled(Pos2::new(x, bar.center().y), 4.0, Color32::RED);
}

// ------------------------------------------------------------------ other tabs

fn messages_ui(ui: &mut egui::Ui, app: &AppCtx) {
    // txt_messagebox: RichTextBox on ControlBGColor
    let r = ui.available_rect_before_wrap();
    ui.painter().rect_filled(r, 0.0, theme::control_bg());
    if let Some(l) = app.link {
        let s = l.shared.read();
        for m in s.messages.iter().rev() {
            theme::label(ui, format!("{} : {}", m.time, m.text));
        }
    }
}

/// One AGauge needle: value, radius (design px), colour.
struct Needle(f32, f32, Color32);

/// AGauge (150x150 design space, centre 75,75) drawn over its face image.
#[allow(clippy::too_many_arguments)]
fn agauge(p: &egui::Painter, r: Rect, face: egui::TextureId, cap: &str, cap_pos: (f32, f32), min: f32, max: f32, arc_start: f32, arc_sweep: f32, major_step: f32, arc_radius: f32, needles: &[Needle]) {
    let s = r.width() / 150.0;
    p.image(face, r, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
    let c = r.min + Vec2::splat(75.0 * s);
    let ang = |v: f32| (arc_start + arc_sweep * ((v - min) / (max - min))).to_radians();
    let dir = |a: f32| Vec2::new(a.cos(), a.sin());
    let majors = ((max - min) / major_step).round() as i32;
    let white = Color32::WHITE;
    let font = fonts::ui_id((8.25 * s * 1.33).max(7.0));
    for i in 0..=majors {
        let v = min + i as f32 * major_step;
        let a = ang(v);
        if arc_sweep >= 360.0 && i == majors {
            break;
        }
        p.line_segment([c + dir(a) * 50.0 * s, c + dir(a) * 60.0 * s.max(0.1)], Stroke::new(2.0 * s.max(0.6), white));
        if i < majors {
            for k in 1..=9 {
                let am = ang(v + major_step * k as f32 / 10.0);
                p.line_segment([c + dir(am) * 55.0 * s, c + dir(am) * 60.0 * s], Stroke::new(1.0, white));
            }
        }
        let label = if (v - v.round()).abs() < 1e-3 { format!("{v:.0}") } else { format!("{v:.1}") };
        p.text(c + dir(a) * 42.0 * s, egui::Align2::CENTER_CENTER, label, font.clone(), white);
    }
    let _ = arc_radius;
    p.text(r.min + Vec2::new(cap_pos.0, cap_pos.1) * s, egui::Align2::LEFT_TOP, cap, font.clone(), white);
    for Needle(v, rad, col) in needles {
        let a = ang(v.clamp(min, max));
        p.line_segment([c - dir(a) * 8.0 * s, c + dir(a) * *rad * s], Stroke::new(2.5 * s.max(0.6), *col));
    }
    p.circle_filled(c, 5.0 * s, Color32::from_gray(90));
    p.circle_stroke(c, 5.0 * s, Stroke::new(1.0, Color32::from_gray(200)));
}

/// Controls/HSI: heading rose that turns under a fixed aircraft, with the nav bearing bug.
fn hsi(p: &egui::Painter, r: Rect, heading: f32, nav: f32) {
    let w = r.width();
    let c = r.center();
    let rin = w / 3.6;
    let rout = w / 2.2;
    let font = fonts::ui_id((w / 14.0) * 1.33);
    let mut a = 0;
    while a < 360 {
        let ang = (a as f32 - heading - 90.0).to_radians();
        let d = Vec2::new(ang.cos(), ang.sin());
        let len = if a % 30 == 0 { 11.0 } else if a % 10 == 0 { 7.0 } else { 4.0 };
        p.line_segment([c + d * rin, c + d * (rin + len)], Stroke::new(1.0, Color32::WHITE));
        if a % 30 == 0 {
            let t = match a {
                0 => "N".to_string(),
                90 => "E".to_string(),
                180 => "S".to_string(),
                270 => "W".to_string(),
                _ => (a / 10).to_string(),
            };
            let g = p.layout_no_wrap(t, font.clone(), Color32::WHITE);
            let pos = c + d * (rout - g.size().y * 0.4);
            p.add(egui::epaint::TextShape::new(pos - Vec2::new(g.size().x / 2.0, 0.0), g, Color32::WHITE).with_angle(ang + std::f32::consts::FRAC_PI_2));
        }
        a += 5;
    }
    let or = Stroke::new(2.0, Color32::from_rgb(0xff, 0x8c, 0x00));
    let k = w / 100.0;
    p.line_segment([c + Vec2::new(0.0, 30.0 * k), c + Vec2::new(0.0, -10.0 * k)], or);
    p.line_segment([c + Vec2::new(-30.0 * k, 0.0), c + Vec2::new(30.0 * k, 0.0)], or);
    p.line_segment([c + Vec2::new(-10.0 * k, 25.0 * k), c + Vec2::new(10.0 * k, 25.0 * k)], or);
    p.line_segment([c + Vec2::new(0.0, -rout), c + Vec2::new(0.0, -rin)], Stroke::new(2.0, Color32::WHITE));
    let bug = (nav - heading).to_radians();
    let rot = |x: f32, y: f32| c + Vec2::new(x * bug.cos() - y * bug.sin(), x * bug.sin() + y * bug.cos());
    let pts = [(-5.0, -rout), (-5.0, -rout + 4.0), (-3.0, -rout + 4.0), (0.0, -rout + 8.0), (3.0, -rout + 4.0), (5.0, -rout + 4.0), (5.0, -rout)];
    p.add(egui::Shape::line(pts.iter().map(|(x, y)| rot(*x, *y)).collect(), or));
}

/// tabGauges: VSI, Speed, Alt, Heading — 2x2 when square, one row when wide
/// (FlightData.tabPage1_Resize).
fn gauges_ui(ui: &mut egui::Ui, cs: &CurrentState, assets: &crate::assets::Assets) {
    let avail = ui.available_size();
    let o = ui.available_rect_before_wrap().min;
    let scale = avail.x / avail.y.max(1.0);
    let (size, cells): (f32, [(f32, f32); 4]) = if scale > 0.5 && scale < 1.9 {
        let s = (avail.x.min(avail.y) / 2.0).floor();
        (s, [(0.0, 0.0), (s, 0.0), (0.0, s), (s, s)])
    } else {
        let s = (avail.x / 4.0).floor();
        (s, [(0.0, 0.0), (s, 0.0), (2.0 * s, 0.0), (3.0 * s, 0.0)])
    };
    let p = ui.painter().clone();
    let rect = |i: usize| Rect::from_min_size(o + Vec2::new(cells[i].0, cells[i].1), Vec2::splat(size));
    agauge(&p, rect(0), assets.gauge_gvspeed.id(), "VSI", (65.0, 85.0), -10.0, 10.0, 20.0, 320.0, 2.0, 60.0, &[Needle(cs.verticalspeed, 50.0, Color32::WHITE)]);
    agauge(&p, rect(1), assets.gauge_gspeed.id(), "Speed", (58.0, 85.0), 0.0, 60.0, 135.0, 270.0, 10.0, 70.0, &[Needle(cs.airspeed, 50.0, Color32::WHITE), Needle(cs.groundspeed, 50.0, Color32::from_rgb(0xa5, 0x2a, 0x2a))]);
    let target = cs.alt - cs.alt_error;
    agauge(&p, rect(2), assets.gauge_galt.id(), "Alt", (68.0, 85.0), 0.0, 9.99, 270.0, 360.0, 1.0, 60.0, &[Needle((cs.alt / 100.0).rem_euclid(10.0), 50.0, Color32::WHITE), Needle((cs.alt / 1000.0).rem_euclid(10.0), 30.0, Color32::WHITE), Needle((target / 100.0).rem_euclid(10.0), 50.0, Color32::RED)]);
    hsi(&p, rect(3), cs.yaw, cs.nav_bearing);
    ui.allocate_rect(Rect::from_min_size(o, avail), Sense::hover());
}

fn active_guided(link: Option<&mp_core::link::Link>) -> Locationwp {
    link.map(|l| l.shared.read().guided_target()).unwrap_or_else(|| Locationwp::waypoint(0.0, 0.0, 0.0, 3))
}

fn guided_prompt_values(firmware: Firmware, settings: &std::collections::BTreeMap<String, String>) -> (String, u8) {
    let default_alt = if firmware == Firmware::ArduCopter2 { 10.0 } else { 100.0 };
    (settings.get("guided_alt").cloned().unwrap_or_else(|| format!("{:.0}", crate::units::alt(default_alt))),
     settings.get("guided_alt_frame").and_then(|s| s.parse().ok()).unwrap_or(3))
}

fn status_ui(ui: &mut egui::Ui, cs: &CurrentState) -> egui::scroll_area::ScrollAreaOutput<()> {
    // tabStatus_Paint: 120 px name, 50 px value, 15 px row height, 190 px column step, horizontal scroll
    let f = fonts::ui_id(fonts::UI_SIZE);
    ui.style_mut().spacing.scroll = egui::style::ScrollStyle::solid();
    let fields = cs.get_item_list(true, false);
    let height = ui.available_height();
    let col_h = (height - 30.0).max(10.0);
    let rows_per_col = ((col_h - 10.0) / 15.0).floor() as usize + 1;
    let num_cols = (fields.len() + rows_per_col - 1) / rows_per_col;
    let total_w = 10.0 + (num_cols as f32) * 190.0 + 30.0;

    let mut scroll = egui::ScrollArea::horizontal().auto_shrink([false, false]);
    // Native capture of the final column, paired with the actual scrollbar-drag test.
    if std::env::var_os("MP_STATUS_END").is_some() { scroll = scroll.horizontal_scroll_offset((total_w - ui.available_width()).max(0.0)); }
    scroll.show(ui, |ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(total_w, col_h + 10.0), Sense::hover());
        let p = ui.painter().clone();
        let o = rect.min;
        let mut x = 10.0;
        let mut y = 10.0;

        for n in fields {
            p.with_clip_rect(Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(120.0, 15.0)).intersect(ui.clip_rect()))
                .text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, n, f.clone(), Color32::WHITE);
            let val_str = match cs.property(n) {
                serde_json::Value::Null => String::new(),
                serde_json::Value::Bool(b) => if b { "True" } else { "False" }.into(),
                serde_json::Value::String(s) => s,
                serde_json::Value::Number(v) => fmt_val(crate::units::field(n, v.as_f64().unwrap_or(0.0)).0),
                v => v.to_string(),
            };
            p.with_clip_rect(Rect::from_min_size(o + Vec2::new(x + 120.0, y), Vec2::new(50.0, 15.0)).intersect(ui.clip_rect()))
                .text(o + Vec2::new(x + 120.0, y), egui::Align2::LEFT_TOP, val_str, f.clone(), Color32::WHITE);

            y += 15.0;
            if y > col_h {
                x += 190.0;
                y = 10.0;
            }
        }
    })
}

fn servo_ui(ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
    let link = app.link.filter(|l| l.status() == LinkStatus::Connected);
    for ch in 5u16..=14 {
        ui.horizontal(|ui| {
            ui.add_sized(Vec2::new(54.0, 18.0), egui::Label::new(RichText::new(format!("Servo {ch}")).color(Color32::WHITE)));
            for (t, pwm) in [("Low", 1100.0), ("Mid", 1500.0), ("High", 1900.0)] {
                if theme::button_sized(ui, t, Vec2::new(40.0, 20.0)).clicked() {
                    if let Some(l) = link {
                        l.send(LinkCommand::CommandLong { command: 183, params: [ch as f32, pwm, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    }
                }
            }
            let cur = cs.ch_out.get(ch as usize - 1).copied().unwrap_or(0);
            if theme::button_sized(ui, "Toggle", Vec2::new(48.0, 20.0)).clicked() {
                if let Some(l) = link {
                    let to = if cur > 1500 { 1100.0 } else { 1900.0 };
                    l.send(LinkCommand::CommandLong { command: 183, params: [ch as f32, to, 0.0, 0.0, 0.0, 0.0, 0.0] });
                }
            }
            theme::label(ui, cur.to_string());
        });
    }
    for relay in 0..6 {
        ui.horizontal(|ui| {
            ui.add_sized(Vec2::new(54.0, 18.0), egui::Label::new(RichText::new(format!("Relay {relay}")).color(Color32::WHITE)));
            for (t, v) in [("Low", 0.0), ("High", 1.0)] {
                if theme::button_sized(ui, t, Vec2::new(40.0, 20.0)).clicked() {
                    if let Some(l) = link {
                        l.send(LinkCommand::CommandLong { command: 181, params: [relay as f32, v, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    }
                }
            }
        });
    }
}

enum HudPrompt {
    Mjpeg(String),
    GStreamer(String),
    HereLink(String),
    Cells(String),
    Ground([u8; 3], [u8; 3]),
    UserItems,
}

fn fmt_val(v: f64) -> String {
    if v.fract() == 0.0 {
        format!("{v:.0}")
    } else {
        format!("{v:.3}")
    }
}

pub fn provider_combo(ui: &mut egui::Ui, p: &mut map::Provider, id: &str) {
    egui::ComboBox::from_id_salt(id).selected_text(p.name()).width(103.0).wrap_mode(egui::TextWrapMode::Truncate).show_ui(ui, |ui| {
        for v in map::Provider::ALL {
            ui.selectable_value(p, v, v.name());
        }
    });
}

pub fn pwm_bar(ui: &mut egui::Ui, label: &str, pwm: u16) {
    ui.horizontal(|ui| {
        theme::label(ui, format!("{label:>2}"));
        let frac = ((pwm as f32 - 800.0) / 1400.0).clamp(0.0, 1.0);
        theme::progress(ui, frac, Vec2::new(ui.available_width().min(260.0), 18.0), &pwm.to_string());
    });
}

#[cfg(test)]
mod guided_tests {
    use super::*;
    #[test]
    fn dragging_status_scrollbar_reaches_the_last_column_inside_small_panels() {
        for size in [Vec2::new(300.0, 400.0), Vec2::new(260.0, 220.0)] {
            let ctx = egui::Context::default();
            let cs = CurrentState::default();
            let mut state = None;
            let mut time = 0.0;
            let mut frame = |events| {
                time += 0.2;
                let mut out = ctx.run_ui(egui::RawInput { time: Some(time), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, size)), events, ..Default::default() }, |ui| { state = Some(status_ui(ui, &cs)); });
                out.textures_delta.clear();
                let output = state.take().unwrap();
                (output.state.offset().x, output.inner_rect, output.content_size.x)
            };
            frame(vec![]);
            let (_, viewport, content) = frame(vec![]);
            let start = Pos2::new(viewport.left() + 5.0, viewport.bottom() + 6.0);
            assert!(start.y < size.y, "Scrollbar is outside the panel: {start:?}");
            frame(vec![egui::Event::PointerMoved(start), egui::Event::PointerButton { pos: start, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() }]);
            let end = Pos2::new(viewport.right() - 2.0, start.y);
            frame(vec![egui::Event::PointerMoved(end)]);
            let (offset, _, _) = frame(vec![egui::Event::PointerButton { pos: end, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() }]);
            assert!(offset >= content - viewport.width() - 20.0, "Could not reach last column: offset={offset}, width={content}, viewport={viewport:?}");
        }
    }
    #[test]
    fn status_columns_scroll_horizontally_with_the_mouse() {
        let ctx = egui::Context::default();
        let cs = CurrentState::default();
        let mut offset = 0.0;
        for i in 0..8 {
            let mut events = vec![egui::Event::PointerMoved(Pos2::new(100.0, 100.0))];
            if i == 1 { events.push(egui::Event::MouseWheel { unit: egui::MouseWheelUnit::Point, delta: Vec2::new(-500.0, 0.0), modifiers: egui::Modifiers::NONE, phase: egui::TouchPhase::Move, source: egui::MouseWheelSource::Unknown }); }
            let mut out = ctx.run_ui(egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(300.0, 400.0))), events, ..Default::default() }, |ui| { offset = status_ui(ui, &cs).state.offset().x; });
            out.textures_delta.clear();
        }
        assert!(offset > 100.0, "Status did not scroll: {offset}");
    }
    #[test]
    fn altitude_dialog_keeps_frame_rejects_bad_altitude_and_cancels_without_writes() {
        let ctx = egui::Context::default();
        let assets = crate::assets::Assets::load(&ctx);
        let mut tiles = crate::map::TileCache::offline();
        let mut toasts = crate::app::Toasts::default();
        let mut settings = crate::app::Settings::default();
        let mut joystick = crate::joystick::Joystick::default();
        let mut page = FlightDataView::default();
        let port = std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
        let link = mp_core::link::Link::connect_with_params(mp_core::comms::ConnectionSpec::UdpListen { port }, None, false);
        { let mut st = link.shared.write(); st.cs.sysid = 1; st.cs.compid = 1; }
        assert_eq!(active_guided(Some(&link)).alt, 0.0);
        for (alt, frame, key) in [("27", 10, egui::Key::Enter), ("bad", 0, egui::Key::Enter), ("99", 3, egui::Key::Escape)] {
            page.prompt = Some(Prompt::AltInput { title: "Enter Alt".into(), prompt_text: "Enter Guided Mode Alt".into(), alt: alt.into(), frame, then: AltInputted::FlyToHereAlt });
            let mut actions = Vec::new();
            let mut out = ctx.run_ui(egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1100.0, 750.0))),
                events: vec![egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() }],
                ..Default::default()
            }, |ui| {
                let mut app = AppCtx { link: Some(&link), extra_links: &[], tiles: &mut tiles, toasts: &mut toasts, settings: &mut settings, actions: &mut actions, sim_running: false, assets: &assets, readonly: false, joystick: &mut joystick };
                page.prompt_ui(ui.ctx(), &mut app);
            });
            out.textures_delta.clear();
            if alt == "27" {
                assert_eq!(active_guided(Some(&link)).alt, 27.0 / crate::units::alt(1.0));
                assert_eq!(active_guided(Some(&link)).frame, 10);
                assert_eq!(settings.planner["guided_alt_frame"], "10");
            } else {
                assert_eq!(active_guided(Some(&link)).frame, 10);
                assert_eq!(settings.planner["guided_alt"], "bad"); // original saves before parsing
            }
        }
        assert_eq!(toasts.0, ["Bad Alt"]);
        // A second MAV starts with no altitude and keeps its own selection.
        link.shared.write().cs.sysid = 2;
        assert_eq!(active_guided(Some(&link)).alt, 0.0);
        link.shared.write().set_guided_altitude(50.0, 0);
        link.shared.write().cs.sysid = 1;
        assert_eq!(active_guided(Some(&link)).frame, 10);
        assert_eq!(active_guided(Some(&link)).alt, 27.0 / crate::units::alt(1.0));
    }
}

#[cfg(test)]
mod quick_caption_tests {
    use super::*;

    #[test]
    fn captions_follow_get_name_and_unit_and_the_designer_defaults() {
        let mut pl = std::collections::BTreeMap::new();
        assert_eq!(quick_label(&pl, 0, "alt"), "Altitude (m)");
        assert_eq!(quick_label(&pl, 1, "groundspeed"), "GroundSpeed (m/s)");
        assert_eq!(quick_label(&pl, 3, "yaw"), "Yaw (deg)");
        // designer quickView6.desc has no CurrentState property behind it
        assert_eq!(quick_label(&pl, 5, "DistToHome"), "DistToMAV");
        pl.insert("quickView6".to_string(), "DistToHome".to_string());
        assert_eq!(quick_label(&pl, 5, "DistToHome"), "Dist to Home (m)");
        assert_eq!(name_and_unit("not_a_property"), "not_a_property");
        // English GetFieldDesc has no L10NU entry: the name itself
        assert_eq!(field_desc("roll"), "roll");
    }
}

#[cfg(test)]
mod action_tests {
    use super::*;

    #[test]
    fn do_action_commands_follow_butactiondo() {
        let copter = Firmware::ArduCopter2;
        assert_eq!(action_command("Preflight_Calibration", copter).unwrap(), (241, [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]));
        assert_eq!(action_command("Preflight_Calibration", Firmware::ArduPlane).unwrap().1[0], 0.0, "gyro only on Copter");
        assert_eq!(action_command("Reboot_Mass_Storage", copter).unwrap(), (246, [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        assert_eq!(action_command("Battery_Reset", copter).unwrap().1[..2], [255.0, 100.0]);
        // Enum.Parse finds no ADSB_OUT_IDENT / DO_START_ADSB_OUT_IDENT
        assert_eq!(action_command("ADSB_Out_Ident", copter), None);
        assert_eq!(ACTIONS.len(), 20);
    }

    #[test]
    fn failed_actions_report_command_failed_and_others_stay_silent() {
        let mut v = FlightDataView::default();
        let mut toasts = crate::app::Toasts::default();
        v.action_ack = Some((17, mp_core::mavcmd::cmd_name(17)));
        assert!(v.on_action_ack(&LinkEvent::CommandAck { command: 17, result: "DENIED".into(), ok: false }, None, &mut toasts));
        assert_eq!(toasts.0, ["The Command failed to execute\n LOITER_UNLIM"]);
        assert_eq!(toasts.1, ["Error"]);
        assert!(!v.on_action_ack(&LinkEvent::CommandAck { command: 17, result: "DENIED".into(), ok: false }, None, &mut toasts), "only the awaited ACK");
        v.action_ack = Some((185, String::new()));
        v.on_action_ack(&LinkEvent::CommandAck { command: 185, result: "DENIED".into(), ok: false }, None, &mut toasts);
        assert_eq!(toasts.0.len(), 1, "Terminate_Flight ignores the result");
    }
}

#[cfg(test)]
mod arm_tests {
    use super::*;

    #[test]
    fn a_refused_forced_arm_reports_rejected_by_mav_and_hides_the_generic_ack() {
        let mut v = FlightDataView::default();
        let mut toasts = crate::app::Toasts::default();
        v.force_pending = true;
        let nack = LinkEvent::CommandAck { command: 400, result: "DENIED".into(), ok: false };
        assert!(v.on_action_ack(&nack, None, &mut toasts), "BUT_ARM_Click reports arming itself");
        v.on_event(&nack);
        assert_eq!(v.arm_error.as_deref(), Some("Error: command rejected by MAV"));
        assert!(!v.force_pending);
    }

    #[test]
    fn arm_timeout_prompts_force_arm_and_enter_key_is_suppressed() {
        let ctx = egui::Context::default();
        let assets = crate::assets::Assets::load(&ctx);
        let mut tiles = crate::map::TileCache::offline();
        let mut toasts = crate::app::Toasts::default();
        let mut settings = crate::app::Settings::default();
        let mut joystick = crate::joystick::Joystick::default();
        let mut actions = Vec::new();

        let mut v = FlightDataView::default();
        let cs = CurrentState::default();

        // Arm sent 9 seconds ago with no ACK
        v.arm_pending = Some((true, 0, Instant::now() - std::time::Duration::from_secs(9)));
        {
            let mut app = AppCtx {
                link: None,
                extra_links: &[],
                tiles: &mut tiles,
                toasts: &mut toasts,
                settings: &mut settings,
                actions: &mut actions,
                sim_running: false,
                assets: &assets,
                readonly: false,
                joystick: &mut joystick,
            };
            v.actions_tick(&mut app, &cs);
        }
        assert!(v.prompt.is_some(), "arm failure prompt created by 8s timeout");

        // Run UI with Enter key pressed - must NOT trigger result
        let input = egui::RawInput {
            events: vec![egui::Event::Key {
                key: egui::Key::Enter,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            }],
            ..Default::default()
        };
        let mut out = ctx.run_ui(input, |ui| {
            let mut app = AppCtx {
                link: None,
                extra_links: &[],
                tiles: &mut tiles,
                toasts: &mut toasts,
                settings: &mut settings,
                actions: &mut actions,
                sim_running: false,
                assets: &assets,
                readonly: false,
                joystick: &mut joystick,
            };
            v.prompt_ui(ui.ctx(), &mut app);
        });
        out.textures_delta.clear();
        assert!(v.prompt.is_some(), "ForceArm prompt must not be dismissed by Enter key");
    }
}
