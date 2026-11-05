//! Port of `MainV2`: main window, top menu (DATA / PLAN / SETUP / CONFIG / SIMULATION / HELP),
//! connection controls and the active link (`MainV2.comPort`).

use std::time::{Duration, Instant};

use egui::{Color32, RichText, Vec2};
use mp_core::comms::{self, ConnectionSpec};
use mp_core::link::{Link, LinkEvent, LinkStatus};
use mp_core::sim::{self, SimHandle};

use crate::map::{MapState, TileCache};
use crate::views::{
    config::ConfigView, flight_data::FlightDataView, flight_plan::FlightPlanView, help,
    setup::SetupView,
};
use crate::{platform, theme};

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum View {
    FlightData,
    FlightPlan,
    Setup,
    Config,
    Simulation,
    Help,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Settings {
    pub view: View,
    pub port: String,
    pub baud: u32,
    pub tcp_host: String,
    pub tcp_port: u16,
    pub udp_port: u16,
    pub udpcl_host: String,
    pub udpcl_port: u16,
    pub fd_map: MapState,
    pub fp_map: MapState,
    pub default_alt: f32,
    pub wp_radius: f32,
    pub loiter_radius: f32,
    pub alt_frame: u8,
    pub quick_fields: Vec<String>,
    /// tableLayoutPanelQuick rows / columns (Set View Count)
    pub quick_rows: usize,
    pub quick_cols: usize,
    /// FlightData tabs hidden with Customize, and tabControlactions.Multiline
    pub fd_hidden_tabs: Vec<String>,
    pub fd_tabs_multiline: bool,
    pub quick_undocked: bool,
    pub sim_port: u16,
    pub user_params: Vec<String>,
    /// POIs shared by DATA and PLAN maps (lat, lng, name)
    pub pois: Vec<(f64, f64, String)>,
    /// FlightPlanner "Tracker Home"
    pub tracker_home: Option<(f64, f64)>,
    /// Antenna TrackerUI fields (Settings "Tracker_*")
    pub tracker_ui: std::collections::BTreeMap<String, String>,
    /// ConfigPlanner settings (Settings.Instance keys)
    pub planner: std::collections::BTreeMap<String, String>,
    /// contextMenuStripHud options
    pub hud: crate::hud::HudOptions,
    /// Set Aspect Ratio: 16:9 instead of 4:3
    pub hud_wide: bool,
    /// Swap With Map
    pub hud_swap: bool,
    pub mjpeg_url: String,
    pub sitl_home: (f64, f64),
    pub sitl_heading: f32,
    pub sitl_version: usize,
    pub sitl_model: String,
    pub sitl_speed: u32,
    pub sitl_extra: String,
    pub sitl_wipe: bool,
    pub fav_params: Vec<String>,
    pub ui_scale: f32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            view: View::FlightData,
            port: if cfg!(any(target_os = "android", target_os = "ios")) {
                "UDP".into()
            } else {
                "AUTO".into()
            },
            baud: 115200,
            tcp_host: "127.0.0.1".into(),
            tcp_port: 5760,
            udp_port: 14550,
            udpcl_host: "127.0.0.1".into(),
            udpcl_port: 14550,
            fd_map: MapState::default(),
            fp_map: MapState::default(),
            default_alt: 100.0,
            wp_radius: 2.0,
            loiter_radius: 45.0,
            alt_frame: 3,
            quick_fields: vec![
                "alt".into(),
                "groundspeed".into(),
                "wp_dist".into(),
                "yaw".into(),
                "verticalspeed".into(),
                "DistToHome".into(),
            ],
            quick_rows: 3,
            quick_cols: 2,
            fd_hidden_tabs: Vec::new(),
            fd_tabs_multiline: false,
            quick_undocked: false,
            sim_port: 14560,
            pois: Vec::new(),
            tracker_home: None,
            tracker_ui: Default::default(),
            user_params: Vec::new(),
            planner: Default::default(),
            hud: Default::default(),
            hud_wide: false,
            hud_swap: false,
            mjpeg_url: "http://127.0.0.1:8080/?action=stream".into(),
            sitl_home: (-35.363261, 149.165230),
            sitl_heading: 0.0,
            sitl_version: 2,
            sitl_model: String::new(),
            sitl_speed: 1,
            sitl_extra: String::new(),
            sitl_wipe: false,
            fav_params: Vec::new(),
            ui_scale: if cfg!(any(target_os = "android", target_os = "ios")) {
                1.6
            } else {
                1.0
            },
        }
    }
}

/// Mission Planner reports errors with a modal `CustomMessageBox`; informational messages
/// only go to the log (MP shows them in the Messages tab / HUD, not as popups).
#[derive(Default)]
pub struct Toasts(Vec<String>);

impl Toasts {
    pub fn info(&mut self, s: impl Into<String>) {
        log::info!("{}", s.into());
    }
    pub fn error(&mut self, s: impl Into<String>) {
        let s = s.into();
        log::warn!("{s}");
        if !self.0.contains(&s) {
            self.0.push(s);
        }
    }
}

pub enum AppAction {
    Connect(ConnectionSpec),
    Disconnect,
    StartSim,
    StopSim,
    DemoFlight,
    PlayTlog(std::path::PathBuf),
    /// ConfigAdvanced / temp: open the MAVLink Inspector
    OpenInspector,
    /// SITL swarm buttons: vehicle type and count, all on the simulator port
    StartSwarm(mp_core::sim::SimKind, u8),
    /// Planner video Start / Stop
    Video(bool),
    /// temp / ConfigAdvanced tool by button text
    Tool(String),
}

/// Shared context handed to each view.
pub struct AppCtx<'a> {
    pub link: Option<&'a Link>,
    pub tiles: &'a mut TileCache,
    pub toasts: &'a mut Toasts,
    pub settings: &'a mut Settings,
    pub actions: &'a mut Vec<AppAction>,
    pub sim_running: bool,
    pub assets: &'a crate::assets::Assets,
    pub readonly: bool,
    /// MainV2.joystick
    pub joystick: &'a mut crate::joystick::Joystick,
}

impl AppCtx<'_> {
    pub fn connected(&self) -> bool {
        self.link
            .map(|l| l.status() == LinkStatus::Connected)
            .unwrap_or(false)
    }
}

pub struct MissionPlannerApp {
    settings: Settings,
    link: Option<Link>,
    sim: Option<SimHandle>,
    swarm: Vec<SimHandle>,
    tiles: TileCache,
    toasts: Toasts,
    ports: Vec<String>,
    ports_refreshed: Instant,
    pub(crate) flight_data: FlightDataView,
    flight_plan: FlightPlanView,
    setup: SetupView,
    config: ConfigView,
    simulation: crate::views::simulation::SimulationView,
    show_conn_options: bool,
    show_stats: bool,
    inspector: crate::tools::Inspector,
    show_temp: bool,
    /// (theme name, revision) currently applied by ThemeManager
    applied_theme: (String, String),
    ctx_clone: egui::Context,
    tools: crate::tool_windows::Tools,
    more_tools: crate::more_tools::MoreTools,
    hotkey_tools: crate::hotkey_tools::HotkeyTools,
    skip_params_once: bool,
    joystick: crate::joystick::Joystick,
    speech: crate::speech::SpeechRules,
    vario: Option<std::time::Instant>,
    menu_hidden: bool,
    last_title: String,
    show_conn_list: bool,
    pub readonly: bool,
    assets: crate::assets::Assets,
    startup: Vec<AppAction>,
    demo: Option<crate::demo::Demo>,
}

const SETTINGS_KEY: &str = "mp_settings";

impl MissionPlannerApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        crate::fonts::install(&cc.egui_ctx);
        theme::apply(&cc.egui_ctx);
        mp_core::param_meta::warm_up();
        let mut settings: Settings = cc
            .storage
            .and_then(|s| eframe::get_value(s, SETTINGS_KEY))
            .unwrap_or_default();
        cc.egui_ctx.set_zoom_factor(settings.ui_scale);
        if let Ok(guid) = std::env::var("MP_DESKTOP") {
            if let Err(e) = platform::move_to_virtual_desktop(cc, &guid) {
                log::warn!("virtual desktop: {e}");
            }
            cc.egui_ctx.send_viewport_cmd(egui::ViewportCommand::Visible(true));
        }
        // Developer switches: MP_AUTOSIM=1 starts the simulator, MP_VIEW picks the screen.
        let mut startup = Vec::new();
        if std::env::var_os("MP_AUTOSIM").is_some() {
            startup.push(AppAction::StartSim);
        }
        if std::env::var_os("MP_SWARM").is_some() {
            startup.push(AppAction::StartSwarm(mp_core::sim::SimKind::Copter, 3));
        }
        if std::env::var_os("MP_DEMO").is_some() {
            startup.push(AppAction::DemoFlight);
        }
        if let Ok(v) = std::env::var("MP_VIEW") {
            settings.view = match v.to_ascii_lowercase().as_str() {
                "plan" => View::FlightPlan,
                "setup" => View::Setup,
                "config" => View::Config,
                "sim" => View::Simulation,
                "help" => View::Help,
                _ => View::FlightData,
            };
        }
        Self {
            settings,
            link: None,
            swarm: Vec::new(),
            sim: None,
            tiles: TileCache::new(Some(platform::tile_cache_dir())),
            toasts: Toasts::default(),
            ports: comms::serial_ports(),
            ports_refreshed: Instant::now(),
            flight_data: FlightDataView::default(),
            flight_plan: FlightPlanView::default(),
            setup: SetupView::default(),
            config: ConfigView::default(),
            simulation: Default::default(),
            show_conn_options: false,
            show_stats: false,
            inspector: crate::tools::Inspector::new(std::env::var_os("MP_INSPECTOR").is_some()),
            show_temp: std::env::var_os("MP_TEMP").is_some(),
            applied_theme: (String::new(), String::new()),
            ctx_clone: cc.egui_ctx.clone(),
            tools: Default::default(),
            more_tools: Default::default(),
            hotkey_tools: Default::default(),
            skip_params_once: false,
            joystick: Default::default(),
            speech: Default::default(),
            vario: None,
            menu_hidden: false,
            last_title: String::new(),
            show_conn_list: false,
            readonly: false,
            assets: crate::assets::Assets::load(&cc.egui_ctx),
            startup,
            demo: None,
        }
    }

    fn connection_spec(&self) -> Option<ConnectionSpec> {
        let s = &self.settings;
        Some(match s.port.as_str() {
            "TCP" => ConnectionSpec::Tcp {
                host: s.tcp_host.clone(),
                port: s.tcp_port,
            },
            "UDP" => ConnectionSpec::UdpListen { port: s.udp_port },
            "UDPCl" => ConnectionSpec::UdpClient {
                host: s.udpcl_host.clone(),
                port: s.udpcl_port,
            },
            "AUTO" => ConnectionSpec::Auto,
            p => ConnectionSpec::Serial {
                port: p.to_string(),
                baud: s.baud,
            },
        })
    }

    fn handle_actions(&mut self, actions: Vec<AppAction>) {
        for a in actions {
            match a {
                AppAction::Connect(spec) => {
                    self.joystick.stop(self.link.as_ref());
                    self.link = None;
                    self.toasts.info(format!("Connecting to {spec}"));
                    let get_params = !std::mem::take(&mut self.skip_params_once);
                    let l = Link::connect_with_params(spec, Some(platform::data_dir().join("logs")), get_params);
                    self.link = Some(l);
                }
                AppAction::Disconnect => {
                    self.joystick.stop(self.link.as_ref());
                    self.link = None;
                }
                AppAction::StartSim => {
                    if self.sim.is_none() {
                        let port = self.settings.sim_port;
                        self.joystick.stop(self.link.as_ref());
                        self.link = None;
                        self.link = Some(Link::connect_with(ConnectionSpec::UdpListen { port }, Some(platform::data_dir().join("logs"))));
                        match sim::spawn(port) {
                            Ok(h) => {
                                self.sim = Some(h);
                                self.toasts.info("Simulator started");
                            }
                            Err(e) => self.toasts.error(format!("Simulator failed: {e}")),
                        }
                    }
                }
                AppAction::PlayTlog(path) => {
                    self.joystick.stop(self.link.as_ref());
                    self.link = None;
                    match Link::playback(&path) {
                        Ok(l) => {
                            l.send(mp_core::link::LinkCommand::Playback(mp_core::tlog::PlaybackCommand::Play(true)));
                            self.link = Some(l);
                        }
                        Err(e) => self.toasts.error(format!("Error: {e}")),
                    }
                }
                AppAction::OpenInspector => self.inspector.open = true,
                AppAction::Video(on) => {
                    if on {
                        let url = self.settings.mjpeg_url.clone();
                        self.flight_data.start_video(&url, &self.ctx_clone);
                    } else {
                        self.flight_data.stop_video();
                    }
                }
                AppAction::Tool(name) => {
                    if !self.tools.activate(&name, self.link.as_ref(), &mut self.toasts) && !self.more_tools.activate(&name, self.link.as_ref(), &mut self.toasts, &mut self.settings) {
                        self.toasts.error(format!("{name}: not available in this port yet"));
                    }
                }
                AppAction::DemoFlight => {
                    self.demo = Some(crate::demo::Demo::default());
                    self.toasts
                        .info("Demo flight: uploading mission, arming, taking off");
                }
                AppAction::StartSwarm(kind, n) => {
                    self.joystick.stop(self.link.as_ref());
                    self.link = None;
                    self.sim = None;
                    self.swarm.clear();
                    let port = self.settings.sim_port;
                    self.link = Some(Link::connect_with(ConnectionSpec::UdpListen { port }, Some(platform::data_dir().join("logs"))));
                    for i in 0..n {
                        // SITL.cs: instances laid out on a grid, 10 m apart
                        let offset = ((i / 3) as f64 * 10.0, (i % 3) as f64 * 10.0);
                        match sim::spawn_with(sim::SimOptions { gcs_port: port, sysid: i + 1, kind, offset }) {
                            Ok(h) => self.swarm.push(h),
                            Err(e) => self.toasts.error(format!("Simulator failed: {e}")),
                        }
                    }
                    self.settings.view = View::FlightData;
                }
                AppAction::StopSim => {
                    self.swarm.clear();
                    self.joystick.stop(self.link.as_ref());
                    self.link = None;
                    if let Some(s) = self.sim.take() {
                        s.stop();
                    }
                    self.toasts.info("Simulator stopped");
                }
            }
        }
    }

    fn drain_events(&mut self) {
        let Some(link) = &self.link else { return };
        let events: Vec<LinkEvent> = link.events.try_iter().collect();
        if let Some(d) = self.demo.as_mut() {
            if d.tick(link, &events) {
                self.demo = None;
            }
        }
        let mut drop_link = false;
        for e in events {
            match &e {
                LinkEvent::Connected => {
                    // CurrentState.rate* from Planner, and "Load Waypoints on connect?"
                    let pl = &self.settings.planner;
                    let rate = |k: &str, d: u16| pl.get(k).and_then(|v| v.parse().ok()).unwrap_or(d);
                    link.send(mp_core::link::LinkCommand::SetStreamRates {
                        attitude: rate("rateattitude", 4),
                        position: rate("rateposition", 2),
                        status: rate("ratestatus", 2),
                        rc: rate("raterc", 2),
                        sensors: rate("ratesensors", 2),
                    });
                    if pl.get("loadwpsonconnect").map(|v| v == "True").unwrap_or(false) {
                        link.send(mp_core::link::LinkCommand::ReadMission { mission_type: 0 });
                    }
                    let cs = link.shared.read().cs.clone();
                    self.toasts
                        .info(format!("Connected: {:?} sysid {}", cs.firmware, cs.sysid));
                }
                LinkEvent::Disconnected(err) => {
                    match err {
                        Some(e) => self.toasts.error(format!("Connection closed: {e}")),
                        None => self.toasts.info("Disconnected"),
                    }
                    drop_link = true;
                }
                LinkEvent::ParamsLoaded(n) => self.toasts.info(format!("Got {n} parameters")),
                LinkEvent::AltitudeChanged(result) => match result {
                    Ok(()) => self.toasts.info("Altitude change accepted"),
                    Err(e) => self.toasts.error(e),
                },
                LinkEvent::OperationError(e) => self.toasts.error(e),
                LinkEvent::CommandAck {
                    command,
                    result,
                    ok,
                } => {
                    let name = mp_core::mavcmd::cmd_name(*command);
                    if *ok {
                        self.toasts.info(format!("{name}: {result}"));
                    } else {
                        self.toasts.error(format!("{name}: {result}"));
                    }
                }
                _ => {}
            }
            self.flight_plan.on_event(&e, &mut self.toasts);
            self.config.on_event(&e, &mut self.toasts);
            self.flight_data.on_event(&e);
            self.flight_data.on_log_event(&e);
        }
        if drop_link {
            self.joystick.stop(self.link.as_ref());
            self.link = None;
        }
    }

    fn top_bar(&mut self, ui: &mut egui::Ui, actions: &mut Vec<AppAction>) {
        // MainV2.MainMenu: MenuStrip on bgdark.jpg, 45x39 icons above Microsoft Sans Serif 8.25pt.
        let bar_h = 60.0;
        let (bar, bar_resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), bar_h), egui::Sense::click());
        let p = ui.painter().clone();
        let uv = egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(bar.width() / 150.0, bar.height() / 150.0));
        p.image(self.assets.bg.id(), bar, uv, Color32::WHITE);

        let font = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let a = &self.assets;
        let items = [
            (View::FlightData, "DATA", a.fd.id(), 49.0),
            (View::FlightPlan, "PLAN", a.fp.id(), 49.0),
            (View::Setup, "SETUP", a.setup.id(), 49.0),
            (View::Config, "CONFIG", a.config.id(), 51.0),
            (View::Simulation, "SIMULATION", a.sim.id(), 77.0),
            (View::Help, "HELP", a.help.id(), 49.0),
        ];
        let top = bar.top() + 2.0;
        let item = |ui: &mut egui::Ui, x: f32, w: f32, tex: egui::TextureId, label: &str, selected: bool, id: &str| -> (bool, f32) {
            // ToolStripButton AutoSize with ImageAboveText: 39px image + 13px text + padding
            let tw = ui.painter().layout_no_wrap(label.to_string(), font.clone(), theme::MENU_TEXT).size().x;
            let w = w.max(tw + 6.0);
            let r = egui::Rect::from_min_size(egui::pos2(x, top), Vec2::new(w, 56.0));
            let resp = ui.interact(r, ui.scope_id().with(id), egui::Sense::click());
            let p = ui.painter();
            if selected {
                p.rect_filled(r, 0.0, theme::control_bg());
            } else if resp.hovered() {
                p.rect_filled(r, 0.0, Color32::from_white_alpha(28));
            }
            let img = egui::Rect::from_center_size(egui::pos2(r.center().x, r.top() + 2.0 + 19.5), Vec2::new(45.0, 39.0));
            p.image(tex, img, egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(1.0, 1.0)), Color32::WHITE);
            p.text(egui::pos2(r.center().x, r.bottom() - 2.0), egui::Align2::CENTER_BOTTOM, label, font.clone(), theme::MENU_TEXT);
            (resp.clicked(), w)
        };
        let mut x = bar.left() + 6.0;
        for (v, label, tex, w) in items {
            let (clicked, w) = item(ui, x, w, tex, label, self.settings.view == v, label);
            if clicked {
                self.settings.view = v;
            }
            x += w;
        }
        let left_end = x;

        // Right-aligned items: CONNECT, then the ConnectionControl, then the ArduPilot logo.
        let status = self.link.as_ref().map(|l| l.status()).unwrap_or(LinkStatus::Disconnected);
        let (clabel, ctex) = if status == LinkStatus::Disconnected {
            ("CONNECT", self.assets.connect.id())
        } else {
            ("DISCONNECT", self.assets.disconnect.id())
        };
        let cw = ui.painter().layout_no_wrap(clabel.to_string(), font.clone(), theme::MENU_TEXT).size().x.max(57.0) + 6.0;
        let mut rx = bar.right() - cw;
        if item(ui, rx, cw, ctex, clabel, false, "connect").0 {
            if status == LinkStatus::Disconnected {
                match self.settings.port.as_str() {
                    // Mission Planner asks for host/port at connect time.
                    "TCP" | "UDP" | "UDPCl" => self.show_conn_options = true,
                    _ => match self.connection_spec() {
                        Some(spec) => actions.push(AppAction::Connect(spec)),
                        None => self.toasts.error("No serial ports found"),
                    },
                }
            } else {
                actions.push(AppAction::Disconnect);
            }
        }

        // ConnectionControl (238x47): cmb_Connection 121x21 @0,0; cmb_Baud 111x21 @127,0;
        // "Stats..." @3,32; cmb_sysid 136x21 @102,27.
        rx -= 238.0;
        if rx > left_end + 10.0 {
            if self.ports_refreshed.elapsed() > Duration::from_secs(3) {
                self.ports = comms::serial_ports();
                self.ports_refreshed = Instant::now();
            }
            let cc = egui::pos2(rx, top);
            let enabled = status == LinkStatus::Disconnected;
            let mut child = ui.new_child(egui::UiBuilder::new().max_rect(egui::Rect::from_min_size(cc, Vec2::new(238.0, 47.0))));
            {
                let v = child.visuals_mut();
                for w in [&mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
                    w.bg_fill = Color32::BLACK;
                    w.weak_bg_fill = Color32::BLACK;
                }
            }
            let ports = self.ports.clone();
            let settings = &mut self.settings;
            child.add_enabled_ui(enabled, |ui| {
                ui.put(egui::Rect::from_min_size(cc, Vec2::new(121.0, 21.0)), |ui: &mut egui::Ui| {
                    egui::ComboBox::from_id_salt("port")
                        .width(113.0)
                        .selected_text(settings.port.clone())
                        .show_ui(ui, |ui| {
                            let mut opts: Vec<String> = Vec::new();
                            if platform::HAS_NATIVE_DIALOGS {
                                opts.push("AUTO".into());
                            }
                            opts.extend(ports.iter().cloned());
                            opts.extend(["TCP".into(), "UDP".into(), "UDPCl".into()]);
                            for o in opts {
                                ui.selectable_value(&mut settings.port, o.clone(), o);
                            }
                        })
                        .response
                });
                ui.put(egui::Rect::from_min_size(cc + Vec2::new(127.0, 0.0), Vec2::new(111.0, 21.0)), |ui: &mut egui::Ui| {
                    egui::ComboBox::from_id_salt("baud")
                        .width(103.0)
                        .selected_text(settings.baud.to_string())
                        .show_ui(ui, |ui| {
                            for b in comms::BAUD_RATES {
                                ui.selectable_value(&mut settings.baud, *b, b.to_string());
                            }
                        })
                        .response
                });
            });
            let stats = child.put(
                egui::Rect::from_min_size(cc + Vec2::new(3.0, 32.0), Vec2::new(40.0, 13.0)),
                egui::Label::new(RichText::new("Stats...").color(Color32::from_rgb(0, 0, 255)).underline()).sense(egui::Sense::click()),
            );
            if stats.clicked() {
                self.show_stats = !self.show_stats;
            }
            if let Some(link) = &self.link {
                let s = link.shared.read();
                if s.status == LinkStatus::Connected {
                    let label = format!("{}-{}-{:?}", s.cs.sysid, s.cs.compid, s.cs.firmware);
                    let mavs: Vec<(u8, u8, String)> = s.mavs.iter().map(|((a, b), m)| (*a, *b, format!("{a}-{b}-{:?}", m.firmware))).collect();
                    drop(s);
                    let mut pick = None;
                    child.put(egui::Rect::from_min_size(cc + Vec2::new(102.0, 27.0), Vec2::new(136.0, 21.0)), |ui: &mut egui::Ui| {
                        egui::ComboBox::from_id_salt("sysid")
                            .width(128.0)
                            .selected_text(label.clone())
                            .show_ui(ui, |ui| {
                                for (a, b, n) in &mavs {
                                    if ui.selectable_label(*n == label, n).clicked() {
                                        pick = Some((*a, *b));
                                    }
                                }
                            })
                            .response
                    });
                    if let Some((sysid, compid)) = pick {
                        self.joystick.stop(Some(link));
                        link.send(mp_core::link::LinkCommand::SetTarget { sysid, compid });
                    }
                }
            }

            // MenuArduPilot: logo scaled to 200x31
            rx -= 200.0;
            if rx > left_end + 10.0 {
                let r = egui::Rect::from_min_size(egui::pos2(rx, bar.center().y - 15.5), Vec2::new(200.0, 31.0));
                let resp = ui.interact(r, ui.scope_id().with("logo"), egui::Sense::click());
                p.image(self.assets.logo.id(), r, egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(1.0, 1.0)), Color32::WHITE);
                if resp.clicked() {
                    ui.ctx().open_url(egui::OpenUrl::new_tab("https://ardupilot.org/"));
                }
            }
        }

        if self.show_conn_options {
            let mut open = true;
            let mut go = false;
            let title = match self.settings.port.as_str() {
                "TCP" => "TCP Connection",
                "UDP" => "UDP Listen",
                _ => "UDP Client",
            };
            egui::Window::new(title)
                .open(&mut open)
                .resizable(false)
                .collapsible(false)
                .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
                .show(ui.ctx(), |ui| {
                    egui::Grid::new("connopts").num_columns(2).show(ui, |ui| match self.settings.port.as_str() {
                        "TCP" => {
                            ui.label("Enter host name/ip (ensure remote end is already started)");
                            ui.text_edit_singleline(&mut self.settings.tcp_host);
                            ui.end_row();
                            ui.label("Enter remote port");
                            ui.add(egui::DragValue::new(&mut self.settings.tcp_port));
                            ui.end_row();
                        }
                        "UDP" => {
                            ui.label("Enter Local port (ensure remote end is already sending)");
                            ui.add(egui::DragValue::new(&mut self.settings.udp_port));
                            ui.end_row();
                        }
                        _ => {
                            ui.label("Enter remote host");
                            ui.text_edit_singleline(&mut self.settings.udpcl_host);
                            ui.end_row();
                            ui.label("Enter remote port");
                            ui.add(egui::DragValue::new(&mut self.settings.udpcl_port));
                            ui.end_row();
                        }
                    });
                    ui.horizontal(|ui| {
                        if theme::button(ui, "OK").clicked() {
                            go = true;
                        }
                        if theme::button(ui, "Cancel").clicked() {
                            self.show_conn_options = false;
                        }
                    });
                });
            if go {
                if let Some(spec) = self.connection_spec() {
                    actions.push(AppAction::Connect(spec));
                }
                self.show_conn_options = false;
            }
            if !open {
                self.show_conn_options = false;
            }
        }

        self.main_menu_ctx(&bar_resp, actions);

        if self.show_stats {
            let mut open = true;
            egui::Window::new("Connection Stats").open(&mut open).resizable(false).collapsible(false).show(ui.ctx(), |ui| match &self.link {
                Some(l) => {
                    let s = l.shared.read();
                    egui::Grid::new("stats").num_columns(2).show(ui, |ui| {
                        ui.label("Packets Received");
                        ui.label(s.cs.packets_received.to_string());
                        ui.end_row();
                        ui.label("Packets Lost");
                        ui.label(s.cs.packets_lost.to_string());
                        ui.end_row();
                        ui.label("Link Quality");
                        ui.label(format!("{:.0}%", s.cs.linkqualitygcs));
                        ui.end_row();
                        ui.label("Connection");
                        ui.label(s.spec.as_ref().map(|c| c.to_string()).unwrap_or_default());
                        ui.end_row();
                    });
                }
                None => {
                    ui.label("Not connected");
                }
            });
            if !open {
                self.show_stats = false;
            }
        }
    }

    /// MainV2.ProcessCmdKey: F12 connect, F2 DATA, F3 PLAN, F4 CONFIG, F5 refresh params.
    fn hotkeys(&mut self, ctx: &egui::Context, actions: &mut Vec<AppAction>) {
        use egui::Key;
        let (f12, f2, f3, f4, f5) = ctx.input(|i| (i.key_pressed(Key::F12), i.key_pressed(Key::F2), i.key_pressed(Key::F3), i.key_pressed(Key::F4), i.key_pressed(Key::F5)));
        // Ctrl+F: temp form
        if ctx.input(|i| i.modifiers.command && i.key_pressed(Key::F)) {
            self.show_temp = true;
        }
        if f12 {
            match self.link.as_ref().map(|l| l.status()) {
                Some(LinkStatus::Connected) | Some(LinkStatus::Connecting) => actions.push(AppAction::Disconnect),
                _ => {
                    if let Some(spec) = self.connection_spec() {
                        actions.push(AppAction::Connect(spec));
                    }
                }
            }
        }
        if f2 {
            self.settings.view = View::FlightData;
        }
        if f3 {
            self.settings.view = View::FlightPlan;
        }
        if f4 {
            self.settings.view = View::Config;
        }
        if f5 {
            if let Some(l) = &self.link {
                l.send(mp_core::link::LinkCommand::RequestParams);
            }
        }
        // Text editors own cut, undo and the other editing shortcuts while focused.
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let ctrl = |k: Key| ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, k));
        // Ctrl+P PluginUI
        if ctrl(Key::P) {
            self.hotkey_tools.plugins_open = true;
        }
        // Ctrl+G SerialOutputNMEA
        if ctrl(Key::G) {
            actions.push(AppAction::Tool("NMEA".into()));
        }
        // Ctrl+X GMAPCache
        if ctrl(Key::X) {
            self.hotkey_tools.cache_open = true;
        }
        // Ctrl+L SpectrogramUI
        if ctrl(Key::L) {
            actions.push(AppAction::Tool("Spectrogram".into()));
        }
        // Ctrl+Z Camera().test: request camera / video / storage info and start streaming
        if ctrl(Key::Z) {
            if let Some(l) = &self.link {
                let cam = l.shared.read().mavs.keys().find(|(_, c)| *c == 100).copied();
                if let Some((sys, comp)) = cam {
                    use mp_core::mavlink::dialects::ardupilotmega::*;
                    for cmd in [MavCmd::MAV_CMD_REQUEST_CAMERA_INFORMATION, MavCmd::MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, MavCmd::MAV_CMD_REQUEST_CAMERA_SETTINGS, MavCmd::MAV_CMD_SET_CAMERA_MODE, MavCmd::MAV_CMD_REQUEST_STORAGE_INFORMATION, MavCmd::MAV_CMD_VIDEO_START_STREAMING] {
                        l.send(mp_core::link::LinkCommand::Send(Box::new(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA { param1: 0.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0, command: cmd, target_system: sys, target_component: comp, confirmation: 0 }))));
                    }
                }
            }
        }
        // Ctrl+T: connect without downloading parameters (comPort.Open(false))
        if ctrl(Key::T) {
            if let Some(spec) = self.connection_spec() {
                actions.push(AppAction::Connect(spec));
                self.skip_params_once = true;
            }
        }
        // Ctrl+Y: PREFLIGHT_STORAGE write
        if ctrl(Key::Y) {
            match &self.link {
                Some(l) => {
                    l.send(mp_core::link::LinkCommand::CommandLong { command: 245, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    self.toasts.info("Storage write requested");
                }
                None => self.toasts.error("Invalid command"),
            }
        }
        // Ctrl+J DevopsUI
        if ctrl(Key::J) {
            self.hotkey_tools.devops_open = true;
        }
    }

    /// CTX_mainmenu: AutoHide, Full Screen, Readonly, Connection Options, Connection List.
    fn main_menu_ctx(&mut self, resp: &egui::Response, _actions: &mut Vec<AppAction>) {
        resp.context_menu(|ui| {
            if ui.checkbox(&mut self.menu_hidden, "AutoHide").clicked() {
                ui.close();
            }
            let fs = ui.ctx().input(|i| i.viewport().fullscreen.unwrap_or(false));
            let mut full = fs;
            if ui.checkbox(&mut full, "Full Screen").clicked() {
                ui.ctx().send_viewport_cmd(egui::ViewportCommand::Fullscreen(full));
                ui.close();
            }
            if ui.checkbox(&mut self.readonly, "Readonly").clicked() {
                ui.close();
            }
            if ui.button("Connection Options").clicked() {
                self.show_conn_options = true;
                ui.close();
            }
            if ui.button("Connection List").clicked() {
                self.show_conn_list = true;
                ui.close();
            }
        });
        if self.show_conn_list {
            let mut open = true;
            egui::Window::new("Connection List").open(&mut open).collapsible(false).show(&resp.ctx, |ui| {
                theme::label(ui, "Active connections:");
                match &self.link {
                    Some(l) => {
                        let s = l.shared.read();
                        theme::label(ui, format!("{}  sysid {}  {:?}", s.spec.as_ref().map(|c| c.to_string()).unwrap_or_default(), s.cs.sysid, s.status));
                    }
                    None => {
                        theme::label(ui, "none");
                    }
                }
            });
            if !open {
                self.show_conn_list = false;
            }
        }
    }

    /// `ProgressReporterDialogue`: title = current step, themed progress bar, Cancel.
    fn progress_window(&self, ctx: &egui::Context) {
        let Some(link) = &self.link else { return };
        let p = link.shared.read().progress.clone();
        if let Some(p) = p {
            egui::Window::new(&p.label)
                .id(egui::Id::unique("progress"))
                .resizable(false)
                .collapsible(false)
                .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
                .fixed_size(Vec2::new(370.0, 110.0))
                .show(ctx, |ui| {
                    theme::label(ui, &p.label);
                    ui.add_space(6.0);
                    let frac = if p.total > 0 { p.done as f32 / p.total as f32 } else { 0.0 };
                    let txt = if p.total > 0 { format!("{}/{}", p.done, p.total) } else { String::new() };
                    theme::progress(ui, frac, Vec2::new(358.0, 23.0), &txt);
                    ui.add_space(6.0);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                        if theme::button(ui, "Cancel").clicked() {
                            link.shared.write().progress = None;
                        }
                    });
                });
        }
    }

    /// `CustomMessageBox`: themed modal with OK.
    fn toasts_ui(&mut self, ctx: &egui::Context) {
        let Some(msg) = self.toasts.0.first().cloned() else { return };
        let mut close = false;
        egui::Modal::new(egui::Id::unique("msgbox")).show(ctx, |ui| {
            ui.set_min_width(300.0);
            ui.label(RichText::new("Mission Planner").strong().color(Color32::WHITE));
            ui.add_space(8.0);
            theme::label(ui, &msg);
            ui.add_space(10.0);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                if theme::button(ui, "OK").clicked() {
                    close = true;
                }
            });
        });
        if close {
            self.toasts.0.remove(0);
        }
    }
}

impl eframe::App for MissionPlannerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.drain_events();
        let mut actions = std::mem::take(&mut self.startup);

        self.hotkeys(&ctx, &mut actions);
        self.inspector.show(&ctx, self.link.as_ref(), &mut self.toasts);
        self.tools.ui(&ctx, self.link.as_ref(), &mut self.toasts);
        self.more_tools.ui(&ctx, self.link.as_ref(), &mut self.toasts);
        // ThemeManager.LoadTheme(CMB_theme.Text)
        let want = (
            self.settings.planner.get("theme").cloned().unwrap_or_else(|| "BurntKermit".into()),
            self.settings.planner.get("theme_rev").cloned().unwrap_or_default(),
        );
        if want != self.applied_theme {
            theme::set_palette(&ctx, theme::palette_for(&want.0));
            self.applied_theme = want;
        }
        crate::units::set_from(&self.settings.planner);
        self.joystick.tick(self.link.as_ref(), self.readonly);
        self.ctx_clone = ctx.clone();
        if let Some(l) = &self.link {
            let (cs, msgs) = {
                let s = l.shared.read();
                (s.cs.clone(), s.messages.clone())
            };
            self.speech.tick(&self.settings.planner, &cs, &msgs);
            // Vario: spoken climb / sink rate every 3 s while enabled
            if self.settings.planner.get("vario").map(|v| v == "True").unwrap_or(false) && self.vario.map(|t| t.elapsed().as_secs() >= 3).unwrap_or(true) {
                self.vario = Some(std::time::Instant::now());
                let v = cs.verticalspeed;
                if v.abs() >= 0.3 {
                    crate::speech::say(format!("{} {:.1}", if v > 0.0 { "up" } else { "down" }, v.abs()));
                }
            }
        }
        let fw_name = self.link.as_ref().map(|l| format!("{:?}", l.shared.read().cs.firmware)).unwrap_or_else(|| "ArduCopter2".into());
        self.joystick.window(&ctx, &fw_name, &mut self.toasts);
        if self.joystick.enabled {
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
        }
        match crate::tools::temp_window(&ctx, &mut self.show_temp, self.link.as_ref(), &mut self.toasts) {
            Some(crate::tools::TempAction::Inspector) => self.inspector.open = true,
            Some(crate::tools::TempAction::HwIds) => {
                self.settings.view = View::Setup;
                self.setup.goto("HW ID");
            }
            Some(crate::tools::TempAction::LogBrowse) => {
                self.settings.view = View::FlightData;
                self.flight_data.open_log_browse();
            }
            Some(crate::tools::TempAction::Unavailable("Geo ref images")) => self.flight_data.georef.open = true,
            Some(crate::tools::TempAction::Unavailable(n)) => {
                if !self.tools.activate(n, self.link.as_ref(), &mut self.toasts) && !self.more_tools.activate(n, self.link.as_ref(), &mut self.toasts, &mut self.settings) {
                    self.toasts.error(format!("{n}: not available in this port yet"));
                }
            }
            None => {}
        }
        if !self.menu_hidden {
            egui::Panel::top("menu").frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| self.top_bar(ui, &mut actions));
        } else {
            // AutoHide: the menu reappears when the pointer reaches the top edge
            let near_top = ctx.input(|i| i.pointer.hover_pos().map(|p| p.y < 6.0).unwrap_or(false));
            if near_top {
                egui::Panel::top("menu").frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| self.top_bar(ui, &mut actions));
            }
        }

        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(theme::bg()))
            .show(ui, |ui| {
                let Self {
                    settings,
                    link,
                    tiles,
                    toasts,
                    flight_data,
                    flight_plan,
                    setup,
                    config,
                    simulation,
                    sim,
                    swarm,
                    assets,
                    readonly,
                    joystick,
                    hotkey_tools,
                    ..
                } = self;
                let view = settings.view;
                let mut app = AppCtx {
                    link: link.as_ref(),
                    tiles,
                    toasts,
                    settings,
                    actions: &mut actions,
                    sim_running: sim.is_some() || !swarm.is_empty(),
                    assets,
                    readonly: *readonly,
                    joystick,
                };
                hotkey_tools.ui(ui.ctx(), &mut app);
                match view {
                    View::FlightData => flight_data.ui(ui, &mut app),
                    View::FlightPlan => flight_plan.ui(ui, &mut app),
                    View::Setup => setup.ui(ui, &mut app),
                    View::Config => config.ui(ui, &mut app),
                    View::Simulation => simulation.ui(ui, &mut app),
                    View::Help => help::ui(ui),
                }
            });

        self.progress_window(&ctx);
        self.toasts_ui(&ctx);
        self.handle_actions(actions);

        // MainV2: titlebar + " " + MAV.VersionString + " on " + MAV.SerialString
        let title = match &self.link {
            Some(l) if l.status() == LinkStatus::Connected => {
                let s = l.shared.read();
                let ver = s.messages.iter().find(|m| m.text.starts_with("Ardu") || m.text.starts_with("PX4")).map(|m| m.text.clone());
                let ser = s.messages.iter().find(|m| m.text.contains("ChibiOS") || m.text.contains("NuttX")).map(|m| m.text.clone());
                match (ver, ser) {
                    (Some(v), Some(sr)) => format!("{} {v} on {sr}", crate::title_base()),
                    (Some(v), None) => format!("{} {v}", crate::title_base()),
                    _ => crate::title_base(),
                }
            }
            _ => crate::title_base(),
        };
        if title != self.last_title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.last_title = title;
        }
        if self.link.is_some() {
            // live telemetry
            ctx.request_repaint_after(Duration::from_millis(50));
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, SETTINGS_KEY, &self.settings);
    }
}
