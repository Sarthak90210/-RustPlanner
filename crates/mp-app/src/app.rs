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
    pub ws_url: String,
    pub ble_address: String,
    pub winusb_path: String,
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
            ws_url: "ws://127.0.0.1:8080/mavlink".into(),
            ble_address: String::new(),
            winusb_path: String::new(),
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
pub struct Toasts(pub(crate) Vec<String>);

impl Toasts {
    pub fn info(&mut self, s: impl Into<String>) {
        log::info!("{}", s.into());
    }
    /// CustomMessageBox texts come from the views and Strings.resx: exact matches translate.
    #[track_caller]
    pub fn error(&mut self, s: impl Into<String>) {
        let s = crate::i18n::t(&s.into());
        log::warn!("{s}");
        if !self.0.contains(&s) {
            self.0.push(s);
        }
    }
}

pub enum AppAction {
    Connect(ConnectionSpec),
    /// ConnectionOptions / Connection List: another MAVLinkInterface added to Comports
    ConnectExtra(ConnectionSpec),
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
    /// MainV2.Comports other than the active comPort
    pub extra_links: &'a [Link],
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
    capture: Option<crate::capture::Capture>,
    /// last renderer frame taken for the web service's image streams
    stream_shot: Instant,
    skip_params_once: bool,
    /// MainV2.Comports besides comPort (the active `link`)
    extra_links: Vec<Link>,
    /// ConnectionOptions form: (port, baud)
    conn_form: Option<(String, u32)>,
    /// InputBox "Enter Password": Some(view) to open a protected screen, None to set it
    pw_prompt: Option<(Option<View>, String)>,
    /// last seen value of the "Password Protect Config" box
    pw_protect_seen: Option<bool>,
    joystick: crate::joystick::Joystick,
    speech: crate::speech::SpeechRules,
    vario: Option<std::time::Instant>,
    menu_hidden: bool,
    menu_reveal_height: f32,
    last_title: String,
    pub readonly: bool,
    assets: crate::assets::Assets,
    startup: Vec<AppAction>,
    /// MP_TOOL=<temp button text>: opened once the startup simulators are connected
    startup_tool: Option<String>,
    demo: Option<crate::demo::Demo>,
}

const SETTINGS_KEY: &str = "mp_settings";

impl MissionPlannerApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        mp_core::param_meta::warm_up();
        let mut settings: Settings = cc
            .storage
            .and_then(|s| eframe::get_value(s, SETTINGS_KEY))
            .unwrap_or_default();
        // Settings "language": the CurrentUICulture for this run
        // MP_LANGUAGE=<culture> overrides it for captures
        let language = std::env::var("MP_LANGUAGE").ok().or_else(|| settings.planner.get("language").cloned()).unwrap_or_else(|| crate::i18n::resolve(&crate::i18n::os_ui_culture()).to_string());
        crate::i18n::init(&language);
        crate::fonts::install(&cc.egui_ctx);
        theme::apply(&cc.egui_ctx);
        cc.egui_ctx.set_zoom_factor(settings.ui_scale);
        if let Ok(guid) = std::env::var("MP_DESKTOP") {
            if let Err(e) = platform::move_to_virtual_desktop(cc, &guid) {
                log::warn!("virtual desktop: {e}");
            }
            cc.egui_ctx.send_viewport_cmd(egui::ViewportCommand::Visible(true));
        }
        // MainV2: DisplayConfiguration from "displayview", forcing the parameter pages
        crate::display::force_on_load(&mut settings.planner);
        // MainV2: PluginLoader.LoadAll
        crate::plugins::start(&settings.planner);
        // MainV2: "start http" - the web service on port 56781
        crate::httpserver::start();
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
        // MP_COORDS=GEO|UTM|MGRS selects PLAN's coordinate system (Settings "fpcoordmouse")
        if let Ok(v) = std::env::var("MP_COORDS") {
            settings.planner.insert("fpcoordmouse".into(), v.to_ascii_uppercase());
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
        let menu_hidden = crate::display::flag(&settings.planner, "autoHideMenuForce")
            || settings.planner.get("menu_autohide").map(|s| s.eq_ignore_ascii_case("true")).unwrap_or(false);
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
            capture: crate::capture::Capture::from_env(),
            stream_shot: Instant::now(),
            skip_params_once: false,
            extra_links: Vec::new(),
            conn_form: None,
            pw_prompt: None,
            pw_protect_seen: None,
            joystick: Default::default(),
            speech: Default::default(),
            vario: None,
            menu_hidden,
            menu_reveal_height: 0.0,
            last_title: String::new(),
            readonly: false,
            assets: crate::assets::Assets::load(&cc.egui_ctx),
            startup,
            startup_tool: std::env::var("MP_TOOL").ok(),
            demo: None,
        }
    }

    fn connection_spec(&self) -> Option<ConnectionSpec> {
        Some(spec_from(&self.settings))
    }
}

/// The connection the CONNECT button would open (ConnectionControl port and baud).
pub fn spec_from(s: &Settings) -> ConnectionSpec {
    {
        match s.port.as_str() {
            "TCP" => ConnectionSpec::Tcp {
                host: s.tcp_host.clone(),
                port: s.tcp_port,
            },
            "UDP" => ConnectionSpec::UdpListen { port: s.udp_port },
            "UDPCl" => ConnectionSpec::UdpClient {
                host: s.udpcl_host.clone(),
                port: s.udpcl_port,
            },
            "WS" => ConnectionSpec::WebSocket { url: s.ws_url.clone() },
            "AUTO" => ConnectionSpec::Auto,
            p if p.starts_with("BLE") => ConnectionSpec::Ble { address: s.ble_address.clone() },
            p if p.starts_with("WinUSB") => ConnectionSpec::WinUsb { path: s.winusb_path.clone() },
            p => ConnectionSpec::Serial {
                port: p.to_string(),
                baud: s.baud,
            },
        }
    }
}

impl MissionPlannerApp {

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
                    // the next comport carries on as the active one
                    if !self.extra_links.is_empty() {
                        self.link = Some(self.extra_links.remove(0));
                    }
                }
                AppAction::ConnectExtra(spec) => {
                    self.toasts.info(format!("Connecting to {spec}"));
                    let l = Link::connect_with(spec, Some(platform::data_dir().join("logs")));
                    if self.link.is_none() {
                        self.link = Some(l);
                    } else {
                        self.extra_links.push(l);
                    }
                }
                AppAction::StartSim => {
                    if self.sim.is_none() {
                        let port = self.settings.sim_port;
                        self.joystick.stop(self.link.as_ref());
                        self.link = None;
                        self.link = Some(Link::connect_with(ConnectionSpec::UdpListen { port }, Some(platform::data_dir().join("logs"))));
                        let kind = match std::env::var("MP_SIM_KIND").as_deref() {
                            Ok("plane") => sim::SimKind::Plane,
                            Ok("rover") => sim::SimKind::Rover,
                            Ok("tracker") => sim::SimKind::Tracker,
                            _ => sim::SimKind::Copter,
                        };
                        match sim::spawn_with(sim::SimOptions { gcs_port: port, sysid: 1, kind, offset: (0.0, 0.0) }) {
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
                        // SITL.cs: home.newpos(NUM_heading = 0, a * 4): a line 4 m apart to the north
                        let offset = (i as f64 * 4.0, 0.0);
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
        // other comports run their own workers; their events are not shown
        for l in &self.extra_links {
            for _ in l.events.try_iter() {}
        }
        self.extra_links.retain(|l| l.status() != LinkStatus::Disconnected || l.shared.read().last_packet.is_none());
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
            self.setup.on_event(&e, link, &mut self.toasts);
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
        // updateLayout: MenuSimulation / MenuHelp follow displaySimulation / displayHelp
        let pl = &self.settings.planner;
        let (sim, help) = (crate::display::flag(pl, "displaySimulation"), crate::display::flag(pl, "displayHelp"));
        let items: Vec<_> = [
            (View::FlightData, "DATA", a.fd.id(), 49.0),
            (View::FlightPlan, "PLAN", a.fp.id(), 49.0),
            (View::Setup, "SETUP", a.setup.id(), 49.0),
            (View::Config, "CONFIG", a.config.id(), 51.0),
            (View::Simulation, "SIMULATION", a.sim.id(), 77.0),
            (View::Help, "HELP", a.help.id(), 49.0),
        ]
        .into_iter()
        .filter(|(v, ..)| match v {
            View::Simulation => sim,
            View::Help => help,
            _ => true,
        })
        .collect();
        let top = bar.top() + 2.0;
        let item = |ui: &mut egui::Ui, x: f32, w: f32, tex: egui::TextureId, label: &str, selected: bool, id: &str| -> (bool, f32) {
            // ToolStripButton AutoSize with ImageAboveText: 39px image + 13px text + padding
            let label = &*crate::i18n::tr(label);
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
                self.goto_view(v);
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
                    // UpdateSysIDS: every MAV on every comport (None = the active one)
                    let mut mavs: Vec<(Option<usize>, u8, u8, String)> = s.mavs.iter().map(|((a, b), m)| (None, *a, *b, format!("{a}-{b}-{:?}", m.firmware))).collect();
                    drop(s);
                    for (i, l) in self.extra_links.iter().enumerate() {
                        let s = l.shared.read();
                        mavs.extend(s.mavs.iter().map(|((a, b), m)| (Some(i), *a, *b, format!("{a}-{b}-{:?}", m.firmware))));
                    }
                    let mut pick = None;
                    child.put(egui::Rect::from_min_size(cc + Vec2::new(102.0, 27.0), Vec2::new(136.0, 21.0)), |ui: &mut egui::Ui| {
                        egui::ComboBox::from_id_salt("sysid")
                            .width(128.0)
                            .selected_text(label.clone())
                            .show_ui(ui, |ui| {
                                for (li, a, b, n) in &mavs {
                                    if ui.selectable_label(li.is_none() && *n == label, n).clicked() {
                                        pick = Some((*li, *a, *b));
                                    }
                                }
                            })
                            .response
                    });
                    if let Some((li, sysid, compid)) = pick {
                        self.joystick.stop(Some(link));
                        // cmb_sysid: comPort becomes the port that MAV is on
                        if let Some(i) = li {
                            let next = self.extra_links.remove(i);
                            if let Some(prev) = self.link.take() {
                                self.extra_links.push(prev);
                            }
                            self.link = Some(next);
                        }
                        if let Some(l) = &self.link {
                            l.send(mp_core::link::LinkCommand::SetTarget { sysid, compid });
                        }
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
                            ui.label(crate::i18n::t("Enter host name/ip (ensure remote end is already started)"));
                            ui.text_edit_singleline(&mut self.settings.tcp_host);
                            ui.end_row();
                            ui.label(crate::i18n::t("Enter remote port"));
                            ui.add(egui::DragValue::new(&mut self.settings.tcp_port));
                            ui.end_row();
                        }
                        "UDP" => {
                            ui.label(crate::i18n::t("Enter Local port (ensure remote end is already sending)"));
                            ui.add(egui::DragValue::new(&mut self.settings.udp_port));
                            ui.end_row();
                        }
                        _ => {
                            ui.label(crate::i18n::t("Enter remote host"));
                            ui.text_edit_singleline(&mut self.settings.udpcl_host);
                            ui.end_row();
                            ui.label(crate::i18n::t("Enter remote port"));
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
            egui::Window::new(crate::i18n::t("Connection Stats")).open(&mut open).resizable(false).collapsible(false).show(ui.ctx(), |ui| match &self.link {
                Some(l) => {
                    let s = l.shared.read();
                    egui::Grid::new("stats").num_columns(2).show(ui, |ui| {
                        ui.label(crate::i18n::t("Packets Received"));
                        ui.label(s.cs.packets_received.to_string());
                        ui.end_row();
                        ui.label(crate::i18n::t("Packets Lost"));
                        ui.label(s.cs.packets_lost.to_string());
                        ui.end_row();
                        ui.label(crate::i18n::t("Link Quality"));
                        ui.label(format!("{:.0}%", s.cs.linkqualitygcs));
                        ui.end_row();
                        ui.label(crate::i18n::t("Connection"));
                        ui.label(s.spec.as_ref().map(|c| c.to_string()).unwrap_or_default());
                        ui.end_row();
                    });
                }
                None => {
                    ui.label(crate::i18n::t("Not connected"));
                }
            });
            if !open {
                self.show_stats = false;
            }
        }
    }

    /// MenuSetup_Click / MenuTuning_Click: ask for the password when "password_protect" is set.
    fn goto_view(&mut self, v: View) {
        let protected = self.settings.planner.get("password_protect").map(|s| s == "True").unwrap_or(false);
        if protected && matches!(v, View::Setup | View::Config) && self.settings.view != v {
            self.pw_prompt = Some((Some(v), String::new()));
        } else {
            self.settings.view = v;
        }
    }

    /// Password.GenerateSaltedHash: SHA-256 of the UTF-8 password followed by "MP", base64.
    pub fn password_hash(pw: &str) -> String {
        use base64::Engine;
        use sha2::Digest;
        let mut h = sha2::Sha256::new();
        h.update(pw.as_bytes());
        h.update(b"MP");
        base64::engine::general_purpose::STANDARD.encode(h.finalize())
    }

    fn password_ui(&mut self, ctx: &egui::Context) {
        let now = self.settings.planner.get("password_protect").map(|s| s == "True").unwrap_or(false);
        match self.pw_protect_seen {
            // CHK_Password_CheckedChanged: ticking the box asks for the new password
            Some(false) if now && self.pw_prompt.is_none() => self.pw_prompt = Some((None, String::new())),
            _ => {}
        }
        self.pw_protect_seen = Some(now);
        let Some((target, pw)) = self.pw_prompt.as_mut() else { return };
        let target = *target;
        let mut done = None;
        egui::Modal::new(egui::Id::new("pw_prompt")).show(ctx, |ui| {
            ui.label(egui::RichText::new("Enter Password").strong().color(Color32::WHITE));
            theme::label(ui, if target.is_some() { "Please enter your password" } else { "Please enter a password" });
            let r = ui.add(egui::TextEdit::singleline(pw).password(true).desired_width(220.0));
            r.request_focus();
            let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            ui.horizontal(|ui| {
                if theme::button(ui, "OK").clicked() || enter {
                    done = Some(true);
                }
                if theme::button(ui, "Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        let Some(ok) = done else { return };
        let (target, pw) = self.pw_prompt.take().unwrap_or_default();
        match target {
            None => {
                // Cancel on InputBox still stores the (empty) password, as MP does
                self.settings.planner.insert("password".into(), Self::password_hash(&pw));
            }
            Some(v) if ok => {
                if self.settings.planner.get("password").map(|h| *h == Self::password_hash(&pw)).unwrap_or(false) {
                    self.settings.view = v;
                } else {
                    self.toasts.error("Bad Password");
                }
            }
            Some(_) => {}
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
            self.goto_view(View::Config);
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
            if !crate::display::flag(&self.settings.planner, "autoHideMenuForce") {
                if ui.checkbox(&mut self.menu_hidden, crate::i18n::t("AutoHide")).clicked() {
                    self.settings.planner.insert("menu_autohide".into(), self.menu_hidden.to_string());
                    ui.close();
                }
            }
            let fs = ui.ctx().input(|i| i.viewport().fullscreen.unwrap_or(false));
            let mut full = fs;
            if ui.checkbox(&mut full, crate::i18n::t("Full Screen")).clicked() {
                ui.ctx().send_viewport_cmd(egui::ViewportCommand::Fullscreen(full));
                ui.close();
            }
            if ui.checkbox(&mut self.readonly, crate::i18n::t("Readonly")).clicked() {
                ui.close();
            }
            if ui.button(crate::i18n::t("Connection Options")).clicked() {
                self.conn_form = Some((self.settings.port.clone(), self.settings.baud));
                ui.close();
            }
            if ui.button(crate::i18n::t("Connection List")).clicked() {
                // connectionListToolStripMenuItem: one connection per line of a text file
                if let Some((_, text)) = platform::open_text("Connection List", "Connection list", &["txt", "*"]) {
                    for spec in parse_connection_list(&text) {
                        _actions.push(AppAction::ConnectExtra(spec));
                    }
                }
                ui.close();
            }
        });
        self.conn_form_ui(&resp.ctx, _actions);
    }

    /// ConnectionOptions: CMB_serialport (ports, TCP, UDP, UDPCl, WS), CMB_baudrate, Connect.
    fn conn_form_ui(&mut self, ctx: &egui::Context, actions: &mut Vec<AppAction>) {
        let Some((port, baud)) = self.conn_form.as_mut() else { return };
        let mut open = true;
        let mut go = false;
        egui::Window::new(crate::i18n::t("Connection Options")).open(&mut open).resizable(false).collapsible(false).show(ctx, |ui| {
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt("co_port").width(113.0).selected_text(port.clone()).show_ui(ui, |ui| {
                    let mut opts = comms::serial_ports();
                    opts.extend(["TCP".into(), "UDP".into(), "UDPCl".into(), "WS".into()]);
                    for o in opts {
                        ui.selectable_value(port, o.clone(), o);
                    }
                });
                egui::ComboBox::from_id_salt("co_baud").width(103.0).selected_text(baud.to_string()).show_ui(ui, |ui| {
                    for b in comms::BAUD_RATES {
                        ui.selectable_value(baud, *b, b.to_string());
                    }
                });
                if theme::button(ui, "Connect").clicked() {
                    go = true;
                }
            });
        });
        if go {
            let (port, baud) = self.conn_form.clone().unwrap_or_default();
            let st = &self.settings;
            let spec = match port.as_str() {
                "TCP" => Some(ConnectionSpec::Tcp { host: st.tcp_host.clone(), port: st.tcp_port }),
                "UDP" => Some(ConnectionSpec::UdpListen { port: st.udp_port }),
                "UDPCl" => Some(ConnectionSpec::UdpClient { host: st.udpcl_host.clone(), port: st.udpcl_port }),
                "WS" => Some(ConnectionSpec::WebSocket { url: st.ws_url.clone() }),
                p if p.starts_with("BLE") => Some(ConnectionSpec::Ble { address: st.ble_address.clone() }),
                p if p.starts_with("WinUSB") => Some(ConnectionSpec::WinUsb { path: st.winusb_path.clone() }),
                "" => None,
                p => Some(ConnectionSpec::Serial { port: p.to_string(), baud }),
            };
            match spec {
                Some(spec) => actions.push(AppAction::ConnectExtra(spec)),
                None => self.toasts.error("Select a serial port, TCP, UDP, UDPCl, WS, BLE or WinUSB"),
            }
        }
        if !open {
            self.conn_form = None;
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
    /// MainV2 closing: every plugin's Exit.
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        crate::plugins::shutdown();
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.drain_events();
        let mut actions = std::mem::take(&mut self.startup);
        let simulated = std::env::var_os("MP_AUTOSIM").is_some() || std::env::var_os("MP_SWARM").is_some();
        let ready = !simulated || self.link.as_ref().is_some_and(|l| l.shared.read().mavs.len() >= if std::env::var_os("MP_SWARM").is_some() { 3 } else { 1 });
        if ready {
            if let Some(tool) = self.startup_tool.take() {
                actions.push(AppAction::Tool(tool));
            }
        }

        self.hotkeys(&ctx, &mut actions);
        self.password_ui(&ctx);
        self.inspector.show(&ctx, self.link.as_ref(), &mut self.toasts);
        self.tools.ui(&ctx, self.link.as_ref(), &mut self.toasts);
        self.more_tools.ui(&ctx, self.link.as_ref(), &mut self.toasts);
        // PluginHost: CurrentState and config for the plugins; their commands and waypoints
        {
            let fields = self.link.as_ref().map(|l| l.shared.read().cs.fields()).unwrap_or_default();
            let rows = self.flight_plan.row_count();
            let (commands, waypoints) = crate::plugins::sync(&fields, &mut self.settings.planner, rows);
            if let Some(l) = &self.link {
                for (command, params) in commands {
                    l.send(mp_core::link::LinkCommand::CommandLong { command, params });
                }
            }
            for (command, p) in waypoints {
                self.flight_plan.add_command(command, p);
            }
        }
        let links: Vec<&Link> = self.link.iter().chain(self.extra_links.iter()).collect();
        self.more_tools.formation.ui(&ctx, &links, &mut self.toasts);
        self.more_tools.layout_editor.ui(&ctx, &links, &mut self.toasts);
        self.more_tools.follow_path.ui(&ctx, &links, &mut self.toasts, &mut actions);
        self.more_tools.wp_control.ui(&ctx, &links, &mut self.toasts);
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
            let near_top = ctx.input(|i| i.pointer.hover_pos().map(|p| p.y < self.menu_reveal_height.max(6.0)).unwrap_or(false));
            if near_top {
                let shown = egui::Panel::top("menu").frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| self.top_bar(ui, &mut actions));
                self.menu_reveal_height = shown.response.rect.max.y;
            } else {
                self.menu_reveal_height = 0.0;
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
                    extra_links,
                    ..
                } = self;
                let view = settings.view;
                if view != View::Setup { setup.close_terminal(); }
                if view != View::Config { config.leave_vehicle_tuning(); }
                let mut app = AppCtx {
                    link: link.as_ref(),
                    extra_links: extra_links.as_slice(),
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
                setup.dialogs(ui.ctx(), &mut app);
                flight_plan.update_kml(app.link, app.extra_links, app.settings);
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
        // hud.jpg / map.jpg / both.jpg clients: 5 Hz renderer frames while one is connected
        if self.capture.is_none() && crate::httpserver::frames_wanted() && self.stream_shot.elapsed() >= Duration::from_millis(200) {
            self.stream_shot = Instant::now();
            let ppp = ctx.pixels_per_point();
            ctx.request_screenshot(move |image| {
                std::thread::spawn(move || crate::httpserver::publish_screenshot(&image, ppp));
            });
        }
        if crate::httpserver::frames_wanted() {
            ctx.request_repaint_after(Duration::from_millis(200));
        }
        if let Some(capture) = &mut self.capture {
            let (connected, params_complete, parameters) = self.link.as_ref().map(|l| {
                let s = l.shared.read();
                (s.status == LinkStatus::Connected, s.params_complete, s.params.len())
            }).unwrap_or((false, false, 0));
            let simulated = std::env::var_os("MP_AUTOSIM").is_some() || std::env::var_os("MP_SWARM").is_some();
            capture.tick(&ctx, !simulated || (connected && params_complete), serde_json::json!({
                "view": format!("{:?}", self.settings.view), "connected": connected,
                "firmware": self.link.as_ref().map(|l| format!("{:?}", l.shared.read().cs.firmware)),
                "params_complete": params_complete, "parameters": parameters,
            }));
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, SETTINGS_KEY, &self.settings);
    }
}

/// connectionListToolStripMenuItem: `tcp://host:port`, `udp://host:port` (listen),
/// `udpcl://host:port` and `serial:PORT:baud`, one per line.
pub fn parse_connection_list(text: &str) -> Vec<ConnectionSpec> {
    let mut out = Vec::new();
    for line in text.lines().map(str::trim) {
        let hp = |rest: &str| rest.rsplit_once(':').and_then(|(h, p)| p.trim().parse::<u16>().ok().map(|p| (h.to_string(), p)));
        if let Some(r) = line.strip_prefix("tcp://") {
            if let Some((host, port)) = hp(r) {
                out.push(ConnectionSpec::Tcp { host, port });
            }
        } else if let Some(r) = line.strip_prefix("udpcl://") {
            if let Some((host, port)) = hp(r) {
                out.push(ConnectionSpec::UdpClient { host, port });
            }
        } else if let Some(r) = line.strip_prefix("udp://") {
            if let Some((_, port)) = hp(r) {
                out.push(ConnectionSpec::UdpListen { port });
            }
        } else if let Some(r) = line.strip_prefix("serial:") {
            if let Some((port, baud)) = r.rsplit_once(':').and_then(|(p, b)| b.trim().parse::<u32>().ok().map(|b| (p.to_string(), b))) {
                out.push(ConnectionSpec::Serial { port, baud });
            }
        } else if line.starts_with("ws://") || line.starts_with("wss://") {
            out.push(ConnectionSpec::WebSocket { url: line.to_string() });
        } else if let Some(r) = line.strip_prefix("ble:") {
            out.push(ConnectionSpec::Ble { address: r.to_string() });
        } else if let Some(r) = line.strip_prefix("winusb:") {
            out.push(ConnectionSpec::WinUsb { path: r.to_string() });
        }
    }
    out
}

#[cfg(test)]
mod connection_list_tests {
    #[test]
    fn parses_mp_connection_list() {
        let v = super::parse_connection_list("tcp://127.0.0.1:5760\nudp://0.0.0.0:14550\nudpcl://10.0.0.2:14551\nserial:COM5:57600\nws://127.0.0.1:8080/ws\nble:AA:BB:CC\nwinusb:DEV1\njunk\n");
        assert_eq!(v.len(), 7);
        assert_eq!(v[3].to_string().contains("COM5"), true);
    }
}
