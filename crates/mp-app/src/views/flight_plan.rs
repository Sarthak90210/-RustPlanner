//! Port of `GCSViews/FlightPlanner`, laid out from FlightPlanner.Designer.cs / .resx:
//! panelMap (MainMap + distance labels + zoom) | panelAction (Dock Right, 131px) over
//! panelWaypoints (Dock Bottom, 166px: settings row + Commands DataGridView).

use egui::{Color32, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use mp_core::firmware::Firmware;
use mp_core::geo::PointLatLngAlt;
use mp_core::link::{LinkCommand, LinkEvent};
use mp_core::mavcmd;
use mp_core::mission::{self, AltMode, Locationwp};

use crate::app::{AppCtx, Toasts};
use crate::views::flight_data::provider_combo;
use crate::{fonts, map, platform, theme};

pub(crate) mod menu;
mod map_controls;

const HOME_IDX: usize = usize::MAX;

/// Commands DataGridView columns: (header, width) from FlightPlanner.resx.
const COLS: [(&str, f32); 21] = [
    ("", 41.0),
    ("Command", 174.0),
    ("P1", 40.0),
    ("P2", 40.0),
    ("P3", 40.0),
    ("P4", 40.0),
    ("Lat", 70.0),
    ("Lon", 70.0),
    ("Alt", 30.0),
    ("Frame", 70.0),
    ("Zone", 57.0),
    ("Easting", 67.0),
    ("Northing", 72.0),
    ("MGRS", 64.0),
    ("Delete", 40.0),
    ("Up", 30.0),
    ("Down", 30.0),
    ("Grad %", 35.0),
    ("Angle", 35.0),
    ("Dist", 35.0),
    ("AZ", 30.0),
];
const COL_ZONE: usize = 10;
const COL_MGRS: usize = 13;
const COL_DELETE: usize = 14;
const COL_GRAD: usize = 17;
const COORD_SYSTEMS: [&str; 3] = ["GEO", "UTM", "MGRS"];

/// coords1_SystemChanged: Lat/Lon for GEO, Zone/Easting/Northing for UTM, MGRS for MGRS.
fn column_visible(col: usize, system: &str) -> bool {
    match col {
        6 | 7 => system == "GEO",
        10..=12 => system == "UTM",
        COL_MGRS => system == "MGRS",
        _ => true,
    }
}

/// The selected Coords system, saved as Settings "fpcoordmouse".
fn coord_system(app: &AppCtx) -> &'static str {
    let s = app.settings.planner.get("fpcoordmouse").map(String::as_str).unwrap_or("GEO");
    COORD_SYSTEMS.iter().copied().find(|c| *c == s).unwrap_or("GEO")
}

/// convertFromGeographic: Zone, Easting, Northing and MGRS cell text; blank at 0,0.
fn coord_cells(lat: f64, lng: f64) -> [String; 4] {
    if lat == 0.0 && lng == 0.0 {
        return Default::default();
    }
    let (zone, east, north) = mp_core::coords::to_utm_signed(lat, lng);
    let mgrs = mp_core::coords::geo_to_mgrs(lat, lng).unwrap_or_default();
    [zone.to_string(), format!("{east:.3}"), format!("{north:.3}"), mgrs]
}

/// Commands_CellEndEdit: convertFromUTM / convertFromMGRS. Unparsable input is ignored.
fn coords_from_cells(col: usize, cells: &[String; 4]) -> Option<(f64, f64)> {
    if col == COL_MGRS {
        let (lat, lng) = mp_core::coords::mgrs_to_geo(&cells[3])?;
        return (lat != 0.0 && lng != 0.0).then_some((lat, lng));
    }
    let zone: i32 = cells[0].trim().parse().ok()?;
    let east: f64 = cells[1].trim().parse().ok()?;
    let north: f64 = cells[2].trim().parse().ok()?;
    if east == 0.0 && north == 0.0 {
        return None;
    }
    Some(mp_core::coords::utm_signed_to_geo(zone, east, north))
}

pub struct FlightPlanView {
    home: Locationwp,
    items: Vec<Locationwp>,
    selected: Option<usize>,
    dragging: Option<usize>,
    firmware: Option<Firmware>,
    ctx_pos: Option<(f64, f64, Option<usize>)>,
    browser_open: bool,
    mouse: Option<(f64, f64)>,
    verify_height: bool,
    spline: bool,
    mavftp: bool,
    /// a MAVFtp mission transfer in progress; reports like the MAVLink protocol does
    ftp_job: Option<crossbeam_channel::Receiver<LinkEvent>>,
    grid: bool,
    alt_warn: f32,
    mission_type: usize,
    waypoints_collapsed: bool,
    wpfile: String,
    /// lists for the mission types not currently shown (index = MAV_MISSION_TYPE)
    stash: [Vec<Locationwp>; 3],
    polygon: Vec<(f64, f64)>,
    drawing_polygon: bool,
    survey_dialog: Option<SurveyParams>,
    grid_ui: Option<Box<super::survey_ui::GridUi>>,
    circle_dialog: Option<(f64, f64, String, String)>,
    /// Fence Circle Around Home: radius prompt
    fence_circle: Option<String>,
    menu_prompt: Option<menu::MenuPrompt>,
    kml_overlay: Vec<Vec<(f64, f64)>>,
    elevation_open: bool,
    /// Switch Docking: waypoint list docked right instead of bottom
    waypoints_right: bool,
    /// visible map area (min lat, min lng, max lat, max lng)
    view_bounds: Option<(f64, f64, f64, f64)>,
    measure: Option<(f64, f64)>,
    /// the UTM/MGRS cell being edited: (row, column, text)
    coord_edit: Option<(usize, usize, String)>,
    kml_server: Option<crate::kml_server::KmlServer>,
}

#[derive(Clone, Copy)]
struct SurveyParams {
    alt: f32,
    spacing: f32,
    angle: f32,
    overshoot: f32,
}

impl Default for SurveyParams {
    fn default() -> Self {
        Self { alt: 100.0, spacing: 50.0, angle: 0.0, overshoot: 0.0 }
    }
}

impl Default for FlightPlanView {
    fn default() -> Self {
        Self {
            home: Locationwp { id: mavcmd::WAYPOINT, ..Default::default() },
            items: Vec::new(),
            selected: None,
            dragging: None,
            firmware: None,
            ctx_pos: None,
            browser_open: false,
            mouse: None,
            verify_height: false,
            spline: false,
            mavftp: false,
            ftp_job: None,
            grid: false,
            alt_warn: 0.0,
            mission_type: 0,
            waypoints_collapsed: false,
            wpfile: "...".into(),
            stash: Default::default(),
            polygon: Vec::new(),
            drawing_polygon: false,
            survey_dialog: None,
            grid_ui: None,
            fence_circle: None,
            menu_prompt: None,
            kml_overlay: Vec::new(),
            elevation_open: false,
            waypoints_right: false,
            view_bounds: None,
            circle_dialog: None,
            measure: None,
            coord_edit: None,
            kml_server: None,
        }
    }
}

fn put_button(ui: &mut egui::Ui, r: Rect, text: &str) -> bool {
    ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, text, r.size())).clicked()
}

impl FlightPlanView {
    pub fn update_kml(&mut self, link: Option<&mp_core::link::Link>, extra: &[mp_core::link::Link]) {
        if let Some(server) = &self.kml_server {
            let mut links = Vec::new();
            if let Some(link) = link { links.push(link.shared.clone()); }
            links.extend(extra.iter().map(|l| l.shared.clone()));
            let mission = if self.mission_type == 0 { self.items.clone() } else { self.stash[0].clone() };
            server.update(crate::kml_server::Snapshot { home: self.home, mission, links });
        }
    }

    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        match e {
            LinkEvent::MissionDownloaded { mission_type: mt @ (1 | 2), items } => {
                *self.list_for(*mt as usize) = items.clone();
                toasts.info(format!("Got {} items", items.len()));
            }
            LinkEvent::MissionUploaded { mission_type: 1 | 2, result } => {
                if let Err(e) = result {
                    toasts.error(format!("Write failed: {e}"));
                }
            }
            LinkEvent::MissionDownloaded { mission_type: 0, items } => {
                if items.is_empty() {
                    toasts.error("No waypoints received");
                    return;
                }
                self.home = items[0];
                *self.list_for(0) = items[1..].to_vec();
                self.selected = None;
            }
            LinkEvent::MissionUploaded { mission_type: 0, result } => match result {
                Ok(()) => toasts.info("Done"),
                Err(e) => toasts.error(format!("Write failed: {e}")),
            },
            _ => {}
        }
    }

    fn fw(&self, app: &AppCtx) -> Firmware {
        app.link.filter(|_| app.connected()).map(|l| l.shared.read().cs.firmware).or(self.firmware).unwrap_or_default()
    }

    fn new_wp(&self, app: &AppCtx, id: u16, lat: f64, lng: f64) -> Locationwp {
        let mut alt = app.settings.default_alt;
        // Verify Height: keep the default height above the ground under this WP (relative alt)
        if self.verify_height && app.settings.alt_frame == 3 && (lat != 0.0 || lng != 0.0) {
            if let (Some(t), Some(h)) = (crate::srtm::altitude(lat, lng), crate::srtm::altitude(self.home.lat, self.home.lng)) {
                alt += (t - h) as f32;
            }
        }
        Locationwp { id, lat, lng, alt, frame: app.settings.alt_frame, ..Default::default() }
    }

    /// Read / Write a mission list: packed over MAVFtp when "MAVFTP" is ticked (falling back
    /// to the mission protocol if that fails), else the MAVLink mission protocol.
    pub(super) fn mission_io(&mut self, l: &mp_core::link::Link, mission_type: u8, read: bool, items: Vec<Locationwp>) {
        if !self.mavftp {
            l.send(if read { LinkCommand::ReadMission { mission_type } } else { LinkCommand::WriteMission { mission_type, items } });
            return;
        }
        let (tx, rx) = crossbeam_channel::bounded(1);
        let mut ftp = mp_core::ftp::Ftp::new(l);
        let link = l.sender();
        std::thread::spawn(move || {
            let ev = if read {
                mp_core::missionpck::read_with(&mut ftp, mission_type).map(|items| LinkEvent::MissionDownloaded { mission_type, items })
            } else {
                mp_core::missionpck::write_with(&mut ftp, mission_type, &items).map(|_| LinkEvent::MissionUploaded { mission_type, result: Ok(()) })
            };
            match ev {
                Ok(e) => {
                    let _ = tx.send(e);
                }
                Err(e) => {
                    log::warn!("mission over MAVFtp failed: {e}; using the mission protocol");
                    let _ = link.send(if read { LinkCommand::ReadMission { mission_type } } else { LinkCommand::WriteMission { mission_type, items } });
                }
            }
        });
        self.ftp_job = Some(rx);
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        if let Some(ev) = self.ftp_job.as_ref().and_then(|rx| rx.try_recv().ok()) {
            self.ftp_job = None;
            self.on_event(&ev, app.toasts);
        }
        if self.ftp_job.is_some() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));
        }
        // developer switch for screenshots: MP_SURVEY=1 opens Survey (Grid) on a sample polygon
        if self.polygon.is_empty() && std::env::var_os("MP_SURVEY").is_some() {
            let (lat, lng) = (-35.363261, 149.165230);
            self.polygon = vec![(lat + 0.0020, lng - 0.0030), (lat + 0.0024, lng + 0.0025), (lat - 0.0015, lng + 0.0032), (lat - 0.0022, lng - 0.0020)];
            let mut g = super::survey_ui::GridUi::new(self.polygon.clone(), (lat, lng), app.settings.fp_map.provider);
            if let Ok(t) = std::env::var("MP_SURVEY").map(|v| v.parse::<usize>().unwrap_or(0)) {
                g.set_tab(t);
            }
            self.grid_ui = Some(Box::new(g));
        }
        if self.home.lat == 0.0 && self.home.lng == 0.0 {
            if let Some(h) = app.link.and_then(|l| l.shared.read().cs.home) {
                self.home.lat = h.lat;
                self.home.lng = h.lng;
                self.home.alt = h.alt as f32;
            }
        }

        let wp_panel = if self.waypoints_right { egui::Panel::right("panelWaypointsR").default_size(520.0) } else { egui::Panel::bottom("panelWaypoints").default_size(166.0) };
        wp_panel
            .resizable(!self.waypoints_collapsed)
            .min_size(40.0)
            .frame(egui::Frame::new().fill(theme::bg()))
            .show(ui, |ui| self.waypoints_panel(ui, app));

        egui::Panel::right("panelAction")
            .resizable(false)
            .exact_size(131.0)
            .frame(egui::Frame::new().fill(theme::bg()))
            .show(ui, |ui| self.action_panel(ui, app));

        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| self.map_ui(ui, app));

        if self.browser_open {
            self.file_browser(ui.ctx(), app);
        }
        self.auto_wp_dialogs(ui.ctx(), app);
        self.menu_dialogs(ui.ctx(), app);
        if self.drawing_polygon {
            egui::Area::new(egui::Id::unique("poly_hint")).anchor(egui::Align2::CENTER_TOP, Vec2::new(0.0, 70.0)).show(ui.ctx(), |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        theme::label(ui, format!("Click the map to add polygon points ({})", self.polygon.len()));
                        if theme::button(ui, "Done").clicked() {
                            self.drawing_polygon = false;
                        }
                    });
                });
            });
        }
    }

    // ------------------------------------------------------------------ panelAction

    fn action_panel(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.max_rect().min;
        let f = fonts::ui_id(fonts::UI_SIZE);
        let at = |x: f32, y: f32| o + Vec2::new(x, y);
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(at(x, y), Vec2::new(w, h));
        let p = ui.painter().clone();
        let white = Color32::WHITE;

        // panel4 (3,3 127x55): coords1 (Vertical), the mouse location in the chosen system
        let (mlat, mlng) = self.mouse.unwrap_or((0.0, 0.0));
        let mut system = coord_system(app);
        ui.put(r(3.0, 5.0, 64.0, 20.0), |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_coordsystem")
                .width(56.0)
                .selected_text(system)
                .show_ui(ui, |ui| {
                    for c in COORD_SYSTEMS {
                        ui.selectable_value(&mut system, c, c);
                    }
                })
                .response
        });
        if system != coord_system(app) {
            app.settings.planner.insert("fpcoordmouse".into(), system.into());
        }
        let (alt, alt_source) = match self.mouse.and_then(|(la, ln)| crate::srtm::altitude(la, ln)) {
            Some(a) => (crate::units::alt(a as f32), "SRTM"),
            None => (0.0, if self.mouse.is_some() { "Invalid" } else { "" }),
        };
        let alt = format!("{alt:.2}{}", crate::units::alt_unit());
        let in_grid = (-80.0..=84.0).contains(&mlat) && mlng < 180.0 && mlng > -180.0;
        let (text, text_at, source_at) = match system {
            "UTM" => (
                in_grid.then(|| mp_core::coords::geo_to_utm(mlat, mlng)).flatten().map(|u| {
                    let s = mp_core::coords::utm_string(&u);
                    format!("{}\n{alt}", s.split(' ').collect::<Vec<_>>().join("\n"))
                }),
                at(70.0, 6.0),
                at(3.0, 33.0),
            ),
            "MGRS" => (
                in_grid.then(|| mp_core::coords::geo_to_mgrs(mlat, mlng)).flatten().map(|m| format!("{m}\n{alt}")),
                at(8.0, 30.0),
                at(71.0, 5.0),
            ),
            _ => (Some(format!("{mlat:.7}\n{mlng:.7}\n{alt}")), at(70.0, 6.0), at(3.0, 33.0)),
        };
        if let Some(text) = text {
            p.text(text_at, egui::Align2::LEFT_TOP, text, f.clone(), white);
            p.text(source_at, egui::Align2::LEFT_TOP, alt_source, f.clone(), white);
        }

        // panel3 (3,64 123x127)
        let px = 3.0;
        let py = 64.0;
        ui.put(r(px + 3.0, py + 3.0, 50.0, 17.0), egui::Checkbox::new(&mut self.grid, "Grid"));
        if ui.put(r(px + 62.0, py + 3.0, 60.0, 13.0), egui::Link::new("View KML")).clicked() {
            if self.kml_server.is_none() {
                match crate::kml_server::KmlServer::bind(56781, Default::default()) {
                    Ok(server) => self.kml_server = Some(server),
                    Err(e) => app.toasts.error(format!("Failed to open url http://127.0.0.1:56781/network.kml\n{e}")),
                }
            }
            self.update_kml(app.link, app.extra_links);
            if let Some(server) = &self.kml_server { ui.ctx().open_url(egui::OpenUrl::new_tab(server.network_url())); }
        }
        ui.put(r(px + 3.0, py + 22.0, 111.0, 21.0), |ui: &mut egui::Ui| {
            let mut prov = app.settings.fp_map.provider;
            provider_combo(ui, &mut prov, "comboBoxMapType");
            app.settings.fp_map.provider = prov;
            ui.allocate_response(Vec2::ZERO, Sense::hover())
        });
        p.text(at(px + 4.0, py + 46.0), egui::Align2::LEFT_TOP, "Status", f.clone(), white);
        if put_button(ui, r(px + 3.0, py + 68.0, 115.0, 23.0), "Inject Custom Map") {
            #[cfg(not(any(target_os = "android", target_os = "ios")))]
            if let Some(dir) = rfd::FileDialog::new().set_title("Inject Custom Map").pick_folder() {
                let counts = map::inject_custom_tiles(&platform::tile_cache_dir(), &dir);
                let total: usize = counts.values().sum();
                let mut msg = String::from("Number of tiles loaded per zoom : \n");
                for (z, n) in &counts {
                    msg.push_str(&format!("\nZoom {z} : {n}"));
                }
                msg.push_str(&format!("\n\n{total} tile{} loaded !", if total == 1 { "" } else { "s" }));
                if total > 0 {
                    app.settings.fp_map.provider = map::Provider::Custom;
                    app.settings.fd_map.provider = map::Provider::Custom;
                }
                app.toasts.error(msg);
            }
        }
        ui.put(r(px + 3.0, py + 97.0, 115.0, 23.0), |ui: &mut egui::Ui| {
            theme::progress(ui, 0.0, Vec2::new(115.0, 23.0), "");
            ui.allocate_response(Vec2::ZERO, Sense::hover())
        });

        // panel2 (3,197 123x75): Load File / Save File / lbl_wpfile
        let py = 197.0;
        if put_button(ui, r(px + 3.0, py + 3.0, 115.0, 23.0), "Load File") {
            if platform::HAS_NATIVE_DIALOGS {
                if let Some((name, text)) = platform::open_text("Open Waypoint File", "Waypoints", &["waypoints", "txt"]) {
                    self.load_text(&name, &text, app.toasts);
                }
            } else {
                self.browser_open = true;
            }
        }
        if put_button(ui, r(px + 3.0, py + 32.0, 115.0, 23.0), "Save File") {
            let text = mission::write_waypoint_file(&self.home, &self.items);
            match platform::save_text("Save Waypoint File", "mission.waypoints", "Waypoints", &["waypoints"], &text) {
                Some(Ok(path)) => {
                    self.wpfile = std::path::Path::new(&path).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or(path);
                }
                Some(Err(e)) => app.toasts.error(e),
                None => {}
            }
        }
        p.text(at(px + 3.0, py + 59.0), egui::Align2::LEFT_TOP, &self.wpfile, f.clone(), white);

        // panel5 (3,278 123x87): Read / Write / Write Fast
        let py = 278.0;
        let connected = app.connected();
        let read = put_button(ui, r(px + 3.0, py + 4.0, 115.0, 23.0), "Read");
        let write = put_button(ui, r(px + 3.0, py + 32.0, 115.0, 23.0), "Write") | put_button(ui, r(px + 3.0, py + 61.0, 115.0, 23.0), "Write Fast");
        if read || write {
            match app.link.filter(|_| connected) {
                None => app.toasts.error("Please Connect First"),
                Some(l) if read => self.mission_io(l, self.mission_type as u8, true, Vec::new()),
                Some(l) => {
                    // home is item 0 of the mission only; fence and rally lists are sent as-is
                    let mut all = if self.mission_type == 0 { vec![self.home] } else { Vec::new() };
                    all.extend(self.items.iter().copied());
                    self.mission_io(l, self.mission_type as u8, false, all);
                }
            }
        }

        // panel1 (3,371 123x89): Home Location
        let py = 371.0;
        p.text(at(px + 18.0, py + 3.0), egui::Align2::LEFT_TOP, "Home Location", f.clone(), white);
        p.text(at(px + 4.0, py + 22.0), egui::Align2::LEFT_TOP, "Lat", f.clone(), white);
        p.text(at(px + 3.0, py + 45.0), egui::Align2::LEFT_TOP, "Long", f.clone(), white);
        p.text(at(px + 3.0, py + 68.0), egui::Align2::LEFT_TOP, "ASL", f.clone(), white);
        ui.put(r(px + 47.0, py + 18.0, 65.0, 20.0), egui::DragValue::new(&mut self.home.lat).speed(0.00001).max_decimals(7));
        ui.put(r(px + 47.0, py + 42.0, 65.0, 20.0), egui::DragValue::new(&mut self.home.lng).speed(0.00001).max_decimals(7));
        ui.put(r(px + 47.0, py + 65.0, 65.0, 20.0), egui::DragValue::new(&mut self.home.alt).speed(0.5).max_decimals(2));

        if !connected {
            let py = 466.0;
            p.text(at(px + 3.0, py), egui::Align2::LEFT_TOP, "Vehicle type (offline)", f.clone(), white);
            ui.put(r(px + 3.0, py + 16.0, 115.0, 21.0), |ui: &mut egui::Ui| {
                let mut fw = self.firmware.unwrap_or_default();
                egui::ComboBox::from_id_salt("fwsel").width(107.0).selected_text(format!("{fw:?}")).show_ui(ui, |ui| {
                    for f in [Firmware::ArduCopter2, Firmware::ArduPlane, Firmware::ArduRover] {
                        ui.selectable_value(&mut fw, f, format!("{f:?}"));
                    }
                });
                self.firmware = Some(fw);
                ui.allocate_response(Vec2::ZERO, Sense::hover())
            });
        }
    }

    // ------------------------------------------------------------------ panelWaypoints

    fn waypoints_panel(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let full = ui.max_rect();
        let o = full.min;
        let f = fonts::ui_id(fonts::UI_SIZE);
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let p = ui.painter().clone();
        let white = Color32::WHITE;

        p.text(o + Vec2::new(9.0, 4.0), egui::Align2::LEFT_TOP, "WP Radius", f.clone(), white);
        // TXT_WPRad / TXT_loiterrad (multiplierdist) and TXT_DefaultAlt (multiplieralt) are in
        // display units; the settings keep metres
        let unit_box = |ui: &mut egui::Ui, rect: Rect, metres: &mut f32, m: f32, max: f32| {
            let mut shown = *metres * m;
            if ui.put(rect, egui::DragValue::new(&mut shown).range(0.0..=max * m).max_decimals(1)).changed() {
                *metres = shown / m;
            }
        };
        unit_box(ui, r(23.0, 20.0, 36.0, 20.0), &mut app.settings.wp_radius, crate::units::dist(1.0), 1000.0);
        p.text(o + Vec2::new(76.0, 4.0), egui::Align2::LEFT_TOP, "Loiter Radius", f.clone(), white);
        unit_box(ui, r(89.0, 20.0, 36.0, 20.0), &mut app.settings.loiter_radius, crate::units::dist(1.0), 10000.0);
        p.text(o + Vec2::new(151.0, 4.0), egui::Align2::LEFT_TOP, "Default Alt", f.clone(), white);
        unit_box(ui, r(154.0, 20.0, 40.0, 20.0), &mut app.settings.default_alt, crate::units::alt(1.0), 10000.0);
        ui.put(r(213.0, 7.0, 79.0, 21.0), |ui: &mut egui::Ui| {
            let cur = AltMode::from_frame(app.settings.alt_frame);
            egui::ComboBox::from_id_salt("CMB_altmode")
                .width(71.0)
                .selected_text(cur.label())
                .show_ui(ui, |ui| {
                    for m in AltMode::ALL {
                        if ui.selectable_label(cur == m, m.label()).clicked() {
                            app.settings.alt_frame = m as u8;
                        }
                    }
                })
                .response
        });
        ui.put(r(298.0, 13.0, 95.0, 17.0), egui::Checkbox::new(&mut self.verify_height, "Verify Height"));
        if put_button(ui, r(398.0, 8.0, 75.0, 23.0), "Add Below") {
            let at = self.selected.map(|i| i + 1).unwrap_or(self.items.len()).min(self.items.len());
            let mut wp = self.new_wp(app, mavcmd::WAYPOINT, 0.0, 0.0);
            if let Some(prev) = at.checked_sub(1).and_then(|i| self.items.get(i)) {
                wp.lat = prev.lat;
                wp.lng = prev.lng;
            }
            self.items.insert(at, wp);
            self.selected = Some(at);
        }
        p.text(o + Vec2::new(479.0, 4.0), egui::Align2::LEFT_TOP, "Alt Warn", f.clone(), white);
        ui.put(r(482.0, 20.0, 40.0, 20.0), egui::DragValue::new(&mut self.alt_warn));
        ui.put(r(528.0, 12.0, 60.0, 17.0), egui::Checkbox::new(&mut self.spline, "Spline"));
        // chk_usemavftp: Settings "UseMissionMAVFTP"
        self.mavftp = app.settings.planner.get("UseMissionMAVFTP").map(|v| v == "True").unwrap_or(false);
        if ui.put(r(589.0, 12.0, 70.0, 17.0), egui::Checkbox::new(&mut self.mavftp, "MAVFTP")).changed() {
            app.settings.planner.insert("UseMissionMAVFTP".into(), if self.mavftp { "True" } else { "False" }.into());
        }
        let min_r = Rect::from_min_size(Pos2::new(full.right() - 37.0, o.y), Vec2::new(37.0, 21.0));
        if put_button(ui, min_r, if self.waypoints_collapsed { "˄" } else { "˅" }) {
            self.waypoints_collapsed = !self.waypoints_collapsed;
        }
        if self.waypoints_collapsed {
            return;
        }
        // Commands grid at (3, 40), anchored on all sides
        let grid = Rect::from_min_max(o + Vec2::new(3.0, 40.0), full.max - Vec2::new(4.0, 0.0));
        ui.scope_builder(egui::UiBuilder::new().max_rect(grid), |ui| {
            egui::ScrollArea::both().auto_shrink([false, false]).show(ui, |ui| self.commands_grid(ui, app));
        });
    }

    fn commands_grid(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let fw = self.fw(app);
        let cmds = mavcmd::commands_for(fw);
        // headers follow the selected row's command (Commands_RowEnter -> ChangeColumnHeader)
        let header_cmd = self.selected.and_then(|i| self.items.get(i)).or(self.items.first()).map(|w| w.id).unwrap_or(mavcmd::WAYPOINT);
        let mut headers: Vec<String> = COLS.iter().map(|(h, _)| h.to_string()).collect();
        if let Some(l) = mavcmd::labels_for(fw, header_cmd) {
            for (k, col) in [2usize, 3, 4, 5, 6, 7, 8].iter().enumerate() {
                // cmdParamNames: "(unit)" appended for unitType alt / speed / dist
                headers[*col] = match unit_mult(&l.units[k]) {
                    Some((_, u)) if !l.labels[k].is_empty() => format!("{} ({u})", l.labels[k]),
                    _ => l.labels[k].clone(),
                };
            }
        }

        let mut derived = Vec::with_capacity(self.items.len());
        let mut prev = PointLatLngAlt::new(self.home.lat, self.home.lng, 0.0);
        let mut prev_alt = 0.0f64;
        for w in &self.items {
            if w.has_location() && (w.lat != 0.0 || w.lng != 0.0) {
                let pt = PointLatLngAlt::new(w.lat, w.lng, 0.0);
                let d = prev.distance(&pt);
                let az = prev.bearing(&pt);
                let dh = w.alt as f64 - prev_alt;
                let grad = if d > 0.0 { dh / d * 100.0 } else { 0.0 };
                derived.push(Some((grad, dh.atan2(d).to_degrees(), d, az)));
                prev = pt;
                prev_alt = w.alt as f64;
            } else {
                derived.push(None);
            }
        }

        let system = coord_system(app);
        let widths: Vec<f32> = COLS.iter().enumerate().map(|(i, c)| if column_visible(i, system) { c.1 } else { 0.0 }).collect();
        let total_w: f32 = widths.iter().sum();
        let (hdr_h, row_h) = (21.0, 22.0);
        let n = self.items.len();
        let (area, _) = ui.allocate_exact_size(Vec2::new(total_w.max(ui.available_width()), hdr_h + row_h * n as f32), Sense::hover());
        let p = ui.painter().clone();
        let f = fonts::ui_id(fonts::UI_SIZE);
        let grid_line = Color32::from_rgb(0xa0, 0xa0, 0xa0);
        p.rect_filled(area, 0.0, theme::bg());

        let mut x = area.left();
        for (i, w) in widths.iter().enumerate() {
            if *w == 0.0 {
                continue;
            }
            let r = Rect::from_min_size(Pos2::new(x, area.top()), Vec2::new(*w, hdr_h));
            p.rect_stroke(r, 0.0, Stroke::new(1.0, grid_line), StrokeKind::Inside);
            p.with_clip_rect(r).text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, &headers[i], f.clone(), Color32::WHITE);
            x += w;
        }

        let mut delete = None;
        let mut swap = None;
        let mut select = None;
        let up = app.assets.up.id();
        let down = app.assets.down.id();
        for i in 0..n {
            let y = area.top() + hdr_h + i as f32 * row_h;
            let row_bg = if self.selected == Some(i) {
                Color32::from_rgb(0x33, 0x99, 0xff)
            } else if i % 2 == 1 {
                theme::bg()
            } else {
                theme::control_bg()
            };
            let mut cells = Vec::with_capacity(COLS.len());
            let mut x = area.left();
            for w in &widths {
                cells.push(Rect::from_min_size(Pos2::new(x, y), Vec2::new(*w, row_h)));
                x += w;
            }
            // Alt Warn: altitude cell turns red when the WP is lower than this above the terrain
            let low = self.alt_warn > 0.0 && {
                let w = &self.items[i];
                w.has_location()
                    && match (crate::srtm::altitude(w.lat, w.lng), crate::srtm::altitude(self.home.lat, self.home.lng)) {
                        (Some(t), Some(h)) => {
                            let amsl = if w.frame == 0 { w.alt as f64 } else { h + w.alt as f64 };
                            amsl - t < self.alt_warn as f64
                        }
                        _ => false,
                    }
            };
            for (c, r) in cells.iter().enumerate().filter(|(_, r)| r.width() > 0.0) {
                p.rect_filled(*r, 0.0, if c == 0 { theme::bg() } else if c == 8 && low { Color32::RED } else { row_bg });
                p.rect_stroke(*r, 0.0, Stroke::new(1.0, grid_line), StrokeKind::Inside);
            }
            p.text(cells[0].center(), egui::Align2::CENTER_CENTER, (i + 1).to_string(), f.clone(), Color32::WHITE);
            if ui.interact(cells[0], ui.scope_id().with(("rowhdr", i)), Sense::click()).clicked() {
                select = Some(i);
            }

            let w = &mut self.items[i];
            ui.put(cells[1].shrink(1.0), |ui: &mut egui::Ui| {
                let resp = egui::ComboBox::from_id_salt(("cmd", i))
                    .width(cells[1].width() - 6.0)
                    .selected_text(mavcmd::cmd_name(w.id))
                    .show_ui(ui, |ui| {
                        for c in cmds {
                            if ui.selectable_label(false, &c.name).clicked() {
                                if let Some(id) = mavcmd::cmd_id(&c.name) {
                                    w.id = id;
                                }
                            }
                        }
                    })
                    .response;
                if resp.clicked() {
                    select = Some(i);
                }
                resp
            });
            let mut coord_clicked = false;
            let mut cell_edit = |ui: &mut egui::Ui, r: Rect, dv: egui::DragValue| {
                let mut child = ui.new_child(egui::UiBuilder::new().max_rect(r.shrink(1.0)));
                {
                    let v = child.visuals_mut();
                    for s in [&mut v.widgets.inactive, &mut v.widgets.hovered] {
                        s.bg_fill = Color32::TRANSPARENT;
                        s.weak_bg_fill = Color32::TRANSPARENT;
                        s.bg_stroke = Stroke::NONE;
                    }
                }
                if child.put(r.shrink(1.0), dv).clicked() {
                    select = Some(i);
                }
            };
            // cmdParamMultipliers of this row's command: cells show value * multiplier and
            // edits are stored divided by it
            let mults: [f32; 7] = mavcmd::labels_for(fw, w.id)
                .map(|l| std::array::from_fn(|k| unit_mult(&l.units[k]).map(|m| m.0).unwrap_or(1.0)))
                .unwrap_or([1.0; 7]);
            for (k, v) in [&mut w.p1, &mut w.p2, &mut w.p3, &mut w.p4].into_iter().enumerate() {
                let m = mults[k];
                let mut shown = *v * m;
                cell_edit(ui, cells[2 + k], egui::DragValue::new(&mut shown).speed(0.1).max_decimals(4));
                if shown != *v * m {
                    *v = shown / m;
                }
            }
            if system == "GEO" {
                cell_edit(ui, cells[6], egui::DragValue::new(&mut w.lat).speed(0.00001).fixed_decimals(7));
                cell_edit(ui, cells[7], egui::DragValue::new(&mut w.lng).speed(0.00001).fixed_decimals(7));
            } else {
                let shown = coord_cells(w.lat, w.lng);
                for c in (COL_ZONE..=COL_MGRS).filter(|c| column_visible(*c, system)) {
                    let k = c - COL_ZONE;
                    let editing = matches!(&self.coord_edit, Some((row, col, _)) if (*row, *col) == (i, c));
                    let mut text = match &self.coord_edit {
                        Some((_, _, t)) if editing => t.clone(),
                        _ => shown[k].clone(),
                    };
                    let resp = ui.put(
                        cells[c].shrink2(Vec2::new(2.0, 1.0)),
                        egui::TextEdit::singleline(&mut text)
                            .frame(egui::Frame::NONE)
                            .margin(Vec2::new(1.0, 0.0))
                            .vertical_align(egui::Align::Center)
                            .min_size(Vec2::new(0.0, cells[c].height() - 2.0))
                            .id(egui::Id::new(("coordcell", i, c)))
                            .desired_width(cells[c].width() - 4.0),
                    );
                    if resp.gained_focus() || resp.clicked() {
                        coord_clicked = true;
                    }
                    if resp.has_focus() {
                        self.coord_edit = Some((i, c, text));
                    } else if editing {
                        self.coord_edit = None;
                        if text != shown[k] {
                            let mut edited = shown.clone();
                            edited[k] = text;
                            if let Some((lat, lng)) = coords_from_cells(c, &edited) {
                                w.lat = lat;
                                w.lng = lng;
                            }
                        }
                    }
                }
            }
            {
                let m = mults[6];
                let mut shown = w.alt * m;
                cell_edit(ui, cells[8], egui::DragValue::new(&mut shown).speed(0.5).max_decimals(2));
                if shown != w.alt * m {
                    w.alt = shown / m;
                }
            }
            if coord_clicked {
                select = Some(i);
            }
            ui.put(cells[9].shrink(1.0), |ui: &mut egui::Ui| {
                let cur = AltMode::from_frame(w.frame);
                egui::ComboBox::from_id_salt(("frame", i))
                    .width(cells[9].width() - 6.0)
                    .selected_text(cur.label())
                    .show_ui(ui, |ui| {
                        for m in AltMode::ALL {
                            if ui.selectable_label(cur == m, m.label()).clicked() {
                                w.frame = m as u8;
                            }
                        }
                    })
                    .response
            });
            let dr = cells[COL_DELETE].shrink(2.0);
            if ui.put(dr, |ui: &mut egui::Ui| system_button(ui, "X", dr.size())).clicked() {
                delete = Some(i);
            }
            for (c, tex, dir) in [(COL_DELETE + 1, up, -1i32), (COL_DELETE + 2, down, 1)] {
                let ir = Rect::from_center_size(cells[c].center(), Vec2::splat(20.0));
                p.image(tex, ir, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
                if ui.interact(cells[c], ui.scope_id().with(("updown", i, c)), Sense::click()).clicked() {
                    let j = i as i32 + dir;
                    if j >= 0 && (j as usize) < n {
                        swap = Some((i, j as usize));
                    }
                }
            }
            if let Some((g, a, d, az)) = derived[i] {
                for (k, v) in [g, a, crate::units::dist(d as f32) as f64, az].iter().enumerate() {
                    let txt = if k >= 2 { format!("{v:.0}") } else { format!("{v:.1}") };
                    p.text(cells[COL_GRAD + k].left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, txt, f.clone(), Color32::WHITE);
                }
            }
        }
        if let Some(i) = select {
            self.selected = Some(i);
        }
        if let Some(i) = delete {
            self.items.remove(i);
            self.selected = None;
        }
        if let Some((a, b)) = swap {
            self.items.swap(a, b);
            self.selected = Some(b);
        }
    }

    // ------------------------------------------------------------------ panelMap

    fn map_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let cs = app.link.map(|l| l.shared.read().cs.clone());
        let home = self.home;
        let selected = self.selected;
        let wp_radius = app.settings.wp_radius;
        let assets = app.assets;
        let zoom = app.settings.fp_map.zoom;
        let bearing = app.settings.fp_map.bearing as f32;

        let mut add_at = None;
        let mut new_drag = self.dragging;
        let mut moved: Option<(usize, f64, f64)> = None;
        let mut select = None;
        let mut ctx_pos = None;
        let mut mouse = None;
        let items = if self.mission_type == 0 { self.items.clone() } else { self.stash[0].clone() };
        let fence = if self.mission_type == 1 { self.items.clone() } else { self.stash[1].clone() };
        let rally = if self.mission_type == 2 { self.items.clone() } else { self.stash[2].clone() };
        let poly = self.polygon.clone();
        let measure = self.measure;

        let kml = self.kml_overlay.clone();
        let pois = app.settings.pois.clone();
        let tracker = app.settings.tracker_home;
        let out = map::show(ui, &mut app.settings.fp_map, app.tiles, |_ui, view, resp, painter| {
            for line in &kml {
                let pts: Vec<Pos2> = line.iter().map(|&(a, b)| view.to_screen(a, b)).collect();
                painter.add(egui::Shape::line(pts, Stroke::new(2.0, Color32::from_rgb(0xff, 0x80, 0x00))));
            }
            for (a, b, n) in &pois {
                let p = view.to_screen(*a, *b);
                map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), p);
                painter.text(p + Vec2::new(10.0, -20.0), egui::Align2::LEFT_TOP, n, fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
            }
            if let Some((a, b)) = tracker {
                let p = view.to_screen(a, b);
                painter.circle_stroke(p, 8.0, Stroke::new(2.0, Color32::from_rgb(0x00, 0xbf, 0xff)));
                painter.text(p + Vec2::new(10.0, -6.0), egui::Align2::LEFT_TOP, "Tracker Home", fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
            }
            let mut screen: Vec<(usize, Pos2)> = Vec::new();
            if home.lat != 0.0 || home.lng != 0.0 {
                screen.push((HOME_IDX, view.to_screen(home.lat, home.lng)));
            }
            for (i, w) in items.iter().enumerate() {
                if w.has_location() && (w.lat != 0.0 || w.lng != 0.0) {
                    screen.push((i, view.to_screen(w.lat, w.lng)));
                }
            }
            let route: Vec<Pos2> = screen.iter().map(|(_, p)| *p).collect();
            if route.len() > 1 {
                painter.add(egui::Shape::line(route, Stroke::new(4.0, Color32::YELLOW)));
            }
            let mpp = view.metres_per_pixel() as f32;
            if wp_radius > 0.0 && mpp > 0.0 {
                for (i, p) in &screen {
                    if *i != HOME_IDX {
                        painter.circle_stroke(*p, wp_radius / mpp, Stroke::new(1.0, Color32::from_white_alpha(140)));
                    }
                }
            }
            let hover = resp.hover_pos();
            for (i, p) in &screen {
                if *i == HOME_IDX {
                    map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), *p);
                    painter.text(*p + Vec2::new(-5.0, -28.0), egui::Align2::LEFT_TOP, "H", fonts::ui_id(fonts::UI_SIZE), Color32::BLACK);
                } else {
                    let hov = hover.map(|h| (h - *p).length() < 16.0).unwrap_or(false);
                    map::draw_wp(painter, assets.marker_green.id(), assets.marker_shadow.id(), *p, &(i + 1).to_string(), selected == Some(*i), zoom, hov);
                }
            }
            // fence (red polygons/circles, return point), rally (R pins), drawn polygon
            draw_fence(painter, view, &fence);
            for (k, r) in rally.iter().enumerate() {
                let p = view.to_screen(r.lat, r.lng);
                map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), p);
                painter.text(p + Vec2::new(-5.0, -28.0), egui::Align2::LEFT_TOP, format!("R{}", k + 1), fonts::ui_id(fonts::UI_SIZE), Color32::BLACK);
            }
            if poly.len() > 1 {
                let mut pts: Vec<Pos2> = poly.iter().map(|&(a, b)| view.to_screen(a, b)).collect();
                pts.push(pts[0]);
                painter.add(egui::Shape::line(pts, Stroke::new(2.0, Color32::from_rgb(255, 0, 255))));
            }
            for &(a, b) in &poly {
                painter.circle_filled(view.to_screen(a, b), 5.0, Color32::from_rgb(255, 0, 255));
            }
            if let (Some((mlat, mlng)), Some(h)) = (measure, hover) {
                let p0 = view.to_screen(mlat, mlng);
                painter.line_segment([p0, h], Stroke::new(2.0, Color32::WHITE));
                let (hl, hg) = view.to_latlng(h);
                let d = PointLatLngAlt::new(mlat, mlng, 0.0).distance(&PointLatLngAlt::new(hl, hg, 0.0));
                painter.text(h + Vec2::new(10.0, 0.0), egui::Align2::LEFT_CENTER, format!("{d:.1} m"), fonts::ui_id(12.0), Color32::WHITE);
            }
            if let Some(cs) = &cs {
                if cs.lat != 0.0 || cs.lng != 0.0 {
                    map::draw_quad(painter, view.to_screen(cs.lat, cs.lng), cs.yaw - bearing, cs.sysid.max(1));
                }
            }

            // Overlay controls own their clicks and drags; they must not edit the mission.
            if _ui.input(|i| i.pointer.interact_pos()).is_some_and(|p| map_controls::over_control(view.rect, p)) {
                if !_ui.input(|i| i.pointer.primary_down()) {
                    new_drag = None;
                }
                return true;
            }

            // hit-test the pin body (tip at the point, 32x32 above it)
            let hit = |pos: Pos2| {
                screen
                    .iter()
                    .rev()
                    .find(|(_, p)| Rect::from_min_size(*p + Vec2::new(-15.0, -31.0), Vec2::splat(32.0)).contains(pos))
                    .map(|(i, _)| *i)
            };
            if let Some(h) = hover {
                mouse = Some(view.to_latlng(h));
            }
            if resp.drag_started_by(egui::PointerButton::Primary) {
                if let Some(pos) = resp.interact_pointer_pos() {
                    new_drag = hit(pos);
                }
            }
            if let Some(idx) = new_drag {
                if resp.dragged_by(egui::PointerButton::Primary) {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        let (lat, lng) = view.to_latlng(pos);
                        moved = Some((idx, lat, lng));
                    }
                }
                if resp.drag_stopped() {
                    new_drag = None;
                }
            }
            if resp.clicked_by(egui::PointerButton::Primary) {
                if let Some(pos) = resp.interact_pointer_pos() {
                    match hit(pos) {
                        Some(i) if i != HOME_IDX => select = Some(i),
                        Some(_) => {}
                        None => add_at = Some(view.to_latlng(pos)),
                    }
                }
            }
            if resp.secondary_clicked() {
                if let Some(pos) = resp.interact_pointer_pos() {
                    let (lat, lng) = view.to_latlng(pos);
                    ctx_pos = Some((lat, lng, hit(pos).filter(|i| *i != HOME_IDX)));
                }
            }
            new_drag.is_some()
        });
        {
            let r = out.view.rect;
            let corners = [r.left_top(), r.right_top(), r.right_bottom(), r.left_bottom()].map(|p| out.view.to_latlng(p));
            self.view_bounds = Some(corners.iter().fold((90.0_f64, 180.0_f64, -90.0_f64, -180.0_f64), |b, &(a, c)| {
                (b.0.min(a), b.1.min(c), b.2.max(a), b.3.max(c))
            }));
        }

        if mouse.is_some() {
            self.mouse = mouse;
        }
        self.dragging = new_drag;
        if let Some((idx, lat, lng)) = moved {
            if idx == HOME_IDX {
                self.home.lat = lat;
                self.home.lng = lng;
            } else if let Some(w) = self.items.get_mut(idx) {
                w.lat = lat;
                w.lng = lng;
            }
        }
        if let Some(i) = select {
            self.selected = Some(i);
        }
        if let (Some((lat, lng)), true) = (add_at, self.drawing_polygon) {
            self.polygon.push((lat, lng));
        } else if let (Some((lat, lng)), Some(_)) = (add_at, self.measure) {
            let _ = (lat, lng);
            self.measure = None;
        } else if let (Some((lat, lng)), 1 | 2) = (add_at, self.mission_type) {
            let id = if self.mission_type == 1 { 5001 } else { 5100 };
            let wp = Locationwp { id, lat, lng, alt: app.settings.default_alt, frame: if id == 5100 { 3 } else { 0 }, ..Default::default() };
            self.items.push(wp);
        } else if let Some((lat, lng)) = add_at {
            if self.home.lat == 0.0 && self.home.lng == 0.0 && self.items.is_empty() && !app.connected() {
                self.home.lat = lat;
                self.home.lng = lng;
            } else {
                let id = if self.spline { mavcmd::SPLINE_WAYPOINT } else { mavcmd::WAYPOINT };
                let wp = self.new_wp(app, id, lat, lng);
                self.items.push(wp);
                self.selected = Some(self.items.len() - 1);
            }
        }
        if ctx_pos.is_some() {
            self.ctx_pos = ctx_pos;
        }
        out.response.context_menu(|ui| self.context_menu(ui, app));
        self.map_controls(ui, out.view.rect, app);

        // overlays: lbl_distance / lbl_prevdist / lbl_homedist, cmb_missiontype, Zoom
        let mrect = out.view.rect;
        let o = mrect.min;
        let (w, h) = (mrect.width(), mrect.height());
        let p = ui.painter().with_clip_rect(mrect);
        let f = fonts::ui_id(fonts::UI_SIZE);
        let total = self.total_distance();
        let (prev, homed) = self.prev_and_home();
        for (y, t) in [
            (5.0, format!("Distance: {:.4} km", total / 1000.0)),
            (18.0, format!("Prev: {prev:.2} m")),
            (31.0, format!("Home: {homed:.2} m")),
        ] {
            let g = p.layout_no_wrap(t, f.clone(), Color32::WHITE);
            let r = Rect::from_min_size(o + Vec2::new(3.0, y), g.size());
            p.rect_filled(r, 0.0, theme::bg());
            p.galley(r.min, g, Color32::WHITE);
        }
        ui.put(Rect::from_min_size(o + Vec2::new(w - 177.0, 3.0), Vec2::new(121.0, 21.0)), |ui: &mut egui::Ui| {
            let names = ["Mission", "Fence", "Rally"];
            let mut to = self.mission_type;
            let r = egui::ComboBox::from_id_salt("cmb_missiontype")
                .width(113.0)
                .selected_text(names[self.mission_type])
                .show_ui(ui, |ui| {
                    for (i, n) in names.iter().enumerate() {
                        ui.selectable_value(&mut to, i, *n);
                    }
                })
                .response;
            self.switch_type(to);
            r
        });
        p.text(o + Vec2::new(w - 45.0, 5.0), egui::Align2::LEFT_TOP, "Zoom", f.clone(), Color32::BLACK);
        ui.put(
            Rect::from_min_size(o + Vec2::new(w - 49.0, 25.0), Vec2::new(45.0, 20.0)),
            egui::DragValue::new(&mut app.settings.fp_map.zoom).speed(0.1).range(2.0..=22.0).max_decimals(1),
        );
        theme::vtrackbar(ui, Rect::from_min_size(o + Vec2::new(w - 50.0, 42.0), Vec2::new(45.0, (h - 48.0).max(40.0))), &mut app.settings.fp_map.zoom, 2.0, 22.0);
    }

    fn prev_and_home(&self) -> (f64, f64) {
        let is_loc = |w: &Locationwp| w.has_location() && (w.lat != 0.0 || w.lng != 0.0);
        let idx = self.selected.filter(|i| self.items.get(*i).map(is_loc).unwrap_or(false)).or_else(|| self.items.iter().rposition(is_loc));
        let Some(i) = idx else { return (0.0, 0.0) };
        let w = &self.items[i];
        let here = PointLatLngAlt::new(w.lat, w.lng, 0.0);
        let prev = self.items[..i]
            .iter()
            .rev()
            .find(|w| is_loc(w))
            .map(|w| PointLatLngAlt::new(w.lat, w.lng, 0.0))
            .unwrap_or(PointLatLngAlt::new(self.home.lat, self.home.lng, 0.0));
        let home = PointLatLngAlt::new(self.home.lat, self.home.lng, 0.0);
        (prev.distance(&here), home.distance(&here))
    }

    fn total_distance(&self) -> f64 {
        let mut prev = PointLatLngAlt::new(self.home.lat, self.home.lng, 0.0);
        let mut total = 0.0;
        for w in &self.items {
            if w.has_location() && (w.lat != 0.0 || w.lng != 0.0) {
                let p = PointLatLngAlt::new(w.lat, w.lng, 0.0);
                if !prev.is_zero() {
                    total += prev.distance(&p);
                }
                prev = p;
            }
        }
        total
    }

    fn load_text(&mut self, name: &str, text: &str, toasts: &mut Toasts) {
        match mission::read_waypoint_file(text) {
            Ok(list) if !list.is_empty() => {
                self.home = list[0];
                self.items = list[1..].to_vec();
                self.selected = None;
                self.wpfile = name.to_string();
            }
            Ok(_) => toasts.error("Invalid Waypoint file"),
            Err(e) => toasts.error(e.to_string()),
        }
    }

    /// The list for a mission type (the active one lives in `items`).
    fn list_for(&mut self, mt: usize) -> &mut Vec<Locationwp> {
        if mt == self.mission_type {
            &mut self.items
        } else {
            &mut self.stash[mt]
        }
    }

    fn fence_list(&mut self) -> &mut Vec<Locationwp> {
        self.list_for(1)
    }

    fn rally_list(&mut self) -> &mut Vec<Locationwp> {
        self.list_for(2)
    }

    fn switch_type(&mut self, to: usize) {
        if to != self.mission_type {
            let cur = std::mem::take(&mut self.items);
            self.stash[self.mission_type] = cur;
            self.items = std::mem::take(&mut self.stash[to]);
            self.mission_type = to;
            self.selected = None;
        }
    }

    fn add_fence_polygon(&mut self, cmd: u16, app: &mut AppCtx) {
        if self.polygon.len() < 3 {
            app.toasts.error("Please define a polygon.");
            return;
        }
        let n = self.polygon.len() as f32;
        let pts = self.polygon.clone();
        for (lat, lng) in pts {
            self.fence_list().push(Locationwp { id: cmd, p1: n, lat, lng, frame: 0, ..Default::default() });
        }
    }

    /// Survey (Grid) / Create WP Circle dialogs.
    fn auto_wp_dialogs(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if let Some(g) = self.grid_ui.as_mut() {
            match g.show(ctx, app) {
                super::survey_ui::GridResult::Open => {}
                super::survey_ui::GridResult::Cancel => self.grid_ui = None,
                super::survey_ui::GridResult::Accept(list) => {
                    self.switch_type(0);
                    self.items.extend(list);
                    self.grid_ui = None;
                }
            }
        }
        let mut close_survey = false;
        let mut accept_survey = None;
        if let Some(sp) = self.survey_dialog.as_mut() {
            egui::Window::new("SimpleGrid").collapsible(false).resizable(false).show(ctx, |ui| {
                egui::Grid::new("survey").num_columns(2).show(ui, |ui| {
                    theme::label(ui, "Altitude (m)");
                    ui.add(egui::DragValue::new(&mut sp.alt).range(1.0..=10000.0));
                    ui.end_row();
                    theme::label(ui, "Distance between lines (m)");
                    ui.add(egui::DragValue::new(&mut sp.spacing).range(1.0..=5000.0));
                    ui.end_row();
                    theme::label(ui, "Angle (deg)");
                    ui.add(egui::DragValue::new(&mut sp.angle).range(0.0..=360.0));
                    ui.end_row();
                    theme::label(ui, "Overshoot (m)");
                    ui.add(egui::DragValue::new(&mut sp.overshoot).range(0.0..=500.0));
                    ui.end_row();
                });
                let wps = mp_core::survey::grid(&self.polygon, sp.spacing as f64, sp.angle as f64, sp.overshoot as f64);
                let area = mp_core::survey::area(&self.polygon);
                theme::label(ui, format!("Area: {area:.0} m2   Waypoints: {}", wps.len()));
                ui.horizontal(|ui| {
                    if theme::button(ui, "Accept").clicked() {
                        accept_survey = Some((wps, sp.alt));
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        close_survey = true;
                    }
                });
            });
        }
        if let Some((wps, alt)) = accept_survey {
            self.switch_type(0);
            for (lat, lng) in wps {
                let mut wp = self.new_wp(app, mavcmd::WAYPOINT, lat, lng);
                wp.alt = alt;
                self.items.push(wp);
            }
            close_survey = true;
        }
        if close_survey {
            self.survey_dialog = None;
        }

        // FENCE_CIRCLE_INCLUSION centred on home with the entered radius
        let mut fc_done = None;
        if let Some(radius) = self.fence_circle.as_mut() {
            egui::Window::new("Radius").collapsible(false).resizable(false).show(ctx, |ui| {
                theme::label(ui, "Radius (m)");
                ui.text_edit_singleline(radius);
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() {
                        fc_done = Some(radius.trim().parse::<f32>().ok());
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        fc_done = Some(None);
                    }
                });
            });
        }
        if let Some(r) = fc_done {
            self.fence_circle = None;
            if let Some(r) = r.filter(|r| *r > 0.0) {
                let (lat, lng) = (self.home.lat, self.home.lng);
                if lat == 0.0 && lng == 0.0 {
                    app.toasts.error("Home location is not set");
                } else {
                    self.fence_list().push(Locationwp { id: 5003, p1: r, lat, lng, frame: 0, ..Default::default() });
                }
            }
        }
        let mut close_circle = false;
        let mut accept_circle = None;
        if let Some((lat, lng, radius, points)) = self.circle_dialog.as_mut() {
            egui::Window::new("Create WP Circle").collapsible(false).resizable(false).show(ctx, |ui| {
                theme::label(ui, "Radius (m)");
                ui.text_edit_singleline(radius);
                theme::label(ui, "Points");
                ui.text_edit_singleline(points);
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() {
                        if let (Ok(r), Ok(n)) = (radius.trim().parse::<f64>(), points.trim().parse::<usize>()) {
                            accept_circle = Some(mp_core::survey::circle((*lat, *lng), r, n));
                        }
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        close_circle = true;
                    }
                });
            });
        }
        if let Some(pts) = accept_circle {
            self.switch_type(0);
            for (lat, lng) in pts {
                let wp = self.new_wp(app, mavcmd::WAYPOINT, lat, lng);
                self.items.push(wp);
            }
            close_circle = true;
        }
        if close_circle {
            self.circle_dialog = None;
        }
    }

    fn file_browser(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let mut open = true;
        let mut load = None;
        egui::Window::new("Open Waypoint File").open(&mut open).collapsible(false).show(ctx, |ui| {
            let files = platform::list_files(&["waypoints", "txt"]);
            if files.is_empty() {
                ui.label("No .waypoints files saved yet.");
            }
            for f in files {
                let name = f.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                if ui.button(&name).clicked() {
                    load = Some((name, f));
                }
            }
        });
        if let Some((name, path)) = load {
            if let Ok(text) = std::fs::read_to_string(path) {
                self.load_text(&name, &text, app.toasts);
            }
            self.browser_open = false;
        }
        if !open {
            self.browser_open = false;
        }
    }
}

/// Plain WinForms system button, as drawn in a DataGridViewButtonColumn cell.
fn system_button(ui: &mut egui::Ui, text: &str, size: Vec2) -> egui::Response {
    let (r, resp) = ui.allocate_exact_size(size, Sense::click());
    let p = ui.painter();
    let fill = if resp.hovered() { Color32::from_rgb(0xe5, 0xf1, 0xfb) } else { Color32::from_rgb(0xe1, 0xe1, 0xe1) };
    p.rect_filled(r, 0.0, fill);
    p.rect_stroke(r, 0.0, Stroke::new(1.0, Color32::from_rgb(0xad, 0xad, 0xad)), StrokeKind::Inside);
    p.text(r.center(), egui::Align2::CENTER_CENTER, text, fonts::ui_id(fonts::UI_SIZE), Color32::BLACK);
    resp
}

/// Fence items: polygon vertex runs (p1 = vertex count), circles (p1 = radius), return point.
fn draw_fence(painter: &egui::Painter, view: &map::MapView, fence: &[Locationwp]) {
    let mut i = 0;
    let mpp = view.metres_per_pixel() as f32;
    while i < fence.len() {
        let f = fence[i];
        match f.id {
            5001 | 5002 => {
                let n = (f.p1 as usize).max(1).min(fence.len() - i);
                let mut pts: Vec<Pos2> = fence[i..i + n].iter().map(|w| view.to_screen(w.lat, w.lng)).collect();
                if pts.len() > 1 {
                    pts.push(pts[0]);
                }
                let col = if f.id == 5001 { Color32::from_rgb(0, 200, 0) } else { Color32::RED };
                painter.add(egui::Shape::line(pts, Stroke::new(3.0, col)));
                i += n;
            }
            5003 | 5004 => {
                let col = if f.id == 5003 { Color32::from_rgb(0, 200, 0) } else { Color32::RED };
                if mpp > 0.0 {
                    painter.circle_stroke(view.to_screen(f.lat, f.lng), f.p1 / mpp, Stroke::new(3.0, col));
                }
                i += 1;
            }
            5000 => {
                let p = view.to_screen(f.lat, f.lng);
                painter.circle_filled(p, 6.0, Color32::from_rgb(255, 165, 0));
                painter.text(p + Vec2::new(8.0, 0.0), egui::Align2::LEFT_CENTER, "Return", fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
                i += 1;
            }
            _ => i += 1,
        }
    }
}

/// unitType -> (CurrentState multiplier, unit name).
fn unit_mult(unit: &str) -> Option<(f32, &'static str)> {
    match unit {
        "alt" => Some((crate::units::alt(1.0), crate::units::alt_unit())),
        "speed" => Some((crate::units::speed(1.0), crate::units::speed_unit())),
        "dist" => Some((crate::units::dist(1.0), crate::units::dist_unit())),
        _ => None,
    }
}

#[cfg(test)]
mod coord_tests {
    use super::*;
    use crate::{app::Settings, assets::Assets, joystick::Joystick, map::TileCache};

    struct Harness {
        ctx: egui::Context,
        view: FlightPlanView,
        settings: Settings,
        tiles: TileCache,
        assets: Assets,
        joystick: Joystick,
        toasts: crate::app::Toasts,
        labels: Vec<(String, Pos2)>,
    }

    impl Harness {
        fn new() -> Self {
            let ctx = egui::Context::default();
            let assets = Assets::load(&ctx);
            let mut view = FlightPlanView::default();
            view.items = vec![Locationwp { id: mavcmd::WAYPOINT, lat: -35.363261, lng: 149.165230, alt: 100.0, frame: 3, ..Default::default() }];
            let mut h = Self { ctx, assets, view, settings: Settings::default(), tiles: TileCache::offline(), joystick: Joystick::default(), toasts: Default::default(), labels: Vec::new() };
            h.frame(Vec::new());
            h.frame(Vec::new());
            h
        }

        fn frame(&mut self, events: Vec<egui::Event>) {
            let input = egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400.0, 400.0))), events, ..Default::default() };
            let mut actions = Vec::new();
            let mut out = self.ctx.run_ui(input, |ui| {
                let mut app = AppCtx {
                    link: None,
                    extra_links: &[],
                    tiles: &mut self.tiles,
                    toasts: &mut self.toasts,
                    settings: &mut self.settings,
                    actions: &mut actions,
                    sim_running: false,
                    assets: &self.assets,
                    readonly: false,
                    joystick: &mut self.joystick,
                };
                ui.scope_builder(egui::UiBuilder::new().max_rect(Rect::from_min_size(Pos2::ZERO, Vec2::new(130.0, 200.0))), |ui| self.view.action_panel(ui, &mut app));
                ui.scope_builder(egui::UiBuilder::new().max_rect(Rect::from_min_size(Pos2::new(0.0, 220.0), Vec2::new(1400.0, 180.0))), |ui| self.view.commands_grid(ui, &mut app));
            });
            out.textures_delta.clear();
            fn walk(shape: &egui::Shape, out: &mut Vec<(String, Pos2)>) {
                match shape {
                    egui::Shape::Text(t) => out.push((t.galley.text().to_string(), t.pos + t.galley.size() / 2.0)),
                    egui::Shape::Vec(v) => v.iter().for_each(|s| walk(s, out)),
                    _ => {}
                }
            }
            self.labels.clear();
            for s in &out.shapes {
                walk(&s.shape, &mut self.labels);
            }
        }

        fn label(&self, text: &str) -> Option<Pos2> {
            self.labels.iter().find(|(t, _)| t == text).map(|l| l.1)
        }

        fn click(&mut self, pos: Pos2) {
            for pressed in [true, false] {
                self.frame(vec![egui::Event::PointerMoved(pos), egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE }]);
            }
            self.frame(Vec::new());
        }

        fn choose_system(&mut self, current: &str, system: &str) {
            let combo = self.label(current).expect("coordinate selector");
            self.click(combo);
            let item = self.labels.iter().rev().find(|(t, _)| t == system).expect("system option").1;
            self.click(item);
            assert_eq!(self.settings.planner.get("fpcoordmouse").map(String::as_str), Some(system));
        }

        /// Replace a grid cell's text below `header` and end the edit with Enter.
        fn edit_cell(&mut self, header: &str, text: &str) {
            // the grid starts at y = 220; the selector above may show the same text
            let h = self.labels.iter().find(|(t, p)| t == header && p.y > 220.0).map(|l| l.1).unwrap_or_else(|| panic!("{header} column hidden: {:?}", self.labels));
            self.click(h + Vec2::new(0.0, 22.0));
            let key = |key, modifiers| egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers };
            self.frame(vec![key(egui::Key::A, egui::Modifiers::COMMAND)]);
            self.frame(vec![egui::Event::Text(text.into())]);
            self.frame(vec![key(egui::Key::Enter, egui::Modifiers::NONE)]);
            self.frame(Vec::new());
        }
    }

    #[test]
    fn utm_and_mgrs_columns_switch_and_edits_convert_back_to_lat_lng() {
        let mut h = Harness::new();
        assert!(h.label("Zone").is_none() && h.label("MGRS").is_none());
        assert!(h.label("-35.3632610\n149.1652300\n0.00m").is_none(), "mouse location starts at 0,0");
        assert!(h.label("0.0000000\n0.0000000\n0.00m").is_some(), "{:?}", h.labels);

        h.choose_system("GEO", "UTM");
        for header in ["Zone", "Easting", "Northing"] {
            assert!(h.label(header).is_some(), "{header}");
        }
        assert!(h.label("MGRS").is_none());
        let origin = mp_core::coords::utm_string(&mp_core::coords::geo_to_utm(0.0, 0.0).unwrap()).replace(' ', "\n");
        assert!(origin.starts_with("31N\n166021.4"));
        assert!(h.label(&format!("{origin}\n0.00m")).is_some(), "UTM mouse display: {:?}", h.labels);
        let (zone, east, north) = mp_core::coords::to_utm_signed(-35.363261, 149.165230);
        assert_eq!(zone, -55);
        h.edit_cell("Easting", &format!("{:.3}", east + 1000.0));
        let (lat, lng) = mp_core::coords::utm_signed_to_geo(zone, east + 1000.0, north);
        let w = h.view.items[0];
        assert!((w.lat - lat).abs() < 1e-9 && (w.lng - lng).abs() < 1e-9, "{} {}", w.lat, w.lng);
        assert!(w.lng > 149.17);
        h.edit_cell("Northing", "not a number");
        assert_eq!((h.view.items[0].lat, h.view.items[0].lng), (w.lat, w.lng), "invalid UTM is ignored");

        h.choose_system("UTM", "MGRS");
        assert!(h.label("Zone").is_none() && h.labels.iter().any(|(t, p)| t == "MGRS" && p.y > 220.0));
        let target = mp_core::coords::geo_to_mgrs(-35.36, 149.16).unwrap();
        h.edit_cell("MGRS", &target);
        let (lat, lng) = mp_core::coords::mgrs_to_geo(&target).unwrap();
        assert_eq!((h.view.items[0].lat, h.view.items[0].lng), (lat, lng));

        h.choose_system("MGRS", "GEO");
        assert!(h.label("MGRS").is_none() && h.label("Zone").is_none() && h.label("GEO").is_some());
    }
}
