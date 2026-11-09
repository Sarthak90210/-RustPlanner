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
    // Up.HeaderText / Down.HeaderText are empty in FlightPlanner.resx
    ("", 30.0),
    ("", 30.0),
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
    // temp2[0].ToString("0.000") in the current culture; a ProjNet exception leaves the cells
    // blank here (the C# catch keeps whatever they held)
    let fmt = |v: f64| if v.is_nan() { String::new() } else { mp_core::dotnet::format_fixed(v, 1, 3, mp_core::dotnet::current_culture()) };
    [zone.to_string(), fmt(east), fmt(north), mgrs]
}

/// Commands_CellEndEdit: convertFromUTM / convertFromMGRS. Unparsable input is ignored.
fn coords_from_cells(col: usize, cells: &[String; 4]) -> Option<(f64, f64)> {
    if col == COL_MGRS {
        let (lat, lng) = mp_core::coords::mgrs_to_geo(&cells[3])?;
        return (lat != 0.0 && lng != 0.0).then_some((lat, lng));
    }
    // int.Parse / double.Parse in the current culture; an exception leaves Lat/Lon unchanged
    let zone: i32 = mp_core::dotnet::int_parse(&cells[0]).ok()?;
    let east: f64 = mp_core::dotnet::double_parse(&cells[1]).ok()?;
    let north: f64 = mp_core::dotnet::double_parse(&cells[2]).ok()?;
    if east == 0.0 && north == 0.0 {
        return None;
    }
    mp_core::coords::utm_signed_to_geo(zone, east, north)
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
    /// BUT_write_Click / but_writewpfast_Click checks, resumed after each prompt
    write_check: Option<WriteCheck>,
    /// BUT_read_Click: "This will clear your existing points, Continue?"
    read_confirm: bool,
    /// saveWPsFast running on its own thread
    fast_job: Option<crossbeam_channel::Receiver<Result<bool, String>>>,
    /// MAV.cs.HomeLocation, refreshed every frame for updateUndoBuffer
    vehicle_home: Option<Locationwp>,
    /// a protocol upload finished: saveWPs' home request and radius parameters are due
    post_upload: Option<std::time::Instant>,
    /// FlightPlanner.history: home and list before each undoable change
    history: Vec<(Locationwp, Vec<Locationwp>)>,
    /// a marker drag records one undo step when it starts
    drag_undo: bool,
    grid: bool,
    alt_warn: f32,
    mission_type: usize,
    waypoints_collapsed: bool,
    wpfile: String,
    /// FlightPlanner.wpfilename: the last waypoint file loaded (the save dialog's name)
    wpfilename: String,
    /// lists for the mission types not currently shown (index = MAV_MISSION_TYPE)
    stash: [Vec<Locationwp>; 3],
    polygon: Vec<(f64, f64)>,
    drawing_polygon: bool,
    /// ExtLibs/SimpleGrid's GridUI form (Auto WP > SimpleGrid)
    simple_grid: Option<Box<crate::extlibs::simplegrid::GridUI>>,
    grid_ui: Option<Box<super::survey_ui::GridUi>>,
    circle_dialog: Option<(f64, f64, String, String)>,
    /// Fence Circle Around Home: radius prompt
    fence_circle: Option<String>,
    menu_prompt: Option<menu::MenuPrompt>,
    /// a context-menu tool's InputBox sequence
    input_chain: Option<menu::InputChain>,
    /// FlightPlanner.zone, the last UTM zone typed (static "50s")
    utm_zone: String,
    /// Measure Dist's MessageShowAgain
    fp_show_again: Option<crate::show_again::ShowAgain>,
    kml_overlay: Vec<Vec<(f64, f64)>>,
    elevation_open: bool,
    /// Switch Docking: waypoint list docked right instead of bottom
    waypoints_right: bool,
    /// visible map area (min lat, min lng, max lat, max lng)
    view_bounds: Option<(f64, f64, f64, f64)>,
    measure: Option<(f64, f64)>,
    /// the UTM/MGRS cell being edited: (row, column, text)
    coord_edit: Option<(usize, usize, String)>,
    /// MainMap_OnCurrentPositionChanged also moves coords1 to the map centre
    last_center: Option<(f64, f64)>,
    /// currentMarker: the last pointer position over the map (SetMouseDisplay)
    last_pointer: Option<(f64, f64)>,
    /// wpOverlay's routes as writeKML last built them, with the inputs they were built from
    /// (writeKML runs when the list changes, not on every repaint)
    wp_overlay_cache: Option<(WpOverlayKey, Vec<mp_core::extlibs::maps::wpoverlay::GMapRoute>)>,
    /// processToScreen's "Reset Home to loaded coords" YesNo box: the loaded home row
    reset_home_prompt: Option<Locationwp>,
    /// a downloaded list (mission type, items; a mission has home first) waiting for
    /// WPtoScreen's processToScreen
    pending_download: Option<(usize, Vec<Locationwp>)>,
    /// the mission type of a read the planner started (BUT_read_Click -> getWPs)
    awaiting_read: Option<u8>,
    /// a BUT_read_Click from outside the panel, run on the next frame
    read_pending: bool,
}

/// What writeKML's `wpOverlay.CreateOverlay` depends on.
#[derive(Clone, PartialEq)]
struct WpOverlayKey {
    home: Locationwp,
    items: Vec<Locationwp>,
    wp_radius: f32,
    loiter_radius: f32,
    alt_frame: u8,
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
            write_check: None,
            read_confirm: false,
            fast_job: None,
            history: Vec::new(),
            post_upload: None,
            vehicle_home: None,
            drag_undo: false,
            grid: false,
            alt_warn: 0.0,
            mission_type: 0,
            waypoints_collapsed: false,
            wpfile: "...".into(),
            wpfilename: String::new(),
            stash: Default::default(),
            polygon: Vec::new(),
            drawing_polygon: false,
            simple_grid: None,
            grid_ui: None,
            fence_circle: None,
            menu_prompt: None,
            input_chain: None,
            utm_zone: "50s".into(),
            fp_show_again: None,
            kml_overlay: Vec::new(),
            elevation_open: false,
            waypoints_right: false,
            view_bounds: None,
            circle_dialog: None,
            measure: None,
            coord_edit: None,
            last_center: None,
            last_pointer: None,
            wp_overlay_cache: None,
            reset_home_prompt: None,
            pending_download: None,
            awaiting_read: None,
            read_pending: false,
        }
    }
}

fn put_button(ui: &mut egui::Ui, r: Rect, text: &str) -> bool {
    ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, text, r.size())).clicked()
}

impl FlightPlanView {
    /// Commands grid rows (PluginHost.AddWPtoList returns the new row's index).
    pub fn row_count(&self) -> usize {
        self.items.len()
    }

    /// FlightPlanner.AddCommand from a plugin: [p1, p2, p3, p4, x (lat), y (lng), z (alt)].
    pub fn add_command(&mut self, command: u16, p: [f64; 7]) {
        self.items.push(Locationwp { id: command, p1: p[0] as f32, p2: p[1] as f32, p3: p[2] as f32, p4: p[3] as f32, lat: p[4], lng: p[5], alt: p[6] as f32, frame: 3, ..Default::default() });
    }

    /// The web service's view of the planner: mission, home, links and stream rates.
    pub fn update_kml(&mut self, link: Option<&mp_core::link::Link>, extra: &[mp_core::link::Link], settings: &crate::app::Settings) {
        let mut links = Vec::new();
        if let Some(link) = link { links.push(link.shared.clone()); }
        links.extend(extra.iter().map(|l| l.shared.clone()));
        let mission = if self.mission_type == 0 { self.items.clone() } else { self.stash[0].clone() };
        let rate = |k: &str, d: u16| settings.planner.get(k).and_then(|v| v.parse().ok()).unwrap_or(d);
        crate::httpserver::update(crate::httpserver::Snapshot {
            home: self.home,
            mission,
            links,
            command: link.map(|l| l.sender()),
            rates: [rate("rateattitude", 4), rate("rateposition", 2), rate("ratestatus", 2), rate("raterc", 2), rate("ratesensors", 2)],
        });
    }

    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        match e {
            // getWPs -> WPtoScreen -> processToScreen for every mission type, an empty list
            // included (it clears the grid); run on the next frame with the app context
            LinkEvent::MissionDownloaded { mission_type: mt @ (0 | 1 | 2), items } if self.awaiting_read == Some(*mt) => {
                self.awaiting_read = None;
                self.pending_download = Some((*mt as usize, items.clone()));
            }
            // getWPs threw: the progress dialog shows the error and the grid is kept
            LinkEvent::MissionDownloadFailed { mission_type } if self.awaiting_read == Some(*mission_type) => {
                self.awaiting_read = None;
            }
            LinkEvent::Disconnected(_) => {
                self.awaiting_read = None;
            }
            LinkEvent::MissionUploaded { mission_type: 1 | 2, result } => {
                match result {
                    Ok(()) => self.post_upload = Some(std::time::Instant::now()),
                    Err(e) => toasts.error(format!("Write failed: {e}")),
                }
            }
            LinkEvent::MissionUploaded { mission_type: 0, result } => match result {
                Ok(()) => {
                    self.post_upload = Some(std::time::Instant::now());
                    toasts.info("Done");
                }
                Err(e) => toasts.error(format!("Write failed: {e}")),
            },
            _ => {}
        }
    }

    /// BUT_write_Click / but_writewpfast_Click up to the upload: each check either fails with
    /// the original message, asks (and resumes here with the answer), or passes.
    fn step_write(&mut self, app: &mut AppCtx) {
        let Some(mut wc) = self.write_check.take() else { return };
        if wc.prompt.is_some() {
            self.write_check = Some(wc);
            return;
        }
        if !wc.alt_asked {
            wc.alt_asked = true;
            if AltMode::from_frame(app.settings.alt_frame) == AltMode::Absolute {
                wc.prompt = Some(WritePrompt::AbsoluteAlt);
                self.write_check = Some(wc);
                return;
            }
        }
        if wc.fast && self.mission_type != 0 {
            app.toasts.error("Only available for missions");
            return;
        }
        if ![self.home.lat, self.home.lng, self.home.alt as f64].iter().all(|v| v.is_finite()) {
            app.toasts.message("Your home location is invalid", "Error");
            return;
        }
        let fw = self.fw(app);
        while wc.row < self.items.len() {
            let w = self.items[wc.row];
            if !wc.zero_ok {
                match write_row_check(&w, wc.row, self.alt_warn, fw) {
                    RowCheck::Fail(msg) => {
                        app.toasts.error(msg);
                        return;
                    }
                    RowCheck::ZeroAlt { plane } if show_again(&app.settings.planner, zero_alt_tag(plane)) => {
                        wc.prompt = Some(WritePrompt::ZeroAlt { plane });
                        self.write_check = Some(wc);
                        return;
                    }
                    _ => {}
                }
            }
            wc.zero_ok = false;
            wc.row += 1;
        }
        let Some(l) = app.link.filter(|_| app.connected()) else {
            app.toasts.message("Please connect first!", "Error");
            return;
        };
        if wc.fast {
            // home is item 0 of the uploaded list
            let mut all = vec![self.home];
            all.extend(self.items.iter().copied());
            let (tx, rx) = crossbeam_channel::bounded(1);
            let (link_tx, shared, rows, altmode) = (l.sender(), l.shared.clone(), self.items.len(), app.settings.alt_frame);
            std::thread::spawn(move || {
                let cancel = std::sync::atomic::AtomicBool::new(false);
                let r = mp_core::wpfast::write_fast(link_tx, shared.clone(), &all, rows, altmode, &cancel);
                shared.write().progress = None;
                let _ = tx.send(r);
            });
            self.fast_job = Some(rx);
        } else {
            // saveWPs: home is item 0 of an ArduPilot mission; a fence is always GLOBAL
            let ardupilot = l.shared.read().cs.firmware != Firmware::Px4;
            let mut all = if self.mission_type == 0 && ardupilot { vec![Locationwp { frame: 0, ..self.home }] } else { Vec::new() };
            all.extend(self.items.iter().copied());
            if self.mission_type == 1 {
                for w in &mut all {
                    w.frame = 0;
                }
            }
            self.post_upload = None;
            self.mission_io(l, self.mission_type as u8, false, all);
        }
    }

    /// The prompts of BUT_read_Click and the write checks.
    fn write_prompts_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if std::mem::take(&mut self.read_pending) {
            self.start_read(app);
        }
        if let Some((mt, cmds)) = self.pending_download.take() {
            self.process_to_screen_type(&cmds, false, mt, app);
        }
        if let Some(cellhome) = self.reset_home_prompt {
            let mut answer = None;
            egui::Modal::new(egui::Id::new("fp_reset_home")).show(ctx, |ui| {
                ui.set_width(300.0);
                ui.label(egui::RichText::new(crate::i18n::t("Reset Home Coords")).strong());
                theme::label(ui, "Reset Home to loaded coords");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if theme::button(ui, "No").clicked() {
                        answer = Some(false);
                    }
                    if theme::button(ui, "Yes").clicked() {
                        answer = Some(true);
                    }
                });
            });
            if let Some(yes) = answer {
                self.reset_home_prompt = None;
                if yes {
                    // TXT_homelat/lng = double.Parse(cell).ToString(); TXT_homealt = ToString("0.00")
                    self.home.lat = cellhome.lat;
                    self.home.lng = cellhome.lng;
                    let alt = mp_core::dotnet::format_fixed(cellhome.alt as f64, 1, 2, mp_core::dotnet::current_culture());
                    self.home.alt = mp_core::dotnet::double_parse(&alt).map(|v| v as f32).unwrap_or(cellhome.alt);
                }
            }
        }
        if self.read_confirm {
            let mut answer = None;
            egui::Modal::new(egui::Id::new("fp_read_confirm")).show(ctx, |ui| {
                ui.set_width(300.0);
                ui.label(egui::RichText::new(crate::i18n::t("Confirm")).strong());
                theme::label(ui, "This will clear your existing points, Continue?");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if theme::button(ui, "Cancel").clicked() {
                        answer = Some(false);
                    }
                    if theme::button(ui, "OK").clicked() {
                        answer = Some(true);
                    }
                });
            });
            if let Some(ok) = answer {
                self.read_confirm = false;
                if ok {
                    self.start_read(app);
                }
            }
        }
        let Some(prompt) = self.write_check.as_ref().and_then(|w| w.prompt) else { return };
        let mut answer = None;
        match prompt {
            WritePrompt::AbsoluteAlt => {
                egui::Modal::new(egui::Id::new("fp_absolute_alt")).show(ctx, |ui| {
                    ui.set_width(300.0);
                    ui.label(egui::RichText::new(crate::i18n::t("Alt Mode")).strong());
                    theme::label(ui, "Absolute Alt is selected are you sure?");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                        if theme::button(ui, "No").clicked() {
                            answer = Some(false);
                        }
                        if theme::button(ui, "Yes").clicked() {
                            answer = Some(true);
                        }
                    });
                });
                if let Some(yes) = answer {
                    // No switches CMB_altmode to Relative; the write goes on either way
                    if !yes {
                        app.settings.alt_frame = AltMode::Relative as u8;
                    }
                    if let Some(w) = self.write_check.as_mut() {
                        w.prompt = None;
                    }
                }
            }
            WritePrompt::ZeroAlt { plane } => {
                let row = self.write_check.as_ref().map(|w| w.row).unwrap_or(0);
                let tag = zero_alt_tag(plane);
                let key = show_again_key(tag);
                let mut again = app.settings.planner.get(&key).map(|v| v != "False").unwrap_or(true);
                let text = zero_alt_text(plane, row + 1);
                let (body, link) = split_link(&text);
                egui::Modal::new(egui::Id::new("fp_zero_alt")).show(ctx, |ui| {
                    ui.set_min_width(300.0);
                    ui.set_max_width(600.0);
                    ui.label(egui::RichText::new(crate::i18n::t("Zero Altitude Warning")).strong());
                    theme::label(ui, &body);
                    if let Some((url, label)) = &link {
                        ui.hyperlink_to(label, url);
                    }
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut again, crate::i18n::t("Show me again?")).changed() {
                            // chk_CheckStateChanged saves at once
                            app.settings.planner.insert(key.clone(), if again { "True" } else { "False" }.into());
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if theme::button(ui, "Cancel").clicked() {
                                answer = Some(false);
                            }
                            if theme::button(ui, "OK").clicked() {
                                answer = Some(true);
                            }
                        });
                    });
                });
                if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
                    answer = Some(false);
                }
                match answer {
                    Some(true) => {
                        if let Some(w) = self.write_check.as_mut() {
                            w.prompt = None;
                            w.zero_ok = true;
                        }
                    }
                    Some(false) => self.write_check = None,
                    None => {}
                }
            }
        }
    }

    /// True while a BUT_read_Click is still asking, queued, or waiting for its download.
    pub fn read_busy(&self) -> bool {
        self.read_pending || self.read_confirm || self.awaiting_read.is_some() || self.pending_download.is_some()
    }

    /// BUT_read_Click from outside the panel (MainV2's "loadwpsonconnect"): the confirm when
    /// the grid has rows, else the read, run on the panel's next frame with the app context.
    pub fn but_read_click(&mut self) {
        if self.items.is_empty() {
            self.read_pending = true;
        } else {
            self.read_confirm = true;
        }
    }

    fn start_read(&mut self, app: &mut AppCtx) {
        match app.link.filter(|_| app.connected()) {
            None => app.toasts.message("Please connect first!", "Error"),
            Some(l) => self.mission_io(l, self.mission_type as u8, true, Vec::new()),
        }
    }

    /// Commands_SelectionChangeCommitted: the new command, with TAKEOFF / LAND /
    /// DO_DIGICAM_CONTROL defaults, and UNKNOWN asking for the MAVLink id.
    fn command_selected(&mut self, i: usize, name: &str, app: &mut AppCtx) {
        let default_alt = app.settings.default_alt as f32;
        let Some(w) = self.items.get_mut(i) else { return };
        if name != "UNKNOWN" {
            if let Some(id) = mavcmd::cmd_id(name) {
                w.id = id;
            }
        }
        apply_command_defaults(w, name, default_alt);
        if name == "UNKNOWN" {
            self.menu_prompt = Some(menu::MenuPrompt::CommandId(i, "-1".into()));
        }
    }

    /// Ctrl+Z: the last history entry back on screen (WPtoScreen)
    fn undo(&mut self) {
        if let Some((home, items)) = self.history.pop() {
            self.home = home;
            self.items = items;
            self.selected = None;
        }
    }

    /// updateUndoBuffer
    /// updateUndoBuffer: the vehicle's home when it has one, else the planned home
    pub(super) fn update_undo_buffer(&mut self) {
        let home = self.vehicle_home.filter(|h| h.lat != 0.0 && h.lng != 0.0).unwrap_or(self.home);
        self.history.push((home, self.items.clone()));
    }

    /// FlightPlanner.ProcessCmdKey: Ctrl+Z undo, Ctrl+O / Ctrl+S waypoint files, and
    /// Ctrl+Shift+F / W / R for Write Fast, Write and Read.
    fn hotkeys(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let key = |m: egui::Modifiers, k: egui::Key| ctx.input_mut(|i| i.consume_key(m, k));
        let cs = egui::Modifiers::COMMAND | egui::Modifiers::SHIFT;
        if key(cs, egui::Key::F) && self.write_check.is_none() {
            self.write_check = Some(WriteCheck { fast: true, ..Default::default() });
        }
        if key(cs, egui::Key::W) && self.write_check.is_none() {
            self.write_check = Some(WriteCheck { fast: false, ..Default::default() });
        }
        if key(cs, egui::Key::R) {
            if self.items.is_empty() {
                self.start_read(app);
            } else {
                self.read_confirm = true;
            }
        }
        if key(egui::Modifiers::COMMAND, egui::Key::Z) {
            self.undo();
        }
        // Ctrl+M: the grid becomes MAV.wps (planned home first)
        if key(egui::Modifiers::COMMAND, egui::Key::M) {
            if let Some(l) = app.link {
                let (sys, comp) = {
                    let s = l.shared.read();
                    (s.cs.sysid, s.cs.compid)
                };
                let mut wps = vec![Locationwp { id: mavcmd::WAYPOINT, frame: 0, ..self.home }];
                for (a, w) in self.items.iter().enumerate() {
                    wps.push(Locationwp { seq: a as u16 + 1, ..*w });
                }
                l.set_mission(sys, comp, wps);
            }
        }
        if key(egui::Modifiers::COMMAND, egui::Key::O) {
            if platform::HAS_NATIVE_DIALOGS {
                if let Some((name, text)) = platform::open_text("Open Waypoint File", "All Supported Types", &["txt", "waypoints", "plan"]) {
                    self.load_text(&name, &text, app);
                }
            } else {
                self.browser_open = true;
            }
        }
        if key(egui::Modifiers::COMMAND, egui::Key::S) {
            self.savewaypoints(app);
        }
    }

    /// savewaypoints: Mission (.waypoints/.txt) or Mission JSON (.mission), remembering the
    /// directory in "WPFileDirectory".
    pub(super) fn savewaypoints(&mut self, app: &mut AppCtx) {
        let dir = app.settings.planner.get("WPFileDirectory").cloned();
        let name = std::path::Path::new(&self.wpfilename).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let Some(path) = platform::save_path("Save As", &name, &[("Mission", &["waypoints", "txt"]), ("Mission JSON", &["mission"])], dir.as_deref()) else { return };
        let mut path = path;
        if path.extension().is_none() {
            path.set_extension("waypoints");
        }
        if let Some(d) = path.parent() {
            app.settings.planner.insert("WPFileDirectory".into(), d.display().to_string());
        }
        let file = path.display().to_string();
        let text = if file.ends_with(".mission") {
            mission::write_mission_json(&self.home, &self.items, app.settings.alt_frame)
        } else {
            mission::write_waypoint_file(&self.home, &self.items)
        };
        match std::fs::write(&path, text) {
            Ok(()) => {
                // the label only changes for the waypoint format (the JSON branch returns first)
                if !file.ends_with(".mission") {
                    self.wpfile = format!("Saved {}", path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default());
                }
            }
            Err(_) => app.toasts.error("Error"),
        }
    }

    fn fw(&self, app: &AppCtx) -> Firmware {
        app.link.filter(|_| app.connected()).map(|l| l.shared.read().cs.firmware).or(self.firmware).unwrap_or_default()
    }

    /// MAV.cs.PlannedHomeLocation: the planner's home row.
    fn planned_home_location(&self) -> mp_core::extlibs::utilities::PointLatLngAlt {
        mp_core::extlibs::utilities::PointLatLngAlt::new_alt(self.home.lat, self.home.lng, self.home.alt as f64)
    }

    /// FlightPlanner.AddCommand (PluginHost.AddWPtoList): `Commands.Rows.Add()` then
    /// FillCommand with p = [p1, p2, p3, p4, x (lng), y (lat), z (alt)]. Returns the row.
    pub(crate) fn add_wp_to_list(&mut self, app: &mut AppCtx, cmd: u16, p: [f64; 7]) -> usize {
        let [p1, p2, p3, p4, x, y, z] = p;
        let mut row = Locationwp { id: cmd, frame: app.settings.alt_frame, ..Default::default() };
        // switch wp to spline if spline checked
        if self.spline && cmd == mavcmd::WAYPOINT {
            row.id = mavcmd::SPLINE_WAYPOINT;
        }
        if cmd == mavcmd::WAYPOINT {
            // add delay if supplied
            row.p1 = p1 as f32;
            self.set_from_map(app, &mut row, y, x, mp_core::dotnet::cast_i32(z), mp_core::dotnet::math_round(p1, 1));
        } else if cmd == mavcmd::LOITER_UNLIM {
            self.set_from_map(app, &mut row, y, x, mp_core::dotnet::cast_i32(z), -1.0);
        } else {
            row.p1 = p1 as f32;
            row.p2 = p2 as f32;
            row.p3 = p3 as f32;
            row.p4 = p4 as f32;
            row.lat = y;
            row.lng = x;
            row.alt = z as f32;
        }
        self.items.push(row);
        self.items.len() - 1
    }

    /// FlightPlanner.setfromMap for a new row (Lat/Long/Alt/Delay headers): Lat and Long go
    /// through their "0.0000000" cell text; Alt is the passed value, else TXT_DefaultAlt,
    /// and with Verify Height the SRTM-derived height for the alt mode.
    fn set_from_map(&self, app: &mut AppCtx, row: &mut Locationwp, lat: f64, lng: f64, alt: i32, p1: f64) {
        use mp_core::dotnet::{current_culture, double_parse, format_fixed};
        let cell = |v: f64| double_parse(&format_fixed(v, 1, 7, current_culture())).unwrap_or(v);
        row.lat = cell(lat);
        row.lng = cell(lng);
        if alt != -1 && alt != -2 {
            // TXT_DefaultAlt must be an int; a 0 asks for one with "100" offered
            let mut default_alt = app.settings.default_alt;
            if default_alt.fract() != 0.0 || !default_alt.is_finite() {
                app.toasts.error("Your default alt is not valid");
                return;
            }
            if default_alt == 0.0 {
                default_alt = 100.0;
                app.settings.default_alt = default_alt;
            }
            let ans = default_alt.trunc();
            let mut value = default_alt;
            if alt != 0 {
                // use passed in value
                value = alt as f32;
            }
            if ans == 0.0 {
                value = if self.fw(app) == Firmware::ArduCopter2 { 15.0 } else { 50.0 };
            }
            if self.verify_height {
                let multalt = crate::units::alt(1.0) as f64;
                let srtm = |a: f64, b: f64| crate::srtm::altitude(a, b).unwrap_or(0.0);
                let def = default_alt as i32 as f64;
                value = match AltMode::from_frame(app.settings.alt_frame) {
                    // is absolute but no verify
                    AltMode::Absolute => (srtm(lat, lng) * multalt + def) as f32,
                    AltMode::Terrain => def as f32,
                    // relative and verify
                    _ => ((srtm(lat, lng) as i32) as f64 * multalt + def - (srtm(self.home.lat, self.home.lng) as i32) as f64 * multalt) as f32,
                };
            }
            row.alt = value;
        }
        // Add more for other params: the Delay column
        if p1 != -1.0 {
            row.p1 = p1 as f32;
        }
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
        if read {
            // getWPs hands its own download to WPtoScreen; other readers (Flight Data's resume,
            // the rally fetch on connect) never touch the planner grid
            self.awaiting_read = Some(mission_type);
        }
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
        self.vehicle_home = app.link.and_then(|l| l.shared.read().cs.home).map(|h| Locationwp { id: mavcmd::WAYPOINT, lat: h.lat, lng: h.lng, alt: h.alt as f32, ..Default::default() });
        self.hotkeys(ui.ctx(), app);
        if let Some(ev) = self.ftp_job.as_ref().and_then(|rx| rx.try_recv().ok()) {
            self.ftp_job = None;
            self.on_event(&ev, app.toasts);
            // a MAVFtp upload returns before the parameter steps
            if matches!(ev, LinkEvent::MissionUploaded { .. }) {
                self.post_upload = None;
            }
        }
        // the end of saveWPs: home again, then the radius parameters (those that exist)
        if self.post_upload.take().is_some() {
            let pts: Vec<(f64, f64)> = self.items.iter().filter(|w| w.lat != 0.0 && w.lng != 0.0).map(|w| (w.lat, w.lng)).collect();
            if !pts.is_empty() {
                std::thread::spawn(move || {
                    for (lat, lng) in pts {
                        let _ = crate::srtm::altitude(lat, lng);
                    }
                });
            }
            if let Some(l) = app.link {
                l.send(LinkCommand::CommandNoAck { command: 512, params: [242.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                // TXT_WPRad / TXT_loiterrad divided by multiplierdist: the settings hold metres
                let wp = app.settings.wp_radius;
                let loiter = app.settings.loiter_radius;
                let have = |n: &str| l.shared.read().params.contains_key(n);
                for (n, v) in [("WP_RADIUS", wp), ("WP_RADIUS_M", wp), ("WPNAV_RADIUS", wp * 100.0)] {
                    if have(n) {
                        l.send(LinkCommand::SetParam { name: n.into(), value: v });
                    }
                }
                if let Some(n) = ["LOITER_RAD", "WP_LOITER_RAD"].into_iter().find(|n| have(n)) {
                    l.send(LinkCommand::SetParam { name: n.into(), value: loiter });
                }
            }
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
        // developer switch for screenshots: MP_SIMPLEGRID=1 runs SimpleGridPlugin.but_Click on a
        // sample polygon
        if self.polygon.is_empty() && self.simple_grid.is_none() && std::env::var_os("MP_SIMPLEGRID").is_some() {
            let (lat, lng) = (-35.363261, 149.165230);
            let mut polygon = vec![(lat + 0.0020, lng - 0.0030), (lat + 0.0024, lng + 0.0025), (lat - 0.0015, lng + 0.0032), (lat - 0.0022, lng - 0.0020)];
            let mut config = std::mem::take(&mut app.settings.planner);
            let mut host = crate::extlibs::simplegrid::PluginHost {
                FPDrawnPolygon: &mut polygon,
                PlannedHomeLocation: self.planned_home_location(),
                config: &mut config,
                FDMapType: app.settings.fd_map.provider,
            };
            let click = crate::extlibs::simplegrid::GridPlugin.but_Click(&mut host);
            app.settings.planner = config;
            self.polygon = polygon;
            if let crate::extlibs::simplegrid::ButClick::Show(g) = click {
                self.simple_grid = Some(g);
            }
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
        ui.put(r(px + 3.0, py + 3.0, 50.0, 17.0), egui::Checkbox::new(&mut self.grid, crate::i18n::t("Grid")));
        // label1_Click: the network link served by MainV2's web service
        if ui.put(r(px + 62.0, py + 3.0, 60.0, 13.0), egui::Link::new(crate::i18n::t("View KML"))).clicked() {
            ui.ctx().open_url(egui::OpenUrl::new_tab("http://127.0.0.1:56781/network.kml"));
        }
        ui.put(r(px + 3.0, py + 22.0, 111.0, 21.0), |ui: &mut egui::Ui| {
            let mut prov = app.settings.fp_map.provider;
            provider_combo(ui, &mut prov, "comboBoxMapType");
            app.settings.fp_map.provider = prov;
            ui.allocate_response(Vec2::ZERO, Sense::hover())
        });
        p.text(at(px + 4.0, py + 46.0), egui::Align2::LEFT_TOP, crate::i18n::t("Status"), f.clone(), white);
        if put_button(ui, r(px + 3.0, py + 68.0, 115.0, 23.0), "Inject Custom Map") {
            if let Some(dir) = platform::pick_folder("Inject Custom Map") {
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
                if let Some((name, text)) = platform::open_text("Open Waypoint File", "All Supported Types", &["txt", "waypoints", "plan"]) {
                    self.load_text(&name, &text, app);
                }
            } else {
                self.browser_open = true;
            }
        }
        if put_button(ui, r(px + 3.0, py + 32.0, 115.0, 23.0), "Save File") {
            self.savewaypoints(app);
        }
        p.text(at(px + 3.0, py + 59.0), egui::Align2::LEFT_TOP, &self.wpfile, f.clone(), white);

        // panel5 (3,278 123x87): Read / Write / Write Fast
        let py = 278.0;
        let connected = app.connected();
        let _ = connected;
        if put_button(ui, r(px + 3.0, py + 4.0, 115.0, 23.0), "Read") {
            // BUT_read_Click asks before replacing a non-empty list
            if self.items.is_empty() {
                self.start_read(app);
            } else {
                self.read_confirm = true;
            }
        }
        if put_button(ui, r(px + 3.0, py + 32.0, 115.0, 23.0), "Write") && self.write_check.is_none() {
            self.write_check = Some(WriteCheck { fast: false, ..Default::default() });
        }
        if put_button(ui, r(px + 3.0, py + 61.0, 115.0, 23.0), "Write Fast") && self.write_check.is_none() {
            self.write_check = Some(WriteCheck { fast: true, ..Default::default() });
        }
        self.step_write(app);
        self.write_prompts_ui(ui.ctx(), app);
        if let Some(r) = self.fast_job.as_ref().and_then(|rx| rx.try_recv().ok()) {
            self.fast_job = None;
            // ProgressReporterDialogue's ShowDoneWithError
            if let Err(e) = r {
                app.toasts.message(e, "Error");
            }
        }
        if self.fast_job.is_some() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));
        }

        // panel1 (3,371 123x89): Home Location
        let py = 371.0;
        p.text(at(px + 18.0, py + 3.0), egui::Align2::LEFT_TOP, crate::i18n::t("Home Location"), f.clone(), white);
        p.text(at(px + 4.0, py + 22.0), egui::Align2::LEFT_TOP, crate::i18n::t("Lat"), f.clone(), white);
        p.text(at(px + 3.0, py + 45.0), egui::Align2::LEFT_TOP, crate::i18n::t("Long"), f.clone(), white);
        p.text(at(px + 3.0, py + 68.0), egui::Align2::LEFT_TOP, crate::i18n::t("ASL"), f.clone(), white);
        ui.put(r(px + 47.0, py + 18.0, 65.0, 20.0), egui::DragValue::new(&mut self.home.lat).speed(0.00001).max_decimals(7));
        ui.put(r(px + 47.0, py + 42.0, 65.0, 20.0), egui::DragValue::new(&mut self.home.lng).speed(0.00001).max_decimals(7));
        ui.put(r(px + 47.0, py + 65.0, 65.0, 20.0), egui::DragValue::new(&mut self.home.alt).speed(0.5).max_decimals(2));

        if !connected {
            let py = 466.0;
            p.text(at(px + 3.0, py), egui::Align2::LEFT_TOP, crate::i18n::t("Vehicle type (offline)"), f.clone(), white);
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

        p.text(o + Vec2::new(9.0, 4.0), egui::Align2::LEFT_TOP, crate::i18n::t("WP Radius"), f.clone(), white);
        // TXT_WPRad / TXT_loiterrad (multiplierdist) and TXT_DefaultAlt (multiplieralt) are in
        // display units; the settings keep metres
        let unit_box = |ui: &mut egui::Ui, rect: Rect, metres: &mut f32, m: f32, max: f32| {
            let mut shown = *metres * m;
            if ui.put(rect, egui::DragValue::new(&mut shown).range(0.0..=max * m).max_decimals(1)).changed() {
                *metres = shown / m;
            }
        };
        unit_box(ui, r(23.0, 20.0, 36.0, 20.0), &mut app.settings.wp_radius, crate::units::dist(1.0), 1000.0);
        p.text(o + Vec2::new(76.0, 4.0), egui::Align2::LEFT_TOP, crate::i18n::t("Loiter Radius"), f.clone(), white);
        unit_box(ui, r(89.0, 20.0, 36.0, 20.0), &mut app.settings.loiter_radius, crate::units::dist(1.0), 10000.0);
        p.text(o + Vec2::new(151.0, 4.0), egui::Align2::LEFT_TOP, crate::i18n::t("Default Alt"), f.clone(), white);
        unit_box(ui, r(154.0, 20.0, 40.0, 20.0), &mut app.settings.default_alt, crate::units::alt(1.0), 10000.0);
        ui.put(r(213.0, 7.0, 79.0, 21.0), |ui: &mut egui::Ui| {
            // CMB_altmode.DataSource = EnumTranslator.EnumToList<altmode>() (Key/Value); a
            // saved value outside the list falls back to index 0 (SelectedIndexChanged)
            let list = altmode_list();
            if !list.iter().any(|(k, _)| *k == app.settings.alt_frame as i32) {
                if let Some((k, _)) = list.first() {
                    app.settings.alt_frame = *k as u8;
                }
            }
            let cur = app.settings.alt_frame as i32;
            egui::ComboBox::from_id_salt("CMB_altmode")
                .width(71.0)
                .selected_text(list.iter().find(|(k, _)| *k == cur).map(|(_, v)| v.as_str()).unwrap_or(""))
                .show_ui(ui, |ui| {
                    for (k, v) in &list {
                        if ui.selectable_label(cur == *k, v.as_str()).clicked() {
                            app.settings.alt_frame = *k as u8;
                        }
                    }
                })
                .response
        });
        // CHK_verifyheight.Visible = displayCheckHeightBox
        if crate::display::flag(&app.settings.planner, "displayCheckHeightBox") {
            ui.put(r(298.0, 13.0, 95.0, 17.0), egui::Checkbox::new(&mut self.verify_height, crate::i18n::t("Verify Height")));
        }
        if put_button(ui, r(398.0, 8.0, 75.0, 23.0), "Add Below") {
            let at = self.selected.map(|i| i + 1).unwrap_or(self.items.len()).min(self.items.len());
            let mut wp = self.new_wp(app, mavcmd::WAYPOINT, 0.0, 0.0);
            if let Some(prev) = at.checked_sub(1).and_then(|i| self.items.get(i)) {
                wp.lat = prev.lat;
                wp.lng = prev.lng;
            }
            self.update_undo_buffer();
            self.items.insert(at, wp);
            self.selected = Some(at);
        }
        p.text(o + Vec2::new(479.0, 4.0), egui::Align2::LEFT_TOP, crate::i18n::t("Alt Warn"), f.clone(), white);
        // TXT_altwarn, kept in Settings "fpminaltwarning"
        if let Some(v) = app.settings.planner.get("fpminaltwarning").and_then(|v| v.parse().ok()) {
            self.alt_warn = v;
        }
        if ui.put(r(482.0, 20.0, 40.0, 20.0), egui::DragValue::new(&mut self.alt_warn)).changed() {
            app.settings.planner.insert("fpminaltwarning".into(), self.alt_warn.to_string());
        }
        ui.put(r(528.0, 12.0, 60.0, 17.0), egui::Checkbox::new(&mut self.spline, crate::i18n::t("Spline")));
        // chk_usemavftp: Settings "UseMissionMAVFTP"
        self.mavftp = app.settings.planner.get("UseMissionMAVFTP").map(|v| v == "True").unwrap_or(false);
        if ui.put(r(589.0, 12.0, 70.0, 17.0), egui::Checkbox::new(&mut self.mavftp, crate::i18n::t("MAVFTP"))).changed() {
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

        // WPOverlay.pointlist and setgradanddistandaz
        let points = overlay_points(&self.home, &self.items, |lat, lng| crate::srtm::altitude(lat, lng));
        let derived = grad_dist_az(&self.home, &points, self.items.len(), crate::units::dist(1.0) as f64);
        let mut cmd_pick: Option<(usize, String)> = None;

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
            p.with_clip_rect(r).text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, header_text(i, &headers[i]), f.clone(), Color32::WHITE);
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
            for (c, r) in cells.iter().enumerate().filter(|(_, r)| r.width() > 0.0) {
                p.rect_filled(*r, 0.0, if c == 0 { theme::bg() } else { row_bg });
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
                        // Command.DataSource: the firmware's commands, then "UNKNOWN"
                        for name in cmds.iter().map(|c| c.name.as_str()).chain(["UNKNOWN"]) {
                            if ui.selectable_label(false, name).clicked() {
                                cmd_pick = Some((i, name.to_string()));
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
                        // Escape cancels a DataGridView cell edit
                        let cancelled = ui.input(|i| i.key_pressed(egui::Key::Escape));
                        if !cancelled && text != shown[k] {
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
                // Frame.DataSource = EnumTranslator.EnumToList<altmode>()
                let list = altmode_list();
                let cur = w.frame as i32;
                egui::ComboBox::from_id_salt(("frame", i))
                    .width(cells[9].width() - 6.0)
                    .selected_text(list.iter().find(|(k, _)| *k == cur).map(|(_, v)| v.as_str()).unwrap_or(""))
                    .show_ui(ui, |ui| {
                        for (k, v) in &list {
                            if ui.selectable_label(cur == *k, v.as_str()).clicked() {
                                w.frame = *k as u8;
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
            if let Some(cols) = &derived[i] {
                for (k, txt) in cols.iter().enumerate() {
                    p.text(cells[COL_GRAD + k].left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, txt, f.clone(), Color32::WHITE);
                }
            }
        }
        if let Some(i) = select {
            self.selected = Some(i);
        }
        if let Some((i, name)) = cmd_pick {
            self.selected = Some(i);
            self.command_selected(i, &name, app);
        }
        if let Some(i) = delete {
            self.update_undo_buffer();
            self.items.remove(i);
            self.selected = None;
        }
        if let Some((a, b)) = swap {
            self.update_undo_buffer();
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

        // FlightPlanner.RegenerateWPRoute: WPOverlay.CreateOverlay(home, commands, wpradius,
        // loiterradius, multiplieralt) builds the yellow route, the dashed home leg and the
        // Spline2 curves through SPLINE_WAYPOINT runs
        let key = WpOverlayKey {
            home,
            items: items.clone(),
            wp_radius,
            loiter_radius: app.settings.loiter_radius,
            alt_frame: app.settings.alt_frame,
        };
        if !matches!(&self.wp_overlay_cache, Some((k, _)) if *k == key) {
            // writeKML: home stays PointLatLngAlt.Zero while the home boxes are empty
            let mut h = if home.lat == 0.0 && home.lng == 0.0 {
                mp_core::extlibs::utilities::PointLatLngAlt::zero()
            } else {
                mp_core::extlibs::utilities::PointLatLngAlt {
                    Tag2: mission::AltMode::from_frame(app.settings.alt_frame).label().to_string(),
                    ..mp_core::extlibs::utilities::PointLatLngAlt::new_tagged(home.lat, home.lng, home.alt as f64, "H")
                }
            };
            let mut overlay = mp_core::extlibs::maps::WPOverlay::new();
            let list: Vec<_> = items.iter().map(mission::to_port).collect();
            let routes = match overlay.CreateOverlay(&mut h, &list, wp_radius as f64, app.settings.loiter_radius as f64, 1.0, None, &mut crate::srtm::altitude) {
                Ok(()) => overlay.overlay.routes,
                Err(e) => {
                    log::error!("{e}");
                    vec![]
                }
            };
            self.wp_overlay_cache = Some((key, routes));
        }
        let wp_routes = self.wp_overlay_cache.as_ref().map(|(_, r)| r.clone()).unwrap_or_default();
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
                painter.text(p + Vec2::new(10.0, -6.0), egui::Align2::LEFT_TOP, crate::i18n::t("Tracker Home"), fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
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
            for r in &wp_routes {
                let pts: Vec<Pos2> = r.points.iter().map(|p| view.to_screen(p.lat(), p.lng())).collect();
                draw_gmap_route(painter, resp.rect.min, &pts, &r.stroke);
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

        let center = (app.settings.fp_map.lat.clamp(-90.0, 90.0), app.settings.fp_map.lng.clamp(-180.0, 180.0));
        if self.last_center != Some(center) {
            self.last_center = Some(center);
            self.mouse = Some(center);
        }
        if mouse.is_some() {
            self.mouse = mouse;
            self.last_pointer = mouse;
        }
        self.dragging = new_drag;
        if moved.is_none() {
            self.drag_undo = false;
        }
        if let Some((idx, lat, lng)) = moved {
            if !self.drag_undo {
                self.drag_undo = true;
                self.update_undo_buffer();
            }
            if idx == HOME_IDX {
                // callMeDrag("H"): the home altitude follows SRTM (0 without data)
                self.home.lat = lat;
                self.home.lng = lng;
                self.home.alt = crate::srtm::altitude(lat, lng).unwrap_or(0.0) as f32;
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
        } else if let (Some((lat, lng)), 1 | 2) = (add_at, self.mission_type) {
            let id = if self.mission_type == 1 { 5001 } else { 5100 };
            let wp = Locationwp { id, lat, lng, alt: app.settings.default_alt, frame: if id == 5100 { 3 } else { 0 }, ..Default::default() };
            self.items.push(wp);
        } else if let Some((lat, lng)) = add_at {
            if self.home.lat == 0.0 && self.home.lng == 0.0 && self.items.is_empty() && !app.connected() {
                self.home.lat = lat;
                self.home.lng = lng;
            } else {
                self.update_undo_buffer();
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
        // SetMouseDisplay: from the last point of the route and from home to the pointer;
        // the labels keep their resx text until the pointer has been over the map
        let (prev_text, home_text) = match (self.last_pointer, self.route_last()) {
            (Some((la, ln)), Some(last)) => {
                let here = PointLatLngAlt::new(la, ln, 0.0);
                let prev = format!("{}: {} AZ: {:.0}", crate::i18n::t("Prev"), format_distance(&app.settings.planner, last.distance(&here) / 1000.0, true), last.bearing(&here));
                let home = PointLatLngAlt::new(self.home.lat, self.home.lng, 0.0);
                let homed = format!("{}: {}", crate::i18n::t("Home"), format_distance(&app.settings.planner, home.distance(&here) / 1000.0, true));
                (prev, homed)
            }
            _ => (crate::i18n::t("Prev"), crate::i18n::t("Home")),
        };
        for (y, t) in [
            (5.0, format!("{}: {}", crate::i18n::t("Distance"), format_distance(&app.settings.planner, total / 1000.0, false))),
            (18.0, prev_text),
            (31.0, home_text),
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
        p.text(o + Vec2::new(w - 45.0, 5.0), egui::Align2::LEFT_TOP, crate::i18n::t("Zoom"), f.clone(), Color32::BLACK);
        ui.put(
            Rect::from_min_size(o + Vec2::new(w - 49.0, 25.0), Vec2::new(45.0, 20.0)),
            egui::DragValue::new(&mut app.settings.fp_map.zoom).speed(0.1).range(2.0..=22.0).max_decimals(1),
        );
        theme::vtrackbar(ui, Rect::from_min_size(o + Vec2::new(w - 50.0, 42.0), Vec2::new(45.0, (h - 48.0).max(40.0))), &mut app.settings.fp_map.zoom, 2.0, 22.0);
    }

    /// pointlist's last point: the last located waypoint, else home.
    fn route_last(&self) -> Option<PointLatLngAlt> {
        let is_loc = |w: &&Locationwp| w.has_location() && (w.lat != 0.0 || w.lng != 0.0);
        self.items
            .iter()
            .rev()
            .find(is_loc)
            .map(|w| PointLatLngAlt::new(w.lat, w.lng, 0.0))
            .or_else(|| (self.home.lat != 0.0 || self.home.lng != 0.0).then(|| PointLatLngAlt::new(self.home.lat, self.home.lng, 0.0)))
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

    /// BUT_loadwpfile_Click after the dialog: a first line starting with '{' is a .plan
    /// (MissionFile.ReadFile + ConvertToLocationwps), anything else goes through
    /// readQGC110wpfile. Both end in processToScreen; lbl_wpfile shows "Loaded <name>".
    fn load_text(&mut self, name: &str, text: &str, app: &mut AppCtx) {
        let first = text.strip_prefix('\u{feff}').unwrap_or(text).lines().next().unwrap_or("");
        if first.starts_with('{') {
            match mission::read_mission_json(text) {
                Ok(cmds) => self.process_to_screen(&cmds, false, app),
                // the exception reaches BUT_loadwpfile_Click's caller unhandled; shown here
                Err(e) => app.toasts.error(e),
            }
        } else {
            self.wpfilename = name.to_string();
            self.read_qgc110wpfile(text, false, app);
        }
        self.wpfile = format!("Loaded {}", std::path::Path::new(name).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default());
    }

    /// readQGC110wpfile(file, append): WaypointFile.ReadWaypointFile, the home row dropped for
    /// fence and rally lists, then processToScreen. An exception shows "Can't open file! ".
    pub(super) fn read_qgc110wpfile(&mut self, text: &str, append: bool, app: &mut AppCtx) {
        let read = mission::read_waypoint_file(text, &mut |m| app.toasts.error(m));
        let mut cmds = match read {
            Ok(cmds) => cmds,
            // ReadWaypointFile shows "Invalid Waypoint file" and returns an empty list
            Err(e) => {
                app.toasts.error(e.to_string());
                Vec::new()
            }
        };
        if self.mission_type == 1 || self.mission_type == 2 {
            if cmds.is_empty() {
                // List.RemoveAt(0) on an empty list
                app.toasts.error("Can't open file! System.ArgumentOutOfRangeException: Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index");
                return;
            }
            cmds.remove(0);
        }
        self.process_to_screen(&cmds, append, app);
    }

    /// processToScreen(cmds, append): the grid is cleared unless appending; rows stop at the
    /// first id 0 or 255 after home; appending skips the list's home. Nav rows set Spline2's
    /// rates from WPNAV_ACCEL/WP_ACC and WPNAV_SPEED/WP_SPD and (not home) the alt mode. For a
    /// mission, row 0 is home: a different, non-zero lat asks "Reset Home to loaded coords",
    /// then the row is removed.
    pub(super) fn process_to_screen(&mut self, cmds: &[Locationwp], append: bool, app: &mut AppCtx) {
        self.process_to_screen_type(cmds, append, self.mission_type, app);
    }

    /// processToScreen into the list of mission type `mt` (the grid shows `cmb_missiontype`'s
    /// list; a download for another type lands in that type's stash).
    fn process_to_screen_type(&mut self, cmds: &[Locationwp], append: bool, mt: usize, app: &mut AppCtx) {
        use mp_core::extlibs::utilities::spline2::Spline2;
        let mut rows: Vec<Locationwp> = if append { self.list_for(mt).clone() } else { Vec::new() };
        self.selected = None;

        if cmds.is_empty() {
            *self.list_for(mt) = rows;
            return;
        }

        let params: std::collections::BTreeMap<String, f32> = app
            .link
            .map(|l| l.shared.read().params.iter().map(|(k, p)| (k.clone(), p.value)).collect())
            .unwrap_or_default();

        let mut i = rows.len() as i64 - 1;
        let mut cmdidx: i64 = -1;
        for temp in cmds {
            i += 1;
            cmdidx += 1;
            if temp.id == 0 && i != 0 {
                break; // 0 and not home
            }
            if temp.id == 255 && i != 0 {
                break; // bad record - never loaded any WP's - but have started the board up.
            }
            if cmdidx == 0 && append {
                // we dont want to add home again.
                i -= 1;
                continue;
            }

            if temp.id == mavcmd::WAYPOINT || temp.id == mavcmd::SPLINE_WAYPOINT || temp.id == mavcmd::TAKEOFF || temp.id == 179 {
                // cm/s - ac, m/s in 4.7+
                Spline2::set_wp_accel_cms(match (params.get("WPNAV_ACCEL"), params.get("WP_ACC")) {
                    (Some(v), _) => *v,
                    (None, Some(v)) => *v * 100.0,
                    _ => 100.0,
                });
                Spline2::set_wp_speed_cms(match (params.get("WPNAV_SPEED"), params.get("WP_SPD")) {
                    (Some(v), _) => *v,
                    (None, Some(v)) => *v * 100.0,
                    _ => 600.0,
                });
                // not home: CMB_altmode.SelectedValue = frame; a value outside the list leaves
                // no selection, and SelectedIndexChanged then picks index 0 (Relative)
                if i != 0 {
                    let list = altmode_list();
                    app.settings.alt_frame = if list.iter().any(|(k, _)| *k == temp.frame as i32) {
                        temp.frame
                    } else {
                        list.first().map(|(k, _)| *k as u8).unwrap_or(temp.frame)
                    };
                }
            }

            // the grid row holds the values, not the item's seq
            let row = Locationwp { seq: 0, ..*temp };
            rows.push(row);
        }

        // setWPParams: TXT_WPRad ("{0:N2}") from WP_RADIUS, then WPNAV_RADIUS / 100, then
        // WP_RADIUS_M (the last present wins); TXT_loiterrad from LOITER_RADIUS else WP_LOITER_RAD
        {
            // "{0:N2}": .NET rounds the 15-significant-digit text half away from zero
            let n2 = |v: f64| {
                let t = mp_core::dotnet::format_fixed(v, 1, 2, &mp_core::dotnet::INVARIANT_CULTURE);
                t.parse::<f64>().unwrap_or(v)
            };
            let mut wprad = None;
            if let Some(v) = params.get("WP_RADIUS") {
                wprad = Some(n2(*v as f64));
            }
            if let Some(v) = params.get("WPNAV_RADIUS") {
                wprad = Some(n2(*v as f64 / 100.0));
            }
            if let Some(v) = params.get("WP_RADIUS_M") {
                wprad = Some(n2(*v as f64));
            }
            if let Some(r) = wprad {
                app.settings.wp_radius = r as f32;
            }
            if let Some(v) = params.get("LOITER_RADIUS").or_else(|| params.get("WP_LOITER_RAD")) {
                app.settings.loiter_radius = *v;
            }
        }

        if !append && mt == 0 {
            if let Some(cellhome) = rows.first().copied() {
                // cellhome.Value.ToString() != TXT_homelat.Text && != "0"
                let cell = mp_core::dotnet::double_to_string(cellhome.lat);
                let homelat = if self.home.lat == 0.0 && self.home.lng == 0.0 { String::new() } else { mp_core::dotnet::double_to_string(self.home.lat) };
                if cell != homelat && cell != "0" {
                    self.reset_home_prompt = Some(cellhome);
                }
                // remove home row
                rows.remove(0);
            }
        }

        *self.list_for(mt) = rows;
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
        // SimpleGrid's GridUI (ShowDialog): Accept adds its rows through PluginHost.AddWPtoList
        // to whichever list the grid shows, then the form closes
        if let Some(mut g) = self.simple_grid.take() {
            let mut polygon = std::mem::take(&mut self.polygon);
            let mut config = std::mem::take(&mut app.settings.planner);
            let mut host = crate::extlibs::simplegrid::PluginHost {
                FPDrawnPolygon: &mut polygon,
                PlannedHomeLocation: self.planned_home_location(),
                config: &mut config,
                FDMapType: app.settings.fd_map.provider,
            };
            let r = g.show(ctx, app, &mut host);
            self.polygon = polygon;
            app.settings.planner = config;
            match r {
                crate::extlibs::simplegrid::GridResult::Open => self.simple_grid = Some(g),
                crate::extlibs::simplegrid::GridResult::Cancel => {}
                crate::extlibs::simplegrid::GridResult::Accept(calls) => {
                    for (cmd, p) in calls {
                        self.add_wp_to_list(app, cmd, p);
                    }
                    // writeKML follows from the list change; then savesettings, then Close
                    g.savesettings(&mut crate::extlibs::simplegrid::PluginHost {
                        FPDrawnPolygon: &mut self.polygon,
                        PlannedHomeLocation: mp_core::extlibs::utilities::PointLatLngAlt::zero(),
                        config: &mut app.settings.planner,
                        FDMapType: app.settings.fd_map.provider,
                    });
                }
            }
        }

        // FENCE_CIRCLE_INCLUSION centred on home with the entered radius
        let mut fc_done = None;
        if let Some(radius) = self.fence_circle.as_mut() {
            egui::Window::new(crate::i18n::t("Radius")).collapsible(false).resizable(false).show(ctx, |ui| {
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
            egui::Window::new(crate::i18n::t("Create WP Circle")).collapsible(false).resizable(false).show(ctx, |ui| {
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
            self.update_undo_buffer();
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
        egui::Window::new(crate::i18n::t("Open Waypoint File")).open(&mut open).collapsible(false).show(ctx, |ui| {
            let files = platform::list_files(&["txt", "waypoints", "plan"]);
            if files.is_empty() {
                ui.label(crate::i18n::t("No .waypoints files saved yet."));
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
                self.load_text(&name, &text, app);
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

/// `EnumTranslator.EnumToList<altmode>()`: (frame, name) in declaration order.
fn altmode_list() -> Vec<(i32, String)> {
    mp_core::extlibs::utilities::enumtranslator::EnumTranslator::EnumToList::<AltMode>().unwrap_or_default()
}

/// GMapRoute.UpdateGraphicsPath + OnRender (ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/
/// GMapRoute.cs): a local point beyond ±99000 px turns the pen solid; DashStyle.Custom draws
/// solid with a 15 px arrow head at the middle of every segment longer than 20 px. A short
/// segment is skipped without moving `last`, and a `last` at the control's (0,0) counts as
/// PointF.Empty, as in the C#.
fn draw_gmap_route(painter: &egui::Painter, origin: Pos2, pts: &[Pos2], pen: &mp_core::extlibs::maps::wpoverlay::Pen) {
    use mp_core::extlibs::maps::wpoverlay::DashStyle;
    let mut dash = pen.dash_style;
    if pts.iter().any(|p| (p.x - origin.x).abs() > 99000.0 || (p.y - origin.y).abs() > 99000.0) {
        dash = DashStyle::Solid;
    }
    let c = &pen.color;
    let stroke = Stroke::new(pen.width, Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), c.a()));
    if dash == DashStyle::Dash {
        // GDI+ DashStyle.Dash: 3 widths on, 1 off
        painter.extend(egui::Shape::dashed_line(pts, stroke, 3.0 * pen.width, pen.width));
    } else {
        painter.add(egui::Shape::line(pts.to_vec(), stroke));
    }
    if dash != DashStyle::Custom || pts.is_empty() {
        return;
    }
    let deg2rad = std::f64::consts::PI / 180.0;
    let mut last = Vec2::ZERO;
    for p in pts {
        let item = *p - origin;
        if last == Vec2::ZERO {
            last = item;
            continue;
        }
        let polx: f32 = item.x - last.x;
        let poly: f32 = item.y - last.y;
        let r = ((polx as f64).powi(2) + (poly as f64).powi(2)).sqrt();
        if r <= 20.0 {
            continue;
        }
        let angle = (poly as f64).atan2(polx as f64);
        if angle.is_nan() {
            continue;
        }
        let midxstart = last.x + polx / 2.0;
        let midystart = last.y + poly / 2.0;
        let leftangle = angle + 210.0 * deg2rad;
        let rightangle = angle - 210.0 * deg2rad;
        let length = 15.0f32;
        let start = origin + Vec2::new(midxstart, midystart);
        for a in [leftangle, rightangle] {
            let end = origin + Vec2::new(midxstart + length * a.cos() as f32, midystart + length * a.sin() as f32);
            painter.line_segment([start, end], stroke);
        }
        last = item;
    }
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
                painter.text(p + Vec2::new(8.0, 0.0), egui::Align2::LEFT_CENTER, crate::i18n::t("Return"), fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
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

#[derive(Clone, Copy, PartialEq, Debug)]
enum WritePrompt {
    /// "Absolute Alt is selected are you sure?"
    AbsoluteAlt,
    /// checkZeroAlts' Common.MessageShowAgain
    ZeroAlt { plane: bool },
}

#[derive(Default)]
struct WriteCheck {
    fast: bool,
    alt_asked: bool,
    row: usize,
    /// the zero-altitude prompt for `row` was answered OK
    zero_ok: bool,
    prompt: Option<WritePrompt>,
}

#[derive(Debug, PartialEq)]
enum RowCheck {
    Pass,
    Fail(String),
    ZeroAlt { plane: bool },
}

/// One grid row of BUT_write_Click's loop: the Alt Warn test (column 0 comes first), the
/// number check of Param1..Alt, then checkZeroAlts.
fn write_row_check(w: &Locationwp, row: usize, alt_warn: f32, fw: Firmware) -> RowCheck {
    let known = mavcmd::cmd_name(w.id) != "UNKNOWN";
    let alt = crate::units::alt(w.alt) as f64;
    if known && w.id < 95 && alt < alt_warn as f64 && ![22, 21, 20].contains(&w.id) {
        return RowCheck::Fail(format!("Low alt on WP#{}\nPlease reduce the alt warning, or increase the altitude", row + 1));
    }
    if ![w.p1 as f64, w.p2 as f64, w.p3 as f64, w.p4 as f64, w.lat, w.lng, w.alt as f64].iter().all(|v| v.is_finite()) {
        return RowCheck::Fail("There are errors in your mission".into());
    }
    let plane = fw == Firmware::ArduPlane;
    if known && (plane || fw == Firmware::ArduCopter2) && alt == 0.0 && [16, 19, 17, 18, 31].contains(&w.id) {
        return RowCheck::ZeroAlt { plane };
    }
    RowCheck::Pass
}

fn zero_alt_tag(plane: bool) -> &'static str {
    if plane { "Zero Altitude Warning Plane" } else { "Zero Altitude Warning" }
}

/// MessageShowAgain's settings key
fn show_again_key(tag: &str) -> String {
    crate::show_again::key(tag)
}

fn show_again(planner: &std::collections::BTreeMap<String, String>, tag: &str) -> bool {
    crate::show_again::should_show(planner, tag)
}

/// Strings.ZeroAltWarningPlane / ZeroAltWarningCopter
fn zero_alt_text(plane: bool, wp: usize) -> String {
    if plane {
        format!("WP# {wp} has zero altitude. If you actually want zero altitude, change it to 0.01 instead for predictable behavior.\n\nOn ArduPlane, zero altitudes may interpreted in two different ways: actual zero altitude, or keep current altitude. WAYPOINT commands will generally honor a zero altitude and LOITER commands will generally interpret it as \"use current altitude\", but only if the altitude frame is \"relative\". There may be more exceptions and this could change in future versions.\n\nONLY SET TO ZERO IF YOU REALLY UNDERSTAND EXACTLY WHAT IS GOING TO HAPPEN IN YOUR CASE (use a simulator to be sure).[link;https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html;Command Documentation]")
    } else {
        format!("WP# {wp} has zero altitude, this means no altitude change! If you want zero altitude, change it to 0.01. Do you want to continue or cancel wp upload?")
    }
}

fn split_link(text: &str) -> (String, Option<(String, String)>) {
    crate::show_again::split_link(text)
}

/// The parts of Commands_SelectionChangeCommitted that set cells for the chosen command.
fn apply_command_defaults(w: &mut Locationwp, name: &str, default_alt: f32) {
    match name {
        // default takeoff to non 0 alt; lat and lon are not used
        "TAKEOFF" => {
            if w.alt == 0.0 {
                w.alt = default_alt;
            }
            w.lat = 0.0;
            w.lng = 0.0;
        }
        "LAND" => w.alt = 0.0,
        // default to take shot
        "DO_DIGICAM_CONTROL" => {
            if w.lat == 0.0 {
                w.lat = 1.0;
            }
        }
        _ => {}
    }
}

/// One entry of WPOverlay.pointlist: absolute altitude and the row it came from ("H" is
/// None, ROI points are marked).
#[derive(Clone, Copy, Debug, PartialEq)]
struct OverlayPoint {
    lat: f64,
    lng: f64,
    alt: f64,
    row: Option<usize>,
    roi: bool,
}

/// WPOverlay.CreateOverlay's pointlist (the entries setgradanddistandaz reads): home, then
/// the located navigation commands, DO_SET_ROI, DO_LAND_START and DO_RETURN_PATH_START with
/// their altitude made absolute (GetHomeAlt).
fn overlay_points(home: &Locationwp, items: &[Locationwp], terrain: impl Fn(f64, f64) -> Option<f64>) -> Vec<Option<OverlayPoint>> {
    const LAST: u16 = 95;
    let (rtl, continue_alt, delay, guided_enable) = (20, 30, 93, 92);
    let (land, vtol_land, roi, land_start, return_path) = (21, 85, 201, 189, 188);
    let mut out = Vec::new();
    if home.lat != 0.0 || home.lng != 0.0 || home.alt != 0.0 {
        out.push(Some(OverlayPoint { lat: home.lat, lng: home.lng, alt: home.alt as f64, row: None, roi: false }));
    }
    let home_alt = |w: &Locationwp| match w.frame {
        0 | 5 => 0.0,
        10 | 11 => terrain(w.lat, w.lng).unwrap_or(-999.0),
        _ => home.alt as f64,
    };
    for (a, w) in items.iter().enumerate() {
        let c = w.id;
        if c == 0 {
            out.push(None);
            continue;
        }
        let nav = c < LAST && ![rtl, continue_alt, delay, guided_enable].contains(&c);
        if !(nav || c == roi || c == land_start || c == return_path) {
            // DO commands (DO_JUMP and the rest) leave no located point
            if c == mavcmd::DO_JUMP {
                out.push(None);
            }
            continue;
        }
        let located = w.lat != 0.0 && w.lng != 0.0;
        let pt = |roi: bool| Some(OverlayPoint { lat: w.lat, lng: w.lng, alt: w.alt as f64 + home_alt(w), row: Some(a), roi });
        if (c == land || c == vtol_land) && !located {
            continue;
        }
        if c == roi {
            out.push(pt(true));
        } else if c == land_start {
            if located {
                out.push(pt(false));
            }
        } else if located {
            out.push(pt(false));
        } else {
            out.push(None);
        }
    }
    out
}

/// setgradanddistandaz: Grad, Angle, Dist (slant, in the distance unit) and AZ text per row,
/// each leg measured from the previous point (home first; an ROI point counts as previous).
fn grad_dist_az(home: &Locationwp, points: &[Option<OverlayPoint>], rows: usize, dist_mult: f64) -> Vec<Option<[String; 4]>> {
    let mut out: Vec<Option<[String; 4]>> = vec![None; rows];
    // `last` starts at the home location whether or not it is on the list
    let mut last = Some(OverlayPoint { lat: home.lat, lng: home.lng, alt: home.alt as f64, row: None, roi: false });
    for p in points.iter().flatten() {
        if let (Some(row), false, Some(l)) = (p.row, p.roi, last) {
            let a = PointLatLngAlt::new(p.lat, p.lng, p.alt);
            let b = PointLatLngAlt::new(l.lat, l.lng, l.alt);
            let height = p.alt - l.alt;
            let distance = a.distance(&b);
            let grad = height / distance;
            if row < rows {
                out[row] = Some([
                    dotnet_fixed(grad * 100.0, 1),
                    dotnet_fixed(grad.atan().to_degrees(), 1),
                    dotnet_fixed((distance * distance + height * height).sqrt() * dist_mult, 1),
                    dotnet_fixed((a.bearing(&b) + 180.0) % 360.0, 0),
                ]);
            }
        }
        last = Some(*p);
    }
    out
}

/// double.ToString("0.0") on .NET Framework: 15-digit precision with MidpointAwayFromZero rounding,
/// and NaN / infinities by name.
fn dotnet_fixed(v: f64, dp: usize) -> String {
    if v.is_nan() {
        "NaN".into()
    } else if v.is_infinite() {
        if v > 0.0 { "Infinity".into() } else { "-Infinity".into() }
    } else {
        use mp_core::rust_decimal::{Decimal, RoundingStrategy};
        let dec = Decimal::from_f64_retain(v).unwrap_or_default();
        let rounded = dec.round_dp_with_strategy(dp as u32, RoundingStrategy::MidpointAwayFromZero);
        let s = if dp == 0 {
            format!("{rounded:.0}")
        } else {
            format!("{rounded:.dp$}")
        };
        if s.trim_start_matches('-').chars().all(|c| c == '0' || c == '.') {
            if dp == 0 { "0".to_string() } else { format!("0.{:0<dp$}", "") }
        } else {
            s
        }
    }
}

#[cfg(test)]
mod grid_stat_tests {
    use super::*;

    #[test]
    fn legs_are_slant_distances_from_the_previous_point() {
        let home = Locationwp { lat: -35.0, lng: 149.0, alt: 100.0, ..Default::default() };
        let wp = |id, lat: f64, alt, frame| Locationwp { id, lat, lng: 149.0, alt, frame, ..Default::default() };
        // 0.001 deg north is ~111 m; relative alt 30 over home 100 → 130 absolute
        let items = [wp(16, -34.999, 30.0, 3), wp(201, -34.998, 0.0, 3), wp(16, -34.997, 30.0, 3), wp(20, 0.0, 0.0, 3), wp(183, 0.0, 0.0, 3)];
        let pts = overlay_points(&home, &items, |_, _| None);
        let d = grad_dist_az(&home, &pts, items.len(), 1.0);
        let row0 = d[0].as_ref().unwrap();
        let ground = PointLatLngAlt::new(-34.999, 149.0, 0.0).distance(&PointLatLngAlt::new(-35.0, 149.0, 0.0));
        assert_eq!(row0[2], format!("{:.1}", (ground * ground + 900.0).sqrt()));
        assert_eq!(row0[0], format!("{:.1}", 30.0 / ground * 100.0));
        assert_eq!(row0[3], "0", "heading north");
        assert!(d[1].is_none(), "the ROI row has no figures");
        // row 2 is measured from the ROI point (alt 100) one step south
        let row2 = d[2].as_ref().unwrap();
        assert_eq!(row2[0], format!("{:.1}", 30.0 / ground * 100.0));
        assert!(d[3].is_none() && d[4].is_none(), "RTL and DO commands");
    }

    #[test]
    fn command_defaults_follow_selectionchangecommitted() {
        let mut w = Locationwp { id: 16, lat: 1.0, lng: 2.0, alt: 0.0, ..Default::default() };
        apply_command_defaults(&mut w, "TAKEOFF", 50.0);
        assert_eq!((w.lat, w.lng, w.alt), (0.0, 0.0, 50.0));
        apply_command_defaults(&mut w, "LAND", 50.0);
        assert_eq!(w.alt, 0.0);
        apply_command_defaults(&mut w, "DO_DIGICAM_CONTROL", 50.0);
        assert_eq!(w.lat, 1.0);
        assert_eq!(dotnet_fixed(f64::NAN, 1), "NaN");
        assert_eq!(dotnet_fixed(f64::INFINITY, 1), "Infinity");
        assert_eq!(dotnet_fixed(-0.04, 1), "0.0");
        assert_eq!(dotnet_fixed(0.25, 1), "0.3");
        assert_eq!(dotnet_fixed(0.75, 1), "0.8");
        assert_eq!(dotnet_fixed(-0.25, 1), "-0.3");
        assert_eq!(dotnet_fixed(180.5, 0), "181");
        assert_eq!(dotnet_fixed(0.0, 0), "0");
    }
}

#[cfg(test)]
mod write_check_tests {
    use super::*;

    #[test]
    fn rows_fail_like_but_write_click() {
        let wp = |id, alt| Locationwp { id, lat: -35.0, lng: 149.0, alt, frame: 3, ..Default::default() };
        assert_eq!(write_row_check(&wp(16, 5.0), 2, 10.0, Firmware::ArduCopter2), RowCheck::Fail("Low alt on WP#3\nPlease reduce the alt warning, or increase the altitude".into()));
        // takeoff, land and RTL may be low; DO commands are not checked
        for id in [22, 21, 20, 183] {
            assert_eq!(write_row_check(&wp(id, 5.0), 0, 10.0, Firmware::ArduCopter2), RowCheck::Pass);
        }
        let mut bad = wp(16, 50.0);
        bad.p1 = f32::NAN;
        assert_eq!(write_row_check(&bad, 0, 0.0, Firmware::ArduCopter2), RowCheck::Fail("There are errors in your mission".into()));
        assert_eq!(write_row_check(&wp(16, 0.0), 0, 0.0, Firmware::ArduCopter2), RowCheck::ZeroAlt { plane: false });
        assert_eq!(write_row_check(&wp(31, 0.0), 0, 0.0, Firmware::ArduPlane), RowCheck::ZeroAlt { plane: true });
        assert_eq!(write_row_check(&wp(16, 0.0), 0, 0.0, Firmware::ArduRover), RowCheck::Pass, "only flying vehicles");
        assert_eq!(write_row_check(&wp(22, 0.0), 0, 0.0, Firmware::ArduCopter2), RowCheck::Pass);
    }

    #[test]
    fn undo_restores_each_recorded_step() {
        let mut v = FlightPlanView::default();
        let wp = |lat| Locationwp { id: 16, lat, lng: 149.0, alt: 10.0, ..Default::default() };
        v.items = vec![wp(-35.0)];
        v.update_undo_buffer();
        v.items.push(wp(-35.1));
        v.update_undo_buffer();
        v.items.remove(0);
        v.undo();
        assert_eq!(v.items.len(), 2);
        v.undo();
        assert_eq!(v.items, vec![wp(-35.0)]);
        v.undo();
        assert_eq!(v.items, vec![wp(-35.0)], "an empty history leaves the list alone");
    }

    #[test]
    fn show_again_keys_and_links() {
        assert_eq!(show_again_key("Zero Altitude Warning Plane"), "SHOWAGAIN_Zero_Altitude_Warning_Plane");
        let mut planner = std::collections::BTreeMap::new();
        assert!(show_again(&planner, "Zero Altitude Warning"));
        planner.insert("SHOWAGAIN_Zero_Altitude_Warning".to_string(), "False".to_string());
        assert!(!show_again(&planner, "Zero Altitude Warning"));
        let (body, link) = split_link(&zero_alt_text(true, 4));
        assert!(body.starts_with("WP# 4 has zero altitude.") && body.ends_with("(use a simulator to be sure)."));
        assert_eq!(link, Some(("https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html".to_string(), "Command Documentation".to_string())));
        assert_eq!(split_link("plain").1, None);
    }

    #[test]
    fn post_upload_pre_caches_terrain_and_writes_radius() {
        let mut v = FlightPlanView::default();
        v.items = vec![Locationwp { id: 16, lat: -35.36, lng: 149.16, alt: 50.0, frame: 3, ..Default::default() }];
        v.post_upload = Some(std::time::Instant::now());

        let pts: Vec<(f64, f64)> = v.items.iter().filter(|w| w.lat != 0.0 && w.lng != 0.0).map(|w| (w.lat, w.lng)).collect();
        assert_eq!(pts, vec![(-35.36, 149.16)]);

        let port = std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
        let link = mp_core::link::Link::connect_with_params(mp_core::comms::ConnectionSpec::UdpListen { port }, None, false);
        {
            let mut l = link.shared.write();
            l.params.insert("WPNAV_RADIUS".into(), mp_core::link::Param { value: 200.0, ptype: 9, index: 0 });
        }
        let have = |n: &str| link.shared.read().params.contains_key(n);
        assert!(have("WPNAV_RADIUS"));
        assert!(v.post_upload.take().is_some());
        assert!(v.post_upload.is_none());
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
            self.edit_cell_with(header, text, egui::Key::Enter);
        }

        fn edit_cell_with(&mut self, header: &str, text: &str, end: egui::Key) {
            // the grid starts at y = 220; the selector above may show the same text
            let h = self.labels.iter().find(|(t, p)| t == header && p.y > 220.0).map(|l| l.1).unwrap_or_else(|| panic!("{header} column hidden: {:?}", self.labels));
            self.click(h + Vec2::new(0.0, 22.0));
            let key = |key, modifiers| egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers };
            self.frame(vec![key(egui::Key::A, egui::Modifiers::COMMAND)]);
            self.frame(vec![egui::Event::Text(text.into())]);
            self.frame(vec![key(end, egui::Modifiers::NONE)]);
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
        let cell = |v: f64| mp_core::dotnet::format_fixed(v, 1, 3, mp_core::dotnet::current_culture());
        h.edit_cell("Easting", &cell(east + 1000.0));
        // convertFromUTM reads both cells back as typed: the expected point uses the same texts
        let back = |s: String| mp_core::dotnet::double_parse(&s).unwrap();
        let (lat, lng) = mp_core::coords::utm_signed_to_geo(zone, back(cell(east + 1000.0)), back(cell(north))).unwrap();
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
        h.edit_cell_with("MGRS", &mp_core::coords::geo_to_mgrs(-35.37, 149.17).unwrap(), egui::Key::Escape);
        assert_eq!((h.view.items[0].lat, h.view.items[0].lng), (lat, lng), "Escape cancels the cell edit");

        h.choose_system("MGRS", "GEO");
        assert!(h.label("MGRS").is_none() && h.label("Zone").is_none() && h.label("GEO").is_some());
    }
}


/// Commands columns by their control names in FlightPlanner.resx.
const COL_CONTROLS: [&str; 21] = ["", "Command", "Param1", "Param2", "Param3", "Param4", "Lat", "Lon", "Alt", "Frame", "coordZone", "coordEasting", "coordNorthing", "MGRS", "Delete", "Up", "Down", "Grad", "Angle", "Dist", "AZ"];

/// A column header: P1-P4/Lat/Lon/Alt come from the command's parameter names
/// (ChangeColumnHeader, untranslated); the rest from the column's own resource.
fn header_text(col: usize, english: &str) -> String {
    if (2..=8).contains(&col) {
        return english.to_string();
    }
    COL_CONTROLS.get(col).and_then(|c| crate::i18n::control_text("FlightPlanner", &format!("{c}.HeaderText"))).unwrap_or_else(|| english.to_string())
}

/// FlightPlanner.FormatDistance: metres/kilometres, or feet/miles with Settings "distunits".
fn format_distance(planner: &std::collections::BTreeMap<String, String>, km: f64, to_meter_or_feet: bool) -> String {
    if planner.get("distunits").map(|u| u == "Feet").unwrap_or(false) {
        if to_meter_or_feet { format!("{:.2} ft", km * 3280.8399) } else { format!("{:.4} miles", km * 0.621371) }
    } else if to_meter_or_feet {
        format!("{:.2} m", km * 1000.0)
    } else {
        format!("{km:.4} km")
    }
}

#[cfg(test)]
mod overlay_tests {
    #[test]
    fn format_distance_follows_the_unit_setting() {
        let mut pl = std::collections::BTreeMap::new();
        assert_eq!(super::format_distance(&pl, 0.95, false), "0.9500 km");
        assert_eq!(super::format_distance(&pl, 0.2, true), "200.00 m");
        pl.insert("distunits".to_string(), "Feet".to_string());
        assert_eq!(super::format_distance(&pl, 1.0, true), "3280.84 ft");
        assert_eq!(super::format_distance(&pl, 1.0, false), "0.6214 miles");
    }
}
