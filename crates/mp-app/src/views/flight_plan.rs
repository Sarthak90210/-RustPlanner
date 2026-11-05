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

const HOME_IDX: usize = usize::MAX;

/// Commands DataGridView columns: (header, width) from FlightPlanner.resx.
const COLS: [(&str, f32); 17] = [
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
    ("Delete", 40.0),
    ("Up", 30.0),
    ("Down", 30.0),
    ("Grad %", 35.0),
    ("Angle", 35.0),
    ("Dist", 35.0),
    ("AZ", 30.0),
];

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
    grid: bool,
    alt_warn: f32,
    mission_type: usize,
    waypoints_collapsed: bool,
    wpfile: String,
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
            grid: false,
            alt_warn: 0.0,
            mission_type: 0,
            waypoints_collapsed: false,
            wpfile: "...".into(),
        }
    }
}

fn put_button(ui: &mut egui::Ui, r: Rect, text: &str) -> bool {
    ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, text, r.size())).clicked()
}

impl FlightPlanView {
    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        match e {
            LinkEvent::MissionDownloaded { mission_type: 0, items } => {
                if items.is_empty() {
                    toasts.error("No waypoints received");
                    return;
                }
                self.home = items[0];
                self.items = items[1..].to_vec();
                self.selected = None;
                toasts.info(format!("Got {} waypoints", self.items.len()));
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
        Locationwp { id, lat, lng, alt: app.settings.default_alt, frame: app.settings.alt_frame, ..Default::default() }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        if self.home.lat == 0.0 && self.home.lng == 0.0 {
            if let Some(h) = app.link.and_then(|l| l.shared.read().cs.home) {
                self.home.lat = h.lat;
                self.home.lng = h.lng;
                self.home.alt = h.alt as f32;
            }
        }

        egui::Panel::bottom("panelWaypoints")
            .resizable(!self.waypoints_collapsed)
            .default_size(166.0)
            .min_size(40.0)
            .frame(egui::Frame::new().fill(theme::BG))
            .show(ui, |ui| self.waypoints_panel(ui, app));

        egui::Panel::right("panelAction")
            .resizable(false)
            .exact_size(131.0)
            .frame(egui::Frame::new().fill(theme::BG))
            .show(ui, |ui| self.action_panel(ui, app));

        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::BG)).show(ui, |ui| self.map_ui(ui, app));

        if self.browser_open {
            self.file_browser(ui.ctx(), app);
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

        // panel4 (3,3 127x55): coords1, the mouse location
        let (mlat, mlng) = self.mouse.unwrap_or((0.0, 0.0));
        p.text(at(6.0, 4.0), egui::Align2::LEFT_TOP, format!("Lat  {mlat:.7}"), f.clone(), white);
        p.text(at(6.0, 19.0), egui::Align2::LEFT_TOP, format!("Lng  {mlng:.7}"), f.clone(), white);
        p.text(at(6.0, 34.0), egui::Align2::LEFT_TOP, "Alt  0", f.clone(), white);
        p.text(at(100.0, 4.0), egui::Align2::LEFT_TOP, "GEO", f.clone(), white);

        // panel3 (3,64 123x127)
        let px = 3.0;
        let py = 64.0;
        ui.put(r(px + 3.0, py + 3.0, 50.0, 17.0), egui::Checkbox::new(&mut self.grid, "Grid"));
        ui.put(r(px + 62.0, py + 3.0, 60.0, 13.0), egui::Hyperlink::from_label_and_url("View KML", "https://ardupilot.org/planner/"));
        ui.put(r(px + 3.0, py + 22.0, 111.0, 21.0), |ui: &mut egui::Ui| {
            let mut prov = app.settings.fp_map.provider;
            provider_combo(ui, &mut prov, "comboBoxMapType");
            app.settings.fp_map.provider = prov;
            ui.allocate_response(Vec2::ZERO, Sense::hover())
        });
        p.text(at(px + 4.0, py + 46.0), egui::Align2::LEFT_TOP, "Status", f.clone(), white);
        if put_button(ui, r(px + 3.0, py + 68.0, 115.0, 23.0), "Inject Custom Map") {
            app.toasts.error("Inject Custom Map: not available in this port yet");
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
                Some(l) if read => l.send(LinkCommand::ReadMission { mission_type: self.mission_type as u8 }),
                Some(l) => {
                    let mut all = vec![self.home];
                    all.extend(self.items.iter().copied());
                    l.send(LinkCommand::WriteMission { mission_type: self.mission_type as u8, items: all });
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
        ui.put(r(23.0, 20.0, 36.0, 20.0), egui::DragValue::new(&mut app.settings.wp_radius).range(0.0..=1000.0));
        p.text(o + Vec2::new(76.0, 4.0), egui::Align2::LEFT_TOP, "Loiter Radius", f.clone(), white);
        ui.put(r(89.0, 20.0, 36.0, 20.0), egui::DragValue::new(&mut app.settings.loiter_radius).range(0.0..=10000.0));
        p.text(o + Vec2::new(151.0, 4.0), egui::Align2::LEFT_TOP, "Default Alt", f.clone(), white);
        ui.put(r(154.0, 20.0, 40.0, 20.0), egui::DragValue::new(&mut app.settings.default_alt).range(0.0..=10000.0));
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
        ui.put(r(589.0, 12.0, 70.0, 17.0), egui::Checkbox::new(&mut self.mavftp, "MAVFTP"));
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
                headers[*col] = l.labels[k].clone();
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

        let total_w: f32 = COLS.iter().map(|c| c.1).sum();
        let (hdr_h, row_h) = (21.0, 22.0);
        let n = self.items.len();
        let (area, _) = ui.allocate_exact_size(Vec2::new(total_w.max(ui.available_width()), hdr_h + row_h * n as f32), Sense::hover());
        let p = ui.painter().clone();
        let f = fonts::ui_id(fonts::UI_SIZE);
        let grid_line = Color32::from_rgb(0xa0, 0xa0, 0xa0);
        p.rect_filled(area, 0.0, theme::BG);

        let mut x = area.left();
        for (i, (_, w)) in COLS.iter().enumerate() {
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
                theme::BG
            } else {
                theme::CONTROL_BG
            };
            let mut cells = Vec::with_capacity(COLS.len());
            let mut x = area.left();
            for (_, w) in COLS.iter() {
                cells.push(Rect::from_min_size(Pos2::new(x, y), Vec2::new(*w, row_h)));
                x += w;
            }
            for (c, r) in cells.iter().enumerate() {
                p.rect_filled(*r, 0.0, if c == 0 { theme::BG } else { row_bg });
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
            for (k, v) in [&mut w.p1, &mut w.p2, &mut w.p3, &mut w.p4].into_iter().enumerate() {
                cell_edit(ui, cells[2 + k], egui::DragValue::new(v).speed(0.1).max_decimals(4));
            }
            cell_edit(ui, cells[6], egui::DragValue::new(&mut w.lat).speed(0.00001).fixed_decimals(7));
            cell_edit(ui, cells[7], egui::DragValue::new(&mut w.lng).speed(0.00001).fixed_decimals(7));
            cell_edit(ui, cells[8], egui::DragValue::new(&mut w.alt).speed(0.5).max_decimals(2));
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
            let dr = cells[10].shrink(2.0);
            if ui.put(dr, |ui: &mut egui::Ui| system_button(ui, "X", dr.size())).clicked() {
                delete = Some(i);
            }
            for (c, tex, dir) in [(11usize, up, -1i32), (12, down, 1)] {
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
                for (k, v) in [g, a, d, az].iter().enumerate() {
                    let txt = if k >= 2 { format!("{v:.0}") } else { format!("{v:.1}") };
                    p.text(cells[13 + k].left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, txt, f.clone(), Color32::WHITE);
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

        let mut add_at = None;
        let mut new_drag = self.dragging;
        let mut moved: Option<(usize, f64, f64)> = None;
        let mut select = None;
        let mut ctx_pos = None;
        let mut mouse = None;
        let items = self.items.clone();

        let out = map::show(ui, &mut app.settings.fp_map, app.tiles, |_ui, view, resp, painter| {
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
            if let Some(cs) = &cs {
                if cs.lat != 0.0 || cs.lng != 0.0 {
                    map::draw_quad(painter, view.to_screen(cs.lat, cs.lng), cs.yaw, cs.sysid.max(1));
                }
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
        if let Some((lat, lng)) = add_at {
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
            p.rect_filled(r, 0.0, theme::BG);
            p.galley(r.min, g, Color32::WHITE);
        }
        ui.put(Rect::from_min_size(o + Vec2::new(w - 177.0, 3.0), Vec2::new(121.0, 21.0)), |ui: &mut egui::Ui| {
            let names = ["Mission", "Fence", "Rally"];
            egui::ComboBox::from_id_salt("cmb_missiontype")
                .width(113.0)
                .selected_text(names[self.mission_type])
                .show_ui(ui, |ui| {
                    for (i, n) in names.iter().enumerate() {
                        ui.selectable_value(&mut self.mission_type, i, *n);
                    }
                })
                .response
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

    /// contextMenuStrip1, in Mission Planner's order.
    fn context_menu(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let Some((lat, lng, hit)) = self.ctx_pos else { return };
        if ui.button("Delete WP").clicked() {
            if let Some(i) = hit.or(self.selected).filter(|i| *i < self.items.len()) {
                self.items.remove(i);
                self.selected = None;
            }
            ui.close();
        }
        if ui.button("Insert Wp").clicked() {
            let at = hit.or(self.selected).unwrap_or(self.items.len()).min(self.items.len());
            let wp = self.new_wp(app, mavcmd::WAYPOINT, lat, lng);
            self.items.insert(at, wp);
            ui.close();
        }
        if ui.button("Insert Spline WP").clicked() {
            let at = hit.or(self.selected).unwrap_or(self.items.len()).min(self.items.len());
            let wp = self.new_wp(app, mavcmd::SPLINE_WAYPOINT, lat, lng);
            self.items.insert(at, wp);
            ui.close();
        }
        ui.menu_button("Loiter", |ui| {
            if ui.button("Forever").clicked() {
                let wp = self.new_wp(app, mavcmd::LOITER_UNLIM, lat, lng);
                self.items.push(wp);
                ui.close();
            }
            if ui.button("Time").clicked() {
                let mut wp = self.new_wp(app, mavcmd::LOITER_TIME, lat, lng);
                wp.p1 = 5.0;
                self.items.push(wp);
                ui.close();
            }
            if ui.button("Circles").clicked() {
                let mut wp = self.new_wp(app, mavcmd::LOITER_TURNS, lat, lng);
                wp.p1 = 1.0;
                self.items.push(wp);
                ui.close();
            }
        });
        ui.menu_button("Jump", |ui| {
            if ui.button("Start").clicked() {
                self.items.push(Locationwp { id: mavcmd::DO_JUMP, p1: 1.0, p2: -1.0, frame: app.settings.alt_frame, ..Default::default() });
                ui.close();
            }
            if ui.button("WP #").clicked() {
                let target = self.selected.map(|s| s as f32 + 1.0).unwrap_or(1.0);
                self.items.push(Locationwp { id: mavcmd::DO_JUMP, p1: target, p2: 1.0, frame: app.settings.alt_frame, ..Default::default() });
                ui.close();
            }
        });
        if ui.button("RTL").clicked() {
            self.items.push(Locationwp { id: mavcmd::RETURN_TO_LAUNCH, frame: app.settings.alt_frame, ..Default::default() });
            ui.close();
        }
        if ui.button("Land").clicked() {
            let mut wp = self.new_wp(app, mavcmd::LAND, lat, lng);
            wp.alt = 0.0;
            self.items.push(wp);
            ui.close();
        }
        if ui.button("Takeoff").clicked() {
            let wp = self.new_wp(app, mavcmd::TAKEOFF, 0.0, 0.0);
            self.items.insert(0, wp);
            ui.close();
        }
        if ui.button("DO_SET_ROI").clicked() {
            let mut wp = self.new_wp(app, mavcmd::DO_SET_ROI, lat, lng);
            wp.alt = 0.0;
            self.items.push(wp);
            ui.close();
        }
        if ui.button("Fence Circle Around Home").clicked() {
            app.toasts.error("Fence Circle Around Home: not available in this port yet");
            ui.close();
        }
        if ui.button("Clear Mission").clicked() {
            self.items.clear();
            self.selected = None;
            ui.close();
        }
        ui.separator();
        for sub in ["Polygon", "Geo-Fence", "Rally Points", "Auto WP", "Map Tool"] {
            ui.menu_button(sub, |ui| {
                ui.add_enabled(false, egui::Button::new("Not available in this port yet"));
            });
        }
        ui.menu_button("File Load/Save", |ui| {
            if ui.button("Save WP File").clicked() {
                let text = mission::write_waypoint_file(&self.home, &self.items);
                if let Some(Err(e)) = platform::save_text("Save Waypoint File", "mission.waypoints", "Waypoints", &["waypoints"], &text) {
                    app.toasts.error(e);
                }
                ui.close();
            }
            if ui.button("Load WP File").clicked() {
                if let Some((name, text)) = platform::open_text("Open Waypoint File", "Waypoints", &["waypoints", "txt"]) {
                    self.load_text(&name, &text, app.toasts);
                }
                ui.close();
            }
        });
        for sub in ["POI", "Tracker Home"] {
            ui.menu_button(sub, |ui| {
                ui.add_enabled(false, egui::Button::new("Not available in this port yet"));
            });
        }
        if ui.button("Modify Alt").clicked() {
            for w in &mut self.items {
                if w.has_location() {
                    w.alt = app.settings.default_alt;
                }
            }
            ui.close();
        }
        ui.add_enabled(false, egui::Button::new("Enter UTM Coord"));
        ui.add_enabled(false, egui::Button::new("Switch Docking"));
        if ui.button("Set Home Here").clicked() {
            self.home.lat = lat;
            self.home.lng = lng;
            ui.close();
        }
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
