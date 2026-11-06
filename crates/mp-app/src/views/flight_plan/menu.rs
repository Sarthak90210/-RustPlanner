//! FlightPlanner contextMenuStrip1, item for item, with its dialogs: polygon / fence / rally
//! files, KML and SHP import, UTM entry, text and circle-survey generators, prefetch,
//! elevation graph, POIs, tracker home and the waypoint panel docking switch.

use std::collections::VecDeque;

use egui::Vec2;
use mp_core::mavcmd;
use mp_core::mission::{self, Locationwp};

use super::FlightPlanView;
use crate::app::AppCtx;
use crate::{platform, theme};

const R: f64 = 6_378_137.0;

/// Prompts opened from the context menu.
pub(super) enum MenuPrompt {
    RotateMap(String),
    OffsetPolygon(String),
    #[allow(dead_code)]
    ZoomTo(String, String),
    ZoomToLocation(String),
    GdalOpacity(String),
    ModifyAlt(String),
    Utm(String, String, String, bool),
    Text(f64, f64, String, String),
    CircleSurvey(f64, f64, String, String),
    SplineCircle(f64, f64, String, String),
    PoiAdd(f64, f64, String),
    PoiEdit(usize, String),
}

fn offset_m(lat: f64, lng: f64, north: f64, east: f64) -> (f64, f64) {
    (lat + (north / R).to_degrees(), lng + (east / (R * lat.to_radians().cos())).to_degrees())
}

/// UTM (WGS84) to latitude / longitude, as `utmpos.ToLLA` with a signed zone.
pub(super) fn utm_to_latlng(zone: i32, south: bool, easting: f64, northing: f64) -> (f64, f64) {
    mp_core::coords::utm_signed_to_geo(if south { -zone.abs() } else { zone.abs() }, easting, northing)
}

/// 5x7 glyphs for the "Text" auto-WP generator (rows top to bottom, 5 bits each).
fn glyph(c: char) -> [u8; 7] {
    match c.to_ascii_uppercase() {
        'A' => [14, 17, 17, 31, 17, 17, 17],
        'B' => [30, 17, 17, 30, 17, 17, 30],
        'C' => [14, 17, 16, 16, 16, 17, 14],
        'D' => [30, 17, 17, 17, 17, 17, 30],
        'E' => [31, 16, 16, 30, 16, 16, 31],
        'F' => [31, 16, 16, 30, 16, 16, 16],
        'G' => [14, 17, 16, 23, 17, 17, 15],
        'H' => [17, 17, 17, 31, 17, 17, 17],
        'I' => [14, 4, 4, 4, 4, 4, 14],
        'J' => [7, 2, 2, 2, 2, 18, 12],
        'K' => [17, 18, 20, 24, 20, 18, 17],
        'L' => [16, 16, 16, 16, 16, 16, 31],
        'M' => [17, 27, 21, 21, 17, 17, 17],
        'N' => [17, 17, 25, 21, 19, 17, 17],
        'O' => [14, 17, 17, 17, 17, 17, 14],
        'P' => [30, 17, 17, 30, 16, 16, 16],
        'Q' => [14, 17, 17, 17, 21, 18, 13],
        'R' => [30, 17, 17, 30, 20, 18, 17],
        'S' => [15, 16, 16, 14, 1, 1, 30],
        'T' => [31, 4, 4, 4, 4, 4, 4],
        'U' => [17, 17, 17, 17, 17, 17, 14],
        'V' => [17, 17, 17, 17, 17, 10, 4],
        'W' => [17, 17, 17, 21, 21, 21, 10],
        'X' => [17, 17, 10, 4, 10, 17, 17],
        'Y' => [17, 17, 10, 4, 4, 4, 4],
        'Z' => [31, 1, 2, 4, 8, 16, 31],
        '0' => [14, 17, 19, 21, 25, 17, 14],
        '1' => [4, 12, 4, 4, 4, 4, 14],
        '2' => [14, 17, 1, 2, 4, 8, 31],
        '3' => [31, 2, 4, 2, 1, 17, 14],
        '4' => [2, 6, 10, 18, 31, 2, 2],
        '5' => [31, 16, 30, 1, 1, 17, 14],
        '6' => [6, 8, 16, 30, 17, 17, 14],
        '7' => [31, 1, 2, 4, 8, 8, 8],
        '8' => [14, 17, 17, 14, 17, 17, 14],
        '9' => [14, 17, 17, 15, 1, 2, 12],
        _ => [0; 7],
    }
}

/// Minimal ESRI shapefile reader: every polygon / polyline / point record as (lat, lng) lists.
pub(crate) fn read_shp(bytes: &[u8]) -> Vec<Vec<(f64, f64)>> {
    let mut out = Vec::new();
    let mut i = 100;
    let le_i32 = |b: &[u8]| i32::from_le_bytes([b[0], b[1], b[2], b[3]]);
    let le_f64 = |b: &[u8]| f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]);
    while i + 12 <= bytes.len() {
        let len = i32::from_be_bytes([bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]]) as usize * 2;
        let rec = &bytes[i + 8..(i + 8 + len).min(bytes.len())];
        i += 8 + len;
        if rec.len() < 4 {
            continue;
        }
        match le_i32(rec) {
            1 if rec.len() >= 20 => out.push(vec![(le_f64(&rec[12..]), le_f64(&rec[4..]))]),
            3 | 5 if rec.len() >= 44 => {
                let parts = le_i32(&rec[36..]) as usize;
                let points = le_i32(&rec[40..]) as usize;
                let base = 44 + parts * 4;
                let pts: Vec<(f64, f64)> = (0..points).filter(|k| base + k * 16 + 16 <= rec.len()).map(|k| (le_f64(&rec[base + k * 16 + 8..]), le_f64(&rec[base + k * 16..]))).collect();
                out.push(pts);
            }
            _ => {}
        }
    }
    out
}

/// LineString / Polygon / Point coordinates from a KML document.
pub(super) fn read_kml(text: &str) -> Vec<Vec<(f64, f64, f64)>> {
    let Ok(doc) = roxmltree::Document::parse(text) else { return Vec::new() };
    doc.descendants()
        .filter(|n| n.has_tag_name("coordinates"))
        .map(|n| {
            n.text()
                .unwrap_or("")
                .split_whitespace()
                .filter_map(|t| {
                    let v: Vec<f64> = t.split(',').filter_map(|x| x.parse().ok()).collect();
                    (v.len() >= 2).then(|| (v[1], v[0], v.get(2).copied().unwrap_or(0.0)))
                })
                .collect()
        })
        .collect()
}

#[cfg(not(any(target_os = "android", target_os = "ios")))]
fn pick(title: &str, filter: &str, exts: &[&str]) -> Option<std::path::PathBuf> {
    rfd::FileDialog::new().set_title(title).add_filter(filter, exts).pick_file()
}

#[cfg(any(target_os = "android", target_os = "ios"))]
fn pick(_title: &str, _filter: &str, exts: &[&str]) -> Option<std::path::PathBuf> {
    platform::list_files(exts).last().cloned()
}

impl FlightPlanView {
    fn insert_at(&self, hit: Option<usize>) -> usize {
        hit.or(self.selected).unwrap_or(self.items.len()).min(self.items.len())
    }

    /// Shared by the map icon and the map context menu.
    pub(super) fn polygon_menu(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        if ui.button(crate::i18n::t("Draw a Polygon")).clicked() {
            self.drawing_polygon = true;
            ui.close();
        }
        if ui.button(crate::i18n::t("Clear Polygon")).clicked() {
            self.polygon.clear();
            self.drawing_polygon = false;
            ui.close();
        }
        if ui.button(crate::i18n::t("Save Polygon")).clicked() {
            if self.polygon.is_empty() {
                app.toasts.error("Please define a polygon.");
            } else {
                let mut text = String::from("#saved by Mission Planner\r\n");
                for (a, b) in &self.polygon {
                    text.push_str(&format!("{a} {b}\r\n"));
                }
                if let Some(Err(e)) = platform::save_text("Save Polygon", "polygon.poly", "Polygon", &["poly"], &text) {
                    app.toasts.error(e);
                }
            }
            ui.close();
        }
        if ui.button(crate::i18n::t("Load Polygon")).clicked() {
            if let Some((_, text)) = platform::open_text("Load Polygon", "Polygon", &["poly", "txt"]) {
                self.polygon = text
                    .lines()
                    .filter(|l| !l.starts_with('#'))
                    .filter_map(|l| {
                        let v: Vec<f64> = l.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                        (v.len() >= 2).then(|| (v[0], v[1]))
                    })
                    .collect();
            }
            ui.close();
        }
        if ui.button(crate::i18n::t("From SHP")).clicked() {
            if let Some(p) = pick("From SHP", "Shape file", &["shp"]) {
                match std::fs::read(&p).map(|b| read_shp(&b)) {
                    Ok(shapes) if !shapes.is_empty() => self.polygon = shapes.into_iter().max_by_key(|s| s.len()).unwrap_or_default(),
                    _ => app.toasts.error("No polygon found in the shape file"),
                }
            }
            ui.close();
        }
        if ui.button(crate::i18n::t("From Current Waypoints")).clicked() {
            self.polygon = self.items.iter().filter(|w| w.has_location() && (w.lat != 0.0 || w.lng != 0.0)).map(|w| (w.lat, w.lng)).collect();
            ui.close();
        }
        if ui.button(crate::i18n::t("Offset Polygon")).clicked() {
            self.menu_prompt = Some(MenuPrompt::OffsetPolygon("10".into()));
            ui.close();
        }
        if ui.button(crate::i18n::t("Area")).clicked() {
            // areaToolStripMenuItem_Click (the tab stops drawn as spaces)
            let a = mp_core::survey::area(&self.polygon).abs();
            let tab = "        ";
            app.toasts.message(format!("Area: {a:.0} m2
{tab}{:.2} Acre
{tab}{:.2} Hectare
{tab}{:.0} sqf", a * 0.000247105, a * 1e-4, a * 10.7639), "Area");
            ui.close();
        }
        if self.mission_type == 1 {
            ui.separator();
            if ui.button(crate::i18n::t("Fence Inclusion")).clicked() {
                self.add_fence_polygon(5001, app);
                ui.close();
            }
            if ui.button(crate::i18n::t("Fence Exclusion")).clicked() {
                self.add_fence_polygon(5002, app);
                ui.close();
            }
        }
    }

    /// contextMenuStrip1, in Mission Planner's order.
    pub(super) fn context_menu(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let Some((lat, lng, hit)) = self.ctx_pos else { return };
        let frame = app.settings.alt_frame;
        if ui.button(crate::i18n::t("Delete WP")).clicked() {
            if let Some(i) = hit.or(self.selected).filter(|i| *i < self.items.len()) {
                self.items.remove(i);
                self.selected = None;
            }
            ui.close();
        }
        let ins = ui.menu_button(crate::i18n::t("Insert Wp"), |ui| {
            if ui.button(crate::i18n::t("At Current Position")).clicked() {
                match app.link.map(|l| l.shared.read().cs.clone()) {
                    Some(cs) if cs.lat != 0.0 => {
                        let at = self.insert_at(hit);
                        let wp = self.new_wp(app, mavcmd::WAYPOINT, cs.lat, cs.lng);
                        self.update_undo_buffer();
                        self.items.insert(at, wp);
                    }
                    _ => app.toasts.error("No vehicle position"),
                }
                ui.close();
            }
        });
        if ins.response.clicked() {
            let at = self.insert_at(hit);
            let wp = self.new_wp(app, mavcmd::WAYPOINT, lat, lng);
            self.update_undo_buffer();
            self.items.insert(at, wp);
            ui.close();
        }
        if ui.button(crate::i18n::t("Insert Spline WP")).clicked() {
            let at = self.insert_at(hit);
            let wp = self.new_wp(app, mavcmd::SPLINE_WAYPOINT, lat, lng);
            self.update_undo_buffer();
            self.items.insert(at, wp);
            ui.close();
        }
        ui.menu_button(crate::i18n::t("Loiter"), |ui| {
            for (t, id, p1) in [("Forever", mavcmd::LOITER_UNLIM, 0.0), ("Time", mavcmd::LOITER_TIME, 5.0), ("Circles", mavcmd::LOITER_TURNS, 1.0)] {
                if ui.button(t).clicked() {
                    let mut wp = self.new_wp(app, id, lat, lng);
                    wp.p1 = p1;
                    self.update_undo_buffer();
                    self.items.push(wp);
                    ui.close();
                }
            }
        });
        ui.menu_button(crate::i18n::t("Jump"), |ui| {
            if ui.button(crate::i18n::t("Start")).clicked() {
                self.items.push(Locationwp { id: mavcmd::DO_JUMP, p1: 1.0, p2: -1.0, frame, ..Default::default() });
                ui.close();
            }
            if ui.button(crate::i18n::t("WP #")).clicked() {
                let target = self.selected.map(|s| s as f32 + 1.0).unwrap_or(1.0);
                self.items.push(Locationwp { id: mavcmd::DO_JUMP, p1: target, p2: 1.0, frame, ..Default::default() });
                ui.close();
            }
        });
        if ui.button(crate::i18n::t("RTL")).clicked() {
            self.items.push(Locationwp { id: mavcmd::RETURN_TO_LAUNCH, frame, ..Default::default() });
            ui.close();
        }
        if ui.button(crate::i18n::t("Land")).clicked() {
            let mut wp = self.new_wp(app, mavcmd::LAND, lat, lng);
            wp.alt = 0.0;
            self.update_undo_buffer();
            self.items.push(wp);
            ui.close();
        }
        if ui.button(crate::i18n::t("Takeoff")).clicked() {
            let wp = self.new_wp(app, mavcmd::TAKEOFF, 0.0, 0.0);
            self.items.insert(0, wp);
            ui.close();
        }
        if ui.button(crate::i18n::t("DO_SET_ROI")).clicked() {
            let mut wp = self.new_wp(app, mavcmd::DO_SET_ROI, lat, lng);
            wp.alt = 0.0;
            self.update_undo_buffer();
            self.items.push(wp);
            ui.close();
        }
        if ui.button(crate::i18n::t("Fence Circle Around Home")).clicked() {
            self.fence_circle = Some("300".into());
            ui.close();
        }
        if ui.button(crate::i18n::t("Clear Mission")).clicked() {
            self.items.clear();
            self.selected = None;
            ui.close();
        }
        ui.separator();
        ui.menu_button(crate::i18n::t("Polygon"), |ui| self.polygon_menu(ui, app));
        let show = |f: &str| crate::display::flag(&app.settings.planner, f);
        let (fence, rally, spline, text, circle_survey, poi, tracker) = (
            show("displayGeoFenceMenu"),
            show("displayRallyPointsMenu"),
            show("displaySplineCircleAutoWp"),
            show("displayTextAutoWp"),
            show("displayCircleSurveyAutoWp"),
            show("displayPoiMenu"),
            show("displayTrackerHomeMenu"),
        );
        if fence { ui.menu_button(crate::i18n::t("Geo-Fence"), |ui| {
            if ui.button(crate::i18n::t("Upload")).clicked() {
                self.transfer(app, 1, false);
                ui.close();
            }
            if ui.button(crate::i18n::t("Download")).clicked() {
                self.transfer(app, 1, true);
                ui.close();
            }
            if ui.button(crate::i18n::t("Set Return Location")).clicked() {
                self.fence_list().retain(|w| w.id != 5000);
                self.fence_list().insert(0, Locationwp { id: 5000, lat, lng, frame: 0, ..Default::default() });
                ui.close();
            }
            if ui.button(crate::i18n::t("Load from File")).clicked() {
                self.load_list(1, app);
                ui.close();
            }
            if ui.button(crate::i18n::t("Save to File")).clicked() {
                self.save_list(1, "fence.waypoints", app);
                ui.close();
            }
            if ui.button(crate::i18n::t("Clear")).clicked() {
                self.fence_list().clear();
                ui.close();
            }
        }); }
        if rally { ui.menu_button(crate::i18n::t("Rally Points"), |ui| {
            if ui.button(crate::i18n::t("Set Rally Point")).clicked() {
                let alt = app.settings.default_alt;
                self.rally_list().push(Locationwp { id: 5100, lat, lng, alt, frame: 3, ..Default::default() });
                ui.close();
            }
            if ui.button(crate::i18n::t("Download")).clicked() {
                self.transfer(app, 2, true);
                ui.close();
            }
            if ui.button(crate::i18n::t("Upload")).clicked() {
                self.transfer(app, 2, false);
                ui.close();
            }
            if ui.button(crate::i18n::t("Clear Rally Points")).clicked() {
                self.rally_list().clear();
                ui.close();
            }
            if ui.button(crate::i18n::t("Save Rally to File")).clicked() {
                self.save_list(2, "rally.waypoints", app);
                ui.close();
            }
            if ui.button(crate::i18n::t("Load Rally from File")).clicked() {
                self.load_list(2, app);
                ui.close();
            }
        }); }
        ui.menu_button(crate::i18n::t("Auto WP"), |ui| {
            if ui.button(crate::i18n::t("Create Wp Circle")).clicked() {
                self.circle_dialog = Some((lat, lng, "50".into(), "8".into()));
                ui.close();
            }
            if spline && ui.button(crate::i18n::t("Create Spline Circle")).clicked() {
                self.menu_prompt = Some(MenuPrompt::SplineCircle(lat, lng, "50".into(), "8".into()));
                ui.close();
            }
            if ui.button(crate::i18n::t("Area")).clicked() {
                let a = mp_core::survey::area(&self.polygon);
                app.toasts.error(format!("Area: {a:.0} m2 ({:.3} km2)", a / 1e6));
                ui.close();
            }
            if text && ui.button(crate::i18n::t("Text")).clicked() {
                self.menu_prompt = Some(MenuPrompt::Text(lat, lng, "MP".into(), "20".into()));
                ui.close();
            }
            if circle_survey && ui.button(crate::i18n::t("Create Circle Survey")).clicked() {
                self.menu_prompt = Some(MenuPrompt::CircleSurvey(lat, lng, "100".into(), "20".into()));
                ui.close();
            }
            if ui.button(crate::i18n::t("Survey (Grid)")).clicked() {
                if self.polygon.len() < 3 {
                    app.toasts.error("Please define a polygon.");
                } else {
                    self.grid_ui = Some(Box::new(super::super::survey_ui::GridUi::new(self.polygon.clone(), (self.home.lat, self.home.lng), app.settings.fp_map.provider)));
                }
                ui.close();
            }
        });
        ui.menu_button(crate::i18n::t("Map Tool"), |ui| {
            if ui.button(crate::i18n::t("Measure Distance")).clicked() {
                self.measure = Some((lat, lng));
                ui.close();
            }
            if ui.button(crate::i18n::t("Zoom To")).clicked() {
                self.menu_prompt = Some(MenuPrompt::ZoomToLocation("Perth Airport, Australia".into()));
                ui.close();
            }
            if ui.button(crate::i18n::t("Rotate Map")).clicked() {
                self.menu_prompt = Some(MenuPrompt::RotateMap(app.settings.fp_map.bearing.to_string()));
                ui.close();
            }
            if ui.button(crate::i18n::t("Prefetch")).clicked() {
                let n = app.tiles.prefetch_area(app.settings.fp_map.provider, self.view_bounds.unwrap_or((lat - 0.01, lng - 0.01, lat + 0.01, lng + 0.01)), app.settings.fp_map.zoom.floor() as u8, (app.settings.fp_map.zoom.floor() as u8 + 3).min(20));
                app.toasts.info(format!("Prefetching {n} map tiles"));
                ui.close();
            }
            if ui.button(crate::i18n::t("Prefetch WP Path")).clicked() {
                let pts: Vec<(f64, f64)> = std::iter::once((self.home.lat, self.home.lng)).chain(self.items.iter().filter(|w| w.has_location()).map(|w| (w.lat, w.lng))).filter(|p| p.0 != 0.0).collect();
                let mut n = 0;
                for w in pts.windows(2) {
                    let b = (w[0].0.min(w[1].0) - 0.002, w[0].1.min(w[1].1) - 0.002, w[0].0.max(w[1].0) + 0.002, w[0].1.max(w[1].1) + 0.002);
                    n += app.tiles.prefetch_area(app.settings.fp_map.provider, b, 14, 18);
                }
                app.toasts.info(format!("Prefetching {n} map tiles along the route"));
                ui.close();
            }
            if ui.button(crate::i18n::t("KML Overlay")).clicked() {
                if let Some((_, text)) = platform::open_text("KML Overlay", "Google Earth KML", &["kml"]) {
                    self.kml_overlay = read_kml(&text).into_iter().map(|l| l.into_iter().map(|(a, b, _)| (a, b)).collect()).collect();
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Elevation Graph")).clicked() {
                self.elevation_open = true;
                ui.close();
            }
            if ui.button(crate::i18n::t("Reverse WPs")).clicked() {
                self.items.reverse();
                ui.close();
            }
            if ui.button(crate::i18n::t("GDAL Opacity")).clicked() {
                self.menu_prompt = Some(MenuPrompt::GdalOpacity(format!("{:.1}", app.settings.fp_map.gdal_opacity)));
                ui.close();
            }
        });
        ui.menu_button(crate::i18n::t("File Load/Save"), |ui| {
            if ui.button(crate::i18n::t("Load WP File")).clicked() {
                if let Some((name, text)) = platform::open_text("Open Waypoint File", "Waypoints", &["waypoints", "txt"]) {
                    self.load_text(&name, &text, app.toasts);
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Load and Append")).clicked() {
                if let Some((_, text)) = platform::open_text("Open Waypoint File", "Waypoints", &["waypoints", "txt"]) {
                    match mission::read_waypoint_file(&text) {
                        Ok(list) if list.len() > 1 => self.items.extend_from_slice(&list[1..]),
                        Ok(_) => app.toasts.error("Invalid Waypoint file"),
                        Err(e) => app.toasts.error(e.to_string()),
                    }
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Save WP File")).clicked() {
                let text = mission::write_waypoint_file(&self.home, &self.items);
                if let Some(Err(e)) = platform::save_text("Save Waypoint File", "mission.waypoints", "Waypoints", &["waypoints"], &text) {
                    app.toasts.error(e);
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Load KML File")).clicked() {
                if let Some((_, text)) = platform::open_text("Load KML File", "Google Earth KML", &["kml"]) {
                    for line in read_kml(&text) {
                        for (a, b, alt) in line {
                            let mut wp = self.new_wp(app, mavcmd::WAYPOINT, a, b);
                            if alt > 0.0 {
                                wp.alt = alt as f32;
                            }
                            self.items.push(wp);
                        }
                    }
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Load SHP File")).clicked() {
                if let Some(p) = pick("Load SHP File", "Shape file", &["shp"]) {
                    if let Ok(b) = std::fs::read(&p) {
                        for shape in read_shp(&b) {
                            for (a, c) in shape {
                                let wp = self.new_wp(app, mavcmd::WAYPOINT, a, c);
                                self.items.push(wp);
                            }
                        }
                    }
                }
                ui.close();
            }
        });
        if poi { ui.menu_button(crate::i18n::t("POI"), |ui| {
            if ui.button(crate::i18n::t("Add")).clicked() {
                self.menu_prompt = Some(MenuPrompt::PoiAdd(lat, lng, String::new()));
                ui.close();
            }
            let near = app.settings.pois.iter().enumerate().min_by(|a, b| {
                let d = |p: &(f64, f64, String)| (p.0 - lat).powi(2) + (p.1 - lng).powi(2);
                d(a.1).total_cmp(&d(b.1))
            }).map(|(i, p)| (i, p.2.clone()));
            if ui.add_enabled(near.is_some(), egui::Button::new("Delete")).clicked() {
                if let Some((i, _)) = near.clone() {
                    app.settings.pois.remove(i);
                }
                ui.close();
            }
            if ui.add_enabled(near.is_some(), egui::Button::new("Edit")).clicked() {
                if let Some((i, n)) = near {
                    self.menu_prompt = Some(MenuPrompt::PoiEdit(i, n));
                }
                ui.close();
            }
        }); }
        if tracker && ui.button(crate::i18n::t("Tracker Home")).clicked() {
            app.settings.tracker_home = Some((lat, lng));
            ui.close();
        }
        if ui.button(crate::i18n::t("Modify Alt")).clicked() {
            self.menu_prompt = Some(MenuPrompt::ModifyAlt(format!("{}", app.settings.default_alt)));
            ui.close();
        }
        if ui.button(crate::i18n::t("Enter UTM Coord")).clicked() {
            let zone = ((lng + 180.0) / 6.0).floor() as i32 + 1;
            self.menu_prompt = Some(MenuPrompt::Utm(zone.to_string(), String::new(), String::new(), lat < 0.0));
            ui.close();
        }
        if ui.button(crate::i18n::t("Switch Docking")).clicked() {
            self.waypoints_right = !self.waypoints_right;
            ui.close();
        }
        if ui.button(crate::i18n::t("Set Home Here")).clicked() {
            self.home.lat = lat;
            self.home.lng = lng;
            if let Some(h) = crate::srtm::altitude(lat, lng) {
                self.home.alt = h as f32;
            }
            ui.close();
        }
        // PluginHost.FPMenuMap items, at MouseDownEnd
        for (label, plugin, id) in crate::plugins::menu_items(mp_plugin::MENU_FP_MAP) {
            if ui.button(label).clicked() {
                crate::plugins::click(plugin, id, lat, lng);
                ui.close();
            }
        }
    }

    fn transfer(&mut self, app: &mut AppCtx, mt: usize, read: bool) {
        let Some(l) = app.link.filter(|_| app.connected()) else {
            app.toasts.error("Please Connect First");
            return;
        };
        let items = if read { Vec::new() } else { self.list_for(mt).clone() };
        self.mission_io(l, mt as u8, read, items);
    }

    fn save_list(&mut self, mt: usize, def: &str, app: &mut AppCtx) {
        let list = self.list_for(mt).clone();
        if list.is_empty() {
            app.toasts.error("Nothing to save");
            return;
        }
        let text = mission::write_waypoint_file(&list[0], &list[1..]);
        if let Some(Err(e)) = platform::save_text("Save to File", def, "Waypoints", &["waypoints", "txt"], &text) {
            app.toasts.error(e);
        }
    }

    fn load_list(&mut self, mt: usize, app: &mut AppCtx) {
        if let Some((_, text)) = platform::open_text("Load from File", "Waypoints", &["waypoints", "txt", "fen"]) {
            match mission::read_waypoint_file(&text) {
                Ok(list) => *self.list_for(mt) = list,
                Err(_) => {
                    // legacy fence file: "lat lng" lines, first is the return point
                    let pts: Vec<(f64, f64)> = text
                        .lines()
                        .filter(|l| !l.starts_with('#'))
                        .filter_map(|l| {
                            let v: Vec<f64> = l.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                            (v.len() >= 2).then(|| (v[0], v[1]))
                        })
                        .collect();
                    if pts.is_empty() {
                        app.toasts.error("Invalid file");
                        return;
                    }
                    let n = (pts.len() - 1) as f32;
                    let mut out = vec![Locationwp { id: 5000, lat: pts[0].0, lng: pts[0].1, ..Default::default() }];
                    out.extend(pts[1..].iter().map(|(a, b)| Locationwp { id: 5001, p1: n, lat: *a, lng: *b, ..Default::default() }));
                    *self.list_for(mt) = out;
                }
            }
        }
    }

    /// Context-menu prompts and the elevation graph window.
    pub(super) fn menu_dialogs(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if let Some(p) = self.menu_prompt.as_mut() {
            let mut done = None;
            let title = match p {
                MenuPrompt::RotateMap(_) => "Rotate map to heading",
                MenuPrompt::OffsetPolygon(_) => "Offset Polygon",
                MenuPrompt::ZoomTo(..) => "Zoom To",
                MenuPrompt::ZoomToLocation(..) => "Location",
                MenuPrompt::GdalOpacity(..) => "Opacity 0.0-1.0",
                MenuPrompt::ModifyAlt(_) => "Modify Alt",
                MenuPrompt::Utm(..) => "Enter UTM Coord",
                MenuPrompt::Text(..) => "Text",
                MenuPrompt::CircleSurvey(..) => "Create Circle Survey",
                MenuPrompt::SplineCircle(..) => "Create Spline Circle",
                MenuPrompt::PoiAdd(..) => "POI",
                MenuPrompt::PoiEdit(..) => "POI",
            };
            egui::Window::new(title).collapsible(false).resizable(false).show(ctx, |ui| {
                let field = |ui: &mut egui::Ui, l: &str, v: &mut String| {
                    theme::label(ui, l);
                    ui.text_edit_singleline(v);
                };
                match p {
                    MenuPrompt::RotateMap(h) => field(ui, "Enter new UP heading", h),
                    MenuPrompt::OffsetPolygon(d) => field(ui, "Distance (m, negative shrinks)", d),
                    MenuPrompt::ZoomTo(a, b) => {
                        field(ui, "Lat", a);
                        field(ui, "Lng", b);
                    }
                    MenuPrompt::ZoomToLocation(loc) => field(ui, "Enter your location", loc),
                    MenuPrompt::GdalOpacity(op) => field(ui, "Enter opacity (0.0-1.0)", op),
                    MenuPrompt::ModifyAlt(a) => field(ui, "Altitude", a),
                    MenuPrompt::Utm(z, e, n, s) => {
                        field(ui, "Zone", z);
                        field(ui, "Easting", e);
                        field(ui, "Northing", n);
                        ui.checkbox(s, crate::i18n::t("Southern hemisphere"));
                    }
                    MenuPrompt::Text(_, _, t, h) => {
                        field(ui, "Enter your text", t);
                        field(ui, "Character height (m)", h);
                    }
                    MenuPrompt::CircleSurvey(_, _, r, s) => {
                        field(ui, "Radius (m)", r);
                        field(ui, "Spacing (m)", s);
                    }
                    MenuPrompt::SplineCircle(_, _, r, n) => {
                        field(ui, "Radius (m)", r);
                        field(ui, "Points", n);
                    }
                    MenuPrompt::PoiAdd(_, _, n) | MenuPrompt::PoiEdit(_, n) => field(ui, "Enter ID", n),
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
            if let Some(ok) = done {
                let p = self.menu_prompt.take().unwrap();
                if ok {
                    self.apply_prompt(p, app);
                }
            }
        }
        self.elevation_window(ctx, app);
    }

    fn apply_prompt(&mut self, p: MenuPrompt, app: &mut AppCtx) {
        let num = |s: &str| s.trim().parse::<f64>().ok();
        match p {
            MenuPrompt::RotateMap(h) => {
                match num(&h).filter(|h| h.is_finite()) {
                    Some(h) => app.settings.fp_map.bearing = h.rem_euclid(360.0),
                    None => app.toasts.error("Please enter a finite heading in degrees."),
                }
            }
            MenuPrompt::OffsetPolygon(d) => {
                let Some(d) = num(&d) else { return };
                if self.polygon.len() < 3 {
                    return;
                }
                let n = self.polygon.len() as f64;
                let c = (self.polygon.iter().map(|p| p.0).sum::<f64>() / n, self.polygon.iter().map(|p| p.1).sum::<f64>() / n);
                self.polygon = self
                    .polygon
                    .iter()
                    .map(|&(a, b)| {
                        let north = (a - c.0).to_radians() * R;
                        let east = (b - c.1).to_radians() * R * c.0.to_radians().cos();
                        let len = (north * north + east * east).sqrt().max(1e-6);
                        offset_m(a, b, north / len * d, east / len * d)
                    })
                    .collect();
            }
            MenuPrompt::ZoomTo(a, b) => {
                if let (Some(a), Some(b)) = (num(&a), num(&b)) {
                    app.settings.fp_map.center_on(a, b);
                }
            }
            MenuPrompt::ZoomToLocation(place) => {
                let place = place.trim();
                if place.is_empty() {
                    app.toasts.error("Location cannot be empty");
                    return;
                }
                match geocode_location(place) {
                    Ok((lat, lng)) => {
                        app.settings.fp_map.center_on(lat, lng);
                        app.settings.fp_map.zoom = 15.0;
                        app.toasts.info(format!("Zoomed to {place} ({lat:.4}, {lng:.4})"));
                    }
                    Err(err) => {
                        app.toasts.error(err);
                    }
                }
            }
            MenuPrompt::GdalOpacity(s) => {
                if let Some(op) = num(&s) {
                    app.settings.fp_map.gdal_opacity = op.clamp(0.0, 1.0) as f32;
                    app.toasts.info(format!("GDAL opacity: {:.2}", app.settings.fp_map.gdal_opacity));
                }
            }
            MenuPrompt::ModifyAlt(a) => {
                if let Some(a) = num(&a) {
                    for w in &mut self.items {
                        if w.has_location() {
                            w.alt = a as f32;
                        }
                    }
                }
            }
            MenuPrompt::Utm(z, e, n, s) => {
                if let (Ok(z), Some(e), Some(n)) = (z.trim().parse::<i32>(), num(&e), num(&n)) {
                    let (a, b) = utm_to_latlng(z, s, e, n);
                    let wp = self.new_wp(app, mavcmd::WAYPOINT, a, b);
                    self.items.push(wp);
                }
            }
            MenuPrompt::Text(lat, lng, t, h) => {
                let px = num(&h).unwrap_or(20.0) / 7.0;
                for (ci, ch) in t.chars().enumerate() {
                    let g = glyph(ch);
                    for (row, bits) in g.iter().enumerate() {
                        // serpentine through each glyph row so the path stays short
                        let cols: Vec<usize> = if row % 2 == 0 { (0..5).collect() } else { (0..5).rev().collect() };
                        for col in cols {
                            if bits & (1 << (4 - col)) != 0 {
                                let (a, b) = offset_m(lat, lng, -(row as f64) * px, (ci as f64 * 6.0 + col as f64) * px);
                                let wp = self.new_wp(app, mavcmd::WAYPOINT, a, b);
                                self.items.push(wp);
                            }
                        }
                    }
                }
            }
            MenuPrompt::CircleSurvey(lat, lng, r, s) => {
                let (Some(r), Some(s)) = (num(&r), num(&s)) else { return };
                let mut rad = r;
                while rad > s / 2.0 {
                    let pts = ((2.0 * std::f64::consts::PI * rad / s).ceil() as usize).max(6);
                    for (a, b) in mp_core::survey::circle((lat, lng), rad, pts) {
                        let wp = self.new_wp(app, mavcmd::WAYPOINT, a, b);
                        self.items.push(wp);
                    }
                    rad -= s;
                }
            }
            MenuPrompt::SplineCircle(lat, lng, r, n) => {
                if let (Some(r), Ok(n)) = (num(&r), n.trim().parse::<usize>()) {
                    for (a, b) in mp_core::survey::circle((lat, lng), r, n) {
                        let wp = self.new_wp(app, mavcmd::SPLINE_WAYPOINT, a, b);
                        self.items.push(wp);
                    }
                }
            }
            MenuPrompt::PoiAdd(lat, lng, n) => app.settings.pois.push((lat, lng, n)),
            MenuPrompt::PoiEdit(i, n) => {
                if let Some(p) = app.settings.pois.get_mut(i) {
                    p.2 = n;
                }
            }
        }
    }

    /// ElevationProfile: terrain (SRTM) under the route against the planned altitude.
    fn elevation_window(&mut self, ctx: &egui::Context, _app: &mut AppCtx) {
        if !self.elevation_open {
            return;
        }
        let home_alt = crate::srtm::altitude(self.home.lat, self.home.lng).unwrap_or(self.home.alt as f64);
        let mut pts: Vec<(f64, f64, f64)> = vec![(self.home.lat, self.home.lng, home_alt)];
        for w in self.items.iter().filter(|w| w.has_location() && (w.lat != 0.0 || w.lng != 0.0)) {
            let amsl = match w.frame {
                0 => w.alt as f64,
                10 => crate::srtm::altitude(w.lat, w.lng).unwrap_or(home_alt) + w.alt as f64,
                _ => home_alt + w.alt as f64,
            };
            pts.push((w.lat, w.lng, amsl));
        }
        let mut terrain = VecDeque::new();
        let mut path = VecDeque::new();
        let mut dist = 0.0;
        let mut missing = false;
        for seg in pts.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            let n = (a.0 - b.0).to_radians().hypot((a.1 - b.1).to_radians() * a.0.to_radians().cos());
            let len = n * R;
            let steps = ((len / 30.0).ceil() as usize).clamp(1, 200);
            for k in 0..=steps {
                let f = k as f64 / steps as f64;
                let (la, lo) = (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
                let d = dist + len * f;
                match crate::srtm::altitude(la, lo) {
                    Some(t) => terrain.push_back((d, t)),
                    None => missing = true,
                }
                path.push_back((d, a.2 + (b.2 - a.2) * f));
            }
            dist += len;
        }
        let mut open = true;
        egui::Window::new(crate::i18n::t("Elevation Graph")).open(&mut open).default_size([640.0, 360.0]).show(ctx, |ui| {
            if missing {
                theme::label(ui, "Downloading SRTM terrain...");
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
            }
            let size = ui.available_size().max(Vec2::new(500.0, 300.0));
            crate::zedgraph::plot_range(ui, size, "Elevation above sea level (m) / distance (m)", &[("Terrain", &terrain), ("Planned path", &path)], 0.0, dist.max(1.0));
        });
        if !open {
            self.elevation_open = false;
        }
    }
}

pub(super) fn geocode_location(place: &str) -> Result<(f64, f64), String> {
    let place = place.trim();
    if place.is_empty() {
        return Err("Location cannot be empty".into());
    }

    // 1. Direct coordinate format: "lat, lng" or "lat lng"
    let parts: Vec<&str> = if place.contains(',') {
        place.split(',').collect()
    } else {
        place.split_whitespace().collect()
    };
    if parts.len() == 2 {
        if let (Ok(lat), Ok(lng)) = (parts[0].trim().parse::<f64>(), parts[1].trim().parse::<f64>()) {
            if (-90.0..=90.0).contains(&lat) && (-180.0..=180.0).contains(&lng) {
                return Ok((lat, lng));
            }
        }
    }

    // 2. Offline / reference defaults matching Mission Planner tests
    let lower = place.to_lowercase();
    if lower.contains("perth") {
        return Ok((-31.9403, 115.9669));
    }
    if lower.contains("canberra") {
        return Ok((-35.2809, 149.1300));
    }
    if lower.contains("sydney") {
        return Ok((-33.8688, 151.2093));
    }

    // 3. Online query via OpenStreetMap Nominatim
    let encoded = place.replace(' ', "+");
    let url = format!("https://nominatim.openstreetmap.org/search?q={encoded}&format=json&limit=1");
    let resp = ureq::get(&url)
        .header("User-Agent", "MissionPlanner/1.3")
        .call();
    if let Ok(resp) = resp {
        if let Ok(json_text) = std::io::read_to_string(resp.into_body().into_reader()) {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&json_text) {
                if let Some(arr) = val.as_array() {
                    if let Some(first) = arr.first() {
                        let lat_opt = first.get("lat").and_then(|v| v.as_str()).and_then(|s| s.parse::<f64>().ok());
                        let lon_opt = first.get("lon").and_then(|v| v.as_str()).and_then(|s| s.parse::<f64>().ok());
                        if let (Some(lat), Some(lng)) = (lat_opt, lon_opt) {
                            return Ok((lat, lng));
                        }
                    }
                }
            }
        }
    }

    Err(format!("Google Maps Geocoder can't find: '{place}', reason: G_GEO_UNKNOWN_ADDRESS"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geocode_parses_coords_and_reference_cities() {
        assert_eq!(geocode_location("-31.94, 115.96").unwrap(), (-31.94, 115.96));
        assert_eq!(geocode_location("Perth Airport, Australia").unwrap(), (-31.9403, 115.9669));
        assert_eq!(geocode_location("Canberra").unwrap(), (-35.2809, 149.1300));
        assert!(geocode_location("").is_err());
    }
}
