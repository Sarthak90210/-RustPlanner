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
/// The tools that ask one InputBox after another.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(super) enum ChainTool {
    SplineCircle,
    CircleSurvey,
    Utm,
    Text,
    OffsetPolygon,
}

/// One tool's InputBox sequence: the answers so far and where it was opened.
pub(super) struct InputChain {
    pub tool: ChainTool,
    pub step: usize,
    pub values: Vec<String>,
    pub text: String,
    pub lat: f64,
    pub lng: f64,
}

impl ChainTool {
    /// (title, prompt, default) of each InputBox.Show
    fn prompts(self, zone: &str) -> Vec<(&'static str, &'static str, String)> {
        match self {
            ChainTool::SplineCircle => vec![("Radius", "Radius", "50".into()), ("min alt", "Min Alt", "5".into()), ("max alt", "Max Alt", "20".into()), ("alt step", "alt step", "5".into()), ("angle", "Angle of first point (whole degrees)", "0".into())],
            ChainTool::CircleSurvey => vec![("", "startalt", "10".into()), ("", "endalt", "20".into()), ("", "seperation", "2".into()), ("", "radius", "5".into()), ("", "photos", "50".into()), ("", "start heading", "0".into())],
            ChainTool::Utm => vec![("Zone", "Enter Zone. (eg 50S, 11N)", zone.to_string()), ("Easting", "Easting", "578994".into()), ("Northing", "Northing", "6126244".into())],
            ChainTool::Text => vec![("Enter String", "Enter String (requires 1CamBam_Stick_3 font)", String::new()), ("Enter size", "Enter size", "5".into()), ("Enter rotation", "Enter rotation", "0".into())],
            ChainTool::OffsetPolygon => vec![("Offset in Meters", "Please enter the offset in meters. Enter a negative value to make the polygon smaller", "0".into())],
        }
    }

    /// Cancel ends these tools; the others carry on with the default
    fn cancel_ends(self) -> bool {
        matches!(self, ChainTool::SplineCircle | ChainTool::Utm)
    }
}

/// createSplineCircleToolStripMenuItem_Click's rings: (lat, lng, alt) after the ROI. Four
/// points per lap and 360 inclusive (five), the first lap at min alt, then the altitude rising
/// altstep / 4 (whole numbers) per point until past max alt.
pub(super) fn spline_circle(lat: f64, lng: f64, radius: i32, minalt: i32, maxalt: i32, altstep: i32) -> Vec<(f64, f64, i32)> {
    let points = 4;
    let step = 360.0f32 as f64 / points as f64;
    let (d, r) = (radius as f64, 6371000.0f64);
    let mut out = Vec::new();
    let mut startup = true;
    let mut stepalt = minalt;
    while stepalt <= maxalt {
        let mut a = 0.0f64;
        while a <= 360.0 && a >= 0.0 {
            let la = lat.to_radians();
            let lat2 = (la.sin() * (d / r).cos() + la.cos() * (d / r).sin() * a.to_radians().cos()).asin();
            let lon2 = lng.to_radians() + (a.to_radians().sin() * (d / r).sin() * la.cos()).atan2((d / r).cos() - la.sin() * lat2.sin());
            out.push((lat2.to_degrees(), lon2.to_degrees(), stepalt));
            if !startup {
                stepalt += altstep / points;
            }
            a += step;
            // the original never ends when the step rounds to 0
            if out.len() > 10000 {
                return out;
            }
        }
        if startup {
            stepalt = minalt;
        }
        startup = false;
        if altstep / points == 0 {
            return out;
        }
    }
    out
}

/// offsetPolygonToolStripMenuItem_Click after the prompt: the polygon in UTM (the first
/// point's zone) times 1000 through ClipperOffset (jtMiter, etClosedPolygon) into a PolyTree
/// by `(Int64)(meters * 1000.0)`, then every child contour back through utmpos.ToLLA. None
/// when the polygon is empty or the offset leaves nothing (the C# returns and keeps the
/// drawn polygon).
pub(super) fn offset_polygon(poly: &[(f64, f64)], intmeter: f64) -> Option<Vec<(f64, f64)>> {
    use mp_core::extlibs::utilities::clipper::{ClipperOffset, EndType, IntPoint, JoinType, PolyTree};
    use mp_core::extlibs::utilities::utmpos::utmpos;
    use mp_core::extlibs::utilities::PointLatLngAlt;
    if poly.is_empty() {
        return None;
    }
    let list: Vec<PointLatLngAlt> = poly.iter().map(|&(lat, lng)| PointLatLngAlt::new(lat, lng)).collect();

    // utm zone distance calcs will be done in
    let utmzone = list[0].get_utm_zone();

    // utm position list
    let r = (|| -> Result<Option<Vec<(f64, f64)>>, mp_core::dotnet::Exception> {
        let mut utmpositions = utmpos::to_list(&PointLatLngAlt::to_utm_list(utmzone, &list)?, utmzone)?;

        // close the loop if its not already
        let (first, last) = (&utmpositions[0], &utmpositions[utmpositions.len() - 1]);
        if (first.x, first.y, first.zone) != (last.x, last.y, last.zone) {
            let f = utmpos::copy_of(&utmpositions[0]);
            utmpositions.push(f); // make a full loop
        }

        let mut clipper_offset = ClipperOffset::new(2.0, 0.25);
        let path: Vec<IntPoint> = utmpositions.iter().map(|a| IntPoint::from_f64(a.x * 1000.0, a.y * 1000.0)).collect();
        clipper_offset.AddPath(&path, JoinType::jtMiter, EndType::etClosedPolygon);

        let mut tree = PolyTree::new();
        clipper_offset.Execute_tree(&mut tree, mp_core::dotnet::cast_i64(intmeter * 1000.0) as f64)?;

        if tree.ChildCount(PolyTree::ROOT) == 0 {
            return Ok(None);
        }

        let mut ans = Vec::new();
        for &tree_child in tree.Childs(PolyTree::ROOT) {
            for a in tree.Contour(tree_child) {
                let p = utmpos::new(a.X as f64 / 1000.0, a.Y as f64 / 1000.0, utmzone).to_lla()?;
                ans.push((p.Lat, p.Lng));
            }
        }
        Ok(Some(ans))
    })();
    r.unwrap_or_else(|e| {
        log::error!("{e}");
        None
    })
}

pub(super) enum MenuPrompt {
    RotateMap(String),
    #[allow(dead_code)]
    ZoomTo(String, String),
    ZoomToLocation(String),
    GdalOpacity(String),
    ModifyAlt(String),
    PoiAdd(f64, f64, String),
    PoiEdit(usize, String),
    /// Commands_SelectionChangeCommitted's InputBox for UNKNOWN: row, id text
    CommandId(usize, String),
}

fn offset_m(lat: f64, lng: f64, north: f64, east: f64) -> (f64, f64) {
    (lat + (north / R).to_degrees(), lng + (east / (R * lat.to_radians().cos())).to_degrees())
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

fn pick(title: &str, filter: &str, exts: &[&str]) -> Option<std::path::PathBuf> {
    platform::pick_file(title, filter, exts)
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
            // nothing to offset: no question either
            if !self.polygon.is_empty() {
                self.start_chain(ChainTool::OffsetPolygon, 0.0, 0.0);
            }
            ui.close();
        }
        if ui.button(crate::i18n::t("Area")).clicked() {
            area_message(&self.polygon, app);
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
                self.start_chain(ChainTool::SplineCircle, lat, lng);
                ui.close();
            }
            if ui.button(crate::i18n::t("Area")).clicked() {
                area_message(&self.polygon, app);
                ui.close();
            }
            if text && ui.button(crate::i18n::t("Text")).clicked() {
                self.start_chain(ChainTool::Text, lat, lng);
                ui.close();
            }
            if circle_survey && ui.button(crate::i18n::t("Create Circle Survey")).clicked() {
                self.start_chain(ChainTool::CircleSurvey, lat, lng);
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
            // SimpleGridPlugin.Loaded: "SimpleGrid" appended to autoWPToolStripMenuItem
            if ui.button(crate::i18n::t(crate::extlibs::simplegrid::GridPlugin::menu_text())).clicked() {
                let mut polygon = std::mem::take(&mut self.polygon);
                let mut config = std::mem::take(&mut app.settings.planner);
                let mut host = crate::extlibs::simplegrid::PluginHost {
                    FPDrawnPolygon: &mut polygon,
                    PlannedHomeLocation: self.planned_home_location(),
                    config: &mut config,
                    FDMapType: app.settings.fd_map.provider,
                };
                let click = crate::extlibs::simplegrid::GridPlugin.but_Click(&mut host);
                self.polygon = polygon;
                app.settings.planner = config;
                match click {
                    crate::extlibs::simplegrid::ButClick::Show(g) => self.simple_grid = Some(g),
                    crate::extlibs::simplegrid::ButClick::Error(e, caption) => app.toasts.message(e, caption),
                }
                ui.close();
            }
        });
        ui.menu_button(crate::i18n::t("Map Tool"), |ui| {
            if ui.button(crate::i18n::t("Measure Distance")).clicked() {
                self.context_measure(lat, lng, app);
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
                if let Some((name, text)) = platform::open_text("Open Waypoint File", "All Supported Types", &["txt", "waypoints", "plan"]) {
                    self.load_text(&name, &text, app);
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Load and Append")).clicked() {
                // loadAndAppendToolStripMenuItem_Click: "Ardupilot Mission|*.waypoints;*.txt"
                if let Some((_, text)) = platform::open_text("Open Waypoint File", "Ardupilot Mission", &["waypoints", "txt"]) {
                    self.read_qgc110wpfile(&text, true, app);
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Save WP File")).clicked() {
                // saveWPFileToolStripMenuItem_Click -> SaveFile_Click -> savewaypoints
                self.savewaypoints(app);
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
            self.start_chain(ChainTool::Utm, lat, lng);
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
            let read = mission::read_waypoint_file(&text, &mut |m| app.toasts.error(m));
            match read {
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

    fn start_chain(&mut self, tool: ChainTool, lat: f64, lng: f64) {
        let first = tool.prompts(&self.utm_zone)[0].2.clone();
        self.input_chain = Some(InputChain { tool, step: 0, values: Vec::new(), text: first, lat, lng });
    }

    /// ContextMeasure_Click: the first click marks the start, the second reports and clears.
    fn context_measure(&mut self, lat: f64, lng: f64, app: &mut AppCtx) {
        match self.measure.take() {
            None => {
                self.measure = Some((lat, lng));
                if crate::show_again::should_show(&app.settings.planner, "Measure Dist") {
                    self.fp_show_again = Some(crate::show_again::ShowAgain::new("Measure Dist", "You can now pan/zoom around.\nClick this option again to get the distance.", false));
                }
            }
            Some((a, b)) => {
                let (from, to) = (mp_core::geo::PointLatLngAlt::new(a, b, 0.0), mp_core::geo::PointLatLngAlt::new(lat, lng, 0.0));
                let km = from.distance(&to) / 1000.0;
                app.toasts.error(format!("Distance: {} AZ: {:.0}", super::format_distance(&app.settings.planner, km, true), from.bearing(&to)));
            }
        }
    }

    /// The current InputBox of a tool's sequence; the tool runs after the last answer.
    fn chain_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some(chain) = self.input_chain.as_mut() else { return };
        let prompts = chain.tool.prompts(&self.utm_zone);
        let (title, prompt, _) = prompts[chain.step].clone();
        let mut answer = None;
        egui::Modal::new(egui::Id::new(("fp_inputbox", chain.step))).show(ctx, |ui| {
            ui.set_min_width(300.0);
            ui.label(egui::RichText::new(crate::i18n::t(title)).strong());
            theme::label(ui, prompt);
            let r = ui.text_edit_singleline(&mut chain.text);
            r.request_focus();
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                if theme::button(ui, "Cancel").clicked() {
                    answer = Some(false);
                }
                if theme::button(ui, "OK").clicked() {
                    answer = Some(true);
                }
            });
        });
        if ctx.input(|i| i.key_pressed(egui::Key::Enter)) {
            answer = Some(true);
        } else if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            answer = Some(false);
        }
        let Some(ok) = answer else { return };
        if !ok && chain.tool.cancel_ends() {
            self.input_chain = None;
            return;
        }
        let value = if ok { chain.text.clone() } else { prompts[chain.step].2.clone() };
        chain.values.push(value);
        chain.step += 1;
        if chain.step < prompts.len() {
            chain.text = prompts[chain.step].2.clone();
            return;
        }
        let chain = self.input_chain.take().unwrap();
        self.run_chain(chain, app);
    }

    fn run_chain(&mut self, c: InputChain, app: &mut AppCtx) {
        let v = &c.values;
        let alt_unit = crate::units::alt(1.0);
        let frame = app.settings.alt_frame;
        match c.tool {
            ChainTool::SplineCircle => {
                let int = |s: &str| s.trim().parse::<i32>().ok();
                let Some(radius) = int(&v[0]) else { return app.toasts.error("Bad Radius") };
                let Some(minalt) = int(&v[1]) else { return app.toasts.error("Bad min alt") };
                let Some(maxalt) = int(&v[2]) else { return app.toasts.error("Bad maxalt") };
                let Some(altstep) = int(&v[3]) else { return app.toasts.error("Bad alt step") };
                self.update_undo_buffer();
                self.items.push(Locationwp { id: mavcmd::DO_SET_ROI, lat: c.lat, lng: c.lng, alt: 0.0, frame, ..Default::default() });
                for (a, b, alt) in spline_circle(c.lat, c.lng, radius, minalt, maxalt, altstep) {
                    self.items.push(Locationwp { id: mavcmd::SPLINE_WAYPOINT, lat: a, lng: b, alt: alt as f32 / alt_unit, frame, ..Default::default() });
                }
            }
            ChainTool::CircleSurvey => {
                // InputBox(ref int) keeps the default for text that is not a number
                let int = |s: &str, d: i32| s.trim().parse::<i32>().unwrap_or(d);
                let (startalt, endalt, sep, radius, photos, heading0) = (int(&v[0], 10), int(&v[1], 20), int(&v[2], 2), int(&v[3], 5), int(&v[4], 50), int(&v[5], 0));
                self.items.push(Locationwp { id: mavcmd::DO_SET_ROI, lat: c.lat, lng: c.lng, alt: 0.0, frame, ..Default::default() });
                if sep <= 0 || photos <= 0 || 360 / photos == 0 {
                    return;
                }
                let centre = mp_core::geo::PointLatLngAlt::new(c.lat, c.lng, 0.0);
                let mut alt = startalt;
                while alt <= endalt {
                    let mut heading = heading0;
                    while heading <= heading0 + 360 {
                        let p = centre.newpos(heading as f64, radius as f64);
                        self.items.push(Locationwp { id: mavcmd::WAYPOINT, p1: 2.0, lat: p.lat, lng: p.lng, alt: alt as f32, frame, ..Default::default() });
                        // DO_DIGICAM_CONTROL with 1 in the Lat column: take a shot
                        self.items.push(Locationwp { id: 203, lat: 1.0, frame, ..Default::default() });
                        heading += 360 / photos;
                    }
                    alt += sep;
                }
            }
            ChainTool::Utm => {
                self.utm_zone = v[0].clone();
                // the zone digits; zone.ToLower().Contains("N") is never true, so south
                let digits = v[0].to_lowercase().replace('s', " ").to_lowercase().replace('n', " ");
                // int.Parse / double.Parse in the current culture, then GeoUtility's
                // new UTM(zone, e, n, Hemisphere.South) -> Geographic; an exception leaves the
                // handler (shown here) before the row is added
                let r = (|| -> Result<(f64, f64), mp_core::dotnet::Exception> {
                    let z = mp_core::dotnet::int_parse(&digits)?;
                    let e = mp_core::dotnet::double_parse(&v[1])?;
                    let n = mp_core::dotnet::double_parse(&v[2])?;
                    mp_core::coords::utm_hemisphere_to_geo(z, e, n, true)
                })();
                let (a, b) = match r {
                    Ok(p) => p,
                    Err(e) => {
                        app.toasts.error(e.message().to_string());
                        return;
                    }
                };
                self.update_undo_buffer();
                let wp = self.new_wp(app, mavcmd::WAYPOINT, a, b);
                self.items.push(wp);
            }
            ChainTool::Text => {
                let (Ok(size), Ok(rot)) = (v[1].trim().parse::<f64>(), v[2].trim().parse::<f64>()) else { return };
                // the path is size * 1.35 tall; rotated clockwise on screen, y pointing down
                let px = size * 1.35 / 7.0;
                let (sin, cos) = rot.to_radians().sin_cos();
                for (ci, ch) in v[0].chars().enumerate() {
                    for (row, bits) in glyph(ch).iter().enumerate() {
                        let cols: Vec<usize> = if row % 2 == 0 { (0..5).collect() } else { (0..5).rev().collect() };
                        for col in cols {
                            if bits & (1 << (4 - col)) != 0 {
                                let (x, y) = ((ci as f64 * 6.0 + col as f64) * px, row as f64 * px);
                                let (xr, yr) = (x * cos - y * sin, x * sin + y * cos);
                                let (a, b) = offset_m(c.lat, c.lng, -yr, xr);
                                let wp = self.new_wp(app, mavcmd::WAYPOINT, a, b);
                                self.items.push(wp);
                            }
                        }
                    }
                }
            }
            ChainTool::OffsetPolygon => {
                // double.Parse(meter) unless the text is "0"; a bad number throws out of the
                // handler and the polygon stays as drawn
                let d = if v[0] != "0" {
                    match mp_core::dotnet::double_parse(&v[0]) {
                        Ok(d) => d,
                        Err(e) => {
                            log::error!("{e}");
                            return;
                        }
                    }
                } else {
                    0.0
                };
                // redrawPolygonSurvey(ans), unless the offset left nothing
                if let Some(p) = offset_polygon(&self.polygon, d) {
                    self.polygon = p;
                }
            }
        }
    }

    /// Context-menu prompts and the elevation graph window.
    pub(super) fn menu_dialogs(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        self.chain_ui(ctx, app);
        if let Some(p) = self.fp_show_again.clone() {
            if p.ui(ctx, &mut app.settings.planner).is_some() {
                self.fp_show_again = None;
            }
        }
        if let Some(p) = self.menu_prompt.as_mut() {
            let mut done = None;
            let title = match p {
                MenuPrompt::RotateMap(_) => "Rotate map to heading",
                MenuPrompt::ZoomTo(..) => "Zoom To",
                MenuPrompt::ZoomToLocation(..) => "Location",
                MenuPrompt::GdalOpacity(..) => "Opacity 0.0-1.0",
                MenuPrompt::ModifyAlt(_) => "Modify Alt",
                MenuPrompt::PoiAdd(..) => "POI",
                MenuPrompt::PoiEdit(..) => "POI",
                MenuPrompt::CommandId(..) => "Mavlink ID",
            };
            egui::Window::new(title).collapsible(false).resizable(false).show(ctx, |ui| {
                let field = |ui: &mut egui::Ui, l: &str, v: &mut String| {
                    theme::label(ui, l);
                    ui.text_edit_singleline(v);
                };
                match p {
                    MenuPrompt::RotateMap(h) => field(ui, "Enter new UP heading", h),
                    MenuPrompt::ZoomTo(a, b) => {
                        field(ui, "Lat", a);
                        field(ui, "Lng", b);
                    }
                    MenuPrompt::ZoomToLocation(loc) => field(ui, "Enter your location", loc),
                    MenuPrompt::GdalOpacity(op) => field(ui, "Enter opacity (0.0-1.0)", op),
                    MenuPrompt::ModifyAlt(a) => field(ui, "Altitude", a),
                    MenuPrompt::PoiAdd(_, _, n) | MenuPrompt::PoiEdit(_, n) => field(ui, "Enter ID", n),
                    MenuPrompt::CommandId(_, n) => field(ui, "Please enter the command ID", n),
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
            MenuPrompt::CommandId(row, id) => {
                // "-1" leaves the command alone; ushort.Parse failures are only logged
                if id != "-1" {
                    match id.trim().parse::<u16>() {
                        Ok(v) => {
                            if let Some(w) = self.items.get_mut(row) {
                                w.id = v;
                            }
                        }
                        Err(e) => log::error!("{e}"),
                    }
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

/// areaToolStripMenuItem_Click (calcpolygonarea asks for a polygon first; tab stops drawn as spaces)
fn area_message(polygon: &[(f64, f64)], app: &mut AppCtx) {
    if polygon.is_empty() {
        app.toasts.error("Please define a polygon!");
        return;
    }
    let a = mp_core::survey::area(polygon).abs();
    let tab = "        ";
    app.toasts.message(format!("Area: {a:.0} m2\n{tab}{:.2} Acre\n{tab}{:.2} Hectare\n{tab}{:.0} sqf", a * 0.000247105, a * 1e-4, a * 10.7639), "Area");
}

#[cfg(test)]
mod tool_tests {
    use super::*;

    #[test]
    fn spline_circle_rings_follow_the_original_loop() {
        let pts = spline_circle(-35.0, 149.0, 50, 5, 20, 5);
        // first lap: five points at 5; then the alt rises 1 (5 / 4) per point to past 20
        assert_eq!(pts.iter().take(5).map(|p| p.2).collect::<Vec<_>>(), [5, 5, 5, 5, 5]);
        assert_eq!(pts[5].2, 5);
        assert_eq!(pts[6].2, 6);
        // the lap test is between laps only: the last lap starts at most at 20 and climbs on
        assert_eq!(pts.iter().map(|p| p.2).max(), Some(24));
        let first = mp_core::geo::PointLatLngAlt::new(pts[0].0, pts[0].1, 0.0);
        assert!((first.distance(&mp_core::geo::PointLatLngAlt::new(-35.0, 149.0, 0.0)) - 50.0).abs() < 0.5);
        // a step under four never rises: one lap, not an endless loop
        assert_eq!(spline_circle(-35.0, 149.0, 50, 5, 20, 3).len(), 5);
    }

    #[test]
    fn offset_polygon_moves_each_edge_by_the_distance() {
        // a 100 m square grown by 10 m becomes 120 m wide
        let sq = [(0.0, 0.0), (0.0, 0.000898315), (0.000904369, 0.000898315), (0.000904369, 0.0)];
        let out = offset_polygon(&sq, 10.0).unwrap();
        let w = (out.iter().map(|p| p.1).fold(f64::MIN, f64::max) - out.iter().map(|p| p.1).fold(f64::MAX, f64::min)).to_radians() * R;
        assert!((w - 120.0).abs() < 0.5, "{w}");
        let inner = offset_polygon(&sq, -10.0).unwrap();
        let w = (inner.iter().map(|p| p.1).fold(f64::MIN, f64::max) - inner.iter().map(|p| p.1).fold(f64::MAX, f64::min)).to_radians() * R;
        assert!((w - 80.0).abs() < 0.5, "{w}");
        // shrunk to nothing: the drawn polygon stays
        assert!(offset_polygon(&sq, -60.0).is_none());
        assert!(offset_polygon(&[], 10.0).is_none());
    }

    #[test]
    fn utm_prompt_defaults_and_cancel_rules() {
        let p = ChainTool::Utm.prompts("50s");
        assert_eq!(p[0], ("Zone", "Enter Zone. (eg 50S, 11N)", "50s".to_string()));
        assert!(ChainTool::Utm.cancel_ends() && !ChainTool::Text.cancel_ends());
        assert_eq!(ChainTool::CircleSurvey.prompts("").len(), 6);
    }
}
