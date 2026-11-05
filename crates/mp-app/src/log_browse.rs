//! Port of `Log/LogBrowse` (LogBrowse.resx): message tree on the right, graph and GPS map on
//! top, the button/checkbox bar, and the data table.

use std::path::PathBuf;

use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, StrokeKind, Vec2};
use mp_core::dataflash::{self, Log};

use crate::{fonts, map, theme};

const COLORS: [Color32; 8] = [
    Color32::RED,
    Color32::GREEN,
    Color32::from_rgb(0x00, 0xbf, 0xff),
    Color32::from_rgb(0xff, 0xa5, 0x00),
    Color32::YELLOW,
    Color32::from_rgb(0xff, 0x00, 0xff),
    Color32::WHITE,
    Color32::from_rgb(0x00, 0xff, 0xff),
];

/// CMB_preselect presets (a subset of Mission Planner's graph presets).
const PRESETS: &[(&str, &[(&str, &str, bool)])] = &[
    ("ATT Roll", &[("ATT", "Roll", false), ("ATT", "DesRoll", false)]),
    ("ATT Pitch", &[("ATT", "Pitch", false), ("ATT", "DesPitch", false)]),
    ("Vibration", &[("VIBE", "VibeX", false), ("VIBE", "VibeY", false), ("VIBE", "VibeZ", false)]),
    ("Altitude", &[("CTUN", "Alt", false), ("CTUN", "DAlt", false), ("BARO", "Alt", false)]),
    ("Battery", &[("BAT", "Volt", false), ("BAT", "Curr", true)]),
    ("GPS", &[("GPS", "NSats", false), ("GPS", "HDop", true)]),
];

pub struct LogBrowse {
    pub open: bool,
    name: String,
    log: Option<Log>,
    selected: Option<(String, String)>,
    graphs: Vec<(String, String, bool, Vec<(f64, f64)>)>,
    show_map: bool,
    show_time: bool,
    show_table: bool,
    show_params: bool,
    show_mode: bool,
    show_errors: bool,
    show_msg: bool,
    show_events: bool,
    preset: usize,
    map_state: map::MapState,
    tiles: map::TileCache,
    error: Option<String>,
}

impl Default for LogBrowse {
    fn default() -> Self {
        Self {
            open: false,
            name: String::new(),
            log: None,
            selected: None,
            graphs: Vec::new(),
            show_map: true,
            show_time: false,
            show_table: true,
            show_params: false,
            show_mode: true,
            show_errors: true,
            show_msg: true,
            show_events: false,
            preset: 0,
            map_state: map::MapState::default(),
            tiles: map::TileCache::new(Some(crate::platform::tile_cache_dir())),
            error: None,
        }
    }
}

impl LogBrowse {
    pub fn load(&mut self, path: PathBuf) {
        match std::fs::read(&path) {
            Ok(b) => {
                let log = dataflash::parse(&b);
                if let Some(&(la, lo, _)) = log.gps_track().first() {
                    self.map_state.center_on(la, lo);
                }
                self.name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                self.log = Some(log);
                self.graphs.clear();
                self.error = None;
                self.open = true;
            }
            Err(e) => self.error = Some(e.to_string()),
        }
    }

    fn add(&mut self, msg: &str, field: &str, right: bool) {
        if let Some(log) = &self.log {
            let s = log.series(msg, field);
            if !s.is_empty() && !self.graphs.iter().any(|g| g.0 == msg && g.1 == field) {
                self.graphs.push((msg.into(), field.into(), right, s));
            }
        }
    }

    pub fn ui(&mut self, ctx: &egui::Context) {
        if !self.open {
            return;
        }
        let mut open = true;
        let title = if self.name.is_empty() { "Log Browse".to_string() } else { format!("Log Browse - {}", self.name) };
        egui::Window::new(title).id(egui::Id::unique("logbrowse")).open(&mut open).default_size(Vec2::new(1200.0, 560.0)).resizable(true).show(ctx, |ui| {
            ui.set_min_size(Vec2::new(900.0, 480.0));
            // splitContainerAllTree: tree on the right (130px) with txt_info
            egui::Panel::right("lb_tree").exact_size(150.0).resizable(true).frame(egui::Frame::new().fill(theme::control_bg())).show(ui, |ui| self.tree(ui));
            egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
                let full = ui.available_rect_before_wrap();
                let top_h = (full.height() * 0.52).max(150.0);
                let top = Rect::from_min_size(full.min, Vec2::new(full.width(), top_h));
                let graph_r = if self.show_map { Rect::from_min_size(top.min, Vec2::new(top.width() / 2.0, top.height())) } else { top };
                self.graph(ui, graph_r);
                if self.show_map {
                    let map_r = Rect::from_min_max(Pos2::new(graph_r.right() + 4.0, top.top()), top.max);
                    self.map(ui, map_r);
                }
                let bar = Rect::from_min_size(Pos2::new(full.left(), top.bottom() + 4.0), Vec2::new(full.width(), 25.0));
                self.buttons(ui, bar);
                let grid = Rect::from_min_max(Pos2::new(full.left(), bar.bottom() + 2.0), full.max);
                if self.show_table {
                    self.table(ui, grid);
                }
                ui.allocate_rect(full, Sense::hover());
            });
        });
        if !open {
            self.open = false;
        }
    }

    fn tree(&mut self, ui: &mut egui::Ui) {
        let Some(log) = &self.log else {
            theme::label(ui, "Load a log");
            return;
        };
        let tree = log.tree();
        let mut add: Option<(String, String)> = None;
        egui::ScrollArea::vertical().max_height(ui.available_height() - 44.0).show(ui, |ui| {
            for (msg, cols) in &tree {
                egui::CollapsingHeader::new(egui::RichText::new(msg).color(Color32::WHITE)).id_salt(msg).show(ui, |ui| {
                    for c in cols.iter().filter(|c| *c != "TimeUS" && *c != "TimeMS") {
                        let sel = self.selected.as_ref().map(|(m, f)| m == msg && f == c).unwrap_or(false);
                        let r = ui.selectable_label(sel, egui::RichText::new(c).color(Color32::WHITE));
                        if r.clicked() {
                            self.selected = Some((msg.clone(), c.clone()));
                        }
                        if r.double_clicked() {
                            add = Some((msg.clone(), c.clone()));
                        }
                    }
                });
            }
        });
        if let Some((m, f)) = add {
            self.add(&m, &f, false);
        }
        // txt_info
        let info = self.log.as_ref().map(|l| format!("{} records\n{} types", l.records.len(), l.formats.len())).unwrap_or_default();
        theme::label(ui, info);
    }

    fn buttons(&mut self, ui: &mut egui::Ui, bar: Rect) {
        let o = bar.min;
        let b = |ui: &mut egui::Ui, x: f32, w: f32, t: &str| {
            let r = Rect::from_min_size(o + Vec2::new(x, 1.0), Vec2::new(w, 23.0));
            ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, t, r.size())).clicked()
        };
        if b(ui, 3.0, 71.0, "Graph Left") {
            if let Some((m, f)) = self.selected.clone() {
                self.add(&m, &f, false);
            }
        }
        if b(ui, 80.0, 72.0, "Graph Right") {
            if let Some((m, f)) = self.selected.clone() {
                self.add(&m, &f, true);
            }
        }
        if b(ui, 158.0, 75.0, "Clear Graph") {
            self.graphs.clear();
        }
        if b(ui, 239.0, 75.0, "Load A Log") {
            if let Some(p) = pick_bin() {
                self.load(p);
            }
        }
        if b(ui, 320.0, 75.0, "Remove Item") {
            if let Some((m, f)) = &self.selected {
                self.graphs.retain(|g| !(g.0 == *m && g.1 == *f));
            }
        }
        let c = |ui: &mut egui::Ui, x: f32, w: f32, v: &mut bool, t: &str| {
            ui.put(Rect::from_min_size(o + Vec2::new(x, 4.0), Vec2::new(w, 17.0)), egui::Checkbox::new(v, t));
        };
        c(ui, 401.0, 50.0, &mut self.show_map, "Map");
        c(ui, 454.0, 52.0, &mut self.show_time, "Time");
        c(ui, 509.0, 82.0, &mut self.show_table, "Data Table");
        c(ui, 594.0, 95.0, &mut self.show_params, "Show Params");
        ui.put(Rect::from_min_size(o + Vec2::new(691.0, 2.0), Vec2::new(100.0, 21.0)), |ui: &mut egui::Ui| {
            let mut sel = self.preset;
            let r = egui::ComboBox::from_id_salt("CMB_preselect")
                .width(92.0)
                .wrap_mode(egui::TextWrapMode::Truncate)
                .selected_text(PRESETS[self.preset].0)
                .show_ui(ui, |ui| {
                    for (i, (n, _)) in PRESETS.iter().enumerate() {
                        ui.selectable_value(&mut sel, i, *n);
                    }
                })
                .response;
            if sel != self.preset {
                self.preset = sel;
                self.graphs.clear();
                for (m, f, right) in PRESETS[sel].1 {
                    self.add(m, f, *right);
                }
            }
            r
        });
        c(ui, 797.0, 56.0, &mut self.show_mode, "Mode");
        c(ui, 856.0, 56.0, &mut self.show_errors, "Errors");
        c(ui, 915.0, 53.0, &mut self.show_msg, "MSG");
        c(ui, 971.0, 62.0, &mut self.show_events, "Events");
    }

    fn graph(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let p = ui.painter().with_clip_rect(rect);
        p.rect_filled(rect, 0.0, Color32::from_rgb(0x37, 0x37, 0x38));
        let chart = Rect::from_min_max(rect.min + Vec2::new(50.0, 20.0), rect.max - Vec2::new(50.0, 34.0));
        p.rect_filled(chart, 0.0, Color32::from_rgb(0x1f, 0x1f, 0x20));
        p.rect_stroke(chart, 0.0, Stroke::new(1.0, Color32::GRAY), StrokeKind::Inside);
        let f = fonts::ui_id(fonts::UI_SIZE);
        p.text(Pos2::new(rect.center().x, rect.top() + 3.0), egui::Align2::CENTER_TOP, &self.name, f.clone(), Color32::WHITE);
        if self.graphs.is_empty() {
            return;
        }
        let (t0, t1) = self.graphs.iter().flat_map(|g| g.3.iter().map(|p| p.0)).fold((f64::MAX, f64::MIN), |(a, b), t| (a.min(t), b.max(t)));
        let range = |right: bool| {
            let (lo, hi) = self.graphs.iter().filter(|g| g.2 == right).flat_map(|g| g.3.iter().map(|p| p.1)).fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
            if lo > hi {
                (0.0, 1.0)
            } else if (hi - lo).abs() < 1e-9 {
                (lo - 1.0, hi + 1.0)
            } else {
                (lo, hi)
            }
        };
        let (l0, l1) = range(false);
        let (r0, r1) = range(true);
        let span = (t1 - t0).max(1e-6);
        let to = |t: f64, v: f64, lo: f64, hi: f64| Pos2::new(chart.left() + ((t - t0) / span) as f32 * chart.width(), chart.bottom() - ((v - lo) / (hi - lo)) as f32 * chart.height());
        for k in 0..=4 {
            let fr = k as f64 / 4.0;
            let y = chart.bottom() - fr as f32 * chart.height();
            p.line_segment([Pos2::new(chart.left(), y), Pos2::new(chart.right(), y)], Stroke::new(1.0, Color32::from_gray(55)));
            p.text(Pos2::new(chart.left() - 4.0, y), egui::Align2::RIGHT_CENTER, format!("{:.1}", l0 + (l1 - l0) * fr), f.clone(), Color32::WHITE);
            if self.graphs.iter().any(|g| g.2) {
                p.text(Pos2::new(chart.right() + 4.0, y), egui::Align2::LEFT_CENTER, format!("{:.1}", r0 + (r1 - r0) * fr), f.clone(), Color32::WHITE);
            }
            let t = t0 + span * fr;
            let x = chart.left() + fr as f32 * chart.width();
            let lab = if self.show_time { format!("{:02}:{:02}", (t as i64) / 60, (t as i64) % 60) } else { format!("{t:.0}") };
            p.text(Pos2::new(x, chart.bottom() + 2.0), egui::Align2::CENTER_TOP, lab, f.clone(), Color32::WHITE);
        }
        for (i, (_, _, right, s)) in self.graphs.iter().enumerate() {
            let (lo, hi) = if *right { (r0, r1) } else { (l0, l1) };
            let step = (s.len() / (chart.width() as usize * 2).max(1)).max(1);
            let pts: Vec<Pos2> = s.iter().step_by(step).map(|&(t, v)| to(t, v, lo, hi)).collect();
            if pts.len() > 1 {
                p.add(Shape::line(pts, Stroke::new(1.5, COLORS[i % COLORS.len()])));
            }
        }
        // MODE / MSG markers
        if let Some(log) = &self.log {
            let markers = |name: &str, col: Color32, field: &str| -> Vec<(f64, String, Color32)> {
                let Some(t) = log.type_by_name(name) else { return Vec::new() };
                let fmt = &log.formats[&t];
                let tc = fmt.columns.iter().position(|c| c == "TimeUS");
                let fc = fmt.columns.iter().position(|c| c == field);
                log.records
                    .iter()
                    .filter(|r| r.type_id == t)
                    .filter_map(|r| Some((r.values.get(tc?)?.as_f64()? * 1e-6, r.values.get(fc?)?.to_string(), col)))
                    .collect()
            };
            let mut all = Vec::new();
            if self.show_mode {
                all.extend(markers("MODE", Color32::from_rgb(0, 200, 0), "Mode"));
            }
            if self.show_msg {
                all.extend(markers("MSG", Color32::WHITE, "Message"));
            }
            if self.show_errors {
                all.extend(markers("ERR", Color32::RED, "ECode"));
            }
            if self.show_events {
                all.extend(markers("EV", Color32::YELLOW, "Id"));
            }
            for (t, text, col) in all {
                if t < t0 || t > t1 {
                    continue;
                }
                let x = to(t, l0, l0, l1).x;
                p.line_segment([Pos2::new(x, chart.top()), Pos2::new(x, chart.bottom())], Stroke::new(1.0, col.gamma_multiply(0.6)));
                p.text(Pos2::new(x + 2.0, chart.top() + 2.0), egui::Align2::LEFT_TOP, text, fonts::ui_id(9.0), col);
            }
        }
        // legend
        let mut x = chart.left();
        for (i, (m, fld, right, _)) in self.graphs.iter().enumerate() {
            let col = COLORS[i % COLORS.len()];
            let t = format!("{m}.{fld}{}", if *right { " (R)" } else { "" });
            p.line_segment([Pos2::new(x, rect.bottom() - 12.0), Pos2::new(x + 14.0, rect.bottom() - 12.0)], Stroke::new(2.0, col));
            let g = p.layout_no_wrap(t, f.clone(), Color32::WHITE);
            let w = g.size().x;
            p.galley(Pos2::new(x + 18.0, rect.bottom() - 19.0), g, Color32::WHITE);
            x += w + 30.0;
        }
    }

    fn map(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let Some(log) = &self.log else { return };
        let track = log.gps_track();
        ui.scope_builder(egui::UiBuilder::new().max_rect(rect), |ui| {
            map::show(ui, &mut self.map_state, &mut self.tiles, |_ui, view, _resp, painter| {
                if track.len() > 1 {
                    let pts: Vec<Pos2> = track.iter().map(|&(a, b, _)| view.to_screen(a, b)).collect();
                    painter.add(Shape::line(pts, Stroke::new(2.0, Color32::from_rgb(0xff, 0x00, 0xff))));
                }
                false
            });
        });
        let p = ui.painter();
        for (x, t) in [(4.0, "GPS"), (39.0, "GPS2"), (80.0, "POS"), (115.0, "GPSB")] {
            p.text(rect.min + Vec2::new(x, 8.0), egui::Align2::LEFT_TOP, t, fonts::ui_id(fonts::UI_SIZE), Color32::BLACK);
        }
    }

    fn table(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let Some(log) = &self.log else { return };
        ui.scope_builder(egui::UiBuilder::new().max_rect(rect), |ui| {
            let n = log.records.len();
            egui::ScrollArea::both().auto_shrink([false, false]).show_rows(ui, 18.0, n, |ui, range| {
                for i in range {
                    let r = &log.records[i];
                    let name = log.name(r.type_id);
                    if !self.show_params && name == "PARM" {
                        continue;
                    }
                    let bg = if i % 2 == 0 { theme::control_bg() } else { theme::bg() };
                    let (row, _) = ui.allocate_exact_size(Vec2::new(ui.available_width().max(1000.0), 18.0), Sense::hover());
                    ui.painter().rect_filled(row, 0.0, bg);
                    let mut s = format!("{i:>7}  {name:<5}");
                    for v in &r.values {
                        s.push_str("  ");
                        s.push_str(&v.to_string());
                    }
                    ui.painter().text(row.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, s, egui::FontId::monospace(11.0), Color32::WHITE);
                }
            });
        });
    }
}

#[cfg(not(any(target_os = "android", target_os = "ios")))]
pub fn pick_bin() -> Option<PathBuf> {
    rfd::FileDialog::new().add_filter("Binary Log", &["bin", "BIN"]).set_directory(crate::platform::data_dir().join("logs")).pick_file()
}

#[cfg(any(target_os = "android", target_os = "ios"))]
pub fn pick_bin() -> Option<PathBuf> {
    crate::platform::list_files_in(&crate::platform::data_dir().join("logs"), &["bin"]).last().cloned()
}
