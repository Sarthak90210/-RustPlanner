//! Port of `GCSViews/SoftwareConfig` (CONFIG): BackstageView menu with ConfigRawParams
//! (Full Parameter List / Tree), ConfigFriendlyParams (Standard / Advanced / tuning pages),
//! ConfigFlightModes and ConfigPlanner.

use std::collections::{BTreeMap, BTreeSet};

use egui::{Color32, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Vec2};
use mp_core::firmware::Firmware;
use mp_core::link::{Link, LinkCommand, LinkEvent};
use mp_core::{param_meta, paramfile};

use crate::app::{AppCtx, Toasts};
use crate::theme::BsvItem;
use crate::{fonts, platform, theme};

const PAGES: [&str; 11] = [
    "GeoFence",
    "Basic Tuning",
    "Extended Tuning",
    "Standard Params",
    "Advanced Params",
    "Onboard OSD",
    "MAVFtp",
    "User Params",
    "Full Parameter List",
    "Full Parameter Tree",
    "Planner",
];

pub struct ConfigView {
    page: usize,
    pub params: ParamEditor,
}

impl Default for ConfigView {
    fn default() -> Self {
        Self { page: 8, params: ParamEditor::default() }
    }
}

impl ConfigView {
    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        self.params.on_event(e, toasts);
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let items: Vec<BsvItem> = PAGES.iter().map(|p| BsvItem { label: p.to_string(), parent: None }).collect();
        egui::Panel::left("cfg_bsv")
            .resizable(false)
            .exact_size(theme::BSV_WIDTH)
            .frame(egui::Frame::new())
            .show(ui, |ui| theme::backstage(ui, &items, &mut self.page));
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::BG).inner_margin(6)).show(ui, |ui| {
            let fw = app.link.map(|l| l.shared.read().cs.firmware).unwrap_or_default();
            match self.page {
                0 => friendly(ui, app, &self.params, |n, _| n.starts_with("FENCE_")),
                1 => friendly(ui, app, &self.params, |n, _| {
                    matches!(n, "ATC_INPUT_TC" | "MOT_THST_HOVER" | "PILOT_ACCEL_Z" | "PILOT_SPEED_UP" | "ATC_RAT_RLL_P" | "ATC_RAT_PIT_P" | "ATC_RAT_YAW_P" | "ATC_ANG_RLL_P" | "ATC_ANG_PIT_P" | "ATC_ANG_YAW_P")
                }),
                2 => friendly(ui, app, &self.params, |n, _| n.starts_with("ATC_") || n.starts_with("PSC_") || n.starts_with("WPNAV_")),
                3 => friendly(ui, app, &self.params, |_, m| m.map(|m| m.user == "Standard").unwrap_or(false)),
                4 => friendly(ui, app, &self.params, |_, m| m.map(|m| m.user == "Advanced").unwrap_or(false)),
                5 => friendly(ui, app, &self.params, |n, _| n.starts_with("OSD")),
                6 => {
                    theme::label(ui, "MAVFtp: not available in this port yet");
                }
                7 => {
                    theme::label(ui, "User Params: add parameters to show here (not available in this port yet)");
                }
                8 => self.params.ui(ui, app, false),
                9 => self.params.ui(ui, app, true),
                _ => planner(ui, app, fw),
            }
        });
    }
}

// ------------------------------------------------------------------ ConfigFriendlyParams

/// One row per parameter: DisplayName (bold), description, then the control and the raw name.
pub fn friendly(ui: &mut egui::Ui, app: &mut AppCtx, _ed: &ParamEditor, filter: impl Fn(&str, Option<&param_meta::ParamMeta>) -> bool) {
    let Some(link) = app.link else {
        theme::label(ui, "Please Connect First");
        return;
    };
    let (params, fw) = {
        let s = link.shared.read();
        (s.params.clone(), s.cs.firmware)
    };
    let names: Vec<&String> = params.keys().filter(|k| filter(k, param_meta::get(fw, k))).collect();
    if names.is_empty() {
        theme::label(ui, if params.is_empty() { "Loading..." } else { "No parameters for this page on this vehicle" });
        return;
    }
    egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        for n in names {
            let meta = param_meta::get(fw, n);
            let val = params[n].value;
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.set_width((ui.available_width() - 260.0).max(200.0));
                    let title = meta.map(|m| m.display_name.clone()).filter(|s| !s.is_empty()).unwrap_or_else(|| n.clone());
                    ui.label(RichText::new(title).font(fonts::bold_id(12.0)).color(Color32::WHITE));
                    if let Some(m) = meta {
                        ui.label(RichText::new(&m.description).color(Color32::from_gray(200)));
                    }
                });
                param_control(ui, link, fw, n, val, 200.0);
                ui.label(RichText::new(format!("({n})")).color(Color32::GRAY));
            });
            ui.add_space(2.0);
            ui.separator();
        }
    });
}

/// MavlinkComboBox for enum params, MavlinkNumericUpDown (+ range) otherwise. Writes on change.
pub fn param_control(ui: &mut egui::Ui, link: &Link, fw: Firmware, name: &str, val: f32, width: f32) {
    let meta = param_meta::get(fw, name);
    match meta.filter(|m| !m.values.is_empty()) {
        Some(m) => {
            let cur = m.values.iter().find(|(k, _)| *k as f32 == val).map(|(_, s)| s.clone()).unwrap_or_else(|| paramfile::format_value(val));
            egui::ComboBox::from_id_salt(("pc", name)).width(width).selected_text(cur).show_ui(ui, |ui| {
                for (k, s) in &m.values {
                    if ui.selectable_label(*k as f32 == val, s).clicked() {
                        link.send(LinkCommand::SetParam { name: name.into(), value: *k as f32 });
                    }
                }
            });
        }
        None => {
            let mut v = val;
            let mut dv = egui::DragValue::new(&mut v).speed(meta.and_then(|m| m.increment).unwrap_or(0.01)).max_decimals(6);
            if let Some((lo, hi)) = meta.and_then(|m| m.range) {
                dv = dv.range(lo..=hi).clamp_existing_to_range(false);
            }
            let r = ui.add_sized(Vec2::new(width.min(100.0), 20.0), dv);
            if (r.drag_stopped() || r.lost_focus()) && v != val {
                link.send(LinkCommand::SetParam { name: name.into(), value: v });
            }
            if let Some((lo, hi)) = meta.and_then(|m| m.range) {
                theme::label(ui, format!("{lo} .. {hi}"));
            }
        }
    }
}

// ------------------------------------------------------------------ ConfigRawParams

#[derive(Default)]
pub struct ParamEditor {
    search: String,
    edits: BTreeMap<String, f32>,
    pending: BTreeSet<String>,
    selected_group: Option<String>,
    modified_only: bool,
    none_default: bool,
    tree_collapsed: bool,
}

impl ParamEditor {
    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        if let LinkEvent::ParamSet { name, ok } = e {
            if self.pending.remove(name) {
                if *ok {
                    self.edits.remove(name);
                } else {
                    toasts.error(format!("Set {name} Failed"));
                }
                if self.pending.is_empty() {
                    toasts.info("Parameters successfully saved.");
                }
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, tree_page: bool) {
        let Some(link) = app.link else {
            theme::label(ui, "Please Connect First");
            return;
        };
        let (params, fw) = {
            let s = link.shared.read();
            (s.params.clone(), s.cs.firmware)
        };

        // tableLayoutPanel1: Dock Right, 122px
        egui::Panel::right("raw_buttons").exact_size(122.0).resizable(false).frame(egui::Frame::new().fill(theme::BG)).show(ui, |ui| {
            let o = ui.max_rect().min;
            let r = |y: f32, h: f32| Rect::from_min_size(o + Vec2::new(6.0, y), Vec2::new(110.0, h));
            let b = |ui: &mut egui::Ui, rr: Rect, t: &str| ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, t, rr.size())).clicked();
            if b(ui, r(6.0, 19.0), "Load from file") {
                let loaded = if platform::HAS_NATIVE_DIALOGS {
                    platform::open_text("Load Parameters", "Param List", &["param", "parm", "txt"]).map(|(_, t)| t)
                } else {
                    platform::list_files(&["param"]).last().and_then(|p| std::fs::read_to_string(p).ok())
                };
                if let Some(text) = loaded {
                    for (k, v) in paramfile::load(&text) {
                        if params.get(&k).map(|p| p.value != v).unwrap_or(false) {
                            self.edits.insert(k, v);
                        }
                    }
                    self.modified_only = true;
                }
            }
            if b(ui, r(31.0, 19.0), "Save to file") {
                let text = paramfile::save(params.iter().map(|(k, p)| (k, p.value)));
                if let Some(Err(e)) = platform::save_text("Save Parameters", "params.param", "Param List", &["param"], &text) {
                    app.toasts.error(e);
                }
            }
            if b(ui, r(66.0, 19.0), "Write Params") {
                if self.edits.is_empty() {
                    app.toasts.info("No changes");
                }
                for (k, v) in &self.edits {
                    link.send(LinkCommand::SetParam { name: k.clone(), value: *v });
                    self.pending.insert(k.clone());
                }
            }
            if b(ui, r(91.0, 19.0), "Refresh Params") {
                self.edits.clear();
                link.send(LinkCommand::RequestParams);
            }
            if b(ui, r(116.0, 19.0), "Compare Params") {
                app.toasts.error("Compare Params: not available in this port yet");
            }
            if b(ui, r(141.0, 22.0), "Commit Params") {
                link.send(LinkCommand::CommandLong { command: 245, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
            }
            let f = fonts::ui_id(fonts::UI_SIZE);
            ui.painter().text(o + Vec2::new(6.0, 169.0), egui::Align2::LEFT_TOP, "All Units are in raw\nformat with no scaling", f.clone(), Color32::WHITE);
            ui.put(r(201.0, 21.0), |ui: &mut egui::Ui| egui::ComboBox::from_id_salt("CMB_paramfiles").width(102.0).selected_text("").show_ui(ui, |_| {}).response);
            if b(ui, r(228.0, 21.0), "Load Presaved") {
                app.toasts.error("Load Presaved: not available in this port yet");
            }
            if b(ui, r(255.0, 21.0), "Reset to Default") {
                app.toasts.error("Reset to Default: not available in this port yet");
            }
            ui.painter().text(o + Vec2::new(6.0, 286.0), egui::Align2::LEFT_TOP, "Search", f.clone(), Color32::WHITE);
            ui.put(r(302.0, 20.0), egui::TextEdit::singleline(&mut self.search));
            ui.put(Rect::from_min_size(o + Vec2::new(6.0, 328.0), Vec2::new(90.0, 17.0)), egui::Checkbox::new(&mut self.modified_only, "Modified"));
            ui.put(Rect::from_min_size(o + Vec2::new(6.0, 351.0), Vec2::new(100.0, 17.0)), egui::Checkbox::new(&mut self.none_default, "None Default"));
            b(ui, r(384.0, 22.0), "Refresh Table");
        });

        // splitContainer1.Panel1: treeView1 (180px) with the "<" collapse button
        let show_tree = tree_page || !self.tree_collapsed;
        if show_tree {
            let mut groups: BTreeMap<String, usize> = BTreeMap::new();
            for k in params.keys() {
                *groups.entry(group_of(k)).or_default() += 1;
            }
            egui::Panel::left("raw_tree").resizable(true).default_size(180.0).frame(egui::Frame::new().fill(theme::CONTROL_BG)).show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    if ui.selectable_label(self.selected_group.is_none(), RichText::new("All").color(Color32::WHITE)).clicked() {
                        self.selected_group = None;
                    }
                    for (g, _) in &groups {
                        let sel = self.selected_group.as_deref() == Some(g);
                        if ui.selectable_label(sel, RichText::new(format!("+ {g}")).color(Color32::WHITE)).clicked() {
                            self.selected_group = Some(g.clone());
                        }
                    }
                });
            });
        }
        egui::Panel::left("raw_collapse").exact_size(18.0).resizable(false).frame(egui::Frame::new().fill(theme::BG)).show(ui, |ui| {
            let r = Rect::from_min_size(ui.max_rect().min, Vec2::new(18.0, 18.0));
            if ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, if show_tree { "<" } else { ">" }, r.size())).clicked() {
                self.tree_collapsed = !self.tree_collapsed;
            }
        });

        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::BG)).show(ui, |ui| {
            let q = self.search.to_lowercase();
            let rows: Vec<(String, f32)> = params
                .iter()
                .filter(|(k, _)| !show_tree || self.selected_group.as_ref().map(|g| group_of(k) == *g).unwrap_or(true))
                .filter(|(k, _)| !self.modified_only || self.edits.contains_key(*k))
                .filter(|(k, _)| {
                    q.is_empty()
                        || k.to_lowercase().contains(&q)
                        || param_meta::get(fw, k).map(|m| m.description.to_lowercase().contains(&q) || m.display_name.to_lowercase().contains(&q)).unwrap_or(false)
                })
                .map(|(k, p)| (k.clone(), p.value))
                .collect();
            self.grid(ui, link, fw, &rows);
        });
    }

    /// The Params DataGridView: Name 130 | Value 70 | Default 70 | Units 60 | Options 150 | Desc | Fav 30
    fn grid(&mut self, ui: &mut egui::Ui, _link: &Link, fw: Firmware, rows: &[(String, f32)]) {
        let widths = [130.0f32, 70.0, 70.0, 60.0, 150.0, 0.0, 30.0];
        let headers = ["Name", "Value", "Default", "Units", "Options", "Desc", "Fav"];
        let avail = ui.available_width();
        let fixed: f32 = widths.iter().sum();
        let desc_w = (avail - fixed - 14.0).max(150.0);
        let col_w: Vec<f32> = widths.iter().map(|w| if *w == 0.0 { desc_w } else { *w }).collect();
        let (hdr_h, row_h) = (21.0, 22.0);
        let f = fonts::ui_id(fonts::UI_SIZE);
        let line = Color32::from_rgb(0xa0, 0xa0, 0xa0);

        // header
        let (hr, _) = ui.allocate_exact_size(Vec2::new(col_w.iter().sum(), hdr_h), Sense::hover());
        let mut x = hr.left();
        for (i, w) in col_w.iter().enumerate() {
            let r = Rect::from_min_size(Pos2::new(x, hr.top()), Vec2::new(*w, hdr_h));
            ui.painter().rect_filled(r, 0.0, theme::BG);
            ui.painter().rect_stroke(r, 0.0, Stroke::new(1.0, line), StrokeKind::Inside);
            ui.painter().text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, headers[i], f.clone(), Color32::WHITE);
            x += w;
        }

        egui::ScrollArea::vertical().auto_shrink([false, false]).show_rows(ui, row_h, rows.len(), |ui, range| {
            for i in range {
                let (name, value) = &rows[i];
                let meta = param_meta::get(fw, name);
                let (rr, _) = ui.allocate_exact_size(Vec2::new(col_w.iter().sum(), row_h), Sense::hover());
                let bg = if i % 2 == 1 { theme::BG } else { theme::CONTROL_BG };
                let mut cells = Vec::new();
                let mut x = rr.left();
                for w in &col_w {
                    cells.push(Rect::from_min_size(Pos2::new(x, rr.top()), Vec2::new(*w, row_h)));
                    x += w;
                }
                let edited = self.edits.get(name).copied();
                for (c, r) in cells.iter().enumerate() {
                    // edited values are highlighted green, as Mission Planner does
                    let fill = if c == 1 && edited.is_some() { Color32::from_rgb(0, 128, 0) } else { bg };
                    ui.painter().rect_filled(*r, 0.0, fill);
                    ui.painter().rect_stroke(*r, 0.0, Stroke::new(1.0, line), StrokeKind::Inside);
                }
                let txt = |ui: &egui::Ui, r: Rect, s: &str| {
                    ui.painter().with_clip_rect(r.shrink(1.0)).text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, s, f.clone(), Color32::WHITE);
                };
                txt(ui, cells[0], name);
                if let Some(m) = meta {
                    ui.interact(cells[0], ui.scope_id().with(("pn", name)), Sense::hover()).on_hover_text(format!("{}\n{}", m.display_name, m.description));
                }
                // Value cell: editable
                let mut v = edited.unwrap_or(*value);
                let mut child = ui.new_child(egui::UiBuilder::new().max_rect(cells[1].shrink(1.0)));
                {
                    let vis = child.visuals_mut();
                    for s in [&mut vis.widgets.inactive, &mut vis.widgets.hovered] {
                        s.bg_fill = Color32::TRANSPARENT;
                        s.weak_bg_fill = Color32::TRANSPARENT;
                        s.bg_stroke = Stroke::NONE;
                    }
                }
                let changed = match meta.filter(|m| !m.values.is_empty()) {
                    Some(m) => {
                        let mut c = false;
                        let cur = m.values.iter().find(|(k, _)| *k as f32 == v).map(|(k, _)| k.to_string()).unwrap_or_else(|| paramfile::format_value(v));
                        child.put(cells[1].shrink(1.0), |ui: &mut egui::Ui| {
                            egui::ComboBox::from_id_salt(("pv", name))
                                .width(cells[1].width() - 6.0)
                                .selected_text(cur)
                                .show_ui(ui, |ui| {
                                    for (k, s) in &m.values {
                                        if ui.selectable_label(*k as f32 == v, format!("{k}: {s}")).clicked() {
                                            v = *k as f32;
                                            c = true;
                                        }
                                    }
                                })
                                .response
                        });
                        c
                    }
                    None => {
                        let mut dv = egui::DragValue::new(&mut v).speed(meta.and_then(|m| m.increment).unwrap_or(0.01)).max_decimals(6);
                        if let Some((lo, hi)) = meta.and_then(|m| m.range) {
                            dv = dv.range(lo..=hi).clamp_existing_to_range(false);
                        }
                        child.put(cells[1].shrink(1.0), dv).changed()
                    }
                };
                if changed {
                    if v == *value {
                        self.edits.remove(name);
                    } else {
                        self.edits.insert(name.clone(), v);
                    }
                }
                txt(ui, cells[2], "");
                txt(ui, cells[3], meta.map(|m| m.units.as_str()).unwrap_or(""));
                if let Some(m) = meta {
                    let opts = if let Some((lo, hi)) = m.range {
                        format!("{lo} {hi}")
                    } else if !m.bitmask.is_empty() {
                        m.bitmask.iter().map(|(b, s)| format!("{b}:{s}")).collect::<Vec<_>>().join(", ")
                    } else {
                        m.values.iter().map(|(k, s)| format!("{k}:{s}")).collect::<Vec<_>>().join(", ")
                    };
                    txt(ui, cells[4], &opts);
                    txt(ui, cells[5], &m.description);
                    if !m.description.is_empty() {
                        ui.interact(cells[5], ui.scope_id().with(("pd", name)), Sense::hover()).on_hover_text(&m.description);
                    }
                }
                let mut fav = false;
                ui.put(Rect::from_center_size(cells[6].center(), Vec2::splat(16.0)), egui::Checkbox::without_text(&mut fav));
            }
        });
    }
}

fn group_of(name: &str) -> String {
    name.split('_').next().unwrap_or(name).to_string()
}

// ------------------------------------------------------------------ ConfigFlightModes

pub fn flight_modes(ui: &mut egui::Ui, app: &mut AppCtx) {
    let Some(link) = app.link else {
        theme::label(ui, "Please Connect First");
        return;
    };
    let (params, cs) = {
        let s = link.shared.read();
        (s.params.clone(), s.cs.clone())
    };
    let fw = cs.firmware;
    let prefix = fw.mode_param_prefix();
    let modes = fw.modes();
    let ch_param = if fw == Firmware::ArduRover { "MODE_CH" } else { "FLTMODE_CH" };
    let ch = params.get(ch_param).map(|p| p.value as usize).unwrap_or(if fw == Firmware::ArduPlane { 8 } else { 5 });
    let pwm = if (1..=18).contains(&ch) { cs.ch_in[ch - 1] } else { 0 };
    let pos = match pwm {
        0 => 0,
        p if p <= 1230 => 1,
        p if p <= 1360 => 2,
        p if p <= 1490 => 3,
        p if p <= 1620 => 4,
        p if p <= 1749 => 5,
        _ => 6,
    };
    let ranges = ["PWM 0 - 1230", "PWM 1231 - 1360", "PWM 1361 - 1490", "PWM 1491 - 1620", "PWM 1621 - 1749", "PWM 1750 +"];
    let mut sel: [u32; 6] = [0; 6];
    for (i, s) in sel.iter_mut().enumerate() {
        *s = params.get(&format!("{prefix}{}", i + 1)).map(|p| p.value as u32).unwrap_or(0);
    }
    let pending_id = egui::Id::unique("fm_pending");
    let mut pending: [u32; 6] = ui.ctx().data(|d| d.get_temp(pending_id)).unwrap_or(sel);

    egui::Grid::new("fltmodes").num_columns(4).spacing(Vec2::new(12.0, 8.0)).show(ui, |ui| {
        for i in 0..6 {
            let label = RichText::new(format!("Flight Mode {}", i + 1)).color(Color32::WHITE);
            let r = ui.label(label);
            if pos == i + 1 {
                ui.painter().rect_filled(r.rect.expand(2.0), 0.0, Color32::from_rgb(0, 128, 0).gamma_multiply(0.6));
                ui.painter().text(r.rect.left_center(), egui::Align2::LEFT_CENTER, format!("Flight Mode {}", i + 1), fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
            }
            let name = modes.iter().find(|(k, _)| *k == pending[i]).map(|(_, n)| n.clone()).unwrap_or_else(|| pending[i].to_string());
            egui::ComboBox::from_id_salt(("fm", i)).width(160.0).selected_text(name).show_ui(ui, |ui| {
                for (k, n) in &modes {
                    ui.selectable_value(&mut pending[i], *k, n);
                }
            });
            let simple = format!("SIMPLE {}", i + 1);
            let mut s = false;
            ui.checkbox(&mut s, "Simple Mode");
            let _ = simple;
            ui.label(RichText::new(ranges[i]).color(Color32::WHITE));
            ui.end_row();
        }
    });
    ui.ctx().data_mut(|d| d.insert_temp(pending_id, pending));
    ui.add_space(8.0);
    ui.label(RichText::new(format!("Current Mode: {}", cs.mode)).color(Color32::WHITE));
    ui.label(RichText::new(format!("Current PWM: {ch}: {pwm}")).color(Color32::WHITE));
    ui.add_space(6.0);
    if theme::button(ui, "Save Modes").clicked() {
        for i in 0..6 {
            if pending[i] != sel[i] {
                link.send(LinkCommand::SetParam { name: format!("{prefix}{}", i + 1), value: pending[i] as f32 });
            }
        }
    }
}

// ------------------------------------------------------------------ ConfigPlanner

fn planner(ui: &mut egui::Ui, app: &mut AppCtx, _fw: Firmware) {
    egui::Grid::new("planner").num_columns(2).spacing(Vec2::new(16.0, 8.0)).show(ui, |ui| {
        theme::label(ui, "Map Type (Flight Data)");
        crate::views::flight_data::provider_combo(ui, &mut app.settings.fd_map.provider, "pl_fd");
        ui.end_row();
        theme::label(ui, "Map Type (Flight Plan)");
        crate::views::flight_data::provider_combo(ui, &mut app.settings.fp_map.provider, "pl_fp");
        ui.end_row();
        theme::label(ui, "Layout / UI Scale");
        if ui.add(egui::Slider::new(&mut app.settings.ui_scale, 0.6..=3.0).step_by(0.1)).changed() {
            ui.ctx().set_zoom_factor(app.settings.ui_scale);
        }
        ui.end_row();
        theme::label(ui, "Units: Distance / Speed");
        theme::label(ui, "Meters / meters_per_second");
        ui.end_row();
        theme::label(ui, "Simulator UDP port");
        ui.add(egui::DragValue::new(&mut app.settings.sim_port));
        ui.end_row();
        theme::label(ui, "Map cache");
        ui.horizontal(|ui| {
            theme::label(ui, platform::tile_cache_dir().display().to_string());
            if theme::button(ui, "Clear").clicked() {
                let _ = std::fs::remove_dir_all(platform::tile_cache_dir());
                app.toasts.info("Map cache cleared");
            }
        });
        ui.end_row();
    });
}
