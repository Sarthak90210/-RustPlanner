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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Page {
    GeoFence,
    BasicTuning,
    ExtendedTuning,
    StandardParams,
    AdvancedParams,
    OnboardOsd,
    MavFtp,
    UserParams,
    FullParamList,
    Planner,
}

impl Page {
    fn label(&self, fw: Firmware) -> &'static str {
        match self {
            Page::GeoFence => "GeoFence",
            Page::BasicTuning => "Basic Tuning",
            Page::ExtendedTuning if fw == Firmware::ArduPlane => "QP Extended Tuning",
            Page::ExtendedTuning => "Extended Tuning",
            Page::StandardParams => "Standard Params",
            Page::AdvancedParams => "Advanced Params",
            Page::OnboardOsd => "Onboard OSD",
            Page::MavFtp => "MAVFtp",
            Page::UserParams => "User Params",
            Page::FullParamList => "Full Parameter List",
            Page::Planner => "Planner",
        }
    }
}

/// SoftwareConfig.cs page list: vehicle pages only once connected with all params.
fn pages(app: &AppCtx) -> Vec<Page> {
    let mut v = Vec::new();
    if let Some(l) = app.link.filter(|_| app.connected()) {
        let s = l.shared.read();
        let fw = s.cs.firmware;
        let has = |p: &str| s.params.keys().any(|k| k.starts_with(p));
        if s.params_complete {
            if fw == Firmware::ArduCopter2 {
                v.push(Page::GeoFence);
            }
            v.push(Page::BasicTuning);
            if matches!(fw, Firmware::ArduCopter2 | Firmware::ArduPlane) {
                v.push(Page::ExtendedTuning);
            }
            if crate::display::friendly_params(&app.settings.planner) {
                v.push(Page::StandardParams);
                v.push(Page::AdvancedParams);
            }
            if has("OSD") {
                v.push(Page::OnboardOsd);
            }
            // MAV_PROTOCOL_CAPABILITY_FTP
            if s.autopilot_version.map(|(_, caps)| caps & 32 != 0).unwrap_or(false) {
                v.push(Page::MavFtp);
            }
            v.push(Page::UserParams);
        }
    }
    v.push(Page::FullParamList);
    v.push(Page::Planner);
    v
}

pub struct ConfigView {
    page: Page,
    pub params: ParamEditor,
    extended: crate::views::tuning::ExtendedTuning,
    user_add: String,
    ftp: crate::views::mavftp::MavFtpUi,
    osd: crate::views::osd_editor::OsdEditor,
    standard: crate::views::friendly::FriendlyParams,
    advanced: crate::views::friendly::FriendlyParams,
    last_page: Option<Page>,
}

impl Default for ConfigView {
    fn default() -> Self {
        let page = match std::env::var("MP_PAGE").ok().and_then(|v| v.parse::<usize>().ok()) {
            Some(1) => Page::BasicTuning,
            Some(2) => Page::ExtendedTuning,
            Some(3) => Page::StandardParams,
            Some(4) => Page::AdvancedParams,
            Some(5) => Page::OnboardOsd,
            Some(6) => Page::MavFtp,
            Some(7) => Page::UserParams,
            Some(10) => Page::Planner,
            Some(0) => Page::GeoFence,
            _ => Page::FullParamList,
        };
        Self { page, params: ParamEditor::default(), extended: Default::default(), user_add: String::new(), ftp: Default::default(), osd: Default::default(), standard: Default::default(), advanced: Default::default(), last_page: None }
    }
}

impl ConfigView {
    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        self.params.on_event(e, toasts);
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let fw = app.link.map(|l| l.shared.read().cs.firmware).unwrap_or_default();
        let list = pages(app);
        let shown = if list.contains(&self.page) { self.page } else { Page::FullParamList };
        let items: Vec<BsvItem> = list.iter().map(|p| BsvItem { label: p.label(fw).to_string(), parent: None }).collect();
        let before = list.iter().position(|p| *p == shown).unwrap_or(0);
        let mut idx = before;
        egui::Panel::left("cfg_bsv")
            .resizable(false)
            .exact_size(theme::BSV_WIDTH)
            .frame(egui::Frame::new())
            .show(ui, |ui| theme::backstage(ui, &items, &mut idx));
        if idx != before {
            self.page = list[idx];
        }
        let page = if idx != before { self.page } else { shown };
        // cbAutoWriteOnLeave
        if self.last_page == Some(Page::OnboardOsd) && page != Page::OnboardOsd {
            self.osd.leave(app);
        }
        self.last_page = Some(page);
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg()).inner_margin(6)).show(ui, |ui| match page {
            Page::GeoFence => crate::views::tuning::geofence(ui, app),
            Page::BasicTuning => crate::views::tuning::basic_tuning(ui, app),
            Page::ExtendedTuning => self.extended.ui(ui, app),
            Page::StandardParams => self.standard.ui(ui, app, "Standard"),
            Page::AdvancedParams => self.advanced.ui(ui, app, "Advanced"),
            Page::OnboardOsd => self.osd.ui(ui, app),
            Page::MavFtp => self.ftp.ui(ui, app),
            Page::UserParams => self.user_params(ui, app),
            Page::FullParamList => self.params.ui(ui, app, false),
            Page::Planner => planner(ui, app, fw),
        });
    }

    /// ConfigUserDefined: a user-chosen list of parameters.
    fn user_params(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let Some(link) = app.link else { return };
        let all: Vec<String> = link.shared.read().params.keys().cloned().collect();
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("user_add").width(200.0).selected_text(self.user_add.clone()).show_ui(ui, |ui| {
                for n in &all {
                    ui.selectable_value(&mut self.user_add, n.clone(), n);
                }
            });
            if theme::button(ui, "Add").clicked() && !self.user_add.is_empty() && !app.settings.user_params.contains(&self.user_add) {
                app.settings.user_params.push(self.user_add.clone());
            }
            if theme::button(ui, "Clear").clicked() {
                app.settings.user_params.clear();
            }
        });
        ui.separator();
        let chosen = app.settings.user_params.clone();
        friendly(ui, app, &self.params, |n, _| chosen.iter().any(|c| c == n));
    }
}

// ------------------------------------------------------------------ ConfigFriendlyParams

/// One row per parameter: DisplayName (bold), description, then the control and the raw name.
pub fn friendly(ui: &mut egui::Ui, app: &mut AppCtx, _ed: &ParamEditor, filter: impl Fn(&str, Option<&param_meta::ParamMeta>) -> bool) {
    let Some(link) = app.link else {
        theme::label(ui, "Please Connect First");
        return;
    };
    let names: Vec<String> = {
        let s = link.shared.read();
        let fw = s.cs.firmware;
        s.params.keys().filter(|k| filter(k, param_meta::get(fw, k))).cloned().collect()
    };
    // pages built from ConfigFriendlyParams blocks write each change straight away
    crate::views::friendly::rows(ui, app, &names, None);
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
    compare: Option<Vec<(String, f32, f32, bool)>>,
    /// CMB_paramfiles: Tools/Frame_params/*.param from the ArduPilot repo (name, path)
    paramfiles: std::sync::Arc<std::sync::Mutex<Option<Result<Vec<(String, String)>, String>>>>,
    paramfiles_requested: bool,
    paramfile_sel: usize,
    /// presaved file fetched in the background
    presaved: std::sync::Arc<std::sync::Mutex<Option<Result<String, String>>>>,
    confirm_reset: bool,
    /// parameter defaults from @PARAM/param.pck?withdefaults=1 (MAVFtp)
    defaults: std::sync::Arc<std::sync::Mutex<Option<BTreeMap<String, f32>>>>,
    defaults_requested: bool,
    editing: Option<(String, String)>,
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

        // MAVLinkInterface.getParamList: the packed list with defaults over MAVFtp
        if !self.defaults_requested && link.shared.read().autopilot_version.map(|(_, c)| c & 32 != 0).unwrap_or(false) {
            self.defaults_requested = true;
            let mut ftp = mp_core::ftp::Ftp::new(link);
            let slot = self.defaults.clone();
            let ctx = ui.ctx().clone();
            std::thread::spawn(move || {
                if let Ok(b) = ftp.get("@PARAM/param.pck?withdefaults=1", |_, _| {}) {
                    if let Some(list) = mp_core::parampck::unpack(&b) {
                        let map: BTreeMap<String, f32> = list.into_iter().filter_map(|(n, _, d)| d.map(|d| (n, d))).collect();
                        if !map.is_empty() {
                            *slot.lock().unwrap() = Some(map);
                            ctx.request_repaint();
                        }
                    }
                }
            });
        }
        let defaults = self.defaults.lock().unwrap().clone();

        // tableLayoutPanel1: Dock Right, 122px
        egui::Panel::right("raw_buttons").exact_size(122.0).resizable(false).frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
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
                if let Some((_, text)) = platform::open_text("Compare Parameters", "Param List", &["param", "parm", "txt"]) {
                    let diffs: Vec<(String, f32, f32, bool)> = paramfile::load(&text)
                        .into_iter()
                        .filter_map(|(k, v)| params.get(&k).filter(|p| p.value != v).map(|p| (k, p.value, v, true)))
                        .collect();
                    self.compare = Some(diffs);
                }
            }
            if b(ui, r(141.0, 22.0), "Commit Params") {
                link.send(LinkCommand::CommandLong { command: 245, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
            }
            let f = fonts::ui_id(fonts::UI_SIZE);
            ui.painter().text(o + Vec2::new(6.0, 169.0), egui::Align2::LEFT_TOP, "All Units are in raw\nformat with no scaling", f.clone(), Color32::WHITE);
            // GitHubContent.GetDirContent("ardupilot", "ardupilot", "/Tools/Frame_params/")
            if !self.paramfiles_requested {
                self.paramfiles_requested = true;
                let slot = self.paramfiles.clone();
                let ctx = ui.ctx().clone();
                std::thread::spawn(move || {
                    let r = crate::firmware_ui::get("https://api.github.com/repos/ardupilot/ardupilot/contents/Tools/Frame_params").and_then(|b| {
                        let v: serde_json::Value = serde_json::from_slice(&b).map_err(|e| e.to_string())?;
                        Ok(v.as_array()
                            .map(|a| {
                                a.iter()
                                    .filter_map(|e| Some((e["name"].as_str()?.to_string(), e["path"].as_str()?.to_string())))
                                    .filter(|(n, _)| n.ends_with(".param"))
                                    .collect()
                            })
                            .unwrap_or_default())
                    });
                    *slot.lock().unwrap() = Some(r);
                    ctx.request_repaint();
                });
            }
            let files: Vec<(String, String)> = match &*self.paramfiles.lock().unwrap() {
                Some(Ok(v)) => v.clone(),
                _ => Vec::new(),
            };
            let sel_name = files.get(self.paramfile_sel).map(|f| f.0.clone()).unwrap_or_default();
            ui.put(r(201.0, 21.0), |ui: &mut egui::Ui| {
                ui.add_enabled_ui(!files.is_empty(), |ui| {
                    egui::ComboBox::from_id_salt("CMB_paramfiles")
                        .width(102.0)
                        .wrap_mode(egui::TextWrapMode::Truncate)
                        .selected_text(sel_name)
                        .show_ui(ui, |ui| {
                            for (i, (n, _)) in files.iter().enumerate() {
                                ui.selectable_value(&mut self.paramfile_sel, i, n);
                            }
                        })
                        .response
                })
                .inner
            });
            if b(ui, r(228.0, 21.0), "Load Presaved") {
                if let Some((_, path)) = files.get(self.paramfile_sel).cloned() {
                    let slot = self.presaved.clone();
                    let ctx = ui.ctx().clone();
                    std::thread::spawn(move || {
                        let r = crate::firmware_ui::get(&format!("https://raw.githubusercontent.com/ArduPilot/ardupilot/master/{path}")).map(|b| String::from_utf8_lossy(&b).into_owned());
                        *slot.lock().unwrap() = Some(r);
                        ctx.request_repaint();
                    });
                } else {
                    app.toasts.error("Failed to load file.\nNo presaved parameter files are available");
                }
            }
            if b(ui, r(255.0, 21.0), "Reset to Default") {
                self.confirm_reset = true;
            }
            ui.painter().text(o + Vec2::new(6.0, 286.0), egui::Align2::LEFT_TOP, "Search", f.clone(), Color32::WHITE);
            ui.put(r(302.0, 20.0), egui::TextEdit::singleline(&mut self.search));
            ui.put(Rect::from_min_size(o + Vec2::new(6.0, 328.0), Vec2::new(90.0, 17.0)), egui::Checkbox::new(&mut self.modified_only, "Modified"));
            // chk_none_default is only shown when the vehicle reported defaults
            if defaults.is_some() {
                ui.put(Rect::from_min_size(o + Vec2::new(6.0, 351.0), Vec2::new(100.0, 17.0)), egui::Checkbox::new(&mut self.none_default, "None Default"));
            }
            b(ui, r(384.0, 22.0), "Refresh Table");
        });

        // splitContainer1.Panel1: treeView1 (180px) with the "<" collapse button
        let show_tree = tree_page || !self.tree_collapsed;
        if show_tree {
            let mut groups: BTreeMap<String, usize> = BTreeMap::new();
            for k in params.keys() {
                *groups.entry(group_of(k)).or_default() += 1;
            }
            egui::Panel::left("raw_tree").resizable(true).default_size(180.0).frame(egui::Frame::new().fill(theme::control_bg())).show(ui, |ui| {
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
        egui::Panel::left("raw_collapse").exact_size(18.0).resizable(false).frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
            let r = Rect::from_min_size(ui.max_rect().min, Vec2::new(18.0, 18.0));
            if ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, if show_tree { "<" } else { ">" }, r.size())).clicked() {
                self.tree_collapsed = !self.tree_collapsed;
            }
        });

        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
            let q = self.search.to_lowercase();
            let rows: Vec<(String, f32)> = params
                .iter()
                .filter(|(k, _)| !show_tree || self.selected_group.as_ref().map(|g| group_of(k) == *g).unwrap_or(true))
                .filter(|(k, _)| !self.modified_only || self.edits.contains_key(*k))
                .filter(|(k, p)| !self.none_default || defaults.as_ref().and_then(|d| d.get(*k)).map(|d| *d != p.value).unwrap_or(false))
                .filter(|(k, _)| {
                    q.is_empty()
                        || k.to_lowercase().contains(&q)
                        || param_meta::get(fw, k).map(|m| m.description.to_lowercase().contains(&q) || m.display_name.to_lowercase().contains(&q)).unwrap_or(false)
                })
                .map(|(k, p)| (k.clone(), p.value))
                .collect();
            let mut rows = rows;
            let favs = app.settings.fav_params.clone();
            rows.sort_by_key(|(k, _)| !favs.contains(k));
            self.grid(ui, link, fw, &rows, &mut app.settings.fav_params, defaults.as_ref());
        });
        // presaved file arrived: show it in ParamCompare
        if let Some(r) = self.presaved.lock().unwrap().take() {
            match r {
                Ok(text) => {
                    let diffs: Vec<(String, f32, f32, bool)> = paramfile::load(&text)
                        .into_iter()
                        .filter_map(|(k, v)| params.get(&k).filter(|p| p.value != v).map(|p| (k, p.value, v, true)))
                        .collect();
                    self.compare = Some(diffs);
                }
                Err(e) => app.toasts.error(format!("Failed to load file.\n{e}")),
            }
        }
        if self.confirm_reset {
            let mut answer = None;
            egui::Modal::new(egui::Id::new("reset_params")).show(ui.ctx(), |ui| {
                ui.set_width(260.0);
                ui.label(egui::RichText::new("Reset").strong());
                theme::label(ui, "Reset all parameters to default\nAre you sure!!");
                ui.horizontal(|ui| {
                    if theme::button(ui, "Yes").clicked() {
                        answer = Some(true);
                    }
                    if theme::button(ui, "No").clicked() {
                        answer = Some(false);
                    }
                });
            });
            if let Some(yes) = answer {
                self.confirm_reset = false;
                if yes && !app.readonly {
                    // FORMAT_VERSION / SYSID_SW_MREV = 0 then reboot: the autopilot reloads defaults
                    for n in ["FORMAT_VERSION", "SYSID_SW_MREV"] {
                        if params.contains_key(n) {
                            link.send(LinkCommand::SetParam { name: n.into(), value: 0.0 });
                        }
                    }
                    link.send(LinkCommand::CommandLong { command: 246, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    app.toasts.error("Your board is now rebooting, You will be required to reconnect to the autopilot.");
                }
            }
        }
        self.compare_window(ui.ctx());
    }

    /// ParamCompare form: diff against a file, tick the rows to take, Continue.
    fn compare_window(&mut self, ctx: &egui::Context) {
        let Some(diffs) = self.compare.as_mut() else { return };
        let mut open = true;
        let mut apply = false;
        egui::Window::new("Param Compare").open(&mut open).collapsible(false).default_size(Vec2::new(480.0, 400.0)).show(ctx, |ui| {
            if diffs.is_empty() {
                theme::label(ui, "No differences");
                return;
            }
            egui::Grid::new("cmp").num_columns(4).striped(true).show(ui, |ui| {
                for h in ["Command", "Value", "New Value", "Use"] {
                    ui.label(RichText::new(h).color(Color32::WHITE).strong());
                }
                ui.end_row();
                for (n, cur, new, take) in diffs.iter_mut() {
                    theme::label(ui, n.as_str());
                    theme::label(ui, paramfile::format_value(*cur));
                    theme::label(ui, paramfile::format_value(*new));
                    ui.checkbox(take, "");
                    ui.end_row();
                }
            });
            if theme::button(ui, "Continue").clicked() {
                apply = true;
            }
        });
        if apply {
            for (n, _, new, take) in self.compare.take().unwrap_or_default() {
                if take {
                    self.edits.insert(n, new);
                }
            }
            self.modified_only = true;
        } else if !open {
            self.compare = None;
        }
    }

    /// The Params DataGridView: Name 130 | Value 70 | Default 70 | Units 60 | Options 150 | Desc | Fav 30
    fn grid(&mut self, ui: &mut egui::Ui, _link: &Link, fw: Firmware, rows: &[(String, f32)], favs: &mut Vec<String>, defaults: Option<&BTreeMap<String, f32>>) {
        // Default_value.Visible = has_defaults
        let widths = [130.0f32, 70.0, if defaults.is_some() { 70.0 } else { -1.0 }, 60.0, 150.0, 0.0, 30.0];
        let headers = ["Name", "Value", "Default", "Units", "Options", "Desc", "Fav"];
        let avail = ui.available_width();
        let fixed: f32 = widths.iter().map(|w| w.max(0.0)).sum();
        let desc_w = (avail - fixed - 14.0).max(150.0);
        let col_w: Vec<f32> = widths.iter().map(|w| if *w == 0.0 { desc_w } else { w.max(0.0) }).collect();
        let (hdr_h, row_h) = (21.0, 22.0);
        let f = fonts::ui_id(fonts::UI_SIZE);
        let line = Color32::from_rgb(0xa0, 0xa0, 0xa0);

        // header
        let (hr, _) = ui.allocate_exact_size(Vec2::new(col_w.iter().sum(), hdr_h), Sense::hover());
        let mut x = hr.left();
        for (i, w) in col_w.iter().enumerate() {
            if *w <= 0.0 {
                continue;
            }
            let r = Rect::from_min_size(Pos2::new(x, hr.top()), Vec2::new(*w, hdr_h));
            ui.painter().rect_filled(r, 0.0, theme::bg());
            ui.painter().rect_stroke(r, 0.0, Stroke::new(1.0, line), StrokeKind::Inside);
            ui.painter().text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, headers[i], f.clone(), Color32::WHITE);
            x += w;
        }

        egui::ScrollArea::vertical().auto_shrink([false, false]).show_rows(ui, row_h, rows.len(), |ui, range| {
            for i in range {
                let (name, value) = &rows[i];
                let meta = param_meta::get(fw, name);
                let (rr, _) = ui.allocate_exact_size(Vec2::new(col_w.iter().sum(), row_h), Sense::hover());
                let bg = if i % 2 == 1 { theme::bg() } else { theme::control_bg() };
                let mut cells = Vec::new();
                let mut x = rr.left();
                for w in &col_w {
                    cells.push(Rect::from_min_size(Pos2::new(x, rr.top()), Vec2::new(*w, row_h)));
                    x += w;
                }
                let edited = self.edits.get(name).copied();
                for (c, r) in cells.iter().enumerate().filter(|(_, r)| r.width() > 0.0) {
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
                // DataGridView: plain text until the cell is clicked (EditingControlShowing)
                let editing = self.editing.as_ref().map(|(n, _)| n == name).unwrap_or(false);
                let mut changed = false;
                if !editing {
                    let resp = child.put(cells[1].shrink(1.0), egui::Label::new(egui::RichText::new(paramfile::format_value(v)).color(Color32::WHITE)).sense(Sense::click()));
                    if resp.clicked() {
                        self.editing = Some((name.clone(), paramfile::format_value(v)));
                    }
                } else if let Some(m) = meta.filter(|m| !m.values.is_empty()) {
                    let cur = paramfile::format_value(v);
                    child.put(cells[1].shrink(1.0), |ui: &mut egui::Ui| {
                        let r = egui::ComboBox::from_id_salt(("pv", name))
                            .width(cells[1].width() - 6.0)
                            .selected_text(cur)
                            .show_ui(ui, |ui| {
                                for (k, s) in &m.values {
                                    if ui.selectable_label(*k as f32 == v, format!("{k}: {s}")).clicked() {
                                        v = *k as f32;
                                        changed = true;
                                    }
                                }
                            })
                            .response;
                        r
                    });
                    if changed {
                        self.editing = None;
                    }
                } else if let Some((_, buf)) = self.editing.as_mut() {
                    let r = child.put(cells[1].shrink(1.0), egui::TextEdit::singleline(buf));
                    if !r.has_focus() && !r.lost_focus() {
                        r.request_focus();
                    }
                    if r.lost_focus() {
                        if let Ok(nv) = buf.trim().parse::<f32>() {
                            v = nv;
                            changed = true;
                        }
                        self.editing = None;
                    }
                }
                if changed {
                    if v == *value {
                        self.edits.remove(name);
                    } else {
                        self.edits.insert(name.clone(), v);
                    }
                }
                if let Some(d) = defaults.and_then(|d| d.get(name)) {
                    txt(ui, cells[2], &paramfile::format_value(*d));
                }
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
                let mut fav = favs.contains(name);
                if ui.put(Rect::from_center_size(cells[6].center(), Vec2::splat(16.0)), egui::Checkbox::without_text(&mut fav)).changed() {
                    if fav {
                        favs.push(name.clone());
                    } else {
                        favs.retain(|f| f != name);
                    }
                }
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
    let current: [u32; 6] = std::array::from_fn(|i| params.get(&format!("{prefix}{}", i + 1)).map(|p| p.value as u32).unwrap_or(0));
    let simple_p = params.get("SIMPLE").map(|p| p.value as u32);
    let ss_p = params.get("SUPER_SIMPLE").map(|p| p.value as u32);
    let state_id = egui::Id::unique("fm_state");
    // the form re-reads the vehicle values whenever they change (refresh / write / late param load)
    let vehicle = (current, simple_p.unwrap_or(0), ss_p.unwrap_or(0));
    let seen_id = egui::Id::unique("fm_seen");
    let seen: Option<([u32; 6], u32, u32)> = ui.ctx().data(|d| d.get_temp(seen_id));
    if seen != Some(vehicle) {
        ui.ctx().data_mut(|d| {
            d.insert_temp(seen_id, vehicle);
            d.remove::<([u32; 6], u32, u32)>(state_id);
        });
    }
    let mut st: ([u32; 6], u32, u32) = ui.ctx().data(|d| d.get_temp(state_id)).unwrap_or(vehicle);

    let o = ui.max_rect().min;
    let f = fonts::ui_id(fonts::UI_SIZE);
    let p = ui.painter().clone();
    let at = |x: f32, y: f32| o + Vec2::new(x, y);
    p.text(at(94.0, 15.0), egui::Align2::LEFT_TOP, "Current Mode:", f.clone(), Color32::WHITE);
    p.text(at(174.0, 15.0), egui::Align2::LEFT_TOP, &cs.mode, f.clone(), Color32::WHITE);
    p.text(at(94.0, 32.0), egui::Align2::LEFT_TOP, "Current PWM:", f.clone(), Color32::WHITE);
    p.text(at(174.0, 32.0), egui::Align2::LEFT_TOP, format!("{ch}: {pwm}"), f.clone(), Color32::WHITE);
    let ty = 48.0;
    for i in 0..6 {
        let y = ty + i as f32 * 25.0;
        let lr = Rect::from_min_size(at(3.0, y), Vec2::new(104.0, 25.0));
        if pos == i + 1 {
            p.rect_filled(lr, 0.0, Color32::from_rgb(0, 128, 0));
        }
        p.text(lr.left_center() + Vec2::new(2.0, 0.0), egui::Align2::LEFT_CENTER, format!("Flight Mode {}", i + 1), f.clone(), Color32::WHITE);
        let name = modes.iter().find(|(k, _)| *k == st.0[i]).map(|(_, n)| n.clone()).unwrap_or_else(|| st.0[i].to_string());
        ui.put(Rect::from_min_size(at(113.0, y + 3.0), Vec2::new(104.0, 20.0)), |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt(("CMB_fmode", i))
                .width(96.0)
                .wrap_mode(egui::TextWrapMode::Truncate)
                .selected_text(name)
                .show_ui(ui, |ui| {
                    for (k, n) in &modes {
                        ui.selectable_value(&mut st.0[i], *k, n);
                    }
                })
                .response
        });
        if simple_p.is_some() {
            let mut b = st.1 & (1 << i) != 0;
            if ui.put(Rect::from_min_size(at(222.0, y + 2.0), Vec2::new(105.0, 16.0)), egui::Checkbox::new(&mut b, "Simple Mode")).changed() {
                st.1 = if b { st.1 | (1 << i) } else { st.1 & !(1 << i) };
            }
        }
        if ss_p.is_some() {
            let mut b = st.2 & (1 << i) != 0;
            if ui.put(Rect::from_min_size(at(332.0, y + 2.0), Vec2::new(135.0, 16.0)), egui::Checkbox::new(&mut b, "Super Simple Mode")).changed() {
                st.2 = if b { st.2 | (1 << i) } else { st.2 & !(1 << i) };
            }
        }
        p.text(at(473.0, y), egui::Align2::LEFT_TOP, ranges[i], f.clone(), Color32::WHITE);
    }
    let save = Rect::from_min_size(at(113.0, ty + 153.0), Vec2::new(104.0, 41.0));
    if ui.put(save, |ui: &mut egui::Ui| theme::button_sized(ui, "Save Modes", save.size())).clicked() {
        if app.readonly {
            app.toasts.error("Readonly mode: parameters not written");
        } else {
            for i in 0..6 {
                if st.0[i] != current[i] {
                    link.send(LinkCommand::SetParam { name: format!("{prefix}{}", i + 1), value: st.0[i] as f32 });
                }
            }
            if simple_p.is_some_and(|v| v != st.1) {
                link.send(LinkCommand::SetParam { name: "SIMPLE".into(), value: st.1 as f32 });
            }
            if ss_p.is_some_and(|v| v != st.2) {
                link.send(LinkCommand::SetParam { name: "SUPER_SIMPLE".into(), value: st.2 as f32 });
            }
            ui.ctx().data_mut(|d| d.remove::<([u32; 6], u32, u32)>(state_id));
            return;
        }
    }
    ui.put(
        Rect::from_min_size(at(333.0, ty + 150.0), Vec2::new(140.0, 24.0)),
        egui::Hyperlink::from_label_and_url("Simple and Super Simple description", "https://ardupilot.org/copter/docs/simpleandsuper-simple-modes.html"),
    );
    ui.ctx().data_mut(|d| d.insert_temp(state_id, st));
    ui.allocate_rect(Rect::from_min_size(o, Vec2::new(593.0, 250.0)), Sense::hover());
}

// ------------------------------------------------------------------ ConfigPlanner

fn planner(ui: &mut egui::Ui, app: &mut AppCtx, _fw: Firmware) {
    // ConfigPlanner.resx, laid out absolutely
    egui::ScrollArea::both().auto_shrink([false, false]).show(ui, |ui| {
        let (area, _) = ui.allocate_exact_size(Vec2::new(949.0, 693.0), Sense::hover());
        let o = area.min;
        let p = ui.painter().clone();
        let f = fonts::ui_id(fonts::UI_SIZE);
        let lab = |x: f32, y: f32, t: &str| {
            p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, t, f.clone(), Color32::WHITE);
        };
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let pl = &mut app.settings.planner;

        for (x, y, t) in [
            (9.0, 11.0, "Video Device"),
            (9.0, 38.0, "Video Format"),
            (9.0, 65.0, "OSD Color"),
            (9.0, 90.0, "Speech"),
            (9.0, 142.0, "UI Language"),
            (9.0, 171.0, "Joystick"),
            (9.0, 198.0, "Dist Units"),
            (249.0, 198.0, "Alt Units"),
            (9.0, 225.0, "Speed Units"),
            (9.0, 254.0, "Telemetry Rates"),
            (104.0, 254.0, "Attitude"),
            (220.0, 254.0, "Position"),
            (330.0, 254.0, "Mode/Status"),
            (461.0, 254.0, "RC"),
            (535.0, 254.0, "Sensor"),
            (9.0, 278.0, "Connect Reset"),
            (9.0, 301.0, "Track Length"),
            (180.0, 303.0, "Dist to Home"),
            (9.0, 327.0, "Waypoints"),
            (9.0, 350.0, "HUD"),
            (9.0, 373.0, "Map Follow"),
            (9.0, 398.0, "Log Path"),
            (9.0, 424.0, "Theme"),
            (9.0, 451.0, "Layout"),
            (9.0, 477.0, "GCS ID"),
            (9.0, 502.0, "Aircraft Icon"),
            (841.0, 502.0, "Line Length"),
            (9.0, 527.0, "Inactive Aircraft"),
            (9.0, 656.0, "Map Access Mode"),
        ] {
            lab(x, y, t);
        }
        lab(252.0, 108.0, "NOTE: Set the low level of SEVERITY to speak");
        lab(480.0, 198.0, "NOTE: The Configuration Tab will NOT display these units,\nas those are raw values.");

        let combo = |ui: &mut egui::Ui, pl: &mut BTreeMap<String, String>, key: &str, rr: Rect, opts: &[&str], def: &str| {
            let cur = pl.get(key).cloned().unwrap_or_else(|| def.to_string());
            ui.put(rr, |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt(("pl", key))
                    .width(rr.width() - 8.0)
                    .wrap_mode(egui::TextWrapMode::Truncate)
                    .selected_text(cur.clone())
                    .show_ui(ui, |ui| {
                        for o in opts {
                            if ui.selectable_label(cur == *o, *o).clicked() {
                                pl.insert(key.to_string(), o.to_string());
                            }
                        }
                    })
                    .response
            });
        };
        let prompt_id = egui::Id::new("speech_prompt");
        let check = |ui: &mut egui::Ui, pl: &mut BTreeMap<String, String>, key: &str, rr: Rect, text: &str, def: bool| {
            let mut b = pl.get(key).map(|v| v == "True").unwrap_or(def);
            if ui.put(rr, egui::Checkbox::new(&mut b, text)).changed() {
                pl.insert(key.to_string(), if b { "True".into() } else { "False".into() });
                // ConfigPlanner: ticking a speech option asks what to say
                let ask: &[&str] = match key {
                    "speechwaypointenabled" => &["speechwaypoint"],
                    "speechmodeenabled" => &["speechmode"],
                    "speechcustomenabled" => &["speechcustom"],
                    "speechbatteryenabled" => &["speechbattery", "speechbatteryvolt", "speechbatterypercent"],
                    "speechaltenabled" => &["speechalt", "speechaltheight"],
                    "speecharmenabled" => &["speecharm", "speechdisarm"],
                    "speechlowspeedenabled" => &["speechlowairspeed", "speechlowairspeedtrigger", "speechlowgroundspeed", "speechlowgroundspeedtrigger"],
                    _ => &[],
                };
                if b && !ask.is_empty() {
                    let fields: Vec<(String, String)> = ask.iter().map(|k| (k.to_string(), pl.get(*k).cloned().unwrap_or_else(|| crate::speech::default_text(k).to_string()))).collect();
                    ui.ctx().data_mut(|d| d.insert_temp(prompt_id, fields));
                }
            }
        };
        combo(ui, pl, "video_device", r(107.0, 8.0, 245.0, 21.0), &["None"], "None");
        combo(ui, pl, "video_format", r(107.0, 35.0, 408.0, 21.0), &["Default"], "Default");
        for (x, t) in [(358.0, "Start"), (439.0, "Stop")] {
            let br = r(x, 6.0, 75.0, 23.0);
            if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, t, br.size())).clicked() {
                app.actions.push(crate::app::AppAction::Video(t == "Start"));
            }
        }
        let pl = &mut app.settings.planner;
        check(ui, pl, "CHK_hudshow", r(520.0, 10.0, 140.0, 18.0), "Enable HUD Overlay", true);
        combo(ui, pl, "hudcolor", r(107.0, 62.0, 138.0, 21.0), &["White", "Red", "Green", "Blue", "Yellow", "Orange", "LightGray"], "White");
        check(ui, pl, "speechenable", r(107.0, 89.0, 105.0, 17.0), "Enable Speech", false);
        check(ui, pl, "speech_armed_only", r(212.0, 89.0, 115.0, 17.0), "Only when Armed", false);
        check(ui, pl, "speechwaypointenabled", r(332.0, 89.0, 75.0, 17.0), "Waypoint", false);
        check(ui, pl, "speechmodeenabled", r(408.0, 89.0, 60.0, 17.0), "Mode", false);
        check(ui, pl, "speechcustomenabled", r(469.0, 89.0, 85.0, 17.0), "30s Interval", false);
        check(ui, pl, "speechbatteryenabled", r(559.0, 89.0, 110.0, 17.0), "Battery Warning", false);
        check(ui, pl, "speechaltenabled", r(671.0, 89.0, 85.0, 17.0), "Alt Warning", false);
        check(ui, pl, "speecharmenabled", r(760.0, 89.0, 85.0, 17.0), "Arm/Disarm", false);
        check(ui, pl, "speechlowspeedenabled", r(853.0, 89.0, 85.0, 17.0), "Low Speed", false);
        combo(ui, pl, "speech_severity", r(107.0, 111.0, 138.0, 21.0), &["EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG"], "CRITICAL");
        combo(ui, pl, "language", r(107.0, 139.0, 138.0, 21.0), &["English (en-US)"], "English (en-US)");
        let jr = r(107.0, 166.0, 99.0, 23.0);
        if ui.put(jr, |ui: &mut egui::Ui| theme::button_sized(ui, "Joystick Setup", jr.size())).clicked() {
            app.joystick.window_open = true;
        }
        let pl = &mut app.settings.planner;
        combo(ui, pl, "distunits", r(107.0, 195.0, 138.0, 21.0), &["Meters", "Feet"], "Meters");
        combo(ui, pl, "altunits", r(320.0, 195.0, 138.0, 21.0), &["Meters", "Feet"], "Meters");
        combo(ui, pl, "speedunits", r(107.0, 222.0, 138.0, 21.0), &["meters_per_second", "fps", "kph", "mph", "knots"], "meters_per_second");
        for (key, x, def) in [("rateattitude", 166.0, "4"), ("rateposition", 282.0, "2"), ("ratestatus", 415.0, "2"), ("raterc", 489.0, "2"), ("ratesensors", 584.0, "2")] {
            combo(ui, pl, key, r(x, 251.0, 40.0, 21.0), &["0", "1", "2", "3", "4", "5", "6", "8", "10", "20", "25", "50"], def);
        }
        check(ui, pl, "CHK_resetapmonconnect", r(107.0, 277.0, 220.0, 17.0), "Reset on USB Connect (toggle DTR)", true);
        check(ui, pl, "CHK_rtsresetesp32", r(320.0, 277.0, 255.0, 19.0), "Disable RTS reset on ESP32 SerialUSB", false);
        let mut tl: f32 = pl.get("NUM_tracklength").and_then(|v| v.parse().ok()).unwrap_or(200.0);
        if ui.put(r(107.0, 300.0, 67.0, 20.0), egui::DragValue::new(&mut tl).range(10.0..=100000.0)).changed() {
            pl.insert("NUM_tracklength".into(), tl.to_string());
        }
        check(ui, pl, "CHK_disttohomeflightdata", r(267.0, 302.0, 135.0, 17.0), "Display in Flightdata", false);
        check(ui, pl, "loadwpsonconnect", r(107.0, 326.0, 180.0, 17.0), "Load Waypoints on connect?", false);
        check(ui, pl, "CHK_GDIPlus", r(107.0, 349.0, 225.0, 17.0), "GDI+ (old type/no HW acceleration)", false);
        check(ui, pl, "CHK_maprotation", r(107.0, 372.0, 210.0, 17.0), "Map is rotated to follow the plane", false);
        let mut logdir = pl.get("logdirectory").cloned().unwrap_or_else(|| platform::data_dir().join("logs").display().to_string());
        if ui.put(r(107.0, 395.0, 386.0, 20.0), egui::TextEdit::singleline(&mut logdir)).changed() {
            pl.insert("logdirectory".into(), logdir.clone());
        }
        let br = r(496.0, 393.0, 75.0, 23.0);
        if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, "Browse", br.size())).clicked() {
            #[cfg(not(any(target_os = "android", target_os = "ios")))]
            if let Some(d) = rfd::FileDialog::new().pick_folder() {
                pl.insert("logdirectory".into(), d.display().to_string());
            }
        }
        combo(ui, pl, "theme", r(107.0, 421.0, 138.0, 21.0), &["BurntKermit", "HighContrast", "Custom"], "BurntKermit");
        let tr = r(249.0, 421.0, 75.0, 20.0);
        if ui.put(tr, |ui: &mut egui::Ui| theme::button_sized(ui, "Custom", tr.size())).clicked() {
            // ThemeManager.CustomColor: edit the current palette, saved as custom.mpsystheme
            let cur = app.settings.planner.get("theme").cloned().unwrap_or_else(|| "BurntKermit".into());
            ui.ctx().data_mut(|d| d.insert_temp(egui::Id::new("theme_edit"), theme::palette_for(&cur)));
        }
        // Custom theme editor window
        let edit_id = egui::Id::new("theme_edit");
        if let Some(mut pal) = ui.ctx().data(|d| d.get_temp::<theme::Palette>(edit_id)) {
            let mut done = None;
            egui::Window::new("Custom Theme").collapsible(false).resizable(false).show(ui.ctx(), |ui| {
                egui::Grid::new("theme_colors").num_columns(2).show(ui, |ui| {
                    for (label, var) in theme::Palette::ITEMS {
                        theme::label(ui, label);
                        if let Some(c) = pal.get_mut(var) {
                            let mut rgb = [c.r(), c.g(), c.b()];
                            if ui.color_edit_button_srgb(&mut rgb).changed() {
                                *c = Color32::from_rgba_unmultiplied(rgb[0], rgb[1], rgb[2], c.a());
                            }
                        }
                        ui.end_row();
                    }
                });
                ui.horizontal(|ui| {
                    if theme::button(ui, "Save").clicked() {
                        done = Some(true);
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        done = Some(false);
                    }
                });
            });
            match done {
                Some(save) => {
                    if save {
                        let _ = std::fs::write(platform::data_dir().join("custom.mpsystheme"), pal.to_mpsystheme("Custom"));
                        let rev = app.settings.planner.get("theme_rev").and_then(|v| v.parse::<u32>().ok()).unwrap_or(0) + 1;
                        app.settings.planner.insert("theme".into(), "Custom".into());
                        app.settings.planner.insert("theme_rev".into(), rev.to_string());
                    }
                    ui.ctx().data_mut(|d| d.remove::<theme::Palette>(edit_id));
                }
                None => {
                    ui.ctx().data_mut(|d| d.insert_temp(edit_id, pal));
                }
            }
        }
        let pl = &mut app.settings.planner;
        combo(ui, pl, "displayview", r(107.0, 448.0, 138.0, 21.0), &["Basic", "Advanced", "Custom"], "Advanced");
        let mut gcsid: f32 = pl.get("gcsid").and_then(|v| v.parse().ok()).unwrap_or(255.0);
        if ui.put(r(107.0, 475.0, 53.0, 20.0), egui::DragValue::new(&mut gcsid).range(1.0..=255.0)).changed() {
            pl.insert("gcsid".into(), gcsid.to_string());
        }
        check(ui, pl, "GMapMarkerBase_DisplayCOG", r(107.0, 501.0, 90.0, 17.0), "Display COG", true);
        check(ui, pl, "GMapMarkerBase_DisplayHeading", r(199.0, 501.0, 108.0, 17.0), "Display Heading", true);
        check(ui, pl, "GMapMarkerBase_DisplayNavBearing", r(308.0, 501.0, 126.0, 17.0), "Display Nav Bearing", true);
        check(ui, pl, "GMapMarkerBase_DisplayRadius", r(436.0, 501.0, 125.0, 17.0), "Display Turn Radius", true);
        check(ui, pl, "GMapMarkerBase_DisplayTarget", r(563.0, 501.0, 98.0, 17.0), "Display Target", true);
        check(ui, pl, "GMapMarkerBase_DisplayToolTip", r(663.0, 501.0, 103.0, 17.0), "Display ToolTip", false);
        let mut ll: f32 = pl.get("GMapMarkerBase_length").and_then(|v| v.parse().ok()).unwrap_or(500.0);
        if ui.put(r(768.0, 500.0, 67.0, 20.0), egui::DragValue::new(&mut ll).range(0.0..=10000.0)).changed() {
            pl.insert("GMapMarkerBase_length".into(), ll.to_string());
        }
        combo(ui, pl, "GMapMarkerBase_InactiveDisplayStyle", r(107.0, 524.0, 138.0, 21.0), &["Normal", "Transparent", "Hidden"], "Normal");
        let vr = r(107.0, 551.0, 99.0, 20.0);
        if ui.put(vr, |ui: &mut egui::Ui| theme::button_sized(ui, "Start/Stop Vario", vr.size())).clicked() {
            let on = app.settings.planner.get("vario").map(|v| v == "True").unwrap_or(false);
            app.settings.planner.insert("vario".into(), if on { "False".into() } else { "True".into() });
        }
        // "What do you want it to say?" prompt for the speech options
        let pending: Option<Vec<(String, String)>> = ui.ctx().data(|d| d.get_temp(prompt_id));
        if let Some(mut fields) = pending {
            let mut done = None;
            egui::Window::new("Notification").collapsible(false).resizable(false).show(ui.ctx(), |ui| {
                for (k, v) in fields.iter_mut() {
                    let label = if k.ends_with("volt") || k.ends_with("percent") || k.ends_with("height") || k.ends_with("trigger") { "Level to warn at?" } else { "What do you want it to say?" };
                    theme::label(ui, format!("{label} ({k})"));
                    ui.add(egui::TextEdit::singleline(v).desired_width(320.0));
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
            match done {
                Some(ok) => {
                    if ok {
                        for (k, v) in fields {
                            app.settings.planner.insert(k, v);
                        }
                    }
                    ui.ctx().data_mut(|d| d.remove::<Vec<(String, String)>>(prompt_id));
                }
                None => {
                    ui.ctx().data_mut(|d| d.insert_temp(prompt_id, fields));
                }
            }
        }
        let pl = &mut app.settings.planner;
        check(ui, pl, "password_protect", r(340.0, 554.0, 150.0, 17.0), "Password Protect Config", false);
        check(ui, pl, "showairports", r(496.0, 554.0, 95.0, 17.0), "Show Airports", false);
        check(ui, pl, "enableadsb", r(598.0, 554.0, 60.0, 17.0), "ADSB", false);
        check(ui, pl, "ShowNoFly", r(671.0, 554.0, 60.0, 17.0), "No Fly", false);
        check(ui, pl, "analyticsoptout", r(107.0, 577.0, 120.0, 17.0), "OptOut Anon Stats", false);
        check(ui, pl, "beta_updates", r(228.0, 577.0, 95.0, 17.0), "Beta Updates", false);
        check(ui, pl, "norcreceiver", r(340.0, 577.0, 110.0, 17.0), "No RC Receiver", false);
        check(ui, pl, "showtfr", r(496.0, 577.0, 60.0, 17.0), "TFR's", false);
        check(ui, pl, "autoParamCommit", r(598.0, 578.0, 130.0, 17.0), "Auto Commit Params", true);
        check(ui, pl, "Params_BG", r(107.0, 600.0, 190.0, 17.0), "Params Download in BackGround", false);
        check(ui, pl, "SlowMachine", r(340.0, 600.0, 160.0, 17.0), "Runing on a slow computer", false);
        check(ui, pl, "mavdebug", r(106.0, 623.0, 160.0, 17.0), "Mavlink Message Debug", false);
        check(ui, pl, "chk_temp", r(340.0, 623.0, 150.0, 17.0), "Testing Screen", false);
        combo(ui, pl, "mapCache", r(107.0, 653.0, 138.0, 21.0), &["ServerAndCache", "ServerOnly", "CacheOnly"], "ServerAndCache");
        let mr = r(251.0, 653.0, 94.0, 21.0);
        if ui.put(mr, |ui: &mut egui::Ui| theme::button_sized(ui, "Open Map Cache", mr.size())).clicked() {
            let dir = platform::tile_cache_dir();
            let _ = std::fs::create_dir_all(&dir);
            ui.ctx().open_url(egui::OpenUrl::new_tab(format!("file:///{}", dir.display())));
        }
    });
}
