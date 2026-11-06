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
    /// ConfigParamLoading while parameters are still arriving
    Loading,
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
            Page::Loading => "Loading",
            Page::Planner => "Planner",
        }
    }
}

/// What SoftwareConfig_Load looks at on the connected vehicle.
#[derive(Clone, Copy, Default)]
struct Vehicle {
    firmware: Firmware,
    all_params: bool,
    osd: bool,
    ftp: bool,
}

fn pages(app: &AppCtx) -> Vec<Page> {
    let vehicle = app.link.filter(|_| app.connected()).map(|l| {
        let s = l.shared.read();
        Vehicle {
            firmware: s.cs.firmware,
            all_params: s.params_complete,
            osd: s.params.keys().any(|k| k.starts_with("OSD")),
            // MAV_PROTOCOL_CAPABILITY_FTP
            ftp: s.autopilot_version.map(|(_, caps)| caps & 32 != 0).unwrap_or(false),
        }
    });
    page_list(&app.settings.planner, vehicle)
}

/// SoftwareConfig_Load: vehicle pages once connected with all params, each behind its
/// DisplayView flag; Full Parameter List, Loading and Planner by connection state.
fn page_list(pl: &std::collections::BTreeMap<String, String>, vehicle: Option<Vehicle>) -> Vec<Page> {
    let shown = |flag: &str| crate::display::flag(pl, flag);
    let mut v = Vec::new();
    let connected = vehicle.is_some();
    let all = vehicle.is_some_and(|x| x.all_params);
    if let Some(x) = vehicle.filter(|x| x.all_params) {
        if x.firmware == Firmware::ArduCopter2 && shown("displayGeoFence") {
            v.push(Page::GeoFence);
        }
        match x.firmware {
            Firmware::ArduCopter2 | Firmware::ArduPlane => {
                if shown("displayBasicTuning") {
                    v.push(Page::BasicTuning);
                }
                if shown("displayExtendedTuning") {
                    v.push(Page::ExtendedTuning);
                }
            }
            // Rover Basic and Tracker Extended are added without a flag
            Firmware::ArduRover => v.push(Page::BasicTuning),
            Firmware::ArduTracker => v.push(Page::ExtendedTuning),
            _ => {}
        }
        if shown("displayStandardParams") {
            v.push(Page::StandardParams);
        }
        if shown("displayAdvancedParams") {
            v.push(Page::AdvancedParams);
        }
        if x.osd && shown("displayOSD") {
            v.push(Page::OnboardOsd);
        }
        if shown("displayMavFTP") && x.ftp {
            v.push(Page::MavFtp);
        }
        if shown("displayUserParam") {
            v.push(Page::UserParams);
        }
    }
    if shown("displayFullParamList") && (!connected || all) {
        v.push(Page::FullParamList);
    }
    if connected && !all {
        v.push(Page::Loading);
    }
    if shown("displayPlannerSettings") {
        v.push(Page::Planner);
    }
    v
}

#[cfg(test)]
mod page_tests {
    use super::*;
    use crate::display::{select, DisplayView, Layout};
    use std::collections::BTreeMap;

    #[test]
    fn config_pages_follow_connection_vehicle_and_display_view() {
        let mut pl = BTreeMap::new();
        assert_eq!(page_list(&pl, None), [Page::FullParamList, Page::Planner]);
        let loading = Vehicle { firmware: Firmware::ArduCopter2, ..Default::default() };
        assert_eq!(page_list(&pl, Some(loading)), [Page::Loading, Page::Planner], "no Full Parameter List while loading");
        let copter = Vehicle { firmware: Firmware::ArduCopter2, all_params: true, osd: true, ftp: true };
        assert_eq!(page_list(&pl, Some(copter)), [Page::GeoFence, Page::BasicTuning, Page::ExtendedTuning, Page::OnboardOsd, Page::MavFtp, Page::UserParams, Page::FullParamList, Page::Planner]);
        let rover = Vehicle { firmware: Firmware::ArduRover, all_params: true, ..Default::default() };
        let tracker = Vehicle { firmware: Firmware::ArduTracker, all_params: true, ..Default::default() };
        // a custom view without tuning or user params
        let mut custom = DisplayView::advanced();
        for f in ["displayBasicTuning", "displayExtendedTuning", "displayUserParam", "displayGeoFence", "displayPlannerSettings"] {
            custom.flags.insert(f.into(), false);
        }
        custom.flags.insert("displayStandardParams".into(), true);
        pl.insert("displayview".into(), custom.to_json());
        assert_eq!(page_list(&pl, Some(copter)), [Page::StandardParams, Page::OnboardOsd, Page::MavFtp, Page::FullParamList]);
        assert_eq!(page_list(&pl, Some(rover)), [Page::BasicTuning, Page::StandardParams, Page::FullParamList], "Rover tuning has no flag");
        assert_eq!(page_list(&pl, Some(tracker)), [Page::ExtendedTuning, Page::StandardParams, Page::FullParamList]);
        select(&mut pl, Layout::Basic);
        assert_eq!(page_list(&pl, Some(rover)), [Page::BasicTuning, Page::UserParams, Page::FullParamList, Page::Planner]);
    }
}

pub struct ConfigView {
    page: Page,
    loading: crate::views::setup_pages::ExtraPages,
    pub params: ParamEditor,
    extended: crate::views::tuning::ExtendedTuning,
    user_add: String,
    ftp: crate::views::mavftp::MavFtpUi,
    osd: crate::views::osd_editor::OsdEditor,
    standard: crate::views::friendly::FriendlyParams,
    advanced: crate::views::friendly::FriendlyParams,
    last_page: Option<Page>,
    vehicle_tuning: crate::views::vehicle_tuning::VehicleTuning,
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
            _ => Page::BasicTuning,
        };
        Self { page, loading: Default::default(), params: ParamEditor::default(), extended: Default::default(), user_add: String::new(), ftp: Default::default(), osd: Default::default(), standard: Default::default(), advanced: Default::default(), last_page: None, vehicle_tuning: Default::default() }
    }
}

impl ConfigView {
    pub fn leave_vehicle_tuning(&mut self) { self.vehicle_tuning.leave(); }
    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        self.params.on_event(e, toasts);
        self.vehicle_tuning.on_event(e, toasts);
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let fw = app.link.map(|l| l.shared.read().cs.firmware).unwrap_or_default();
        let list = pages(app);
        let default_page = if app.link.is_none() {
            Page::Planner
        } else if fw == Firmware::ArduTracker {
            Page::ExtendedTuning
        } else if list.contains(&Page::BasicTuning) {
            Page::BasicTuning
        } else {
            list.first().copied().unwrap_or(Page::Planner)
        };
        let shown = if list.contains(&self.page) { self.page } else { default_page };
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
        if self.last_page != Some(page) { self.vehicle_tuning.leave(); }
        self.last_page = Some(page);
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg()).inner_margin(6)).show(ui, |ui| match page {
            Page::GeoFence => crate::views::tuning::geofence(ui, app),
            Page::BasicTuning => match super::vehicle_tuning::form_for(fw, false) { Some(form) => self.vehicle_tuning.ui(ui, app, form), None => crate::views::tuning::basic_tuning(ui, app) },
            Page::ExtendedTuning => match super::vehicle_tuning::form_for(fw, true) { Some(form) => self.vehicle_tuning.ui(ui, app, form), None => self.extended.ui(ui, app) },
            Page::StandardParams => self.standard.ui(ui, app, "Standard"),
            Page::AdvancedParams => self.advanced.ui(ui, app, "Advanced"),
            Page::OnboardOsd => self.osd.ui(ui, app),
            Page::MavFtp => self.ftp.ui(ui, app),
            Page::UserParams => self.user_params(ui, app),
            Page::FullParamList => self.params.ui(ui, app, false),
            Page::Loading => self.loading.param_loading(ui, app),
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
            // MavlinkNumericUpDown: range and increment from the metadata
            let inc = meta.and_then(|m| m.increment).filter(|i| *i > 0.0).unwrap_or(if val.fract() == 0.0 { 1.0 } else { 0.01 });
            let (lo, hi) = meta.and_then(|m| m.range).unwrap_or((f64::MIN, f64::MAX));
            let (lo, hi) = (lo.min(val as f64), hi.max(val as f64));
            let mut v = val as f64;
            let (r, _) = ui.allocate_exact_size(Vec2::new(width.min(100.0), 20.0), Sense::hover());
            if theme::numeric_updown(ui, r, &mut v, lo, hi, inc, theme::decimals_for(inc).max(theme::value_decimals(val))).changed() {
                link.send(LinkCommand::SetParam { name: name.into(), value: v as f32 });
            }
        }
    }
}

// ------------------------------------------------------------------ ConfigRawParams

#[cfg(test)]
mod parameter_file_tests {
    use super::*;
    use crate::platform::{FilePicker, picker_test_support as picker};
    #[test]
    fn explicit_parameter_selection_stages_load_and_compare_cancel_and_errors_preserve_state() {
        let root = picker::root();
        std::fs::write(root.join("a.param"), "ALT,5\nKEEP,2\nUNKNOWN,7\n").unwrap();
        std::fs::write(root.join("z.param"), "ALT,99\n").unwrap();
        std::fs::write(root.join("bad.txt"), "ALT,6\nKEEP,invalid\n").unwrap();
        let params = BTreeMap::from([
            ("ALT".into(), mp_core::link::Param { value: 1.0, ptype: 9, index: 0 }),
            ("KEEP".into(), mp_core::link::Param { value: 2.0, ptype: 9, index: 1 }),
        ]);
        let ctx = egui::Context::default();
        let mut editor = ParamEditor::default(); let mut toasts = Toasts::default();
        editor.file_target = Some((42,1));
        editor.file_request = Some((ParamFileAction::Load, FilePicker::in_directory("Parameters", root.clone(), &["param", "txt"], None)));
        { let mut draw = |ctx: &egui::Context| editor.file_ui(ctx, &params, (42,1), &mut toasts, true); picker::click(&ctx, &mut draw, "a.param"); picker::click(&ctx, &mut draw, "Open"); }
        assert_eq!(editor.edits, BTreeMap::from([("ALT".into(), 5.0)]));
        assert!(editor.modified_only && editor.pending.is_empty());
        assert_eq!(editor.missing_dialog, Some(vec!["UNKNOWN".to_string()]));
        assert!(editor.file_request.is_none() && editor.file_target.is_none());
        editor.file_target = Some((42,1));
        editor.file_request = Some((ParamFileAction::Compare, FilePicker::in_directory("Compare", root.clone(), &["param"], None)));
        { let mut draw = |ctx: &egui::Context| editor.file_ui(ctx, &params, (42,1), &mut toasts, true); picker::click(&ctx, &mut draw, "Cancel"); }
        assert!(editor.compare.is_none()); assert_eq!(editor.edits["ALT"], 5.0);
        editor.file_target = Some((42,1));
        editor.file_request = Some((ParamFileAction::Compare, FilePicker::in_directory("Compare", root.clone(), &["param"], None)));
        { let mut draw = |ctx: &egui::Context| editor.file_ui(ctx, &params, (42,1), &mut toasts, true); picker::click(&ctx, &mut draw, "z.param"); picker::click(&ctx, &mut draw, "Open"); }
        assert_eq!(editor.compare, Some(vec![("ALT".into(), 1.0, 99.0, true)]));
        assert_eq!(editor.edits["ALT"], 5.0); assert!(editor.pending.is_empty());
        editor.file_target = Some((42,1));
        editor.file_request = Some((ParamFileAction::Load, FilePicker::in_directory("Parameters", root.clone(), &["txt"], None)));
        { let mut draw = |ctx: &egui::Context| editor.file_ui(ctx, &params, (42,1), &mut toasts, true); picker::click(&ctx, &mut draw, "bad.txt"); picker::click(&ctx, &mut draw, "Open"); }
        assert_eq!(editor.edits["ALT"], 5.0); assert!(editor.file_target.is_none());
        assert!(toasts.0.last().unwrap().contains("bad.txt"));
        for name in ["a.param", "z.param", "bad.txt"] { std::fs::remove_file(root.join(name)).unwrap(); } std::fs::remove_dir(root).unwrap();
    }

    #[test]
    fn changing_vehicle_cancels_a_pending_parameter_file() {
        let root = picker::root(); let ctx = egui::Context::default();
        let mut editor = ParamEditor::default(); let mut toasts = Toasts::default();
        editor.file_target = Some((42,1));
        editor.file_request = Some((ParamFileAction::Load, FilePicker::in_directory("Parameters", root.clone(), &["param"], None)));
        picker::frame(&ctx, &mut |ctx| editor.file_ui(ctx, &BTreeMap::new(), (43,1), &mut toasts, true), Vec::new());
        assert!(editor.file_request.is_none() && editor.edits.is_empty());
        assert!(toasts.0.last().unwrap().contains("Vehicle changed"));
        std::fs::remove_dir(root).unwrap();
    }

    #[test]
    fn sort_enable_matches_csharp_sorter() {
        let mut keys = vec![
            "WPNAV_SPEED".to_string(),
            "BATT_ENABLE".to_string(),
            "ARMING_CHECK".to_string(),
            "ACRO_ENABLE".to_string(),
            "RTL_ALT".to_string(),
            "MOT_SPIN_ARM".to_string(),
        ];
        sort_enable(&mut keys);
        assert_eq!(keys, vec![
            "ACRO_ENABLE",
            "BATT_ENABLE",
            "ARMING_CHECK",
            "MOT_SPIN_ARM",
            "RTL_ALT",
            "WPNAV_SPEED",
        ]);
    }

    #[test]
    fn offline_parameter_editing_loads_file_edits_and_saves() {
        let root = picker::root();
        let file_path = root.join("offline.param");
        std::fs::write(&file_path, "ARMING_CHECK,1\nWPNAV_SPEED,500\nBATT_ENABLE,1\n").unwrap();
        let ctx = egui::Context::default();
        let mut editor = ParamEditor::default();
        let mut toasts = Toasts::default();
        editor.file_request = Some((ParamFileAction::Load, FilePicker::in_directory("Parameters", root.clone(), &["param"], None)));
        {
            let mut draw = |ctx: &egui::Context| editor.file_ui(ctx, &BTreeMap::new(), (0, 0), &mut toasts, false);
            picker::click(&ctx, &mut draw, "offline.param");
            picker::click(&ctx, &mut draw, "Open");
        }
        assert_eq!(editor.offline_params.len(), 3);
        assert_eq!(editor.offline_params["WPNAV_SPEED"].value, 500.0);
        assert!(editor.missing_dialog.is_none(), "Offline mode does not show missing params dialog");

        // Edit a param offline
        editor.edits.insert("WPNAV_SPEED".into(), 650.0);

        // Verify that saving includes the edited value
        let saved_text = paramfile::save(editor.offline_params.iter().map(|(k, p)| (k, editor.edits.get(k).copied().unwrap_or(p.value))));
        let reloaded = paramfile::load(&saved_text).unwrap();
        assert_eq!(reloaded["WPNAV_SPEED"], 650.0);
        assert_eq!(reloaded["ARMING_CHECK"], 1.0);

        // Test Write Params sorting logic
        let mut keys: Vec<String> = editor.edits.keys().cloned().collect();
        keys.push("BATT_ENABLE".into());
        sort_enable(&mut keys);
        assert_eq!(keys, vec!["BATT_ENABLE", "WPNAV_SPEED"]);

        std::fs::remove_file(file_path).unwrap();
        std::fs::remove_dir(root).unwrap();
    }

    #[test]
    fn write_confirm_dialog_flow_escape_cancels_and_modal_presents_sorted_changes() {
        let ctx = egui::Context::default();
        let mut editor = ParamEditor::default();
        let changes = vec![
            ("BATT_ENABLE".into(), 0.0, 1.0),
            ("WPNAV_SPEED".into(), 500.0, 650.0),
        ];
        editor.write_confirm = Some(changes.clone());

        let mut out = ctx.run_ui(egui::RawInput {
            events: vec![egui::Event::Key {
                key: egui::Key::Escape,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            }],
            ..Default::default()
        }, |ui| {
            editor.write_confirm_window(ui.ctx(), None, Firmware::ArduCopter2);
        });
        out.textures_delta.clear();

        // After Escape, the confirmation modal is dismissed and changes cleared
        assert!(editor.write_confirm.is_none());
        assert!(editor.pending.is_empty());
    }

    #[test]
    fn a_failed_write_keeps_the_edit_and_only_counts_for_its_batch() {
        let mut editor = ParamEditor::default();
        let mut toasts = Toasts::default();
        editor.edits.insert("WPNAV_SPEED".into(), 600.0);
        editor.edits.insert("ANGLE_MAX".into(), 4500.0);
        editor.start_batch(2);
        editor.pending.insert("WPNAV_SPEED".into());
        editor.pending.insert("ANGLE_MAX".into());
        editor.on_event(&LinkEvent::ParamSet { name: "WPNAV_SPEED".into(), ok: false }, &mut toasts);
        // the cell stays green (still in _changes) and is not "invalid" red
        assert!(editor.edits.contains_key("WPNAV_SPEED"));
        assert!(!editor.failed.contains("WPNAV_SPEED"));
        editor.on_event(&LinkEvent::ParamSet { name: "ANGLE_MAX".into(), ok: true }, &mut toasts);
        assert!(!editor.edits.contains_key("ANGLE_MAX"));
        assert_eq!(toasts.0, ["Set WPNAV_SPEED Failed", "Not all parameters successfully saved."]);
        // the next batch succeeds on its own
        let mut toasts = Toasts::default();
        editor.start_batch(1);
        editor.pending.insert("WPNAV_SPEED".into());
        editor.on_event(&LinkEvent::ParamSet { name: "WPNAV_SPEED".into(), ok: true }, &mut toasts);
        assert_eq!(toasts.0, ["1 parameters successfully saved."]);
    }

    #[test]
    fn finished_writes_show_the_original_message_boxes() {
        let mut editor = ParamEditor::default();
        let mut toasts = Toasts::default();
        editor.reboot_required_staged = true;
        editor.write_count = 2;
        editor.pending.insert("INS_ACC_ID".into());
        editor.pending.insert("ANGLE_MAX".into());
        editor.on_event(&LinkEvent::ParamSet { name: "INS_ACC_ID".into(), ok: true }, &mut toasts);
        assert!(toasts.0.is_empty(), "nothing until the last write");
        editor.on_event(&LinkEvent::ParamSet { name: "ANGLE_MAX".into(), ok: true }, &mut toasts);
        assert_eq!(toasts.0, ["2 parameters successfully saved.", "Reboot is required for some parameters to take effect."]);
        assert_eq!(toasts.1, ["Saved", "Reboot Required"]);
        assert!(!editor.reboot_required_staged);

        let mut toasts = Toasts::default();
        editor.write_count = 1;
        editor.pending.insert("X".into());
        editor.on_event(&LinkEvent::ParamSet { name: "X".into(), ok: false }, &mut toasts);
        assert_eq!(toasts.0, ["Set X Failed", "Not all parameters successfully saved."]);
        assert_eq!(toasts.1, ["", "Saved"]);
    }

    #[test]
    fn commit_params_reports_the_autopilots_answer() {
        let mut editor = ParamEditor::default();
        let mut toasts = Toasts::default();
        // an ACK nobody asked for is ignored
        editor.on_event(&LinkEvent::CommandAck { command: 245, result: "ACCEPTED".into(), ok: true }, &mut toasts);
        assert!(toasts.0.is_empty());
        editor.commit_pending = true;
        editor.on_event(&LinkEvent::CommandAck { command: 245, result: "ACCEPTED".into(), ok: true }, &mut toasts);
        assert_eq!(toasts.0, ["Parameters committed to non-volatile memory"]);
        editor.commit_pending = true;
        editor.on_event(&LinkEvent::CommandAck { command: 245, result: "Timeout".into(), ok: false }, &mut toasts);
        assert_eq!(toasts.0.last().unwrap(), "Invalid command");
        assert!(!editor.commit_pending);
    }

    #[test]
    fn every_value_change_runs_readonly_then_range() {
        let mut editor = ParamEditor::default();
        let mut toasts = Toasts::default();
        let ranged = mp_core::param_meta::ParamMeta { range: Some((0.0, 10.0)), ..Default::default() };
        let readonly = mp_core::param_meta::ParamMeta { read_only: true, ..Default::default() };
        editor.queue_value("RO", 3.0, 1.0, Some(&readonly), &mut toasts);
        assert_eq!(toasts.0, ["RO is marked as ReadOnly, and will not be changed"]);
        assert_eq!(toasts.1, ["ReadOnly"]);
        assert!(editor.edits.is_empty());
        editor.queue_value("R", 20.0, 1.0, Some(&ranged), &mut toasts);
        assert_eq!(editor.range_confirm, Some(("R".to_string(), 20.0)));
        assert!(editor.edits.is_empty(), "waits for Yes");
        editor.range_confirm = None;
        editor.queue_value("R", 5.0, 1.0, Some(&ranged), &mut toasts);
        assert_eq!(editor.edits.get("R"), Some(&5.0));
        // back to the vehicle's value clears the edit
        editor.queue_value("R", 1.0, 1.0, Some(&ranged), &mut toasts);
        assert!(editor.edits.is_empty());
    }

    #[test]
    fn build_tree_nests_shared_prefixes_like_buildtree() {
        let mut names: Vec<String> = ["Q_PLT_Y_RATE", "Q_PLT_Y_RATE_TC", "BATT_MONITOR", "BATT_CAPACITY", "BATT2_MONITOR", "ARMING_CHECK", "RC10_MIN", "RC1_MIN", "RC2_MIN"].iter().map(|s| s.to_string()).collect();
        names.sort_by(|a, b| natural_compare(a, b));
        // '_' sorts after digits, so BATT2 comes before BATT_
        assert_eq!(names, ["ARMING_CHECK", "BATT2_MONITOR", "BATT_CAPACITY", "BATT_MONITOR", "Q_PLT_Y_RATE", "Q_PLT_Y_RATE_TC", "RC1_MIN", "RC2_MIN", "RC10_MIN"]);
        let t = build_tree(&names);
        assert_eq!(t.text, "All");
        let top: Vec<&str> = t.children.iter().map(|c| c.text.as_str()).collect();
        assert_eq!(top, ["ARMING_CHECK", "BATT2_MONITOR", "BATT", "Q", "RC1_MIN", "RC2_MIN", "RC10_MIN"]);
        let batt = &t.children[2];
        let q = &t.children[3];
        assert_eq!(q.children[0].text, "Q_PLT");
        assert_eq!(q.children[0].children[0].text, "Q_PLT_Y");
        assert_eq!(q.children[0].children[0].children.iter().map(|c| c.text.as_str()).collect::<Vec<_>>(), ["Q_PLT_Y_RATE", "Q_PLT_Y_RATE_TC"]);
        assert_eq!(batt.children.iter().map(|c| c.text.as_str()).collect::<Vec<_>>(), ["BATT_CAPACITY", "BATT_MONITOR"]);
    }

    #[test]
    fn filter_list_matches_a_regex_against_every_cell() {
        let cells = |name: &str, desc: &str, fav: bool| RowCells { name: name.into(), value: "1".into(), default: "NaN".into(), units: String::new(), options: String::new(), desc: desc.into(), fav };
        let re = search_regex("batt*volt").unwrap();
        assert!(row_visible(&cells("BATT_VOLT_PIN", "", false), "", Some(&re)));
        assert!(row_visible(&cells("X", "Battery voltage pin", false), "", Some(&re)), "any cell, case-insensitive");
        assert!(!row_visible(&cells("ANGLE_MAX", "", false), "", Some(&re)));
        // the Fav cell's value is True/False
        assert!(row_visible(&cells("ANGLE_MAX", "", true), "", search_regex("true").as_ref()));
        // the tree prefix: the node itself or names under it
        assert!(row_visible(&cells("BATT_MONITOR", "", false), "BATT_", None));
        assert!(row_visible(&cells("BATT", "", false), "BATT_", None));
        assert!(!row_visible(&cells("BATT2_MONITOR", "", false), "BATT_", None));
        assert!(search_regex("(").is_none());
    }

    #[test]
    fn tooltips_wrap_like_addnewlinesfortooltip() {
        assert_eq!(tooltip_lines("short"), "short");
        let long = "Maximum lean angle in all flight modes except the ones that do not use attitude control";
        let wrapped = tooltip_lines(long);
        assert!(wrapped.contains('\n'));
        assert_eq!(wrapped.replace('\n', " "), long);
        assert!(wrapped.lines().all(|l| l.len() <= (long.len() as f64).sqrt() as usize * 2 + 12));
    }

    #[test]
    fn post_write_parameter_count_divergence_triggers_check_flag() {
        let mut editor = ParamEditor::default();
        let mut toasts = Toasts::default();

        editor.pending.insert("CAN_D1_PROTOCOL".into());
        assert!(!editor.check_param_count_divergence);

        // When write finishes, check flag is set to true to inspect count divergence
        editor.on_event(&LinkEvent::ParamSet { name: "CAN_D1_PROTOCOL".into(), ok: true }, &mut toasts);
        assert!(editor.check_param_count_divergence);
        assert!(editor.pending.is_empty());
    }

    #[test]
    fn cell_values_follow_cellvaluechanged() {
        // comma decimal separator, mXparser arithmetic
        assert_eq!(cell_value("ATC_RAT_RLL_P", "1,25"), Some(1.25));
        assert_eq!(cell_value("WPNAV_SPEED", "2*(100+50)"), Some(300.0));
        assert_eq!(cell_value("X", "-2^3 + 10 / 4"), Some(-5.5));
        assert_eq!(cell_value("X", " 7 "), Some(7.0));
        // RC/HS reverse: 0 means reversed (-1)
        assert_eq!(cell_value("RC1_REV", "0"), Some(-1.0));
        assert_eq!(cell_value("HS3_REV", "0"), Some(-1.0));
        assert_eq!(cell_value("SERVO1_REVERSED", "0"), Some(0.0));
        // not a number, or NaN/infinity: the cell turns red
        assert_eq!(cell_value("X", "not_a_number"), None);
        assert_eq!(cell_value("X", "1/0"), None);
        assert_eq!(cell_value("X", "(1+2"), None);
        assert_eq!(cell_value("X", ""), None);
    }

    #[test]
    fn disconnect_mid_write_stops_like_the_write_loop() {
        let mut editor = ParamEditor::default();
        let mut toasts = Toasts::default();
        editor.edits.insert("ATC_RAT_RLL_P".into(), 0.2);
        editor.pending.insert("ATC_RAT_RLL_P".to_string());
        editor.disconnected_mid_write(&mut toasts);
        assert!(editor.pending.is_empty());
        assert!(editor.edits.contains_key("ATC_RAT_RLL_P"), "still queued");
        assert_eq!(toasts.0, ["Your are not connected"]);
        assert_eq!(toasts.1, ["Error"]);
    }
}

/// Params_CellValueChanged: RCn_REV/HSn_REV "0" becomes "-1", ',' is a decimal point, and the
/// text is an mXparser expression; NaN and infinity are rejected (the cell turns red).
pub fn cell_value(name: &str, raw: &str) -> Option<f64> {
    let mut text = raw.trim().to_string();
    if name.ends_with("_REV") && (name.starts_with("RC") || name.starts_with("HS")) && text == "0" {
        text = "-1".into();
    }
    let text = text.replace(',', ".");
    let mut p = Expr { s: text.as_bytes(), i: 0 };
    let v = p.sum()?;
    p.ws();
    (p.i == p.s.len() && v.is_finite()).then_some(v)
}

/// The arithmetic subset of mXparser's Expression: + - * / % ^, unary signs, brackets, pi, e.
struct Expr<'a> {
    s: &'a [u8],
    i: usize,
}

impl Expr<'_> {
    fn ws(&mut self) {
        while self.s.get(self.i).is_some_and(|c| c.is_ascii_whitespace()) {
            self.i += 1;
        }
    }
    fn eat(&mut self, c: u8) -> bool {
        self.ws();
        if self.s.get(self.i) == Some(&c) {
            self.i += 1;
            true
        } else {
            false
        }
    }
    fn sum(&mut self) -> Option<f64> {
        let mut v = self.product()?;
        loop {
            if self.eat(b'+') {
                v += self.product()?;
            } else if self.eat(b'-') {
                v -= self.product()?;
            } else {
                return Some(v);
            }
        }
    }
    fn product(&mut self) -> Option<f64> {
        let mut v = self.unary()?;
        loop {
            if self.eat(b'*') {
                v *= self.unary()?;
            } else if self.eat(b'/') {
                v /= self.unary()?;
            } else if self.eat(b'%') {
                v %= self.unary()?;
            } else {
                return Some(v);
            }
        }
    }
    fn unary(&mut self) -> Option<f64> {
        if self.eat(b'-') {
            return Some(-self.unary()?);
        }
        if self.eat(b'+') {
            return self.unary();
        }
        self.power()
    }
    fn power(&mut self) -> Option<f64> {
        let base = self.atom()?;
        if self.eat(b'^') {
            // right associative, binds tighter than a leading minus (-2^2 = -4)
            return Some(base.powf(self.unary()?));
        }
        Some(base)
    }
    fn atom(&mut self) -> Option<f64> {
        if self.eat(b'(') {
            let v = self.sum()?;
            return self.eat(b')').then_some(v);
        }
        self.ws();
        let start = self.i;
        while self.s.get(self.i).is_some_and(|c| c.is_ascii_alphanumeric() || *c == b'.') {
            // exponent sign: 1e-3
            self.i += 1;
            if matches!(self.s.get(self.i - 1), Some(b'e' | b'E')) && self.s[start].is_ascii_digit() && matches!(self.s.get(self.i), Some(b'-' | b'+')) {
                self.i += 1;
            }
        }
        let tok = std::str::from_utf8(&self.s[start..self.i]).ok()?;
        match tok {
            "pi" => Some(std::f64::consts::PI),
            "e" => Some(std::f64::consts::E),
            _ if tok.starts_with(|c: char| c.is_ascii_digit() || c == '.') => tok.parse().ok(),
            _ => None,
        }
    }
}

/// ConfigRawParams.NaturalStringComparer.NaturalCompare: digit runs compare as numbers
/// (leading zeros skipped), other characters case-insensitively.
pub fn natural_compare(x: &str, y: &str) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    let (x, y): (Vec<char>, Vec<char>) = (x.chars().collect(), y.chars().collect());
    let (mut ix, mut iy) = (0, 0);
    loop {
        if ix == x.len() {
            return if iy == y.len() { Ordering::Equal } else { Ordering::Less };
        }
        if iy == y.len() {
            return Ordering::Greater;
        }
        let (cx, cy) = (x[ix], y[iy]);
        if cx.is_ascii_digit() && cy.is_ascii_digit() {
            while ix < x.len() && x[ix] == '0' {
                ix += 1;
            }
            while iy < y.len() && y[iy] == '0' {
                iy += 1;
            }
            let (mut ex, mut ey) = (ix, iy);
            while ex < x.len() && x[ex].is_ascii_digit() {
                ex += 1;
            }
            while ey < y.len() && y[ey].is_ascii_digit() {
                ey += 1;
            }
            if ex - ix != ey - iy {
                return (ex - ix).cmp(&(ey - iy));
            }
            while ix < ex {
                if x[ix] != y[iy] {
                    return x[ix].cmp(&y[iy]);
                }
                ix += 1;
                iy += 1;
            }
        } else {
            let o = cx.to_uppercase().cmp(cy.to_uppercase());
            if o != Ordering::Equal {
                return o;
            }
            ix += 1;
            iy += 1;
        }
    }
}

/// A treeView1 node: a shared name prefix (without its trailing '_'), or a parameter.
#[derive(Debug, Clone, PartialEq)]
pub struct TreeNode {
    pub text: String,
    pub children: Vec<TreeNode>,
}

fn leaf(text: &str) -> TreeNode {
    TreeNode { text: text.to_string(), children: Vec::new() }
}

/// ConfigRawParams.BuildTree over naturally sorted names: a branch for every prefix that this
/// parameter and the next one share.
pub fn build_tree(commands: &[String]) -> TreeNode {
    let mut stack = vec![leaf("All")];
    let mut prefix = String::new();
    let pop = |stack: &mut Vec<TreeNode>| {
        let n = stack.pop().unwrap();
        stack.last_mut().unwrap().children.push(n);
    };
    for (i, param) in commands.iter().enumerate() {
        // step up while the parameter is outside the current branch
        while !param.starts_with(&prefix) && stack.len() > 1 {
            let last = format!("{}_", stack.last().unwrap().text.split('_').last().unwrap_or(""));
            if prefix.ends_with(&last) {
                prefix.truncate(prefix.len() - last.len());
            }
            pop(&mut stack);
        }
        if i == commands.len() - 1 {
            stack.last_mut().unwrap().children.push(leaf(param));
            break;
        }
        let next = &commands[i + 1];
        let seg = |prefix: &str| format!("{}_", param[prefix.len()..].split('_').next().unwrap_or(""));
        let mut node = seg(&prefix);
        while node.len() > 1 && param.starts_with(&format!("{prefix}{node}")) && next.starts_with(&format!("{prefix}{node}")) {
            prefix += &node;
            stack.push(leaf(&prefix[..prefix.len() - 1]));
            node = seg(&prefix);
        }
        stack.last_mut().unwrap().children.push(leaf(param));
    }
    while stack.len() > 1 {
        pop(&mut stack);
    }
    stack.pop().unwrap()
}

/// treeView1: "All" expanded; selecting a node sets filterPrefix to its text + "_".
fn tree_ui(ui: &mut egui::Ui, node: &TreeNode, sel: &mut Option<String>, top: bool) {
    let selected = if top { sel.is_none() } else { sel.as_deref() == Some(node.text.as_str()) };
    let mut label = |ui: &mut egui::Ui| {
        if ui.selectable_label(selected, RichText::new(&node.text).color(Color32::WHITE)).clicked() {
            *sel = if top { None } else { Some(node.text.clone()) };
        }
    };
    if node.children.is_empty() {
        label(ui);
        return;
    }
    let id = ui.make_persistent_id(("raw_tree_node", &node.text));
    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, top)
        .show_header(ui, |ui| label(ui))
        .body(|ui| {
            for c in &node.children {
                tree_ui(ui, c, sel, false);
            }
        });
}

/// The text of a grid row's cells, as filterList reads them.
pub struct RowCells {
    pub name: String,
    pub value: String,
    pub default: String,
    pub units: String,
    pub options: String,
    pub desc: String,
    pub fav: bool,
}

fn row_cells(name: &str, value: f32, fw: Firmware, defaults: Option<&BTreeMap<String, f32>>, fav: bool) -> RowCells {
    let meta = param_meta::get(fw, name);
    RowCells {
        name: name.to_string(),
        value: paramfile::format_value(value),
        default: defaults.and_then(|d| d.get(name)).map(|d| paramfile::format_value(*d)).unwrap_or_else(|| "NaN".into()),
        units: meta.map(|m| m.units.clone()).unwrap_or_default(),
        options: meta.map(options_text).unwrap_or_default(),
        desc: meta.map(|m| m.description.clone()).unwrap_or_default(),
        fav,
    }
}

/// filterList's Regex: '*' becomes ".*", case-insensitive, '.' matching newlines. None when
/// the text is not a valid expression.
pub fn search_regex(text: &str) -> Option<regex::Regex> {
    let pattern = text.replace('*', ".*").replace("..*", ".*");
    regex::RegexBuilder::new(&pattern).case_insensitive(true).dot_matches_new_line(true).build().ok()
}

/// filterList for one row: inside the tree prefix, and some cell matching the search.
pub fn row_visible(cells: &RowCells, prefix: &str, re: Option<&regex::Regex>) -> bool {
    if cells.name != prefix.trim_end_matches('_') && !cells.name.starts_with(prefix) {
        return false;
    }
    let Some(re) = re else { return true };
    let fav = if cells.fav { "True" } else { "False" };
    [cells.name.as_str(), &cells.value, &cells.default, &cells.units, &cells.options, &cells.desc, fav].iter().any(|c| re.is_match(c))
}

/// The Options cell: the Range metadata, then the values one per line.
fn options_text(m: &param_meta::ParamMeta) -> String {
    let range = m.range.map(|(lo, hi)| format!("{lo} {hi}")).unwrap_or_default();
    let values = m.values.iter().map(|(k, s)| format!("{k}:{s}")).collect::<Vec<_>>().join("\n");
    format!("{range}\n{values}").trim().to_string()
}

/// Options.ToolTipText: one value per line, or rows of `columns` past 50 values.
fn options_tooltip(m: &param_meta::ParamMeta) -> Option<String> {
    if m.values.is_empty() {
        return None;
    }
    let opts: Vec<String> = m.values.iter().map(|(k, s)| format!("{k}:{s}")).collect();
    let n = opts.len() - 1; // the comma count
    if n <= 50 {
        return Some(opts.join("\n"));
    }
    let columns = (n - 1) / 50 + 1;
    let mut ans = String::new();
    let mut i = 0;
    loop {
        for _ in 0..columns {
            ans += &format!("{}, ", opts[i]);
            i += 1;
            if i >= n {
                break;
            }
        }
        if i >= n {
            break;
        }
        ans.push('\n');
    }
    Some(ans)
}

/// AddNewLinesForTooltip: text of 50 characters or more breaks at whitespace after about
/// 2*sqrt(length) characters per line.
pub fn tooltip_lines(text: &str) -> String {
    let chars: Vec<char> = text.chars().collect();
    if chars.len() < 50 {
        return text.to_string();
    }
    let line_length = (chars.len() as f64).sqrt() as usize * 2;
    let mut out = String::new();
    let mut pos = 0;
    let mut i = 0;
    while i < chars.len() {
        if pos >= line_length && chars[i].is_whitespace() {
            out.push('\n');
            pos = 0;
        }
        if pos == 0 {
            while i < chars.len() && chars[i].is_whitespace() {
                i += 1;
            }
        }
        if i < chars.len() {
            out.push(chars[i]);
        }
        pos += 1;
        i += 1;
    }
    out
}


/// Matches C# ListExtension.SortENABLE:
/// Orders parameters so those ending with "ENABLE" come first (alphabetically),
/// followed by all other parameters (alphabetically).
pub fn sort_enable(keys: &mut [String]) {
    keys.sort_by(|a, b| {
        let a_en = a.ends_with("ENABLE");
        let b_en = b.ends_with("ENABLE");
        match (a_en, b_en) {
            (true, true) => a.cmp(b),
            (true, false) => std::cmp::Ordering::Less,
            (false, true) => std::cmp::Ordering::Greater,
            (false, false) => a.cmp(b),
        }
    });
}

#[derive(Clone, Copy)]
enum ParamFileAction { Load, Compare }

#[derive(Default)]
pub struct ParamEditor {
    search: String,
    file_request: Option<(ParamFileAction, platform::FilePicker)>,
    file_target: Option<(u8, u8)>,
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
    /// Params_RowEnter: the current row, whose Options cell holds the editor control
    current: Option<String>,
    /// MavlinkCheckBoxBitMask shown by "Set Bitmask" (param name)
    bitmask_edit: Option<String>,
    missing_dialog: Option<Vec<String>>,
    /// Offline parameters loaded when not connected to a vehicle
    pub offline_params: BTreeMap<String, mp_core::link::Param>,
    /// Write parameters confirmation dialog: list of (name, old_value, new_value)
    pub write_confirm: Option<Vec<(String, f32, f32)>>,
    /// Set of parameter names whose write to vehicle failed
    pub failed: BTreeSet<String>,
    /// Set of parameter names whose write was confirmed during session
    pub confirmed: BTreeSet<String>,
    /// Staged flag indicating whether reboot is required after current save completes
    pub reboot_required_staged: bool,
    /// parameters in the write batch, for "{n} parameters successfully saved."
    pub write_count: usize,
    /// Commit Params sent PREFLIGHT_STORAGE and waits for its ACK
    pub commit_pending: bool,
    /// Whether to check if available parameter count diverged from vehicle reported count post-save
    pub check_param_count_divergence: bool,
    /// "Out of range" Yes/No for an edited value (name, value)
    range_confirm: Option<(String, f32)>,
    /// writes of the current batch that failed (BUT_writePIDS_Click's `error`)
    batch_errors: usize,
    /// txt_search after the 500 ms filter timer (and filterList's length rule)
    search_applied: String,
    search_changed: Option<std::time::Instant>,
    /// BuildTree's nodes, rebuilt when the parameter list changes
    tree: Option<(Vec<String>, TreeNode)>,
}

impl ParamEditor {
    fn file_ui(&mut self, ctx: &egui::Context, params: &BTreeMap<String, mp_core::link::Param>, target: (u8, u8), toasts: &mut Toasts, is_connected: bool) {
        if is_connected && self.file_request.is_some() && self.file_target != Some(target) {
            self.file_request = None; self.file_target = None;
            toasts.error("Vehicle changed while selecting a parameter file");
            return;
        }
        if let Some((action, path)) = platform::poll_file_action(&mut self.file_request, ctx) {
            self.file_target = None;
            let values = match paramfile::read(&path) { Ok(values) => values, Err(e) => { toasts.error(format!("{}: {e}", path.display())); return; } };
            match action {
                ParamFileAction::Load => {
                    if is_connected {
                        let mut missing = Vec::new();
                        for (name, value) in values {
                            if let Some(p) = params.get(&name) {
                                if p.value != value {
                                    self.edits.insert(name, value);
                                }
                            } else {
                                missing.push(name);
                            }
                        }
                        self.modified_only = true;
                        if !missing.is_empty() {
                            self.missing_dialog = Some(missing);
                        }
                    } else {
                        // Offline mode: populate offline_params from file
                        for (idx, (name, value)) in values.into_iter().enumerate() {
                            self.offline_params.insert(name, mp_core::link::Param {
                                value,
                                ptype: 9, // REAL32
                                index: idx as u16,
                            });
                        }
                        self.edits.clear();
                        self.modified_only = false;
                        toasts.info(format!("Loaded {} parameters offline.", self.offline_params.len()));
                    }
                }
                ParamFileAction::Compare => {
                    self.compare = Some(values.into_iter()
                        .filter_map(|(name, value)| params.get(&name).filter(|p| p.value != value).map(|p| (name, p.value, value, true))).collect());
                }
            }
        }
        if self.file_request.is_none() { self.file_target = None; }
    }

    pub fn on_event(&mut self, e: &LinkEvent, toasts: &mut Toasts) {
        if let LinkEvent::CommandAck { command: 245, ok, .. } = e {
            if std::mem::take(&mut self.commit_pending) {
                // BUT_commitToFlash_Click: doCommand throws on a refusal or timeout
                if *ok {
                    toasts.error("Parameters committed to non-volatile memory");
                } else {
                    toasts.error("Invalid command");
                }
            }
        }
        if let LinkEvent::ParamSet { name, ok } = e {
            if self.pending.remove(name) {
                if *ok {
                    self.edits.remove(name);
                    self.failed.remove(name);
                    self.confirmed.insert(name.clone());
                } else {
                    self.batch_errors += 1;
                    toasts.error(format!("Set {name} Failed"));
                }
                if self.pending.is_empty() {
                    if self.batch_errors > 0 {
                        toasts.message("Not all parameters successfully saved.", "Saved");
                    } else {
                        toasts.message(format!("{} parameters successfully saved.", self.write_count), "Saved");
                    }
                    if self.reboot_required_staged {
                        self.reboot_required_staged = false;
                        toasts.message("Reboot is required for some parameters to take effect.", "Reboot Required");
                    }
                    self.check_param_count_divergence = true;
                }
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, tree_page: bool) {
        let is_connected = app.link.is_some();
        let (params, fw, target) = if let Some(link) = app.link {
            let s = link.shared.read();
            (s.params.clone(), s.cs.firmware, (s.cs.sysid, s.cs.compid))
        } else {
            (self.offline_params.clone(), Firmware::ArduCopter2, (0, 0))
        };

        if !is_connected && !self.pending.is_empty() {
            // the link went away before PARAM_VALUE came back: the write never confirmed
            self.disconnected_mid_write(app.toasts);
        }

        if self.check_param_count_divergence {
            self.check_param_count_divergence = false;
            if let Some(link) = app.link {
                let s = link.shared.read();
                let received = s.params.len();
                let reported = s.param_count as usize;
                if reported > 0 && received != reported {
                    if s.cs.armed {
                        app.toasts.message("The number of available parameters changed, until full param refresh is done, some parameters will not be available.", "Params");
                    } else {
                        app.toasts.message("The number of available parameters changed. A full param refresh will be done to show all params.", "Params");
                        link.send(LinkCommand::RequestParams);
                    }
                }
            }
        }

        self.file_ui(ui.ctx(), &params, target, app.toasts, is_connected);

        if is_connected {
            let values: BTreeMap<String, f32> = params.iter().map(|(k, p)| (k.clone(), p.value)).collect();
            self.bitmask_window(ui.ctx(), fw, &values, app.toasts);
        }
        // MAVLinkInterface.getParamList: the packed list with defaults over MAVFtp
        if let Some(link) = app.link {
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
        }
        let defaults = if is_connected { self.defaults.lock().unwrap().clone() } else { None };

        // tableLayoutPanel1: Dock Right, 122px
        egui::Panel::right("raw_buttons").exact_size(122.0).resizable(false).frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
            let o = ui.max_rect().min;
            let r = |y: f32, h: f32| Rect::from_min_size(o + Vec2::new(6.0, y), Vec2::new(110.0, h));
            let b = |ui: &mut egui::Ui, rr: Rect, t: &str| ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, t, rr.size())).clicked();
            if b(ui, r(6.0, 19.0), "Load from file") {
                self.file_target = Some(target);
                self.file_request = Some((ParamFileAction::Load, platform::FilePicker::new("Load Parameters", &["param", "parm", "txt"], None)));
            }
            if b(ui, r(31.0, 19.0), "Save to file") {
                let text = paramfile::save(params.iter().map(|(k, p)| (k, self.edits.get(k).copied().unwrap_or(p.value))));
                if let Some(Err(e)) = platform::save_text("Save Parameters", "params.param", "Param List", &["param"], &text) {
                    app.toasts.error(e);
                }
            }
            ui.add_enabled_ui(is_connected && !app.readonly, |ui| {
                if b(ui, r(66.0, 19.0), "Write Params") {
                    if self.edits.is_empty() {
                        app.toasts.message("No parameters were changed.", "No changes");
                    } else {
                        let mut keys: Vec<String> = self.edits.keys().cloned().collect();
                        sort_enable(&mut keys);
                        let changes: Vec<(String, f32, f32)> = keys.into_iter().map(|k| {
                            let old = params.get(&k).map(|p| p.value).unwrap_or(0.0);
                            let new = self.edits[&k];
                            (k, old, new)
                        }).collect();
                        self.write_confirm = Some(changes);
                    }
                }
            });
            ui.add_enabled_ui(is_connected, |ui| {
                if b(ui, r(91.0, 19.0), "Refresh Params") {
                    self.edits.clear();
                    self.failed.clear();
                    if let Some(link) = app.link {
                        link.send(LinkCommand::RequestParams);
                    }
                }
            });
            if b(ui, r(116.0, 19.0), "Compare Params") {
                self.file_target = Some(target);
                self.file_request = Some((ParamFileAction::Compare, platform::FilePicker::new("Compare Parameters", &["param", "parm", "txt"], None)));
            }
            if crate::display::flag(&app.settings.planner, "displayParamCommitButton") {
                ui.add_enabled_ui(is_connected, |ui| {
                    if b(ui, r(141.0, 22.0), "Commit Params") {
                        if let Some(link) = app.link {
                            link.send(LinkCommand::CommandLong { command: 245, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                            // BUT_commitToFlash_Click's message box follows the ACK (on_event)
                            self.commit_pending = true;
                        }
                    }
                });
            }
            let f = fonts::ui_id(fonts::UI_SIZE);
            ui.painter().text(o + Vec2::new(6.0, 169.0), egui::Align2::LEFT_TOP, crate::i18n::t("All Units are in raw\nformat with no scaling"), f.clone(), Color32::WHITE);
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
            ui.add_enabled_ui(is_connected, |ui| {
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
            });
            ui.painter().text(o + Vec2::new(6.0, 286.0), egui::Align2::LEFT_TOP, crate::i18n::t("Search"), f.clone(), Color32::WHITE);
            ui.put(r(302.0, 20.0), egui::TextEdit::singleline(&mut self.search));
            ui.put(Rect::from_min_size(o + Vec2::new(6.0, 328.0), Vec2::new(90.0, 17.0)), egui::Checkbox::new(&mut self.modified_only, crate::i18n::t("Modified")));
            // chk_none_default is only shown when the vehicle reported defaults
            if defaults.is_some() {
                ui.put(Rect::from_min_size(o + Vec2::new(6.0, 351.0), Vec2::new(100.0, 17.0)), egui::Checkbox::new(&mut self.none_default, crate::i18n::t("None Default")));
            }
            b(ui, r(384.0, 22.0), "Refresh Table");
        });

        // splitContainer1.Panel1: treeView1 (180px) with the "<" collapse button
        let show_tree = tree_page || !self.tree_collapsed;
        // txt_search_TextChanged restarts a 500 ms timer; filterList ignores one character
        if self.search != self.search_applied && self.search_changed.is_none() {
            self.search_changed = Some(std::time::Instant::now());
        }
        if let Some(t) = self.search_changed {
            if t.elapsed() >= std::time::Duration::from_millis(500) {
                self.search_changed = None;
                if self.search.chars().count() != 1 {
                    self.search_applied = self.search.clone();
                }
            } else {
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(500) - t.elapsed());
            }
        }
        if show_tree {
            let mut names: Vec<String> = params.keys().cloned().collect();
            names.sort_by(|a, b| natural_compare(a, b));
            if self.tree.as_ref().is_none_or(|(n, _)| *n != names) {
                let t = build_tree(&names);
                self.tree = Some((names, t));
            }
            let tree = &self.tree.as_ref().unwrap().1;
            let mut sel = self.selected_group.clone();
            egui::Panel::left("raw_tree").resizable(true).default_size(180.0).frame(egui::Frame::new().fill(theme::control_bg())).show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    tree_ui(ui, tree, &mut sel, true);
                });
            });
            // treeView1_AfterSelect
            self.selected_group = sel;
        } else {
            // but_collapse_Click clears filterPrefix
            self.selected_group = None;
        }
        egui::Panel::left("raw_collapse").exact_size(18.0).resizable(false).frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
            let r = Rect::from_min_size(ui.max_rect().min, Vec2::new(18.0, 18.0));
            if ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, if show_tree { "<" } else { ">" }, r.size())).clicked() {
                self.tree_collapsed = !self.tree_collapsed;
            }
        });

        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
            if !is_connected && params.is_empty() {
                ui.vertical_centered(|ui| {
                    ui.add_space(40.0);
                    theme::label(ui, "No parameters loaded.\nClick 'Load from file' to edit parameters offline, or connect to a vehicle.");
                });
                return;
            }
            // filterList: the tree prefix and the search over every cell, then the Modified and
            // None Default boxes decide on their own
            let prefix = self.selected_group.as_ref().map(|t| format!("{t}_")).unwrap_or_default();
            let re = search_regex(&self.search_applied);
            let favs = app.settings.fav_params.clone();
            let mut rows: Vec<(String, f32)> = params
                .iter()
                .filter(|(k, p)| {
                    let shown = self.edits.get(*k).copied().unwrap_or(p.value);
                    let cells = row_cells(k, shown, fw, defaults.as_ref(), favs.contains(k));
                    let mut visible = (self.search_applied.is_empty() && prefix.is_empty()) || row_visible(&cells, &prefix, re.as_ref());
                    if self.modified_only {
                        visible = self.edits.contains_key(*k);
                    }
                    if self.none_default {
                        visible = cells.default != cells.value;
                    }
                    visible
                })
                .map(|(k, p)| (k.clone(), p.value))
                .collect();
            // OnParamsOnSortCompare: favourites first, then NaturalCompare
            rows.sort_by(|a, b| favs.contains(&b.0).cmp(&favs.contains(&a.0)).then_with(|| natural_compare(&a.0, &b.0)));
            self.grid(ui, &mut app.toasts, app.link, fw, &rows, &mut app.settings.fav_params, defaults.as_ref());
        });
        // presaved file arrived: show it in ParamCompare
        if let Some(r) = self.presaved.lock().unwrap().take() {
            match r.and_then(|text| paramfile::load(&text).map_err(|e| e.to_string())) {
                Ok(values) => {
                    let diffs: Vec<(String, f32, f32, bool)> = values
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
                    if let Some(link) = app.link {
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
        }
        self.compare_window(ui.ctx());
        self.missing_window(ui.ctx());
        self.write_confirm_window(ui.ctx(), app.link, fw);
        self.range_confirm_window(ui.ctx(), &params);
    }

    /// Params_CellValueChanged after parsing: ReadOnly refuses, an out-of-range value asks
    /// first, anything else is queued (and a value equal to the vehicle's drops the edit).
    fn queue_value(&mut self, name: &str, nv: f64, current: f32, meta: Option<&param_meta::ParamMeta>, toasts: &mut Toasts) {
        if meta.is_some_and(|m| m.read_only) {
            toasts.message(format!("{name}{}", crate::i18n::t(" is marked as ReadOnly, and will not be changed")), "ReadOnly");
            return;
        }
        if meta.and_then(|m| m.range).is_some_and(|(lo, hi)| nv < lo || nv > hi) {
            self.range_confirm = Some((name.to_string(), nv as f32));
            return;
        }
        self.failed.remove(name);
        if nv as f32 == current {
            self.edits.remove(name);
        } else {
            self.edits.insert(name.to_string(), nv as f32);
        }
    }

    fn start_batch(&mut self, n: usize) {
        self.write_count = n;
        self.batch_errors = 0;
    }

    /// The write loop's "Your are not connected": it stops, and the rest stay queued.
    fn disconnected_mid_write(&mut self, toasts: &mut Toasts) {
        self.pending.clear();
        self.reboot_required_staged = false;
        toasts.message("Your are not connected", "Error");
    }

    /// Params_CellValueChanged's "Out of range" box: No puts the old value back.
    fn range_confirm_window(&mut self, ctx: &egui::Context, params: &BTreeMap<String, mp_core::link::Param>) {
        let Some((name, value)) = self.range_confirm.clone() else { return };
        let mut answer = None;
        egui::Modal::new(egui::Id::new("param_out_of_range")).show(ctx, |ui| {
            ui.set_width(300.0);
            ui.label(RichText::new(crate::i18n::t("Out of range")).strong());
            theme::label(ui, &format!("{name}{}", crate::i18n::t(" value is out of range. Do you want to continue?")));
            ui.horizontal(|ui| {
                if theme::button(ui, "Yes").clicked() {
                    answer = Some(true);
                }
                if theme::button(ui, "No").clicked() {
                    answer = Some(false);
                }
            });
        });
        let Some(yes) = answer else { return };
        self.range_confirm = None;
        if yes {
            self.failed.remove(&name);
            if params.get(&name).is_some_and(|p| p.value == value) {
                self.edits.remove(&name);
            } else {
                self.edits.insert(name, value);
            }
        }
    }

    /// Confirmation dialog for parameter changes, matching C# ConfigRawParams BUT_writePIDS_Click
    fn write_confirm_window(&mut self, ctx: &egui::Context, link: Option<&Link>, fw: Firmware) {
        let Some(changes) = self.write_confirm.as_ref() else { return };
        let mut answer = None;
        let reboot_needed = changes.iter().any(|(name, _, _)| {
            param_meta::get(fw, name).map(|m| m.reboot_required).unwrap_or(false)
        });
        egui::Modal::new(egui::Id::new("confirm_write_params")).show(ctx, |ui| {
            ui.set_width(380.0);
            ui.label(RichText::new(crate::i18n::t("Confirm Parameter Changes")).strong());
            ui.add_space(6.0);
            let count = changes.len();
            if count <= 20 {
                ui.label(format!("You are about to change {count} parameters. Please review the changes below:"));
                ui.add_space(4.0);
                egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                    egui::Grid::new("confirm_write_grid").num_columns(1).striped(true).show(ui, |ui| {
                        for (name, old, new) in changes {
                            let is_reboot = param_meta::get(fw, name).map(|m| m.reboot_required).unwrap_or(false);
                            let tag = if is_reboot { " [Reboot Required]" } else { "" };
                            ui.label(RichText::new(format!("{name}: {} -> {}{tag}", paramfile::format_value(*old), paramfile::format_value(*new))).monospace());
                            ui.end_row();
                        }
                    });
                });
                if reboot_needed {
                    ui.add_space(6.0);
                    ui.colored_label(Color32::from_rgb(255, 180, 0), "⚠️ Note: One or more parameters require an autopilot reboot to take effect.");
                }
                ui.add_space(6.0);
                ui.label(crate::i18n::t("Do you want to proceed?"));
            } else {
                ui.label(format!("You are about to change {count} parameters. Are you sure you want to proceed?"));
                if reboot_needed {
                    ui.add_space(6.0);
                    ui.colored_label(Color32::from_rgb(255, 180, 0), "⚠️ Note: One or more parameters require an autopilot reboot to take effect.");
                }
            }
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                if theme::button(ui, "Yes").clicked() {
                    answer = Some(true);
                }
                if theme::button(ui, "No").clicked() {
                    answer = Some(false);
                }
            });
        });
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            answer = Some(false);
        }
        if let Some(yes) = answer {
            let changes = self.write_confirm.take().unwrap();
            if yes {
                if reboot_needed {
                    self.reboot_required_staged = true;
                }
                if let Some(link) = link {
                    self.start_batch(changes.len());
                    for (name, _, new) in changes {
                        link.send(LinkCommand::SetParam { name: name.clone(), value: new });
                        self.pending.insert(name);
                    }
                }
            }
        }
    }

    /// "No matching Params": dialog showing missing params that were not present on the vehicle
    fn missing_window(&mut self, ctx: &egui::Context) {
        let Some(missing) = self.missing_dialog.as_ref() else { return };
        let mut open = true;
        let mut dismiss = false;
        egui::Window::new(crate::i18n::t("No matching Params"))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_size(Vec2::new(420.0, 260.0))
            .show(ctx, |ui| {
                ui.label(RichText::new(format!("Missing {} params", missing.len())).strong().color(Color32::WHITE));
                ui.add_space(4.0);
                egui::ScrollArea::vertical().max_height(180.0).show(ui, |ui| {
                    let list = missing.join(" ");
                    ui.label(RichText::new(list).color(Color32::LIGHT_GRAY));
                });
                ui.add_space(8.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if theme::button(ui, "OK").clicked() {
                        dismiss = true;
                    }
                });
            });
        if !open || dismiss {
            self.missing_dialog = None;
        }
    }

    /// ParamCompare form: diff against a file, tick the rows to take, Continue.
    fn compare_window(&mut self, ctx: &egui::Context) {
        let Some(diffs) = self.compare.as_mut() else { return };
        let mut open = true;
        let mut apply = false;
        egui::Window::new(crate::i18n::t("Param Compare")).open(&mut open).collapsible(false).default_size(Vec2::new(480.0, 400.0)).show(ctx, |ui| {
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
    fn grid(&mut self, ui: &mut egui::Ui, toasts: &mut Toasts, _link: Option<&Link>, fw: Firmware, rows: &[(String, f32)], favs: &mut Vec<String>, defaults: Option<&BTreeMap<String, f32>>) {
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
                let is_failed = self.failed.contains(name);
                // Value cell: Green when queued (_changes), Red when the entry could not be evaluated
                let val_fill = if is_failed {
                    Color32::RED
                } else if edited.is_some() {
                    Color32::from_rgb(0, 128, 0)
                } else {
                    bg
                };
                for (c, r) in cells.iter().enumerate().filter(|(_, r)| r.width() > 0.0) {
                    let fill = if c == 1 { val_fill } else { bg };
                    ui.painter().rect_filled(*r, 0.0, fill);
                    ui.painter().rect_stroke(*r, 0.0, Stroke::new(1.0, line), StrokeKind::Inside);
                }
                let txt = |ui: &egui::Ui, r: Rect, s: &str| {
                    ui.painter().with_clip_rect(r.shrink(1.0)).text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, s, f.clone(), Color32::WHITE);
                };
                // ToolTipText: the word-wrapped description on Name, Value and Desc
                let tip = meta.map(|m| m.description.as_str()).filter(|d| !d.is_empty()).map(tooltip_lines);

                txt(ui, cells[0], name);
                let mut name_resp = ui.interact(cells[0], ui.scope_id().with(("pn", name)), Sense::click().union(Sense::hover()));
                if let Some(t) = &tip {
                    name_resp = name_resp.on_hover_text(t);
                }
                if name_resp.clicked() {
                    self.current = Some(name.clone());
                }

                // Value cell: editable
                let v = edited.unwrap_or(*value);
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
                let mut queued: Option<f64> = None;
                if !editing {
                    let mut resp = child.put(cells[1].shrink(1.0), egui::Label::new(egui::RichText::new(paramfile::format_value(v)).color(Color32::WHITE)).sense(Sense::click().union(Sense::hover())));
                    if let Some(t) = &tip {
                        resp = resp.on_hover_text(t);
                    }
                    if resp.clicked() {
                        self.editing = Some((name.clone(), paramfile::format_value(v)));
                        self.current = Some(name.clone());
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
                                        queued = Some(*k as f64);
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
                        // Escape cancels the edit (DataGridView CancelEdit); anything else ends it
                        if !ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                            // Params_CellValueChanged: parse, then the checked path
                            match cell_value(name, buf) {
                                None => {
                                    self.failed.insert(name.clone());
                                }
                                Some(nv) => queued = Some(nv),
                            }
                        }
                        self.editing = None;
                    }
                }
                if let Some(nv) = queued {
                    self.queue_value(name, nv, *value, meta, toasts);
                }
                // Default_value: the vehicle's default, else "NaN"
                if let Some(d) = defaults {
                    txt(ui, cells[2], &d.get(name).map(|v| paramfile::format_value(*v)).unwrap_or_else(|| "NaN".into()));
                }
                txt(ui, cells[3], meta.map(|m| m.units.as_str()).unwrap_or(""));
                if let Some(m) = meta {
                    let opts = options_text(m);
                    let current = self.current.as_deref() == Some(name.as_str());
                    if current && !m.bitmask.is_empty() {
                        // a bitmask gets a button that opens the checkbox editor
                        let br = cells[4].shrink(1.0);
                        if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, "Set Bitmask", br.size())).clicked() {
                            self.bitmask_edit = Some(name.clone());
                        }
                    } else if current && !m.values.is_empty() {
                        // options: a combo box in the cell
                        let mut pick = None;
                        ui.put(cells[4].shrink(1.0), |ui: &mut egui::Ui| {
                            let sel = m.values.iter().find(|(k, _)| *k as f32 == v).map(|(_, s)| s.clone()).unwrap_or_default();
                            egui::ComboBox::from_id_salt(("popt", name)).width(cells[4].width() - 8.0).selected_text(sel).show_ui(ui, |ui| {
                                for (k, s) in &m.values {
                                    if ui.selectable_label(*k as f32 == v, s).clicked() {
                                        pick = Some(*k as f32);
                                    }
                                }
                            }).response
                        });
                        if let Some(nv) = pick {
                            // the Options combo sets the Value cell, which runs CellValueChanged
                            self.queue_value(name, nv as f64, *value, meta, toasts);
                        }
                    } else {
                        txt(ui, cells[4], &opts);
                        // Options.ToolTipText: the values one per line (in columns past 50)
                        if let Some(t) = options_tooltip(m) {
                            ui.interact(cells[4], ui.scope_id().with(("po", name)), Sense::hover()).on_hover_text(t);
                        }
                    }
                    txt(ui, cells[5], &m.description);
                    if let Some(t) = &tip {
                        ui.interact(cells[5], ui.scope_id().with(("pd", name)), Sense::hover()).on_hover_text(t);
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

impl ParamEditor {
    /// The MavlinkCheckBoxBitMask window opened from "Set Bitmask"; ticks update the cell value.
    fn bitmask_window(&mut self, ctx: &egui::Context, fw: Firmware, params: &BTreeMap<String, f32>, toasts: &mut Toasts) {
        let Some(name) = self.bitmask_edit.clone() else { return };
        let Some(m) = param_meta::get(fw, &name) else {
            self.bitmask_edit = None;
            return;
        };
        let cur = self.edits.get(&name).copied().or_else(|| params.get(&name).copied()).unwrap_or(0.0);
        let mut value = cur as u32;
        let mut open = true;
        egui::Window::new(m.display_name.clone()).id(egui::Id::new("raw_bitmask")).open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            ui.label(RichText::new(&m.display_name).font(fonts::bold_id(13.0)).color(Color32::WHITE));
            ui.add(egui::Label::new(RichText::new(&m.description).color(theme::text())).wrap());
            ui.set_max_width(520.0);
            ui.horizontal_wrapped(|ui| {
                for (bit, label) in &m.bitmask {
                    let mut on = value & (1 << bit) != 0;
                    if ui.checkbox(&mut on, label).changed() {
                        value = if on { value | (1 << bit) } else { value & !(1 << bit) };
                    }
                }
            });
        });
        if value as f32 != cur {
            let current = params.get(&name).copied().unwrap_or(0.0);
            self.queue_value(&name, value as f64, current, Some(m), toasts);
        }
        if !open {
            self.bitmask_edit = None;
        }
    }
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
    p.text(at(94.0, 15.0), egui::Align2::LEFT_TOP, crate::i18n::t("Current Mode:"), f.clone(), Color32::WHITE);
    p.text(at(174.0, 15.0), egui::Align2::LEFT_TOP, &cs.mode, f.clone(), Color32::WHITE);
    p.text(at(94.0, 32.0), egui::Align2::LEFT_TOP, crate::i18n::t("Current PWM:"), f.clone(), Color32::WHITE);
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
        let standard_only = crate::display::flag(&app.settings.planner, "standardFlightModesOnly");
        if !standard_only && simple_p.is_some() {
            let mut b = st.1 & (1 << i) != 0;
            if ui.put(Rect::from_min_size(at(222.0, y + 2.0), Vec2::new(105.0, 16.0)), egui::Checkbox::new(&mut b, crate::i18n::t("Simple Mode"))).changed() {
                st.1 = if b { st.1 | (1 << i) } else { st.1 & !(1 << i) };
            }
        }
        if !standard_only && ss_p.is_some() {
            let mut b = st.2 & (1 << i) != 0;
            if ui.put(Rect::from_min_size(at(332.0, y + 2.0), Vec2::new(135.0, 16.0)), egui::Checkbox::new(&mut b, crate::i18n::t("Super Simple Mode"))).changed() {
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
            p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, crate::i18n::t(t), f.clone(), Color32::WHITE);
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
            (9.0, 477.0, "GCS ID"),
            (9.0, 502.0, "Aircraft Icon"),
            (841.0, 502.0, "Line Length"),
            (9.0, 527.0, "Inactive Aircraft"),
            (9.0, 656.0, "Map Access Mode"),
        ] {
            lab(x, y, t);
        }
        if crate::display::flag(pl, "displayPlannerLayout") {
            lab(9.0, 451.0, "Layout");
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
        // CMB_language: CultureInfo display names; a change is saved and needs a restart
        {
            let lr = r(107.0, 139.0, 138.0, 21.0);
            let cur = pl.get("language").cloned().unwrap_or_else(|| crate::i18n::culture().to_string());
            let shown = crate::i18n::CULTURES.iter().find(|(c, _)| c.eq_ignore_ascii_case(&cur)).map(|(_, n)| *n).unwrap_or("English (United States)");
            let mut pick = None;
            ui.put(lr, |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt(("pl", "language")).width(lr.width() - 8.0).wrap_mode(egui::TextWrapMode::Truncate).selected_text(shown).show_ui(ui, |ui| {
                    for (code, name) in crate::i18n::CULTURES {
                        if ui.selectable_label(cur.eq_ignore_ascii_case(code), *name).clicked() {
                            pick = Some(*code);
                        }
                    }
                }).response
            });
            if let Some(code) = pick.filter(|c| !c.eq_ignore_ascii_case(&cur)) {
                pl.insert("language".into(), code.to_string());
                ui.ctx().data_mut(|d| d.insert_temp(egui::Id::new("language_restart"), true));
            }
            if ui.ctx().data(|d| d.get_temp::<bool>(egui::Id::new("language_restart"))).unwrap_or(false) {
                egui::Modal::new(egui::Id::new("language_restart_modal")).show(ui.ctx(), |ui| {
                    theme::label(ui, "Please Restart the Planner");
                    if theme::button(ui, "OK").clicked() {
                        // MainV2.instance.Close()
                        ui.ctx().data_mut(|d| d.remove::<bool>(egui::Id::new("language_restart")));
                        ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
            }
        }
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
            egui::Window::new(crate::i18n::t("Custom Theme")).collapsible(false).resizable(false).show(ui.ctx(), |ui| {
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
        // CMB_Layout: hidden when the view has displayPlannerLayout off
        if crate::display::flag(pl, "displayPlannerLayout") {
            let cur = crate::display::layout(pl);
            let rr = r(107.0, 448.0, 138.0, 21.0);
            ui.put(rr, |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt(("pl", "displayview"))
                    .width(rr.width() - 8.0)
                    .selected_text(cur.name())
                    .show_ui(ui, |ui| {
                        for o in [crate::display::Layout::Basic, crate::display::Layout::Advanced, crate::display::Layout::Custom] {
                            if ui.selectable_label(cur == o, o.name()).clicked() {
                                crate::display::select(pl, o);
                            }
                        }
                    })
                    .response
            });
        }
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
            egui::Window::new(crate::i18n::t("Notification")).collapsible(false).resizable(false).show(ui.ctx(), |ui| {
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
        if crate::display::flag(pl, "displayParamCommitButton") {
            check(ui, pl, "autoParamCommit", r(598.0, 578.0, 130.0, 17.0), "Auto Commit Params", true);
        }
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
