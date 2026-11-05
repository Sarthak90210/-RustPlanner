//! Port of `GCSViews/FlightData` laid out from FlightData.Designer.cs / FlightData.resx:
//! MainH (left 300px: HUD over tabControlactions) | tableMap (map + 25px bottom panel).

use std::time::Instant;

use egui::{Color32, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Vec2};
use mp_core::link::{LinkCommand, LinkEvent, LinkStatus};
use mp_core::mavcmd;
use mp_core::mission::Locationwp;
use mp_core::state::CurrentState;

use crate::app::AppCtx;
use crate::{fonts, hud, map, theme};

const TABS: [&str; 13] = [
    "Quick",
    "Actions",
    "Messages",
    "PreFlight",
    "Gauges",
    "Transponder",
    "Status",
    "Servo/Relay",
    "Aux Function",
    "Scripts",
    "Payload Control",
    "Telemetry Logs",
    "DataFlash Logs",
];

/// quickView1..6 numberColor from FlightData.Designer.cs
const QUICK_COLORS: [Color32; 6] = [
    Color32::from_rgb(209, 151, 248),
    Color32::from_rgb(254, 132, 46),
    Color32::from_rgb(255, 96, 91),
    Color32::from_rgb(0, 255, 83),
    Color32::from_rgb(254, 254, 86),
    Color32::from_rgb(0, 255, 252),
];

/// CMB_action items (MAV_CMD names offered by the Actions tab).
const ACTIONS: &[(&str, u16, [f32; 7])] = &[
    ("RETURN_TO_LAUNCH", 20, [0.0; 7]),
    ("LAND", 21, [0.0; 7]),
    ("MISSION_START", 300, [0.0; 7]),
    ("PREFLIGHT_CALIBRATION", 241, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ("PREFLIGHT_REBOOT_SHUTDOWN", 246, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ("DO_DIGICAM_CONTROL", 203, [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
    ("DO_SET_SERVO", 183, [0.0; 7]),
    ("DO_REPEAT_SERVO", 184, [0.0; 7]),
    ("DO_SET_RELAY", 181, [0.0; 7]),
];

const MOUNT_MODES: &[&str] = &["RETRACT", "NEUTRAL", "MAVLINK_TARGETING", "RC_TARGETING", "GPS_POINT"];

pub struct FlightDataView {
    tab: usize,
    auto_pan: bool,
    tuning: bool,
    mission: Vec<Locationwp>,
    mode_sel: Option<u32>,
    wp_sel: u16,
    action_sel: usize,
    mount_sel: usize,
    change_speed: f32,
    change_alt: f32,
    loiter_rad: f32,
    quick_edit: Option<usize>,
    guided_alt: f32,
    guided_target: Option<(f64, f64)>,
    last_mode: String,
    mode_changed: Instant,
    last_armed: bool,
    armed_at: Instant,
    left_w: f32,
}

impl Default for FlightDataView {
    fn default() -> Self {
        Self {
            tab: 0,
            auto_pan: true,
            tuning: false,
            mission: Vec::new(),
            mode_sel: None,
            wp_sel: 0,
            action_sel: 0,
            mount_sel: 0,
            change_speed: 0.0,
            change_alt: 100.0,
            loiter_rad: 0.0,
            quick_edit: None,
            guided_alt: 100.0,
            guided_target: None,
            last_mode: String::new(),
            mode_changed: Instant::now(),
            last_armed: false,
            armed_at: Instant::now(),
            left_w: 300.0,
        }
    }
}

impl FlightDataView {
    pub fn on_event(&mut self, e: &LinkEvent) {
        if let LinkEvent::MissionDownloaded { mission_type: 0, items } = e {
            self.mission = items.clone();
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let cs = app.link.map(|l| l.shared.read().cs.clone()).unwrap_or_default();
        let connected = app.connected();
        if cs.mode != self.last_mode {
            self.last_mode = cs.mode.clone();
            self.mode_changed = Instant::now();
        }
        if cs.armed && !self.last_armed {
            self.armed_at = Instant::now();
        }
        self.last_armed = cs.armed;

        let narrow = ui.available_width() < 700.0;
        if narrow {
            // phone: HUD, map, tabs stacked
            let w = ui.available_width();
            self.hud_ui(ui, &cs, connected, Vec2::new(w, (w * 0.75).round()));
            egui::Panel::bottom("fd_tabs")
                .resizable(true)
                .default_size(ui.available_height() * 0.45)
                .frame(egui::Frame::new().fill(theme::BG))
                .show(ui, |ui| self.tabs_ui(ui, app, &cs));
            egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::BG)).show(ui, |ui| self.map_area(ui, app, &cs));
            return;
        }

        // MainH: SplitterDistance = 300, FixedPanel = Panel1
        let r = egui::Panel::left("MainH")
            .resizable(true)
            .default_size(self.left_w)
            .size_range(200.0..=ui.available_width() * 0.7)
            .frame(egui::Frame::new().fill(theme::BG))
            .show(ui, |ui| {
                let w = ui.available_width();
                // SubMainLeft: HUD keeps a 4:3 aspect
                let hud_h = (w * 0.75).round().min(ui.available_height() - 120.0);
                self.hud_ui(ui, &cs, connected, Vec2::new(w, hud_h));
                ui.add_space(4.0); // splitter
                self.tabs_ui(ui, app, &cs);
            });
        self.left_w = r.response.rect.width();
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(theme::BG).inner_margin(egui::Margin { left: 4, right: 1, top: 4, bottom: 1 }))
            .show(ui, |ui| self.map_area(ui, app, &cs));
    }

    fn hud_ui(&self, ui: &mut egui::Ui, cs: &CurrentState, connected: bool, size: Vec2) {
        hud::show(
            ui,
            &hud::HudInput {
                cs,
                connected,
                mode_changed_secs: self.mode_changed.elapsed().as_secs_f64(),
                armed_secs: self.armed_at.elapsed().as_secs_f64(),
            },
            size,
        );
    }

    // ------------------------------------------------------------------ tabControlactions

    fn tabs_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        theme::tab_strip(ui, "tabControlactions", &TABS, &mut self.tab);
        let rect = ui.available_rect_before_wrap();
        ui.painter().rect_filled(rect, 0.0, theme::BG);
        ui.scope_builder(egui::UiBuilder::new().max_rect(rect.shrink(3.0)), |ui| {
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| match self.tab {
                0 => self.quick_ui(ui, app, cs),
                1 => self.actions_ui(ui, app, cs),
                2 => messages_ui(ui, app),
                3 => preflight_ui(ui, cs),
                4 => gauges_ui(ui, cs),
                5 => transponder_ui(ui, app),
                6 => status_ui(ui, cs),
                7 => servo_ui(ui, app, cs),
                8 => aux_ui(ui, app),
                9 => scripts_ui(ui, app),
                10 => payload_ui(ui, app),
                11 => tlogs_ui(ui, app),
                _ => dflogs_ui(ui, app),
            });
        });
    }

    fn quick_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        // tableLayoutPanelQuick: 2 columns x 3 rows, Dock = Fill
        let area = ui.available_rect_before_wrap();
        let h = area.height().max(150.0) / 3.0;
        let w = area.width() / 2.0;
        for i in 0..6 {
            let (col, row) = (i % 2, i / 2);
            let r = Rect::from_min_size(area.min + Vec2::new(col as f32 * w, row as f32 * h), Vec2::new(w, h)).shrink(3.0);
            let name = app.settings.quick_fields[i].clone();
            let val = cs.field(&name).unwrap_or(0.0);
            let resp = ui.interact(r, ui.scope_id().with(("qv", i)), Sense::click());
            quick_view(ui.painter(), r, &quick_label(&name), val, QUICK_COLORS[i]);
            if resp.double_clicked() || resp.secondary_clicked() {
                self.quick_edit = Some(i);
            }
        }
        ui.allocate_rect(Rect::from_min_size(area.min, Vec2::new(area.width(), h * 3.0)), Sense::hover());

        if let Some(i) = self.quick_edit {
            let mut open = true;
            egui::Window::new("Display This").open(&mut open).collapsible(false).show(ui.ctx(), |ui| {
                egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                    egui::Grid::new("qsel").num_columns(4).show(ui, |ui| {
                        for (k, (n, _)) in cs.fields().into_iter().enumerate() {
                            if ui.checkbox(&mut (app.settings.quick_fields[i] == n), n).clicked() {
                                app.settings.quick_fields[i] = n.to_string();
                                self.quick_edit = None;
                            }
                            if k % 4 == 3 {
                                ui.end_row();
                            }
                        }
                    });
                });
            });
            if !open {
                self.quick_edit = None;
            }
        }
    }

    fn actions_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        let link = app.link.filter(|l| l.status() == LinkStatus::Connected);
        let modes = cs.firmware.modes();
        // tableLayoutPanel1: 287 x 181, 5 columns x 5 rows
        let origin = ui.available_rect_before_wrap().min;
        let col_x = [0.0, 57.0, 114.0, 171.0, 228.0];
        let col_w = [57.0, 57.0, 57.0, 57.0, 59.0];
        let row_y = [0.0, 36.0, 72.0, 108.0, 144.0];
        let cell = |c: usize, r: usize| Rect::from_min_size(origin + Vec2::new(col_x[c], row_y[r]), Vec2::new(col_w[c], 36.0)).shrink(3.0);
        let mut clicked: Option<&str> = None;
        let mut btn = |ui: &mut egui::Ui, c: usize, r: usize, text: &'static str| {
            let rr = cell(c, r);
            if ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, text, rr.size())).clicked() {
                clicked = Some(text);
            }
        };
        let rr = cell(0, 0);
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_action")
                .width(rr.width() - 4.0)
                .selected_text(ACTIONS[self.action_sel].0)
                .show_ui(ui, |ui| {
                    for (i, a) in ACTIONS.iter().enumerate() {
                        ui.selectable_value(&mut self.action_sel, i, a.0);
                    }
                })
                .response
        });
        let rr = cell(0, 1);
        let wp_count = self.mission.len().max(1) as u16;
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_setwp")
                .width(rr.width() - 4.0)
                .selected_text(self.wp_sel.to_string())
                .show_ui(ui, |ui| {
                    for i in 0..wp_count {
                        ui.selectable_value(&mut self.wp_sel, i, i.to_string());
                    }
                })
                .response
        });
        let sel_mode = self.mode_sel.unwrap_or(cs.custom_mode);
        let sel_name = modes.iter().find(|(k, _)| *k == sel_mode).map(|(_, n)| n.clone()).unwrap_or_default();
        let rr = cell(0, 2);
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_modes")
                .width(rr.width() - 4.0)
                .selected_text(sel_name)
                .show_ui(ui, |ui| {
                    for (k, n) in &modes {
                        if ui.selectable_label(sel_mode == *k, n).clicked() {
                            self.mode_sel = Some(*k);
                        }
                    }
                })
                .response
        });
        let rr = cell(0, 3);
        ui.put(rr, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_mountmode")
                .width(rr.width() - 4.0)
                .selected_text(MOUNT_MODES[self.mount_sel])
                .show_ui(ui, |ui| {
                    for (i, m) in MOUNT_MODES.iter().enumerate() {
                        ui.selectable_value(&mut self.mount_sel, i, *m);
                    }
                })
                .response
        });
        btn(ui, 1, 0, "Do Action");
        btn(ui, 1, 1, "Set WP");
        btn(ui, 1, 2, "Set Mode");
        btn(ui, 1, 3, "Set Mount");
        btn(ui, 2, 0, "Auto");
        btn(ui, 2, 1, "Loiter");
        btn(ui, 2, 2, "RTL");
        btn(ui, 2, 3, "Joystick");
        btn(ui, 2, 4, "Message");
        btn(ui, 3, 0, "Set Home Alt");
        btn(ui, 3, 1, "Restart Mission");
        btn(ui, 3, 2, "Raw Sensor View");
        btn(ui, 3, 3, "Arm/ Disarm");
        btn(ui, 3, 4, "Resume Mission");
        btn(ui, 4, 3, "Clear Track");
        btn(ui, 4, 4, "Abort Landing");
        // column 4 rows 0-2: ModifyandSet (numeric above button)
        for (r, (val, text)) in [(&mut self.change_speed, "Change Speed"), (&mut self.change_alt, "Change Alt"), (&mut self.loiter_rad, "Set Loiter Rad")]
            .into_iter()
            .enumerate()
        {
            let rr = cell(4, r);
            let top = Rect::from_min_size(rr.min, Vec2::new(rr.width(), 14.0));
            let bot = Rect::from_min_max(Pos2::new(rr.left(), rr.top() + 15.0), rr.max);
            ui.put(top, egui::DragValue::new(val).speed(0.5).max_decimals(1));
            if ui.put(bot, |ui: &mut egui::Ui| theme::button_sized(ui, text, bot.size())).clicked() {
                clicked = Some(text);
            }
        }
        ui.allocate_rect(Rect::from_min_size(origin, Vec2::new(287.0, 181.0)), Sense::hover());

        let Some(text) = clicked else { return };
        let Some(link) = link else {
            app.toasts.error("Not connected");
            return;
        };
        let find = |name: &str| modes.iter().find(|(_, n)| n.eq_ignore_ascii_case(name)).map(|(k, _)| *k);
        let fw = cs.firmware;
        match text {
            "Do Action" => {
                let (_, cmd, p) = ACTIONS[self.action_sel];
                link.send(LinkCommand::CommandLong { command: cmd, params: p });
            }
            "Set WP" => link.send(LinkCommand::SetCurrentWp(self.wp_sel)),
            "Set Mode" => link.send(LinkCommand::SetMode(sel_mode)),
            "Auto" | "Loiter" | "RTL" => {
                if let Some(k) = find(text) {
                    link.send(LinkCommand::SetMode(k));
                }
            }
            "Arm/ Disarm" => link.send(LinkCommand::Arm { arm: !cs.armed, force: false }),
            "Restart Mission" => {
                link.send(LinkCommand::SetCurrentWp(0));
                if let Some(k) = find("Auto") {
                    link.send(LinkCommand::SetMode(k));
                }
            }
            "Resume Mission" => {
                if let Some(k) = find("Auto") {
                    link.send(LinkCommand::SetMode(k));
                }
            }
            "Clear Track" => link.shared.write().cs.track.clear(),
            "Change Speed" => link.send(LinkCommand::CommandLong { command: 178, params: [1.0, self.change_speed, -1.0, 0.0, 0.0, 0.0, 0.0] }),
            "Change Alt" => link.send(LinkCommand::Guided { lat: cs.lat, lng: cs.lng, alt: self.change_alt }),
            "Set Loiter Rad" => {
                let plane = fw == mp_core::firmware::Firmware::ArduPlane;
                let name = if plane { "WP_LOITER_RAD" } else { "CIRCLE_RADIUS" };
                let scale = if plane { 1.0 } else { 100.0 };
                link.send(LinkCommand::SetParam { name: name.into(), value: self.loiter_rad * scale });
            }
            "Abort Landing" => link.send(LinkCommand::CommandLong { command: 191, params: [0.0; 7] }),
            "Set Home Alt" => {
                if let Some(h) = cs.home {
                    app.toasts.info(format!("Home alt {:.1} m", h.alt));
                }
            }
            "Set Mount" => link.send(LinkCommand::CommandLong { command: 204, params: [self.mount_sel as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }),
            other => not_ported(app, other),
        }
    }

    // ------------------------------------------------------------------ tableMap

    fn map_area(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        // panel1 (25px) along the bottom: coords1, Tuning, Auto Pan
        egui::Panel::bottom("fd_panel1").exact_size(25.0).frame(egui::Frame::new().fill(theme::BG)).show(ui, |ui| {
            let o = ui.max_rect().min;
            let coords = Rect::from_min_size(o + Vec2::new(7.0, 2.0), Vec2::new(247.0, 21.0));
            ui.painter().rect_filled(coords, 0.0, theme::CONTROL_BG);
            ui.painter().rect_stroke(coords, 0.0, Stroke::new(1.0, Color32::from_rgb(0x7a, 0x7a, 0x7a)), StrokeKind::Inside);
            ui.painter().text(
                coords.left_center() + Vec2::new(4.0, 0.0),
                egui::Align2::LEFT_CENTER,
                format!("{:.6} {:.6}  {:.2}  GEO", cs.lat, cs.lng, cs.alt),
                fonts::ui_id(fonts::UI_SIZE),
                Color32::WHITE,
            );
            ui.put(Rect::from_min_size(o + Vec2::new(260.0, 3.0), Vec2::new(59.0, 17.0)), egui::Checkbox::new(&mut self.tuning, "Tuning"));
            ui.put(Rect::from_min_size(o + Vec2::new(325.0, 3.0), Vec2::new(75.0, 17.0)), egui::Checkbox::new(&mut self.auto_pan, "Auto Pan"));
        });

        egui::CentralPanel::default().frame(egui::Frame::new()).show(ui, |ui| {
            if self.tuning {
                // splitContainer1.Panel1: zg1 tuning graph
                let h = ui.available_height() * 0.5;
                let (r, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), h), Sense::hover());
                ui.painter().rect_filled(r, 0.0, Color32::from_rgb(0x37, 0x37, 0x38));
                ui.painter().rect_filled(r.shrink(20.0), 0.0, Color32::from_rgb(0x1f, 0x1f, 0x20));
                ui.painter().text(r.center_top() + Vec2::new(0.0, 4.0), egui::Align2::CENTER_TOP, "Tuning", fonts::ui_id(12.0), Color32::WHITE);
                ui.add_space(4.0);
            }
            self.map_ui(ui, app, cs);
        });
    }

    fn map_ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        let has_pos = cs.lat != 0.0 || cs.lng != 0.0;
        if self.auto_pan && has_pos {
            app.settings.fd_map.center_on(cs.lat, cs.lng);
        }
        let mission = self.mission.clone();
        let home = cs.home;
        let assets = app.assets;
        let zoom = app.settings.fd_map.zoom;
        let guided = self.guided_target;
        let mut context_latlng = None;
        let out = map::show(ui, &mut app.settings.fd_map, app.tiles, |_ui, view, resp, painter| {
            let pts: Vec<Pos2> = mission
                .iter()
                .skip(1)
                .filter(|w| w.has_location() && (w.lat != 0.0 || w.lng != 0.0))
                .map(|w| view.to_screen(w.lat, w.lng))
                .collect();
            if pts.len() > 1 {
                painter.add(egui::Shape::line(pts, Stroke::new(2.0, Color32::YELLOW)));
            }
            let hover = resp.hover_pos();
            for (i, w) in mission.iter().enumerate().skip(1) {
                if w.has_location() && (w.lat != 0.0 || w.lng != 0.0) {
                    let p = view.to_screen(w.lat, w.lng);
                    let hov = hover.map(|h| (h - p).length() < 16.0).unwrap_or(false);
                    map::draw_wp(painter, assets.marker_green.id(), assets.marker_shadow.id(), p, &i.to_string(), false, zoom, hov);
                }
            }
            if let Some(h) = home {
                map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), view.to_screen(h.lat, h.lng));
            }
            if let Some((lat, lng)) = guided {
                map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), view.to_screen(lat, lng));
            }
            if cs.track.len() > 1 {
                let t: Vec<Pos2> = cs.track.iter().map(|&(a, b)| view.to_screen(a, b)).collect();
                painter.add(egui::Shape::line(t, Stroke::new(2.0, Color32::from_rgb(255, 0, 255))));
            }
            if has_pos {
                let p = view.to_screen(cs.lat, cs.lng);
                map::draw_heading_lines(painter, p, cs.yaw, cs.groundcourse, cs.target_bearing);
                map::draw_quad(painter, p, cs.yaw, cs.sysid.max(1));
            }
            if resp.secondary_clicked() {
                if let Some(pos) = resp.interact_pointer_pos() {
                    context_latlng = Some(view.to_latlng(pos));
                }
            }
            false
        });
        let mrect = out.view.rect;
        if let Some(ll) = context_latlng {
            ui.ctx().data_mut(|d| d.insert_temp(egui::Id::unique("fd_ctx_ll"), ll));
        }
        if out.response.dragged() {
            self.auto_pan = false;
        }
        let link = app.link;
        out.response.context_menu(|ui| {
            let ll: (f64, f64) = ui.ctx().data(|d| d.get_temp(egui::Id::unique("fd_ctx_ll"))).unwrap_or_default();
            if ui.button("Fly To Here").clicked() {
                if let Some(l) = link {
                    l.send(LinkCommand::Guided { lat: ll.0, lng: ll.1, alt: self.guided_alt });
                    self.guided_target = Some(ll);
                }
                ui.close();
            }
            ui.horizontal(|ui| {
                ui.label("Fly To Here Alt");
                ui.add(egui::DragValue::new(&mut self.guided_alt).suffix(" m"));
            });
            if ui.button("Point Camera Here").clicked() {
                if let Some(l) = link {
                    l.send(LinkCommand::CommandLong { command: mavcmd::DO_SET_ROI, params: [0.0, 0.0, 0.0, 0.0, ll.0 as f32, ll.1 as f32, 0.0] });
                }
                ui.close();
            }
            if ui.button("Trigger Camera NOW").clicked() {
                if let Some(l) = link {
                    l.send(LinkCommand::CommandLong { command: 203, params: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0] });
                }
                ui.close();
            }
            if ui.button("Set Home Here").clicked() {
                if let Some(l) = link {
                    l.send(LinkCommand::CommandLong { command: 179, params: [0.0, 0.0, 0.0, 0.0, ll.0 as f32, ll.1 as f32, 0.0] });
                }
                ui.close();
            }
            if ui.button("TakeOff").clicked() {
                if let Some(l) = link {
                    l.send(LinkCommand::CommandLong { command: mavcmd::TAKEOFF, params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.guided_alt] });
                }
                ui.close();
            }
        });

        // ---- overlays positioned as in splitContainer1.Panel2
        let p = ui.painter().with_clip_rect(mrect);
        let o = mrect.min;
        let (w, h) = (mrect.width(), mrect.height());
        wind_dir(&p, Rect::from_min_size(o, Vec2::splat(60.0)), cs.wind_dir, cs.wind_vel);
        distance_bar(&p, Rect::from_min_size(o + Vec2::new(66.0, 0.0), Vec2::new((w - 131.0).max(10.0), 22.0)), cs, &mission);
        p.text(o + Vec2::new(w - 40.0, 3.0), egui::Align2::LEFT_TOP, "Zoom", fonts::ui_id(fonts::UI_SIZE), Color32::BLACK);
        let zl = Rect::from_min_size(o + Vec2::new(w - 44.0, 22.0), Vec2::new(42.0, 20.0));
        ui.put(zl, egui::DragValue::new(&mut app.settings.fd_map.zoom).speed(0.1).range(2.0..=22.0).max_decimals(1));
        let trk = Rect::from_min_size(o + Vec2::new(w - 45.0, 40.0), Vec2::new(45.0, (h - 54.0).max(40.0)));
        theme::vtrackbar(ui, trk, &mut app.settings.fd_map.zoom, 2.0, 22.0);
        let small = fonts::ui_id(fonts::UI_SIZE);
        p.text(o + Vec2::new(-1.0, h - 48.0), egui::Align2::LEFT_TOP, format!("hdop: {:.1}", cs.gpshdop), small.clone(), Color32::WHITE);
        p.text(o + Vec2::new(-1.0, h - 29.0), egui::Align2::LEFT_TOP, format!("Sats: {}", cs.satcount), small.clone(), Color32::WHITE);
        let ly = h - 30.0;
        p.text(o + Vec2::new(45.0, ly), egui::Align2::LEFT_TOP, "Current Heading", small.clone(), Color32::RED);
        p.text(o + Vec2::new(135.0, ly), egui::Align2::LEFT_TOP, "Direct to current WP", small.clone(), Color32::from_rgb(255, 128, 0));
        p.text(o + Vec2::new(245.0, ly), egui::Align2::LEFT_TOP, "Target Heading", small.clone(), Color32::from_rgb(0, 128, 0));
        let gl = p.layout_no_wrap("GPS Track (Black)".into(), small, Color32::WHITE);
        let gr = Rect::from_min_size(o + Vec2::new(332.0, ly), gl.size());
        p.rect_filled(gr, 0.0, Color32::BLACK);
        p.galley(gr.min, gl, Color32::WHITE);
    }
}

// ------------------------------------------------------------------ QuickView

/// `ExtLibs/Controls/QuickView`: description centred at y=5, the number scaled to fill.
fn quick_view(p: &egui::Painter, r: Rect, desc: &str, number: f64, color: Color32) {
    let dg = p.layout_no_wrap(desc.to_string(), fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
    let y = 5.0 + dg.size().y;
    p.galley(Pos2::new(r.center().x - dg.size().x / 2.0, r.top() + 5.0), dg, Color32::WHITE);
    let numb = format!("{number:.2}");
    let avail_h = r.height() - y;
    let probe = "0".repeat(numb.len() + 1);
    let base = p.layout_no_wrap(probe, fonts::ui_id(100.0), color).size();
    let ratio = (r.width() / base.x).min(avail_h / base.y);
    let mut size = 100.0 * ratio;
    size -= size % 5.0;
    if size < 8.0 {
        size = 8.0;
    }
    let ng = p.layout_no_wrap(numb, fonts::ui_id(size), color);
    let pos = Pos2::new(r.center().x - ng.size().x / 2.0, r.top() + y + (avail_h - ng.size().y) / 2.0);
    p.galley(pos, ng, color);
}

fn quick_label(name: &str) -> String {
    match name {
        "alt" => "Altitude (m)".into(),
        "groundspeed" => "GroundSpeed (m/s)".into(),
        "wp_dist" => "Dist to WP (m)".into(),
        "yaw" => "Yaw (deg)".into(),
        "verticalspeed" => "Vertical Speed (m/s)".into(),
        "DistToHome" => "DistToMAV (m)".into(),
        n => n.to_string(),
    }
}

// ------------------------------------------------------------------ map overlays

/// `Controls/WindDir`: circle with the wind arrow and speed.
fn wind_dir(p: &egui::Painter, r: Rect, dir: f32, speed: f32) {
    let c = r.center();
    let rad = r.width() / 2.0 - 4.0;
    p.circle_stroke(c, rad, Stroke::new(1.0, Color32::BLACK));
    let a = (dir + 180.0 - 90.0).to_radians();
    let d = Vec2::new(a.cos(), a.sin()) * rad;
    p.arrow(c - d, d * 2.0, Stroke::new(2.0, Color32::BLACK));
    p.text(r.center_bottom() - Vec2::new(0.0, 2.0), egui::Align2::CENTER_BOTTOM, format!("{speed:.1}"), fonts::ui_id(9.0), Color32::BLACK);
}

/// `Controls/DistanceBar`: route waypoints with the vehicle's progress marker.
fn distance_bar(p: &egui::Painter, r: Rect, cs: &CurrentState, mission: &[Locationwp]) {
    let n = mission.iter().skip(1).filter(|w| w.has_location() && (w.lat != 0.0 || w.lng != 0.0)).count();
    if n < 2 {
        return;
    }
    let bar = Rect::from_center_size(r.center(), Vec2::new(r.width(), 8.0));
    p.rect_filled(bar, 0.0, Color32::from_black_alpha(120));
    p.rect_stroke(bar, 0.0, Stroke::new(1.0, Color32::WHITE), StrokeKind::Inside);
    for i in 0..n {
        let x = bar.left() + bar.width() * i as f32 / (n - 1) as f32;
        p.line_segment([Pos2::new(x, bar.top() - 2.0), Pos2::new(x, bar.bottom() + 2.0)], Stroke::new(1.0, Color32::WHITE));
    }
    let cur = (cs.wpno as usize).saturating_sub(1).min(n - 1);
    let x = bar.left() + bar.width() * cur as f32 / (n - 1) as f32;
    p.circle_filled(Pos2::new(x, bar.center().y), 4.0, Color32::RED);
}

// ------------------------------------------------------------------ other tabs

fn messages_ui(ui: &mut egui::Ui, app: &AppCtx) {
    // txt_messagebox: RichTextBox on ControlBGColor
    let r = ui.available_rect_before_wrap();
    ui.painter().rect_filled(r, 0.0, theme::CONTROL_BG);
    if let Some(l) = app.link {
        let s = l.shared.read();
        for m in s.messages.iter().rev() {
            theme::label(ui, format!("{} : {}", m.time, m.text));
        }
    }
}

fn preflight_ui(ui: &mut egui::Ui, cs: &CurrentState) {
    // Checklist control (checklistDefault.xml items)
    let vibe = cs.vibex.max(cs.vibey).max(cs.vibez);
    let items = [
        ("GPS Fix", cs.gpsstatus >= 3, cs.gps_fix_name().to_string()),
        ("Satellites >= 6", cs.satcount >= 6, cs.satcount.to_string()),
        ("Battery > 50%", cs.battery_remaining > 50, format!("{}%", cs.battery_remaining)),
        ("hdop < 2", cs.gpshdop > 0.0 && cs.gpshdop < 2.0, format!("{:.2}", cs.gpshdop)),
        ("EKF healthy", cs.ekfstatus < 0.5, format!("{:.2}", cs.ekfstatus)),
        ("Vibration < 30", vibe < 30.0, format!("{vibe:.1}")),
    ];
    for (t, ok, v) in items {
        ui.horizontal(|ui| {
            let mut c = ok;
            ui.add_enabled(false, egui::Checkbox::new(&mut c, ""));
            theme::label(ui, t);
            ui.label(RichText::new(v).color(if ok { Color32::from_rgb(0, 200, 0) } else { Color32::RED }));
        });
    }
    theme::button(ui, "Edit");
}

fn gauges_ui(ui: &mut egui::Ui, cs: &CurrentState) {
    let size = (ui.available_width() / 2.0 - 6.0).min(150.0);
    ui.horizontal_wrapped(|ui| {
        for (label, v, min, max) in [
            ("Speed", cs.groundspeed, 0.0, 60.0),
            ("Alt", cs.alt, 0.0, 600.0),
            ("VSI", cs.verticalspeed, -10.0, 10.0),
            ("Heading", cs.yaw, 0.0, 360.0),
        ] {
            let (r, _) = ui.allocate_exact_size(Vec2::splat(size), Sense::hover());
            gauge(ui.painter(), r, label, v, min, max);
        }
    });
}

fn gauge(p: &egui::Painter, r: Rect, label: &str, v: f32, min: f32, max: f32) {
    let c = r.center();
    let rad = r.width() / 2.0 - 4.0;
    p.circle_filled(c, rad, Color32::from_rgb(0x1f, 0x1f, 0x20));
    p.circle_stroke(c, rad, Stroke::new(3.0, Color32::from_gray(160)));
    let (a0, sweep) = (135.0f32, 270.0f32);
    for i in 0..=10 {
        let a = (a0 + sweep * i as f32 / 10.0).to_radians();
        let d = Vec2::new(a.cos(), a.sin());
        p.line_segment([c + d * (rad - 10.0), c + d * (rad - 2.0)], Stroke::new(2.0, Color32::WHITE));
        if i % 2 == 0 {
            let val = min + (max - min) * i as f32 / 10.0;
            p.text(c + d * (rad - 20.0), egui::Align2::CENTER_CENTER, format!("{val:.0}"), fonts::ui_id(9.0), Color32::WHITE);
        }
    }
    let f = ((v - min) / (max - min)).clamp(0.0, 1.0);
    let a = (a0 + sweep * f).to_radians();
    p.line_segment([c, c + Vec2::new(a.cos(), a.sin()) * (rad - 12.0)], Stroke::new(3.0, Color32::RED));
    p.circle_filled(c, 4.0, Color32::from_gray(200));
    p.text(c + Vec2::new(0.0, rad * 0.45), egui::Align2::CENTER_CENTER, format!("{label}\n{v:.1}"), fonts::ui_id(10.0), Color32::WHITE);
}

fn transponder_ui(ui: &mut egui::Ui, app: &mut AppCtx) {
    egui::Grid::new("xpdr").num_columns(2).show(ui, |ui| {
        for l in ["Squawk", "Flight ID", "Mode"] {
            theme::label(ui, l);
            ui.add_enabled(false, egui::TextEdit::singleline(&mut String::new()).desired_width(100.0));
            ui.end_row();
        }
    });
    if theme::button(ui, "Ident").clicked() {
        not_ported(app, "Transponder");
    }
}

fn status_ui(ui: &mut egui::Ui, cs: &CurrentState) {
    // tabStatus: flow layout of name/value label pairs
    ui.horizontal_wrapped(|ui| {
        for (n, v) in cs.fields() {
            ui.allocate_ui(Vec2::new(130.0, 16.0), |ui| {
                ui.horizontal(|ui| {
                    ui.add_sized(Vec2::new(70.0, 14.0), egui::Label::new(RichText::new(n).color(Color32::WHITE)).truncate());
                    ui.label(RichText::new(fmt_val(v)).color(Color32::WHITE));
                });
            });
        }
    });
}

fn servo_ui(ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
    let link = app.link.filter(|l| l.status() == LinkStatus::Connected);
    for ch in 5u16..=14 {
        ui.horizontal(|ui| {
            ui.add_sized(Vec2::new(54.0, 18.0), egui::Label::new(RichText::new(format!("Servo {ch}")).color(Color32::WHITE)));
            for (t, pwm) in [("Low", 1100.0), ("Mid", 1500.0), ("High", 1900.0)] {
                if theme::button_sized(ui, t, Vec2::new(40.0, 20.0)).clicked() {
                    if let Some(l) = link {
                        l.send(LinkCommand::CommandLong { command: 183, params: [ch as f32, pwm, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    }
                }
            }
            let cur = cs.ch_out.get(ch as usize - 1).copied().unwrap_or(0);
            if theme::button_sized(ui, "Toggle", Vec2::new(48.0, 20.0)).clicked() {
                if let Some(l) = link {
                    let to = if cur > 1500 { 1100.0 } else { 1900.0 };
                    l.send(LinkCommand::CommandLong { command: 183, params: [ch as f32, to, 0.0, 0.0, 0.0, 0.0, 0.0] });
                }
            }
            theme::label(ui, cur.to_string());
        });
    }
    for relay in 0..6 {
        ui.horizontal(|ui| {
            ui.add_sized(Vec2::new(54.0, 18.0), egui::Label::new(RichText::new(format!("Relay {relay}")).color(Color32::WHITE)));
            for (t, v) in [("Low", 0.0), ("High", 1.0)] {
                if theme::button_sized(ui, t, Vec2::new(40.0, 20.0)).clicked() {
                    if let Some(l) = link {
                        l.send(LinkCommand::CommandLong { command: 181, params: [relay as f32, v, 0.0, 0.0, 0.0, 0.0, 0.0] });
                    }
                }
            }
        });
    }
}

fn not_ported(app: &mut AppCtx, what: &str) {
    app.toasts.error(format!("{what}: not available in this port yet"));
}

fn placed_buttons(ui: &mut egui::Ui, app: &mut AppCtx, what: &str, btns: &[(f32, f32, f32, f32, &str)]) {
    let o = ui.available_rect_before_wrap().min;
    let mut maxx: f32 = 0.0;
    let mut maxy: f32 = 0.0;
    for &(x, y, w, h, t) in btns {
        let r = Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        if ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, t, r.size())).clicked() {
            not_ported(app, what);
        }
        maxx = maxx.max(x + w);
        maxy = maxy.max(y + h);
    }
    ui.allocate_rect(Rect::from_min_size(o, Vec2::new(maxx, maxy)), Sense::hover());
}

fn aux_ui(ui: &mut egui::Ui, app: &mut AppCtx) {
    placed_buttons(ui, app, "Aux Function", &[(3.0, 3.0, 75.0, 23.0, "Refresh")]);
}

fn scripts_ui(ui: &mut egui::Ui, app: &mut AppCtx) {
    let o = ui.available_rect_before_wrap().min;
    ui.painter().text(o + Vec2::new(19.0, 10.0), egui::Align2::LEFT_TOP, "Script Status: No Script Running", fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
    ui.painter().text(o + Vec2::new(19.0, 29.0), egui::Align2::LEFT_TOP, "Selected Script: None", fonts::ui_id(fonts::UI_SIZE), Color32::WHITE);
    placed_buttons(
        ui,
        app,
        "Scripts",
        &[
            (16.0, 80.0, 71.0, 23.0, "Select Script"),
            (92.0, 80.0, 71.0, 23.0, "Run Script"),
            (169.0, 80.0, 80.0, 23.0, "Abort Running Script"),
            (255.0, 79.0, 80.0, 23.0, "Edit Selected Script"),
        ],
    );
}

fn payload_ui(ui: &mut egui::Ui, app: &mut AppCtx) {
    placed_buttons(ui, app, "Payload Control", &[(10.0, 134.0, 56.0, 23.0, "Reset Position"), (73.0, 134.0, 56.0, 23.0, "Video Control")]);
}

fn tlogs_ui(ui: &mut egui::Ui, app: &mut AppCtx) {
    placed_buttons(
        ui,
        app,
        "Telemetry log playback",
        &[
            (3.0, 3.0, 80.0, 27.0, "Load Log"),
            (3.0, 36.0, 80.0, 27.0, "Play/Pause"),
            (3.0, 69.0, 80.0, 46.0, "Tlog > Kml or Graph"),
            (96.0, 23.0, 23.0, 30.0, "0.1"),
            (125.0, 23.0, 27.0, 30.0, "0.25"),
            (158.0, 23.0, 23.0, 30.0, "0.5"),
            (187.0, 23.0, 23.0, 30.0, "1x"),
            (96.0, 59.0, 23.0, 30.0, "2x"),
            (125.0, 59.0, 23.0, 30.0, "5x"),
            (154.0, 59.0, 23.0, 30.0, "10x"),
        ],
    );
}

fn dflogs_ui(ui: &mut egui::Ui, app: &mut AppCtx) {
    placed_buttons(
        ui,
        app,
        "DataFlash logs",
        &[
            (3.0, 3.0, 110.0, 38.0, "Download DataFlash Log Via Mavlink"),
            (119.0, 3.0, 87.0, 38.0, "Review a Log"),
            (235.0, 3.0, 87.0, 38.0, "Auto Analysis"),
            (3.0, 47.0, 110.0, 38.0, "Create KML + gpx"),
            (119.0, 47.0, 87.0, 38.0, "Convert .Bin to .Log"),
            (235.0, 47.0, 88.0, 38.0, "Create Matlab File"),
            (3.0, 91.0, 110.0, 40.0, "Geo Reference Images"),
        ],
    );
}

fn fmt_val(v: f64) -> String {
    if v.fract() == 0.0 {
        format!("{v:.0}")
    } else {
        format!("{v:.3}")
    }
}

pub fn provider_combo(ui: &mut egui::Ui, p: &mut map::Provider, id: &str) {
    egui::ComboBox::from_id_salt(id).selected_text(p.name()).width(103.0).wrap_mode(egui::TextWrapMode::Truncate).show_ui(ui, |ui| {
        for v in map::Provider::ALL {
            ui.selectable_value(p, v, v.name());
        }
    });
}

pub fn pwm_bar(ui: &mut egui::Ui, label: &str, pwm: u16) {
    ui.horizontal(|ui| {
        theme::label(ui, format!("{label:>2}"));
        let frac = ((pwm as f32 - 800.0) / 1400.0).clamp(0.0, 1.0);
        theme::progress(ui, frac, Vec2::new(ui.available_width().min(260.0), 18.0), &pwm.to_string());
    });
}
