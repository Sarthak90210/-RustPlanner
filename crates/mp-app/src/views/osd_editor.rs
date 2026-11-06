//! CONFIG > Onboard OSD (`ConfigOSD.cs` + ExtLibs/OSDConfigurator): one tab per OSD screen
//! with the 30x16 character canvas (items drawn in the clarity OSD font, dragged to move),
//! the item list with enable boxes, item X/Y options, screen options, and the
//! Write customization / Refresh / Discard all changes buttons.

use std::collections::BTreeMap;

use egui::{Color32, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use mp_core::link::{Link, LinkCommand};

use crate::app::AppCtx;
use crate::theme;

const COLS: i32 = 30;
const ROWS: i32 = 16;
const NTSC_ROWS: i32 = 13;

/// Sample captions (ConfigOSD.DoGetCaption) in plain ASCII.
fn caption(item: &str) -> String {
    match item {
        "ASPEED" | "ASPD1" | "ASPD2" => "A10m/s".into(),
        "ESCRPM" | "BLHRPM" => "10krpm".into(),
        "HDOP" => "HD1.2".into(),
        "ALTITUDE" => "11m".into(),
        "BAT_VOLT" | "BAT2_VLT" | "RESTVOLT" | "BATVOLT" => "B11.8V".into(),
        "RSSI" => "R93".into(),
        "CURRENT" | "CURRENT2" | "ESCAMPS" | "BLHAMPS" => "8.3A".into(),
        "FLTMODE" => "STAB*".into(),
        "SATS" => "S13".into(),
        "BATUSED" | "BAT2USED" => "125mAh".into(),
        "HORIZON" => "---<o>---".into(),
        "COMPASS" => "N.E.S.W".into(),
        "GPSLONG" => "  30.5003901".into(),
        "GPSLAT" => "  50.3534305".into(),
        "HOME" => "H> 101m".into(),
        "GSPEED" => "G> 17km/h".into(),
        "PITCH" => "P 10d".into(),
        "ROLL" => "R  3d".into(),
        "VSPEED" => "^ 0m/s".into(),
        "THROTTLE" => "0%".into(),
        "HEADING" => "32d".into(),
        "LINK_Q" => "99LQ".into(),
        "RNGF" => "r 1.23m".into(),
        "FENCE" => "F".into(),
        "AVGCELLV" | "CELLVOLT" => "B3.85V".into(),
        "VTX_PWR" => "25mW".into(),
        "CALLSIGN" => "TOPGUN".into(),
        "PLUSCODE" => "MP97+Q6H".into(),
        "SIDEBARS" => "|           |".into(),
        "CLK" => "C12:00".into(),
        "ATEMP" | "BTEMP" | "TEMP" | "ESCTEMP" | "BLHTEMP" => "25C".into(),
        "EFF" => "E100mAh".into(),
        "CLIMBEFF" => "^E2.1m".into(),
        "FLTIME" => "T03:00".into(),
        "DIST" => "D1.2km".into(),
        "MESSAGE" => "ArduCopter V4.6.0".into(),
        "WIND" => "W> 5m/s".into(),
        "WAYPOINT" => " #1 >  12m".into(),
        "XTRACK" => "X  1m".into(),
        "HOMEDIR" => ">".into(),
        "HOMEDIST" => "H 101m".into(),
        "POWER" => "100W".into(),
        "BATTBAR" => "[#######  ]".into(),
        "ARMING" => "ARMED".into(),
        "STATS" => "MAX 120m".into(),
        "TER_HGT" => "T 12m".into(),
        "CRSSHAIR" => "+".into(),
        other => other.into(),
    }
}

pub struct OsdEditor {
    screen: usize,
    selected: Option<String>,
    /// pending edits (param -> value), written by "Write customization"
    changes: BTreeMap<String, f32>,
    name_captions: bool,
    auto_write: bool,
    drag: Option<(String, Vec2)>,
    font: Option<egui::TextureHandle>,
}

impl Default for OsdEditor {
    fn default() -> Self {
        Self { screen: 1, selected: None, changes: BTreeMap::new(), name_captions: false, auto_write: false, drag: None, font: None }
    }
}

impl OsdEditor {
    fn value(&self, params: &BTreeMap<String, mp_core::link::Param>, name: &str) -> Option<f32> {
        self.changes.get(name).copied().or_else(|| params.get(name).map(|p| p.value))
    }

    fn set(&mut self, params: &BTreeMap<String, mp_core::link::Param>, name: &str, v: f32) {
        if params.get(name).map(|p| p.value) == Some(v) {
            self.changes.remove(name);
        } else {
            self.changes.insert(name.to_string(), v);
        }
    }

    fn font(&mut self, ctx: &egui::Context) -> egui::TextureId {
        self.font
            .get_or_insert_with(|| {
                let img = image::load_from_memory(include_bytes!("../../assets/osd_clarity.png")).map(|i| i.to_rgba8()).unwrap_or_default();
                let ci = egui::ColorImage::from_rgba_unmultiplied([img.width() as usize, img.height() as usize], img.as_raw());
                ctx.load_texture("osd_clarity", ci, egui::TextureOptions::NEAREST)
            })
            .id()
    }

    fn write(&mut self, link: &Link, app: &mut AppCtx) {
        if app.readonly {
            return;
        }
        let n = self.changes.len();
        for (k, v) in std::mem::take(&mut self.changes) {
            link.send(LinkCommand::SetParam { name: k, value: v });
        }
        app.toasts.error(format!("{n} parameters written"));
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let Some(link) = app.link.filter(|_| app.connected()) else {
            theme::label(ui, "Please Connect First");
            return;
        };
        let params = link.shared.read().params.clone();
        let screens: Vec<usize> = (1..=4).filter(|n| params.contains_key(&format!("OSD{n}_ENABLE"))).collect();
        if screens.is_empty() {
            theme::label(ui, "This vehicle has no onboard OSD parameters (OSD_TYPE)");
            return;
        }
        if !screens.contains(&self.screen) {
            self.screen = screens[0];
        }
        let o = ui.max_rect().min;
        // panel1: 182 wide, docked right (x=957 at the 1140px design size)
        let px = (ui.max_rect().right() - 182.0).max(o.x + 912.0).min(o.x + 957.0);
        let btn = |ui: &mut egui::Ui, y: f32, h: f32, t: &str| {
            let r = Rect::from_min_size(Pos2::new(px + 20.0, o.y + 8.0 + y), Vec2::new(145.0, h));
            ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, t, r.size())).clicked()
        };
        if btn(ui, 23.0, 32.0, "Write customization") {
            self.write(link, app);
        }
        ui.put(Rect::from_min_size(Pos2::new(px + 24.0, o.y + 8.0 + 63.0), Vec2::new(145.0, 17.0)), egui::Checkbox::new(&mut self.auto_write, "Auto write on leaving"));
        if btn(ui, 112.0, 42.0, "OSD / Telmetry Slots Config") {
            self.screen = 5.min(*screens.last().unwrap_or(&1));
        }
        if btn(ui, 197.0, 32.0, "Refresh") {
            link.send(LinkCommand::RequestParams);
        }
        if btn(ui, 235.0, 32.0, "Discard all changes") {
            self.changes.clear();
        }
        // tabControl: one tab per screen plus Settings (tab 0)
        let mut labels: Vec<String> = screens.iter().map(|n| format!("   Screen {n}   ")).collect();
        labels.push("   Settings   ".into());
        let mut tab = screens.iter().position(|s| *s == self.screen).unwrap_or(0);
        if self.screen == 0 {
            tab = labels.len() - 1;
        }
        let label_refs: Vec<&str> = labels.iter().map(|s| s.as_str()).collect();
        ui.scope_builder(egui::UiBuilder::new().max_rect(Rect::from_min_size(o, Vec2::new(912.0, 36.0))), |ui| theme::tab_strip(ui, "osd_tabs", &label_refs, &mut tab));
        self.screen = if tab + 1 == labels.len() { 0 } else { screens[tab] };
        let body = Rect::from_min_size(o + Vec2::new(4.0, 39.0), Vec2::new(904.0, 662.0));
        if self.screen == 0 {
            ui.scope_builder(egui::UiBuilder::new().max_rect(body), |ui| {
                crate::views::config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("OSD_"));
            });
            return;
        }
        let sp = format!("OSD{}_", self.screen);
        let items: Vec<String> = params.keys().filter_map(|k| k.strip_prefix(&sp).and_then(|r| r.strip_suffix("_EN")).map(|s| s.to_string())).collect();

        // LayoutControl: 696x540 canvas
        let cw = 696.0 / COLS as f32;
        let ch = 540.0 / ROWS as f32;
        let canvas = Rect::from_min_size(body.min + Vec2::new(3.0, 3.0), Vec2::new(cw * COLS as f32, ch * ROWS as f32));
        let resp = ui.interact(canvas, ui.scope_id().with("osd_canvas"), Sense::click_and_drag());
        let p = ui.painter().with_clip_rect(canvas);
        p.rect_filled(canvas, 0.0, Color32::from_rgb(0x55, 0x6b, 0x7a));
        let ny = canvas.top() + ch * NTSC_ROWS as f32;
        p.line_segment([Pos2::new(canvas.left(), ny), Pos2::new(canvas.right(), ny)], Stroke::new(1.0, Color32::DARK_GRAY));
        p.text(Pos2::new(canvas.left() + 6.0, ny - 12.0), egui::Align2::LEFT_TOP, "NTSC", egui::FontId::proportional(8.0), Color32::BLACK);
        p.text(Pos2::new(canvas.left() + 6.0, ny + 3.0), egui::Align2::LEFT_TOP, "PAL", egui::FontId::proportional(8.0), Color32::BLACK);
        let font = self.font(ui.ctx());
        let mut hits: Vec<(String, Rect)> = Vec::new();
        for it in &items {
            if self.value(&params, &format!("{sp}{it}_EN")).unwrap_or(0.0) == 0.0 {
                continue;
            }
            let x = self.value(&params, &format!("{sp}{it}_X")).unwrap_or(0.0) as i32;
            let y = self.value(&params, &format!("{sp}{it}_Y")).unwrap_or(0.0) as i32;
            let text = if self.name_captions { it.clone() } else { caption(it) };
            let start = Pos2::new(canvas.left() + x as f32 * cw, canvas.top() + y as f32 * ch);
            for (k, c) in text.bytes().enumerate() {
                let (col, row) = ((c % 16) as f32, (c / 16) as f32);
                let uv = Rect::from_min_size(Pos2::new(col * 13.0 / 207.0, row * 19.0 / 303.0), Vec2::new(12.0 / 207.0, 18.0 / 303.0));
                p.image(font, Rect::from_min_size(start + Vec2::new(k as f32 * cw, 0.0), Vec2::new(cw, ch)), uv, Color32::WHITE);
            }
            let r = Rect::from_min_size(start, Vec2::new(text.len() as f32 * cw, ch));
            if self.selected.as_deref() == Some(it.as_str()) {
                p.rect_stroke(r.expand(5.0), 0.0, Stroke::new(1.0, Color32::YELLOW), StrokeKind::Outside);
            }
            hits.push((it.clone(), r));
        }
        if resp.drag_started() || resp.clicked() {
            if let Some(pos) = resp.interact_pointer_pos() {
                if let Some((n, r)) = hits.iter().rev().find(|(_, r)| r.contains(pos)) {
                    self.selected = Some(n.clone());
                    self.drag = Some((n.clone(), pos - r.min));
                }
            }
        }
        if resp.dragged() {
            if let (Some((n, off)), Some(pos)) = (self.drag.clone(), resp.interact_pointer_pos()) {
                let tl = pos - off - canvas.min;
                let x = ((tl.x / cw).round() as i32).clamp(0, COLS - 1);
                let y = ((tl.y / ch).round() as i32).clamp(0, ROWS - 1);
                self.set(&params, &format!("{sp}{n}_X"), x as f32);
                self.set(&params, &format!("{sp}{n}_Y"), y as f32);
            }
        }
        if resp.drag_stopped() {
            self.drag = None;
        }

        // tableRight: Editor Options / Screen Options / item list / Item Options
        let rx = canvas.right() + 14.0;
        let rw = (body.right() - rx - 4.0).max(200.0);
        let rp = ui.painter().clone();
        crate::views::tuning::group_box(&rp, Rect::from_min_size(Pos2::new(rx, body.top()), Vec2::new(rw, 70.0)), "Editor Options");
        ui.put(Rect::from_min_size(Pos2::new(rx + 23.0, body.top() + 30.0), Vec2::new(rw - 30.0, 17.0)), egui::Checkbox::new(&mut self.name_captions, "Use item names as captions"));
        crate::views::tuning::group_box(&rp, Rect::from_min_size(Pos2::new(rx, body.top() + 76.0), Vec2::new(rw, 90.0)), "Screen Options");
        for (k, opt) in ["ENABLE", "CHAN_MIN", "CHAN_MAX"].iter().enumerate() {
            let n = format!("{sp}{opt}");
            if let Some(v) = self.value(&params, &n) {
                let y = body.top() + 94.0 + k as f32 * 22.0;
                rp.text(Pos2::new(rx + 8.0, y + 3.0), egui::Align2::LEFT_TOP, *opt, crate::fonts::ui_id(crate::fonts::UI_SIZE), theme::text());
                let mut nv = v;
                if *opt == "ENABLE" {
                    let mut on = v != 0.0;
                    if ui.put(Rect::from_min_size(Pos2::new(rx + 100.0, y), Vec2::new(20.0, 18.0)), egui::Checkbox::without_text(&mut on)).changed() {
                        nv = if on { 1.0 } else { 0.0 };
                    }
                } else {
                    ui.put(Rect::from_min_size(Pos2::new(rx + 100.0, y), Vec2::new(70.0, 18.0)), egui::DragValue::new(&mut nv).range(0.0..=2200.0));
                }
                if nv != v {
                    self.set(&params, &n, nv);
                }
            }
        }
        let list = Rect::from_min_max(Pos2::new(rx, body.top() + 172.0), Pos2::new(rx + rw, body.top() + 172.0 + 260.0));
        rp.rect_filled(list, 0.0, theme::control_bg());
        let mut sel = None;
        let mut toggles = Vec::new();
        ui.scope_builder(egui::UiBuilder::new().max_rect(list.shrink(3.0)), |ui| {
            egui::ScrollArea::vertical().id_salt("osd_items").auto_shrink([false, false]).show(ui, |ui| {
                for it in &items {
                    let en_name = format!("{sp}{it}_EN");
                    let mut on = self.value(&params, &en_name).unwrap_or(0.0) != 0.0;
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut on, "").changed() {
                            toggles.push((en_name.clone(), on));
                        }
                        let is_sel = self.selected.as_deref() == Some(it.as_str());
                        if ui.selectable_label(is_sel, it.as_str()).clicked() {
                            sel = Some(it.clone());
                        }
                    });
                }
            });
        });
        for (n, on) in toggles {
            self.set(&params, &n, if on { 1.0 } else { 0.0 });
        }
        if sel.is_some() {
            self.selected = sel;
        }
        let gy = list.bottom() + 6.0;
        crate::views::tuning::group_box(&rp, Rect::from_min_size(Pos2::new(rx, gy), Vec2::new(rw, 70.0)), "Item Options");
        if let Some(it) = self.selected.clone() {
            for (k, axis) in ["X", "Y"].iter().enumerate() {
                let n = format!("{sp}{it}_{axis}");
                if let Some(v) = self.value(&params, &n) {
                    let y = gy + 20.0 + k as f32 * 22.0;
                    rp.text(Pos2::new(rx + 8.0, y + 3.0), egui::Align2::LEFT_TOP, *axis, crate::fonts::ui_id(crate::fonts::UI_SIZE), theme::text());
                    let mut nv = v;
                    let max = if k == 0 { COLS - 1 } else { ROWS - 1 } as f32;
                    ui.put(Rect::from_min_size(Pos2::new(rx + 30.0, y), Vec2::new(60.0, 18.0)), egui::DragValue::new(&mut nv).range(0.0..=max));
                    if nv != v {
                        self.set(&params, &n, nv);
                    }
                }
            }
        }
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(px + 182.0 - o.x, 705.0)), Sense::hover());
    }

    /// cbAutoWriteOnLeave: called when the page is left.
    pub fn leave(&mut self, app: &mut AppCtx) {
        if self.auto_write && !self.changes.is_empty() {
            if let Some(l) = app.link {
                self.write(l, app);
            }
        }
    }
}
