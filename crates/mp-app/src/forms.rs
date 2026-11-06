//! Generic renderer for WinForms designer pages exported to JSON (`assets/forms/*.json`,
//! produced from each page's .resx / .Designer.cs / .cs): absolute layout, group boxes,
//! labels, pictures, and MAVLink-bound check boxes / combo boxes / numeric boxes. Page code
//! handles the buttons and reads or writes the unbound controls through [`FormState`].

use std::collections::HashMap;
use std::sync::OnceLock;

use egui::{Color32, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use mp_core::link::{Link, LinkCommand, Param};

use crate::app::AppCtx;
use crate::{fonts, theme};

#[derive(serde::Deserialize, Clone)]
pub struct Ctl {
    pub name: String,
    pub kind: String,
    pub parent: String,
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
    #[serde(default)]
    pub text: Option<String>,
    #[serde(default)]
    pub hidden: bool,
    #[serde(default)]
    pub font: Option<f32>,
    #[serde(default)]
    pub bold: bool,
    #[serde(default)]
    pub param: Option<String>,
    #[serde(default)]
    pub nums: Vec<f64>,
    #[serde(default)]
    pub image: Option<String>,
}

#[derive(serde::Deserialize)]
pub struct Form {
    pub name: String,
    pub w: f32,
    pub h: f32,
    pub controls: Vec<Ctl>,
}

macro_rules! forms {
    ($($n:literal),*) => {
        &[$(($n, include_str!(concat!("../assets/forms/", $n, ".json")))),*]
    };
}

const FORMS: &[(&str, &str)] = forms!(
    "rangefinder", "airspeed", "optflow", "mount", "parachute", "tracker", "bluetooth", "esp8266", "cubeid", "dronecan", "repl", "initialparams", "compassmot", "serial", "sikradio", "osd", "adsb", "gpsorder", "hwosd", "georef", "fwdisabled", "paramloading", "secureap", "tradheli", "trackerui", "rtkinject", "devops", "gmapcache"
);

const IMAGES: &[(&str, &[u8])] = &[
    ("Parachute.png", include_bytes!("../assets/forms/Parachute.png")),
    ("Shutter.png", include_bytes!("../assets/forms/Shutter.png")),
    ("airspeed.jpg", include_bytes!("../assets/forms/airspeed.jpg")),
    ("cameraGimalPitch1.png", include_bytes!("../assets/forms/cameraGimalPitch1.png")),
    ("cameraGimalRoll1.png", include_bytes!("../assets/forms/cameraGimalRoll1.png")),
    ("cameraGimalYaw.png", include_bytes!("../assets/forms/cameraGimalYaw.png")),
    ("opticalflow.jpg", include_bytes!("../assets/forms/opticalflow.jpg")),
    ("sonar.jpg", include_bytes!("../assets/forms/sonar.jpg")),
    ("MinimOSD.jpg", include_bytes!("../assets/forms/MinimOSD.jpg")),
];

pub fn form(name: &str) -> &'static Form {
    static CACHE: OnceLock<HashMap<&'static str, Form>> = OnceLock::new();
    let all = CACHE.get_or_init(|| {
        FORMS
            .iter()
            .filter_map(|(n, j)| match serde_json::from_str::<Form>(j) {
                Ok(f) => Some((*n, f)),
                Err(e) => {
                    log::error!("form {n}: {e}");
                    None
                }
            })
            .collect()
    });
    &all[name]
}

fn texture(ctx: &egui::Context, name: &str) -> Option<egui::TextureHandle> {
    let id = egui::Id::new(("form_img", name));
    if let Some(t) = ctx.data(|d| d.get_temp::<egui::TextureHandle>(id)) {
        return Some(t);
    }
    let bytes = IMAGES.iter().find(|(n, _)| *n == name)?.1;
    let img = image::load_from_memory(bytes).ok()?.to_rgba8();
    let ci = egui::ColorImage::from_rgba_unmultiplied([img.width() as usize, img.height() as usize], img.as_raw());
    let t = ctx.load_texture(name, ci, egui::TextureOptions::LINEAR);
    ctx.data_mut(|d| d.insert_temp(id, t.clone()));
    Some(t)
}

/// Parameter name as written by Mission Planner, or the numbered 4.x name ("RNGFND_TYPE" ->
/// "RNGFND1_TYPE", "MNT_TYPE" -> "MNT1_TYPE").
pub fn resolve(params: &std::collections::BTreeMap<String, Param>, name: &str) -> Option<String> {
    if name.starts_with('_') || name.is_empty() {
        return None;
    }
    if params.contains_key(name) {
        return Some(name.to_string());
    }
    let (head, tail) = name.split_once('_')?;
    let alt = format!("{head}1_{tail}");
    params.contains_key(&alt).then_some(alt)
}

/// Values of the controls that are not bound to a parameter (text boxes, combos, checks).
#[derive(Default)]
pub struct FormState {
    pub text: HashMap<String, String>,
    pub checked: HashMap<String, bool>,
    pub items: HashMap<String, Vec<String>>,
    pub progress: HashMap<String, f32>,
    pub disabled: std::collections::HashSet<String>,
    /// Track bar ranges (default 0..100).
    pub ranges: HashMap<String, (f64, f64)>,
    /// Runtime Visible overrides (true shows a designer-hidden control, false hides one).
    pub visible: HashMap<String, bool>,
}

impl FormState {
    pub fn get(&self, ctl: &str) -> &str {
        self.text.get(ctl).map(|s| s.as_str()).unwrap_or("")
    }

    pub fn set(&mut self, ctl: &str, v: impl Into<String>) {
        self.text.insert(ctl.to_string(), v.into());
    }
}

fn origin_of(f: &Form, ctl: &Ctl) -> Vec2 {
    let mut off = Vec2::ZERO;
    let mut p = ctl.parent.as_str();
    let mut guard = 0;
    while p != "$this" && guard < 10 {
        guard += 1;
        match f.controls.iter().find(|c| c.name == p) {
            Some(c) => {
                off += Vec2::new(c.x, c.y);
                p = c.parent.as_str();
            }
            None => break,
        }
    }
    off
}

fn hidden(f: &Form, ctl: &Ctl, st: &FormState) -> bool {
    let off = |c: &Ctl| st.visible.get(&c.name).map(|v| !*v).unwrap_or(c.hidden);
    // a 0x0 control (AESKEY) has no visible area in WinForms
    if off(ctl) || (ctl.w == 0.0 && ctl.h == 0.0 && ctl.kind != "label") {
        return true;
    }
    let mut p = ctl.parent.as_str();
    while p != "$this" {
        match f.controls.iter().find(|c| c.name == p) {
            Some(c) if off(c) => return true,
            Some(c) => p = c.parent.as_str(),
            None => break,
        }
    }
    false
}

/// Where `ctl` sits relative to the form origin (offset, size).
pub fn ctl_rect(form_name: &str, ctl: &str) -> Option<(Vec2, Vec2)> {
    let f = form(form_name);
    let c = f.controls.iter().find(|c| c.name == ctl)?;
    Some((origin_of(f, c) + Vec2::new(c.x, c.y), Vec2::new(c.w, c.h)))
}

fn font_for(c: &Ctl) -> egui::FontId {
    let px = c.font.map(|pt| pt * 96.0 / 72.0).unwrap_or(fonts::UI_SIZE);
    if c.bold {
        fonts::bold_id(px)
    } else {
        fonts::ui_id(px)
    }
}

/// Draw `name` at the top left of the available area. Returns the buttons clicked this frame.
pub fn show(ui: &mut egui::Ui, app: &mut AppCtx, name: &str, st: &mut FormState) -> Vec<String> {
    show_impl(ui, app, name, st, None)
}

/// Like [`show`] but reserves only `height` px, so page code can lay out below the header.
pub fn show_fixed(ui: &mut egui::Ui, app: &mut AppCtx, name: &str, st: &mut FormState, height: f32) -> Vec<String> {
    show_impl(ui, app, name, st, Some(height))
}

fn show_impl(ui: &mut egui::Ui, app: &mut AppCtx, name: &str, st: &mut FormState, alloc_h: Option<f32>) -> Vec<String> {
    let f = form(name);
    let o = ui.available_rect_before_wrap().min;
    let p = ui.painter().clone();
    let link: Option<&Link> = app.link;
    let (params, fw) = link.map(|l| {
        let s = l.shared.read();
        (s.params.clone(), s.cs.firmware)
    }).unwrap_or_default();
    let mut clicked = Vec::new();
    let mut maxy: f32 = 0.0;
    for c in &f.controls {
        if hidden(f, c, st) {
            continue;
        }
        let at = o + origin_of(f, c) + Vec2::new(c.x, c.y);
        let r = Rect::from_min_size(at, Vec2::new(c.w.max(1.0), c.h.max(1.0)));
        maxy = maxy.max(r.bottom() - o.y);
        let text = c.text.clone().unwrap_or_default();
        let bound = c.param.as_deref().and_then(|n| resolve(&params, n));
        let enabled = !st.disabled.contains(&c.name);
        match c.kind.as_str() {
            "label" => {
                let t = st.text.get(&c.name).cloned().unwrap_or(text);
                // a one-line label is AutoSize: it grows instead of wrapping
                let wrap = if c.h <= 16.0 || c.w <= 0.0 { f32::INFINITY } else { c.w.max(20.0) + 40.0 };
                let g = p.layout(t, font_for(c), Color32::WHITE, wrap);
                p.galley(at, g, Color32::WHITE);
            }
            "link" => {
                p.text(at, egui::Align2::LEFT_TOP, text, font_for(c), Color32::from_rgb(0x00, 0x66, 0xcc));
            }
            "group" => {
                if c.h <= 6.0 {
                    p.line_segment([Pos2::new(r.left(), r.top() + 2.0), Pos2::new(r.right(), r.top() + 2.0)], Stroke::new(1.0, Color32::from_rgb(0xd5, 0xdf, 0xe5)));
                } else {
                    crate::views::tuning::group_box(&p, r, &text);
                }
            }
            "picture" => {
                if let Some(t) = c.image.as_deref().and_then(|n| texture(ui.ctx(), n)) {
                    let sz = t.size_vec2();
                    let s = (r.width() / sz.x).min(r.height() / sz.y);
                    let ir = Rect::from_center_size(r.center(), sz * s);
                    p.image(t.id(), ir, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
                }
            }
            "mcheck" => {
                let on_v = c.nums.first().copied().unwrap_or(1.0) as f32;
                let off_v = c.nums.get(1).copied().unwrap_or(0.0) as f32;
                let cur = bound.as_ref().map(|n| params[n].value);
                let mut on = cur.map(|v| v == on_v).unwrap_or(false);
                let resp = ui.put(r, |ui: &mut egui::Ui| ui.add_enabled(cur.is_some(), egui::Checkbox::new(&mut on, &text)));
                if resp.changed() && !app.readonly {
                    if let (Some(l), Some(n)) = (link, &bound) {
                        l.send(LinkCommand::SetParam { name: n.clone(), value: if on { on_v } else { off_v } });
                    }
                }
            }
            "mcombo" => match (&bound, link) {
                (Some(n), Some(l)) => {
                    let v = params[n].value;
                    ui.scope_builder(egui::UiBuilder::new().max_rect(r), |ui| crate::views::config::param_control(ui, l, fw, n, v, r.width() - 8.0));
                }
                _ => {
                    ui.put(r, |ui: &mut egui::Ui| ui.add_enabled_ui(false, |ui| egui::ComboBox::from_id_salt((name, &c.name)).width(r.width() - 8.0).selected_text("").show_ui(ui, |_| {})).response);
                }
            },
            "mnum" => {
                let scale = c.nums.get(2).copied().filter(|s| *s != 0.0).unwrap_or(1.0);
                let (lo, hi) = (c.nums.first().copied().unwrap_or(f64::MIN), c.nums.get(1).copied().unwrap_or(f64::MAX));
                match &bound {
                    Some(n) => {
                        let cur = params[n].value as f64;
                        let mut v = cur / scale;
                        let inc = c.nums.get(3).copied().unwrap_or(1.0);
                        let resp = ui.put(r, egui::DragValue::new(&mut v).range(lo..=hi).speed(inc.max(0.01)));
                        if (resp.drag_stopped() || resp.lost_focus()) && (v * scale - cur).abs() > 1e-9 && !app.readonly {
                            if let Some(l) = link {
                                l.send(LinkCommand::SetParam { name: n.clone(), value: (v * scale) as f32 });
                            }
                        }
                    }
                    None => {
                        let mut s = String::new();
                        ui.put(r, |ui: &mut egui::Ui| ui.add_enabled(false, egui::TextEdit::singleline(&mut s)));
                    }
                }
            }
            "check" | "radio" => {
                let on = st.checked.entry(c.name.clone()).or_insert(false);
                let resp = if c.kind == "radio" {
                    ui.put(r, |ui: &mut egui::Ui| ui.add_enabled(enabled, egui::RadioButton::new(*on, &text)))
                } else {
                    ui.put(r, |ui: &mut egui::Ui| ui.add_enabled(enabled, egui::Checkbox::new(on, &text)))
                };
                if resp.clicked() {
                    if c.kind == "radio" {
                        *on = true;
                    }
                    clicked.push(c.name.clone());
                }
            }
            "combo" => {
                let items = st.items.get(&c.name).cloned().unwrap_or_default();
                let cur = st.text.entry(c.name.clone()).or_default();
                let before = cur.clone();
                ui.put(r, |ui: &mut egui::Ui| {
                    ui.add_enabled_ui(enabled, |ui| {
                        egui::ComboBox::from_id_salt((name, &c.name))
                            .width(r.width() - 8.0)
                            .wrap_mode(egui::TextWrapMode::Truncate)
                            .selected_text(cur.as_str())
                            .show_ui(ui, |ui| {
                                for it in &items {
                                    ui.selectable_value(cur, it.clone(), it);
                                }
                            })
                            .response
                    })
                    .inner
                });
                if *cur != before {
                    clicked.push(c.name.clone());
                }
            }
            "num" | "text" => {
                let v = st.text.entry(c.name.clone()).or_insert_with(|| text.clone());
                if c.h > 30.0 && c.kind == "text" {
                    ui.put(r, |ui: &mut egui::Ui| ui.add_enabled(enabled, egui::TextEdit::multiline(v).font(egui::TextStyle::Monospace)));
                } else {
                    ui.put(r, |ui: &mut egui::Ui| ui.add_enabled(enabled, egui::TextEdit::singleline(v)));
                }
            }
            "button" => {
                let text = st.text.get(&c.name).cloned().unwrap_or(text);
                let resp = ui.put(r, |ui: &mut egui::Ui| ui.add_enabled_ui(enabled, |ui| theme::button_sized(ui, &text, r.size())).inner);
                if resp.clicked() {
                    clicked.push(c.name.clone());
                }
            }
            "progress" | "vprogress" => {
                let v = st.progress.get(&c.name).copied().unwrap_or(0.0);
                ui.put(r, |ui: &mut egui::Ui| {
                    theme::progress(ui, v, r.size(), "");
                    ui.allocate_response(Vec2::ZERO, Sense::hover())
                });
            }
            "track" => {
                let (lo, hi) = st.ranges.get(&c.name).copied().unwrap_or((0.0, 100.0));
                let mut v = st.text.get(&c.name).and_then(|s| s.parse::<f64>().ok()).unwrap_or(lo.max(0.0).min(hi));
                if enabled && theme::htrackbar(ui, r, &mut v, lo, hi, 10).changed() {
                    st.text.insert(c.name.clone(), format!("{}", v.round()));
                    clicked.push(c.name.clone());
                }
            }
            "grid" | "list" | "graph" => {
                p.rect_filled(r, 0.0, if c.kind == "graph" { Color32::from_rgb(0x37, 0x37, 0x38) } else { theme::control_bg() });
                p.rect_stroke(r, 0.0, Stroke::new(1.0, Color32::from_gray(0x80)), StrokeKind::Inside);
                if let Some(t) = st.text.get(&c.name) {
                    let g = p.layout(t.clone(), egui::FontId::monospace(11.0), Color32::WHITE, r.width() - 6.0);
                    p.with_clip_rect(r).galley(r.min + Vec2::new(3.0, 3.0), g, Color32::WHITE);
                }
            }
            _ => {}
        }
    }
    let h = alloc_h.unwrap_or(maxy.max(f.h.min(maxy + 10.0)));
    ui.allocate_rect(Rect::from_min_size(o, Vec2::new(f.w, h)), Sense::hover());
    clicked
}
