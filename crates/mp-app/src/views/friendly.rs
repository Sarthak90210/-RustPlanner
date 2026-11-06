//! ConfigFriendlyParams (Standard / Advanced Params): Write Params, Refresh Params, Find and
//! "Advanced Menu View" above a list of RangeControl / MavlinkCheckBoxBitMask / ValuesControl
//! blocks. Edits are held until Write Params, as in Mission Planner.

use std::collections::BTreeMap;

use egui::{Color32, Pos2, Rect, Sense, Vec2};
use mp_core::firmware::Firmware;
use mp_core::link::{Link, LinkCommand};
use mp_core::param_meta::{self, ParamMeta};

use crate::app::AppCtx;
use crate::{fonts, theme};

#[derive(Default)]
pub struct FriendlyParams {
    pending: BTreeMap<String, f32>,
    search: String,
    search_prompt: Option<String>,
    refresh_confirm: bool,
}

enum Kind {
    /// RangeControl: (min, max, increment, display scale)
    Range(f64, f64, f64, f64),
    Bitmask,
    Values,
}

fn kind(m: &ParamMeta) -> Option<Kind> {
    // AddControl: range (with an increment) first, then bitmask, then values
    if let (Some((lo, hi)), Some(inc)) = (m.range, m.increment.filter(|i| *i > 0.0)) {
        let scale = if m.units.eq_ignore_ascii_case("centi-degrees") { 100.0 } else { 1.0 };
        return Some(Kind::Range(lo, hi, inc, scale));
    }
    if !m.bitmask.is_empty() {
        return Some(Kind::Bitmask);
    }
    if !m.values.is_empty() {
        return Some(Kind::Values);
    }
    None
}

/// FitDescriptionText: "Units: x" line, then "Description: " with a break every width/40 words.
fn fit_description(m: &ParamMeta, width: f32) -> String {
    let mut out = String::new();
    let units = if m.units.eq_ignore_ascii_case("centi-degrees") { "Degrees (Scaled)" } else { m.units.as_str() };
    if !units.is_empty() {
        out.push_str(&format!("Units: {units}\n"));
    }
    if !m.description.is_empty() {
        out.push_str("Description: ");
        let every = ((width / 40.0) as usize).max(1);
        for (i, w) in m.description.split(' ').enumerate() {
            out.push_str(w);
            out.push(' ');
            if i != 0 && i % every == 0 {
                out.push('\n');
            }
        }
    }
    out
}

/// MavlinkCheckBoxBitMask: checkbox positions (x, y, label) and the panel height.
fn bitmask_layout(ui: &egui::Ui, m: &ParamMeta) -> (Vec<(f32, f32, u32, String)>, f32) {
    let f = fonts::ui_id(fonts::UI_SIZE);
    let (mut left, mut top, mut bottom) = (9.0f32, 9.0f32, 0.0f32);
    let mut out = Vec::new();
    for (bit, name) in &m.bitmask {
        let w = ui.painter().layout_no_wrap(name.clone(), f.clone(), Color32::WHITE).size().x + 20.0;
        out.push((left, top, *bit, name.clone()));
        bottom = top + 17.0;
        left += w + 5.0;
        if left > 500.0 {
            top += 17.0 + 5.0;
            left = 9.0;
        }
    }
    (out, bottom)
}

fn lines(s: &str) -> usize {
    s.lines().count().max(1)
}

fn row_height(ui: &egui::Ui, m: &ParamMeta, width: f32) -> f32 {
    match kind(m) {
        Some(Kind::Range(..)) => 108.0,
        Some(Kind::Values) => 89.0,
        Some(Kind::Bitmask) => {
            let desc_h = lines(&fit_description(m, width)) as f32 * 13.0;
            let (_, panel) = bitmask_layout(ui, m);
            23.0 + desc_h + 8.0 + panel + 25.0
        }
        None => 0.0,
    }
}

/// Send `value` now, or hold it for Write Params.
fn change(link: &Link, pending: &mut Option<&mut BTreeMap<String, f32>>, name: &str, value: f32) {
    match pending {
        Some(p) => {
            p.insert(name.to_string(), value);
        }
        None => link.send(LinkCommand::SetParam { name: name.into(), value }),
    }
}

/// The parameter blocks for `names` (sorted), each in its Mission Planner control.
pub fn rows(ui: &mut egui::Ui, app: &mut AppCtx, names: &[String], mut pending: Option<&mut BTreeMap<String, f32>>) {
    let Some(link) = app.link else {
        theme::label(ui, "Please Connect First");
        return;
    };
    let (params, fw): (BTreeMap<String, f32>, Firmware) = {
        let s = link.shared.read();
        (s.params.iter().map(|(k, p)| (k.clone(), p.value)).collect(), s.cs.firmware)
    };
    let width = (ui.available_width() - 50.0).max(365.0);
    let list: Vec<(String, &'static ParamMeta, f32)> = names
        .iter()
        .filter_map(|n| {
            let m = param_meta::get(fw, n)?;
            kind(m)?;
            Some((n.clone(), m, row_height(ui, m, width)))
        })
        .collect();
    if list.is_empty() {
        theme::label(ui, if params.is_empty() { "Loading..." } else { "No parameters for this page on this vehicle" });
        return;
    }
    let total: f32 = list.iter().map(|r| r.2 + 6.0).sum();
    egui::ScrollArea::vertical().auto_shrink([false, false]).show_viewport(ui, |ui, view| {
        let (area, _) = ui.allocate_exact_size(Vec2::new(width, total), Sense::hover());
        let mut y = 0.0;
        for (n, m, h) in &list {
            let top = y;
            y += h + 6.0;
            if top + h < view.min.y || top > view.max.y {
                continue;
            }
            let r = Rect::from_min_size(Pos2::new(area.left(), area.top() + top), Vec2::new(width, *h));
            let cur = pending.as_ref().and_then(|p| p.get(n).copied()).unwrap_or_else(|| params.get(n).copied().unwrap_or(0.0));
            let changed = pending.as_ref().map(|p| p.contains_key(n)).unwrap_or(false);
            block(ui, link, &mut pending, r, n, m, cur, changed);
        }
    });
}

#[allow(clippy::too_many_arguments)]
fn block(ui: &mut egui::Ui, link: &Link, pending: &mut Option<&mut BTreeMap<String, f32>>, r: Rect, n: &str, m: &ParamMeta, cur: f32, changed: bool) {
    let p = ui.painter().clone();
    let white = theme::text();
    let bold = fonts::bold_id(13.0);
    let f = fonts::ui_id(fonts::UI_SIZE);
    let title = if m.display_name.is_empty() { n.to_string() } else { m.display_name.clone() };
    let at = |x: f32, y: f32| r.min + Vec2::new(x, y);
    let desc = fit_description(m, r.width());
    let resp = ui.interact(r, ui.id().with(("friendly", n)), Sense::hover());
    resp.on_hover_text(n);
    match kind(m) {
        Some(Kind::Range(lo, hi, inc, scale)) => {
            p.text(at(3.0, 0.0), egui::Align2::LEFT_TOP, &title, bold, white);
            let g = p.layout(desc, f.clone(), white, r.width());
            p.with_clip_rect(Rect::from_min_size(at(3.0, 15.0), Vec2::new(r.width(), 39.0))).galley(at(3.0, 15.0), g, white);
            // numericUpDown1 57x20 at (6, 58), orange when outside the documented range
            let mut v = cur as f64 / scale;
            let out = (cur as f64) < lo || (cur as f64) > hi;
            let nr = Rect::from_min_size(at(6.0, 58.0), Vec2::new(57.0, 20.0));
            if out {
                p.rect_filled(nr, 0.0, Color32::from_rgb(0xff, 0xa5, 0x00));
            }
            let nresp = theme::numeric_updown(ui, nr, &mut v, (lo / scale).min(cur as f64 / scale), (hi / scale).max(cur as f64 / scale), inc / scale, 3);
            if nresp.changed() && (v * scale - cur as f64).abs() > 1e-9 {
                change(link, pending, n, (v * scale) as f32);
            }
            // trackBar1 at (69, 58), anchored left and right
            let tr = Rect::from_min_size(at(69.0, 58.0), Vec2::new(r.width() - 72.0, 30.0));
            let drag_id = ui.id().with(("ftrk", n));
            let mut t = ui.data(|d| d.get_temp::<f64>(drag_id)).unwrap_or(cur as f64);
            let tresp = theme::htrackbar(ui, tr, &mut t, lo, hi, 10);
            if tresp.dragged() {
                ui.data_mut(|d| d.insert_temp(drag_id, t));
            }
            if tresp.drag_stopped() || tresp.clicked() {
                ui.data_mut(|d| d.remove::<f64>(drag_id));
                // snap to the increment
                let snapped = lo + ((t - lo) / inc).round() * inc;
                if (snapped - cur as f64).abs() > 1e-9 {
                    change(link, pending, n, snapped as f32);
                }
            }
            p.text(at(72.0, 90.0), egui::Align2::LEFT_TOP, trim(lo / scale), f.clone(), white);
            p.text(at(r.width() - 3.0, 103.0), egui::Align2::RIGHT_BOTTOM, trim(hi / scale), f, white);
        }
        Some(Kind::Values) => {
            p.text(at(4.0, 3.0), egui::Align2::LEFT_TOP, &title, bold, white);
            let g = p.layout(desc, f.clone(), white, r.width() - 6.0);
            p.with_clip_rect(Rect::from_min_size(at(3.0, 19.0), Vec2::new(r.width() - 6.0, 43.0))).galley(at(3.0, 19.0), g, white);
            let cr = Rect::from_min_size(at(3.0, 65.0), Vec2::new(207.0, 21.0));
            let sel = m.values.iter().find(|(k, _)| *k as f32 == cur).map(|(_, s)| s.clone()).unwrap_or_else(|| mp_core::paramfile::format_value(cur));
            let mut pick = None;
            ui.put(cr, |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt(("fv", n)).width(199.0).selected_text(sel).show_ui(ui, |ui| {
                    for (k, s) in &m.values {
                        if ui.selectable_label(*k as f32 == cur, s).clicked() {
                            pick = Some(*k as f32);
                        }
                    }
                }).response
            });
            if let Some(v) = pick {
                change(link, pending, n, v);
            }
        }
        Some(Kind::Bitmask) => {
            p.text(at(3.0, 3.0), egui::Align2::LEFT_TOP, &title, bold, white);
            let g = p.layout(desc.clone(), f.clone(), white, r.width() - 6.0);
            p.galley(at(6.0, 33.0), g, white);
            let desc_h = lines(&desc) as f32 * 13.0;
            let (boxes, _) = bitmask_layout(ui, m);
            let mut value = cur as u32;
            let mut any = false;
            for (x, y, bit, label) in boxes {
                let mut on = value & (1 << bit) != 0;
                let w = ui.painter().layout_no_wrap(label.clone(), f.clone(), white).size().x + 20.0;
                let br = Rect::from_min_size(at(6.0 + x, 28.0 + desc_h + 8.0 + y), Vec2::new(w, 17.0));
                if ui.put(br, egui::Checkbox::new(&mut on, &label)).changed() {
                    value = if on { value | (1 << bit) } else { value & !(1 << bit) };
                    any = true;
                }
            }
            if any {
                change(link, pending, n, value as f32);
            }
        }
        None => {}
    }
    if changed {
        p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::from_rgb(0xff, 0xa5, 0x00)), egui::StrokeKind::Inside);
    }
}

fn trim(v: f64) -> String {
    let s = format!("{v:.4}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

impl FriendlyParams {
    /// The page: buttons on top, the blocks for `user` ("Standard" / "Advanced") below.
    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, user: &str) {
        let Some(link) = app.link else {
            theme::label(ui, "Please Connect First");
            return;
        };
        let o = ui.max_rect().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        if ui.put(r(12.0, 11.0, 103.0, 19.0), |ui: &mut egui::Ui| theme::button_sized(ui, "Write Params", Vec2::new(103.0, 19.0))).clicked() {
            // SortENABLE: *_ENABLE parameters are written last
            let mut list: Vec<(String, f32)> = std::mem::take(&mut self.pending).into_iter().collect();
            list.sort_by_key(|(k, _)| k.ends_with("_ENABLE") || k.ends_with("_ENABLED"));
            if app.readonly {
                app.toasts.error("Readonly mode");
            } else {
                for (k, v) in list {
                    link.send(LinkCommand::SetParam { name: k, value: v });
                }
                app.toasts.info("Parameters successfully saved.");
            }
        }
        if ui.put(r(121.0, 11.0, 103.0, 19.0), |ui: &mut egui::Ui| theme::button_sized(ui, "Refresh Params", Vec2::new(103.0, 19.0))).clicked() {
            self.refresh_confirm = true;
        }
        if ui.put(r(230.0, 11.0, 103.0, 19.0), |ui: &mut egui::Ui| theme::button_sized(ui, "Find", Vec2::new(103.0, 19.0))).clicked() {
            self.search_prompt = Some(self.search.clone());
        }
        let pl = &mut app.settings.planner;
        let mut adv = pl.get("advancedview").map(|v| v == "True").unwrap_or(false);
        if ui.put(r(507.0, 13.0, 131.0, 17.0), egui::Checkbox::new(&mut adv, crate::i18n::t("Advanced Menu View"))).changed() {
            pl.insert("advancedview".into(), if adv { "True" } else { "False" }.into());
        }
        let (fw, all): (Firmware, Vec<String>) = {
            let s = link.shared.read();
            (s.cs.firmware, s.params.keys().cloned().collect())
        };
        let needle = self.search.to_lowercase();
        let names: Vec<String> = all
            .into_iter()
            .filter(|n| {
                let Some(m) = param_meta::get(fw, n) else { return false };
                m.user == user && (needle.is_empty() || n.to_lowercase().contains(&needle) || m.display_name.to_lowercase().contains(&needle) || m.description.to_lowercase().contains(&needle))
            })
            .collect();
        let body = Rect::from_min_max(o + Vec2::new(12.0, 36.0), ui.max_rect().max);
        let mut pending = std::mem::take(&mut self.pending);
        ui.scope_builder(egui::UiBuilder::new().max_rect(body), |ui| rows(ui, app, &names, Some(&mut pending)));
        self.pending = pending;
        self.dialogs(ui.ctx(), link);
    }

    fn dialogs(&mut self, ctx: &egui::Context, link: &Link) {
        if let Some(s) = self.search_prompt.as_mut() {
            let mut done = None;
            egui::Modal::new(egui::Id::new("friendly_find")).show(ctx, |ui| {
                ui.label(egui::RichText::new("Search For").strong().color(Color32::WHITE));
                theme::label(ui, "Enter a single word to search for");
                let resp = ui.add(egui::TextEdit::singleline(s).desired_width(220.0));
                resp.request_focus();
                let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() || enter {
                        done = Some(true);
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        done = Some(false);
                    }
                });
            });
            if let Some(ok) = done {
                let s = self.search_prompt.take().unwrap_or_default();
                self.search = if ok { s } else { String::new() };
            }
        }
        if self.refresh_confirm {
            let mut done = None;
            egui::Modal::new(egui::Id::new("friendly_refresh")).show(ctx, |ui| {
                ui.label(egui::RichText::new("Refresh Params").strong().color(Color32::WHITE));
                theme::label(ui, "Update Params
DON'T DO THIS IF YOU ARE IN THE AIR");
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
                self.refresh_confirm = false;
                if ok {
                    link.send(LinkCommand::RequestParams);
                }
            }
        }
    }
}
