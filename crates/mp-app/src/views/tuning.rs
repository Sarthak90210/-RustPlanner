//! CONFIG tuning forms: ConfigSimplePids (Basic Tuning, driven by acsimplepids.xml),
//! ConfigArducopter (Extended Tuning group boxes) and ConfigAC_Fence (GeoFence).

use std::collections::BTreeMap;

use egui::{Color32, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use mp_core::firmware::Firmware;
use mp_core::link::{Link, LinkCommand, Param};
use mp_core::param_meta;

use crate::app::AppCtx;
use crate::{fonts, theme};

const SIMPLE_XML: &str = include_str!("../../assets/acsimplepids.xml");

/// WinForms GroupBox: thin border with the caption over it.
pub fn group_box(p: &egui::Painter, r: Rect, title: &str) {
    let f = fonts::ui_id(fonts::UI_SIZE);
    let g = p.layout_no_wrap(title.to_string(), f, Color32::WHITE);
    p.rect_stroke(Rect::from_min_max(r.min + Vec2::new(0.0, 6.0), r.max), 3.0, Stroke::new(1.0, Color32::from_rgb(0xd5, 0xdf, 0xe5)), StrokeKind::Inside);
    let tr = Rect::from_min_size(r.min + Vec2::new(6.0, 0.0), g.size() + Vec2::new(4.0, 0.0));
    p.rect_filled(tr, 0.0, theme::BG);
    p.galley(tr.min + Vec2::new(2.0, 0.0), g, Color32::WHITE);
}

fn resolve<'a>(params: &'a BTreeMap<String, Param>, names: &[&'a str]) -> Option<&'a str> {
    names.iter().copied().find(|n| params.contains_key(*n))
}

// ------------------------------------------------------------------ Basic Tuning

struct SimpleItem {
    title: String,
    desc: String,
    name: String,
    min: f32,
    max: f32,
    relations: Vec<(String, f32)>,
}

fn simple_items() -> Vec<SimpleItem> {
    let Ok(doc) = roxmltree::Document::parse(SIMPLE_XML.trim_start_matches('\u{feff}')) else { return Vec::new() };
    let mut out = Vec::new();
    for p in doc.descendants().filter(|n| n.has_tag_name("param") && n.parent().map(|x| x.has_tag_name("ac")).unwrap_or(false)) {
        let text = |tag: &str| p.children().find(|c| c.has_tag_name(tag)).and_then(|c| c.text()).unwrap_or("").trim().to_string();
        let relations = p
            .children()
            .filter(|c| c.has_tag_name("relation"))
            .map(|r| {
                let t = |tag: &str| r.children().find(|c| c.has_tag_name(tag)).and_then(|c| c.text()).unwrap_or("").trim().to_string();
                (t("param"), t("multiplier").parse().unwrap_or(1.0))
            })
            .collect();
        out.push(SimpleItem {
            title: text("title"),
            desc: text("desc"),
            name: text("name"),
            min: text("min").parse().unwrap_or(0.0),
            max: text("max").parse().unwrap_or(1.0),
            relations,
        });
    }
    out
}

/// ConfigSimplePids: one RangeControl (365x108) per acsimplepids.xml item in panel1
/// (4,4 556x218, auto-scroll) and the TXT_info box (4,227 556x58) below it.
pub fn basic_tuning(ui: &mut egui::Ui, app: &mut AppCtx) {
    let Some(link) = app.link else {
        theme::label(ui, "Please Connect First");
        return;
    };
    let (params, fw) = {
        let s = link.shared.read();
        (s.params.clone(), s.cs.firmware)
    };
    let items: Vec<SimpleItem> = simple_items().into_iter().filter(|i| params.contains_key(&i.name)).collect();
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    let f = fonts::ui_id(fonts::UI_SIZE);
    let info_id = egui::Id::new("simplepids_info");
    let mut info: String = ui.ctx().data(|d| d.get_temp(info_id)).unwrap_or_else(|| "NOTE: using this interface may reset some off your custom pids.".to_string());
    let panel = Rect::from_min_size(o + Vec2::new(4.0, 4.0), Vec2::new(556.0, 218.0));
    ui.scope_builder(egui::UiBuilder::new().max_rect(panel), |ui| {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let (area, _) = ui.allocate_exact_size(Vec2::new(540.0, 10.0 + 108.0 * items.len() as f32), Sense::hover());
            let mut y = area.top() + 10.0;
            for it in &items {
                let val = params[&it.name].value as f64;
                let meta = param_meta::get(fw, &it.name);
                let (mut min, mut max) = (it.min as f64, it.max as f64);
                if let Some((lo, hi)) = meta.and_then(|m| m.range) {
                    min = lo;
                    max = hi;
                }
                min = min.min(val);
                max = max.max(val);
                let inc = meta.and_then(|m| m.increment).unwrap_or(0.01);
                let r = Rect::from_min_size(Pos2::new(area.left() + 10.0, y), Vec2::new(365.0, 108.0));
                let pp = ui.painter();
                pp.text(r.min + Vec2::new(3.0, 0.0), egui::Align2::LEFT_TOP, &it.title, fonts::bold_id(fonts::UI_SIZE), Color32::WHITE);
                let g = pp.layout(it.desc.clone(), f.clone(), Color32::WHITE, 365.0);
                pp.galley(r.min + Vec2::new(3.0, 15.0), g, Color32::WHITE);
                let mut v = val;
                let nr = Rect::from_min_size(r.min + Vec2::new(6.0, 58.0), Vec2::new(57.0, 20.0));
                let dv = ui.put(nr, egui::DragValue::new(&mut v).range(min..=max).speed(inc).max_decimals(3));
                let tr = Rect::from_min_size(r.min + Vec2::new(69.0, 58.0), Vec2::new(293.0, 45.0));
                let tb = theme::htrackbar(ui, tr, &mut v, min, max, 10);
                let pp = ui.painter();
                pp.text(Pos2::new(tr.left(), tr.bottom()), egui::Align2::LEFT_BOTTOM, format!("{min}"), f.clone(), Color32::WHITE);
                pp.text(Pos2::new(tr.right(), tr.bottom()), egui::Align2::RIGHT_BOTTOM, format!("{max}"), f.clone(), Color32::WHITE);
                let commit = (tb.drag_stopped() || tb.clicked() || dv.lost_focus() || (dv.changed() && !dv.has_focus())) && (v - val).abs() > 1e-9;
                if commit && !app.readonly {
                    let v = ((v / inc).round() * inc) as f32;
                    link.send(LinkCommand::SetParam { name: it.name.clone(), value: v });
                    info = format!("Set {} to {v}\n", it.name);
                    for (rel, mult) in &it.relations {
                        if params.contains_key(rel) {
                            link.send(LinkCommand::SetParam { name: rel.clone(), value: v * mult });
                            info.push_str(&format!("Set {} to {}\n", rel, v * mult));
                        }
                    }
                }
                y += 108.0;
            }
        });
    });
    let ir = Rect::from_min_size(o + Vec2::new(4.0, 227.0), Vec2::new(556.0, 58.0));
    p.rect_filled(ir, 0.0, theme::CONTROL_BG);
    p.rect_stroke(ir, 0.0, Stroke::new(1.0, Color32::from_gray(0x80)), StrokeKind::Inside);
    let g = p.layout(info.clone(), f, Color32::WHITE, 550.0);
    p.galley(ir.min + Vec2::new(3.0, 3.0), g, Color32::WHITE);
    ui.ctx().data_mut(|d| d.insert_temp(info_id, info));
}

// ------------------------------------------------------------------ Extended Tuning

/// (label, label xy, control xy, param candidates) inside a group.
type Field = (&'static str, (f32, f32), (f32, f32), &'static [&'static str]);

const GROUPS: &[(&str, (f32, f32, f32, f32), &[Field])] = &[
    ("Stabilize Roll (Error to Rate)", (7.0, 5.0, 170.0, 68.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["STB_RLL_P", "ATC_ANG_RLL_P", "Q_A_ANG_RLL_P"]),
        ("ACCEL MAX", (6.0, 42.0), (80.0, 39.0), &["ATC_ACCEL_R_MAX", "Q_A_ACCEL_R_MAX", "ATC_ACC_R_MAX", "Q_A_ACC_R_MAX"]),
    ]),
    ("Stabilize Pitch (Error to Rate)", (183.0, 5.0, 170.0, 68.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["STB_PIT_P", "ATC_ANG_PIT_P", "Q_A_ANG_PIT_P"]),
        ("ACCEL MAX", (6.0, 42.0), (80.0, 39.0), &["ATC_ACCEL_P_MAX", "Q_A_ACCEL_P_MAX", "ATC_ACC_P_MAX", "Q_A_ACC_P_MAX"]),
    ]),
    ("Stabilize Yaw (Error to Rate)", (359.0, 5.0, 170.0, 68.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["STB_YAW_P", "ATC_ANG_YAW_P", "Q_A_ANG_YAW_P"]),
        ("ACCEL MAX", (6.0, 42.0), (80.0, 39.0), &["ATC_ACCEL_Y_MAX", "Q_A_ACCEL_Y_MAX", "ATC_ACC_Y_MAX", "Q_A_ACC_Y_MAX"]),
    ]),
    ("Position XY (Dist to Speed)", (532.0, 5.0, 170.0, 68.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["HLD_LAT_P", "POS_XY_P", "PSC_POSXY_P", "Q_P_POSXY_P"]),
        ("INPUT TC", (6.0, 42.0), (80.0, 39.0), &["ATC_INPUT_TC", "Q_A_INPUT_TC"]),
    ]),
    ("Rate Roll", (7.0, 100.0, 170.0, 180.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["RATE_RLL_P", "ATC_RAT_RLL_P", "Q_A_RAT_RLL_P"]),
        ("I", (6.0, 40.0), (80.0, 37.0), &["RATE_RLL_I", "ATC_RAT_RLL_I", "Q_A_RAT_RLL_I"]),
        ("D", (6.0, 63.0), (80.0, 60.0), &["RATE_RLL_D", "ATC_RAT_RLL_D", "Q_A_RAT_RLL_D"]),
        ("IMAX", (6.0, 86.0), (80.0, 83.0), &["ATC_RAT_RLL_IMAX", "Q_A_RAT_RLL_IMAX", "RATE_RLL_IMAX"]),
        ("FLTE", (6.0, 108.0), (80.0, 106.0), &["RATE_RLL_FILT", "ATC_RAT_RLL_FILT", "ATC_RAT_RLL_FLTE", "Q_A_RAT_RLL_FLTE"]),
        ("FLTD", (6.0, 131.0), (80.0, 129.0), &["ATC_RAT_RLL_FLTD", "Q_A_RAT_RLL_FLTD"]),
        ("FLTT", (7.0, 154.0), (80.0, 152.0), &["ATC_RAT_RLL_FLTT", "Q_A_RAT_RLL_FLTT"]),
    ]),
    ("Rate Pitch", (183.0, 100.0, 170.0, 180.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["RATE_PIT_P", "ATC_RAT_PIT_P", "Q_A_RAT_PIT_P"]),
        ("I", (6.0, 40.0), (80.0, 37.0), &["RATE_PIT_I", "ATC_RAT_PIT_I", "Q_A_RAT_PIT_I"]),
        ("D", (6.0, 63.0), (80.0, 60.0), &["RATE_PIT_D", "ATC_RAT_PIT_D", "Q_A_RAT_PIT_D"]),
        ("IMAX", (6.0, 86.0), (80.0, 84.0), &["ATC_RAT_PIT_IMAX", "Q_A_RAT_PIT_IMAX", "RATE_PIT_IMAX"]),
        ("FLTE", (6.0, 108.0), (80.0, 106.0), &["RATE_PIT_FILT", "ATC_RAT_PIT_FILT", "ATC_RAT_PIT_FLTE", "Q_A_RAT_PIT_FLTE"]),
        ("FLTD", (6.0, 131.0), (80.0, 129.0), &["ATC_RAT_PIT_FLTD", "Q_A_RAT_PIT_FLTD"]),
        ("FLTT", (5.0, 156.0), (80.0, 152.0), &["ATC_RAT_PIT_FLTT", "Q_A_RAT_PIT_FLTT"]),
    ]),
    ("Rate Yaw", (359.0, 100.0, 170.0, 180.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["RATE_YAW_P", "ATC_RAT_YAW_P", "Q_A_RAT_YAW_P"]),
        ("I", (6.0, 40.0), (80.0, 37.0), &["RATE_YAW_I", "ATC_RAT_YAW_I", "Q_A_RAT_YAW_I"]),
        ("D", (6.0, 63.0), (80.0, 60.0), &["RATE_YAW_D", "ATC_RAT_YAW_D", "Q_A_RAT_YAW_D"]),
        ("IMAX", (6.0, 86.0), (80.0, 83.0), &["ATC_RAT_YAW_IMAX", "Q_A_RAT_YAW_IMAX", "RATE_YAW_IMAX"]),
        ("FLTE", (6.0, 108.0), (80.0, 106.0), &["RATE_YAW_FILT", "ATC_RAT_YAW_FILT", "ATC_RAT_YAW_FLTE", "Q_A_RAT_YAW_FLTE"]),
        ("FLTD", (6.0, 131.0), (79.0, 129.0), &["ATC_RAT_YAW_FLTD", "Q_A_RAT_YAW_FLTD"]),
        ("FLTT", (6.0, 154.0), (79.0, 152.0), &["ATC_RAT_YAW_FLTT", "Q_A_RAT_YAW_FLTT"]),
    ]),
    ("Velocity XY (Vel to Accel)", (532.0, 100.0, 170.0, 108.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["LOITER_LAT_P", "VEL_XY_P", "PSC_VELXY_P", "Q_P_VELXY_P", "PSC_NE_VEL_P", "Q_P_NE_VEL_P"]),
        ("I", (6.0, 40.0), (80.0, 37.0), &["LOITER_LAT_I", "VEL_XY_I", "PSC_VELXY_I", "Q_P_VELXY_I", "PSC_NE_VEL_I", "Q_P_NE_VEL_I"]),
        ("D", (6.0, 63.0), (80.0, 60.0), &["LOITER_LAT_D", "PSC_VELXY_D", "Q_P_VELXY_D", "PSC_NE_VEL_D", "Q_P_NE_VEL_D"]),
        ("IMAX", (6.0, 87.0), (80.0, 84.0), &["LOITER_LAT_IMAX", "VEL_XY_IMAX", "PSC_VELXY_IMAX", "Q_P_VELXY_IMAX", "PSC_NE_VEL_IMAX", "Q_P_NE_VEL_IMAX"]),
    ]),
    ("Basic Filters", (532.0, 210.0, 170.0, 70.0), &[
        ("Gyro", (8.0, 16.0), (79.0, 14.0), &["INS_GYRO_FILTER"]),
        ("Accel", (8.0, 39.0), (80.0, 37.0), &["INS_ACCEL_FILTER"]),
    ]),
    ("Throttle Accel (Accel to motor)", (7.0, 287.0, 170.0, 122.0), &[
        ("P", (6.0, 22.0), (80.0, 19.0), &["THR_ACCEL_P", "ACCEL_Z_P", "PSC_ACCZ_P", "Q_P_ACCZ_P", "PSC_D_ACC_P", "Q_P_D_ACC_P"]),
        ("I", (6.0, 46.0), (80.0, 43.0), &["THR_ACCEL_I", "ACCEL_Z_I", "PSC_ACCZ_I", "Q_P_ACCZ_I", "PSC_D_ACC_I", "Q_P_D_ACC_I"]),
        ("D", (6.0, 69.0), (80.0, 66.0), &["THR_ACCEL_D", "ACCEL_Z_D", "PSC_ACCZ_D", "Q_P_ACCZ_D", "PSC_D_ACC_D", "Q_P_D_ACC_D"]),
        ("IMAX", (6.0, 92.0), (80.0, 89.0), &["THR_ACCEL_IMAX", "ACCEL_Z_IMAX", "PSC_ACCZ_IMAX", "Q_P_ACCZ_IMAX", "PSC_D_ACC_IMAX", "Q_P_D_ACC_IMAX"]),
    ]),
    ("Throttle Rate (VSpd to accel)", (183.0, 287.0, 170.0, 50.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["THR_RATE_P", "VEL_Z_P", "PSC_VELZ_P", "Q_P_VELZ_P", "PSC_D_VEL_P", "Q_P_D_VEL_P"]),
    ]),
    ("Altitude Hold (Alt to climbrate)", (359.0, 287.0, 170.0, 50.0), &[
        ("P", (6.0, 16.0), (80.0, 13.0), &["THR_ALT_P", "POS_Z_P", "PSC_POSZ_P", "Q_P_POSZ_P", "PSC_D_POS_P", "Q_P_D_POS_P"]),
    ]),
    ("WPNav (cm's)", (532.0, 287.0, 170.0, 138.0), &[
        ("Speed ", (6.0, 16.0), (80.0, 13.0), &["WPNAV_SPEED", "Q_WP_SPEED", "WP_SPD", "Q_WP_SPD"]),
        ("Radius", (6.0, 40.0), (80.0, 37.0), &["WPNAV_RADIUS", "Q_WP_RADIUS", "WP_RADIUS_M", "Q_WP_RADIUS_M"]),
        ("Speed Up", (6.0, 63.0), (80.0, 60.0), &["WPNAV_SPEED_UP", "Q_WP_SPEED_UP", "WP_SPD_UP", "Q_WP_SPD_UP"]),
        ("Speed Dn", (6.0, 87.0), (80.0, 84.0), &["WPNAV_SPEED_DN", "Q_WP_SPEED_DN", "WP_SPD_DN", "Q_WP_SPD_DN"]),
        ("Loiter Speed", (6.0, 110.0), (80.0, 107.0), &["WPNAV_LOIT_SPEED", "LOIT_SPEED", "Q_LOIT_SPEED", "LOIT_SPEED_MS", "Q_LOIT_SPEED_MS"]),
    ]),
    ("Filter Logs", (7.0, 415.0, 344.0, 48.0), &[
        ("Mask", (6.0, 16.0), (80.0, 13.0), &["INS_LOG_BAT_MASK"]),
        ("Options", (182.0, 16.0), (256.0, 14.0), &["INS_LOG_BAT_OPT"]),
    ]),
    ("Static Notch Filter", (7.0, 469.0, 170.0, 133.0), &[
        ("Enabled", (8.0, 29.0), (80.0, 26.0), &["INS_NOTCH_ENABLE"]),
        ("Frequency", (6.0, 54.0), (80.0, 52.0), &["INS_NOTCH_FREQ"]),
        ("BandWidth", (8.0, 79.0), (80.0, 77.0), &["INS_NOTCH_BW"]),
        ("Attenuation", (8.0, 104.0), (80.0, 102.0), &["INS_NOTCH_ATT"]),
    ]),
    ("Harmonic Notch Filter", (186.0, 469.0, 342.0, 133.0), &[
        ("Enabled", (6.0, 29.0), (64.0, 26.0), &["INS_HNTCH_ENABLE"]),
        ("Mode", (6.0, 54.0), (64.0, 52.0), &["INS_HNTCH_MODE"]),
        ("Reference", (6.0, 79.0), (64.0, 77.0), &["INS_HNTCH_REF"]),
        ("Frequency", (6.0, 104.0), (64.0, 102.0), &["INS_HNTCH_FREQ"]),
        ("Attenuation", (158.0, 29.0), (229.0, 27.0), &["INS_HNTCH_ATT"]),
        ("Bandwidth", (158.0, 54.0), (229.0, 52.0), &["INS_HNTCH_BW"]),
        ("Options", (158.0, 79.0), (230.0, 77.0), &["INS_HNTCH_OPTS"]),
        ("Harmonics", (158.0, 104.0), (230.0, 102.0), &["INS_HNTCH_HMNCS"]),
    ]),
];

#[derive(Default)]
pub struct ExtendedTuning {
    changes: BTreeMap<String, f32>,
    lock_rp: bool,
}

fn number_box(ui: &mut egui::Ui, r: Rect, name: &str, cur: f32, changed: bool, fw: Firmware) -> Option<f32> {
    let meta = param_meta::get(fw, name);
    let mut child = ui.new_child(egui::UiBuilder::new().max_rect(r));
    if changed {
        let v = child.visuals_mut();
        for w in [&mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active] {
            w.weak_bg_fill = Color32::from_rgb(0, 128, 0);
            w.bg_fill = Color32::from_rgb(0, 128, 0);
        }
    }
    let mut v = cur;
    let resp = child.put(r, egui::DragValue::new(&mut v).speed(meta.and_then(|m| m.increment).unwrap_or(0.001)).max_decimals(4));
    if let Some(m) = meta {
        resp.on_hover_text(format!("{}\n{}", m.display_name, m.description));
    }
    (v != cur).then_some(v)
}

impl ExtendedTuning {
    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let Some(link) = app.link else {
            theme::label(ui, "Please Connect First");
            return;
        };
        let (params, fw) = {
            let s = link.shared.read();
            (s.params.clone(), s.cs.firmware)
        };
        egui::ScrollArea::both().auto_shrink([false, false]).show(ui, |ui| {
            let (area, _) = ui.allocate_exact_size(Vec2::new(710.0, 640.0), Sense::hover());
            let o = area.min;
            let p = ui.painter().clone();
            let f = fonts::ui_id(fonts::UI_SIZE);
            let mut edits: Vec<(String, f32)> = Vec::new();
            for (title, (gx, gy, gw, gh), fields) in GROUPS {
                let gr = Rect::from_min_size(o + Vec2::new(*gx, *gy), Vec2::new(*gw, *gh));
                group_box(&p, gr, title);
                for (label, (lx, ly), (cx, cy), names) in fields.iter() {
                    p.text(gr.min + Vec2::new(*lx, *ly), egui::Align2::LEFT_TOP, *label, f.clone(), Color32::WHITE);
                    let cr = Rect::from_min_size(gr.min + Vec2::new(*cx, *cy), Vec2::new(78.0, 20.0));
                    match resolve(&params, names) {
                        Some(name) => {
                            let cur = self.changes.get(name).copied().unwrap_or(params[name].value);
                            if let Some(v) = number_box(ui, cr, name, cur, self.changes.contains_key(name), fw) {
                                edits.push((name.to_string(), v));
                            }
                        }
                        None => {
                            ui.put(cr, |ui: &mut egui::Ui| ui.add_enabled(false, egui::DragValue::new(&mut 0.0f32)));
                        }
                    }
                }
            }
            ui.put(Rect::from_min_size(o + Vec2::new(7.0, 79.0), Vec2::new(170.0, 17.0)), egui::Checkbox::new(&mut self.lock_rp, "Lock Pitch and Roll Values"));
            for (name, v) in edits {
                if self.lock_rp {
                    // CHK_lockrollpitch mirrors roll <-> pitch rate/stab values
                    let twin = if name.contains("RLL") { Some(name.replace("RLL", "PIT")) } else if name.contains("PIT") { Some(name.replace("PIT", "RLL")) } else { None };
                    if let Some(t) = twin.filter(|t| params.contains_key(t)) {
                        self.set_change(&params, &t, v);
                    }
                }
                self.set_change(&params, &name, v);
            }

            // Tune / RC options
            let lab = |x: f32, y: f32, t: &str| p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, t, f.clone(), Color32::WHITE);
            lab(183.0, 332.0, "Tune");
            lab(183.0, 357.0, "Min");
            for (y, t, names) in [
                (361.0, "RC7 Opt", &["CH7_OPT", "CH7_OPTION", "RC7_OPTION"][..]),
                (388.0, "RC8 Opt", &["CH8_OPT", "CH8_OPTION", "RC8_OPTION"][..]),
                (415.0, "RC9 Opt", &["CH9_OPT", "CH9_OPTION", "RC9_OPTION"][..]),
                (442.0, "RC10 Opt", &["CH10_OPT", "CH10_OPTION", "RC10_OPTION"][..]),
            ] {
                lab(357.0, y, t);
                if let Some(n) = resolve(&params, names) {
                    let r = Rect::from_min_size(o + Vec2::new(416.0, y), Vec2::new(112.0, 21.0));
                    ui.put(r, |ui: &mut egui::Ui| {
                        crate::views::config::param_control(ui, link, fw, n, params[n].value, 104.0);
                        ui.allocate_response(Vec2::ZERO, Sense::hover())
                    });
                }
            }
            if let Some(n) = resolve(&params, &["TUNE"]) {
                let r = Rect::from_min_size(o + Vec2::new(242.0, 332.0), Vec2::new(112.0, 21.0));
                ui.put(r, |ui: &mut egui::Ui| {
                    crate::views::config::param_control(ui, link, fw, n, params[n].value, 104.0);
                    ui.allocate_response(Vec2::ZERO, Sense::hover())
                });
            }
            for (x, w, names) in [(218.0, 51.0, &["TUNE_LOW", "TUNE_MIN"][..]), (308.0, 46.0, &["TUNE_HIGH", "TUNE_MAX"][..])] {
                if let Some(n) = resolve(&params, names) {
                    let r = Rect::from_min_size(o + Vec2::new(x, 360.0), Vec2::new(w, 20.0));
                    let cur = self.changes.get(n).copied().unwrap_or(params[n].value);
                    if let Some(v) = number_box(ui, r, n, cur, self.changes.contains_key(n), fw) {
                        self.set_change(&params, n, v);
                    }
                }
            }

            let b = |ui: &mut egui::Ui, x: f32, t: &str| {
                let r = Rect::from_min_size(o + Vec2::new(x, 608.0), Vec2::new(103.0, 25.0));
                ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, t, r.size())).clicked()
            };
            if b(ui, 208.0, "Write Params") {
                write_changes(app, link, &mut self.changes);
            }
            if b(ui, 317.0, "Refresh Params") {
                self.changes.clear();
                link.send(LinkCommand::RequestParams);
            }
            if b(ui, 426.0, "Refresh Screen") {
                self.changes.clear();
            }
        });
    }

    fn set_change(&mut self, params: &BTreeMap<String, Param>, name: &str, v: f32) {
        if params.get(name).map(|p| p.value == v).unwrap_or(false) {
            self.changes.remove(name);
        } else {
            self.changes.insert(name.to_string(), v);
        }
    }
}

fn write_changes(app: &mut AppCtx, link: &Link, changes: &mut BTreeMap<String, f32>) {
    if app.readonly {
        app.toasts.error("Readonly mode: parameters not written");
        return;
    }
    for (k, v) in changes.iter() {
        link.send(LinkCommand::SetParam { name: k.clone(), value: *v });
    }
    changes.clear();
}

// ------------------------------------------------------------------ GeoFence (ConfigAC_Fence)

pub fn geofence(ui: &mut egui::Ui, app: &mut AppCtx) {
    let Some(link) = app.link else {
        theme::label(ui, "Please Connect First");
        return;
    };
    let (params, fw) = {
        let s = link.shared.read();
        (s.params.clone(), s.cs.firmware)
    };
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    p.text(o + Vec2::new(19.0, 4.0), egui::Align2::LEFT_TOP, "Geo Fence", fonts::bold_id(15.0), Color32::WHITE);
    p.line_segment([o + Vec2::new(23.0, 27.0), o + Vec2::new(483.0, 27.0)], Stroke::new(1.0, Color32::GRAY));
    let t = o + Vec2::new(23.0, 35.0);
    let f = fonts::ui_id(fonts::UI_SIZE);
    let rows: [(&str, f32, &[&str]); 7] = [
        ("Enable", 0.0, &["FENCE_ENABLE"]),
        ("Type", 25.0, &["FENCE_TYPE"]),
        ("Action", 50.0, &["FENCE_ACTION"]),
        ("Max Alt", 75.0, &["FENCE_ALT_MAX"]),
        ("Min Alt", 100.0, &["FENCE_ALT_MIN"]),
        ("Max Radius", 125.0, &["FENCE_RADIUS"]),
        ("RTL Altitude", 145.0, &["RTL_ALT_M", "RTL_ALT"]),
    ];
    for (label, y, names) in rows {
        p.text(t + Vec2::new(3.0, y), egui::Align2::LEFT_TOP, label, f.clone(), Color32::WHITE);
        if let Some(n) = resolve(&params, names) {
            let r = Rect::from_min_size(t + Vec2::new(136.0, y + 2.0), Vec2::new(121.0, 20.0));
            let val = params[n].value;
            if n == "FENCE_ENABLE" {
                let mut b = val != 0.0;
                if ui.put(r, egui::Checkbox::new(&mut b, "Enable")).changed() && !app.readonly {
                    link.send(LinkCommand::SetParam { name: n.into(), value: if b { 1.0 } else { 0.0 } });
                }
            } else {
                ui.put(r, |ui: &mut egui::Ui| {
                    crate::views::config::param_control(ui, link, fw, n, val, 113.0);
                    ui.allocate_response(Vec2::ZERO, Sense::hover())
                });
            }
        }
    }
    ui.allocate_rect(Rect::from_min_size(o, Vec2::new(507.0, 231.0)), Sense::hover());
    let _ = Pos2::ZERO;
}
