//! The Flight Data tuning graph (`zg1`, a ZedGraph control): live series sampled from
//! CurrentState over the last 20 s, styled with the theme's ZedGraph colours. Double-click
//! to choose the plotted values, as in Mission Planner.

use std::collections::VecDeque;
use std::time::Instant;

use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, StrokeKind, Vec2};
use mp_core::state::CurrentState;

use crate::fonts;

const CHART: Color32 = Color32::from_rgb(0x1f, 0x1f, 0x20);
const PANE: Color32 = Color32::from_rgb(0x37, 0x37, 0x38);
const LEGEND: Color32 = Color32::from_rgb(0x85, 0x84, 0x83);
const COLORS: [Color32; 10] = [
    Color32::RED,
    Color32::GREEN,
    Color32::from_rgb(0x00, 0xbf, 0xff),
    Color32::from_rgb(0xff, 0xa5, 0x00),
    Color32::YELLOW,
    Color32::from_rgb(0xff, 0x00, 0xff),
    Color32::WHITE,
    Color32::from_rgb(0x00, 0xff, 0xff),
    Color32::from_rgb(0xa5, 0x2a, 0x2a),
    Color32::from_rgb(0x80, 0x80, 0xff),
];
const WINDOW: f64 = 20.0;

pub struct TuningGraph {
    start: Instant,
    last: f64,
    pub selected: Vec<String>,
    data: Vec<VecDeque<(f64, f64)>>,
    picking: bool,
}

impl Default for TuningGraph {
    fn default() -> Self {
        let selected: Vec<String> = ["roll", "pitch", "nav_roll", "nav_pitch"].iter().map(|s| s.to_string()).collect();
        Self { start: Instant::now(), last: -1.0, data: vec![VecDeque::new(); selected.len()], selected, picking: false }
    }
}

impl TuningGraph {
    pub fn sample(&mut self, cs: &CurrentState) {
        let t = self.start.elapsed().as_secs_f64();
        if t - self.last < 0.1 {
            return;
        }
        self.last = t;
        for (i, name) in self.selected.iter().enumerate() {
            if let Some(v) = cs.field(name) {
                let q = &mut self.data[i];
                q.push_back((t, v));
                while q.front().map(|(x, _)| t - x > WINDOW).unwrap_or(false) {
                    q.pop_front();
                }
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, cs: &CurrentState, size: Vec2) {
        self.sample(cs);
        let series: Vec<(&str, &VecDeque<(f64, f64)>)> = self.selected.iter().map(|s| s.as_str()).zip(self.data.iter()).collect();
        let resp = plot(ui, size, "Tuning", &series, self.last);
        if resp.double_clicked() {
            self.picking = true;
        }
        if self.picking {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Graph This")).open(&mut open).collapsible(false).show(ui.ctx(), |ui| {
                egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                    egui::Grid::new("zg_pick").num_columns(4).show(ui, |ui| {
                        for (k, (n, _)) in cs.fields().into_iter().enumerate() {
                            let mut on = self.selected.iter().any(|s| s == n);
                            if ui.checkbox(&mut on, n).changed() {
                                if on {
                                    self.selected.push(n.to_string());
                                    self.data.push(VecDeque::new());
                                } else if let Some(idx) = self.selected.iter().position(|s| s == n) {
                                    self.selected.remove(idx);
                                    self.data.remove(idx);
                                }
                            }
                            if k % 4 == 3 {
                                ui.end_row();
                            }
                        }
                    });
                });
            });
            if !open {
                self.picking = false;
            }
        }
    }
}

/// Draw a ZedGraph-styled time plot of `series` over the last 20 s ending at `t_last`.
pub fn plot(ui: &mut egui::Ui, size: Vec2, title: &str, series: &[(&str, &VecDeque<(f64, f64)>)], t_last: f64) -> egui::Response {
    let t1 = t_last.max(WINDOW);
    plot_range(ui, size, title, series, t1 - WINDOW, t1)
}

/// Draw a ZedGraph-styled plot of `series` between times `t0` and `t1`.
pub fn plot_range(ui: &mut egui::Ui, size: Vec2, title: &str, series: &[(&str, &VecDeque<(f64, f64)>)], t0: f64, t1: f64) -> egui::Response {
    let span = (t1 - t0).max(1e-6);
    let (rect, resp) = ui.allocate_exact_size(size, Sense::click());
    let p = ui.painter().with_clip_rect(rect);
    p.rect_filled(rect, 0.0, PANE);
    let f = fonts::ui_id(fonts::UI_SIZE);
    p.text(Pos2::new(rect.center().x, rect.top() + 3.0), egui::Align2::CENTER_TOP, title, fonts::bold_id(13.0), Color32::WHITE);
    let chart = Rect::from_min_max(rect.min + Vec2::new(46.0, 22.0), rect.max - Vec2::new(10.0, 40.0));
    p.rect_filled(chart, 0.0, CHART);
    p.rect_stroke(chart, 0.0, Stroke::new(1.0, Color32::GRAY), StrokeKind::Inside);

    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for (_, q) in series {
        for (_, v) in q.iter() {
            lo = lo.min(*v);
            hi = hi.max(*v);
        }
    }
    if lo > hi {
        lo = -1.0;
        hi = 1.0;
    }
    if (hi - lo).abs() < 1e-6 {
        hi += 1.0;
        lo -= 1.0;
    }
    let pad = (hi - lo) * 0.1;
    let (lo, hi) = (lo - pad, hi + pad);
    let to = |t: f64, v: f64| Pos2::new(chart.left() + ((t - t0) / span) as f32 * chart.width(), chart.bottom() - ((v - lo) / (hi - lo)) as f32 * chart.height());
    // grid + axis labels
    for k in 0..=4 {
        let v = lo + (hi - lo) * k as f64 / 4.0;
        let y = to(t0, v).y;
        p.line_segment([Pos2::new(chart.left(), y), Pos2::new(chart.right(), y)], Stroke::new(1.0, Color32::from_gray(60)));
        p.text(Pos2::new(chart.left() - 4.0, y), egui::Align2::RIGHT_CENTER, format!("{v:.1}"), f.clone(), Color32::WHITE);
    }
    for k in 0..=4 {
        let t = t0 + span * k as f64 / 4.0;
        let x = to(t, lo).x;
        p.text(Pos2::new(x, chart.bottom() + 2.0), egui::Align2::CENTER_TOP, format!("{t:.0}"), f.clone(), Color32::WHITE);
    }
    for (i, (_, q)) in series.iter().enumerate() {
        if q.len() > 1 {
            let pts: Vec<Pos2> = q.iter().map(|&(t, v)| to(t, v)).collect();
            p.add(Shape::line(pts, Stroke::new(1.5, COLORS[i % COLORS.len()])));
        }
    }
    // legend
    let mut x = rect.left() + 46.0;
    let ly = rect.bottom() - 18.0;
    let lr = Rect::from_min_max(Pos2::new(rect.left() + 40.0, ly - 3.0), Pos2::new(rect.right() - 10.0, ly + 14.0));
    p.rect_filled(lr, 0.0, LEGEND);
    for (i, (name, _)) in series.iter().enumerate() {
        let col = COLORS[i % COLORS.len()];
        p.line_segment([Pos2::new(x, ly + 6.0), Pos2::new(x + 16.0, ly + 6.0)], Stroke::new(2.0, col));
        let g = p.layout_no_wrap(name.to_string(), f.clone(), Color32::BLACK);
        let w = g.size().x;
        p.galley(Pos2::new(x + 20.0, ly), g, Color32::BLACK);
        x += 30.0 + w;
    }
    resp
}
