//! BurntKermit theme (BurntKermit.mpsystheme + ThemeManager.cs) and the custom-drawn controls
//! Mission Planner uses: MyButton, the WinForms tab strip, BackstageView and MyTrackBar.

use egui::epaint::{Mesh, Vertex, WHITE_UV};
use egui::{
    Align, Color32, CornerRadius, FontId, Pos2, Rect, Response, Sense, Shape, Stroke, StrokeKind,
    Ui, Vec2, Visuals,
};

use crate::fonts;

// ThemeManager colours (BurntKermit.mpsystheme)
pub const BG: Color32 = Color32::from_rgb(0x26, 0x27, 0x28);
pub const CONTROL_BG: Color32 = Color32::from_rgb(0x43, 0x44, 0x45);
pub const TEXT: Color32 = Color32::WHITE;
pub const BUT_TOP: Color32 = Color32::from_rgb(0x94, 0xc1, 0x1f);
pub const BUT_BOT: Color32 = Color32::from_rgb(0xcd, 0xe2, 0x96);
pub const BUT_TEXT: Color32 = Color32::from_rgb(0x40, 0x57, 0x04);
pub const BUT_OUTLINE: Color32 = Color32::from_rgb(0x79, 0x94, 0x29);
pub const MOUSE_OVER: Color32 = Color32::from_rgba_unmultiplied_const(0x2b, 0x3a, 0x03, 73);
pub const MOUSE_DOWN: Color32 = Color32::from_rgba_unmultiplied_const(0x2b, 0x3a, 0x03, 150);
pub const NOT_ENABLED: Color32 = Color32::from_rgba_unmultiplied_const(0x2b, 0x3a, 0x03, 150);
pub const BANNER1: Color32 = Color32::from_rgb(0x40, 0x57, 0x04);
pub const BANNER2: Color32 = Color32::from_rgb(0x94, 0xc1, 0x1f);
pub const PROGRESS_TOP: Color32 = Color32::from_rgb(0x66, 0x8b, 0x1a);
pub const PROGRESS_BOT: Color32 = Color32::from_rgb(0x7c, 0xa4, 0x28);
pub const PROGRESS_OUTLINE: Color32 = Color32::from_rgb(0x96, 0xae, 0x70);
/// SystemColors.ControlLight: menu item text.
pub const MENU_TEXT: Color32 = Color32::from_rgb(0xe3, 0xe3, 0xe3);
pub const WHITE_SMOKE: Color32 = Color32::from_rgb(0xf5, 0xf5, 0xf5);
pub const DARK_GRAY: Color32 = Color32::from_rgb(0xa9, 0xa9, 0xa9);
pub const GREEN: Color32 = BUT_TOP;
pub const GREEN_LIGHT: Color32 = BUT_BOT;

pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
    let mut v = Visuals::dark();
    v.panel_fill = BG;
    v.window_fill = BG;
    v.window_stroke = Stroke::new(1.0, DARK_GRAY);
    v.window_corner_radius = CornerRadius::ZERO;
    v.menu_corner_radius = CornerRadius::ZERO;
    v.extreme_bg_color = CONTROL_BG;
    v.text_edit_bg_color = Some(CONTROL_BG);
    v.faint_bg_color = BG;
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = Color32::from_rgb(0x33, 0x99, 0xff);
    v.selection.stroke = Stroke::new(1.0, Color32::WHITE);
    v.hyperlink_color = Color32::from_rgb(0x66, 0xb2, 0xff);
    let r = CornerRadius::ZERO;
    let border = Color32::from_rgb(0x7a, 0x7a, 0x7a);
    for w in [&mut v.widgets.noninteractive, &mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
        w.corner_radius = r;
        w.expansion = 0.0;
    }
    v.widgets.noninteractive.bg_fill = BG;
    v.widgets.noninteractive.weak_bg_fill = BG;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x55, 0x56, 0x57));
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.inactive.bg_fill = CONTROL_BG;
    v.widgets.inactive.weak_bg_fill = CONTROL_BG;
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, border);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.hovered.bg_fill = CONTROL_BG;
    v.widgets.hovered.weak_bg_fill = CONTROL_BG;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x33, 0x99, 0xff));
    v.widgets.hovered.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.active.bg_fill = CONTROL_BG;
    v.widgets.active.weak_bg_fill = CONTROL_BG;
    v.widgets.active.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x33, 0x99, 0xff));
    v.widgets.active.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.open = v.widgets.active;
    ctx.set_visuals(v);
    ctx.global_style_mut(|s| {
        use egui::TextStyle::*;
        s.text_styles.insert(Body, fonts::ui_id(fonts::UI_SIZE));
        s.text_styles.insert(Button, fonts::ui_id(fonts::UI_SIZE));
        s.text_styles.insert(Small, fonts::ui_id(9.0));
        s.text_styles.insert(Heading, fonts::ui_id(14.0));
        s.text_styles.insert(Monospace, egui::FontId::monospace(11.0));
        s.spacing.item_spacing = egui::vec2(3.0, 3.0);
        s.spacing.button_padding = egui::vec2(4.0, 2.0);
        s.spacing.interact_size = egui::vec2(40.0, 21.0);
        s.spacing.combo_height = 300.0;
        s.spacing.window_margin = egui::Margin::same(4);
        s.spacing.menu_margin = egui::Margin::same(2);
    });
}

/// Vertical linear gradient fill, as GDI `LinearGradientBrush(..., Vertical)`.
pub fn vgradient(painter: &egui::Painter, rect: Rect, top: Color32, bottom: Color32) {
    let mut m = Mesh::default();
    let i = 0;
    for (p, c) in [(rect.left_top(), top), (rect.right_top(), top), (rect.right_bottom(), bottom), (rect.left_bottom(), bottom)] {
        m.vertices.push(Vertex { pos: p, uv: WHITE_UV, color: c });
    }
    m.indices.extend_from_slice(&[i, i + 1, i + 2, i, i + 2, i + 3]);
    painter.add(Shape::mesh(m));
}

/// Horizontal linear gradient fill.
pub fn hgradient(painter: &egui::Painter, rect: Rect, left: Color32, right: Color32) {
    let mut m = Mesh::default();
    for (p, c) in [(rect.left_top(), left), (rect.right_top(), right), (rect.right_bottom(), right), (rect.left_bottom(), left)] {
        m.vertices.push(Vertex { pos: p, uv: WHITE_UV, color: c });
    }
    m.indices.extend_from_slice(&[0, 1, 2, 0, 2, 3]);
    painter.add(Shape::mesh(m));
}

fn centered_text(painter: &egui::Painter, rect: Rect, text: &str, font: FontId, color: Color32) {
    let mut job = egui::text::LayoutJob::simple(text.to_string(), font, color, (rect.width() - 4.0).max(10.0));
    job.halign = Align::Center;
    let galley = painter.layout_job(job);
    let pos = rect.center() - galley.rect.center().to_vec2();
    painter.galley(pos, galley, color);
}

// ------------------------------------------------------------------ MyButton

/// `MissionPlanner.Controls.MyButton`: green vertical gradient, 1px outline, dark green text.
pub fn button_sized(ui: &mut Ui, text: &str, size: Vec2) -> Response {
    let (rect, resp) = ui.allocate_exact_size(size, Sense::click());
    if ui.is_rect_visible(rect) {
        let p = ui.painter();
        vgradient(p, rect, BUT_TOP, BUT_BOT);
        p.rect_stroke(rect.shrink(0.5), CornerRadius::same(1), Stroke::new(1.0, BUT_OUTLINE), StrokeKind::Inside);
        if !ui.is_enabled() {
            p.rect_filled(rect, 0.0, NOT_ENABLED);
        } else if resp.is_pointer_button_down_on() {
            p.rect_filled(rect, 0.0, MOUSE_DOWN);
        } else if resp.hovered() {
            p.rect_filled(rect, 0.0, MOUSE_OVER);
        }
        centered_text(p, rect, text, fonts::ui_id(fonts::UI_SIZE), BUT_TEXT);
    }
    resp
}

/// MyButton sized to its text (WinForms default 75x23 minimum).
pub fn button(ui: &mut Ui, text: &str) -> Response {
    let w = ui.painter().layout_no_wrap(text.to_string(), fonts::ui_id(fonts::UI_SIZE), TEXT).size().x + 16.0;
    button_sized(ui, text, Vec2::new(w.max(75.0), 23.0))
}

/// Button filling the full width of the current cell, as `Dock = Fill` in a TableLayoutPanel.
pub fn button_fill(ui: &mut Ui, text: &str) -> Response {
    let size = ui.available_size();
    button_sized(ui, text, size)
}

// ------------------------------------------------------------------ WinForms TabControl

/// OS-drawn TabControl header strip (light tabs, black text) with the ◄ ► scroller when the
/// tabs overflow.
pub fn tab_strip(ui: &mut Ui, id: &str, labels: &[&str], selected: &mut usize) {
    let font = fonts::ui_id(fonts::UI_SIZE);
    let h = 21.0;
    let full = ui.available_width();
    let (strip, _) = ui.allocate_exact_size(Vec2::new(full, h + 2.0), Sense::hover());
    let p = ui.painter().with_clip_rect(strip);
    let first_id = ui.scope_id().with(id).with("first");
    let mut first: usize = ui.ctx().data(|d| d.get_temp(first_id)).unwrap_or(0);
    let widths: Vec<f32> = labels.iter().map(|l| p.layout_no_wrap(l.to_string(), font.clone(), Color32::BLACK).size().x + 12.0).collect();
    let total: f32 = widths.iter().sum();
    let scroll = total > full;
    let avail = if scroll { full - 34.0 } else { full };
    first = first.min(labels.len().saturating_sub(1));
    let mut x = strip.left() + 2.0;
    let bottom = strip.bottom();
    let border = Color32::from_rgb(0xd9, 0xd9, 0xd9);
    for (i, l) in labels.iter().enumerate().skip(if scroll { first } else { 0 }) {
        let w = widths[i];
        if x + w > strip.left() + avail + 1.0 && i != *selected {
            break;
        }
        let sel = i == *selected;
        let r = if sel {
            Rect::from_min_max(Pos2::new(x - 2.0, bottom - h - 2.0), Pos2::new(x + w + 2.0, bottom + 1.0))
        } else {
            Rect::from_min_max(Pos2::new(x, bottom - h), Pos2::new(x + w, bottom))
        };
        let resp = ui.interact(r, ui.scope_id().with(id).with(i), Sense::click());
        let fill = if sel {
            Color32::WHITE
        } else if resp.hovered() {
            Color32::from_rgb(0xe5, 0xf1, 0xfb)
        } else {
            Color32::from_rgb(0xf0, 0xf0, 0xf0)
        };
        p.rect_filled(r, 0.0, fill);
        p.rect_stroke(r, 0.0, Stroke::new(1.0, border), StrokeKind::Inside);
        p.text(r.center(), egui::Align2::CENTER_CENTER, *l, font.clone(), Color32::BLACK);
        if resp.clicked() {
            *selected = i;
        }
        x += w;
    }
    p.line_segment([Pos2::new(strip.left(), bottom), Pos2::new(strip.right(), bottom)], Stroke::new(1.0, border));
    if scroll {
        for (k, glyph) in [(0, "◀"), (1, "▶")] {
            let r = Rect::from_min_size(Pos2::new(strip.right() - 34.0 + k as f32 * 17.0, bottom - 17.0), Vec2::new(16.0, 16.0));
            let resp = ui.interact(r, ui.scope_id().with(id).with("arrow").with(k), Sense::click());
            p.rect_filled(r, 0.0, Color32::from_rgb(0xf0, 0xf0, 0xf0));
            p.rect_stroke(r, 0.0, Stroke::new(1.0, Color32::from_rgb(0xad, 0xad, 0xad)), StrokeKind::Inside);
            let c = r.center();
            let tri = if glyph == "◀" {
                vec![Pos2::new(c.x - 3.0, c.y), Pos2::new(c.x + 2.0, c.y - 4.0), Pos2::new(c.x + 2.0, c.y + 4.0)]
            } else {
                vec![Pos2::new(c.x + 3.0, c.y), Pos2::new(c.x - 2.0, c.y - 4.0), Pos2::new(c.x - 2.0, c.y + 4.0)]
            };
            p.add(Shape::convex_polygon(tri, Color32::BLACK, Stroke::NONE));
            if resp.clicked() {
                if k == 0 {
                    first = first.saturating_sub(1);
                } else if first + 1 < labels.len() {
                    first += 1;
                }
            }
        }
    }
    ui.ctx().data_mut(|d| d.insert_temp(first_id, first));
}

// ------------------------------------------------------------------ BackstageView

pub struct BsvItem {
    pub label: String,
    /// Index of the parent item, for child pages.
    pub parent: Option<usize>,
}

pub const BSV_WIDTH: f32 = 172.0;

/// `BackstageView` menu: black button area, 30px bold items, green gradient selection with the
/// arrow pointing at the content page. Children are shown while their parent or a sibling is
/// selected, prefixed like the original (">> " for parents, indented children).
pub fn backstage(ui: &mut Ui, items: &[BsvItem], selected: &mut usize) {
    let rect = ui.max_rect();
    let p = ui.painter();
    p.rect_filled(rect, 0.0, Color32::BLACK);
    let grad = Rect::from_min_max(Pos2::new(rect.right() - 20.0, rect.top()), rect.right_bottom());
    hgradient(p, grad, Color32::BLACK, BG);
    p.line_segment([rect.right_top() - Vec2::new(1.0, 0.0), rect.right_bottom() - Vec2::new(1.0, 0.0)], Stroke::new(1.0, DARK_GRAY));

    let sel_parent = items.get(*selected).and_then(|i| i.parent).unwrap_or(*selected);
    let has_child = |i: usize| items.iter().any(|c| c.parent == Some(i));
    let font = fonts::bold_id(13.9);
    let mut y = rect.top();
    for (i, it) in items.iter().enumerate() {
        if let Some(par) = it.parent {
            if par != sel_parent {
                continue;
            }
        }
        let label = if has_child(i) {
            format!(">> {}", it.label)
        } else if it.parent.is_some() {
            format!("      {}", it.label)
        } else {
            it.label.clone()
        };
        let r = Rect::from_min_size(Pos2::new(rect.left(), y), Vec2::new(rect.width(), 30.0));
        let resp = ui.interact(r, ui.scope_id().with("bsv").with(i), Sense::click());
        let p = ui.painter();
        if i == *selected {
            hgradient(p, r, BANNER2, BANNER1);
            p.line_segment([r.left_top(), r.right_top()], Stroke::new(1.0, BANNER1));
            p.line_segment([r.left_bottom() - Vec2::new(0.0, 1.0), r.right_bottom() - Vec2::new(0.0, 1.0)], Stroke::new(1.0, BANNER1));
            p.text(r.left_top() + Vec2::new(5.0, 6.0), egui::Align2::LEFT_TOP, &label, font.clone(), Color32::WHITE);
            p.line_segment([r.right_top() - Vec2::new(1.0, 0.0), r.right_bottom() - Vec2::new(1.0, 1.0)], Stroke::new(1.0, DARK_GRAY));
            let mid = r.center().y;
            let pts = vec![Pos2::new(r.right(), mid + 8.0), Pos2::new(r.right() - 8.0, mid), Pos2::new(r.right(), mid - 8.0)];
            p.add(Shape::convex_polygon(pts.clone(), BG, Stroke::NONE));
            p.add(Shape::closed_line(pts, Stroke::new(1.0, DARK_GRAY)));
        } else {
            if resp.hovered() {
                p.rect_filled(r, 0.0, Color32::from_rgba_unmultiplied(0xa0, 0xa0, 0xa0, 10));
                p.line_segment([r.left_top(), r.right_top()], Stroke::new(1.0, DARK_GRAY));
                p.line_segment([r.left_bottom() - Vec2::new(0.0, 1.0), r.right_bottom() - Vec2::new(0.0, 1.0)], Stroke::new(1.0, DARK_GRAY));
            }
            p.text(r.left_top() + Vec2::new(5.0, 6.0), egui::Align2::LEFT_TOP, &label, font.clone(), WHITE_SMOKE);
        }
        if resp.clicked() {
            // A parent with children opens its first child, like BackstageView.
            *selected = if has_child(i) { items.iter().position(|c| c.parent == Some(i)).unwrap_or(i) } else { i };
        }
        y += 30.0;
    }
}

// ------------------------------------------------------------------ MyTrackBar (vertical)

/// Vertical TrackBar (Zoom slider on the maps). Top = max.
pub fn vtrackbar(ui: &mut Ui, rect: Rect, value: &mut f64, min: f64, max: f64) -> Response {
    let resp = ui.interact(rect, ui.scope_id().with("trk").with(rect.left() as i32), Sense::click_and_drag());
    let p = ui.painter();
    let groove = Rect::from_center_size(Pos2::new(rect.center().x, rect.center().y), Vec2::new(4.0, rect.height() - 16.0));
    p.rect_filled(groove, 0.0, Color32::from_rgb(0xe7, 0xea, 0xea));
    p.rect_stroke(groove, 0.0, Stroke::new(1.0, Color32::from_rgb(0xd6, 0xd6, 0xd6)), StrokeKind::Inside);
    let steps = (max - min).round().max(1.0) as i32;
    for s in 0..=steps {
        let y = groove.bottom() - (s as f32 / steps as f32) * groove.height();
        p.line_segment([Pos2::new(groove.right() + 6.0, y), Pos2::new(groove.right() + 10.0, y)], Stroke::new(1.0, Color32::from_rgb(0xc4, 0xc4, 0xc4)));
    }
    if let Some(pos) = resp.interact_pointer_pos() {
        if resp.dragged() || resp.clicked() {
            let f = ((groove.bottom() - pos.y) / groove.height()).clamp(0.0, 1.0) as f64;
            *value = min + f * (max - min);
        }
    }
    let f = ((*value - min) / (max - min)).clamp(0.0, 1.0) as f32;
    let ty = groove.bottom() - f * groove.height();
    let thumb = Rect::from_center_size(Pos2::new(rect.center().x, ty), Vec2::new(20.0, 10.0));
    p.rect_filled(thumb, 0.0, if resp.hovered() { Color32::from_rgb(0x1c, 0x97, 0xea) } else { Color32::from_rgb(0x00, 0x7a, 0xd9) });
    resp
}

/// WinForms Label: plain white text at the UI font.
pub fn label(ui: &mut Ui, text: impl Into<String>) -> Response {
    ui.add(egui::Label::new(egui::RichText::new(text.into()).color(TEXT)).selectable(false))
}

/// HorizontalProgressBar2 styling from the theme.
pub fn progress(ui: &mut Ui, frac: f32, size: Vec2, text: &str) {
    let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
    let p = ui.painter();
    p.rect_filled(rect, 0.0, BG);
    let fill = Rect::from_min_size(rect.min, Vec2::new(rect.width() * frac.clamp(0.0, 1.0), rect.height()));
    vgradient(p, fill, PROGRESS_TOP, PROGRESS_BOT);
    p.rect_stroke(rect, 0.0, Stroke::new(1.0, PROGRESS_OUTLINE), StrokeKind::Inside);
    p.text(rect.center(), egui::Align2::CENTER_CENTER, text, fonts::ui_id(fonts::UI_SIZE), TEXT);
}
