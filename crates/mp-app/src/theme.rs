//! BurntKermit theme (BurntKermit.mpsystheme + ThemeManager.cs) and the custom-drawn controls
//! Mission Planner uses: MyButton, the WinForms tab strip, BackstageView and MyTrackBar.

use egui::epaint::{Mesh, Vertex, WHITE_UV};
use egui::{
    Align, Color32, CornerRadius, FontId, Pos2, Rect, Response, Sense, Shape, Stroke, StrokeKind,
    Ui, Vec2, Visuals,
};

use crate::fonts;

// ThemeManager colours (BurntKermit.mpsystheme)
/// SystemColors.ControlLight: menu item text.
pub const MENU_TEXT: Color32 = Color32::from_rgb(0xe3, 0xe3, 0xe3);
pub const WHITE_SMOKE: Color32 = Color32::from_rgb(0xf5, 0xf5, 0xf5);
pub const DARK_GRAY: Color32 = Color32::from_rgb(0xa9, 0xa9, 0xa9);

/// ThemeManager colours (one `.mpsystheme`), swapped at runtime by the Planner theme setting.
#[derive(Debug, Clone, Copy)]
pub struct Palette {
    pub bg: Color32,
    pub control_bg: Color32,
    pub text: Color32,
    pub textbox_bg: Color32,
    pub but_text: Color32,
    pub but_top: Color32,
    pub but_bot: Color32,
    pub but_outline: Color32,
    pub progress_top: Color32,
    pub progress_bot: Color32,
    pub progress_outline: Color32,
    pub banner1: Color32,
    pub banner2: Color32,
    pub not_enabled: Color32,
    pub mouse_over: Color32,
    pub mouse_down: Color32,
}

impl Palette {
    pub const BURNT_KERMIT: Palette = Palette {
        bg: Color32::from_rgb(0x26, 0x27, 0x28),
        control_bg: Color32::from_rgb(0x43, 0x44, 0x45),
        text: Color32::WHITE,
        textbox_bg: Color32::from_rgb(0x43, 0x44, 0x45),
        but_text: Color32::from_rgb(0x40, 0x57, 0x04),
        but_top: Color32::from_rgb(0x94, 0xc1, 0x1f),
        but_bot: Color32::from_rgb(0xcd, 0xe2, 0x96),
        but_outline: Color32::from_rgb(0x79, 0x94, 0x29),
        progress_top: Color32::from_rgb(0x66, 0x8b, 0x1a),
        progress_bot: Color32::from_rgb(0x7c, 0xa4, 0x28),
        progress_outline: Color32::from_rgb(0x96, 0xae, 0x70),
        banner1: Color32::from_rgb(0x40, 0x57, 0x04),
        banner2: Color32::from_rgb(0x94, 0xc1, 0x1f),
        not_enabled: Color32::from_rgba_unmultiplied_const(0x2b, 0x3a, 0x03, 150),
        mouse_over: Color32::from_rgba_unmultiplied_const(0x2b, 0x3a, 0x03, 73),
        mouse_down: Color32::from_rgba_unmultiplied_const(0x2b, 0x3a, 0x03, 150),
    };

    /// The editable entries (ThemeColor strColorItemName, strVariableName).
    pub const ITEMS: [(&'static str, &'static str); 15] = [
        ("Background", "BGColor"),
        ("Control Background", "ControlBGColor"),
        ("Text", "TextColor"),
        ("TextBox Background", "BGColorTextBox"),
        ("Button Text", "ButtonTextColor"),
        ("Button Background top", "ButBG"),
        ("Button Background bottom", "ButBGBot"),
        ("ProgressBar Top", "ProgressBarColorTop"),
        ("ProgressBar Bottom", "ProgressBarColorBot"),
        ("ProgressBar Outline", "ProgressBarOutlineColor"),
        ("BannerColor1", "BannerColor1"),
        ("BannerColor2", "BannerColor2"),
        ("Disabled Button", "ColorNotEnabled"),
        ("Mouse Over", "ColorMouseOver"),
        ("Mouse Down", "ColorMouseDown"),
    ];

    pub fn get_mut(&mut self, var: &str) -> Option<&mut Color32> {
        Some(match var {
            "BGColor" => &mut self.bg,
            "ControlBGColor" => &mut self.control_bg,
            "TextColor" => &mut self.text,
            "BGColorTextBox" => &mut self.textbox_bg,
            "ButtonTextColor" => &mut self.but_text,
            "ButBG" => &mut self.but_top,
            "ButBGBot" => &mut self.but_bot,
            "ProgressBarColorTop" => &mut self.progress_top,
            "ProgressBarColorBot" => &mut self.progress_bot,
            "ProgressBarOutlineColor" => &mut self.progress_outline,
            "BannerColor1" => &mut self.banner1,
            "BannerColor2" => &mut self.banner2,
            "ColorNotEnabled" => &mut self.not_enabled,
            "ColorMouseOver" => &mut self.mouse_over,
            "ColorMouseDown" => &mut self.mouse_down,
            _ => return None,
        })
    }

    /// Parse a ThemeColorTable (.mpsystheme) over the BurntKermit defaults.
    pub fn from_mpsystheme(xml: &str) -> Palette {
        let mut p = Palette::BURNT_KERMIT;
        let Ok(doc) = roxmltree::Document::parse(xml.trim_start_matches('\u{feff}')) else { return p };
        for c in doc.descendants().filter(|n| n.has_tag_name("ThemeColor")) {
            let var = c.children().find(|n| n.has_tag_name("strVariableName")).and_then(|n| n.text()).unwrap_or("");
            let web = c.children().find(|n| n.has_tag_name("clrColor")).and_then(|n| n.attribute("Web")).unwrap_or("");
            if let (Some(slot), Some(col)) = (p.get_mut(var), parse_web(web)) {
                // the overlay colours keep their translucency
                let a = slot.a();
                *slot = if a < 255 { Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), a) } else { col };
            }
        }
        // MyButton border: a darker shade of the bottom gradient colour
        p.but_outline = Color32::from_rgb((p.but_top.r() as u16 * 4 / 5) as u8, (p.but_top.g() as u16 * 4 / 5) as u8, (p.but_top.b() as u16 * 4 / 5) as u8);
        p
    }

    pub fn to_mpsystheme(&self, name: &str) -> String {
        let mut p = *self;
        let mut s = format!("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<ThemeColorTable xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">\n  <colors>\n");
        for (label, var) in Palette::ITEMS {
            let c = *p.get_mut(var).unwrap();
            s.push_str(&format!("    <ThemeColor>\n      <strColorItemName>{label}</strColorItemName>\n      <clrColor Web=\"#{:02X}{:02X}{:02X}\" />\n      <strVariableName>{var}</strVariableName>\n    </ThemeColor>\n", c.r(), c.g(), c.b()));
        }
        s.push_str(&format!("  </colors>\n  <strThemeName>{name}</strThemeName>\n</ThemeColorTable>\n"));
        s
    }
}

/// .NET Color web names used by the theme files.
fn parse_web(w: &str) -> Option<Color32> {
    if let Some(h) = w.strip_prefix('#') {
        let v = u32::from_str_radix(h, 16).ok()?;
        return Some(Color32::from_rgb((v >> 16) as u8, (v >> 8) as u8, v as u8));
    }
    Some(match w {
        "White" => Color32::WHITE,
        "Black" => Color32::BLACK,
        "Green" => Color32::from_rgb(0, 128, 0),
        "Blue" => Color32::from_rgb(0, 0, 255),
        "LightBlue" => Color32::from_rgb(0xad, 0xd8, 0xe6),
        "LightGray" => Color32::from_rgb(0xd3, 0xd3, 0xd3),
        "WhiteSmoke" => Color32::from_rgb(0xf5, 0xf5, 0xf5),
        "Gray" => Color32::GRAY,
        "Red" => Color32::RED,
        "Yellow" => Color32::YELLOW,
        _ => return None,
    })
}

static PALETTE: std::sync::RwLock<Palette> = std::sync::RwLock::new(Palette::BURNT_KERMIT);

pub fn palette() -> Palette {
    *PALETTE.read().unwrap()
}

/// ThemeManager.LoadTheme + ApplyThemeTo: switch palette and restyle egui.
pub fn set_palette(ctx: &egui::Context, p: Palette) {
    *PALETTE.write().unwrap() = p;
    apply(ctx);
}

/// Planner theme names -> palette ("Custom" reads custom.mpsystheme from the data folder).
pub fn palette_for(name: &str) -> Palette {
    match name {
        "HighContrast" => Palette::from_mpsystheme(include_str!("../assets/HighContrast.mpsystheme")),
        "Custom" => std::fs::read_to_string(crate::platform::data_dir().join("custom.mpsystheme")).map(|s| Palette::from_mpsystheme(&s)).unwrap_or(Palette::BURNT_KERMIT),
        _ => Palette::BURNT_KERMIT,
    }
}

pub fn bg() -> Color32 { palette().bg }
pub fn control_bg() -> Color32 { palette().control_bg }
pub fn text() -> Color32 { palette().text }
pub fn but_top() -> Color32 { palette().but_top }
pub fn but_bot() -> Color32 { palette().but_bot }
pub fn but_text() -> Color32 { palette().but_text }
pub fn but_outline() -> Color32 { palette().but_outline }
pub fn mouse_over() -> Color32 { palette().mouse_over }
pub fn mouse_down() -> Color32 { palette().mouse_down }
pub fn not_enabled() -> Color32 { palette().not_enabled }
pub fn banner1() -> Color32 { palette().banner1 }
pub fn banner2() -> Color32 { palette().banner2 }
pub fn progress_top() -> Color32 { palette().progress_top }
pub fn progress_bot() -> Color32 { palette().progress_bot }
pub fn progress_outline() -> Color32 { palette().progress_outline }
pub fn green() -> Color32 { palette().but_top }
pub fn green_light() -> Color32 { palette().but_bot }

pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
    let mut v = Visuals::dark();
    v.panel_fill = bg();
    v.window_fill = bg();
    v.window_stroke = Stroke::new(1.0, DARK_GRAY);
    v.window_corner_radius = CornerRadius::ZERO;
    v.menu_corner_radius = CornerRadius::ZERO;
    v.extreme_bg_color = control_bg();
    v.text_edit_bg_color = Some(control_bg());
    v.faint_bg_color = bg();
    v.override_text_color = Some(text());
    v.selection.bg_fill = Color32::from_rgb(0x33, 0x99, 0xff);
    v.selection.stroke = Stroke::new(1.0, Color32::WHITE);
    v.hyperlink_color = Color32::from_rgb(0x66, 0xb2, 0xff);
    let r = CornerRadius::ZERO;
    let border = Color32::from_rgb(0x7a, 0x7a, 0x7a);
    for w in [&mut v.widgets.noninteractive, &mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
        w.corner_radius = r;
        w.expansion = 0.0;
    }
    v.widgets.noninteractive.bg_fill = bg();
    v.widgets.noninteractive.weak_bg_fill = bg();
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x55, 0x56, 0x57));
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, text());
    v.widgets.inactive.bg_fill = control_bg();
    v.widgets.inactive.weak_bg_fill = control_bg();
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, border);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, text());
    v.widgets.hovered.bg_fill = control_bg();
    v.widgets.hovered.weak_bg_fill = control_bg();
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x33, 0x99, 0xff));
    v.widgets.hovered.fg_stroke = Stroke::new(1.0, text());
    v.widgets.active.bg_fill = control_bg();
    v.widgets.active.weak_bg_fill = control_bg();
    v.widgets.active.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x33, 0x99, 0xff));
    v.widgets.active.fg_stroke = Stroke::new(1.0, text());
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
    let text = &*crate::i18n::tr(text);
    let (rect, resp) = ui.allocate_exact_size(size, Sense::click());
    if ui.is_rect_visible(rect) {
        let p = ui.painter();
        vgradient(p, rect, but_top(), but_bot());
        p.rect_stroke(rect.shrink(0.5), CornerRadius::same(1), Stroke::new(1.0, but_outline()), StrokeKind::Inside);
        if !ui.is_enabled() {
            p.rect_filled(rect, 0.0, not_enabled());
        } else if resp.is_pointer_button_down_on() {
            p.rect_filled(rect, 0.0, mouse_down());
        } else if resp.hovered() {
            p.rect_filled(rect, 0.0, mouse_over());
        }
        centered_text(p, rect, text, fonts::ui_id(fonts::UI_SIZE), but_text());
    }
    resp
}

/// MyButton sized to its text (WinForms default 75x23 minimum).
pub fn button(ui: &mut Ui, text: &str) -> Response {
    let text = &*crate::i18n::tr(text);
    let w = ui.painter().layout_no_wrap(text.to_string(), fonts::ui_id(fonts::UI_SIZE), self::text()).size().x + 16.0;
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
    let owned: Vec<String> = labels.iter().map(|l| crate::i18n::t(l)).collect();
    let labels: Vec<&str> = owned.iter().map(String::as_str).collect();
    let labels = labels.as_slice();
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

/// TabControl with Multiline = true: tabs wrap onto extra rows instead of scrolling.
/// Returns the strip rect (for its context menu).
pub fn tab_strip_multiline(ui: &mut Ui, id: &str, labels: &[&str], selected: &mut usize) -> Rect {
    let owned: Vec<String> = labels.iter().map(|l| crate::i18n::t(l)).collect();
    let labels: Vec<&str> = owned.iter().map(String::as_str).collect();
    let labels = labels.as_slice();
    let font = fonts::ui_id(fonts::UI_SIZE);
    let h = 21.0;
    let full = ui.available_width();
    let widths: Vec<f32> = labels.iter().map(|l| ui.painter().layout_no_wrap(l.to_string(), font.clone(), Color32::BLACK).size().x + 12.0).collect();
    let mut rows: Vec<Vec<usize>> = vec![Vec::new()];
    let mut x = 0.0;
    for (i, w) in widths.iter().enumerate() {
        if x + w > full - 4.0 && !rows.last().unwrap().is_empty() {
            rows.push(Vec::new());
            x = 0.0;
        }
        rows.last_mut().unwrap().push(i);
        x += w;
    }
    let (strip, _) = ui.allocate_exact_size(Vec2::new(full, h * rows.len() as f32 + 2.0), Sense::hover());
    let p = ui.painter().with_clip_rect(strip);
    let border = Color32::from_rgb(0xd9, 0xd9, 0xd9);
    for (ri, row) in rows.iter().enumerate() {
        let mut x = strip.left() + 2.0;
        let top = strip.top() + ri as f32 * h;
        for &i in row {
            let r = Rect::from_min_size(Pos2::new(x, top), Vec2::new(widths[i], h));
            let resp = ui.interact(r, ui.scope_id().with(id).with(("ml", i)), Sense::click());
            let sel = i == *selected;
            let fill = if sel { Color32::WHITE } else if resp.hovered() { Color32::from_rgb(0xe5, 0xf1, 0xfb) } else { Color32::from_rgb(0xf0, 0xf0, 0xf0) };
            p.rect_filled(r, 0.0, fill);
            p.rect_stroke(r, 0.0, Stroke::new(1.0, border), StrokeKind::Inside);
            p.text(r.center(), egui::Align2::CENTER_CENTER, labels[i], font.clone(), Color32::BLACK);
            if resp.clicked() {
                *selected = i;
            }
            x += widths[i];
        }
    }
    strip
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
    hgradient(p, grad, Color32::BLACK, bg());
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
        let name = crate::i18n::t(&it.label);
        let label = if has_child(i) {
            format!(">> {name}")
        } else if it.parent.is_some() {
            format!("      {name}")
        } else {
            name
        };
        let r = Rect::from_min_size(Pos2::new(rect.left(), y), Vec2::new(rect.width(), 30.0));
        let resp = ui.interact(r, ui.scope_id().with("bsv").with(i), Sense::click());
        let p = ui.painter();
        if i == *selected {
            hgradient(p, r, banner2(), banner1());
            p.line_segment([r.left_top(), r.right_top()], Stroke::new(1.0, banner1()));
            p.line_segment([r.left_bottom() - Vec2::new(0.0, 1.0), r.right_bottom() - Vec2::new(0.0, 1.0)], Stroke::new(1.0, banner1()));
            p.text(r.left_top() + Vec2::new(5.0, 6.0), egui::Align2::LEFT_TOP, &label, font.clone(), Color32::WHITE);
            p.line_segment([r.right_top() - Vec2::new(1.0, 0.0), r.right_bottom() - Vec2::new(1.0, 1.0)], Stroke::new(1.0, DARK_GRAY));
            let mid = r.center().y;
            let pts = vec![Pos2::new(r.right(), mid + 8.0), Pos2::new(r.right() - 8.0, mid), Pos2::new(r.right(), mid - 8.0)];
            p.add(Shape::convex_polygon(pts.clone(), bg(), Stroke::NONE));
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

/// Horizontal WinForms TrackBar (TickStyle BottomRight): groove, blue thumb, `ticks` ticks.
pub fn htrackbar(ui: &mut Ui, rect: Rect, value: &mut f64, min: f64, max: f64, ticks: usize) -> Response {
    let before = *value;
    let mut resp = ui.interact(rect, ui.scope_id().with("htrk").with((rect.left() as i32, rect.top() as i32)), Sense::click_and_drag());
    let p = ui.painter();
    let groove = Rect::from_min_size(Pos2::new(rect.left() + 13.0, rect.top() + 10.0), Vec2::new(rect.width() - 26.0, 4.0));
    p.rect_filled(groove, 0.0, Color32::from_rgb(0xe7, 0xea, 0xea));
    p.rect_stroke(groove, 0.0, Stroke::new(1.0, Color32::from_rgb(0xd6, 0xd6, 0xd6)), StrokeKind::Inside);
    let n = ticks.max(1);
    for s in 0..=n {
        let x = groove.left() + (s as f32 / n as f32) * groove.width();
        p.line_segment([Pos2::new(x, groove.bottom() + 12.0), Pos2::new(x, groove.bottom() + 16.0)], Stroke::new(1.0, Color32::from_rgb(0xc4, 0xc4, 0xc4)));
    }
    let span = if max > min { max - min } else { 1.0 };
    if let Some(pos) = resp.interact_pointer_pos() {
        if ui.is_enabled() && (resp.dragged() || resp.clicked()) {
            let f = ((pos.x - groove.left()) / groove.width()).clamp(0.0, 1.0) as f64;
            *value = min + f * span;
        }
    }
    let f = ((*value - min) / span).clamp(0.0, 1.0) as f32;
    let tx = groove.left() + f * groove.width();
    let thumb = Rect::from_center_size(Pos2::new(tx, groove.center().y + 2.0), Vec2::new(11.0, 21.0));
    p.rect_filled(thumb, 0.0, if resp.hovered() || resp.dragged() { Color32::from_rgb(0x1c, 0x97, 0xea) } else { Color32::from_rgb(0x00, 0x7a, 0xd9) });
    if *value != before { resp.mark_changed(); }
    resp
}

pub fn vtrackbar_ticks(ui: &mut Ui, rect: Rect, value: &mut f64, min: f64, max: f64, ticks: usize) -> Response {
    let before = *value;
    let mut resp = ui.interact(rect, ui.scope_id().with("vtrk").with((rect.left() as i32, rect.top() as i32)), Sense::click_and_drag());
    let p = ui.painter();
    let groove = Rect::from_min_size(rect.min + Vec2::new(10.0, 13.0), Vec2::new(4.0, (rect.height() - 26.0).max(1.0)));
    p.rect_filled(groove, 0.0, Color32::from_rgb(0xe7, 0xea, 0xea));
    let n = ticks.max(1);
    for s in 0..=n {
        let y = groove.top() + s as f32 / n as f32 * groove.height();
        p.line_segment([Pos2::new(groove.right() + 12.0, y), Pos2::new(groove.right() + 16.0, y)], Stroke::new(1.0, Color32::from_rgb(0xc4, 0xc4, 0xc4)));
    }
    let span = if max > min { max - min } else { 1.0 };
    if let Some(pos) = resp.interact_pointer_pos() {
        if ui.is_enabled() && (resp.dragged() || resp.clicked()) { *value = min + ((groove.bottom() - pos.y) / groove.height()).clamp(0.0, 1.0) as f64 * span; }
    }
    let y = groove.bottom() - ((*value - min) / span).clamp(0.0, 1.0) as f32 * groove.height();
    let thumb = Rect::from_center_size(Pos2::new(groove.center().x + 2.0, y), Vec2::new(21.0, 11.0));
    p.rect_filled(thumb, 0.0, if resp.hovered() || resp.dragged() { Color32::from_rgb(0x1c, 0x97, 0xea) } else { Color32::from_rgb(0x00, 0x7a, 0xd9) });
    if *value != before { resp.mark_changed(); }
    resp
}

/// WinForms Label: plain white text at the UI font.
pub fn label(ui: &mut Ui, text: impl Into<String>) -> Response {
    let text = crate::i18n::t(&text.into());
    ui.add(egui::Label::new(egui::RichText::new(text).color(self::text())).selectable(false))
}

/// HorizontalProgressBar2 styling from the theme.
pub fn progress(ui: &mut Ui, frac: f32, size: Vec2, text: &str) {
    let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
    let p = ui.painter();
    p.rect_filled(rect, 0.0, bg());
    let fill = Rect::from_min_size(rect.min, Vec2::new(rect.width() * frac.clamp(0.0, 1.0), rect.height()));
    vgradient(p, fill, progress_top(), progress_bot());
    p.rect_stroke(rect, 0.0, Stroke::new(1.0, progress_outline()), StrokeKind::Inside);
    p.text(rect.center(), egui::Align2::CENTER_CENTER, text, fonts::ui_id(fonts::UI_SIZE), self::text());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trackbars_report_pointer_changes_in_both_orientations() {
        for vertical in [false, true] {
            let ctx = egui::Context::default();
            let mut value = 1500.0;
            let rect = Rect::from_min_size(Pos2::new(20.0, 20.0), if vertical { Vec2::new(45.0, 154.0) } else { Vec2::new(144.0, 45.0) });
            let pos = if vertical { Pos2::new(35.0, 42.0) } else { Pos2::new(145.0, 32.0) };
            let mut changed = false;
            for events in [Vec::new(), vec![egui::Event::PointerMoved(pos), egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: true, modifiers: egui::Modifiers::NONE }], vec![egui::Event::PointerMoved(pos), egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: false, modifiers: egui::Modifiers::NONE }]] {
                let mut output = ctx.run_ui(egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::splat(200.0))), events, ..Default::default() }, |ui| {
                    let response = if vertical { vtrackbar_ticks(ui, rect, &mut value, 1000.0, 2000.0, 10) } else { htrackbar(ui, rect, &mut value, 1000.0, 2000.0, 10) };
                    changed |= response.changed();
                });
                output.textures_delta.clear();
            }
            assert!(changed && value > 1750.0, "vertical={vertical}: value={value}, changed={changed}");
        }
    }

    #[test]
    fn high_contrast_theme_parses() {
        let p = Palette::from_mpsystheme(include_str!("../assets/HighContrast.mpsystheme"));
        assert_eq!(p.bg, Color32::from_rgb(0xee, 0xee, 0xee));
        assert_eq!(p.text, Color32::BLACK);
        assert_eq!(p.but_top, Color32::from_rgb(0xff, 0xff, 0x99));
        let back = Palette::from_mpsystheme(&p.to_mpsystheme("Custom"));
        assert_eq!(back.control_bg, p.control_bg);
    }
}

// ------------------------------------------------------------------ NumericUpDown

/// Decimal places Mission Planner shows for an increment (MavlinkNumericUpDown.setup).
pub fn decimals_for(increment: f64) -> usize {
    let mut d = 0;
    let mut inc = increment.abs();
    while d < 6 && inc > 0.0 && (inc - inc.round()).abs() > 1e-9 {
        inc *= 10.0;
        d += 1;
    }
    d
}

/// WinForms NumericUpDown: a text box with up/down arrows on the right. `changed()` is set
/// when the value is committed (Enter, focus lost, an arrow click or the mouse wheel).
pub fn numeric_updown(ui: &mut Ui, rect: Rect, value: &mut f64, min: f64, max: f64, increment: f64, decimals: usize) -> Response {
    numeric_updown_fill(ui, rect, value, min, max, increment, decimals, control_bg())
}

pub fn numeric_updown_fill(ui: &mut Ui, rect: Rect, value: &mut f64, min: f64, max: f64, increment: f64, decimals: usize, fill: Color32) -> Response {
    let id = ui.id().with(("nud", (rect.left() as i32, rect.top() as i32)));
    let inc = if increment > 0.0 { increment } else { 1.0 };
    let p = ui.painter().clone();
    let enabled = ui.is_enabled();
    p.rect_filled(rect, 0.0, fill);
    p.rect_stroke(rect, 0.0, Stroke::new(1.0, Color32::from_gray(0x80)), StrokeKind::Inside);
    let arrows = Rect::from_min_max(Pos2::new(rect.right() - 17.0, rect.top() + 1.0), rect.max - Vec2::new(1.0, 1.0));
    let up = Rect::from_min_max(arrows.min, Pos2::new(arrows.right(), arrows.center().y));
    let down = Rect::from_min_max(Pos2::new(arrows.left(), arrows.center().y), arrows.max);
    let fmt = |v: f64| format!("{v:.decimals$}");
    let mut buf: String = ui.data(|d| d.get_temp::<String>(id)).unwrap_or_else(|| fmt(*value));
    let text_rect = Rect::from_min_max(rect.min + Vec2::new(2.0, 1.0), Pos2::new(arrows.left() - 1.0, rect.bottom() - 1.0));
    let te = ui.put(text_rect, egui::TextEdit::singleline(&mut buf).frame(egui::Frame::NONE).text_color(self::text()).vertical_align(Align::Center));
    let mut commit = None;
    if te.has_focus() {
        ui.data_mut(|d| d.insert_temp(id, buf.clone()));
        let wheel = ui.input(|i| i.smooth_scroll_delta.y);
        if wheel != 0.0 {
            commit = Some(buf.trim().parse::<f64>().unwrap_or(*value) + inc * wheel.signum() as f64);
        }
    } else {
        ui.data_mut(|d| d.remove::<String>(id));
    }
    if te.lost_focus() {
        commit = buf.trim().parse::<f64>().ok().or(Some(*value));
    }
    for (r, dir) in [(up, 1.0), (down, -1.0)] {
        let resp = ui.interact(r, id.with(dir as i32), Sense::click());
        let fill = if resp.is_pointer_button_down_on() { mouse_down() } else if resp.hovered() { mouse_over() } else { bg() };
        p.rect_filled(r, 0.0, fill);
        let c = r.center();
        let col = if enabled { self::text() } else { Color32::GRAY };
        let tri = if dir > 0.0 { vec![c + Vec2::new(-3.0, 1.5), c + Vec2::new(3.0, 1.5), c + Vec2::new(0.0, -1.5)] } else { vec![c + Vec2::new(-3.0, -1.5), c + Vec2::new(3.0, -1.5), c + Vec2::new(0.0, 1.5)] };
        p.add(egui::Shape::convex_polygon(tri, col, Stroke::NONE));
        if resp.clicked() {
            commit = Some(*value + inc * dir);
        }
    }
    let mut out = te;
    if let Some(v) = commit {
        let v = v.clamp(min.min(max), max.max(min));
        let rounded = (v * 10f64.powi(decimals as i32)).round() / 10f64.powi(decimals as i32);
        ui.data_mut(|d| d.remove::<String>(id));
        if (rounded - *value).abs() > 1e-12 {
            *value = rounded;
            out.mark_changed();
        }
    }
    out
}

/// MavlinkNumericUpDown.Value: DecimalPlaces grows to the value's own decimals.
pub fn value_decimals(v: f32) -> usize {
    format!("{v}").split_once('.').map(|(_, f)| f.len()).unwrap_or(0).min(6)
}
