//! Port of `ExtLibs/Controls/MyButton.cs`
//!
//! Mission Planner's button: a vertical BGGradTop -> BGGradBot gradient inside a 1 px Outline
//! path, then (in this order, each painted over the last) the mouse-over tint, the mouse-down
//! tint and the not-enabled tint, then the text centred both ways with its first '&' removed.
//! ThemeManager may replace the colours; `theme` passes the themed ones in.

#![allow(non_snake_case)]

use egui::{Color32, CornerRadius, FontId, Rect, Stroke, StrokeKind};

/// `public class MyButton : Button`: the colour properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MyButton {
    pub BGGradTop: Color32,
    pub BGGradBot: Color32,
    pub TextColor: Color32,
    /// `TextColorNotEnabled`: empty (None) falls back to `TextColor`
    pub _TextColorNotEnabled: Option<Color32>,
    pub Outline: Color32,
    pub ColorNotEnabled: Color32,
    pub ColorMouseOver: Color32,
    pub ColorMouseDown: Color32,
}

/// `Color.FromArgb(a, r, g, b)`
const fn argb(a: u8, r: u8, g: u8, b: u8) -> Color32 {
    Color32::from_rgba_unmultiplied_const(r, g, b, a)
}

impl Default for MyButton {
    /// `new MyButton()`
    fn default() -> Self {
        MyButton {
            BGGradTop: Color32::from_rgb(0x94, 0xc1, 0x1f),
            BGGradBot: Color32::from_rgb(0xcd, 0xe2, 0x96),
            TextColor: Color32::from_rgb(0x40, 0x57, 0x04),
            _TextColorNotEnabled: None,
            Outline: Color32::from_rgb(0x79, 0x94, 0x29),
            ColorNotEnabled: argb(73, 0x2b, 0x3a, 0x03),
            ColorMouseOver: argb(73, 0x2b, 0x3a, 0x03),
            ColorMouseDown: argb(150, 0x2b, 0x3a, 0x03),
        }
    }
}

/// The mouse state `OnMouseEnter/Leave/Down/Up` track, and `Enabled`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ButtonState {
    pub mouseover: bool,
    pub mousedown: bool,
    pub enabled: bool,
}

impl MyButton {
    /// `TextColorNotEnabled`
    pub fn TextColorNotEnabled(&self) -> Color32 {
        self._TextColorNotEnabled.unwrap_or(self.TextColor)
    }

    /// The label as drawn: the first '&' removed.
    pub fn display_text(text: &str) -> String {
        match text.find('&') {
            Some(i) => {
                let mut s = text.to_string();
                s.remove(i);
                s
            }
            None => text.to_string(),
        }
    }

    /// `OnPaint` into `rect`.
    pub fn OnPaint(&self, painter: &egui::Painter, rect: Rect, text: &str, font: FontId, state: ButtonState) {
        // the outline path runs on (0,0)-(Width-1,Height-1) with 1 px corner arcs
        let path = Rect::from_min_max(rect.min, rect.max - egui::vec2(1.0, 1.0));
        vgradient(painter, path, self.BGGradTop, self.BGGradBot);
        painter.rect_stroke(path.translate(egui::vec2(0.5, 0.5)).shrink(0.5).expand(0.5), CornerRadius::same(1), Stroke::new(1.0, self.Outline), StrokeKind::Middle);
        let brush = if state.enabled { self.TextColor } else { self.TextColorNotEnabled() };
        if state.mouseover {
            painter.rect_filled(path, CornerRadius::same(1), self.ColorMouseOver);
        }
        if state.mousedown {
            painter.rect_filled(path, CornerRadius::same(1), self.ColorMouseDown);
        }
        if !state.enabled {
            painter.rect_filled(path, CornerRadius::same(1), self.ColorNotEnabled);
        }
        // StringAlignment.Center both ways over the whole client rectangle
        let display = Self::display_text(text);
        let mut job = egui::text::LayoutJob::simple(display, font, brush, rect.width().max(1.0));
        job.halign = egui::Align::Center;
        let galley = painter.layout_job(job);
        let pos = rect.center() - galley.rect.center().to_vec2();
        painter.galley(pos, galley, brush);
    }
}

/// `LinearGradientBrush(outside, top, bottom, LinearGradientMode.Vertical)`
fn vgradient(painter: &egui::Painter, rect: Rect, top: Color32, bottom: Color32) {
    let mut m = egui::Mesh::default();
    m.colored_vertex(rect.left_top(), top);
    m.colored_vertex(rect.right_top(), top);
    m.colored_vertex(rect.right_bottom(), bottom);
    m.colored_vertex(rect.left_bottom(), bottom);
    m.add_triangle(0, 1, 2);
    m.add_triangle(0, 2, 3);
    painter.add(m);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_and_text() {
        let b = MyButton::default();
        assert_eq!(b.TextColorNotEnabled(), b.TextColor, "an empty TextColorNotEnabled falls back");
        assert_eq!(b.ColorMouseDown, Color32::from_rgba_unmultiplied(0x2b, 0x3a, 0x03, 150));
        assert_eq!(MyButton::display_text("&Save && Exit"), "Save && Exit", "only the first '&' goes");
        assert_eq!(MyButton::display_text("Plain"), "Plain");
    }
}
