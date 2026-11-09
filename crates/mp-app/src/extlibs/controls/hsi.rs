//! Port of `ExtLibs/Controls/HSI.cs`
//!
//! Horizontal situation indicator: a heading card (labels every 30 degrees and N/E/S/W, ticks
//! every 5) rotated by -Heading under a fixed orange aircraft, a white lubber line at the top
//! and an orange heading bug at NavHeading. As in the C#, each label is drawn at -y and its
//! tick at +y of the same rotated frame, so the ticks sit opposite their labels. OnResize makes
//! the control square (Width = Height).

#![allow(non_snake_case)]

use egui::{Color32, Pos2, Rect, Stroke, Vec2};

/// `public partial class HSI : MyUserControl`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct HSI {
    _heading: i32,
    _navbearing: i32,
}

/// `Graphics.RotateTransform(deg)` applied to a point (clockwise on screen).
fn rot(p: Vec2, deg: f32) -> Vec2 {
    let (s, c) = deg.to_radians().sin_cos();
    Vec2::new(p.x * c - p.y * s, p.x * s + p.y * c)
}

impl HSI {
    pub fn Heading(&self) -> i32 {
        self._heading
    }
    pub fn set_Heading(&mut self, value: i32) {
        self._heading = value;
    }
    pub fn NavHeading(&self) -> i32 {
        self._navbearing
    }
    pub fn set_NavHeading(&mut self, value: i32) {
        self._navbearing = value;
    }

    /// `OnPaint` into `rect` (made square on its height).
    pub fn OnPaint(&self, painter: &egui::Painter, rect: Rect) {
        let side = rect.height();
        let rect = Rect::from_min_size(rect.min, Vec2::splat(side));
        let width = side as i32;
        let radiusinside = (width as f32 / 3.6) as i32 as f32;
        let radiusoutside = (width as f32 / 2.2) as i32 as f32;
        // the centre after TranslateTransform(Width / 2, Height / 2): integer halves
        let c = rect.min + Vec2::new((width / 2) as f32, (width / 2) as f32);
        let white = Stroke::new(1.0, Color32::WHITE);
        // the heading card, rotated by -Heading
        let font = width / 14;
        let font_id = egui::FontId::proportional(font as f32 * 96.0 / 72.0);
        let line = |deg: f32, a: Vec2, b: Vec2, s: Stroke| painter.line_segment([c + rot(a, deg), c + rot(b, deg)], s);
        let label = |deg: f32, text: &str| {
            let pos = c + rot(Vec2::new(-(font as f32), -radiusoutside), deg);
            let galley = painter.layout_no_wrap(text.to_string(), font_id.clone(), Color32::WHITE);
            painter.add(egui::epaint::TextShape::new(pos, galley, Color32::WHITE).with_angle(deg.to_radians()));
        };
        let mut a = 0;
        while a <= 360 {
            let deg = a as f32 - self._heading as f32;
            let tick = |len: f32| line(deg, Vec2::new(0.0, radiusinside), Vec2::new(0.0, radiusinside + len), white);
            match a {
                0 => {
                    label(deg, " N");
                    tick(11.0);
                }
                90 => {
                    label(deg, " E");
                    tick(11.0);
                }
                180 => {
                    label(deg, " S");
                    tick(11.0);
                }
                270 => {
                    label(deg, " W");
                    tick(11.0);
                }
                // ignore it, as we process it at 0
                360 => {}
                _ if a % 30 == 0 => {
                    label(deg, &format!("{:>2}", a / 10));
                    tick(11.0);
                }
                // larger line / small line
                _ if a % 10 == 0 => {
                    tick(7.0);
                }
                _ if a % 5 == 0 => {
                    tick(4.0);
                }
                _ => {}
            }
            a += 5;
        }
        let orange = Stroke::new(2.0, Color32::from_rgb(255, 140, 0));
        let at = |x: f32, y: f32| c + Vec2::new(x, y);
        // body, wing, tail
        painter.line_segment([at(0.0, 30.0), at(0.0, -10.0)], orange);
        painter.line_segment([at(-30.0, 0.0), at(30.0, 0.0)], orange);
        painter.line_segment([at(-10.0, 25.0), at(10.0, 25.0)], orange);
        painter.line_segment([at(0.0, -radiusoutside), at(0.0, -radiusinside)], Stroke::new(2.0, Color32::WHITE));
        // the heading bug at NavHeading - Heading
        let bug_deg = (self._navbearing - self._heading) as f32;
        let pts = [(-5.0, 0.0), (-5.0, 4.0), (-3.0, 4.0), (0.0, 8.0), (3.0, 4.0), (5.0, 4.0), (5.0, 0.0)];
        let poly: Vec<Pos2> = pts.iter().map(|&(x, y)| c + rot(Vec2::new(x, -radiusoutside + y), bug_deg)).collect();
        painter.add(egui::Shape::line(poly, orange));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_is_clockwise() {
        let p = rot(Vec2::new(0.0, -10.0), 90.0);
        assert!((p.x - 10.0).abs() < 1e-4 && p.y.abs() < 1e-4, "{p:?}");
    }
}
