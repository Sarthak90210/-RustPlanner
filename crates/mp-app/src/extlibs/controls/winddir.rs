//! Port of `ExtLibs/Controls/WindDir.cs`
//!
//! The wind indicator on the Flight Data map: an outer and an inner black circle, a red line
//! from the centre toward where the wind blows (Direction + 180) whose length is Speed over a
//! 10 m/s full scale (or the speed itself above that), a two-stroke arrow head, and the speed
//! ("0") in red at the top left.

#![allow(non_snake_case)]

use egui::{Color32, FontId, Pos2, Rect, Stroke, Vec2};

const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// `public partial class WindDir : MyUserControl`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WindDir {
    _direction: f64,
    _speed: f64,
    maxspeed: f64,
}

impl Default for WindDir {
    fn default() -> Self {
        WindDir { _direction: 0.0, _speed: 0.0, maxspeed: 10.0 }
    }
}

impl WindDir {
    /// `Direction` getter: the stored value is the set value + 180.
    pub fn Direction(&self) -> f64 {
        self._direction
    }

    /// `Direction` setter
    pub fn set_Direction(&mut self, value: f64) {
        if self._direction == value + 180.0 {
            return;
        }
        self._direction = value + 180.0;
    }

    pub fn Speed(&self) -> f64 {
        self._speed
    }

    pub fn set_Speed(&mut self, value: f64) {
        self._speed = value;
    }

    /// `OnPaint` into `rect`; `font` is the control font (8.25pt bold on Flight Data).
    pub fn OnPaint(&mut self, painter: &egui::Painter, rect: Rect, font: FontId) {
        if self._direction > 360.0 {
            self._direction %= 360.0;
        }
        let (w, h) = (rect.width() as i32, rect.height() as i32);
        let o = rect.min;
        let black = Stroke::new(2.0, Color32::BLACK);
        let red = Stroke::new(2.0, Color32::RED);
        // DrawArc(outside = (1, 1, Width - 3, Height - 3), 0, 360)
        ellipse(painter, Rect::from_min_size(o + Vec2::new(1.0, 1.0), Vec2::new((w - 3) as f32, (h - 3) as f32)), black);
        // inside = (Width / 4, Height / 4, (Width / 4) * 2, (Height / 4) * 2)
        ellipse(painter, Rect::from_min_size(o + Vec2::new((w / 4) as f32, (h / 4) as f32), Vec2::new(((w / 4) * 2) as f32, ((h / 4) * 2) as f32)), black);

        let mut x = (w / 2) as f64 * ((self._direction - 90.0) * DEG2RAD).cos();
        let mut y = (h / 2) as f64 * ((self._direction - 90.0) * DEG2RAD).sin();
        // full scale is 10ms
        let scale = mp_core::dotnet::math_max(self.maxspeed, self._speed);
        x = x / scale * self._speed;
        y = y / scale * self._speed;
        if x != 0.0 || y != 0.0 {
            let outx = ((w / 2) as f64 + x) as f32;
            let outy = ((h / 2) as f64 + y) as f32;
            let at = |px: f32, py: f32| o + Vec2::new(px, py);
            painter.line_segment([at((w / 2) as f32, (h / 2) as f32), at(outx, outy)], red);
            let x1 = (w / 7) as f32 * ((self._direction - 60.0) * DEG2RAD).cos() as f32;
            let y1 = (h / 7) as f32 * ((self._direction - 60.0) * DEG2RAD).sin() as f32;
            painter.line_segment([at(outx, outy), at(outx - x1, outy - y1)], red);
            let x1 = (w / 7) as f32 * ((self._direction + 60.0 + 180.0) * DEG2RAD).cos() as f32;
            let y1 = (h / 7) as f32 * ((self._direction + 60.0 + 180.0) * DEG2RAD).sin() as f32;
            painter.line_segment([at(outx, outy), at(outx - x1, outy - y1)], red);
        }
        let text = mp_core::dotnet::format_fixed(self._speed, 1, 0, mp_core::dotnet::current_culture());
        painter.text(o, egui::Align2::LEFT_TOP, text, font, Color32::RED);
    }
}

/// `Graphics.DrawArc(pen, rect, 0, 360)`: the ellipse inscribed in `rect`.
fn ellipse(painter: &egui::Painter, rect: Rect, stroke: Stroke) {
    let c = rect.center();
    let (rx, ry) = (rect.width() / 2.0, rect.height() / 2.0);
    let pts: Vec<Pos2> = (0..64)
        .map(|i| {
            let a = i as f32 / 64.0 * std::f32::consts::TAU;
            Pos2::new(c.x + rx * a.cos(), c.y + ry * a.sin())
        })
        .collect();
    painter.add(egui::Shape::closed_line(pts, stroke));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direction_is_stored_plus_180() {
        let mut w = WindDir::default();
        w.set_Direction(90.0);
        assert_eq!(w.Direction(), 270.0);
        w.set_Direction(270.0);
        assert_eq!(w.Direction(), 450.0, "wrapped only when painted");
    }
}
