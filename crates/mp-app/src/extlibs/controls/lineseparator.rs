//! Port of `ExtLibs/Controls/LineSeparator.cs`
//!
//! A separator made of two 1 px lines, the top 'Primary' and the bottom 'Secondary' (so it is
//! always 2 px high). Each line's opacity runs from Opacity1 at the left through Opacity2 at
//! the middle to Opacity3 at the right, as two linear-gradient pens.

#![allow(non_snake_case)]

use egui::{Color32, Pos2, Rect};

/// `public partial class LineSeparator : MyUserControl`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineSeparator {
    /// Primary Color of the secondary line (default DarkGray)
    pub PrimaryColor: Color32,
    /// Secondary Color of the secondary line (default White)
    pub SecondaryColor: Color32,
    /// Opacity at the left most point
    pub Opacity1: f32,
    /// Opacity at the mid point
    pub Opacity2: f32,
    /// Opacity at the right most point
    pub Opacity3: f32,
}

impl Default for LineSeparator {
    /// `new LineSeparator()`: DarkGray over White, fully opaque; `Height = 2`.
    fn default() -> Self {
        LineSeparator { PrimaryColor: Color32::from_rgb(169, 169, 169), SecondaryColor: Color32::WHITE, Opacity1: 1.0, Opacity2: 1.0, Opacity3: 1.0 }
    }
}

impl LineSeparator {
    /// `MinimumSize` / `MaximumSize`: the height is pinned to 2.
    pub const HEIGHT: f32 = 2.0;

    /// `LineSeparator_Paint` into `rect` (its Location and Width; the height is always 2).
    pub fn paint(&self, painter: &egui::Painter, rect: Rect) {
        self.DrawLine(painter, rect, self.PrimaryColor, 0.0);
        self.DrawLine(painter, rect, self.SecondaryColor, 1.0);
    }

    /// `DrawLine(baseColor, y, g)`: (int)(opacity * 255) alpha at the left, middle and right,
    /// blended along two pens.
    fn DrawLine(&self, painter: &egui::Painter, rect: Rect, base: Color32, y: f32) {
        let a = |o: f32| (o * 255.0) as i32 as u8;
        let c = |o: f32| Color32::from_rgba_unmultiplied(base.r(), base.g(), base.b(), a(o));
        let (c1, c2, c3) = (c(self.Opacity1), c(self.Opacity2), c(self.Opacity3));
        let yy = rect.top() + y + 0.5;
        let left = rect.left();
        // new Point(Width / 2, y): integer division
        let mid = rect.left() + ((rect.width() as i32) / 2) as f32;
        let right = rect.left() + rect.width();
        gradient_segment(painter, Pos2::new(left, yy), Pos2::new(mid, yy), c1, c2);
        gradient_segment(painter, Pos2::new(mid, yy), Pos2::new(right, yy), c2, c3);
    }
}

/// A 1 px horizontal line whose colour runs from `a` to `b` (a LinearGradientBrush pen).
fn gradient_segment(painter: &egui::Painter, p0: Pos2, p1: Pos2, a: Color32, b: Color32) {
    if p1.x <= p0.x {
        return;
    }
    let mut mesh = egui::Mesh::default();
    let h = 0.5;
    mesh.colored_vertex(Pos2::new(p0.x, p0.y - h), a);
    mesh.colored_vertex(Pos2::new(p1.x, p1.y - h), b);
    mesh.colored_vertex(Pos2::new(p1.x, p1.y + h), b);
    mesh.colored_vertex(Pos2::new(p0.x, p0.y + h), a);
    mesh.add_triangle(0, 1, 2);
    mesh.add_triangle(0, 2, 3);
    painter.add(mesh);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_the_constructor() {
        let l = LineSeparator::default();
        assert_eq!(l.PrimaryColor, Color32::from_rgb(169, 169, 169));
        assert_eq!((l.Opacity1, l.Opacity2, l.Opacity3), (1.0, 1.0, 1.0));
        assert_eq!(LineSeparator::HEIGHT, 2.0);
    }
}
