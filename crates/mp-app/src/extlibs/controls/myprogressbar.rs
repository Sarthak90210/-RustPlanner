//! Port of `ExtLibs/Controls/MyProgressBar.cs`
//!
//! Mission Planner's progress bar: BackColor inside a 1 px inset, a vertical BGGradTop ->
//! BGGradBot bar `(int)((Value / (float)(Maximum - Minimum)) * Width)` wide (Value is not
//! offset by Minimum, and the width is not clamped), and a 1 px Outline. Marquee style moves a
//! BarSize-wide block instead. The control draws no text.

#![allow(non_snake_case)]

use egui::{Color32, Rect, Stroke, StrokeKind, Vec2};

/// `ProgressBarStyle`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ProgressBarStyle {
    Blocks,
    #[default]
    Continuous,
    Marquee,
}

/// `public partial class MyProgressBar : MyUserControl`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MyProgressBar {
    pub Maximum: i32,
    pub Minimum: i32,
    pub Value: i32,
    pub BarSize: i32,
    pub BGGradTop: Color32,
    pub BGGradBot: Color32,
    pub TextColor: Color32,
    pub Outline: Color32,
    pub Style: ProgressBarStyle,
}

impl Default for MyProgressBar {
    fn default() -> Self {
        MyProgressBar {
            Maximum: 100,
            Minimum: 0,
            Value: 0,
            BarSize: 40,
            BGGradTop: Color32::from_rgb(102, 139, 26),
            BGGradBot: Color32::from_rgb(127, 167, 42),
            TextColor: Color32::from_rgb(31, 54, 8),
            Outline: Color32::from_rgb(150, 171, 112),
            Style: ProgressBarStyle::Continuous,
        }
    }
}

impl MyProgressBar {
    /// `marquee_Tick`: advance the marquee block one step.
    pub fn marquee_Tick(&mut self) {
        let mut delta = self.Maximum.wrapping_sub(self.Minimum);
        if delta == 0 {
            delta = 100;
        }
        let mut newvalue = self.Value.wrapping_add(delta / 50);
        if newvalue > self.Maximum {
            newvalue = -self.BarSize;
        }
        if newvalue < self.Minimum - self.BarSize {
            newvalue = -self.BarSize;
        }
        self.Value = newvalue;
    }

    /// `OnPaint` into `rect`, with `back` as BackColor.
    pub fn OnPaint(&self, painter: &egui::Painter, rect: Rect, back: Color32) {
        let mut delta = self.Maximum.wrapping_sub(self.Minimum);
        if delta == 0 {
            delta = 100;
        }
        let width = rect.width() as i32;
        let height = rect.height() as i32;
        let position = ((self.Value as f32 / delta as f32) * width as f32) as i32;
        let o = rect.min;
        let outside = Rect::from_min_size(o + Vec2::new(1.0, 1.0), Vec2::new((width - 2) as f32, (height - 2) as f32));
        let progressdone = if self.Style == ProgressBarStyle::Marquee {
            Rect::from_min_size(o + Vec2::new(position as f32, 1.0), Vec2::new(self.BarSize as f32, (height - 2) as f32))
        } else {
            Rect::from_min_size(o + Vec2::new(1.0, 1.0), Vec2::new(position as f32, (height - 2) as f32))
        };
        if width > 4 && height > 4 {
            let clip = painter.with_clip_rect(rect.intersect(painter.clip_rect()));
            // fill the background
            clip.rect_filled(outside, 0.0, back);
            // draw the progress (LinearGradientBrush over `outside`, vertical)
            if progressdone.width() > 0.0 {
                let t = |y: f32| ((y - outside.top()) / outside.height().max(1.0)).clamp(0.0, 1.0);
                let lerp = |a: Color32, b: Color32, f: f32| {
                    let m = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * f).round() as u8;
                    Color32::from_rgb(m(a.r(), b.r()), m(a.g(), b.g()), m(a.b(), b.b()))
                };
                let top = lerp(self.BGGradTop, self.BGGradBot, t(progressdone.top()));
                let bot = lerp(self.BGGradTop, self.BGGradBot, t(progressdone.bottom()));
                let mut m = egui::Mesh::default();
                m.colored_vertex(progressdone.left_top(), top);
                m.colored_vertex(progressdone.right_top(), top);
                m.colored_vertex(progressdone.right_bottom(), bot);
                m.colored_vertex(progressdone.left_bottom(), bot);
                m.add_triangle(0, 1, 2);
                m.add_triangle(0, 2, 3);
                clip.add(m);
            }
            // draw the outside: DrawRectangle(0, 0, Width - 1, Height - 1)
            let outline = Rect::from_min_size(o, Vec2::new((width - 1) as f32, (height - 1) as f32));
            clip.rect_stroke(outline.translate(Vec2::splat(0.5)), 0.0, Stroke::new(1.0, self.Outline), StrokeKind::Middle);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marquee_wraps_to_minus_barsize() {
        let mut b = MyProgressBar { Style: ProgressBarStyle::Marquee, Value: 99, ..Default::default() };
        b.marquee_Tick();
        assert_eq!(b.Value, -40, "past Maximum wraps to -BarSize");
        b.marquee_Tick();
        assert_eq!(b.Value, -38);
    }
}
