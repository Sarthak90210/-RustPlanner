//! Port of `ExtLibs/Controls/QuickView.cs`
//!
//! A Flight Data quick-view tile: `desc` centred at y = 5 in the control font and ForeColor,
//! and below it `number.ToString(numberformat)` in `numberColor`, in a font size that keeps
//! being rescaled to fit (`newSize` persists between paints, starting at 8 pt, rounded down to
//! a multiple of 5 and reset to 8 outside 8..999999).

#![allow(non_snake_case)]

use egui::{Color32, FontId, Pos2, Rect};

use crate::fonts;

/// `public partial class QuickView : SKControl`
#[derive(Debug, Clone)]
pub struct QuickView {
    _desc: String,
    _number: f64,
    _numberformat: String,
    pub numberColor: Color32,
    /// a backup store for numberColor, so it can be changed temporarily
    pub numberColorBackup: Color32,
    /// `float newSize = 8`: the number's font size in points, carried from paint to paint
    newSize: f32,
}

impl Default for QuickView {
    fn default() -> Self {
        QuickView { _desc: String::new(), _number: -9999.0, _numberformat: "0.00".into(), numberColor: Color32::BLACK, numberColorBackup: Color32::BLACK, newSize: 8.0 }
    }
}

impl QuickView {
    pub fn desc(&self) -> &str {
        &self._desc
    }
    pub fn set_desc(&mut self, value: &str) {
        if self._desc != value {
            self._desc = value.to_string();
        }
    }
    pub fn number(&self) -> f64 {
        self._number
    }
    /// `number` setter: `_number.Equals(value)` (NaN equals NaN) skips the invalidate.
    pub fn set_number(&mut self, value: f64) {
        if self._number == value || (self._number.is_nan() && value.is_nan()) {
            return;
        }
        self._number = value;
    }
    pub fn numberformat(&self) -> &str {
        &self._numberformat
    }
    pub fn set_numberformat(&mut self, value: &str) {
        if self._numberformat != value {
            self._numberformat = value.to_string();
        }
    }

    /// `OnPaintSurface`: `font_pt` is the control's Font size, `fore` its ForeColor.
    pub fn OnPaintSurface(&mut self, painter: &egui::Painter, rect: Rect, font_pt: f32, fore: Color32) {
        let px = |pt: f32| pt * 96.0 / 72.0;
        // MeasureString(...).ToSize(): whole pixels, truncated
        let measure = |s: &str, pt: f32| {
            let g = painter.layout_no_wrap(s.to_string(), FontId::proportional(px(pt)), fore);
            (g.size().x.trunc(), g.size().y.trunc())
        };
        let (dw, dh) = measure(&self._desc, font_pt);
        let mid = (dw as i32 / 2) as f32;
        let width = rect.width().trunc();
        let height = rect.height().trunc();
        painter.text(Pos2::new(rect.left() + ((width as i32 / 2) as f32 - mid), rect.top() + 5.0), egui::Align2::LEFT_TOP, &self._desc, fonts::ui_id(px(font_pt)), fore);
        let y = dh;

        let numb = format_number(self._number, &self._numberformat);
        let probe = "0".repeat(numb.chars().count() + 1);
        let (ew, eh) = measure(&probe, self.newSize);
        let h_ratio = (height - y) / eh.max(1.0);
        let w_ratio = width / ew.max(1.0);
        let ratio = if h_ratio < w_ratio { h_ratio } else { w_ratio };
        self.newSize *= ratio;
        self.newSize -= self.newSize % 5.0;
        if self.newSize < 8.0 || self.newSize > 999999.0 {
            self.newSize = 8.0;
        }
        let (nw, nh) = measure(&numb, self.newSize);
        let x = (width as i32 / 2) as f32 - (nw as i32 / 2) as f32;
        let yy = y + (((height - y) as i32 / 2) as f32 - (nh as i32 / 2) as f32);
        painter.text(Pos2::new(rect.left() + x, rect.top() + yy), egui::Align2::LEFT_TOP, numb, FontId::proportional(px(self.newSize)), self.numberColor);
    }
}

/// `double.ToString(numberformat)` for the "0.00" / "#0.0#" style formats the quick views use.
fn format_number(v: f64, format: &str) -> String {
    let c = mp_core::dotnet::current_culture();
    let (int_part, frac) = format.split_once('.').unwrap_or((format, ""));
    let simple = |s: &str| s.chars().all(|ch| ch == '0' || ch == '#');
    if !format.is_empty() && simple(int_part) && simple(frac) {
        let min_int = int_part.chars().filter(|&ch| ch == '0').count();
        let min_dec = frac.chars().take_while(|&ch| ch == '0').count() as u32;
        let max_dec = frac.chars().count() as u32;
        return mp_core::dotnet::format_fixed_optional(v, min_int, min_dec, max_dec, c);
    }
    mp_core::dotnet::format_fixed(v, 1, 2, c)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn number_formats_and_setters() {
        assert_eq!(format_number(1.5, "0.00"), "1.50");
        assert_eq!(format_number(-0.004, "0.00"), "0.00");
        assert_eq!(format_number(12.345, "0.#"), "12.3");
        assert_eq!(format_number(3.0, "0"), "3");
        let mut q = QuickView::default();
        assert_eq!(q.number(), -9999.0);
        q.set_number(f64::NAN);
        q.set_number(f64::NAN);
        assert!(q.number().is_nan());
    }
}
