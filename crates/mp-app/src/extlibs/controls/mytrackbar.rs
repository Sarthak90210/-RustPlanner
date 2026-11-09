//! Port of `ExtLibs/Controls/MyTrackBar.cs`
//!
//! A TrackBar with float properties: each is kept in the base (int) TrackBar as
//! `(int)(float)(value * 1000.0f)` and read back divided by 1000, so values are quantised to
//! thousandths in single precision. The base `TrackBar.Value` setter throws when the value is
//! outside Minimum..Maximum.

#![allow(non_snake_case)]

use egui::{Rect, Response, Ui};
use mp_core::dotnet::Exception;

/// `(int)(float)(value * 1000.0f)`
fn to_base(value: f32) -> i32 {
    mp_core::dotnet::cast_i32((value * 1000.0) as f64)
}

/// `public class MyTrackBar : TrackBar`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MyTrackBar {
    minimum: i32,
    maximum: i32,
    value: i32,
    tick_frequency: i32,
    small_change: i32,
    large_change: i32,
}

impl Default for MyTrackBar {
    /// `new TrackBar()`: Minimum 0, Maximum 10, Value 0, TickFrequency 1, SmallChange 1,
    /// LargeChange 5 (in base units).
    fn default() -> Self {
        MyTrackBar { minimum: 0, maximum: 10, value: 0, tick_frequency: 1, small_change: 1, large_change: 5 }
    }
}

impl MyTrackBar {
    pub fn Maximum(&self) -> f32 {
        self.maximum as f32 / 1000.0
    }
    /// `TrackBar.Maximum` setter: a maximum below Minimum also lowers Minimum; Value is clamped.
    pub fn set_Maximum(&mut self, value: f32) {
        self.maximum = to_base(value);
        if self.minimum > self.maximum {
            self.minimum = self.maximum;
        }
        if self.value > self.maximum {
            self.value = self.maximum;
        }
    }
    pub fn Minimum(&self) -> f32 {
        self.minimum as f32 / 1000.0
    }
    /// `TrackBar.Minimum` setter: a minimum above Maximum also raises Maximum; Value is clamped.
    pub fn set_Minimum(&mut self, value: f32) {
        self.minimum = to_base(value);
        if self.maximum < self.minimum {
            self.maximum = self.minimum;
        }
        if self.value < self.minimum {
            self.value = self.minimum;
        }
    }
    pub fn Value(&self) -> f32 {
        self.value as f32 / 1000.0
    }
    /// `TrackBar.Value` setter: outside Minimum..Maximum throws.
    pub fn set_Value(&mut self, value: f32) -> Result<(), Exception> {
        let v = to_base(value);
        if v < self.minimum || v > self.maximum {
            return Err(Exception::ArgumentOutOfRange(format!(
                "Value of '{v}' is not valid for 'Value'. 'Value' should be between 'Minimum' and 'Maximum'.\r\nParameter name: Value"
            )));
        }
        self.value = v;
        Ok(())
    }
    pub fn TickFrequency(&self) -> f32 {
        self.tick_frequency as f32 / 1000.0
    }
    pub fn set_TickFrequency(&mut self, value: f32) {
        self.tick_frequency = to_base(value);
    }
    pub fn SmallChange(&self) -> f32 {
        self.small_change as f32 / 1000.0
    }
    pub fn set_SmallChange(&mut self, value: f32) {
        self.small_change = to_base(value);
    }
    pub fn LargeChange(&self) -> f32 {
        self.large_change as f32 / 1000.0
    }
    pub fn set_LargeChange(&mut self, value: f32) {
        self.large_change = to_base(value);
    }

    /// The vertical bar (Orientation Vertical, as the zoom bars are); the response is changed
    /// when the user scrolled it.
    pub fn show_vertical(&mut self, ui: &mut Ui, rect: Rect) -> Response {
        let mut v = self.Value() as f64;
        let resp = crate::theme::vtrackbar(ui, rect, &mut v, self.Minimum() as f64, self.Maximum() as f64);
        if resp.changed() {
            let _ = self.set_Value(v as f32);
        }
        resp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thousandths_and_range() {
        let mut t = MyTrackBar::default();
        t.set_Maximum(24.0);
        t.set_Minimum(1.0);
        t.set_Value(2.3456).unwrap();
        assert_eq!(t.Value(), 2.345);
        assert!(t.set_Value(30.0).is_err());
        assert_eq!(t.Value(), 2.345, "a refused value leaves it alone");
        t.set_SmallChange(0.001);
        assert_eq!(t.SmallChange(), 0.001);
    }
}
