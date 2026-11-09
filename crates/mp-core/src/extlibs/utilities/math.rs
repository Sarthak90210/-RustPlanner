//! Port of `ExtLibs/Utilities/Math.cs`

use crate::dotnet::{math_max, math_min};

/// `public static class MathHelper`
pub struct MathHelper;

impl MathHelper {
    pub const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
    /// `1.0 / rad2deg`, which is not bit-identical to `PI / 180`.
    pub const DEG2RAD: f64 = 1.0 / Self::RAD2DEG;

    pub const RAD2DEGF: f32 = (180.0 / std::f64::consts::PI) as f32;
    pub const DEG2RADF: f32 = (1.0 / Self::RAD2DEG) as f32;

    /// `Degrees(rad)`
    pub fn degrees(rad: f64) -> f64 {
        rad * Self::RAD2DEG
    }

    /// `Radians(deg)`
    pub fn radians(deg: f64) -> f64 {
        deg * Self::DEG2RAD
    }

    /// `map(x, in_min, in_max, out_min, out_max)`
    pub fn map(x: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> f64 {
        (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    }

    /// `mapConstrained(...)`
    pub fn map_constrained(x: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> f64 {
        let output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        let output = math_max(output, out_min);
        math_min(output, out_max)
    }

    /// `constrain(x, out_min, out_max)`
    pub fn constrain(x: f64, out_min: f64, out_max: f64) -> f64 {
        let x = math_max(x, out_min);
        math_min(x, out_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helpers() {
        assert_eq!(MathHelper::map(5.0, 0.0, 10.0, 0.0, 100.0), 50.0);
        assert_eq!(MathHelper::constrain(15.0, 0.0, 10.0), 10.0);
        assert!(MathHelper::constrain(f64::NAN, 0.0, 10.0).is_nan());
        assert_eq!(MathHelper::DEG2RAD, 1.0 / (180.0 / std::f64::consts::PI));
    }
}
