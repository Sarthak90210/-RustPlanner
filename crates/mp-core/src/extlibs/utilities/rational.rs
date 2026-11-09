//! Port of `ExtLibs/Utilities/Rational.cs`
//!
//! (namespace `MissionPlanner.GeoRef`)
//!
//! An EXIF RATIONAL: whole numbers as n/1, anything else as n/100.

use crate::dotnet::cast_u32;

/// `public class Rational`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rational {
    dem: u32,
    num: u32,
}

impl Rational {
    pub fn new(input: f64) -> Self {
        let mut r = Rational { dem: 0, num: 0 };
        r.set_value(input);
        r
    }

    /// `GetBytes()`: numerator then denominator, little endian.
    pub fn get_bytes(&self) -> [u8; 8] {
        let mut answer = [0u8; 8];
        answer[..4].copy_from_slice(&self.num.to_le_bytes());
        answer[4..].copy_from_slice(&self.dem.to_le_bytes());
        answer
    }

    /// `Value` getter: `num/dem` is integer division, then widened to double.
    pub fn value(&self) -> f64 {
        (self.num / self.dem) as f64
    }

    /// `Value` setter.
    pub fn set_value(&mut self, value: f64) {
        if (value % 1.0) != 0.0 {
            self.dem = 100;
            self.num = cast_u32(value * self.dem as f64);
        } else {
            self.dem = 1;
            self.num = cast_u32(value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hundredths_and_integer_division() {
        let r = Rational::new(2.5);
        assert_eq!(r.get_bytes(), [250, 0, 0, 0, 100, 0, 0, 0]);
        assert_eq!(r.value(), 2.0);
        assert_eq!(Rational::new(7.0).get_bytes(), [7, 0, 0, 0, 1, 0, 0, 0]);
    }
}
