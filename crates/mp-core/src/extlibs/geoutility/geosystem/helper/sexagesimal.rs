//! Port of `ExtLibs/GeoUtility/GeoSystem/Helper/Sexagesimal.cs`

use crate::dotnet::{cast_i32, format_int_zero, math_round, Exception};

/// `Math.Abs(int)`: `int.MinValue` has no positive counterpart and throws.
fn math_abs_i32(v: i32) -> Result<i32, Exception> {
    v.checked_abs().ok_or_else(|| Exception::Overflow("Negating the minimum value of a twos complement number is invalid.".into()))
}

/// `public class Sexagesimal`: degrees, arc minutes, arc seconds and thirds (1/60 s) or
/// milliseconds.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sexagesimal {
    degree: i32,
    minutes: i32,
    seconds: i32,
    thirds: i32,
    millisec: f64,
}

impl Sexagesimal {
    /// `new Sexagesimal()`
    pub fn new() -> Self {
        Self::default()
    }

    /// `new Sexagesimal(double dec)`: from decimal degrees. A NaN or huge value makes `(int)l`
    /// `int.MinValue`, and the `Minutes` setter's `Math.Abs` then throws, as in C#.
    pub fn from_decimal(dec: f64) -> Result<Self, Exception> {
        let mut s = Self::default();
        let mut l = math_round(dec, 8);
        s.set_degree(cast_i32(l));
        l = math_round(((l - s.degree as f64) * 60.0).abs(), 6);
        s.set_minutes(cast_i32(l))?;
        l = math_round((l - s.minutes as f64) * 60.0, 4);
        s.set_seconds(cast_i32(l))?;
        l = math_round(l - s.seconds as f64, 2);
        s.millisec = l;
        l *= 60.0;
        s.thirds = cast_i32(l.round_ties_even());
        Ok(s)
    }

    /// `new Sexagesimal(int degree, int minutes, int seconds, int thirds)`
    pub fn from_dmst(degree: i32, minutes: i32, seconds: i32, thirds: i32) -> Result<Self, Exception> {
        let mut s = Self::default();
        s.set_degree(degree);
        s.set_minutes(minutes)?;
        s.set_seconds(seconds)?;
        s.set_thirds(thirds)?;
        s.set_millisec(thirds as f64 / 60.0);
        Ok(s)
    }

    /// `new Sexagesimal(int degree, int minutes, double seconds)`
    pub fn from_dms(degree: i32, minutes: i32, seconds: f64) -> Result<Self, Exception> {
        let mut s = Self::default();
        s.set_degree(degree);
        s.set_minutes(minutes)?;
        s.set_seconds(cast_i32(seconds))?;
        s.set_millisec(seconds.abs() - s.seconds as f64);
        s.set_thirds(cast_i32((s.millisec * 60.0).round_ties_even()))?;
        Ok(s)
    }

    /// `Degree`
    pub fn degree(&self) -> i32 {
        self.degree
    }

    pub fn set_degree(&mut self, value: i32) {
        self.degree = value;
    }

    /// `Minutes`
    pub fn minutes(&self) -> i32 {
        self.minutes
    }

    pub fn set_minutes(&mut self, value: i32) -> Result<(), Exception> {
        self.minutes = math_abs_i32(value)?;
        Ok(())
    }

    /// `Seconds`
    pub fn seconds(&self) -> i32 {
        self.seconds
    }

    pub fn set_seconds(&mut self, value: i32) -> Result<(), Exception> {
        self.seconds = math_abs_i32(value)?;
        Ok(())
    }

    /// `Thirds` (1/60 s, rarely used)
    pub fn thirds(&self) -> i32 {
        self.thirds
    }

    pub fn set_thirds(&mut self, value: i32) -> Result<(), Exception> {
        self.thirds = math_abs_i32(value)?;
        Ok(())
    }

    /// `Millisec`
    pub fn millisec(&self) -> f64 {
        self.millisec
    }

    pub fn set_millisec(&mut self, value: f64) {
        self.millisec = value.abs();
    }

    /// `ToDecimal()`
    pub fn to_decimal(&self) -> f64 {
        let msec = self.millisec;
        let sec = (msec + self.seconds as f64) / 60.0;
        let mut min = (sec + self.minutes as f64) / 60.0;
        if self.degree < 0 {
            min *= -1.0; // negative degrees subtract the fraction
        }
        min + self.degree as f64
    }
}

impl std::fmt::Display for Sexagesimal {
    /// `ToString()`: `D° MM' SS'' TT'''`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}° {}' {}'' {}'''",
            self.degree,
            format_int_zero(self.minutes as i64, 2),
            format_int_zero(self.seconds as i64, 2),
            format_int_zero(self.thirds as i64, 2)
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_decimal() {
        let s = Sexagesimal::from_decimal(8.5125).unwrap();
        assert!(Sexagesimal::from_decimal(f64::NAN).is_err());
        assert_eq!((s.degree(), s.minutes(), s.seconds()), (8, 30, 45));
        assert_eq!(s.to_string(), "8° 30' 45'' 00'''");
        assert!(Sexagesimal::from_dmst(1, i32::MIN, 0, 0).is_err());
    }
}
