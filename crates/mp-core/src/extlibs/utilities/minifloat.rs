//! Port of `ExtLibs/Utilities/minifloat.cs`
//!
//! https://en.wikipedia.org/wiki/Minifloat 1.4.3.−2 minifloat.

use crate::dotnet::{cast_i32, float_to_string, Exception};

const IEEE_754_STANDARD: bool = true;
const SIGN_BIT: bool = IEEE_754_STANDARD || true;
const BITS: i32 = 8;
const EXPONENT_BITS: i32 = 4 + if SIGN_BIT { 0 } else { 1 };
#[allow(dead_code)]
const MANTISSA_BITS: i32 = BITS - EXPONENT_BITS - if SIGN_BIT { 1 } else { 0 };
const EXPONENT_BIAS: i32 = -(((1 << BITS) - 1) >> (BITS - (EXPONENT_BITS - 1)));
const MAX_EXPONENT: i32 = EXPONENT_BIAS + ((1 << EXPONENT_BITS) - 1) - if IEEE_754_STANDARD { 1 } else { 0 };
const SIGNALING_EXPONENT: i32 = (MAX_EXPONENT - EXPONENT_BIAS + if IEEE_754_STANDARD { 1 } else { 0 }) << MANTISSA_BITS;

/// `public struct minifloat`
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct minifloat {
    _value: u8,
}

impl minifloat {
    /// `minifloat(double d)`
    pub fn new(d: f64) -> Self {
        let sign: i32 = if d < 0.0 { 1 } else { 0 };
        let mut exponent = cast_i32(d.abs().log2()).wrapping_add(1);
        let mut significand = (d - (0.0625 * 2f64.powi(exponent) * 8.0)).abs() / (0.0625 * 2f64.powi(exponent));
        if d > -1.0 && d < 1.0 {
            exponent = 0;
            significand = d.abs() * 8.0;
        }
        if d.is_infinite() {
            exponent = SIGNALING_EXPONENT >> 3;
        }
        if d.is_nan() {
            exponent = SIGNALING_EXPONENT >> 3;
            significand = 1.0;
        }
        let v = (sign << 7).wrapping_add((exponent.wrapping_shl(3)) & 0x78).wrapping_add(cast_i32(significand) & 0x7);
        minifloat { _value: v as u8 }
    }

    /// The raw byte.
    pub fn raw(self) -> u8 {
        self._value
    }

    pub fn from_raw(b: u8) -> Self {
        minifloat { _value: b }
    }

    /// `Value`
    pub fn value(self) -> f32 {
        Self::to_float(self)
    }

    #[allow(dead_code)]
    fn epsilon() -> Self {
        minifloat { _value: 1 }
    }
    #[allow(dead_code)]
    fn max_value() -> Self {
        minifloat { _value: (SIGNALING_EXPONENT - 1) as u8 }
    }
    #[allow(dead_code)]
    fn nan() -> Self {
        minifloat { _value: (SIGNALING_EXPONENT | 1) as u8 }
    }
    #[allow(dead_code)]
    fn positive_infinity() -> Self {
        minifloat { _value: SIGNALING_EXPONENT as u8 }
    }

    /// `ToFloat(minifloat q)`
    pub fn to_float(q: minifloat) -> f32 {
        let mut sign = ((q._value & 0x80) >> 7) as i32;
        let exponent = ((q._value & 0x78) >> 3) as i32;
        let significand = (q._value & 0x7) as i32;

        if exponent == 0xf {
            if significand == 0 && sign == 0 {
                return f32::INFINITY;
            }
            if significand == 0 && sign == 1 {
                return f32::NEG_INFINITY;
            }
            return f32::NAN;
        }

        sign = if sign == 0 { 1 } else { -1 };

        if exponent == 0 {
            let part1 = 0.125f32 * significand as f32;
            part1 * sign as f32
        } else {
            (0.0625 * (significand + 8) as f64 * 2f64.powi(exponent)) as f32 * sign as f32
        }
    }

    /// `CompareTo(object obj)`: null sorts first; anything but a minifloat throws.
    pub fn compare_to(&self, obj: Option<&dyn std::any::Any>) -> Result<i32, Exception> {
        let Some(obj) = obj else { return Ok(1) };
        match obj.downcast_ref::<minifloat>() {
            Some(o) => Ok(match self._value.cmp(&o._value) {
                std::cmp::Ordering::Less => -1,
                std::cmp::Ordering::Equal => 0,
                std::cmp::Ordering::Greater => 1,
            }),
            None => Err(Exception::InvalidCast("Specified cast is not valid.".into())),
        }
    }

    /// `GetTypeCode()`
    pub fn get_type_code(&self) -> i32 {
        19
    }

    pub fn to_boolean(&self) -> bool {
        self._value != 0
    }
    pub fn to_byte(&self) -> u8 {
        crate::dotnet::cast_u8(self.value() as f64)
    }
    pub fn to_char(&self) -> u16 {
        crate::dotnet::cast_u16(self.value() as f64)
    }
    pub fn to_date_time(&self) -> Result<crate::dotnet::DateTime, Exception> {
        Err(Exception::NotImplemented("The method or operation is not implemented.".into()))
    }
    pub fn to_decimal(&self) -> Result<rust_decimal::Decimal, Exception> {
        rust_decimal::Decimal::try_from(self.value())
            .map_err(|_| Exception::Overflow("Value was either too large or too small for a Decimal.".into()))
    }
    pub fn to_double(&self) -> f64 {
        self.value() as f64
    }
    pub fn to_int16(&self) -> i16 {
        crate::dotnet::cast_i16(self.value() as f64)
    }
    pub fn to_int32(&self) -> i32 {
        cast_i32(self.value() as f64)
    }
    pub fn to_int64(&self) -> i64 {
        let v = self.value() as f64;
        if v.is_nan() || v.is_infinite() {
            i64::MIN
        } else {
            v as i64
        }
    }
    pub fn to_sbyte(&self) -> i8 {
        crate::dotnet::cast_i8(self.value() as f64)
    }
    pub fn to_single(&self) -> f32 {
        self.value()
    }
    pub fn to_uint16(&self) -> u16 {
        crate::dotnet::cast_u16(self.value() as f64)
    }
    pub fn to_uint32(&self) -> u32 {
        crate::dotnet::cast_u32(self.value() as f64)
    }
    pub fn to_uint64(&self) -> u64 {
        let v = self.value() as f64;
        if v.is_nan() || v.is_infinite() {
            0x8000_0000_0000_0000
        } else {
            v as i64 as u64
        }
    }
    pub fn to_type(&self) -> Result<(), Exception> {
        Err(Exception::NotImplemented("The method or operation is not implemented.".into()))
    }

    /// `test()`: prints all 256 values, eight per line.
    pub fn test() {
        for a in 0..=255u8 {
            let fl = minifloat { _value: a };
            crate::dotnet::io::console_write(&format!("{}\t", float_to_string(Self::to_float(fl))));
            if (a as i32 + 1) % 8 == 0 {
                crate::dotnet::io::console_write_line("");
            }
            let _fl2 = minifloat::new(fl.value() as f64);
        }
    }

    /// `operator ++`: `a._value++` yields the old byte, which converts to a minifloat as a number.
    pub fn increment(a: minifloat) -> minifloat {
        minifloat::new(a._value as f64)
    }

    /// `operator --`
    pub fn decrement(a: minifloat) -> minifloat {
        minifloat::new(a._value as f64)
    }
}

impl From<f64> for minifloat {
    fn from(v: f64) -> Self {
        minifloat::new(v)
    }
}

impl std::fmt::Display for minifloat {
    /// `ToString()`: the float value in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&float_to_string(self.value()))
    }
}

macro_rules! op {
    ($tr:ident, $m:ident, $o:tt) => {
        impl std::ops::$tr for minifloat {
            type Output = minifloat;
            fn $m(self, b: minifloat) -> minifloat {
                minifloat::new((self.value() $o b.value()) as f64)
            }
        }
    };
}
op!(Add, add, +);
op!(Sub, sub, -);
op!(Div, div, /);
op!(Mul, mul, *);
op!(Rem, rem, %);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_match_comments() {
        assert_eq!(EXPONENT_BIAS, -7);
        assert_eq!(MAX_EXPONENT, 7);
        assert_eq!(SIGNALING_EXPONENT, 0b0111_1000);
    }

    #[test]
    fn values() {
        assert_eq!(minifloat::new(1.0).value(), 1.0);
        assert_eq!(minifloat::new(0.25).value(), 0.25);
        assert_eq!(minifloat::new(0.0).raw(), 0);
        assert!(minifloat::new(f64::INFINITY).value().is_infinite());
        assert!(minifloat::new(f64::NAN).value().is_nan());
        assert_eq!(minifloat::increment(minifloat::new(1.0)), minifloat::new(minifloat::new(1.0).raw() as f64));
        assert_eq!(minifloat::new(1.0).compare_to(None).unwrap(), 1);
    }
}
