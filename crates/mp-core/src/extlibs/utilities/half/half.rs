//! Port of `ExtLibs/Utilities/Half/Half.cs`
//!
//! (namespace `System`)
//!
//! A half-precision float stored as its 16 bits. Arithmetic converts to `float`, operates,
//! and converts back (truncating, see `HalfHelper`). Not ported: the `ToString(string format,
//! ...)` overloads, which need .NET's general numeric format strings, and
//! `IConvertible.ToType`.

use super::halfhelper::HalfHelper;
use crate::dotnet::{
    bit_converter, cast_i32, cast_i64, cast_u16, cast_u32, cast_u64, cast_u8, cast_i8, cast_i16, float_parse_culture,
    float_to_string_culture, float_try_parse_culture, current_culture, CultureInfo, Exception, INVARIANT_CULTURE,
};

/// `public struct Half`
#[derive(Debug, Clone, Copy, Default)]
pub struct Half {
    /// `internal ushort value`
    pub(crate) value: u16,
}

impl Half {
    /// `Half.Epsilon`: smallest positive value.
    pub const EPSILON: Half = Half { value: 0x0001 };
    /// `Half.MaxValue`
    pub const MAX_VALUE: Half = Half { value: 0x7bff };
    /// `Half.MinValue`
    pub const MIN_VALUE: Half = Half { value: 0xfbff };
    /// `Half.NaN`
    pub const NAN: Half = Half { value: 0xfe00 };
    /// `Half.NegativeInfinity`
    pub const NEGATIVE_INFINITY: Half = Half { value: 0xfc00 };
    /// `Half.PositiveInfinity`
    pub const POSITIVE_INFINITY: Half = Half { value: 0x7c00 };

    /// `new Half(float value)` (also the `int`, `long`, `double`, `uint`, `ulong` and decimal
    /// constructors, which cast to float first)
    pub fn new(value: f32) -> Half {
        HalfHelper::single_to_half(value)
    }

    /// `new Half(double value)`: `(float)value` first.
    pub fn from_f64(value: f64) -> Half {
        Half::new(value as f32)
    }

    /// `new Half(int value)`
    pub fn from_i32(value: i32) -> Half {
        Half::new(value as f32)
    }

    /// `new Half(long value)`
    pub fn from_i64(value: i64) -> Half {
        Half::new(value as f32)
    }

    /// `new Half(uint value)`
    pub fn from_u32(value: u32) -> Half {
        Half::new(value as f32)
    }

    /// `new Half(ulong value)`
    pub fn from_u64(value: u64) -> Half {
        Half::new(value as f32)
    }

    /// `ToHalf(ushort bits)`
    pub fn to_half(bits: u16) -> Half {
        Half { value: bits }
    }

    /// `ToHalf(byte[] value, int startIndex)`: `BitConverter.ToInt16`, with its checks.
    pub fn to_half_bytes(value: &[u8], start_index: i32) -> Result<Half, Exception> {
        Ok(Half::to_half(bit_converter::to_i16(value, start_index)? as u16))
    }

    /// `GetBytes(Half value)`: little endian.
    pub fn get_bytes(value: Half) -> [u8; 2] {
        value.value.to_le_bytes()
    }

    /// `GetBits(Half value)`
    pub fn get_bits(value: Half) -> u16 {
        value.value
    }

    /// `implicit operator float(Half value)`
    pub fn to_f32(self) -> f32 {
        HalfHelper::half_to_single(self)
    }

    /// `implicit operator double(Half value)`
    pub fn to_f64(self) -> f64 {
        self.to_f32() as f64
    }

    /// `explicit operator byte(Half)`: `(byte)(float)value`, unchecked.
    pub fn to_u8(self) -> u8 {
        cast_u8(self.to_f64())
    }

    /// `explicit operator sbyte(Half)`
    pub fn to_i8(self) -> i8 {
        cast_i8(self.to_f64())
    }

    /// `explicit operator char(Half)`
    pub fn to_char(self) -> u16 {
        cast_u16(self.to_f64())
    }

    /// `explicit operator short(Half)`
    pub fn to_i16(self) -> i16 {
        cast_i16(self.to_f64())
    }

    /// `explicit operator ushort(Half)`
    pub fn to_u16(self) -> u16 {
        cast_u16(self.to_f64())
    }

    /// `explicit operator int(Half)`
    pub fn to_i32(self) -> i32 {
        cast_i32(self.to_f64())
    }

    /// `explicit operator uint(Half)`
    pub fn to_u32(self) -> u32 {
        cast_u32(self.to_f64())
    }

    /// `explicit operator long(Half)`
    pub fn to_i64(self) -> i64 {
        cast_i64(self.to_f64())
    }

    /// `explicit operator ulong(Half)`
    pub fn to_u64(self) -> u64 {
        cast_u64(self.to_f64())
    }

    /// `explicit operator decimal(Half)` and `IConvertible.ToDecimal`: `new decimal(float)`,
    /// which keeps 7 significant digits.
    pub fn to_decimal(self) -> Result<rust_decimal::Decimal, Exception> {
        let f = self.to_f32();
        if !f.is_finite() {
            return Err(Exception::Overflow("Value was either too large or too small for a Decimal.".into()));
        }
        rust_decimal::Decimal::from_scientific(&format!("{:.6e}", f))
            .map(|d| d.normalize())
            .map_err(|_| Exception::Overflow("Value was either too large or too small for a Decimal.".into()))
    }

    /// `Negate(Half)` / unary `-`
    pub fn negate(half: Half) -> Half {
        HalfHelper::negate(half)
    }

    /// `operator ++`: `(Half)(half + 1f)`
    pub fn increment(self) -> Half {
        Half::new(self.to_f32() + 1.0)
    }

    /// `operator --`
    pub fn decrement(self) -> Half {
        Half::new(self.to_f32() - 1.0)
    }

    /// `operator ==`: false when the left operand is NaN, otherwise the bits compared (so
    /// +0 and -0 differ).
    pub fn op_equality(half1: Half, half2: Half) -> bool {
        !Half::is_nan(half1) && (half1.value == half2.value)
    }

    /// `operator !=`: the bits compared, NaN included (so NaN != NaN is false when the bits
    /// match, unlike `==`).
    pub fn op_inequality(half1: Half, half2: Half) -> bool {
        half1.value != half2.value
    }

    /// `operator <`
    pub fn op_less_than(half1: Half, half2: Half) -> bool {
        half1.to_f32() < half2.to_f32()
    }

    /// `operator >`
    pub fn op_greater_than(half1: Half, half2: Half) -> bool {
        half1.to_f32() > half2.to_f32()
    }

    /// `operator <=`
    pub fn op_less_than_or_equal(half1: Half, half2: Half) -> bool {
        Half::op_equality(half1, half2) || Half::op_less_than(half1, half2)
    }

    /// `operator >=`
    pub fn op_greater_than_or_equal(half1: Half, half2: Half) -> bool {
        Half::op_equality(half1, half2) || Half::op_greater_than(half1, half2)
    }

    /// `CompareTo(Half other)`: NaN sorts below every number.
    pub fn compare_to(self, other: Half) -> i32 {
        let mut result = 0;
        if Half::op_less_than(self, other) {
            result = -1;
        } else if Half::op_greater_than(self, other) {
            result = 1;
        } else if Half::op_inequality(self, other) {
            if !Half::is_nan(self) {
                result = 1;
            } else if !Half::is_nan(other) {
                result = -1;
            }
        }
        result
    }

    /// `CompareTo(object obj)`: 1 for null.
    pub fn compare_to_object(self, obj: Option<Half>) -> i32 {
        match obj {
            None => 1,
            Some(h) => self.compare_to(h),
        }
    }

    /// `Equals(Half other)`: `==`, or both NaN.
    pub fn equals(self, other: Half) -> bool {
        Half::op_equality(other, self) || (Half::is_nan(other) && Half::is_nan(self))
    }

    /// `GetHashCode()`: `ushort.GetHashCode()`.
    pub fn get_hash_code(self) -> i32 {
        self.value as i32
    }

    /// `GetTypeCode()`: `(TypeCode)255`.
    pub fn get_type_code(self) -> i32 {
        255
    }

    /// `Sign(Half value)`: throws ArithmeticException for NaN.
    pub fn sign(value: Half) -> Result<i32, Exception> {
        let zero = Half::from_i32(0);
        if Half::op_less_than(value, zero) {
            return Ok(-1);
        } else if Half::op_greater_than(value, zero) {
            return Ok(1);
        } else if Half::op_inequality(value, zero) {
            return Err(Exception::Arithmetic("Function does not accept floating point Not-a-Number values.".into()));
        }
        Ok(0)
    }

    /// `Abs(Half value)`
    pub fn abs(value: Half) -> Half {
        HalfHelper::abs(value)
    }

    /// `Max(value1, value2)`
    pub fn max(value1: Half, value2: Half) -> Half {
        if Half::op_less_than(value1, value2) {
            value2
        } else {
            value1
        }
    }

    /// `Min(value1, value2)`
    pub fn min(value1: Half, value2: Half) -> Half {
        if Half::op_less_than(value1, value2) {
            value1
        } else {
            value2
        }
    }

    /// `IsNaN(Half)`
    pub fn is_nan(half: Half) -> bool {
        HalfHelper::is_nan(half)
    }

    /// `IsInfinity(Half)`
    pub fn is_infinity(half: Half) -> bool {
        HalfHelper::is_infinity(half)
    }

    /// `IsNegativeInfinity(Half)`
    pub fn is_negative_infinity(half: Half) -> bool {
        HalfHelper::is_negative_infinity(half)
    }

    /// `IsPositiveInfinity(Half)`
    pub fn is_positive_infinity(half: Half) -> bool {
        HalfHelper::is_positive_infinity(half)
    }

    /// `Parse(string value)`: `float.Parse` in the invariant culture.
    pub fn parse(value: &str) -> Result<Half, Exception> {
        Ok(Half::new(float_parse_culture(value, &INVARIANT_CULTURE)?))
    }

    /// `Parse(string value, IFormatProvider provider)`
    pub fn parse_provider(value: &str, provider: &CultureInfo) -> Result<Half, Exception> {
        Ok(Half::new(float_parse_culture(value, provider)?))
    }

    /// `TryParse(string value, out Half result)`: `float.TryParse` in the current culture.
    pub fn try_parse(value: &str) -> Option<Half> {
        float_try_parse_culture(value, current_culture()).map(Half::new)
    }

    /// `TryParse(string value, NumberStyles style, IFormatProvider provider, out Half result)`
    /// with the default float style.
    pub fn try_parse_provider(value: &str, provider: &CultureInfo) -> Option<Half> {
        float_try_parse_culture(value, provider).map(Half::new)
    }

    /// `ToString(IFormatProvider formatProvider)`
    pub fn to_string_provider(self, format_provider: &CultureInfo) -> String {
        float_to_string_culture(self.to_f32(), format_provider)
    }

    /// `IConvertible.ToBoolean`: `Convert.ToBoolean(float)`.
    pub fn to_boolean(self) -> bool {
        self.to_f32() != 0.0
    }

    /// `IConvertible.ToChar` / `ToDateTime`: invalid casts.
    pub fn to_char_convertible(self) -> Result<char, Exception> {
        Err(Exception::InvalidCast("Invalid cast from 'Half' to 'Char'.".into()))
    }

    /// `IConvertible.ToInt32`: `Convert.ToInt32(float)` rounds half to even and checks range.
    pub fn convert_to_int32(self) -> Result<i32, Exception> {
        let d = self.to_f64();
        if (-2147483648.5..2147483647.5).contains(&d) {
            let r = d.round_ties_even();
            return Ok(r as i32);
        }
        Err(Exception::Overflow("Value was either too large or too small for an Int32.".into()))
    }
}

impl PartialEq for Half {
    /// `operator ==` (NaN on the left is never equal; +0 and -0 differ).
    fn eq(&self, other: &Half) -> bool {
        Half::op_equality(*self, *other)
    }

    /// `operator !=`: the bits compared, so it is not the negation of `==` for NaN.
    #[allow(clippy::partialeq_ne_impl)]
    fn ne(&self, other: &Half) -> bool {
        Half::op_inequality(*self, *other)
    }
}

impl PartialOrd for Half {
    fn partial_cmp(&self, other: &Half) -> Option<std::cmp::Ordering> {
        self.to_f32().partial_cmp(&other.to_f32())
    }

    /// `operator <`
    fn lt(&self, other: &Half) -> bool {
        Half::op_less_than(*self, *other)
    }

    /// `operator <=`: `(half1 == half2) || (half1 < half2)`, so +0 <= -0 is false.
    fn le(&self, other: &Half) -> bool {
        Half::op_less_than_or_equal(*self, *other)
    }

    /// `operator >`
    fn gt(&self, other: &Half) -> bool {
        Half::op_greater_than(*self, *other)
    }

    /// `operator >=`
    fn ge(&self, other: &Half) -> bool {
        Half::op_greater_than_or_equal(*self, *other)
    }
}

impl std::ops::Neg for Half {
    type Output = Half;
    fn neg(self) -> Half {
        HalfHelper::negate(self)
    }
}

impl std::ops::Add for Half {
    type Output = Half;
    fn add(self, o: Half) -> Half {
        Half::new(self.to_f32() + o.to_f32())
    }
}

impl std::ops::Sub for Half {
    type Output = Half;
    fn sub(self, o: Half) -> Half {
        Half::new(self.to_f32() - o.to_f32())
    }
}

impl std::ops::Mul for Half {
    type Output = Half;
    fn mul(self, o: Half) -> Half {
        Half::new(self.to_f32() * o.to_f32())
    }
}

impl std::ops::Div for Half {
    type Output = Half;
    fn div(self, o: Half) -> Half {
        Half::new(self.to_f32() / o.to_f32())
    }
}

impl std::fmt::Display for Half {
    /// `ToString()`: the float value in the invariant culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&float_to_string_culture(self.to_f32(), &INVARIANT_CULTURE))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn behaves_like_the_csharp() {
        let one = Half::new(1.0);
        assert_eq!((one + one).to_f32(), 2.0);
        assert_eq!(one.to_string(), "1");
        // C#: NaN == NaN is false, and NaN != NaN is also false (bits equal)
        assert!(!(Half::NAN == Half::NAN));
        assert!(!(Half::NAN != Half::NAN));
        assert!(!(Half::new(0.0) <= Half::new(-0.0)));
        assert_eq!(Half::from_f64(f64::NAN).to_u64(), 0);
        assert!(Half::NAN.equals(Half::NAN));
        // +0 and -0 have different bits
        assert!(Half::new(0.0) != Half::new(-0.0));
        assert!(Half::sign(Half::NAN).is_err());
        assert_eq!(Half::parse("1.5").unwrap().to_f32(), 1.5);
        assert_eq!(Half::to_half_bytes(&[0x00, 0x3c], 0).unwrap(), one);
        assert_eq!(Half::NAN.compare_to(one), -1);
    }
}
