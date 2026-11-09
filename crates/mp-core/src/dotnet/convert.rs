//! `System.Convert` and primitive `CompareTo` behaviour (.NET Framework 4.x reference source)
//! that the ported ExtLibs/Core and ExtLibs/Utilities files need.

use super::{current_culture, group_separator, CultureInfo, Exception};

fn overflow_int32() -> Exception {
    Exception::Overflow("Value was either too large or too small for an Int32.".into())
}

/// `Convert.ToInt32(double)`: banker's rounding, `OverflowException` outside the int range
/// (and for NaN).
pub fn to_int32_f64(value: f64) -> Result<i32, Exception> {
    if value >= -2147483648.5 && value < 2147483647.5 {
        let mut result = value as i32;
        let dif = value - result as f64;
        if dif > 0.5 || (dif == 0.5 && (result & 1) != 0) {
            result += 1;
        } else if dif < -0.5 || (dif == -0.5 && (result & 1) != 0) {
            result -= 1;
        }
        return Ok(result);
    }
    Err(overflow_int32())
}

/// `Convert.ToInt64(double)`
pub fn to_int64_f64(value: f64) -> Result<i64, Exception> {
    if value > -9223372036854775809.0 && value < 9223372036854775807.0 {
        let r = value.round_ties_even();
        if r >= -9223372036854775808.0 && r < 9223372036854775808.0 {
            return Ok(r as i64);
        }
    }
    Err(Exception::Overflow("Value was either too large or too small for an Int64.".into()))
}

/// `ParseNumbers.StringToInt(s, radix, IsTight [| TreatAsUnsigned])`, the parser behind
/// `Convert.ToInt32(string, fromBase)` / `Convert.ToByte(string, fromBase)`. Returns the
/// 32-bit result before the caller's range check.
fn string_to_int(s: &str, radix: u32, treat_as_unsigned: bool) -> Result<i32, Exception> {
    if ![2, 8, 10, 16].contains(&radix) {
        return Err(Exception::Argument("Invalid Base.".into()));
    }
    let chars: Vec<char> = s.chars().collect();
    if chars.is_empty() {
        return Err(Exception::ArgumentOutOfRange("Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: startIndex".into()));
    }
    let mut i = 0usize;
    let mut sign = 1i64;
    if chars[i] == '-' {
        if radix != 10 {
            return Err(Exception::Argument("String cannot contain a minus sign if the base is not 10.".into()));
        }
        if treat_as_unsigned {
            return Err(Exception::Overflow("The string was being parsed as an unsigned number and could not have a negative sign.".into()));
        }
        sign = -1;
        i += 1;
    } else if chars[i] == '+' {
        i += 1;
    }
    if radix == 16 && i + 1 < chars.len() && chars[i] == '0' && (chars[i + 1] == 'x' || chars[i + 1] == 'X') {
        i += 2;
    }
    let start = i;
    let mut result: u32 = 0;
    let max_val: u32 = if radix == 10 && !treat_as_unsigned { 0x7FFF_FFFF } else { u32::MAX };
    while i < chars.len() {
        let Some(d) = chars[i].to_digit(36).filter(|&d| d < radix) else { break };
        let next = (result as u64) * radix as u64 + d as u64;
        if next > max_val as u64 {
            // GrabInts: radix 10 signed allows exactly 0x80000000 only when negative
            if radix == 10 && !treat_as_unsigned && sign == -1 && next == 0x8000_0000 && i + 1 == chars.len() {
                return Ok(i32::MIN);
            }
            return Err(overflow_int32());
        }
        result = next as u32;
        i += 1;
    }
    if i == start {
        return Err(Exception::Format("Could not find any recognizable digits.".into()));
    }
    if i < chars.len() {
        return Err(Exception::Format("Additional non-parsable characters are at the end of the string.".into()));
    }
    Ok(if sign == -1 { -(result as i64) as i32 } else { result as i32 })
}

/// `Convert.ToInt32(string value, int fromBase)`; a null value converts to 0.
pub fn to_int32_from_base(value: Option<&str>, from_base: u32) -> Result<i32, Exception> {
    match value {
        None => Ok(0),
        Some(s) => string_to_int(s, from_base, false),
    }
}

/// `Convert.ToByte(string value, int fromBase)`; a null value converts to 0.
pub fn to_byte_from_base(value: Option<&str>, from_base: u32) -> Result<u8, Exception> {
    let Some(s) = value else { return Ok(0) };
    let r = string_to_int(s, from_base, true)?;
    if !(0..=255).contains(&r) {
        return Err(Exception::Overflow("Value was either too large or too small for an unsigned byte.".into()));
    }
    Ok(r as u8)
}

/// `double.CompareTo(double)`: NaN sorts below everything and equals itself.
pub fn double_compare_to(a: f64, b: f64) -> i32 {
    if a < b {
        return -1;
    }
    if a > b {
        return 1;
    }
    if a == b {
        return 0;
    }
    if a.is_nan() {
        if b.is_nan() {
            0
        } else {
            -1
        }
    } else {
        1
    }
}

/// `int.TryParse(s, NumberStyles.AllowThousands, culture, out v)`: digits and group
/// separators only (no white space, no sign).
pub fn int_try_parse_allow_thousands(s: &str, c: &CultureInfo) -> Option<i32> {
    let grp = group_separator(c);
    let mut rest = s;
    let mut digits = String::new();
    loop {
        if let Some(ch) = rest.chars().next().filter(|ch| ch.is_ascii_digit()) {
            digits.push(ch);
            rest = &rest[1..];
        } else if !digits.is_empty() && !grp.is_empty() && rest.starts_with(grp) {
            rest = &rest[grp.len()..];
        } else {
            break;
        }
    }
    if digits.is_empty() || !rest.is_empty() {
        return None;
    }
    digits.parse::<i32>().ok()
}

/// `int.TryParse(s, NumberStyles.AllowThousands, CultureInfo.CurrentCulture, out v)`
pub fn int_try_parse_allow_thousands_current(s: &str) -> Option<i32> {
    int_try_parse_allow_thousands(s, current_culture())
}

/// `Convert.ToDouble(string)` in the current culture: null converts to 0.
pub fn to_double_str(value: Option<&str>) -> Result<f64, Exception> {
    match value {
        None => Ok(0.0),
        Some(s) => super::double_parse(s),
    }
}

/// `string.CompareTo(string)` (culture-sensitive; a null argument sorts first).
pub fn string_compare_to(a: &str, b: Option<&str>) -> i32 {
    match b {
        None => 1,
        Some(b) => super::culture_compare(a, b) as i32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convert_rules() {
        assert_eq!(to_int32_f64(2.5).unwrap(), 2);
        assert_eq!(to_int32_f64(3.5).unwrap(), 4);
        assert_eq!(to_int32_f64(-2.5).unwrap(), -2);
        assert!(to_int32_f64(f64::NAN).is_err());
        assert!(to_int32_f64(3e9).is_err());
        assert_eq!(to_int32_from_base(Some("BE"), 16).unwrap(), 190);
        assert_eq!(to_int32_from_base(Some("0xff"), 16).unwrap(), 255);
        assert!(matches!(to_int32_from_base(Some("0x"), 16), Err(Exception::Format(_))));
        assert!(matches!(to_int32_from_base(Some("G1"), 16), Err(Exception::Format(_))));
        assert!(matches!(to_int32_from_base(Some("-1"), 16), Err(Exception::Argument(_))));
        assert_eq!(to_byte_from_base(Some("fe"), 16).unwrap(), 254);
        assert_eq!(double_compare_to(f64::NAN, 1.0), -1);
        assert_eq!(double_compare_to(f64::NAN, f64::NAN), 0);
        let en = super::super::get_culture_info("en-US").unwrap();
        assert_eq!(int_try_parse_allow_thousands("30,000", en), Some(30000));
        assert_eq!(int_try_parse_allow_thousands("-3", en), None);
        assert_eq!(int_try_parse_allow_thousands(" 3", en), None);
    }
}
