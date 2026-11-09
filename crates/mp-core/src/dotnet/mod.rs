//! .NET Framework 4.7.2 base-class-library behaviour that the ported C# relies on.
//!
//! This module is not a port of a Mission Planner file. It stands in for the BCL calls the C#
//! makes: number and date formatting under the current culture, `Math.Round`, numeric casts,
//! exceptions, `DateTime`/`TimeSpan` and `Stream`. Behaviour follows the .NET Framework runtime
//! Mission Planner ships on (`net472`), not .NET Core. Members are added only when a ported
//! file needs them.

use std::sync::OnceLock;

pub mod collections;
pub mod crypto;
pub mod drawing;
pub mod environment;
pub mod io;
pub mod json;
pub mod net;
pub mod ports;
pub mod uri;
pub mod version;
pub mod xml;

/// `NumberFormat.NumberGroupSeparator` for the cultures in the table.
pub fn group_separator(c: &CultureInfo) -> &'static str {
    match c.name.split('-').next().unwrap_or("") {
        "" | "en" | "ja" | "ko" | "zh" | "ar" => ",",
        "de" | "it" | "nl" | "id" | "tr" | "es" => ".",
        "pt" if c.name == "pt-BR" => ".",
        _ => "\u{a0}",
    }
}

/// Canonical text for a number in the given culture under `NumberStyles.Float |
/// AllowThousands` (exponent allowed) or `NumberStyles.Number` (no exponent): surrounding
/// white space, a leading sign, group separators in the integer part, one decimal separator.
fn normalize_number(s: &str, c: &CultureInfo, allow_exponent: bool) -> Option<String> {
    let t = s.trim_matches(|ch: char| ch.is_whitespace() || ch == '\0');
    let (neg, body) = if let Some(r) = t.strip_prefix('-') {
        (true, r)
    } else if let Some(r) = t.strip_prefix('+') {
        (false, r)
    } else {
        (false, t)
    };
    let dec = c.decimal_separator;
    let grp = group_separator(c);
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    let mut rest = body;
    let mut int_digits = 0;
    loop {
        if let Some(ch) = rest.chars().next().filter(|ch| ch.is_ascii_digit()) {
            out.push(ch);
            int_digits += 1;
            rest = &rest[1..];
        } else if int_digits > 0 && !grp.is_empty() && rest.starts_with(grp) && !rest.starts_with(dec) {
            rest = &rest[grp.len()..];
        } else {
            break;
        }
    }
    let mut frac_digits = 0;
    if let Some(r) = rest.strip_prefix(dec) {
        out.push('.');
        rest = r;
        while let Some(ch) = rest.chars().next().filter(|ch| ch.is_ascii_digit()) {
            out.push(ch);
            frac_digits += 1;
            rest = &rest[1..];
        }
    }
    if int_digits + frac_digits == 0 {
        return None;
    }
    if allow_exponent {
        if let Some(r) = rest.strip_prefix(['e', 'E']) {
            let (sign, r) = if let Some(x) = r.strip_prefix('-') {
                ("-", x)
            } else if let Some(x) = r.strip_prefix('+') {
                ("", x)
            } else {
                ("", r)
            };
            let digits: String = r.chars().take_while(|ch| ch.is_ascii_digit()).collect();
            if digits.is_empty() {
                return None;
            }
            out.push('e');
            out.push_str(sign);
            out.push_str(&digits);
            rest = &r[digits.len()..];
        }
    }
    if !rest.is_empty() {
        return None;
    }
    Some(out)
}

/// `double.TryParse(s, out v)` under the given culture (`NumberStyles.Float | AllowThousands`).
pub fn double_try_parse_culture(s: &str, c: &CultureInfo) -> Option<f64> {
    let t = s.trim();
    match t {
        "Infinity" => return Some(f64::INFINITY),
        "-Infinity" => return Some(f64::NEG_INFINITY),
        "NaN" => return Some(f64::NAN),
        _ => {}
    }
    let v: f64 = normalize_number(s, c, true)?.parse().ok()?;
    // .NET Framework reports an overflow as a failed parse
    if v.is_infinite() {
        return None;
    }
    Some(v)
}

/// `double.TryParse(s, out v)` in the current culture.
pub fn double_try_parse(s: &str) -> Option<f64> {
    double_try_parse_culture(s, current_culture())
}

/// `float.TryParse(s, out v)` in the current culture.
pub fn float_try_parse(s: &str) -> Option<f32> {
    let v = double_try_parse(s)?;
    if v.is_finite() && v.abs() > f32::MAX as f64 {
        return None;
    }
    Some(v as f32)
}

/// `decimal.TryParse(s, out v)` in the current culture (`NumberStyles.Number`).
pub fn decimal_try_parse(s: &str) -> Option<rust_decimal::Decimal> {
    decimal_parse(s).ok()
}

/// `System.Decimal`: a scaled integer that keeps its scale ("50.00" stays 2 decimals).
pub use rust_decimal::Decimal;

/// `decimal.Parse(s)` in the current culture (`NumberStyles.Number`: leading/trailing white,
/// a leading or trailing sign, a decimal point and thousands separators). A value beyond
/// +-79228162514264337593543950335 throws OverflowException; digits past 28 are rounded.
pub fn decimal_parse(s: &str) -> Result<Decimal, Exception> {
    let format = || Exception::Format("Input string was not in a correct format.".into());
    let t = s.trim_matches(|ch: char| ch.is_whitespace() || ch == '\0');
    let has_leading = t.starts_with('-') || t.starts_with('+');
    let (body, trailing_neg) = match (t.strip_suffix('-'), t.strip_suffix('+')) {
        (Some(b), _) if !has_leading => (b, true),
        (_, Some(b)) if !has_leading => (b, false),
        _ => (t, false),
    };
    if body.ends_with(|ch: char| ch.is_whitespace()) && body.len() != t.len() {
        return Err(format());
    }
    let norm = normalize_number(body, current_culture(), false).ok_or_else(format)?;
    let norm = if trailing_neg { format!("-{norm}") } else { norm };
    norm.parse::<Decimal>().map_err(|_| Exception::Overflow("Value was either too large or too small for a Decimal.".into()))
}

/// `(decimal)double`: the double rounded to 15 significant digits, trailing zeros dropped.
pub fn decimal_from_double(x: f64) -> Result<Decimal, Exception> {
    if !x.is_finite() || x.abs() >= 7.922_816_251_426_434e28 {
        return Err(Exception::Overflow("Value was either too large or too small for a Decimal.".into()));
    }
    if x == 0.0 {
        return Ok(Decimal::ZERO);
    }
    // 15 significant digits as mantissa * 10^(exp - 14); the scale is capped at 28, the
    // digits beyond it rounded half to even
    let sci = format!("{:.14e}", x.abs());
    let (m, e) = sci.split_once('e').unwrap_or((&sci, "0"));
    let mut mant: i128 = m.replace('.', "").parse().unwrap_or(0);
    let exp: i32 = e.parse().unwrap_or(0);
    let mut scale = 14 - exp;
    if scale > 28 {
        let drop = (scale - 28) as u32;
        if drop > 15 {
            return Ok(Decimal::ZERO);
        }
        let div = 10i128.pow(drop);
        let (q, r) = (mant / div, mant % div);
        mant = if r * 2 > div || (r * 2 == div && q % 2 == 1) { q + 1 } else { q };
        scale = 28;
    }
    while scale < 0 {
        mant *= 10;
        scale += 1;
    }
    let mut d = Decimal::from_i128_with_scale(mant, scale as u32);
    if x < 0.0 {
        d.set_sign_negative(true);
    }
    Ok(d.normalize())
}

/// `(double)decimal` (VarR8FromDec): `((double)lo64 + (double)hi32 * 2^64) / 10^scale`.
pub fn decimal_to_double(d: Decimal) -> f64 {
    const POW10: [f64; 29] = [
        1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22, 1e23,
        1e24, 1e25, 1e26, 1e27, 1e28,
    ];
    let m = d.mantissa().unsigned_abs();
    let lo64 = (m & u64::MAX as u128) as u64;
    let hi32 = (m >> 64) as u64;
    let v = (lo64 as f64 + hi32 as f64 * 18_446_744_073_709_551_616.0) / POW10[d.scale() as usize];
    if d.is_sign_negative() { -v } else { v }
}

/// `decimal.ToString()` in the current culture: every digit of the scale is kept.
pub fn decimal_to_string(d: Decimal) -> String {
    d.to_string().replace('.', current_culture().decimal_separator)
}

/// `decimal.ToString("F<n>")` / a "0.00"-style custom format: `decimals` places, rounded half
/// away from zero, with at least `min_int_digits` integer digits.
pub fn decimal_format_fixed(d: Decimal, min_int_digits: usize, decimals: u32, c: &CultureInfo) -> String {
    use rust_decimal::RoundingStrategy;
    let r = d.round_dp_with_strategy(decimals, RoundingStrategy::MidpointAwayFromZero);
    let neg = r.is_sign_negative() && !r.is_zero();
    let s = format!("{:.*}", decimals as usize, r.abs());
    let (int_part, frac) = match s.split_once('.') {
        Some((a, b)) => (a.to_string(), b.to_string()),
        None => (s.clone(), String::new()),
    };
    let int_part = if int_part == "0" && min_int_digits == 0 { String::new() } else { int_part };
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    for _ in int_part.len()..min_int_digits {
        out.push('0');
    }
    out.push_str(&int_part);
    if decimals > 0 {
        out.push_str(c.decimal_separator);
        out.push_str(&frac);
    }
    out
}

/// `decimal.ToString("0.##")`-style custom format: rounded to `max_decimals`, trailing zeros
/// dropped down to `min_decimals`.
pub fn decimal_format_fixed_optional(d: Decimal, min_int_digits: usize, min_decimals: u32, max_decimals: u32, c: &CultureInfo) -> String {
    let s = decimal_format_fixed(d, min_int_digits, max_decimals, c);
    if max_decimals == 0 {
        return s;
    }
    let Some(pos) = s.rfind(c.decimal_separator) else {
        return s;
    };
    let (head, frac) = s.split_at(pos + c.decimal_separator.len());
    let mut frac = frac.to_string();
    while frac.len() > min_decimals as usize && frac.ends_with('0') {
        frac.pop();
    }
    if frac.is_empty() {
        return head[..pos].to_string();
    }
    format!("{head}{frac}")
}

/// `int.TryParse(s, out v)`
pub fn int_try_parse(s: &str) -> Option<i32> {
    int_parse(s).ok()
}

/// `byte.TryParse(s, out v)`
pub fn byte_try_parse(s: &str) -> Option<u8> {
    let v = int_parse(s).ok()?;
    u8::try_from(v).ok()
}

/// `bool.TryParse(s, out v)`: "True"/"False" in any case, white space and NULs trimmed.
pub fn bool_try_parse(s: &str) -> Option<bool> {
    let t = s.trim_matches(|ch: char| ch.is_whitespace() || ch == '\0');
    if t.eq_ignore_ascii_case("true") {
        Some(true)
    } else if t.eq_ignore_ascii_case("false") {
        Some(false)
    } else {
        None
    }
}

/// `WebUtility.UrlEncode`: space to '+', `A-Z a-z 0-9 - _ . ! * ( )` kept, everything else as
/// `%XX` (upper case) of its UTF-8 bytes.
pub fn url_encode(s: &str) -> String {
    let mut out = String::new();
    for b in s.bytes() {
        match b {
            b' ' => out.push('+'),
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'!' | b'*' | b'(' | b')' => out.push(b as char),
            _ => out.push_str(&format!("%{:02X}", b)),
        }
    }
    out
}

/// `WebUtility.UrlDecode`: '+' to space, `%XX` as UTF-8 bytes; malformed escapes kept.
pub fn url_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'+' => out.push(b' '),
            b'%' if i + 2 < b.len() => {
                match std::str::from_utf8(&b[i + 1..i + 3]).ok().and_then(|h| u8::from_str_radix(h, 16).ok()) {
                    Some(v) => {
                        out.push(v);
                        i += 3;
                        continue;
                    }
                    None => out.push(b'%'),
                }
            }
            x => out.push(x),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// `string.Compare(a, b)` in the current culture, as used by `OrderBy(a => a)`: Windows NLS
/// word sort for the characters Mission Planner keys use. Punctuation sorts before digits,
/// digits before letters, letters compare case-insensitively first, then lower case before
/// upper case, then the shorter string first.
pub fn culture_compare(a: &str, b: &str) -> std::cmp::Ordering {
    fn class(c: char) -> (u8, char) {
        if c.is_ascii_digit() {
            (1, c)
        } else if c.is_alphabetic() {
            (2, c.to_lowercase().next().unwrap_or(c))
        } else {
            (0, c)
        }
    }
    let pa: Vec<(u8, char)> = a.chars().map(class).collect();
    let pb: Vec<(u8, char)> = b.chars().map(class).collect();
    match pa.cmp(&pb) {
        std::cmp::Ordering::Equal => {}
        o => return o,
    }
    // tertiary: lower case first
    for (x, y) in a.chars().zip(b.chars()) {
        if x != y {
            return match (x.is_lowercase(), y.is_lowercase()) {
                (true, false) => std::cmp::Ordering::Less,
                (false, true) => std::cmp::Ordering::Greater,
                _ => x.cmp(&y),
            };
        }
    }
    a.len().cmp(&b.len())
}

/// A C# `enum`: a newtype over the underlying integer, so every value converts in as in C#,
/// with the members as associated constants, `ToString()` (member name or the number),
/// `Enum.Parse`, `Enum.GetName` and `Enum.GetValues`.
#[macro_export]
macro_rules! cs_enum {
    ($(#[$m:meta])* $vis:vis $name:ident : $t:ty { $($(#[$vm:meta])* $var:ident = $val:expr),* $(,)? }) => {
        $(#[$m])*
        #[allow(non_camel_case_types)]
        #[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
        $vis struct $name(pub $t);

        #[allow(non_upper_case_globals, dead_code)]
        impl $name {
            $( $(#[$vm])* pub const $var: $name = $name($val); )*

            /// (name, value) in declaration order.
            pub const MEMBERS: &'static [(&'static str, $t)] = &[$((stringify!($var), $val)),*];

            /// The members as .NET Framework's `GetEnumValuesAndNames` holds them: declaration
            /// order sorted by value as `ulong` with `Array.Sort` (introsort).
            fn sorted_members() -> Vec<(&'static str, $t)> {
                let mut v: Vec<(&'static str, $t)> = Self::MEMBERS.to_vec();
                let n = v.len();
                let _ = $crate::dotnet::collections::introspective_sort(&mut v, n, |a: &(&'static str, $t), b: &(&'static str, $t)| {
                    Ok((a.1 as i64 as u64).cmp(&(b.1 as i64 as u64)) as i32)
                });
                v
            }

            /// `Enum.GetName`: a binary search of the sorted values, so for a value several
            /// members share it returns the one .NET's search lands on.
            pub fn get_name(self) -> Option<&'static str> {
                let v = Self::sorted_members();
                let key = self.0 as i64 as u64;
                let (mut lo, mut hi) = (0isize, v.len() as isize - 1);
                while lo <= hi {
                    let i = lo + ((hi - lo) >> 1);
                    let c = v[i as usize].1 as i64 as u64;
                    if c == key {
                        return Some(v[i as usize].0);
                    }
                    if c < key {
                        lo = i + 1;
                    } else {
                        hi = i - 1;
                    }
                }
                None
            }

            /// `Enum.IsDefined`
            pub fn is_defined(self) -> bool {
                self.get_name().is_some()
            }

            /// `Enum.Parse(type, s)`: a member name (case sensitive) or a number.
            pub fn parse(s: &str) -> Option<Self> {
                let s = s.trim();
                if let Some(m) = Self::MEMBERS.iter().find(|m| m.0 == s) {
                    return Some($name(m.1));
                }
                s.parse::<$t>().ok().map($name)
            }

            /// `Enum.Parse(type, s, ignoreCase: true)`
            pub fn parse_ignore_case(s: &str) -> Option<Self> {
                let s = s.trim();
                if let Some(m) = Self::MEMBERS.iter().find(|m| m.0.eq_ignore_ascii_case(s)) {
                    return Some($name(m.1));
                }
                s.parse::<$t>().ok().map($name)
            }

            /// `Enum.GetValues`: the members sorted by value as `ulong`.
            pub fn get_values() -> Vec<Self> {
                Self::sorted_members().into_iter().map(|m| $name(m.1)).collect()
            }

            /// `Enum.GetNames`, in `GetValues` order.
            pub fn get_names() -> Vec<&'static str> {
                Self::sorted_members().into_iter().map(|m| m.0).collect()
            }
        }

        impl ::std::fmt::Display for $name {
            fn fmt(&self, f: &mut ::std::fmt::Formatter<'_>) -> ::std::fmt::Result {
                match self.get_name() {
                    Some(n) => f.write_str(n),
                    None => write!(f, "{}", self.0),
                }
            }
        }

        impl ::std::fmt::Debug for $name {
            fn fmt(&self, f: &mut ::std::fmt::Formatter<'_>) -> ::std::fmt::Result {
                ::std::fmt::Display::fmt(self, f)
            }
        }

        impl From<$t> for $name {
            fn from(v: $t) -> Self {
                $name(v)
            }
        }
    };
}

/// `double.ToString("000.00", culture)`-style custom format: at least `min_int_digits` integer
/// digits and exactly `decimals` decimals. Like .NET, the value is taken to 15 significant
/// digits and then rounded half away from zero; a result of zero has no minus sign.
pub fn format_fixed(x: f64, min_int_digits: usize, decimals: u32, c: &CultureInfo) -> String {
    format_fixed_sig(x, 15, min_int_digits, decimals, c)
}

/// `float.ToString("000.00", culture)`: as `format_fixed`, but a float carries only 7
/// significant digits into the formatting.
pub fn format_fixed_float(x: f32, min_int_digits: usize, decimals: u32, c: &CultureInfo) -> String {
    format_fixed_sig(x as f64, 7, min_int_digits, decimals, c)
}

fn format_fixed_sig(x: f64, sig_digits: usize, min_int_digits: usize, decimals: u32, c: &CultureInfo) -> String {
    use rust_decimal::{Decimal, RoundingStrategy};
    if x.is_nan() {
        return "NaN".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "Infinity".into() } else { "-Infinity".into() };
    }
    let sci = format!("{:.*e}", sig_digits - 1, x);
    let d = Decimal::from_scientific(&sci).unwrap_or_default();
    let r = d.round_dp_with_strategy(decimals, RoundingStrategy::MidpointAwayFromZero);
    let neg = r.is_sign_negative() && !r.is_zero();
    let s = format!("{:.*}", decimals as usize, r.abs());
    let (int_part, frac) = match s.split_once('.') {
        Some((a, b)) => (a.to_string(), b.to_string()),
        None => (s.clone(), String::new()),
    };
    let int_part = if int_part == "0" && min_int_digits == 0 { String::new() } else { int_part };
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    for _ in int_part.len()..min_int_digits {
        out.push('0');
    }
    out.push_str(&int_part);
    if decimals > 0 {
        out.push_str(c.decimal_separator);
        out.push_str(&frac);
    }
    out
}

/// Custom format with optional decimals, e.g. `"#0.0#####"` is `format_fixed_optional(x, 1,
/// 1, 6, c)`: rounded to `max_decimals`, then trailing zeros dropped down to `min_decimals`.
pub fn format_fixed_optional(x: f64, min_int_digits: usize, min_decimals: u32, max_decimals: u32, c: &CultureInfo) -> String {
    let s = format_fixed(x, min_int_digits, max_decimals, c);
    if max_decimals == 0 || !x.is_finite() {
        return s;
    }
    let Some(pos) = s.rfind(c.decimal_separator) else {
        return s;
    };
    let (head, frac) = s.split_at(pos + c.decimal_separator.len());
    let mut frac = frac.to_string();
    while frac.len() > min_decimals as usize && frac.ends_with('0') {
        frac.pop();
    }
    if frac.is_empty() {
        return head[..pos].to_string();
    }
    format!("{head}{frac}")
}

/// `System.BitConverter` reads (little endian), with the .NET argument checks.
pub mod bit_converter {
    use super::Exception;

    fn take<const N: usize>(value: &[u8], start_index: i32) -> Result<[u8; N], Exception> {
        if start_index < 0 || start_index as usize >= value.len() {
            return Err(Exception::ArgumentOutOfRange(
                "Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: startIndex".into(),
            ));
        }
        let s = start_index as usize;
        if s + N > value.len() {
            return Err(Exception::Argument(
                "Destination array is not long enough to copy all the items in the collection. Check array index and length.".into(),
            ));
        }
        let mut a = [0u8; N];
        a.copy_from_slice(&value[s..s + N]);
        Ok(a)
    }

    pub fn to_u16(value: &[u8], start_index: i32) -> Result<u16, Exception> {
        take::<2>(value, start_index).map(u16::from_le_bytes)
    }
    pub fn to_i16(value: &[u8], start_index: i32) -> Result<i16, Exception> {
        take::<2>(value, start_index).map(i16::from_le_bytes)
    }
    pub fn to_u32(value: &[u8], start_index: i32) -> Result<u32, Exception> {
        take::<4>(value, start_index).map(u32::from_le_bytes)
    }
    pub fn to_i32(value: &[u8], start_index: i32) -> Result<i32, Exception> {
        take::<4>(value, start_index).map(i32::from_le_bytes)
    }
    pub fn to_single(value: &[u8], start_index: i32) -> Result<f32, Exception> {
        take::<4>(value, start_index).map(f32::from_le_bytes)
    }
    pub fn to_u64(value: &[u8], start_index: i32) -> Result<u64, Exception> {
        take::<8>(value, start_index).map(u64::from_le_bytes)
    }
    pub fn to_double(value: &[u8], start_index: i32) -> Result<f64, Exception> {
        take::<8>(value, start_index).map(f64::from_le_bytes)
    }
}

/// `int.ToString("00")`: zero-padded to `digits`, the sign in front of the padding.
pub fn format_int_zero(v: i64, digits: usize) -> String {
    if v < 0 {
        format!("-{:0w$}", v.unsigned_abs(), w = digits)
    } else {
        format!("{:0w$}", v, w = digits)
    }
}

/// The parts of `System.Globalization.CultureInfo` the ported code formats with.
///
/// Mission Planner never assigns `CurrentCulture` (MainV2.changelanguage only sets
/// `CurrentUICulture`), so formatting follows the user's OS regional settings.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CultureInfo {
    pub name: &'static str,
    /// `NumberFormat.NumberDecimalSeparator`
    pub decimal_separator: &'static str,
    /// `DateTimeFormat.ShortDatePattern`
    pub short_date_pattern: &'static str,
    /// `DateTimeFormat.LongTimePattern`
    pub long_time_pattern: &'static str,
    pub am_designator: &'static str,
    pub pm_designator: &'static str,
}

const fn culture(
    name: &'static str,
    dec: &'static str,
    date: &'static str,
    time: &'static str,
    am: &'static str,
    pm: &'static str,
) -> CultureInfo {
    CultureInfo { name, decimal_separator: dec, short_date_pattern: date, long_time_pattern: time, am_designator: am, pm_designator: pm }
}

/// Windows 10 NLS data for the cultures Mission Planner ships translations for, plus the
/// common English variants.
static CULTURES: &[CultureInfo] = &[
    culture("en-US", ".", "M/d/yyyy", "h:mm:ss tt", "AM", "PM"),
    culture("en-GB", ".", "dd/MM/yyyy", "HH:mm:ss", "AM", "PM"),
    culture("en-AU", ".", "d/MM/yyyy", "h:mm:ss tt", "AM", "PM"),
    culture("en-CA", ".", "yyyy-MM-dd", "h:mm:ss tt", "a.m.", "p.m."),
    culture("en-IN", ".", "dd-MM-yyyy", "HH:mm:ss", "AM", "PM"),
    culture("de-DE", ",", "dd.MM.yyyy", "HH:mm:ss", "", ""),
    culture("fr-FR", ",", "dd/MM/yyyy", "HH:mm:ss", "", ""),
    culture("es-ES", ",", "dd/MM/yyyy", "H:mm:ss", "", ""),
    culture("it-IT", ",", "dd/MM/yyyy", "HH:mm:ss", "", ""),
    culture("pt-BR", ",", "dd/MM/yyyy", "HH:mm:ss", "", ""),
    culture("pt-PT", ",", "dd/MM/yyyy", "HH:mm:ss", "", ""),
    culture("pl-PL", ",", "dd.MM.yyyy", "HH:mm:ss", "", ""),
    culture("ru-RU", ",", "dd.MM.yyyy", "H:mm:ss", "", ""),
    culture("ru-KZ", ",", "dd.MM.yyyy", "H:mm:ss", "", ""),
    culture("uk-UA", ",", "dd.MM.yyyy", "H:mm:ss", "", ""),
    culture("tr-TR", ",", "d.MM.yyyy", "HH:mm:ss", "ÖÖ", "ÖS"),
    culture("az-Latn-AZ", ",", "dd.MM.yyyy", "H:mm:ss", "AM", "PM"),
    culture("id-ID", ",", "dd/MM/yyyy", "H:mm:ss", "", ""),
    culture("ar-SA", ".", "dd/MM/yy", "hh:mm:ss tt", "ص", "م"),
    culture("ja-JP", ".", "yyyy/MM/dd", "H:mm:ss", "午前", "午後"),
    culture("ko-KR", ".", "yyyy-MM-dd", "tt h:mm:ss", "오전", "오후"),
    culture("zh-CN", ".", "yyyy/M/d", "H:mm:ss", "上午", "下午"),
    culture("zh-TW", ".", "yyyy/M/d", "tt hh:mm:ss", "上午", "下午"),
    culture("zh-HK", ".", "d/M/yyyy", "H:mm:ss", "上午", "下午"),
    culture("nl-NL", ",", "d-M-yyyy", "HH:mm:ss", "", ""),
    culture("sv-SE", ",", "yyyy-MM-dd", "HH:mm:ss", "", ""),
    culture("cs-CZ", ",", "dd.MM.yyyy", "H:mm:ss", "dop.", "odp."),
];

/// `CultureInfo.InvariantCulture`
pub static INVARIANT_CULTURE: CultureInfo = culture("", ".", "MM/dd/yyyy", "HH:mm:ss", "AM", "PM");

fn os_locale_name() -> Option<String> {
    #[cfg(windows)]
    {
        use windows::Win32::Globalization::GetUserDefaultLocaleName;
        let mut buf = [0u16; 85];
        let n = unsafe { GetUserDefaultLocaleName(&mut buf) };
        if n > 1 {
            return Some(String::from_utf16_lossy(&buf[..(n - 1) as usize]));
        }
        None
    }
    #[cfg(not(windows))]
    {
        // Mono (get_posix_locale): LC_ALL, then LANG
        let v = ["LC_ALL", "LANG"].iter().find_map(|k| std::env::var(k).ok().filter(|s| !s.is_empty()))?;
        let v = v.split('.').next().unwrap_or("").replace('_', "-");
        if v.is_empty() || v == "C" || v == "POSIX" {
            None
        } else {
            Some(v)
        }
    }
}

/// `CultureInfo.CurrentCulture`: the OS locale, matched by full name then by language;
/// en-US when the locale is unknown.
pub fn current_culture() -> &'static CultureInfo {
    static CURRENT: OnceLock<&'static CultureInfo> = OnceLock::new();
    CURRENT.get_or_init(|| {
        let name = os_locale_name().unwrap_or_default();
        CULTURES
            .iter()
            .find(|c| c.name.eq_ignore_ascii_case(&name))
            .or_else(|| {
                let lang = name.split('-').next().unwrap_or("");
                CULTURES.iter().find(|c| c.name.split('-').next().is_some_and(|l| l.eq_ignore_ascii_case(lang)))
            })
            .unwrap_or(&CULTURES[0])
    })
}

static CURRENT_UI_CULTURE: std::sync::RwLock<Option<String>> = std::sync::RwLock::new(None);

/// The OS display language, which .NET Framework uses for the initial `CurrentUICulture`
/// (`GetUserDefaultUILanguage`), as opposed to the regional format (`CurrentCulture`).
fn os_ui_language_name() -> Option<String> {
    #[cfg(windows)]
    {
        use windows::Win32::Globalization::{GetUserDefaultUILanguage, LCIDToLocaleName};
        let langid = unsafe { GetUserDefaultUILanguage() };
        let mut buf = [0u16; 85];
        let n = unsafe { LCIDToLocaleName(langid as u32, Some(&mut buf), 0) };
        if n > 1 {
            return Some(String::from_utf16_lossy(&buf[..(n - 1) as usize]));
        }
        None
    }
    #[cfg(not(windows))]
    {
        // Mono derives CurrentUICulture from the same POSIX locale as CurrentCulture
        os_locale_name()
    }
}

/// `CultureInfo.CurrentUICulture.Name`: the OS UI language until a caller assigns one
/// (MainV2.changelanguage).
pub fn current_ui_culture_name() -> String {
    if let Some(n) = CURRENT_UI_CULTURE.read().unwrap_or_else(|e| e.into_inner()).clone() {
        return n;
    }
    os_ui_language_name().unwrap_or_else(|| "en-US".into())
}

/// `Thread.CurrentThread.CurrentUICulture = ci`
pub fn set_current_ui_culture(name: &str) {
    *CURRENT_UI_CULTURE.write().unwrap_or_else(|e| e.into_inner()) = Some(name.to_string());
}

/// Look up a culture by name (`CultureInfo.GetCultureInfo`).
pub fn get_culture_info(name: &str) -> Option<&'static CultureInfo> {
    if name.is_empty() {
        return Some(&INVARIANT_CULTURE);
    }
    CULTURES.iter().find(|c| c.name.eq_ignore_ascii_case(name))
}

/// Format with `digits` significant digits using the .NET "G" rules: fixed notation when the
/// decimal exponent is in `-5..digits`, otherwise `d.dddE+XX` with at least two exponent digits.
pub fn format_general(x: f64, digits: usize, c: &CultureInfo) -> String {
    if x.is_nan() {
        return "NaN".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "Infinity".into() } else { "-Infinity".into() };
    }
    if x == 0.0 {
        return "0".into();
    }
    let sci = format!("{:.*e}", digits - 1, x);
    let (mant, exp) = sci.split_once('e').unwrap();
    let exp: i32 = exp.parse().unwrap();
    let neg = mant.starts_with('-');
    let mant = mant.trim_start_matches('-');
    let mut ds: String = mant.chars().filter(|c| c.is_ascii_digit()).collect();
    while ds.len() > 1 && ds.ends_with('0') {
        ds.pop();
    }
    let dec = c.decimal_separator;
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    if exp > -5 && exp < digits as i32 {
        if exp < 0 {
            out.push('0');
            out.push_str(dec);
            for _ in 0..(-exp - 1) {
                out.push('0');
            }
            out.push_str(&ds);
        } else {
            let int_len = exp as usize + 1;
            if ds.len() <= int_len {
                out.push_str(&ds);
                for _ in ds.len()..int_len {
                    out.push('0');
                }
            } else {
                out.push_str(&ds[..int_len]);
                out.push_str(dec);
                out.push_str(&ds[int_len..]);
            }
        }
    } else {
        out.push_str(&ds[..1]);
        if ds.len() > 1 {
            out.push_str(dec);
            out.push_str(&ds[1..]);
        }
        out.push('E');
        out.push(if exp < 0 { '-' } else { '+' });
        out.push_str(&format!("{:02}", exp.abs()));
    }
    out
}

/// `double.ToString()` on .NET Framework: "G15" in the current culture.
pub fn double_to_string(x: f64) -> String {
    format_general(x, 15, current_culture())
}

/// `double.ToString(provider)`: "G15" in the given culture.
pub fn double_to_string_culture(x: f64, c: &CultureInfo) -> String {
    format_general(x, 15, c)
}

/// `float.ToString()` on .NET Framework: "G7" in the current culture.
pub fn float_to_string(x: f32) -> String {
    format_general(x as f64, 7, current_culture())
}

/// `float.ToString(provider)`
pub fn float_to_string_culture(x: f32, c: &CultureInfo) -> String {
    format_general(x as f64, 7, c)
}

/// `Math.Round(x, digits)` (MidpointRounding.ToEven). .NET Framework scales by 10^digits,
/// rounds half to even, and scales back.
pub fn math_round(x: f64, digits: i32) -> f64 {
    if digits == 0 {
        return x.round_ties_even();
    }
    let m = 10f64.powi(digits);
    (x * m).round_ties_even() / m
}

/// `double.GetHashCode()` on .NET Framework: 0 for either zero, otherwise the low and high
/// halves of the bit pattern XORed.
pub fn double_get_hash_code(d: f64) -> i32 {
    if d == 0.0 {
        return 0;
    }
    let value = d.to_bits() as i64;
    (value as i32) ^ ((value >> 32) as i32)
}

/// `BitConverter.DoubleToInt64Bits(d)`
pub fn double_to_int64_bits(d: f64) -> i64 {
    d.to_bits() as i64
}

/// `Math.Max(double, double)`: NaN if either argument is NaN.
pub fn math_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        return f64::NAN;
    }
    if a > b {
        a
    } else {
        b
    }
}

/// `Math.Min(double, double)`: NaN if either argument is NaN.
pub fn math_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        return f64::NAN;
    }
    if a < b {
        a
    } else {
        b
    }
}

/// C# unchecked `(T)double` casts as the x64 JIT emits them: narrow integer targets truncate
/// through a 64-bit `cvttsd2si`, then wrap; `(int)` uses the 32-bit instruction, whose
/// out-of-range result is `int.MinValue`.
pub fn cast_u8(x: f64) -> u8 {
    cast_i64(x) as u8
}
pub fn cast_i8(x: f64) -> i8 {
    cast_i64(x) as i8
}
pub fn cast_u16(x: f64) -> u16 {
    cast_i64(x) as u16
}
pub fn cast_i16(x: f64) -> i16 {
    cast_i64(x) as i16
}
pub fn cast_u32(x: f64) -> u32 {
    cast_i64(x) as u32
}
pub fn cast_i32(x: f64) -> i32 {
    if x.is_nan() || x >= 2147483648.0 || x <= -2147483649.0 {
        i32::MIN
    } else {
        x as i32
    }
}
/// `(long)double`: truncation; out of range or NaN gives `long.MinValue` (cvttsd2si).
pub fn cast_i64(x: f64) -> i64 {
    if x.is_nan() || x >= 9.223372036854775807e18 || x < -9.223372036854775808e18 {
        i64::MIN
    } else {
        x as i64
    }
}

/// `(ulong)double`: the JIT helper subtracts 2^63 for values at or above it; below that it
/// is the signed conversion reinterpreted.
pub fn cast_u64(x: f64) -> u64 {
    const TWO63: f64 = 9.223372036854775808e18;
    if x.is_nan() {
        return 0;
    }
    if x >= TWO63 {
        (cast_i64(x - TWO63) as u64).wrapping_add(1u64 << 63)
    } else {
        cast_i64(x) as u64
    }
}

/// `float.Parse(s, provider)` on .NET Framework: parsed as a double, then an infinite
/// `(float)` result throws OverflowException; the infinity and NaN symbols parse directly.
pub fn float_parse_culture(s: &str, c: &CultureInfo) -> Result<f32, Exception> {
    match s.trim() {
        "Infinity" => return Ok(f32::INFINITY),
        "-Infinity" => return Ok(f32::NEG_INFINITY),
        "NaN" => return Ok(f32::NAN),
        _ => {}
    }
    let d = double_parse_culture(s, c).map_err(|e| match e {
        Exception::Overflow(_) => Exception::Overflow("Value was either too large or too small for a Single.".into()),
        e => e,
    })?;
    let f = d as f32;
    if f.is_infinite() {
        return Err(Exception::Overflow("Value was either too large or too small for a Single.".into()));
    }
    Ok(f)
}

/// `float.TryParse(s, style, provider, out f)`
pub fn float_try_parse_culture(s: &str, c: &CultureInfo) -> Option<f32> {
    float_parse_culture(s, c).ok()
}

/// The .NET exception types the ported code throws and catches. C# `throw new X(msg)` maps
/// to `Err(Exception::X(msg))`; a `catch (X)` is a match on the variant.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Exception {
    ArgumentOutOfRange(String),
    Argument(String),
    InvalidData(String),
    InvalidOperation(String),
    InvalidCast(String),
    EndOfStream(String),
    Timeout(String),
    Format(String),
    Io(String),
    NotSupported(String),
    Socket(String),
    ObjectDisposed(String),
    NullReference(String),
    KeyNotFound(String),
    NotImplemented(String),
    Overflow(String),
    FileNotFound(String),
    UnauthorizedAccess(String),
    ArgumentNull(String),
    UriFormat(String),
    WebSocket(String),
    IndexOutOfRange(String),
    Authentication(String),
    JsonReader(String),
    JsonSerialization(String),
    Cryptographic(String),
    Xml(String),
    Serialization(String),
    TypeInitialization(String),
    /// `GeoUtility.ErrorProvider.GeoException` (ExtLibs/GeoUtility)
    Geo(String),
    DivideByZero(String),
    DirectoryNotFound(String),
    Arithmetic(String),
    /// An exception class Mission Planner itself defines: (full type name, message).
    Custom(&'static str, String),
    Other(String),
}

impl Exception {
    fn parts(&self) -> (&'static str, &str) {
        match self {
            Exception::ArgumentOutOfRange(m) => ("System.ArgumentOutOfRangeException", m),
            Exception::Argument(m) => ("System.ArgumentException", m),
            Exception::InvalidData(m) => ("System.IO.InvalidDataException", m),
            Exception::InvalidOperation(m) => ("System.InvalidOperationException", m),
            Exception::InvalidCast(m) => ("System.InvalidCastException", m),
            Exception::EndOfStream(m) => ("System.IO.EndOfStreamException", m),
            Exception::Timeout(m) => ("System.TimeoutException", m),
            Exception::Format(m) => ("System.FormatException", m),
            Exception::Io(m) => ("System.IO.IOException", m),
            Exception::NotSupported(m) => ("System.NotSupportedException", m),
            Exception::Socket(m) => ("System.Net.Sockets.SocketException", m),
            Exception::ObjectDisposed(m) => ("System.ObjectDisposedException", m),
            Exception::NullReference(m) => ("System.NullReferenceException", m),
            Exception::KeyNotFound(m) => ("System.Collections.Generic.KeyNotFoundException", m),
            Exception::NotImplemented(m) => ("System.NotImplementedException", m),
            Exception::Overflow(m) => ("System.OverflowException", m),
            Exception::FileNotFound(m) => ("System.IO.FileNotFoundException", m),
            Exception::UnauthorizedAccess(m) => ("System.UnauthorizedAccessException", m),
            Exception::ArgumentNull(m) => ("System.ArgumentNullException", m),
            Exception::UriFormat(m) => ("System.UriFormatException", m),
            Exception::WebSocket(m) => ("System.Net.WebSockets.WebSocketException", m),
            Exception::IndexOutOfRange(m) => ("System.IndexOutOfRangeException", m),
            Exception::Authentication(m) => ("System.Security.Authentication.AuthenticationException", m),
            Exception::JsonReader(m) => ("Newtonsoft.Json.JsonReaderException", m),
            Exception::JsonSerialization(m) => ("Newtonsoft.Json.JsonSerializationException", m),
            Exception::Cryptographic(m) => ("System.Security.Cryptography.CryptographicException", m),
            Exception::Xml(m) => ("System.Xml.XmlException", m),
            Exception::Serialization(m) => ("System.Runtime.Serialization.SerializationException", m),
            Exception::TypeInitialization(m) => ("System.TypeInitializationException", m),
            Exception::Geo(m) => ("GeoUtility.ErrorProvider.GeoException", m),
            Exception::DivideByZero(m) => ("System.DivideByZeroException", m),
            Exception::DirectoryNotFound(m) => ("System.IO.DirectoryNotFoundException", m),
            Exception::Arithmetic(m) => ("System.ArithmeticException", m),
            Exception::Custom(t, m) => (*t, m),
            Exception::Other(m) => ("System.Exception", m),
        }
    }

    /// `ex.Message`
    pub fn message(&self) -> &str {
        self.parts().1
    }
}

impl std::fmt::Display for Exception {
    /// `ex.ToString()` without the stack trace.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (t, m) = self.parts();
        write!(f, "{t}: {m}")
    }
}

impl std::error::Error for Exception {}

impl From<std::io::Error> for Exception {
    fn from(e: std::io::Error) -> Self {
        match e.kind() {
            std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock => Exception::Timeout(e.to_string()),
            std::io::ErrorKind::UnexpectedEof => Exception::EndOfStream(e.to_string()),
            _ => Exception::Io(e.to_string()),
        }
    }
}

pub const TICKS_PER_MILLISECOND: i64 = 10_000;
pub const TICKS_PER_SECOND: i64 = 10_000_000;
/// Ticks between 0001-01-01 and 1970-01-01.
pub const UNIX_EPOCH_TICKS: i64 = 621_355_968_000_000_000;
const MAX_TICKS: i64 = 3_155_378_975_999_999_999;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DateTimeKind {
    #[default]
    Unspecified,
    Utc,
    Local,
}

/// `System.DateTime`: 100 ns ticks since 0001-01-01 plus a `Kind`. Comparisons use ticks only,
/// as .NET does.
#[derive(Debug, Clone, Copy, Default)]
pub struct DateTime {
    pub ticks: i64,
    pub kind: DateTimeKind,
}

impl PartialEq for DateTime {
    fn eq(&self, o: &Self) -> bool {
        self.ticks == o.ticks
    }
}
impl Eq for DateTime {}
impl PartialOrd for DateTime {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for DateTime {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.ticks.cmp(&o.ticks)
    }
}

/// `System.TimeSpan`
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct TimeSpan {
    pub ticks: i64,
}

impl TimeSpan {
    /// `TotalMilliseconds`: `ticks * MillisecondsPerTick`, clamped to the representable
    /// milliseconds as .NET Framework does.
    pub fn total_milliseconds(self) -> f64 {
        const MILLISECONDS_PER_TICK: f64 = 1.0 / TICKS_PER_MILLISECOND as f64;
        const MAX_MILLISECONDS: f64 = (i64::MAX / TICKS_PER_MILLISECOND) as f64;
        const MIN_MILLISECONDS: f64 = (i64::MIN / TICKS_PER_MILLISECOND) as f64;
        let temp = self.ticks as f64 * MILLISECONDS_PER_TICK;
        if temp > MAX_MILLISECONDS {
            return MAX_MILLISECONDS;
        }
        if temp < MIN_MILLISECONDS {
            return MIN_MILLISECONDS;
        }
        temp
    }

    /// `TotalSeconds`: `ticks * SecondsPerTick`.
    pub fn total_seconds(self) -> f64 {
        const SECONDS_PER_TICK: f64 = 1.0 / TICKS_PER_SECOND as f64;
        self.ticks as f64 * SECONDS_PER_TICK
    }

    /// `TotalDays`: `ticks * DaysPerTick`.
    pub fn total_days(self) -> f64 {
        const DAYS_PER_TICK: f64 = 1.0 / (TICKS_PER_SECOND as f64 * 86400.0);
        self.ticks as f64 * DAYS_PER_TICK
    }
}

impl DateTime {
    /// `DateTime.Add(double value, int scale)`: the value in milliseconds, rounded half away
    /// from zero, range checked, then added as ticks.
    fn add_scaled(self, value: f64, scale_ms: f64) -> Result<Self, Exception> {
        const MAX_MILLIS: i64 = 3_652_059 * 86_400_000;
        let millis = cast_i64(value * scale_ms + if value >= 0.0 { 0.5 } else { -0.5 });
        if millis <= -MAX_MILLIS || millis >= MAX_MILLIS {
            return Err(Exception::ArgumentOutOfRange("Value to add was out of range.\r\nParameter name: value".into()));
        }
        self.add_ticks(millis * TICKS_PER_MILLISECOND)
    }

    /// `AddTicks(value)`: throws when the result leaves 0001..9999.
    pub fn add_ticks(self, value: i64) -> Result<Self, Exception> {
        match self.ticks.checked_add(value) {
            Some(t) if (0..=MAX_TICKS).contains(&t) => Ok(DateTime { ticks: t, kind: self.kind }),
            _ => Err(Exception::ArgumentOutOfRange(
                "The added or subtracted value results in an un-representable DateTime.\r\nParameter name: value".into(),
            )),
        }
    }

    /// `AddDays(value)`
    pub fn add_days(self, value: f64) -> Result<Self, Exception> {
        self.add_scaled(value, 86_400_000.0)
    }

    /// `AddSeconds(value)`
    pub fn add_seconds(self, value: f64) -> Result<Self, Exception> {
        self.add_scaled(value, 1000.0)
    }
}

impl DateTime {
    pub const MIN_VALUE: DateTime = DateTime { ticks: 0, kind: DateTimeKind::Unspecified };
    pub const MAX_VALUE: DateTime = DateTime { ticks: MAX_TICKS, kind: DateTimeKind::Unspecified };

    /// `new DateTime(year, month, day, hour, minute, second, kind)`; out-of-range parts throw.
    pub fn new(year: i32, month: u32, day: u32, hour: u32, minute: u32, second: u32, kind: DateTimeKind) -> Result<Self, Exception> {
        let leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
        let dim = [31, if leap { 29 } else { 28 }, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
        if !(1..=9999).contains(&year) || !(1..=12).contains(&month) || day < 1 || day > dim[month as usize - 1] {
            return Err(Exception::ArgumentOutOfRange("Year, Month, and Day parameters describe an un-representable DateTime.".into()));
        }
        if hour >= 24 || minute >= 60 || second >= 60 {
            return Err(Exception::ArgumentOutOfRange("Hour, Minute, and Second parameters describe an un-representable DateTime.".into()));
        }
        let days = days_from_civil(year as i64, month as i64, day as i64) - days_from_civil(1, 1, 1);
        let secs = days * 86400 + hour as i64 * 3600 + minute as i64 * 60 + second as i64;
        Ok(DateTime { ticks: secs * TICKS_PER_SECOND, kind })
    }

    pub fn utc_now() -> Self {
        let d = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
        DateTime { ticks: UNIX_EPOCH_TICKS + (d.as_nanos() / 100) as i64, kind: DateTimeKind::Utc }
    }

    pub fn now() -> Self {
        DateTime::utc_now().to_local_time()
    }

    /// The local zone's UTC offset at the UTC instant `utc_ticks` (daylight saving applied for
    /// that date, as `TimeZoneInfo.Local` does).
    fn local_offset_at_utc(utc_ticks: i64) -> i64 {
        use chrono::{Local, Offset, TimeZone};
        let secs = (utc_ticks - UNIX_EPOCH_TICKS).div_euclid(TICKS_PER_SECOND);
        match chrono::DateTime::from_timestamp(secs, 0) {
            Some(dt) => Local.offset_from_utc_datetime(&dt.naive_utc()).fix().local_minus_utc() as i64 * TICKS_PER_SECOND,
            None => crate::local_utc_offset_secs() * TICKS_PER_SECOND,
        }
    }

    /// `ToLocalTime()`: a Local value is returned unchanged; Utc and Unspecified are treated as UTC.
    pub fn to_local_time(self) -> Self {
        if self.kind == DateTimeKind::Local {
            return self;
        }
        let t = self.ticks + Self::local_offset_at_utc(self.ticks);
        DateTime { ticks: t.clamp(0, MAX_TICKS), kind: DateTimeKind::Local }
    }

    /// `ToUniversalTime()`: a Utc value is returned unchanged; Local and Unspecified are
    /// treated as local time, converted with the offset in force at that time.
    pub fn to_universal_time(self) -> Self {
        if self.kind == DateTimeKind::Utc {
            return self;
        }
        let guess = self.ticks - Self::local_offset_at_utc(self.ticks);
        let t = self.ticks - Self::local_offset_at_utc(guess);
        DateTime { ticks: t.clamp(0, MAX_TICKS), kind: DateTimeKind::Utc }
    }

    /// `AddMilliseconds`: .NET Framework rounds the argument to a whole millisecond, and throws
    /// when the result leaves the representable range.
    pub fn add_milliseconds(self, value: f64) -> Result<Self, Exception> {
        let millis = (value + if value >= 0.0 { 0.5 } else { -0.5 }) as i64;
        let t = self.ticks.checked_add(millis.saturating_mul(TICKS_PER_MILLISECOND));
        match t {
            Some(t) if (0..=MAX_TICKS).contains(&t) => Ok(DateTime { ticks: t, kind: self.kind }),
            _ => Err(Exception::ArgumentOutOfRange(
                "The added or subtracted value results in an un-representable DateTime.\r\nParameter name: value".into(),
            )),
        }
    }

    /// (year, month, day, hour, minute, second, millisecond)
    pub fn parts(self) -> (i32, u32, u32, u32, u32, u32, u32) {
        let total_secs = self.ticks.div_euclid(TICKS_PER_SECOND);
        let ms = (self.ticks.rem_euclid(TICKS_PER_SECOND) / TICKS_PER_MILLISECOND) as u32;
        let days = total_secs.div_euclid(86400) + days_from_civil(1, 1, 1);
        let sod = total_secs.rem_euclid(86400);
        let (y, m, d) = civil_from_days(days);
        (y as i32, m as u32, d as u32, (sod / 3600) as u32, ((sod / 60) % 60) as u32, (sod % 60) as u32, ms)
    }

    /// `ToString(format, culture)` for a .NET custom date/time pattern (d, dd, M, MM, yy, yyyy,
    /// h, hh, H, HH, m, mm, s, ss, f.., t, tt, quoted literals and `\` escapes).
    pub fn format(self, pattern: &str, c: &CultureInfo) -> String {
        let (y, mo, d, h, mi, s, ms) = self.parts();
        let chars: Vec<char> = pattern.chars().collect();
        let mut out = String::new();
        let mut i = 0;
        while i < chars.len() {
            let ch = chars[i];
            let mut n = 1;
            while i + n < chars.len() && chars[i + n] == ch {
                n += 1;
            }
            match ch {
                'd' if n <= 2 => out.push_str(&format!("{:0w$}", d, w = n)),
                'M' if n <= 2 => out.push_str(&format!("{:0w$}", mo, w = n)),
                'y' => {
                    if n <= 2 {
                        out.push_str(&format!("{:0w$}", y % 100, w = n))
                    } else {
                        out.push_str(&format!("{:0w$}", y, w = n))
                    }
                }
                'h' => {
                    let h12 = if h % 12 == 0 { 12 } else { h % 12 };
                    out.push_str(&format!("{:0w$}", h12, w = n.min(2)))
                }
                'H' => out.push_str(&format!("{:0w$}", h, w = n.min(2))),
                'm' => out.push_str(&format!("{:0w$}", mi, w = n.min(2))),
                's' => out.push_str(&format!("{:0w$}", s, w = n.min(2))),
                'f' | 'F' => {
                    let frac = format!("{:07}", self.ticks.rem_euclid(TICKS_PER_SECOND));
                    let mut part = frac[..n.min(7)].to_string();
                    if ch == 'F' {
                        part = part.trim_end_matches('0').to_string();
                    }
                    let _ = ms;
                    out.push_str(&part);
                }
                't' => {
                    let des = if h < 12 { c.am_designator } else { c.pm_designator };
                    if n == 1 {
                        out.push_str(&des.chars().take(1).collect::<String>());
                    } else {
                        out.push_str(des);
                    }
                }
                '\'' | '"' => {
                    let mut j = i + 1;
                    while j < chars.len() && chars[j] != ch {
                        out.push(chars[j]);
                        j += 1;
                    }
                    i = j + 1;
                    continue;
                }
                '\\' => {
                    if i + 1 < chars.len() {
                        out.push(chars[i + 1]);
                    }
                    i += 2;
                    continue;
                }
                _ => {
                    for _ in 0..n {
                        out.push(ch);
                    }
                }
            }
            i += n;
        }
        out
    }

    /// `ToString()`: the "G" pattern (short date, space, long time) of the current culture.
    pub fn to_cs_string(self) -> String {
        let c = current_culture();
        self.to_cs_string_culture(c)
    }

    /// `ToString(provider)`
    pub fn to_cs_string_culture(self, c: &CultureInfo) -> String {
        format!("{} {}", self.format(c.short_date_pattern, c), self.format(c.long_time_pattern, c))
    }
}

impl std::ops::Sub for DateTime {
    type Output = TimeSpan;
    fn sub(self, o: DateTime) -> TimeSpan {
        TimeSpan { ticks: self.ticks - o.ticks }
    }
}

impl std::fmt::Display for DateTime {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.to_cs_string())
    }
}

/// `System.IO.Stream`, reduced to the members the ported code calls. `Position` and `Length`
/// throw on streams that do not support them, hence the `Result`.
pub trait Stream: Send {
    fn can_seek(&self) -> bool {
        false
    }
    fn position(&self) -> Result<i64, Exception> {
        Err(Exception::NotSupported("This stream does not support seek operations.".into()))
    }
    fn length(&self) -> Result<i64, Exception> {
        Err(Exception::NotSupported("This stream does not support seek operations.".into()))
    }
    fn can_timeout(&self) -> bool {
        false
    }
    fn read_timeout(&self) -> i32 {
        -1
    }
    /// `Read(buffer, 0, buffer.Length)`; 0 means no data (end of a seekable stream).
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception>;
    /// `Write(buffer, 0, buffer.Length)`
    fn write(&mut self, _buffer: &[u8]) -> Result<(), Exception> {
        Err(Exception::NotSupported("Stream does not support writing.".into()))
    }
    fn flush(&mut self) -> Result<(), Exception> {
        Ok(())
    }
}

/// `System.IO.SeekOrigin`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeekOrigin {
    Begin,
    Current,
    End,
}

/// `System.IO.MemoryStream` over a growable byte vector.
#[derive(Debug, Default, Clone)]
pub struct MemoryStream {
    pub data: Vec<u8>,
    pub pos: usize,
}

impl MemoryStream {
    pub fn new(data: Vec<u8>) -> Self {
        MemoryStream { data, pos: 0 }
    }

    pub fn len(&self) -> i64 {
        self.data.len() as i64
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn pos(&self) -> i64 {
        self.pos as i64
    }

    /// `Seek(offset, origin)`; a negative result throws IOException.
    pub fn seek(&mut self, offset: i64, origin: SeekOrigin) -> Result<i64, Exception> {
        let base = match origin {
            SeekOrigin::Begin => 0,
            SeekOrigin::Current => self.pos as i64,
            SeekOrigin::End => self.data.len() as i64,
        };
        let p = base + offset;
        if p < 0 {
            return Err(Exception::Io("An attempt was made to move the position before the beginning of the stream.".into()));
        }
        self.pos = p as usize;
        Ok(p)
    }

    /// `SetLength(value)`: truncates or zero-extends; the position is clamped to the new length.
    pub fn set_length(&mut self, value: i64) {
        self.data.resize(value.max(0) as usize, 0);
        if self.pos > self.data.len() {
            self.pos = self.data.len();
        }
    }

    /// `Read(buffer, offset, count)`
    pub fn read_into(&mut self, buffer: &mut [u8], offset: usize, count: usize) -> usize {
        let n = count.min(self.data.len().saturating_sub(self.pos));
        buffer[offset..offset + n].copy_from_slice(&self.data[self.pos..self.pos + n]);
        self.pos += n;
        n
    }

    /// `Write(buffer, offset, count)`: overwrites from the position, growing the stream.
    pub fn write_from(&mut self, buffer: &[u8], offset: usize, count: usize) {
        let end = self.pos + count;
        if end > self.data.len() {
            self.data.resize(end, 0);
        }
        self.data[self.pos..end].copy_from_slice(&buffer[offset..offset + count]);
        self.pos = end;
    }

    /// `ToArray()`
    pub fn to_array(&self) -> Vec<u8> {
        self.data.clone()
    }
}

impl Stream for MemoryStream {
    fn can_seek(&self) -> bool {
        true
    }
    fn position(&self) -> Result<i64, Exception> {
        Ok(self.pos as i64)
    }
    fn length(&self) -> Result<i64, Exception> {
        Ok(self.data.len() as i64)
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let n = buffer.len();
        Ok(self.read_into(buffer, 0, n))
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        self.write_from(buffer, 0, buffer.len());
        Ok(())
    }
}

/// `Stream.Null`: reads return 0 and writes are discarded.
#[derive(Debug, Default, Clone, Copy)]
pub struct NullStream;

impl Stream for NullStream {
    fn can_seek(&self) -> bool {
        true
    }
    fn position(&self) -> Result<i64, Exception> {
        Ok(0)
    }
    fn length(&self) -> Result<i64, Exception> {
        Ok(0)
    }
    fn read(&mut self, _buffer: &mut [u8]) -> Result<usize, Exception> {
        Ok(0)
    }
    fn write(&mut self, _buffer: &[u8]) -> Result<(), Exception> {
        Ok(())
    }
}

/// A `Stream` reference is itself a stream, as C# passes streams around by reference.
impl<S: Stream + ?Sized> Stream for &mut S {
    fn can_seek(&self) -> bool {
        (**self).can_seek()
    }
    fn position(&self) -> Result<i64, Exception> {
        (**self).position()
    }
    fn length(&self) -> Result<i64, Exception> {
        (**self).length()
    }
    fn can_timeout(&self) -> bool {
        (**self).can_timeout()
    }
    fn read_timeout(&self) -> i32 {
        (**self).read_timeout()
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        (**self).read(buffer)
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        (**self).write(buffer)
    }
    fn flush(&mut self) -> Result<(), Exception> {
        (**self).flush()
    }
}

fn object_disposed_file() -> Exception {
    Exception::ObjectDisposed("Cannot access a closed file.".into())
}

/// `System.IO.FileStream` opened by `File.OpenRead`.
pub struct FileStream {
    file: Option<std::io::BufReader<std::fs::File>>,
    pos: i64,
    len: i64,
}

impl FileStream {
    /// `File.OpenRead(path)`
    pub fn open_read(path: &str) -> Result<Self, Exception> {
        let f = std::fs::File::open(path).map_err(|e| match e.kind() {
            std::io::ErrorKind::NotFound => Exception::FileNotFound(format!("Could not find file '{path}'.")),
            _ => Exception::Io(e.to_string()),
        })?;
        let len = f.metadata()?.len() as i64;
        Ok(FileStream { file: Some(std::io::BufReader::with_capacity(4096, f)), pos: 0, len })
    }

    /// `CanRead`: false once disposed.
    pub fn can_read(&self) -> bool {
        self.file.is_some()
    }

    /// `ReadByte()`: -1 at the end of the file.
    pub fn read_byte(&mut self) -> Result<i32, Exception> {
        let mut b = [0u8; 1];
        Ok(if Stream::read(self, &mut b)? == 0 { -1 } else { b[0] as i32 })
    }

    /// `Read(buffer, offset, count)`
    pub fn read_at(&mut self, buffer: &mut [u8], offset: usize, count: usize) -> Result<usize, Exception> {
        use std::io::Read;
        let f = self.file.as_mut().ok_or_else(object_disposed_file)?;
        let mut total = 0;
        while total < count {
            let n = f.read(&mut buffer[offset + total..offset + count])?;
            if n == 0 {
                break;
            }
            total += n;
        }
        self.pos += total as i64;
        Ok(total)
    }

    /// `Dispose()`
    pub fn dispose(&mut self) {
        self.file = None;
    }
}

impl Stream for FileStream {
    fn can_seek(&self) -> bool {
        self.file.is_some()
    }
    fn position(&self) -> Result<i64, Exception> {
        if self.file.is_none() {
            return Err(object_disposed_file());
        }
        Ok(self.pos)
    }
    fn length(&self) -> Result<i64, Exception> {
        if self.file.is_none() {
            return Err(object_disposed_file());
        }
        Ok(self.len)
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let n = buffer.len();
        self.read_at(buffer, 0, n)
    }
}

/// `int.Parse(s)` under the current culture: optional surrounding white space and sign.
pub fn int_parse(s: &str) -> Result<i32, Exception> {
    let t = s.trim_matches(|c: char| c.is_whitespace());
    let digits = t.strip_prefix(['+', '-']).unwrap_or(t);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return Err(Exception::Format("Input string was not in a correct format.".into()));
    }
    t.parse::<i32>().map_err(|_| Exception::Overflow("Value was either too large or too small for an Int32.".into()))
}

/// `long.Parse(s)` (`NumberStyles.Integer`)
pub fn long_parse(s: &str) -> Result<i64, Exception> {
    let t = s.trim_matches(|c: char| c.is_whitespace());
    let digits = t.strip_prefix(['+', '-']).unwrap_or(t);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return Err(Exception::Format("Input string was not in a correct format.".into()));
    }
    t.parse::<i64>().map_err(|_| Exception::Overflow("Value was either too large or too small for an Int64.".into()))
}

/// `double.Parse(s)` under the current culture (`NumberStyles.Float | AllowThousands`).
pub fn double_parse(s: &str) -> Result<f64, Exception> {
    double_parse_culture(s, current_culture())
}

/// `double.Parse(s, provider)`
pub fn double_parse_culture(s: &str, c: &CultureInfo) -> Result<f64, Exception> {
    if let Some(v) = double_try_parse_culture(s, c) {
        return Ok(v);
    }
    // TryParse fails on overflow as well; Parse reports that as OverflowException.
    match normalize_number(s, c, true).and_then(|n| n.parse::<f64>().ok()) {
        Some(v) if v.is_infinite() => Err(Exception::Overflow("Value was either too large or too small for a Double.".into())),
        _ => Err(Exception::Format("Input string was not in a correct format.".into())),
    }
}

/// `string.ToUpper()` under the current culture: per character, Turkish and Azeri map `i` to
/// `İ`, and characters whose upper case is more than one character (such as `ß`) are kept,
/// as .NET does.
pub fn to_upper_culture(s: &str, c: &CultureInfo) -> String {
    let turkic = matches!(c.name.split('-').next(), Some("tr") | Some("az"));
    s.chars()
        .map(|ch| {
            if turkic && ch == 'i' {
                return '\u{130}';
            }
            let mut up = ch.to_uppercase();
            match (up.next(), up.next()) {
                (Some(u), None) => u,
                _ => ch,
            }
        })
        .collect()
}

/// `string.ToUpper()` in the current culture.
pub fn to_upper(s: &str) -> String {
    to_upper_culture(s, current_culture())
}

/// `array[i]` / `str.ToCharArray()[i]` past the end.
pub fn index_out_of_range() -> Exception {
    Exception::IndexOutOfRange("Index was outside the bounds of the array.".into())
}

/// `str.Substring(start, length)` with the .NET argument checks (indices in UTF-16 code units;
/// the ported callers only pass ASCII).
pub fn substring(s: &str, start: i32, length: i32) -> Result<String, Exception> {
    let units: Vec<u16> = s.encode_utf16().collect();
    if start < 0 {
        return Err(Exception::ArgumentOutOfRange("StartIndex cannot be less than zero.\r\nParameter name: startIndex".into()));
    }
    if start as usize > units.len() {
        return Err(Exception::ArgumentOutOfRange(
            "startIndex cannot be larger than length of string.\r\nParameter name: startIndex".into(),
        ));
    }
    if length < 0 {
        return Err(Exception::ArgumentOutOfRange("Length cannot be less than zero.\r\nParameter name: length".into()));
    }
    if start as usize + length as usize > units.len() {
        return Err(Exception::ArgumentOutOfRange(
            "Index and length must refer to a location within the string.\r\nParameter name: length".into(),
        ));
    }
    Ok(String::from_utf16_lossy(&units[start as usize..start as usize + length as usize]))
}

/// `str.PadLeft(totalWidth, ch)`
pub fn pad_left(s: &str, total_width: usize, ch: char) -> String {
    let n = s.encode_utf16().count();
    if n >= total_width {
        return s.to_string();
    }
    let mut out: String = std::iter::repeat(ch).take(total_width - n).collect();
    out.push_str(s);
    out
}

/// `str.PadRight(totalWidth, ch)`
pub fn pad_right(s: &str, total_width: usize, ch: char) -> String {
    let n = s.encode_utf16().count();
    let mut out = s.to_string();
    for _ in n..total_width {
        out.push(ch);
    }
    out
}

/// `Encoding.ASCII.GetString`: bytes above 0x7F decode to '?'.
pub fn ascii_get_string(bytes: &[u8]) -> String {
    bytes.iter().map(|&b| if b < 0x80 { b as char } else { '?' }).collect()
}

/// `Encoding.ASCII.GetBytes` / `new ASCIIEncoding().GetBytes`: characters above 0x7F encode as '?'.
pub fn ascii_get_bytes(s: &str) -> Vec<u8> {
    s.chars().map(|c| if (c as u32) < 0x80 { c as u8 } else { b'?' }).collect()
}

fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146097 + doe - 719468
}

fn civil_from_days(z: i64) -> (i64, i64, i64) {
    let z = z + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (if m <= 2 { y + 1 } else { y }, m, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn en() -> &'static CultureInfo {
        get_culture_info("en-US").unwrap()
    }

    #[test]
    fn general_format_matches_net_framework() {
        let f = |x| double_to_string_culture(x, en());
        assert_eq!(f(0.1), "0.1");
        assert_eq!(f(0.800000011920929), "0.800000011920929");
        assert_eq!(f(1e15), "1E+15");
        assert_eq!(f(123456789012345.0), "123456789012345");
        assert_eq!(f(0.0001), "0.0001");
        assert_eq!(f(0.00001), "1E-05");
        assert_eq!(f(-2.5), "-2.5");
        assert_eq!(f(f64::NAN), "NaN");
        assert_eq!(float_to_string_culture(0.8f32, en()), "0.8");
        assert_eq!(float_to_string_culture(1234567.0, en()), "1234567");
        assert_eq!(float_to_string_culture(12345678.0, en()), "1.234568E+07");
        let de = get_culture_info("de-DE").unwrap();
        assert_eq!(double_to_string_culture(-2.5, de), "-2,5");
        assert_eq!(math_round(2.5, 0), 2.0);
        assert_eq!(math_round(0.125, 2), 0.12);
    }

    #[test]
    fn parse_and_encoding() {
        assert_eq!(int_parse(" 5760 ").unwrap(), 5760);
        assert_eq!(int_parse("-3").unwrap(), -3);
        assert!(matches!(int_parse(""), Err(Exception::Format(_))));
        assert!(matches!(int_parse("12a"), Err(Exception::Format(_))));
        assert!(matches!(int_parse("99999999999"), Err(Exception::Overflow(_))));
        assert_eq!(ascii_get_string(&[0x41, 0xC3]), "A?");
        assert_eq!(ascii_get_bytes("aé"), vec![b'a', b'?']);
        let mut m = MemoryStream::default();
        m.write_from(b"abcdef", 0, 6);
        m.seek(2, SeekOrigin::Begin).unwrap();
        let mut b = [0u8; 2];
        assert_eq!(m.read_into(&mut b, 0, 2), 2);
        assert_eq!(&b, b"cd");
        m.set_length(3);
        assert_eq!(m.pos(), 3);
    }

    #[test]
    fn casts_follow_x64_jit() {
        assert_eq!(cast_u8(300.0), 44);
        assert_eq!(cast_i8(-1.0), -1);
        assert_eq!(cast_u32(-1.0), u32::MAX);
        assert_eq!(cast_i32(3e9), i32::MIN);
        assert_eq!(cast_i32(f64::NAN), i32::MIN);
    }

    #[test]
    fn datetime_ticks_and_formatting() {
        let e = DateTime::new(1970, 1, 1, 0, 0, 0, DateTimeKind::Utc).unwrap();
        assert_eq!(e.ticks, UNIX_EPOCH_TICKS);
        let t = DateTime::new(2024, 3, 5, 13, 4, 9, DateTimeKind::Utc).unwrap();
        assert_eq!(t.to_cs_string_culture(en()), "3/5/2024 1:04:09 PM");
        assert_eq!(t.to_cs_string_culture(get_culture_info("de-DE").unwrap()), "05.03.2024 13:04:09");
        assert_eq!(t.format("yyyy-MM-dd HH-mm-ss", &INVARIANT_CULTURE), "2024-03-05 13-04-09");
        assert_eq!(DateTime::MIN_VALUE.to_cs_string_culture(en()), "1/1/0001 12:00:00 AM");
        assert!(DateTime::new(2023, 2, 29, 0, 0, 0, DateTimeKind::Utc).is_err());
        assert!(DateTime::MIN_VALUE.add_milliseconds(-1.0).is_err());
        assert_eq!(e.add_milliseconds(1.4).unwrap().ticks - e.ticks, TICKS_PER_MILLISECOND);
    }
}

#[cfg(test)]
mod decimal_tests {
    use super::*;

    #[test]
    fn decimal_from_double_matches_net() {
        assert_eq!(decimal_from_double(0.1).unwrap().to_string(), "0.1");
        assert_eq!(decimal_from_double(45.0).unwrap().to_string(), "45");
        assert_eq!(decimal_from_double(1e-15).unwrap().to_string(), "0.000000000000001");
        assert_eq!(decimal_from_double(123.456789012345678).unwrap().to_string(), "123.456789012346");
        assert_eq!(decimal_from_double(-2.5).unwrap().to_string(), "-2.5");
        assert!(decimal_from_double(1e29).is_err());
    }

    #[test]
    fn decimal_parse_number_style() {
        assert_eq!(decimal_parse(" 50.00 ").unwrap().to_string(), "50.00");
        assert_eq!(decimal_parse("5-").unwrap().to_string(), "-5");
        assert!(matches!(decimal_parse("abc"), Err(Exception::Format(_))));
        assert!(matches!(decimal_parse("79228162514264337593543950336"), Err(Exception::Overflow(_))));
        assert_eq!(decimal_to_double(Decimal::new(5, 1)), 0.5);
    }
}

pub mod convert;
pub mod random;
