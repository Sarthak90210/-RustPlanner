//! .NET Framework 4.7.2 base-class-library behaviour that the ported C# relies on.
//!
//! This module is not a port of a Mission Planner file. It stands in for the BCL calls the C#
//! makes: number and date formatting under the current culture, `Math.Round`, numeric casts,
//! exceptions, `DateTime`/`TimeSpan` and `Stream`. Behaviour follows the .NET Framework runtime
//! Mission Planner ships on (`net472`), not .NET Core. Members are added only when a ported
//! file needs them.

use std::sync::OnceLock;

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
        let v = ["LC_ALL", "LC_NUMERIC", "LANG"].iter().find_map(|k| std::env::var(k).ok().filter(|s| !s.is_empty()))?;
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

/// Look up a culture by name (`CultureInfo.GetCultureInfo`).
pub fn get_culture_info(name: &str) -> Option<&'static CultureInfo> {
    if name.is_empty() {
        return Some(&INVARIANT_CULTURE);
    }
    CULTURES.iter().find(|c| c.name.eq_ignore_ascii_case(name))
}

/// Format with `digits` significant digits using the .NET "G" rules: fixed notation when the
/// decimal exponent is in `-5..digits`, otherwise `d.dddE+XX` with at least two exponent digits.
fn format_general(x: f64, digits: usize, c: &CultureInfo) -> String {
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
fn cast_i64(x: f64) -> i64 {
    if x.is_nan() || x >= 9.223372036854775807e18 || x < -9.223372036854775808e18 {
        i64::MIN
    } else {
        x as i64
    }
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
    /// `TotalMilliseconds`
    pub fn total_milliseconds(self) -> f64 {
        self.ticks as f64 / TICKS_PER_MILLISECOND as f64
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

/// `System.IO.Stream`, reduced to the members the ported code calls.
pub trait Stream: Send {
    fn can_seek(&self) -> bool {
        false
    }
    fn position(&self) -> i64 {
        0
    }
    fn length(&self) -> i64 {
        0
    }
    fn can_timeout(&self) -> bool {
        false
    }
    fn read_timeout(&self) -> i32 {
        -1
    }
    /// `Read(buffer, 0, buffer.Length)`; 0 means no data (end of a seekable stream).
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception>;
}

/// `System.IO.MemoryStream` over a byte vector.
#[derive(Debug, Default, Clone)]
pub struct MemoryStream {
    pub data: Vec<u8>,
    pub pos: usize,
}

impl MemoryStream {
    pub fn new(data: Vec<u8>) -> Self {
        MemoryStream { data, pos: 0 }
    }
}

impl Stream for MemoryStream {
    fn can_seek(&self) -> bool {
        true
    }
    fn position(&self) -> i64 {
        self.pos as i64
    }
    fn length(&self) -> i64 {
        self.data.len() as i64
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let n = buffer.len().min(self.data.len().saturating_sub(self.pos));
        buffer[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
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
