//! .NET Framework 4.7.2 base-class-library behaviour that the ported C# relies on.
//!
//! This module is not a port of a Mission Planner file. It stands in for the BCL calls that
//! appear throughout the C# code: number formatting (`double.ToString()`, `float.ToString()`,
//! `ToString("0.00")`), `Math.Round` (banker's rounding), and `BitConverter`. Behaviour follows
//! the .NET Framework runtime Mission Planner ships on (`net472`), not .NET Core.

/// Format with `digits` significant digits using the .NET "G" rules: fixed notation when the
/// decimal exponent is in `-5..digits`, otherwise `d.dddE+XX` with at least two exponent digits.
fn format_general(x: f64, digits: usize) -> String {
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
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    if exp > -5 && exp < digits as i32 {
        if exp < 0 {
            out.push_str("0.");
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
                out.push('.');
                out.push_str(&ds[int_len..]);
            }
        }
    } else {
        out.push_str(&ds[..1]);
        if ds.len() > 1 {
            out.push('.');
            out.push_str(&ds[1..]);
        }
        out.push('E');
        out.push(if exp < 0 { '-' } else { '+' });
        out.push_str(&format!("{:02}", exp.abs()));
    }
    out
}

/// `double.ToString()` on .NET Framework: "G15".
pub fn double_to_string(x: f64) -> String {
    format_general(x, 15)
}

/// `float.ToString()` on .NET Framework: "G7".
pub fn float_to_string(x: f32) -> String {
    format_general(x as f64, 7)
}

/// `Math.Round(x)` / `Math.Round(x, digits)`: MidpointRounding.ToEven.
pub fn math_round(x: f64, digits: i32) -> f64 {
    if digits == 0 {
        return x.round_ties_even();
    }
    let m = 10f64.powi(digits);
    (x * m).round_ties_even() / m
}

/// `Math.Round(x, digits, MidpointRounding.AwayFromZero)`.
pub fn math_round_away(x: f64, digits: i32) -> f64 {
    let m = 10f64.powi(digits);
    (x * m).round() / m
}

/// C# `(int)double` / `(byte)double` style truncating casts follow Rust `as` for in-range
/// values. .NET leaves out-of-range float-to-int conversions unspecified (x86 yields the
/// "integer indefinite" value); unchecked narrowing between integer types wraps, which these
/// helpers reproduce by going through i64 first.
pub fn cast_u8(x: f64) -> u8 {
    (x as i64) as u8
}
pub fn cast_i8(x: f64) -> i8 {
    (x as i64) as i8
}
pub fn cast_u16(x: f64) -> u16 {
    (x as i64) as u16
}
pub fn cast_i16(x: f64) -> i16 {
    (x as i64) as i16
}
pub fn cast_u32(x: f64) -> u32 {
    (x as i64) as u32
}
pub fn cast_i32(x: f64) -> i32 {
    if x.is_nan() || x >= 2147483648.0 || x < -2147483648.0 {
        i32::MIN
    } else {
        x as i32
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
    pub fn from_milliseconds(ms: f64) -> Self {
        TimeSpan { ticks: (ms * TICKS_PER_MILLISECOND as f64) as i64 }
    }
    pub fn from_seconds(s: f64) -> Self {
        TimeSpan { ticks: (s * TICKS_PER_SECOND as f64) as i64 }
    }
    pub fn total_milliseconds(self) -> f64 {
        self.ticks as f64 / TICKS_PER_MILLISECOND as f64
    }
    pub fn total_seconds(self) -> f64 {
        self.ticks as f64 / TICKS_PER_SECOND as f64
    }
}

impl DateTime {
    pub const MIN_VALUE: DateTime = DateTime { ticks: 0, kind: DateTimeKind::Unspecified };
    pub const MAX_VALUE: DateTime = DateTime { ticks: 3_155_378_975_999_999_999, kind: DateTimeKind::Unspecified };

    /// `new DateTime(year, month, day, hour, minute, second, kind)`
    pub fn new(year: i32, month: u32, day: u32, hour: u32, minute: u32, second: u32, kind: DateTimeKind) -> Self {
        let days = days_from_civil(year as i64, month as i64, day as i64) - days_from_civil(1, 1, 1);
        let secs = days * 86400 + hour as i64 * 3600 + minute as i64 * 60 + second as i64;
        DateTime { ticks: secs * TICKS_PER_SECOND, kind }
    }

    pub fn utc_now() -> Self {
        let d = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
        DateTime { ticks: UNIX_EPOCH_TICKS + (d.as_nanos() / 100) as i64, kind: DateTimeKind::Utc }
    }

    pub fn now() -> Self {
        DateTime::utc_now().to_local_time()
    }

    fn local_offset_ticks() -> i64 {
        crate::local_utc_offset_secs() * TICKS_PER_SECOND
    }

    /// `ToLocalTime()`: a Local value is returned unchanged; Utc and Unspecified are treated as UTC.
    pub fn to_local_time(self) -> Self {
        if self.kind == DateTimeKind::Local {
            return self;
        }
        DateTime { ticks: (self.ticks + Self::local_offset_ticks()).clamp(0, Self::MAX_VALUE.ticks), kind: DateTimeKind::Local }
    }

    /// `ToUniversalTime()`: a Utc value is returned unchanged; Local and Unspecified are treated as local.
    pub fn to_universal_time(self) -> Self {
        if self.kind == DateTimeKind::Utc {
            return self;
        }
        DateTime { ticks: (self.ticks - Self::local_offset_ticks()).clamp(0, Self::MAX_VALUE.ticks), kind: DateTimeKind::Utc }
    }

    /// `AddMilliseconds`: .NET Framework rounds the argument to a whole millisecond.
    pub fn add_milliseconds(self, ms: f64) -> Self {
        let ms = (ms + if ms >= 0.0 { 0.5 } else { -0.5 }) as i64;
        DateTime { ticks: self.ticks + ms * TICKS_PER_MILLISECOND, kind: self.kind }
    }

    pub fn add_seconds(self, s: f64) -> Self {
        self.add_milliseconds(s * 1000.0)
    }

    pub fn add(self, t: TimeSpan) -> Self {
        DateTime { ticks: self.ticks + t.ticks, kind: self.kind }
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

    /// `ToString()` in en-US: `M/d/yyyy h:mm:ss tt`.
    pub fn to_cs_string(self) -> String {
        let (y, mo, d, h, mi, s, _) = self.parts();
        let (h12, ampm) = match h {
            0 => (12, "AM"),
            1..=11 => (h, "AM"),
            12 => (12, "PM"),
            _ => (h - 12, "PM"),
        };
        format!("{mo}/{d}/{y:04} {h12}:{mi:02}:{s:02} {ampm}")
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
    fn write(&mut self, _buffer: &[u8]) -> Result<(), Exception> {
        Err(Exception::NotSupported("Stream does not support writing.".into()))
    }
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
    pub fn to_array(&self) -> Vec<u8> {
        self.data.clone()
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
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let end = self.pos + buffer.len();
        if end > self.data.len() {
            self.data.resize(end, 0);
        }
        self.data[self.pos..end].copy_from_slice(buffer);
        self.pos = end;
        Ok(())
    }
}

/// `System.IO.FileStream` / `BufferedStream` over a file.
pub struct FileStream {
    file: std::io::BufReader<std::fs::File>,
    pos: i64,
    len: i64,
}

impl FileStream {
    pub fn open(path: &std::path::Path) -> Result<Self, Exception> {
        let f = std::fs::File::open(path)?;
        let len = f.metadata()?.len() as i64;
        Ok(FileStream { file: std::io::BufReader::with_capacity(1 << 16, f), pos: 0, len })
    }
    pub fn seek(&mut self, pos: i64) -> Result<(), Exception> {
        use std::io::Seek;
        self.file.seek(std::io::SeekFrom::Start(pos as u64))?;
        self.pos = pos;
        Ok(())
    }
}

impl Stream for FileStream {
    fn can_seek(&self) -> bool {
        true
    }
    fn position(&self) -> i64 {
        self.pos
    }
    fn length(&self) -> i64 {
        self.len
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        use std::io::Read;
        let n = self.file.read(buffer)?;
        self.pos += n as i64;
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

    #[test]
    fn datetime_ticks() {
        let e = DateTime::new(1970, 1, 1, 0, 0, 0, DateTimeKind::Utc);
        assert_eq!(e.ticks, UNIX_EPOCH_TICKS);
        assert_eq!(DateTime::new(2024, 3, 5, 13, 4, 9, DateTimeKind::Utc).to_cs_string(), "3/5/2024 1:04:09 PM");
        assert_eq!(DateTime::MIN_VALUE.to_cs_string(), "1/1/0001 12:00:00 AM");
    }

    #[test]
    fn general_format_matches_net_framework() {
        assert_eq!(double_to_string(0.1), "0.1");
        assert_eq!(double_to_string(0.800000011920929), "0.800000011920929");
        assert_eq!(double_to_string(1e15), "1E+15");
        assert_eq!(double_to_string(123456789012345.0), "123456789012345");
        assert_eq!(double_to_string(0.0001), "0.0001");
        assert_eq!(double_to_string(0.00001), "1E-05");
        assert_eq!(double_to_string(-2.5), "-2.5");
        assert_eq!(double_to_string(f64::NAN), "NaN");
        assert_eq!(float_to_string(0.8f32), "0.8");
        assert_eq!(float_to_string(1234567.0), "1234567");
        assert_eq!(float_to_string(12345678.0), "1.234568E+07");
        assert_eq!(math_round(2.5, 0), 2.0);
        assert_eq!(math_round_away(2.5, 0), 3.0);
    }
}
