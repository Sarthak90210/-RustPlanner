//! Port of `ExtLibs/GeoUtility/GeoSystem/UTM.cs`
//!
//! The Gauss-Krueger and MapService conversions and the generic `ConvertFrom<T>` /
//! `ConvertTo<T>` are not ported: they need the unported Gauss-Krueger/MapService code and
//! nothing in Mission Planner calls them.

use std::ops::{Deref, DerefMut};

use super::base::basesystem::TRANSFORM;
use super::base::geocentric::GeocentricKind;
use super::base::{Geocentric, Hemisphere, Target};
use super::{Geographic, TryParseResult, MGRS};
use crate::dotnet::collections::Dictionary;
use crate::dotnet::{cast_i32, current_culture, double_to_string, format_fixed, int_parse, to_upper, Exception};
use crate::extlibs::geoutility::errorprovider::{text_or_empty, ErrorMessage};

/// `public class UTM : Geocentric`
#[derive(Debug, Clone)]
pub struct UTM {
    base: Geocentric,
}

impl Deref for UTM {
    type Target = Geocentric;
    fn deref(&self) -> &Geocentric {
        &self.base
    }
}

impl DerefMut for UTM {
    fn deref_mut(&mut self) -> &mut Geocentric {
        &mut self.base
    }
}

impl Default for UTM {
    /// `new UTM()`
    fn default() -> Self {
        UTM { base: Geocentric::empty(GeocentricKind::Utm) }
    }
}

/// `char.IsLetter(s, 0)`: throws when the string is empty.
pub(crate) fn is_letter_at0(s: &str) -> Result<bool, Exception> {
    match s.chars().next() {
        Some(c) => Ok(c.is_alphabetic()),
        None => Err(Exception::ArgumentOutOfRange(
            "Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into(),
        )),
    }
}

/// `Convert.ToChar(string)`: the string must be exactly one character long.
pub(crate) fn convert_to_char(s: &str) -> Result<char, Exception> {
    let mut it = s.chars();
    match (it.next(), it.next()) {
        (Some(c), None) if (c as u32) <= 0xFFFF => Ok(c),
        _ => Err(Exception::Format("String must be exactly one character long.".into())),
    }
}

impl UTM {
    /// `new UTM()`
    pub fn new() -> Self {
        Self::default()
    }

    /// `new UTM(int zone, string band, double east, double north)`
    pub fn with_band(zone: i32, band: &str, east: f64, north: f64) -> Result<Self, Exception> {
        Ok(UTM { base: Geocentric::with_band(GeocentricKind::Utm, zone, band, east, north)? })
    }

    /// `new UTM(int zone, double east, double north, Hemisphere hem)`: the band is computed.
    pub fn with_hemisphere(zone: i32, east: f64, north: f64, hem: Hemisphere) -> Result<Self, Exception> {
        Ok(UTM { base: Geocentric::with_hemisphere(GeocentricKind::Utm, zone, east, north, hem)? })
    }

    /// `isEmpty`: no coordinate values set yet.
    pub fn is_empty(&self) -> bool {
        (self.zone() == 0) && (self.band().is_empty()) && (self.east() == 0.0) && (self.north() == 0.0)
    }

    /// `static bool TryParse(string Zone, string Band, string East, string North, out UTM Utm,
    /// out string ErrorMessage, out Dictionary<string, bool> ValidItems)`. An empty `Band`
    /// throws (`char.IsLetter(Band, 0)` is outside the C# try blocks).
    pub fn try_parse(zone_s: &str, band_s: &str, east_s: &str, north_s: &str) -> Result<TryParseResult<UTM>, Exception> {
        let mut result = true;
        let mut sb = String::new();
        let mut utm = None;
        let mut list: Dictionary<String, bool> = Dictionary::new();
        let mut zone = 0;
        let mut east = 0;
        let mut north = 0;
        let mut band = 'A';
        let band_s = to_upper(band_s);

        let mut converted = match int_parse(zone_s) {
            Ok(v) => {
                zone = v;
                true
            }
            Err(_) => false,
        };
        if (zone < 1) || (zone > 60) {
            converted = false;
        }
        list.insert("Zone".into(), converted);
        if !converted {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_UTM_ZONE2")) + "\r\n");
            result = false;
        }

        converted = is_letter_at0(&band_s)?;
        match convert_to_char(&band_s) {
            Ok(c) => band = c,
            Err(_) => converted = false,
        }
        if (band < 'C') || (band > 'X') || (band == 'I') || (band == 'O') {
            converted = false;
        }
        list.insert("Band".into(), converted);
        if !converted {
            // "ERROR_UTM_Band" is not in the resources (they have ERROR_UTM_BAND): the text is null
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_UTM_Band")) + "\r\n");
            result = false;
        }

        converted = match int_parse(east_s) {
            Ok(v) => {
                east = v;
                true
            }
            Err(_) => false,
        };
        if (east < 160000) || (east > 840000) {
            converted = false;
        }
        list.insert("East".into(), converted);
        if !converted {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_UTM_EAST")) + "\r\n");
            result = false;
        }

        converted = match int_parse(north_s) {
            Ok(v) => {
                north = v;
                true
            }
            Err(_) => false,
        };
        if (band <= 'M' && ((north < 1116915) || (north > 10000000))) || (band >= 'N' && ((north < 0) || (north > 9329005))) {
            converted = false;
        }
        list.insert("North".into(), converted);
        if !converted {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_UTM_NORTH")) + "\r\n");
            result = false;
        }

        if result {
            utm = Some(UTM::with_band(zone, &band.to_string(), east as f64, north as f64)?);
        }
        Ok(TryParseResult { ok: result, value: utm, error_message: sb, valid_items: list })
    }

    /// `static bool TryParse(string UTMString, out UTM Utm, out string ErrorMessage, out
    /// Dictionary<string, bool> ValidItems)`: "32U 312345 4123456" or "32 U 312345 4123456".
    pub fn try_parse_string(utm_string: &str) -> Result<TryParseResult<UTM>, Exception> {
        let mut list: Dictionary<String, bool> = Dictionary::new();
        let mut utmstring = utm_string.trim().to_string();
        let mut field: Vec<String> = utmstring.split(' ').map(str::to_string).collect();

        if (field.len() == 3) || (field.len() == 4) {
            if field.len() == 3 {
                let n = field[0].encode_utf16().count();
                if (n == 2) || (n == 3) {
                    let mut units: Vec<u16> = utmstring.encode_utf16().collect();
                    units.insert(n - 1, b' ' as u16);
                    utmstring = String::from_utf16_lossy(&units);
                    field = utmstring.split(' ').map(str::to_string).collect();
                }
            }
            if field.len() == 4 {
                return UTM::try_parse(&field[0], &field[1], &field[2], &field[3]);
            }
        }

        list.insert("UTMString".into(), false);
        Ok(TryParseResult {
            ok: false,
            value: None,
            error_message: ErrorMessage::new("ERROR_UTM_STRING").text().unwrap_or_default(),
            valid_items: list,
        })
    }

    /// `ToShortString()`: e.g. "32U 412345 5567890" with the formatted values.
    pub fn to_short_string(&self) -> String {
        if self.is_empty() {
            return String::new();
        }
        self.zoneband() + " " + self.east_string() + " " + self.north_string()
    }

    /// `ToLongString()`: e.g. "32U E 412345 N 5567890".
    pub fn to_long_string(&self) -> String {
        if self.is_empty() {
            return String::new();
        }
        self.zoneband() + " E " + &double_to_string(self.east()) + " N " + &double_to_string(self.north())
    }

    /// `ToLongFormatString()`: the long string with the formatted values.
    pub fn to_long_format_string(&self) -> String {
        if self.is_empty() {
            return String::new();
        }
        self.zoneband() + " E " + self.east_string() + " N " + self.north_string()
    }

    /// `explicit operator MGRS(UTM utm)`
    pub fn to_mgrs(&self) -> Result<MGRS, Exception> {
        TRANSFORM.utmmgr(self)
    }

    /// `explicit operator Geographic(UTM utm)`
    pub fn to_geographic(&self) -> Result<Geographic, Exception> {
        TRANSFORM.utmwgs(self)
    }

    /// `MemberwiseClone()`: a new UTM from zone, band, east and north (the precision resets).
    pub fn memberwise_clone(&self) -> Result<UTM, Exception> {
        UTM::with_band(self.zone(), self.band(), self.east(), self.north())
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        cast_i32(self.east() / 100.0) ^ cast_i32(self.north() / 100.0)
    }
}

/// `UTM.Format(Target)`: East with 6 and North with 7 integer digits, `Precision` decimals,
/// in the current culture.
pub(crate) fn format(g: &mut Geocentric, target: Target) {
    let decimals = if g.precision() > 0 { g.precision() as u32 } else { 0 };
    if (target == Target::East) || (target == Target::All) {
        let s = format_fixed(g.east(), 6, decimals, current_culture());
        g.set_east_string(&s);
    }
    if (target == Target::North) || (target == Target::All) {
        let s = format_fixed(g.north(), 7, decimals, current_culture());
        g.set_north_string(&s);
    }
}

impl std::fmt::Display for UTM {
    /// `ToString()`: e.g. "32U 412345 5567890", "" when empty.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.is_empty() {
            return Ok(());
        }
        write!(f, "{} {} {}", self.zoneband(), double_to_string(self.east()), double_to_string(self.north()))
    }
}

impl PartialEq for UTM {
    /// `Equals` / `operator ==`: the `ToString()` texts compared.
    fn eq(&self, other: &UTM) -> bool {
        self.to_string() == other.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn band_is_clamped_and_strings_formatted() {
        let utm = UTM::with_band(32, "a", 412345.0, 5567890.0).unwrap();
        assert_eq!(utm.band(), "C");
        assert_eq!(utm.east_string(), "412345");
        assert_eq!(utm.north_string(), "5567890");
        assert!(UTM::with_band(32, "", 1.0, 1.0).is_err());
        assert_eq!(UTM::new().to_string(), "");
    }

    #[test]
    fn try_parse_string_splits_zoneband() {
        let r = UTM::try_parse_string("32U 412345 5567890").unwrap();
        assert!(r.ok);
        let utm = r.value.unwrap();
        assert_eq!((utm.zone(), utm.band()), (32, "U"));
        let r = UTM::try_parse_string("32U 412345").unwrap();
        assert!(!r.ok);
        assert_eq!(r.error_message, "UTM string could not be parsed (format unknown).");
    }
}
