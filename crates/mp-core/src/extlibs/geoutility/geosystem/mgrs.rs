//! Port of `ExtLibs/GeoUtility/GeoSystem/MGRS.cs`
//!
//! The Gauss-Krueger and MapService conversions and the generic `ConvertFrom<T>` /
//! `ConvertTo<T>` are not ported: they need the unported Gauss-Krueger/MapService code and
//! nothing in Mission Planner calls them.

use std::ops::{Deref, DerefMut};

use super::base::basesystem::TRANSFORM;
use super::base::geocentric::GeocentricKind;
use super::base::{Geocentric, Target};
use super::utm::{convert_to_char, is_letter_at0};
use super::{Geographic, TryParseResult, UTM};
use crate::dotnet::collections::Dictionary;
use crate::dotnet::{
    cast_i32, double_parse, double_to_string, int_parse, pad_left, pad_right, substring, to_upper, Exception,
};
use crate::extlibs::geoutility::errorprovider::{geo_exception, text_or_empty, ErrorMessage};

/// `public class MGRS : Geocentric` (Military Grid Reference System, UTMREF)
#[derive(Debug, Clone)]
pub struct MGRS {
    base: Geocentric,
    grid: String,
}

impl Deref for MGRS {
    type Target = Geocentric;
    fn deref(&self) -> &Geocentric {
        &self.base
    }
}

impl DerefMut for MGRS {
    fn deref_mut(&mut self) -> &mut Geocentric {
        &mut self.base
    }
}

impl Default for MGRS {
    /// `new MGRS()`
    fn default() -> Self {
        MGRS { base: Geocentric::empty(GeocentricKind::Mgrs), grid: String::new() }
    }
}

/// `Math.Round(x).ToString()`: banker's rounding, then `double.ToString()`.
fn round_to_string(x: f64) -> String {
    double_to_string(x.round_ties_even())
}

impl MGRS {
    /// `new MGRS(string mgrs)`: from a combined coordinate such as "32UMA1234567890".
    pub fn parse(mgrs: &str) -> Result<MGRS, Exception> {
        let r = (|| -> Result<MGRS, Exception> {
            let parsed = MGRS::try_parse(mgrs);
            match parsed.value {
                Some(m) if parsed.ok => {
                    let mut this = MGRS::default();
                    this.set_zone(m.zone());
                    this.set_band(m.band())?;
                    this.set_grid(m.grid());
                    this.set_east(m.east());
                    this.set_north(m.north());
                    Ok(this)
                }
                _ => Err(Exception::Other(parsed.error_message)),
            }
        })();
        r.map_err(|_| geo_exception(ErrorMessage::new("ERROR_MGRS")))
    }

    /// `new MGRS(int zone, string band, string grid, double east, double north)`: East and
    /// North are rounded and padded to the same number of digits.
    pub fn new(zone: i32, band: &str, grid: &str, east: f64, north: f64) -> Result<MGRS, Exception> {
        let mut this = MGRS::default();
        this.set_zone(zone);
        this.set_band(band)?;
        this.set_grid(grid);

        let mut eastvalue = round_to_string(east);
        let mut northvalue = round_to_string(north);

        let (el, nl) = (eastvalue.encode_utf16().count(), northvalue.encode_utf16().count());
        if el < nl {
            eastvalue = pad_left(&eastvalue, nl, '0');
        } else if nl < el {
            northvalue = pad_left(&northvalue, el, '0');
        }

        this.set_east(double_parse(&eastvalue)?);
        this.set_north(double_parse(&northvalue)?);
        Ok(this)
    }

    /// `Grid`: the 100 km square letters.
    pub fn grid(&self) -> &str {
        &self.grid
    }

    pub fn set_grid(&mut self, value: &str) {
        self.grid = value.to_string();
    }

    /// `isEmpty`
    pub fn is_empty(&self) -> bool {
        (self.zone() == 0) && (self.band().is_empty()) && (self.grid.is_empty()) && (self.east() == 0.0) && (self.north() == 0.0)
    }

    /// `static bool TryParse(string mstring, out MGRS Mgrs, out string ErrorMessage)`. Any
    /// failure, including the parse exceptions, gives `false` and the ERROR_MGRS text.
    pub fn try_parse(mstring: &str) -> TryParseResult<MGRS> {
        let attempt = (|| -> Result<MGRS, Exception> {
            let mut mgrs = MGRS::default();
            let mut mstring = to_upper(&mstring.replace(' ', ""));

            // At least zone (1), band (1), grid (2), east (1), north (1) = 6
            if mstring.encode_utf16().count() < 6 {
                return Err(geo_exception(ErrorMessage::new("ERROR_MGRS")));
            }

            // one or two digit zone
            let zone;
            let c0 = convert_to_char(&substring(&mstring, 0, 1)?)?;
            if c0.is_ascii_digit() {
                let c1 = convert_to_char(&substring(&mstring, 1, 1)?)?;
                if c1.is_ascii_digit() {
                    zone = int_parse(&substring(&mstring, 0, 2)?)?;
                    mstring = mstring[2..].to_string();
                } else {
                    zone = int_parse(&substring(&mstring, 0, 1)?)?;
                    mstring = mstring[1..].to_string();
                }
            } else {
                return Err(geo_exception(ErrorMessage::new("ERROR_MGRS")));
            }
            mgrs.set_zone(zone);

            if mstring.encode_utf16().count() < 5 {
                return Err(geo_exception(ErrorMessage::new("ERROR_MGRS")));
            }

            let band = substring(&mstring, 0, 1)?.chars().next().ok_or_else(crate::dotnet::index_out_of_range)?;
            if (band < 'C') || (band > 'X') || (band == 'I') || (band == 'O') {
                return Err(geo_exception(ErrorMessage::new("ERROR_MGRS_GRID")));
            }
            mgrs.set_band(&band.to_string())?;
            mstring = mstring.chars().skip(1).collect();

            let grid = substring(&mstring, 0, 2)?;
            let mut gc = grid.chars();
            let (c0, c1) = (gc.next().unwrap_or('\0'), gc.next().unwrap_or('\0'));
            if (c0 < 'A') || (c0 >= 'Z') || (c1 < 'A') || (c1 >= 'Z') || (c0 == 'I') || (c0 == 'O') || (c1 == 'I') || (c1 == 'O') {
                return Err(geo_exception(ErrorMessage::new("ERROR_MGRS_GRID")));
            }
            mgrs.set_grid(&grid);
            mstring = mstring.chars().skip(2).collect();

            let n = mstring.encode_utf16().count();
            if n % 2 > 0 {
                return Err(geo_exception(ErrorMessage::new("ERROR_MGRS")));
            }
            let len = (n / 2) as i32;
            let mut eaststring = substring(&mstring, 0, len)?;
            let mut northstring = substring(&mstring, len, n as i32 - len)?;

            let el = eaststring.encode_utf16().count();
            if el > 5 {
                eaststring = substring(&eaststring, 0, 5)?;
            } else if el < 5 {
                eaststring = pad_right(&eaststring, 5, '0');
            }

            let nl = northstring.encode_utf16().count();
            if nl > 5 {
                northstring = substring(&northstring, 0, 5)?;
            } else if nl < 5 {
                northstring = pad_right(&northstring, 5, '0');
            }

            mgrs.set_east(double_parse(&eaststring)?);
            mgrs.set_north(double_parse(&northstring)?);
            Ok(mgrs)
        })();

        match attempt {
            Ok(m) => TryParseResult { ok: true, value: Some(m), error_message: String::new(), valid_items: Dictionary::new() },
            Err(_) => TryParseResult {
                ok: false,
                value: None,
                error_message: ErrorMessage::new("ERROR_MGRS").text().unwrap_or_default(),
                valid_items: Dictionary::new(),
            },
        }
    }

    /// `static bool TryParse(string Zone, string Band, string Grid, string East, string North,
    /// out MGRS Mgrs, out string ErrorMessage, out Dictionary<string, bool> ValidItems)`.
    /// The MGRS is built from `Grid` as passed, before the upper-casing that the check uses.
    /// An empty `Band` throws (`char.IsLetter(Band, 0)` is outside the C# try blocks).
    pub fn try_parse_parts(
        zone_s: &str,
        band_s: &str,
        grid_s: &str,
        east_s: &str,
        north_s: &str,
    ) -> Result<TryParseResult<MGRS>, Exception> {
        let mut result = true;
        let mut sb = String::new();
        let mut mgrs = None;
        let mut list: Dictionary<String, bool> = Dictionary::new();
        let mut zone = 0;
        let mut east = 0;
        let mut north = 0;
        let mut band = 'A';
        let grid = grid_s.to_string();
        let band_s = to_upper(band_s);
        let grid_upper = to_upper(grid_s);

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
            // "ERROR_UTM_Band" is not in the resources: the text is null
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_UTM_Band")) + "\r\n");
            result = false;
        }
        converted = true;

        let mut grid_ok = grid_upper.encode_utf16().count() == 2;
        if grid_ok {
            let mut gc = grid_upper.chars();
            let (c0, c1) = (gc.next().unwrap_or('\0'), gc.next().unwrap_or('\0'));
            if (c0 < 'A') || (c0 >= 'Z') || (c1 < 'A') || (c1 >= 'Z') || (c0 == 'I') || (c0 == 'O') || (c1 == 'I') || (c1 == 'O') {
                grid_ok = false;
            }
        }
        list.insert("Grid".into(), grid_ok);
        if !grid_ok {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_MGRS_GRID")) + "\r\n");
            result = false;
        }

        let mut east_s = east_s.to_string();
        if east_s.encode_utf16().count() > 5 {
            east_s = substring(&east_s, 0, 5)?;
        } else if east_s.encode_utf16().count() < 5 {
            east_s = pad_right(&east_s, 5, '0');
        }
        match int_parse(&east_s) {
            Ok(v) => east = v,
            Err(_) => converted = false,
        }
        if (east < 0) || (east >= 100000) {
            converted = false;
        }
        list.insert("East".into(), converted);
        if !converted {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_MGRS_EAST")) + "\r\n");
            result = false;
        }
        converted = true;

        let mut north_s = north_s.to_string();
        if north_s.encode_utf16().count() > 5 {
            north_s = substring(&north_s, 0, 5)?;
        } else if north_s.encode_utf16().count() < 5 {
            north_s = pad_right(&north_s, 5, '0');
        }
        match int_parse(&north_s) {
            Ok(v) => north = v,
            Err(_) => converted = false,
        }
        if (north < 0) || (north >= 100000) {
            converted = false;
        }
        list.insert("North".into(), converted);
        if !converted {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_MGRS_NORTH")) + "\r\n");
            result = false;
        }

        if result {
            mgrs = Some(MGRS::new(zone, &band.to_string(), &grid, east as f64, north as f64)?);
        }
        Ok(TryParseResult { ok: result, value: mgrs, error_message: sb, valid_items: list })
    }

    /// `ToLongString()`: e.g. "32U MA 12345 67890".
    pub fn to_long_string(&self) -> String {
        if self.is_empty() {
            return String::new();
        }
        self.zoneband() + " " + &self.grid + " " + self.east_string() + " " + self.north_string()
    }

    /// `explicit operator UTM(MGRS mgrs)`
    pub fn to_utm(&self) -> Result<UTM, Exception> {
        TRANSFORM.mgrutm(self)
    }

    /// `explicit operator Geographic(MGRS mgrs)`
    pub fn to_geographic(&self) -> Result<Geographic, Exception> {
        let utm = TRANSFORM.mgrutm(self)?;
        TRANSFORM.utmwgs(&utm)
    }

    /// `MemberwiseClone()`
    pub fn memberwise_clone(&self) -> Result<MGRS, Exception> {
        MGRS::new(self.zone(), self.band(), &self.grid, self.east(), self.north())
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        cast_i32(self.east()) ^ cast_i32(self.north())
    }
}

/// `MGRS.Format(Target)`: the target is ignored. East and North are rounded, padded to five
/// digits, and their common trailing zeros dropped, so both always have the same length.
pub(crate) fn format(g: &mut Geocentric, _target: Target) {
    let mut e = pad_left(&round_to_string(g.east()), 5, '0');
    let mut n = pad_left(&round_to_string(g.north()), 5, '0');
    while e.ends_with('0') && n.ends_with('0') {
        e.pop();
        n.pop();
    }
    g.set_east_string(&e);
    g.set_north_string(&n);
}

impl std::fmt::Display for MGRS {
    /// `ToString()`: e.g. "32UMA1234567890", "" when empty.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.is_empty() {
            return Ok(());
        }
        write!(f, "{}{}{}{}", self.zoneband(), self.grid, self.east_string(), self.north_string())
    }
}

impl PartialEq for MGRS {
    /// `Equals` / `operator ==`: the `ToString()` texts compared.
    fn eq(&self, other: &MGRS) -> bool {
        self.to_string() == other.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_and_format() {
        let m = MGRS::parse("32UMA1234567890").unwrap();
        assert_eq!((m.zone(), m.band(), m.grid()), (32, "U", "MA"));
        assert_eq!((m.east(), m.north()), (12345.0, 67890.0));
        assert_eq!(m.to_string(), "32UMA1234567890");
        // common trailing zeros are dropped
        let m = MGRS::new(32, "U", "MA", 12300.0, 45600.0).unwrap();
        assert_eq!(m.to_string(), "32UMA123456");
        assert!(!MGRS::try_parse("32U").ok);
        assert!(MGRS::parse("XX").is_err());
    }
}
