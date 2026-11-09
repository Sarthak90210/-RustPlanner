//! Port of `ExtLibs/GeoUtility/GeoSystem/Geographic.cs`
//!
//! The Gauss-Krueger and MapService conversions and the generic `ConvertFrom<T>` /
//! `ConvertTo<T>` are not ported: they need the unported Gauss-Krueger/MapService code and
//! nothing in Mission Planner calls them.

use super::base::basesystem::TRANSFORM;
use super::helper::{GeoDatum, Sexagesimal};
use super::{TryParseResult, MGRS, UTM};
use crate::dotnet::collections::Dictionary;
use crate::dotnet::{cast_i32, current_culture, double_parse, format_fixed_optional, Exception};
use crate::extlibs::geoutility::errorprovider::{text_or_empty, ErrorMessage};
use crate::extlibs::geoutility::localizer::Message;

/// `string.Format(format, a, b)` for the two-argument resource strings; a null format throws.
fn string_format2(format: Option<String>, a: &str, b: &str) -> Result<String, Exception> {
    let f = format.ok_or_else(|| Exception::ArgumentNull("Value cannot be null.\r\nParameter name: format".into()))?;
    Ok(f.replace("{0}", a).replace("{1}", b))
}

/// `public class Geographic : BaseSystem`: longitude/latitude in decimal degrees.
#[derive(Debug, Clone, Copy, Default)]
pub struct Geographic {
    longtude: f64,
    latitude: f64,
    datum: GeoDatum,
}

impl Geographic {
    /// `new Geographic(double lon, double lat)`: WGS84 datum.
    pub fn new(lon: f64, lat: f64) -> Self {
        Geographic { longtude: lon, latitude: lat, datum: GeoDatum::WGS84 }
    }

    /// `new Geographic(double lon, double lat, GeoDatum datum)`
    pub fn with_datum(lon: f64, lat: f64, datum: GeoDatum) -> Self {
        Geographic { longtude: lon, latitude: lat, datum }
    }

    /// `Longitude`
    pub fn longitude(&self) -> f64 {
        self.longtude
    }

    pub fn set_longitude(&mut self, value: f64) {
        self.longtude = value;
    }

    /// `Latitude`
    pub fn latitude(&self) -> f64 {
        self.latitude
    }

    pub fn set_latitude(&mut self, value: f64) {
        self.latitude = value;
    }

    /// `LonString`
    pub fn lon_string(&self) -> String {
        Self::format(self.longitude())
    }

    /// `LatString`
    pub fn lat_string(&self) -> String {
        Self::format(self.latitude())
    }

    /// `LonSexagesimal`
    pub fn lon_sexagesimal(&self) -> Result<Sexagesimal, Exception> {
        Sexagesimal::from_decimal(self.longitude())
    }

    /// `LatSexagesimal`
    pub fn lat_sexagesimal(&self) -> Result<Sexagesimal, Exception> {
        Sexagesimal::from_decimal(self.latitude())
    }

    /// `Datum`
    pub fn datum(&self) -> GeoDatum {
        self.datum
    }

    /// `Datum` (internal setter)
    #[allow(dead_code)]
    pub(crate) fn set_datum_field(&mut self, value: GeoDatum) {
        self.datum = value;
    }

    /// `isEmpty`
    pub fn is_empty(&self) -> bool {
        (self.longtude == 0.0) && (self.latitude == 0.0)
    }

    /// `static bool TryParse(string Lon, string Lat, out Geographic Geograph, out string
    /// ErrorMessage, out Dictionary<string, bool> ValidItems)`. The C# replaces '.' with ','
    /// before `double.Parse` in the current culture, so under en-US "8.5" reads as 85.
    pub fn try_parse(lon_s: &str, lat_s: &str) -> TryParseResult<Geographic> {
        let mut result = true;
        let mut sb = String::new();
        let mut geo = None;
        let mut list: Dictionary<String, bool> = Dictionary::new();
        let mut lon = 0.0;
        let mut lat = 0.0;
        let lon_s = lon_s.replace('.', ",");
        let lat_s = lat_s.replace('.', ",");

        let mut converted = match double_parse(&lon_s) {
            Ok(v) => {
                lon = v;
                true
            }
            Err(_) => false,
        };
        if (lon < -180.0) || (lon > 180.0) {
            converted = false;
        }
        list.insert("Longitude".into(), converted);
        if !converted {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_GEO_LONGITUDE")) + "\r\n");
            result = false;
        }

        converted = match double_parse(&lat_s) {
            Ok(v) => {
                lat = v;
                true
            }
            Err(_) => false,
        };
        if (lat < -80.0) || (lat > 84.0) {
            converted = false;
        }
        list.insert("Latitude".into(), converted);
        if !converted || (lat < -80.0) || (lat > 84.0) {
            sb += &(text_or_empty(&ErrorMessage::new("ERROR_GEO_LATITUDE")) + "\r\n");
            result = false;
        }

        if result {
            geo = Some(Geographic::new(lon, lat));
        }
        TryParseResult { ok: result, value: geo, error_message: sb, valid_items: list }
    }

    /// `SetDatum(GeoDatum datum)`: shifts the coordinates into the other datum. The datum is
    /// assigned before the shift, so a failed shift to Potsdam leaves the datum changed and
    /// the coordinates as they were, as in C#.
    pub fn set_datum(&mut self, datum: GeoDatum) -> Result<(), Exception> {
        if datum != self.datum() {
            self.datum = datum;
            let g = if datum == GeoDatum::Potsdam { TRANSFORM.wgspod(self)? } else { TRANSFORM.podwgs(self) };
            self.set_longitude(g.longitude());
            self.set_latitude(g.latitude());
        }
        Ok(())
    }

    /// `Format(double)`: `"#0.0#####"` in the current culture.
    fn format(value: f64) -> String {
        format_fixed_optional(value, 1, 1, 6, current_culture())
    }

    /// `ToLongString()`: e.g. "Longitude: 8.12345; Latitude: 50.56789 (WGS84)".
    pub fn to_long_string(&self) -> Result<String, Exception> {
        Ok(string_format2(Message::new("GEOGRAPHIC_STRING").text(), &self.lon_string(), &self.lat_string())?
            + " ("
            + &self.datum().to_string()
            + ")")
    }

    /// `ToSexagesimalString()`
    pub fn to_sexagesimal_string(&self) -> Result<String, Exception> {
        let lon = self.lon_sexagesimal()?.to_string();
        let lat = self.lat_sexagesimal()?.to_string();
        Ok(string_format2(Message::new("GEOGRAPHIC_STRING").text(), &lon, &lat)? + " (" + &self.datum().to_string() + ")")
    }

    /// `ToHybridString()`: sexagesimal with the decimal value in brackets.
    pub fn to_hybrid_string(&self) -> Result<String, Exception> {
        let lon = self.lon_sexagesimal()?.to_string() + " (" + &self.lon_string() + ")";
        let lat = self.lat_sexagesimal()?.to_string() + " (" + &self.lat_string() + ")";
        string_format2(Message::new("GEOGRAPHIC_STRING").text(), &lon, &lat)
    }

    /// `explicit operator UTM(Geographic geo)`
    pub fn to_utm(&self) -> Result<UTM, Exception> {
        TRANSFORM.wgsutm(self)
    }

    /// `explicit operator MGRS(Geographic geo)`: a Potsdam-datum object is shifted to WGS84
    /// in place first (C# passes the reference).
    pub fn to_mgrs(&mut self) -> Result<MGRS, Exception> {
        if self.datum() != GeoDatum::WGS84 {
            self.set_datum(GeoDatum::WGS84)?;
        }
        let utm = TRANSFORM.wgsutm(self)?;
        TRANSFORM.utmmgr(&utm)
    }

    /// `MemberwiseClone()`: a new object from longitude and latitude, so the datum resets to
    /// WGS84 as in C#.
    pub fn memberwise_clone(&self) -> Geographic {
        Geographic::new(self.longitude(), self.latitude())
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        cast_i32(self.longitude() * 100.0) ^ cast_i32(self.latitude())
    }
}

impl std::fmt::Display for Geographic {
    /// `ToString()`: e.g. "8.12345; 50.56789".
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}; {}", self.lon_string(), self.lat_string())
    }
}

impl PartialEq for Geographic {
    /// `Equals` / `operator ==`: longitude and latitude only.
    fn eq(&self, geo: &Geographic) -> bool {
        (self.longitude() == geo.longitude()) && (self.latitude() == geo.latitude())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utm_round_trip() {
        let geo = Geographic::new(8.12345, 50.56789);
        let utm = geo.to_utm().unwrap();
        assert_eq!((utm.zone(), utm.band()), (32, "U"));
        let back = utm.to_geographic().unwrap();
        assert!((back.longitude() - 8.12345).abs() < 1e-6);
        assert!((back.latitude() - 50.56789).abs() < 1e-6);
        assert!(Geographic::new(0.0, 85.0).to_utm().is_err());
    }

    #[test]
    fn mgrs_of_a_point() {
        let mut geo = Geographic::new(8.12345, 50.56789);
        let mgrs = geo.to_mgrs().unwrap();
        // east 437xxx in zone 32 (zone % 3 == 2) gives column M; north 56xxxxx gives row B
        assert!(mgrs.to_string().starts_with("32UMB"), "{}", mgrs);
    }
}
