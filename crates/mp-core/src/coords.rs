//! GEO / UTM / MGRS entry points for the app, as a layer over the file-by-file ports Mission
//! Planner calls: GeoUtility (`extlibs::geoutility`, `GeoSystem/{Geographic,UTM,MGRS}.cs`) for
//! the grid readouts and MGRS entry, `PointLatLngAlt.GetUTMZone/ToUTM` (ProjNet) for signed-zone
//! UTM, and `utmpos.ToLLA` (ProjNet inverse) back to WGS84.

use crate::extlibs::geoutility::geosystem::{Geographic, MGRS, UTM};
use crate::extlibs::utilities::pointlatlngalt::PointLatLngAlt;
use crate::extlibs::utilities::utmpos::utmpos;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Utm {
    pub zone: i32,
    pub band: char,
    pub east: f64,
    pub north: f64,
}

/// .NET Framework `double.ToString()` in the current culture (G15).
pub fn net_double(x: f64) -> String {
    crate::dotnet::double_to_string(x)
}

fn from_port(u: &UTM) -> Option<Utm> {
    Some(Utm { zone: u.zone(), band: u.band().chars().next()?, east: u.east(), north: u.north() })
}

fn to_port(u: &Utm) -> Option<UTM> {
    UTM::with_band(u.zone, &u.band.to_string(), u.east, u.north).ok()
}

/// `(UTM)new Geographic(lng, lat)`; None where GeoUtility throws (outside -80..84, bad longitude).
pub fn geo_to_utm(lat: f64, lng: f64) -> Option<Utm> {
    from_port(&Geographic::new(lng, lat).to_utm().ok()?)
}

/// `UTM.ToString()`: "55H 694420.123 6083869.456".
pub fn utm_string(u: &Utm) -> String {
    to_port(u).map(|p| p.to_string()).unwrap_or_default()
}

/// `(Geographic)utm` (`UTMWGS`); NaN where GeoUtility throws.
pub fn utm_to_geo(u: &Utm) -> (f64, f64) {
    to_port(u)
        .and_then(|p| p.to_geographic().ok())
        .map(|g| (g.latitude(), g.longitude()))
        .unwrap_or((f64::NAN, f64::NAN))
}

/// `PointLatLngAlt.GetUTMZone()`: negative in the southern hemisphere.
pub fn signed_zone(lat: f64, lng: f64) -> i32 {
    PointLatLngAlt::new(lat, lng).get_utm_zone()
}

fn first_two(v: Result<Vec<f64>, crate::dotnet::Exception>) -> (f64, f64) {
    match v {
        Ok(v) if v.len() >= 2 => (v[0], v[1]),
        _ => (f64::NAN, f64::NAN),
    }
}

/// `PointLatLngAlt.ToUTM()`: [easting, northing] in the signed zone (ProjNet).
pub fn to_utm_signed(lat: f64, lng: f64) -> (i32, f64, f64) {
    let p = PointLatLngAlt::new(lat, lng);
    let zone = p.get_utm_zone();
    let (e, n) = first_two(p.to_utm_zone(zone));
    (zone, e, n)
}

/// `PointLatLngAlt.ToUTM(utmzone, lat, lng)`: UTM in a given signed zone.
pub fn to_utm_in_zone(zone: i32, lat: f64, lng: f64) -> (i32, f64, f64) {
    let (e, n) = first_two(PointLatLngAlt::to_utm_lat_lng(zone, lat, lng));
    (zone, e, n)
}

/// `new utmpos(east, north, zone).ToLLA()`: a negative zone is southern. None where ProjNet
/// throws (callers leave their values unchanged, as the C# catch blocks do).
pub fn utm_signed_to_geo(zone: i32, east: f64, north: f64) -> Option<(f64, f64)> {
    utmpos::new(east, north, zone).to_lla().ok().map(|p| (p.Lat, p.Lng))
}

/// `(Geographic)new UTM(zone, east, north, hemisphere)` through GeoUtility's UTMWGS series, as
/// FlightPlanner's Enter UTM Coord does (the band is computed from the hemisphere).
pub fn utm_hemisphere_to_geo(zone: i32, east: f64, north: f64, south: bool) -> Result<(f64, f64), crate::dotnet::Exception> {
    use crate::extlibs::geoutility::geosystem::base::Hemisphere;
    let utm = UTM::with_hemisphere(zone, east, north, if south { Hemisphere::South } else { Hemisphere::North })?;
    let g = utm.to_geographic()?;
    Ok((g.latitude(), g.longitude()))
}

/// `((MGRS)new Geographic(lng, lat)).ToString()`, e.g. "55HFA9442083869".
pub fn geo_to_mgrs(lat: f64, lng: f64) -> Option<String> {
    Geographic::new(lng, lat).to_mgrs().ok().map(|m| m.to_string())
}

/// `new MGRS(text)` then `(Geographic)`; None where GeoUtility throws.
pub fn mgrs_to_geo(text: &str) -> Option<(f64, f64)> {
    let g = MGRS::parse(text).ok()?.to_geographic().ok()?;
    Some((g.latitude(), g.longitude()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canberra_round_trips_through_utm_and_mgrs() {
        let (lat, lng) = (-35.363261, 149.165230);
        let u = geo_to_utm(lat, lng).unwrap();
        assert_eq!((u.zone, u.band), (55, 'H'));
        let s = utm_string(&u);
        assert!(s.starts_with("55H 6"), "{s}");
        let (la, lo) = utm_to_geo(&u);
        assert!((la - lat).abs() < 1e-7 && (lo - lng).abs() < 1e-7, "{la} {lo}");

        let m = geo_to_mgrs(lat, lng).unwrap();
        assert!(m.starts_with("55HFA"), "{m}");
        let (la, lo) = mgrs_to_geo(&m).unwrap();
        // one-metre MGRS cell
        assert!((la - lat).abs() < 2e-5 && (lo - lng).abs() < 2e-5, "{la} {lo}");
        // MGRS.TryParse drops spaces and upper-cases
        assert_eq!(mgrs_to_geo(&m.to_lowercase().replace("fa", " fa ")), mgrs_to_geo(&m));

        // ProjNet's exact transverse Mercator against GeoUtility's series rounded to 1 mm
        let (z, e, n) = to_utm_signed(lat, lng);
        assert_eq!(z, -55);
        assert!((e - u.east).abs() < 1e-2 && (n - u.north).abs() < 1e-2, "{e} {n} {u:?}");
        let (la, lo) = utm_signed_to_geo(z, e, n).unwrap();
        assert!((la - lat).abs() < 1e-7 && (lo - lng).abs() < 1e-7);
    }

    #[test]
    fn northern_zone_exceptions_and_invalid_input() {
        assert_eq!(geo_to_utm(60.0, 5.0).unwrap().zone, 32);
        assert_eq!(geo_to_utm(78.0, 15.0).unwrap().zone, 33);
        assert!(geo_to_utm(85.0, 0.0).is_none() && geo_to_mgrs(-81.0, 0.0).is_none());
        for bad in ["", "55H", "55IFA1234", "55HFA123", "XXHFA1234"] {
            assert!(mgrs_to_geo(bad).is_none(), "{bad}");
        }
        assert_eq!(signed_zone(-35.0, 149.0), -55);
        assert_eq!(signed_zone(51.0, -0.1), 30);
    }
}

#[cfg(test)]
mod reference_tests {
    use super::*;

    /// UTM values from the independent PyPI `utm` package. MGRS digits are that package's metres
    /// rounded, as GeoUtility's UTMMGR rounds where standard MGRS truncates; GeoUtility also omits
    /// the zone's leading zero ("6W" not "06W") and strips shared trailing zeros.
    #[test]
    fn matches_independent_reference_points() {
        for (lat, lng, mgrs, east, north) in [
            (-35.363261, 149.165230, "55HFA9671984520", 696719.245, 6084519.587),
            (51.5007, -0.1246, "30UXC9956809428", 699567.540, 5709427.563),
            (40.7128, -74.0060, "18TWL8395907351", 583959.372, 4507350.998),
            (-33.8568, 151.2153, "56HLH3490152289", 334900.570, 6252288.753),
            (60.0, 5.0, "32VKM7698058157", 276979.926, 6658157.203),
            (0.0001, 0.0001, "31NAA6603300011", 166032.586, 11.068),
            (64.5, -147.7, "6WVS6637552918", 466375.136, 7152917.862),
        ] {
            let u = geo_to_utm(lat, lng).unwrap();
            assert!((u.east - east).abs() < 0.01 && (u.north - north).abs() < 0.01, "{lat},{lng}: {u:?}");
            assert_eq!(geo_to_mgrs(lat, lng).unwrap(), mgrs, "{lat},{lng}");
            let (la, lo) = mgrs_to_geo(mgrs).unwrap();
            assert!((la - lat).abs() < 2e-5 && (lo - lng).abs() < 2e-5, "{mgrs}: {la},{lo}");
        }
    }
}
