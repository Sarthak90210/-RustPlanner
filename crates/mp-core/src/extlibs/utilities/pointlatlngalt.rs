//! Port of `ExtLibs/Utilities/PointLatLngAlt.cs`
//!
//! Not ported yet: `Transform(Coordinate[], string sourceWkt, string targetWkt)`, which needs
//! ProjNet's WKT reader and is not called anywhere in Mission Planner.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use super::locationwp::Locationwp;
use super::math::MathHelper;
use crate::dotnet::drawing::Color;
use crate::dotnet::{cast_i32, double_to_int64_bits, double_to_string, double_try_parse, Exception};
use crate::extlibs::geoutility::geosystem::base::Hemisphere;
use crate::extlibs::geoutility::geosystem::{Geographic, UTM};
use crate::extlibs::gmap_net_core::gmap_net::PointLatLng;
use crate::extlibs::projnet::coordinatesystems::projectedcoordinatesystem::ProjectedCoordinateSystem;
use crate::extlibs::projnet::coordinatesystems::projections::transversemercator::TransverseMercator;
use crate::extlibs::projnet::coordinatesystems::transformations::coordinatetransformationfactory;
use crate::extlibs::projnet::coordinatesystems::transformations::mathtransform::MathTransform;

/// `Math.Abs(int)`: throws for `int.MinValue`.
fn math_abs_i32(v: i32) -> Result<i32, Exception> {
    v.checked_abs().ok_or_else(|| Exception::Overflow("Negating the minimum value of a twos complement number is invalid.".into()))
}

/// `list[index]` past the end of a `List<T>`.
fn list_index_out_of_range() -> Exception {
    Exception::ArgumentOutOfRange(
        "Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into(),
    )
}

/// `public class PointLatLngAlt : IComparable`
#[allow(non_snake_case)]
#[derive(Debug, Clone)]
pub struct PointLatLngAlt {
    pub Lat: f64,
    pub Lng: f64,
    /// Altitude in meters
    pub Alt: f64,
    pub Tag: String,
    pub Tag2: String,
    pub color: Color,
}

impl Default for PointLatLngAlt {
    /// `new PointLatLngAlt()`
    fn default() -> Self {
        PointLatLngAlt { Lat: 0.0, Lng: 0.0, Alt: 0.0, Tag: String::new(), Tag2: String::new(), color: Color::WHITE }
    }
}

/// `private static Dictionary<int, ICoordinateTransformation> coordtrans`
fn coordtrans() -> &'static Mutex<HashMap<i32, Arc<TransverseMercator>>> {
    static C: OnceLock<Mutex<HashMap<i32, Arc<TransverseMercator>>>> = OnceLock::new();
    C.get_or_init(|| Mutex::new(HashMap::new()))
}

impl PointLatLngAlt {
    /// `PointLatLngAlt.Zero`
    pub fn zero() -> Self {
        Self::default()
    }

    /// `new PointLatLngAlt(double lat, double lng, double alt, string tag)`
    pub fn new_tagged(lat: f64, lng: f64, alt: f64, tag: &str) -> Self {
        PointLatLngAlt { Lat: lat, Lng: lng, Alt: alt, Tag: tag.to_string(), ..Default::default() }
    }

    /// `new PointLatLngAlt(double lat, double lng)`
    pub fn new(lat: f64, lng: f64) -> Self {
        PointLatLngAlt { Lat: lat, Lng: lng, ..Default::default() }
    }

    /// `new PointLatLngAlt(double lat, double lng, double alt)`
    pub fn new_alt(lat: f64, lng: f64, alt: f64) -> Self {
        PointLatLngAlt { Lat: lat, Lng: lng, Alt: alt, ..Default::default() }
    }

    /// `new PointLatLngAlt(GMap.NET.PointLatLng pll)`
    pub fn from_point(pll: PointLatLng) -> Self {
        PointLatLngAlt { Lat: pll.lat(), Lng: pll.lng(), ..Default::default() }
    }

    /// `new PointLatLngAlt(Locationwp locwp)`
    pub fn from_locationwp(locwp: &Locationwp) -> Self {
        PointLatLngAlt { Lat: locwp.lat, Lng: locwp.lng, Alt: locwp.alt as f64, ..Default::default() }
    }

    /// `new PointLatLngAlt(double[] dblarr)`: lat, lng[, alt].
    pub fn from_dblarr(dblarr: &[f64]) -> Result<Self, Exception> {
        if dblarr.len() < 2 {
            return Err(crate::dotnet::index_out_of_range());
        }
        let mut p = PointLatLngAlt { Lat: dblarr[0], Lng: dblarr[1], ..Default::default() };
        if dblarr.len() > 2 {
            p.Alt = dblarr[2];
        }
        Ok(p)
    }

    /// `new PointLatLngAlt(PointLatLngAlt plla)`: copies Lat, Lng, Alt, color and Tag (not
    /// Tag2).
    pub fn copy_of(plla: &PointLatLngAlt) -> Self {
        PointLatLngAlt { Lat: plla.Lat, Lng: plla.Lng, Alt: plla.Alt, color: plla.color.clone(), Tag: plla.Tag.clone(), ..Default::default() }
    }

    /// `Point()`
    pub fn point(&self) -> PointLatLng {
        PointLatLng::new(self.Lat, self.Lng)
    }

    /// `implicit operator double[](PointLatLngAlt a)`: `{ Lng, Lat }`.
    pub fn to_double_array(&self) -> Vec<f64> {
        vec![self.Lng, self.Lat]
    }

    /// `implicit operator PointLatLngAlt(double[] a)`: `{ Lng, Lat[, Alt] }`, the reverse
    /// order of the `double[]` constructor.
    pub fn from_lng_lat_array(a: &[f64]) -> Result<Self, Exception> {
        if a.len() < 2 {
            return Err(crate::dotnet::index_out_of_range());
        }
        if a.len() == 3 {
            return Ok(PointLatLngAlt { Lng: a[0], Lat: a[1], Alt: a[2], ..Default::default() });
        }
        Ok(PointLatLngAlt { Lng: a[0], Lat: a[1], ..Default::default() })
    }

    /// `operator ==(PointLatLngAlt p1, PointLatLng p2)`: Lat and Lng only.
    pub fn eq_point(&self, p2: &PointLatLng) -> bool {
        self.Lat == p2.lat() && self.Lng == p2.lng()
    }

    /// `GetHashCode()`: the XOR of the bit patterns, truncated to 32 bits.
    pub fn get_hash_code(&self) -> i32 {
        (double_to_int64_bits(self.Lat) ^ double_to_int64_bits(self.Lng) ^ double_to_int64_bits(self.Alt)) as i32
    }

    /// `GetUTMZone()`: negative in the southern hemisphere.
    pub fn get_utm_zone(&self) -> i32 {
        let mut zone = cast_i32((self.Lng - -186.0) / 6.0);
        if self.Lat < 0.0 {
            zone = zone.wrapping_mul(-1);
        }
        zone
    }

    /// `GetLngStartFromZone()`
    pub fn get_lng_start_from_zone(&self) -> Result<i32, Exception> {
        let zone = self.get_utm_zone();
        Ok(((math_abs_i32(zone)?.wrapping_mul(6)).wrapping_sub(180)).wrapping_sub(6))
    }

    /// `GetLngEndFromZone()`
    pub fn get_lng_end_from_zone(&self) -> Result<i32, Exception> {
        let zone = self.get_utm_zone();
        Ok((math_abs_i32(zone)?.wrapping_mul(6)).wrapping_sub(180))
    }

    /// `GetLatStartUTM()`
    pub fn get_lat_start_utm(&self) -> i32 {
        cast_i32(self.Lat - (self.Lat % 8.0))
    }

    /// `GetLatEndUTM()`
    pub fn get_lat_end_utm(&self) -> i32 {
        cast_i32(self.Lat - (self.Lat % 8.0)).wrapping_sub(8)
    }

    /// `GetFriendlyZone()`: `ToString("0N;0S")`, e.g. "32N" or "32S" (the negative section
    /// has no minus sign; zero uses the first section).
    pub fn get_friendly_zone(&self) -> String {
        let z = self.get_utm_zone();
        if z < 0 {
            format!("{}S", (z as i64).abs())
        } else {
            format!("{z}N")
        }
    }

    /// `GetMGRS()`
    pub fn get_mgrs(&self) -> Result<String, Exception> {
        let mut geo = Geographic::new(self.Lng, self.Lat);
        let mgrs = geo.to_mgrs()?;
        Ok(mgrs.to_string())
    }

    /// `FromUTM(int zone, double x, double y)`: a negative zone means the southern hemisphere.
    pub fn from_utm(zone: i32, x: f64, y: f64) -> Result<PointLatLngAlt, Exception> {
        let utm = UTM::with_hemisphere(
            math_abs_i32(zone)?,
            x,
            y,
            if zone < 0 { Hemisphere::South } else { Hemisphere::North },
        )?;

        let ans: PointLatLngAlt = utm.to_geographic()?.into();

        Ok(ans)
    }

    /// `ToUTM()`: in this point's own zone.
    pub fn to_utm(&self) -> Result<Vec<f64>, Exception> {
        self.to_utm_zone(self.get_utm_zone())
    }

    /// `ToUTM(int utmzone)`: force a zone.
    pub fn to_utm_zone(&self, utmzone: i32) -> Result<Vec<f64>, Exception> {
        let trans = Self::try_get_transform(utmzone, self.Lat)?;

        let pll = [self.Lng, self.Lat];

        // get leader utm coords
        trans.transform(&pll)
    }

    /// `TryGetTransform(int utmzone, double lat)`: one cached transformation per signed zone.
    /// The cache key is the zone after the sign fix-up, but the hemisphere comes from `lat`:
    /// the first call for a key decides which hemisphere every later call with it gets, as in
    /// the C#.
    fn try_get_transform(mut utmzone: i32, lat: f64) -> Result<Arc<TransverseMercator>, Exception> {
        if lat < 0.0 && utmzone > 0 {
            utmzone = utmzone.wrapping_mul(-1);
        }

        {
            let c = coordtrans().lock().unwrap_or_else(|e| e.into_inner());
            if let Some(t) = c.get(&utmzone) {
                return Ok(t.clone());
            }
        }

        let utm = ProjectedCoordinateSystem::wgs84_utm(math_abs_i32(utmzone)?, !(lat < 0.0));
        let trans = Arc::new(coordinatetransformationfactory::create_from_wgs84_to_projected(&utm)?);

        let mut c = coordtrans().lock().unwrap_or_else(|e| e.into_inner());
        c.insert(utmzone, trans);
        Ok(c[&utmzone].clone())
    }

    /// `static List<double[]> ToUTM(int utmzone, List<PointLatLngAlt> list)`: the hemisphere
    /// comes from the first point.
    pub fn to_utm_list(utmzone: i32, list: &[PointLatLngAlt]) -> Result<Vec<Vec<f64>>, Exception> {
        let first = list.first().ok_or_else(list_index_out_of_range)?;
        let trans = Self::try_get_transform(utmzone, first.Lat)?;

        let data: Vec<Vec<f64>> = list.iter().map(|x| x.to_double_array()).collect();

        trans.transform_list(&data)
    }

    /// `static double[] ToUTM(int utmzone, double lat, double lng)`
    pub fn to_utm_lat_lng(utmzone: i32, lat: f64, lng: f64) -> Result<Vec<f64>, Exception> {
        let trans = Self::try_get_transform(utmzone, lat)?;

        trans.transform(&[lng, lat])
    }

    /// `newpos(bearing, distance)`: extrapolate latitude/longitude given a heading and
    /// distance (http://www.movable-type.co.uk/scripts/latlong.html).
    pub fn newpos(&self, bearing: f64, distance: f64) -> PointLatLngAlt {
        let radius_of_earth = 6378100.0; // in meters

        let lat1 = MathHelper::DEG2RAD * (self.Lat);
        let lon1 = MathHelper::DEG2RAD * (self.Lng);
        let brng = MathHelper::DEG2RAD * (bearing);
        let dr = distance / radius_of_earth;

        let lat2 = (lat1.sin() * dr.cos() + lat1.cos() * dr.sin() * brng.cos()).asin();
        let lon2 = lon1 + (brng.sin() * dr.sin() * lat1.cos()).atan2(dr.cos() - lat1.sin() * lat2.sin());

        let latout = MathHelper::RAD2DEG * (lat2);
        let lngout = MathHelper::RAD2DEG * (lon2);

        PointLatLngAlt::new_tagged(latout, lngout, self.Alt, &self.Tag)
    }

    /// `gps_offset(east, north)`: move a point a specific number of meters.
    pub fn gps_offset(&self, east: f64, north: f64) -> PointLatLngAlt {
        let bearing = east.atan2(north) * MathHelper::RAD2DEG;
        let distance = (east.powf(2.0) + north.powf(2.0)).sqrt();
        self.newpos(bearing, distance)
    }

    /// `GetBearing(p2)`: initial great-circle bearing in degrees, 0..360.
    pub fn get_bearing(&self, p2: &PointLatLngAlt) -> f64 {
        let latitude1 = MathHelper::DEG2RAD * (self.Lat);
        let latitude2 = MathHelper::DEG2RAD * (p2.Lat);
        let longitude_difference = MathHelper::DEG2RAD * (p2.Lng - self.Lng);

        let y = longitude_difference.sin() * latitude2.cos();
        let x = latitude1.cos() * latitude2.sin() - latitude1.sin() * latitude2.cos() * longitude_difference.cos();

        (MathHelper::RAD2DEG * (y.atan2(x)) + 360.0) % 360.0
    }

    /// `GetAngle(point, heading)`: bearing to the point relative to the heading, -180..180.
    pub fn get_angle(&self, point: &PointLatLngAlt, heading: f64) -> f64 {
        let mut angle = self.get_bearing(point) - heading;
        if angle < -180.0 {
            angle += 360.0;
        }
        if angle > 180.0 {
            angle -= 360.0;
        }
        angle
    }

    /// `GetDistance(p2)`: haversine distance in meters (radius 6371 km).
    pub fn get_distance(&self, p2: &PointLatLngAlt) -> f64 {
        let d = self.Lat * 0.017453292519943295;
        let num2 = self.Lng * 0.017453292519943295;
        let num3 = p2.Lat * 0.017453292519943295;
        let num4 = p2.Lng * 0.017453292519943295;
        let num5 = num4 - num2;
        let num6 = num3 - d;
        let num7 = (num6 / 2.0).sin().powf(2.0) + ((d.cos() * num3.cos()) * (num5 / 2.0).sin().powf(2.0));
        let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
        (6371.0 * num8) * 1000.0 // M
    }

    /// `GetDistance2(p2)`: the same haversine written out, with `MathHelper.deg2rad`.
    pub fn get_distance2(&self, p2: &PointLatLngAlt) -> f64 {
        // http://www.movable-type.co.uk/scripts/latlong.html
        let r = 6371.0; // 6371 km
        let d_lat = (p2.Lat - self.Lat) * MathHelper::DEG2RAD;
        let d_lon = (p2.Lng - self.Lng) * MathHelper::DEG2RAD;
        let lat1 = self.Lat * MathHelper::DEG2RAD;
        let lat2 = p2.Lat * MathHelper::DEG2RAD;

        let a = (d_lat / 2.0).sin() * (d_lat / 2.0).sin() + (d_lon / 2.0).sin() * (d_lon / 2.0).sin() * lat1.cos() * lat2.cos();
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        r * c * 1000.0 // M
    }

    /// `GetGreatCirclePathPoint(p2, f)`: the point at fraction `f` (0-1) along the great
    /// circle (https://www.movable-type.co.uk/scripts/latlong.html).
    pub fn get_great_circle_path_point(&self, p2: &PointLatLngAlt, f: f64) -> PointLatLngAlt {
        let _d_lat = (p2.Lat - self.Lat) * MathHelper::DEG2RAD;
        let _d_lon = (p2.Lng - self.Lng) * MathHelper::DEG2RAD;
        let r = 6378100.0; // 6371 km
        let angdist = self.get_distance(p2) / r;

        let a = ((1.0 - f) * angdist).sin() / angdist.sin();
        let b = (f * angdist).sin() / angdist.sin();
        let x = a * (self.Lat * MathHelper::DEG2RAD).cos() * (self.Lng * MathHelper::DEG2RAD).cos()
            + b * (p2.Lat * MathHelper::DEG2RAD).cos() * (p2.Lng * MathHelper::DEG2RAD).cos();
        let y = a * (self.Lat * MathHelper::DEG2RAD).cos() * (self.Lng * MathHelper::DEG2RAD).sin()
            + b * (p2.Lat * MathHelper::DEG2RAD).cos() * (p2.Lng * MathHelper::DEG2RAD).sin();
        let z = a * (self.Lat * MathHelper::DEG2RAD).sin() + b * (p2.Lat * MathHelper::DEG2RAD).sin();
        let alat = z.atan2((x * x + y * y).sqrt());
        let alon = y.atan2(x);

        PointLatLngAlt::new(alat * MathHelper::RAD2DEG, alon * MathHelper::RAD2DEG)
    }

    /// `CompareTo(object obj)`: orders by the numeric value of `Tag`; 1 when `obj` is not a
    /// `PointLatLngAlt`, 0 when either tag is not a number.
    pub fn compare_to(&self, obj: Option<&PointLatLngAlt>) -> i32 {
        let Some(pnt) = obj else {
            return 1;
        };

        let Some(wpnothis) = double_try_parse(&self.Tag) else {
            return 0;
        };

        match double_try_parse(&pnt.Tag) {
            Some(wpno) => {
                if wpno < wpnothis {
                    return 1;
                }
                if wpno > wpnothis {
                    return -1;
                }
                0
            }
            None => 0,
        }
    }
}

impl From<PointLatLng> for PointLatLngAlt {
    /// `implicit operator PointLatLngAlt(PointLatLng a)`
    fn from(a: PointLatLng) -> Self {
        PointLatLngAlt::from_point(a)
    }
}

impl From<&PointLatLngAlt> for PointLatLng {
    /// `implicit operator PointLatLng(PointLatLngAlt a)`
    fn from(a: &PointLatLngAlt) -> Self {
        a.point()
    }
}

impl From<Geographic> for PointLatLngAlt {
    /// `implicit operator PointLatLngAlt(GeoUtility.GeoSystem.Geographic geo)`
    fn from(geo: Geographic) -> Self {
        PointLatLngAlt { Lat: geo.latitude(), Lng: geo.longitude(), ..Default::default() }
    }
}

impl From<&Locationwp> for PointLatLngAlt {
    /// `explicit operator PointLatLngAlt(Locationwp v)`
    fn from(v: &Locationwp) -> Self {
        PointLatLngAlt::new_alt(v.lat, v.lng, v.alt as f64)
    }
}

impl PartialEq for PointLatLngAlt {
    /// `Equals(object)` / `operator ==`: Lat, Lng, Alt, color and Tag (not Tag2).
    fn eq(&self, plla: &PointLatLngAlt) -> bool {
        self.Lat == plla.Lat && self.Lng == plla.Lng && self.Alt == plla.Alt && self.color == plla.color && self.Tag == plla.Tag
    }
}

impl std::fmt::Display for PointLatLngAlt {
    /// `ToString()`: "Lat,Lng,Alt,Tag" in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{},{},{},{}", double_to_string(self.Lat), double_to_string(self.Lng), double_to_string(self.Alt), self.Tag)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zones() {
        let p = PointLatLngAlt::new(-35.36, 149.16);
        assert_eq!(p.get_utm_zone(), -55);
        assert_eq!(p.get_friendly_zone(), "55S");
        assert_eq!(PointLatLngAlt::new(51.5, -0.12).get_friendly_zone(), "30N");
        assert_eq!(p.get_lng_start_from_zone().unwrap(), 144);
        assert_eq!(p.get_lng_end_from_zone().unwrap(), 150);
    }

    #[test]
    fn utm_round_trip() {
        let p = PointLatLngAlt::new(-35.36, 149.16);
        let utm = p.to_utm().unwrap();
        // false northing 10,000 km in the south; easting near the 147E central meridian
        assert!(utm[0] > 600000.0 && utm[0] < 750000.0, "{utm:?}");
        assert!(utm[1] > 6_000_000.0 && utm[1] < 6_200_000.0, "{utm:?}");
        let back = PointLatLngAlt::from_utm(p.get_utm_zone(), utm[0], utm[1]).unwrap();
        assert!((back.Lat - p.Lat).abs() < 1e-5 && (back.Lng - p.Lng).abs() < 1e-5, "{back}");
    }

    #[test]
    fn distance_and_bearing() {
        let a = PointLatLngAlt::new(0.0, 0.0);
        let b = a.newpos(90.0, 1000.0);
        assert!((a.get_distance(&b) - 1000.0).abs() < 2.0);
        assert!((a.get_bearing(&b) - 90.0).abs() < 1e-6);
        assert_eq!(a.compare_to(None), 1);
    }

    #[test]
    fn equality_includes_color_and_tag() {
        let a = PointLatLngAlt::new_tagged(1.0, 2.0, 3.0, "x");
        let mut b = PointLatLngAlt::copy_of(&a);
        assert_eq!(a, b);
        b.Tag2 = "y".into();
        assert_eq!(a, b);
        b.color = Color::RED;
        assert_ne!(a, b);
    }
}
