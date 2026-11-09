//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/PureProjection.cs`
//!
//! `public abstract class PureProjection` is a trait: implementors give the abstract members
//! and a `ProjectionCache` (the two per-zoom dictionaries the base constructor creates); the
//! virtual and non-virtual members are provided methods. The protected static math helpers
//! are free functions.

#![allow(non_snake_case)]

use std::collections::HashMap;
use std::sync::Mutex;

use super::gpoint::GPoint;
use super::gsize::GSize;
use super::pointlatlng::PointLatLng;
use super::rectlatlng::RectLatLng;
use crate::dotnet::{cast_i64, math_max, math_min, Exception};

/// A `PointLatLng` dictionary key: `Equals` compares Lat and Lng with `==` (0 equals -0).
/// A point with a NaN coordinate equals nothing, so it is never found (`None`).
fn key(p: PointLatLng) -> Option<(u64, u64)> {
    if p.lat().is_nan() || p.lng().is_nan() {
        return None;
    }
    let k = |v: f64| if v == 0.0 { 0 } else { v.to_bits() };
    Some((k(p.lat()), k(p.lng())))
}

/// `FromLatLngToPixelCache` / `FromPixelToLatLngCache`: 33 dictionaries each (one per zoom).
#[derive(Debug)]
pub struct ProjectionCache {
    to_pixel: Mutex<Vec<HashMap<(u64, u64), GPoint>>>,
    to_latlng: Mutex<Vec<HashMap<GPoint, PointLatLng>>>,
}

impl Default for ProjectionCache {
    fn default() -> Self {
        ProjectionCache { to_pixel: Mutex::new(vec![HashMap::new(); 33]), to_latlng: Mutex::new(vec![HashMap::new(); 33]) }
    }
}

fn list_index() -> Exception {
    Exception::ArgumentOutOfRange("Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into())
}

/// `public abstract class PureProjection`
pub trait PureProjection: Send + Sync {
    /// the base class's caches
    fn cache(&self) -> &ProjectionCache;

    /// size of tile
    fn TileSize(&self) -> GSize;
    /// Semi-major axis of ellipsoid, in meters
    fn Axis(&self) -> f64;
    /// Flattening of ellipsoid
    fn Flattening(&self) -> f64;
    /// get pixel coordinates from lat/lng
    fn FromLatLngToPixel(&self, lat: f64, lng: f64, zoom: i32) -> GPoint;
    /// gets lat/lng coordinates from pixel coordinates
    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng;
    /// min. tile in tiles at custom zoom level
    fn GetTileMatrixMinXY(&self, zoom: i32) -> GSize;
    /// max. tile in tiles at custom zoom level
    fn GetTileMatrixMaxXY(&self, zoom: i32) -> GSize;

    /// `FromLatLngToPixel(PointLatLng p, int zoom)`
    fn FromLatLngToPixel_p(&self, p: PointLatLng, zoom: i32) -> GPoint {
        self.FromLatLngToPixel(p.lat(), p.lng(), zoom)
    }

    /// `FromLatLngToPixel(PointLatLng p, int zoom, bool useCache)`
    fn FromLatLngToPixel_cached(&self, p: PointLatLng, zoom: i32, useCache: bool) -> Result<GPoint, Exception> {
        if !useCache {
            return Ok(self.FromLatLngToPixel(p.lat(), p.lng(), zoom));
        }
        let z = usize::try_from(zoom).map_err(|_| list_index())?;
        {
            let c = self.cache().to_pixel.lock().unwrap();
            let m = c.get(z).ok_or_else(list_index)?;
            if let Some(r) = key(p).and_then(|k| m.get(&k)) {
                return Ok(*r);
            }
        }
        let ret = self.FromLatLngToPixel(p.lat(), p.lng(), zoom);
        // a NaN key is added again on every call and never found; only lookups are visible
        if let Some(k) = key(p) {
            self.cache().to_pixel.lock().unwrap()[z].insert(k, ret);
        }
        // for reverse cache
        self.cache().to_latlng.lock().unwrap()[z].entry(ret).or_insert(p);
        log::debug!("FromLatLngToPixelCache[{zoom}] added {p:?} with {ret}");
        Ok(ret)
    }

    /// `FromPixelToLatLng(GPoint p, int zoom)`
    fn FromPixelToLatLng_p(&self, p: GPoint, zoom: i32) -> PointLatLng {
        self.FromPixelToLatLng(p.x(), p.y(), zoom)
    }

    /// `FromPixelToLatLng(GPoint p, int zoom, bool useCache)`
    fn FromPixelToLatLng_cached(&self, p: GPoint, zoom: i32, useCache: bool) -> Result<PointLatLng, Exception> {
        if !useCache {
            return Ok(self.FromPixelToLatLng(p.x(), p.y(), zoom));
        }
        let z = usize::try_from(zoom).map_err(|_| list_index())?;
        {
            let c = self.cache().to_latlng.lock().unwrap();
            let m = c.get(z).ok_or_else(list_index)?;
            if let Some(r) = m.get(&p) {
                return Ok(*r);
            }
        }
        let ret = self.FromPixelToLatLng(p.x(), p.y(), zoom);
        self.cache().to_latlng.lock().unwrap()[z].insert(p, ret);
        // for reverse cache
        if let Some(k) = key(ret) {
            self.cache().to_pixel.lock().unwrap()[z].entry(k).or_insert(p);
        }
        log::debug!("FromPixelToLatLngCache[{zoom}] added {p} with {ret:?}");
        Ok(ret)
    }

    /// gets tile coordinate from pixel coordinates
    fn FromPixelToTileXY(&self, p: GPoint) -> GPoint {
        GPoint::new(p.x() / self.TileSize().width(), p.y() / self.TileSize().height())
    }

    /// gets pixel coordinate from tile coordinate
    fn FromTileXYToPixel(&self, p: GPoint) -> GPoint {
        GPoint::new(p.x().wrapping_mul(self.TileSize().width()), p.y().wrapping_mul(self.TileSize().height()))
    }

    /// gets matrix size in tiles
    fn GetTileMatrixSizeXY(&self, zoom: i32) -> GSize {
        let s_min = self.GetTileMatrixMinXY(zoom);
        let s_max = self.GetTileMatrixMaxXY(zoom);
        GSize::new(s_max.width() - s_min.width() + 1, s_max.height() - s_min.height() + 1)
    }

    /// tile matrix size in pixels at custom zoom level
    fn GetTileMatrixItemCount(&self, zoom: i32) -> i64 {
        let s = self.GetTileMatrixSizeXY(zoom);
        s.width().wrapping_mul(s.height())
    }

    /// gets matrix size in pixels
    fn GetTileMatrixSizePixel(&self, zoom: i32) -> GSize {
        let s = self.GetTileMatrixSizeXY(zoom);
        GSize::new(s.width().wrapping_mul(self.TileSize().width()), s.height().wrapping_mul(self.TileSize().height()))
    }

    /// gets all tiles in rect at specific zoom
    fn GetAreaTileList(&self, rect: RectLatLng, zoom: i32, padding: i32) -> Vec<GPoint> {
        let mut ret: Vec<GPoint> = Vec::new();
        let top_left = self.FromPixelToTileXY(self.FromLatLngToPixel_p(rect.location_top_left(), zoom));
        let right_bottom = self.FromPixelToTileXY(self.FromLatLngToPixel_p(rect.location_right_bottom(), zoom));
        let mut x = top_left.x() - padding as i64;
        while x <= right_bottom.x() + padding as i64 {
            let mut y = top_left.y() - padding as i64;
            while y <= right_bottom.y() + padding as i64 {
                let p = GPoint::new(x, y);
                if !ret.contains(&p) && p.x() >= 0 && p.y() >= 0 {
                    ret.push(p);
                }
                y += 1;
            }
            x += 1;
        }
        ret
    }

    /// get number of tiles in rect at specific zoom
    fn GetAreaTileNumber(&self, rect: RectLatLng, zoom: i32, _padding: i32) -> i64 {
        let top_left = self.FromPixelToTileXY(self.FromLatLngToPixel_p(rect.location_top_left(), zoom));
        let right_bottom = self.FromPixelToTileXY(self.FromLatLngToPixel_p(rect.location_right_bottom(), zoom));
        (right_bottom.x() - top_left.x() + 1).wrapping_mul(right_bottom.y() - top_left.y() + 1)
    }

    /// The ground resolution: metres on the ground per pixel.
    fn GetGroundResolution(&self, zoom: i32, latitude: f64) -> f64 {
        ((latitude * (std::f64::consts::PI / 180.0)).cos() * 2.0 * std::f64::consts::PI * self.Axis()) / self.GetTileMatrixSizePixel(zoom).width() as f64
    }

    /// gets boundaries
    fn Bounds(&self) -> RectLatLng {
        RectLatLng::from_ltrb(-180.0, 90.0, 180.0, -90.0)
    }

    /// distance (in km) between two points specified by latitude/longitude (Haversine on
    /// `Axis`)
    fn GetDistance(&self, p1: PointLatLng, p2: PointLatLng) -> f64 {
        let d = std::f64::consts::PI / 180.0;
        let (lat1, lng1, lat2, lng2) = (p1.lat() * d, p1.lng() * d, p2.lat() * d, p2.lng() * d);
        let d_longitude = lng2 - lng1;
        let d_latitude = lat2 - lat1;
        let a = (d_latitude / 2.0).sin().powf(2.0) + lat1.cos() * lat2.cos() * (d_longitude / 2.0).sin().powf(2.0);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        (self.Axis() / 1000.0) * c
    }

    fn GetDistanceInPixels(&self, point1: GPoint, point2: GPoint) -> f64 {
        let a = (point2.x() - point1.x()) as f64;
        let b = (point2.y() - point1.y()) as f64;
        (a * a + b * b).sqrt()
    }

    /// Accepts two coordinates in degrees; 0 to 360.
    fn GetBearing(&self, p1: PointLatLng, p2: PointLatLng) -> f64 {
        let latitude1 = DegreesToRadians(p1.lat());
        let latitude2 = DegreesToRadians(p2.lat());
        let longitude_difference = DegreesToRadians(p2.lng() - p1.lng());
        let y = longitude_difference.sin() * latitude2.cos();
        let x = latitude1.cos() * latitude2.sin() - latitude1.sin() * latitude2.cos() * longitude_difference.cos();
        (RadiansToDegrees(y.atan2(x)) + 360.0) % 360.0
    }

    /// geodetic (degrees, height in m) to earth-centred cartesian, as (X, Y, Z)
    fn FromGeodeticToCartesian(&self, lat: f64, lng: f64, height: f64) -> (f64, f64, f64) {
        let lat = (std::f64::consts::PI / 180.0) * lat;
        let lng = (std::f64::consts::PI / 180.0) * lng;
        let axis = self.Axis();
        let b = axis * (1.0 - self.Flattening());
        let ee = 1.0 - (b / axis) * (b / axis);
        let n = axis / (1.0 - ee * lat.sin() * lat.sin()).sqrt();
        let x = (n + height) * lat.cos() * lng.cos();
        let y = (n + height) * lat.cos() * lng.sin();
        let z = (n * (b / axis) * (b / axis) + height) * lat.sin();
        (x, y, z)
    }

    /// earth-centred cartesian to geodetic, as (Lat, Lng) in degrees
    fn FromCartesianTGeodetic(&self, x: f64, y: f64, z: f64) -> (f64, f64) {
        let f = self.Flattening();
        let axis = self.Axis();
        let e = f * (2.0 - f);
        let lng = y.atan2(x);
        let p = (x * x + y * y).sqrt();
        let theta = z.atan2(p * (1.0 - f));
        let st = theta.sin();
        let ct = theta.cos();
        let lat = (z + e / (1.0 - f) * axis * st * st * st).atan2(p - e * axis * ct * ct * ct);
        (lat / (std::f64::consts::PI / 180.0), lng / (std::f64::consts::PI / 180.0))
    }
}

// -- math functions --

/// `PI`
pub const PI: f64 = std::f64::consts::PI;
/// Half of PI
pub const HALF_PI: f64 = PI * 0.5;
/// PI * 2
pub const TWO_PI: f64 = PI * 2.0;
/// `EPSLoN`
pub const EPSLON: f64 = 1.0e-10;
/// `MAX_VAL`
pub const MAX_VAL: f64 = 4.0;
/// `MAXLONG`
pub const MAXLONG: f64 = 2147483647.0;
/// `DBLLONG`
pub const DBLLONG: f64 = 4.61168601e18;
const R2D: f64 = 180.0 / std::f64::consts::PI;
const D2R: f64 = std::f64::consts::PI / 180.0;

pub fn DegreesToRadians(deg: f64) -> f64 {
    D2R * deg
}

pub fn RadiansToDegrees(rad: f64) -> f64 {
    R2D * rad
}

/// return the sign of an argument (0 counts as positive)
pub fn Sign(x: f64) -> f64 {
    if x < 0.0 {
        -1.0
    } else {
        1.0
    }
}

pub fn AdjustLongitude(mut x: f64) -> f64 {
    let mut count: i64 = 0;
    loop {
        if x.abs() <= PI {
            break;
        } else if cast_i64((x / std::f64::consts::PI).abs()) < 2 {
            x -= Sign(x) * TWO_PI;
        } else if (cast_i64((x / TWO_PI).abs()) as f64) < MAXLONG {
            x -= cast_i64(x / TWO_PI) as f64 * TWO_PI;
        } else if (cast_i64((x / (MAXLONG * TWO_PI)).abs()) as f64) < MAXLONG {
            x -= cast_i64(x / (MAXLONG * TWO_PI)) as f64 * (TWO_PI * MAXLONG);
        } else if (cast_i64((x / (DBLLONG * TWO_PI)).abs()) as f64) < MAXLONG {
            x -= cast_i64(x / (DBLLONG * TWO_PI)) as f64 * (TWO_PI * DBLLONG);
        } else {
            x -= Sign(x) * TWO_PI;
        }
        count += 1;
        if count as f64 > MAX_VAL {
            break;
        }
    }
    x
}

/// calculates the sine and cosine
pub fn SinCos(val: f64) -> (f64, f64) {
    (val.sin(), val.cos())
}

/// e0..e3: the meridian-distance series constants for eccentricity squared `x`
pub fn e0fn(x: f64) -> f64 {
    1.0 - 0.25 * x * (1.0 + x / 16.0 * (3.0 + 1.25 * x))
}

pub fn e1fn(x: f64) -> f64 {
    0.375 * x * (1.0 + 0.25 * x * (1.0 + 0.46875 * x))
}

pub fn e2fn(x: f64) -> f64 {
    0.05859375 * x * x * (1.0 + 0.75 * x)
}

pub fn e3fn(x: f64) -> f64 {
    x * x * x * (35.0 / 3072.0)
}

/// the distance along a meridian from the Equator to latitude phi
pub fn mlfn(e0: f64, e1: f64, e2: f64, e3: f64, phi: f64) -> f64 {
    e0 * phi - e1 * (2.0 * phi).sin() + e2 * (4.0 * phi).sin() - e3 * (6.0 * phi).sin()
}

/// calculates UTM zone number
pub fn GetUTMzone(lon: f64) -> i64 {
    cast_i64(((lon + 180.0) / 6.0) + 1.0)
}

/// Clips a number to the specified minimum and maximum values.
pub fn Clip(n: f64, min_value: f64, max_value: f64) -> f64 {
    math_min(math_max(n, min_value), max_value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn math_helpers() {
        assert_eq!(Sign(0.0), 1.0);
        assert_eq!(Sign(-0.5), -1.0);
        assert!((AdjustLongitude(3.0 * PI) - PI).abs() < 1e-12);
        assert!((AdjustLongitude(-5.0 * PI) + PI).abs() < 1e-9);
        assert!((AdjustLongitude(0.5) - 0.5).abs() < 1e-15);
        assert_eq!(GetUTMzone(149.0), 55);
        assert_eq!(Clip(5.0, 0.0, 3.0), 3.0);
        assert!(Clip(f64::NAN, 0.0, 3.0).is_nan());
        assert!((e0fn(0.0) - 1.0).abs() < 1e-15 && e3fn(0.0) == 0.0);
        assert!((RadiansToDegrees(DegreesToRadians(30.0)) - 30.0).abs() < 1e-12);
    }
}
