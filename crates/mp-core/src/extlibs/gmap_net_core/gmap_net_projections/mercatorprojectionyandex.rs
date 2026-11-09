//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/MercatorProjectionYandex.cs`
//!
//! The ellipsoidal Mercator Yandex tiles use. `class MercatorProjectionYandex` is internal
//! in C#.

#![allow(non_snake_case)]

use std::sync::OnceLock;

use crate::dotnet::cast_i64;
use crate::extlibs::gmap_net_core::gmap_net::pureprojection::{Clip, ProjectionCache, PureProjection};
use crate::extlibs::gmap_net_core::gmap_net::{GPoint, GSize, PointLatLng, RectLatLng};

const MIN_LATITUDE: f64 = -85.05112878;
const MAX_LATITUDE: f64 = 85.05112878;
const MIN_LONGITUDE: f64 = -177.0;
const MAX_LONGITUDE: f64 = 177.0;
const RAD_DEG: f64 = 180.0 / std::f64::consts::PI;
const DEG_RAD: f64 = std::f64::consts::PI / 180.0;
const MATH_PI_DIV4: f64 = std::f64::consts::PI / 4.0;

/// `class MercatorProjectionYandex : PureProjection`
#[derive(Debug, Default)]
pub struct MercatorProjectionYandex {
    cache: ProjectionCache,
}

impl MercatorProjectionYandex {
    /// `MercatorProjectionYandex.Instance`
    pub fn Instance() -> &'static MercatorProjectionYandex {
        static INSTANCE: OnceLock<MercatorProjectionYandex> = OnceLock::new();
        INSTANCE.get_or_init(MercatorProjectionYandex::default)
    }
}

impl PureProjection for MercatorProjectionYandex {
    fn cache(&self) -> &ProjectionCache {
        &self.cache
    }

    fn Bounds(&self) -> RectLatLng {
        RectLatLng::from_ltrb(MIN_LONGITUDE, MAX_LATITUDE, MAX_LONGITUDE, MIN_LATITUDE)
    }

    fn TileSize(&self) -> GSize {
        GSize::new(256, 256)
    }

    fn Axis(&self) -> f64 {
        6356752.3142
    }

    fn Flattening(&self) -> f64 {
        1.0 / 298.257223563
    }

    fn FromLatLngToPixel(&self, lat: f64, lng: f64, zoom: i32) -> GPoint {
        let lat = Clip(lat, MIN_LATITUDE, MAX_LATITUDE);
        let lng = Clip(lng, MIN_LONGITUDE, MAX_LONGITUDE);

        let r_lon = lng * DEG_RAD; // Math.PI / 180;
        let r_lat = lat * DEG_RAD; // Math.PI / 180;

        let a = 6378137.0;
        let k = 0.0818191908426;

        let z = (MATH_PI_DIV4 + r_lat / 2.0).tan() / (MATH_PI_DIV4 + (k * r_lat.sin()).asin() / 2.0).tan().powf(k);
        let z1 = 2f64.powf(23i32.wrapping_sub(zoom) as f64);

        let dx = (20037508.342789 + a * r_lon) * 53.5865938 / z1;
        let dy = (20037508.342789 - a * z.ln()) * 53.5865938 / z1;

        let mut ret = GPoint::EMPTY;
        ret.set_x(cast_i64(dx));
        ret.set_y(cast_i64(dy));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        // the C# reads the matrix size into unused locals
        let _s = self.GetTileMatrixSizePixel(zoom);

        let a = 6378137.0;
        let c1 = 0.00335655146887969;
        let c2 = 0.00000657187271079536;
        let c3 = 0.00000001764564338702;
        let c4 = 0.00000000005328478445;
        let z1 = 23i32.wrapping_sub(zoom) as f64;
        let merc_x = (x as f64 * 2f64.powf(z1)) / 53.5865938 - 20037508.342789;
        let merc_y = 20037508.342789 - (y as f64 * 2f64.powf(z1)) / 53.5865938;

        let g = std::f64::consts::PI / 2.0 - 2.0 * (1.0 / (merc_y / a).exp()).atan();
        let z = g + c1 * (2.0 * g).sin() + c2 * (4.0 * g).sin() + c3 * (6.0 * g).sin() + c4 * (8.0 * g).sin();

        let mut ret = PointLatLng::EMPTY;
        ret.set_lat(z * RAD_DEG);
        ret.set_lng(merc_x / a * RAD_DEG);
        ret
    }

    fn GetTileMatrixMinXY(&self, _zoom: i32) -> GSize {
        GSize::new(0, 0)
    }

    fn GetTileMatrixMaxXY(&self, zoom: i32) -> GSize {
        let xy = 1i32.wrapping_shl(zoom as u32) as i64;
        GSize::new(xy - 1, xy - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ellipsoidal_mercator() {
        let p = MercatorProjectionYandex::Instance();
        let c = p.FromLatLngToPixel(0.0, 0.0, 1);
        assert_eq!(c, GPoint::new(255, 255), "20037508.342789 * 53.5865938 / 2^22 truncates to 255");
        let px = p.FromLatLngToPixel(55.75, 37.62, 10);
        let back = p.FromPixelToLatLng(px.x(), px.y(), 10);
        assert!((back.lat() - 55.75).abs() < 0.01 && (back.lng() - 37.62).abs() < 0.01);
        // longitude is clipped to +-177
        assert_eq!(p.FromLatLngToPixel(0.0, 180.0, 0), p.FromLatLngToPixel(0.0, 177.0, 0));
        assert_eq!(p.Axis(), 6356752.3142);
    }
}
