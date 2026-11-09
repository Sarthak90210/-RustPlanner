//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/MercatorProjection.cs`
//!
//! The Mercator projection (World_Mercator on the WGS84 spheroid, 256 px tiles).

#![allow(non_snake_case)]

use std::sync::OnceLock;

use crate::dotnet::cast_i64;
use crate::extlibs::gmap_net_core::gmap_net::gpoint::GPoint;
use crate::extlibs::gmap_net_core::gmap_net::gsize::GSize;
use crate::extlibs::gmap_net_core::gmap_net::pureprojection::{Clip, ProjectionCache, PureProjection};
use crate::extlibs::gmap_net_core::gmap_net::rectlatlng::RectLatLng;
use crate::extlibs::gmap_net_core::gmap_net::PointLatLng;

const MIN_LATITUDE: f64 = -85.05112878;
const MAX_LATITUDE: f64 = 85.05112878;
const MIN_LONGITUDE: f64 = -180.0;
const MAX_LONGITUDE: f64 = 180.0;

/// `public class MercatorProjection : PureProjection`
#[derive(Debug, Default)]
pub struct MercatorProjection {
    cache: ProjectionCache,
}

impl MercatorProjection {
    /// `MercatorProjection.Instance`
    pub fn Instance() -> &'static MercatorProjection {
        static INSTANCE: OnceLock<MercatorProjection> = OnceLock::new();
        INSTANCE.get_or_init(MercatorProjection::default)
    }
}

impl PureProjection for MercatorProjection {
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
        6378137.0
    }

    fn Flattening(&self) -> f64 {
        1.0 / 298.257223563
    }

    fn FromLatLngToPixel(&self, lat: f64, lng: f64, zoom: i32) -> GPoint {
        let mut ret = GPoint::EMPTY;
        let lat = Clip(lat, MIN_LATITUDE, MAX_LATITUDE);
        let lng = Clip(lng, MIN_LONGITUDE, MAX_LONGITUDE);
        let x = (lng + 180.0) / 360.0;
        let sin_latitude = (lat * std::f64::consts::PI / 180.0).sin();
        let y = 0.5 - ((1.0 + sin_latitude) / (1.0 - sin_latitude)).ln() / (4.0 * std::f64::consts::PI);
        let s = self.GetTileMatrixSizePixel(zoom);
        let map_size_x = s.width();
        let map_size_y = s.height();
        ret.set_x(cast_i64(Clip(x * map_size_x as f64 + 0.5, 0.0, (map_size_x - 1) as f64)));
        ret.set_y(cast_i64(Clip(y * map_size_y as f64 + 0.5, 0.0, (map_size_y - 1) as f64)));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let mut ret = PointLatLng::EMPTY;
        let s = self.GetTileMatrixSizePixel(zoom);
        let map_size_x = s.width() as f64;
        let map_size_y = s.height() as f64;
        let xx = (Clip(x as f64, 0.0, map_size_x - 1.0) / map_size_x) - 0.5;
        let yy = 0.5 - (Clip(y as f64, 0.0, map_size_y - 1.0) / map_size_y);
        ret.set_lat(90.0 - 360.0 * (-yy * 2.0 * std::f64::consts::PI).exp().atan() / std::f64::consts::PI);
        ret.set_lng(360.0 * xx);
        ret
    }

    fn GetTileMatrixMinXY(&self, _zoom: i32) -> GSize {
        GSize::new(0, 0)
    }

    fn GetTileMatrixMaxXY(&self, zoom: i32) -> GSize {
        // (1 << zoom) is an int shift (the count masked to 5 bits), then widened
        let xy = 1i32.wrapping_shl(zoom as u32) as i64;
        GSize::new(xy - 1, xy - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pixels_round_trip() {
        let m = MercatorProjection::Instance();
        assert_eq!(m.FromLatLngToPixel(0.0, 0.0, 2), GPoint::new(512, 512));
        assert_eq!(m.FromLatLngToPixel(90.0, -180.0, 2), GPoint::new(0, 0));
        let p = m.FromPixelToLatLng(512, 512, 2);
        assert!(p.lat().abs() < 1e-9 && p.lng().abs() < 1e-9);
        assert_eq!(m.GetTileMatrixSizePixel(3), GSize::new(2048, 2048));
        let d = m.GetDistance(PointLatLng::new(0.0, 0.0), PointLatLng::new(0.0, 1.0));
        assert!((d - 111.319_490_793).abs() < 1e-6);
        let c = m.FromLatLngToPixel_cached(PointLatLng::new(1.0, 2.0), 5, true).unwrap();
        assert_eq!(m.FromLatLngToPixel_cached(PointLatLng::new(1.0, 2.0), 5, true).unwrap(), c);
        assert!(m.FromLatLngToPixel_cached(PointLatLng::new(1.0, 2.0), 33, true).is_err());
    }
}
