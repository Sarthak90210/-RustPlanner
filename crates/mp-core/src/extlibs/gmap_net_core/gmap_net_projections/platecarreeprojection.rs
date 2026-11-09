//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/PlateCarreeProjection.cs`
//!
//! Plate Carrée (literally, "plane square") projection:
//! PROJCS["WGS 84 / World Equidistant Cylindrical", ... SPHEROID["WGS_1984",6378137,298.257223563] ...]

#![allow(non_snake_case)]

use std::sync::OnceLock;

use crate::dotnet::cast_i64;
use crate::extlibs::gmap_net_core::gmap_net::pureprojection::{Clip, ProjectionCache, PureProjection};
use crate::extlibs::gmap_net_core::gmap_net::{GPoint, GSize, PointLatLng, RectLatLng};

const MIN_LATITUDE: f64 = -85.05112878;
const MAX_LATITUDE: f64 = 85.05112878;
const MIN_LONGITUDE: f64 = -180.0;
const MAX_LONGITUDE: f64 = 180.0;

/// `public class PlateCarreeProjection : PureProjection` (512x512 tiles)
#[derive(Debug, Default)]
pub struct PlateCarreeProjection {
    cache: ProjectionCache,
}

impl PlateCarreeProjection {
    /// `PlateCarreeProjection.Instance`
    pub fn Instance() -> &'static PlateCarreeProjection {
        static INSTANCE: OnceLock<PlateCarreeProjection> = OnceLock::new();
        INSTANCE.get_or_init(PlateCarreeProjection::default)
    }
}

impl PureProjection for PlateCarreeProjection {
    fn cache(&self) -> &ProjectionCache {
        &self.cache
    }

    fn Bounds(&self) -> RectLatLng {
        RectLatLng::from_ltrb(MIN_LONGITUDE, MAX_LATITUDE, MAX_LONGITUDE, MIN_LATITUDE)
    }

    fn TileSize(&self) -> GSize {
        GSize::new(512, 512)
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
        let s = self.GetTileMatrixSizePixel(zoom);
        let map_size_x = s.width() as f64;
        let scale = 360.0 / map_size_x;
        ret.set_y(cast_i64((90.0 - lat) / scale));
        ret.set_x(cast_i64((lng + 180.0) / scale));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let mut ret = PointLatLng::EMPTY;
        let s = self.GetTileMatrixSizePixel(zoom);
        let map_size_x = s.width() as f64;
        let scale = 360.0 / map_size_x;
        ret.set_lat(90.0 - (y as f64 * scale));
        ret.set_lng((x as f64 * scale) - 180.0);
        ret
    }

    fn GetTileMatrixMaxXY(&self, zoom: i32) -> GSize {
        let y = cast_i64(2f64.powf(zoom as f64));
        GSize::new(2i64.wrapping_mul(y).wrapping_sub(1), y.wrapping_sub(1))
    }

    fn GetTileMatrixMinXY(&self, _zoom: i32) -> GSize {
        GSize::new(0, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_by_one_tiles_at_zoom_zero() {
        let p = PlateCarreeProjection::Instance();
        assert_eq!(p.GetTileMatrixMaxXY(0), GSize::new(1, 0));
        assert_eq!(p.GetTileMatrixSizePixel(0), GSize::new(1024, 512));
        assert_eq!(p.FromLatLngToPixel(0.0, 0.0, 0), GPoint::new(512, 256));
        // latitude is clipped to the Mercator limits
        assert_eq!(p.FromLatLngToPixel(90.0, -180.0, 0), GPoint::new(0, 14));
        let ll = p.FromPixelToLatLng(512, 256, 0);
        assert_eq!((ll.lat(), ll.lng()), (0.0, 0.0));
    }
}
