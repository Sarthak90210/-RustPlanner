//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/NoProjection.cs`
//!
//! The C# class lives in namespace `GMap.NET.Core.GMap.NET.Projections`. Its `Instance` is a
//! settable static property typed `PureProjection` (null allowed), so it is a lock around an
//! optional projection reference.

#![allow(non_snake_case)]

use std::sync::{LazyLock, RwLock};

use crate::dotnet::cast_i64;
use crate::extlibs::gmap_net_core::gmap_net::pureprojection::{Clip, ProjectionCache, PureProjection};
use crate::extlibs::gmap_net_core::gmap_net::{GPoint, GSize, PointLatLng};

/// `public class NoProjection : PureProjection`: equirectangular over the whole globe.
#[derive(Debug, Default)]
pub struct NoProjection {
    cache: ProjectionCache,
}

static INSTANCE: LazyLock<RwLock<Option<&'static dyn PureProjection>>> = LazyLock::new(|| RwLock::new(Some(Box::leak(Box::new(NoProjection::default())))));

impl NoProjection {
    /// `public static PureProjection Instance { get; }`
    pub fn Instance() -> Option<&'static dyn PureProjection> {
        *INSTANCE.read().unwrap_or_else(|e| e.into_inner())
    }

    /// `public static PureProjection Instance { set; }`
    pub fn set_Instance(value: Option<&'static dyn PureProjection>) {
        *INSTANCE.write().unwrap_or_else(|e| e.into_inner()) = value;
    }
}

impl PureProjection for NoProjection {
    fn cache(&self) -> &ProjectionCache {
        &self.cache
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
        let x = (lng + 180.0) / 360.0;
        let y = (90.0 - lat) / 180.0;
        let s = self.GetTileMatrixSizePixel(zoom);
        let map_size_x = s.width();
        let map_size_y = s.height();
        ret.set_x(cast_i64(Clip(x * map_size_x as f64 + 0.5, 0.0, map_size_x.wrapping_sub(1) as f64)));
        ret.set_y(cast_i64(Clip(y * map_size_y as f64 + 0.5, 0.0, map_size_y.wrapping_sub(1) as f64)));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let mut ret = PointLatLng::EMPTY;
        let s = self.GetTileMatrixSizePixel(zoom);
        let map_size_x = s.width() as f64;
        let map_size_y = s.height() as f64;
        let xx = Clip(x as f64, 0.0, map_size_x - 1.0) / map_size_x;
        let yy = Clip(y as f64, 0.0, map_size_y - 1.0) / map_size_y;
        ret.set_lat(90.0 - 180.0 * yy);
        ret.set_lng(360.0 * xx - 180.0);
        ret
    }

    fn GetTileMatrixMaxXY(&self, zoom: i32) -> GSize {
        // (1 << zoom) is an int shift, then widened
        let xy = 1i32.wrapping_shl(zoom as u32) as i64;
        GSize::new(xy - 1, xy - 1)
    }

    fn GetTileMatrixMinXY(&self, _zoom: i32) -> GSize {
        GSize::new(0, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn equirectangular_and_settable_instance() {
        let p = NoProjection::Instance().unwrap();
        assert_eq!(p.FromLatLngToPixel(0.0, 0.0, 1), GPoint::new(256, 256));
        assert_eq!(p.FromLatLngToPixel(90.0, -180.0, 1), GPoint::new(0, 0));
        // not clipped to the map: south of -90 lands on the last pixel row
        assert_eq!(p.FromLatLngToPixel(-95.0, 0.0, 0).y(), 255);
        let ll = p.FromPixelToLatLng(128, 128, 0);
        assert_eq!((ll.lat(), ll.lng()), (0.0, 0.0));
        let other = Box::leak(Box::new(NoProjection::default()));
        NoProjection::set_Instance(Some(other));
        assert!(std::ptr::addr_eq(NoProjection::Instance().unwrap(), other as &dyn PureProjection));
    }
}
