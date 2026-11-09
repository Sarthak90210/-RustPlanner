//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/MercatorProjectionGCJ.cs`
//!
//! Spherical Mercator over GCJ-02 ("Mars") coordinates: points inside China are shifted with
//! the GCJ transform before projecting, and the inverse subtracts the shift once.

#![allow(non_snake_case)]

use std::sync::OnceLock;

use crate::dotnet::cast_i64;
use crate::extlibs::gmap_net_core::gmap_net::pureprojection::{Clip, ProjectionCache, PureProjection};
use crate::extlibs::gmap_net_core::gmap_net::{GPoint, GSize, PointLatLng, RectLatLng};

const MAX_LATITUDE: f64 = 85.05112878;
const MAX_LONGITUDE: f64 = 180.0;
const MIN_LATITUDE: f64 = -85.05112878;
const MIN_LONGITUDE: f64 = -180.0;

/// `public class MercatorProjectionGCJ : PureProjection`
#[derive(Debug)]
pub struct MercatorProjectionGCJ {
    cache: ProjectionCache,
    /// `internal double a`
    pub a: f64,
    /// `internal double ee`
    pub ee: f64,
}

impl Default for MercatorProjectionGCJ {
    fn default() -> Self {
        MercatorProjectionGCJ { cache: ProjectionCache::default(), a: 6378245.0, ee: 0.0066934216229659433 }
    }
}

impl MercatorProjectionGCJ {
    /// `MercatorProjectionGCJ.Instance`
    pub fn Instance() -> &'static MercatorProjectionGCJ {
        static INSTANCE: OnceLock<MercatorProjectionGCJ> = OnceLock::new();
        INSTANCE.get_or_init(MercatorProjectionGCJ::default)
    }

    fn outOfChina(lat: f64, lon: f64) -> bool {
        !(lon >= 72.004 && lon <= 137.8347 && lat >= 0.8293 && lat <= 55.8271)
    }

    /// `transform(wgLat, wgLon, latlng)`: returns latlng[0], latlng[1]
    fn transform(&self, wgLat: f64, wgLon: f64) -> (f64, f64) {
        if Self::outOfChina(wgLat, wgLon) {
            (wgLat, wgLon)
        } else {
            let mut num = self.transformLat(wgLon - 105.0, wgLat - 35.0);
            let mut num2 = self.transformLon(wgLon - 105.0, wgLat - 35.0);
            let a = (wgLat / 180.0) * 3.1415926535897931;
            let mut d = a.sin();
            d = 1.0 - ((self.ee * d) * d);
            let num5 = d.sqrt();
            num = (num * 180.0) / (((self.a * (1.0 - self.ee)) / (d * num5)) * 3.1415926535897931);
            num2 = (num2 * 180.0) / (((self.a / num5) * a.cos()) * 3.1415926535897931);
            (wgLat + num, wgLon + num2)
        }
    }

    fn transformLat(&self, x: f64, y: f64) -> f64 {
        let mut num = ((((-100.0 + (2.0 * x)) + (3.0 * y)) + ((0.2 * y) * y)) + ((0.1 * x) * y)) + (0.2 * x.abs().sqrt());
        num += (((20.0 * ((6.0 * x) * 3.1415926535897931).sin()) + (20.0 * ((2.0 * x) * 3.1415926535897931).sin())) * 2.0) / 3.0;
        num += (((20.0 * (y * 3.1415926535897931).sin()) + (40.0 * ((y / 3.0) * 3.1415926535897931).sin())) * 2.0) / 3.0;
        num + ((((160.0 * ((y / 12.0) * 3.1415926535897931).sin()) + (320.0 * ((y * 3.1415926535897931) / 30.0).sin())) * 2.0) / 3.0)
    }

    fn transformLon(&self, x: f64, y: f64) -> f64 {
        let mut num = ((((300.0 + x) + (2.0 * y)) + ((0.1 * x) * x)) + ((0.1 * x) * y)) + (0.1 * x.abs().sqrt());
        num += (((20.0 * ((6.0 * x) * 3.1415926535897931).sin()) + (20.0 * ((2.0 * x) * 3.1415926535897931).sin())) * 2.0) / 3.0;
        num += (((20.0 * (x * 3.1415926535897931).sin()) + (40.0 * ((x / 3.0) * 3.1415926535897931).sin())) * 2.0) / 3.0;
        num + ((((150.0 * ((x / 12.0) * 3.1415926535897931).sin()) + (300.0 * ((x / 30.0) * 3.1415926535897931).sin())) * 2.0) / 3.0)
    }
}

impl PureProjection for MercatorProjectionGCJ {
    fn cache(&self) -> &ProjectionCache {
        &self.cache
    }

    fn FromLatLngToPixel(&self, lat: f64, lng: f64, zoom: i32) -> GPoint {
        let (lat, lng) = self.transform(lat, lng);
        let mut empty = GPoint::EMPTY;
        let lat = Clip(lat, MIN_LATITUDE, MAX_LATITUDE);
        let lng = Clip(lng, MIN_LONGITUDE, MAX_LONGITUDE);
        let num = (lng + 180.0) / 360.0;
        let num2 = ((lat * 3.1415926535897931) / 180.0).sin();
        let num3 = 0.5 - (((1.0 + num2) / (1.0 - num2)).ln() / 12.566370614359172);
        let tile_matrix_size_pixel = self.GetTileMatrixSizePixel(zoom);
        let width = tile_matrix_size_pixel.width();
        let height = tile_matrix_size_pixel.height();
        empty.set_x(cast_i64(Clip((num * width as f64) + 0.5, 0.0, width.wrapping_sub(1) as f64)));
        empty.set_y(cast_i64(Clip((num3 * height as f64) + 0.5, 0.0, height.wrapping_sub(1) as f64)));
        empty
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let mut empty = PointLatLng::EMPTY;
        let tile_matrix_size_pixel = self.GetTileMatrixSizePixel(zoom);
        let width = tile_matrix_size_pixel.width() as f64;
        let height = tile_matrix_size_pixel.height() as f64;
        let num3 = (Clip(x as f64, 0.0, width - 1.0) / width) - 0.5;
        let num4 = 0.5 - (Clip(y as f64, 0.0, height - 1.0) / height);
        empty.set_lat(90.0 - ((360.0 * ((-num4 * 2.0) * 3.1415926535897931).exp().atan()) / 3.1415926535897931));
        empty.set_lng(360.0 * num3);
        let (lat2, lng2) = self.transform(empty.lat(), empty.lng());
        empty.set_lat(empty.lat() - (lat2 - empty.lat()));
        empty.set_lng(empty.lng() - (lng2 - empty.lng()));
        empty
    }

    fn GetTileMatrixMaxXY(&self, zoom: i32) -> GSize {
        let num = 1i32.wrapping_shl(zoom as u32) as i64;
        GSize::new(num - 1, num - 1)
    }

    fn GetTileMatrixMinXY(&self, _zoom: i32) -> GSize {
        GSize::new(0, 0)
    }

    fn Axis(&self) -> f64 {
        6378137.0
    }

    fn Bounds(&self) -> RectLatLng {
        RectLatLng::from_ltrb(MIN_LONGITUDE, MAX_LATITUDE, MAX_LONGITUDE, MIN_LATITUDE)
    }

    fn Flattening(&self) -> f64 {
        0.0033528106647474805
    }

    fn TileSize(&self) -> GSize {
        GSize::new(0x100, 0x100)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extlibs::gmap_net_core::gmap_net_projections::MercatorProjection;

    #[test]
    fn shifts_only_inside_china() {
        let g = MercatorProjectionGCJ::Instance();
        let m = MercatorProjection::Instance();
        // outside China it is plain Mercator
        assert_eq!(g.FromLatLngToPixel(51.5, -0.12, 12), m.FromLatLngToPixel(51.5, -0.12, 12));
        // Beijing moves by a few hundred metres (about 0.6 m a pixel at zoom 16)
        let a = g.FromLatLngToPixel(39.9, 116.4, 16);
        let b = m.FromLatLngToPixel(39.9, 116.4, 16);
        assert_ne!(a, b);
        assert!((a.x() - b.x()).abs() < 1000 && (a.y() - b.y()).abs() < 1000);
        let back = g.FromPixelToLatLng(a.x(), a.y(), 16);
        assert!((back.lat() - 39.9).abs() < 1e-3 && (back.lng() - 116.4).abs() < 1e-3);
    }
}
