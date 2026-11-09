//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/EPSG3995Projection.cs`
//!
//! EPSG:3995 - WGS 84 / Arctic Polar Stereographic. Standard parallel: 71N, central
//! meridian: 0. The tile grid matches the ArcGIS Arctic Bathymetry Basemap (256x256 tiles).
//! `(1 << zoom)` is an `int` shift (count masked to 5 bits), widened where C# widens it.

#![allow(non_snake_case, non_upper_case_globals)]

use std::sync::{LazyLock, OnceLock};

use crate::dotnet::cast_i64;
use crate::extlibs::gmap_net_core::gmap_net::pureprojection::{Clip, DegreesToRadians, ProjectionCache, PureProjection, RadiansToDegrees, HALF_PI};
use crate::extlibs::gmap_net_core::gmap_net::{GPoint, GSize, PointLatLng, RectLatLng};

// WGS84 ellipsoid
const SemiMajorAxis: f64 = 6378137.0;
const InverseFlattening: f64 = 298.257223563;
const flat: f64 = 1.0 / InverseFlattening;
static e2: LazyLock<f64> = LazyLock::new(|| 2.0 * flat - flat * flat);
static ecc: LazyLock<f64> = LazyLock::new(|| e2.sqrt());

// EPSG:3995 parameters
static phi_c: LazyLock<f64> = LazyLock::new(|| 71.0 * std::f64::consts::PI / 180.0); // standard parallel
const lambda_0: f64 = 0.0; // central meridian in radians

// ArcGIS tile grid parameters
const OriginX: f64 = -30636100.0;
const OriginY: f64 = 30636100.0;
const BaseResolution: f64 = 67733.46880027094; // meters/pixel at zoom 0

// Geographic bounds
const MinLatitude: f64 = 30.0;
const MaxLatitude: f64 = 90.0;
const MinLongitude: f64 = -180.0;
const MaxLongitude: f64 = 180.0;

/// The static constructor's pre-computed values for the standard parallel: (m_c, t_c).
static STATIC: LazyLock<(f64, f64)> = LazyLock::new(|| {
    let sin_phi_c = phi_c.sin();
    let m_c = phi_c.cos() / (1.0 - *e2 * sin_phi_c * sin_phi_c).sqrt();
    let t_c = ComputeT(*phi_c);
    (m_c, t_c)
});

fn ComputeT(phi: f64) -> f64 {
    let sin_phi = phi.sin();
    (std::f64::consts::PI / 4.0 - phi / 2.0).tan() / ((1.0 - *ecc * sin_phi) / (1.0 + *ecc * sin_phi)).powf(*ecc / 2.0)
}

/// `1 << zoom` as C# evaluates it (`int`)
fn shl(zoom: i32) -> i32 {
    1i32.wrapping_shl(zoom as u32)
}

/// `public class EPSG3995Projection : PureProjection`
#[derive(Debug, Default)]
pub struct EPSG3995Projection {
    cache: ProjectionCache,
}

impl EPSG3995Projection {
    /// `EPSG3995Projection.Instance`
    pub fn Instance() -> &'static EPSG3995Projection {
        static INSTANCE: OnceLock<EPSG3995Projection> = OnceLock::new();
        INSTANCE.get_or_init(EPSG3995Projection::default)
    }
}

impl PureProjection for EPSG3995Projection {
    fn cache(&self) -> &ProjectionCache {
        &self.cache
    }

    fn Bounds(&self) -> RectLatLng {
        RectLatLng::from_ltrb(MinLongitude, MaxLatitude, MaxLongitude, MinLatitude)
    }

    fn TileSize(&self) -> GSize {
        GSize::new(256, 256)
    }

    fn Axis(&self) -> f64 {
        SemiMajorAxis
    }

    fn Flattening(&self) -> f64 {
        flat
    }

    fn FromLatLngToPixel(&self, lat: f64, lng: f64, zoom: i32) -> GPoint {
        let (m_c, t_c) = *STATIC;
        let mut ret = GPoint::EMPTY;
        let lat = Clip(lat, MinLatitude, MaxLatitude);
        let lng = Clip(lng, MinLongitude, MaxLongitude);
        let phi = DegreesToRadians(lat);
        let lambda = DegreesToRadians(lng);

        // Forward polar stereographic: lat/lng -> projected meters
        let t = ComputeT(phi);
        let rho = SemiMajorAxis * m_c * t / t_c;
        let x = rho * (lambda - lambda_0).sin();
        let y = -rho * (lambda - lambda_0).cos();

        // Meters -> pixels using ArcGIS tile grid
        let resolution = BaseResolution / shl(zoom) as f64;
        let pixel_x = (x - OriginX) / resolution;
        let pixel_y = (OriginY - y) / resolution;

        let total_pixels = 3i64.wrapping_mul(shl(zoom) as i64).wrapping_mul(256);
        ret.set_x(cast_i64(Clip(pixel_x + 0.5, 0.0, total_pixels.wrapping_sub(1) as f64)));
        ret.set_y(cast_i64(Clip(pixel_y + 0.5, 0.0, total_pixels.wrapping_sub(1) as f64)));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let (m_c, t_c) = *STATIC;
        let mut ret = PointLatLng::EMPTY;

        // Pixels -> meters
        let resolution = BaseResolution / shl(zoom) as f64;
        let mx = x as f64 * resolution + OriginX;
        let my = OriginY - y as f64 * resolution;

        // Inverse polar stereographic: meters -> lat/lng
        let rho = (mx * mx + my * my).sqrt();
        if rho < 1.0 {
            ret.set_lat(90.0);
            ret.set_lng(0.0);
            return ret;
        }

        let t = rho * t_c / (SemiMajorAxis * m_c);

        // Iterative inverse for latitude
        let mut phi = HALF_PI - 2.0 * t.atan();
        for _ in 0..15 {
            let sin_phi = phi.sin();
            let new_phi = HALF_PI - 2.0 * (t * ((1.0 - *ecc * sin_phi) / (1.0 + *ecc * sin_phi)).powf(*ecc / 2.0)).atan();
            if (new_phi - phi).abs() < 1e-12 {
                break;
            }
            phi = new_phi;
        }

        // Longitude
        let lambda = lambda_0 + mx.atan2(-my);

        ret.set_lat(Clip(RadiansToDegrees(phi), MinLatitude, MaxLatitude));
        ret.set_lng(RadiansToDegrees(lambda));

        // Normalize longitude to [-180, 180]
        while ret.lng() > 180.0 {
            ret.set_lng(ret.lng() - 360.0);
        }
        while ret.lng() < -180.0 {
            ret.set_lng(ret.lng() + 360.0);
        }

        ret
    }

    fn GetTileMatrixMinXY(&self, _zoom: i32) -> GSize {
        GSize::new(0, 0)
    }

    fn GetTileMatrixMaxXY(&self, zoom: i32) -> GSize {
        // 3 * 2^zoom tiles per axis covers the data extent with margin
        let xy = 3i64.wrapping_mul(shl(zoom) as i64);
        GSize::new(xy - 1, xy - 1)
    }

    fn GetGroundResolution(&self, zoom: i32, _latitude: f64) -> f64 {
        BaseResolution / shl(zoom) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pole_and_round_trip() {
        let p = EPSG3995Projection::Instance();
        // the pole: (0 - OriginX) / BaseResolution = 452.3, +0.5 and truncated
        assert_eq!(p.FromLatLngToPixel(90.0, 0.0, 0), GPoint::new(452, 452));
        assert_eq!(p.GetTileMatrixMaxXY(2), GSize::new(11, 11));
        let px = p.FromLatLngToPixel(75.0, 30.0, 12);
        let back = p.FromPixelToLatLng(px.x(), px.y(), 12);
        assert!((back.lat() - 75.0).abs() < 0.01 && (back.lng() - 30.0).abs() < 0.01);
        let pole = p.FromPixelToLatLng(452, 452, 0);
        assert!(pole.lat() > 89.5);
        // latitude below the bounds is clipped to 30
        assert_eq!(p.FromLatLngToPixel(10.0, 0.0, 0), p.FromLatLngToPixel(30.0, 0.0, 0));
        assert_eq!(p.GetGroundResolution(1, 0.0), BaseResolution / 2.0);
    }
}
