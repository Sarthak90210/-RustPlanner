//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/EPSG5936Projection.cs`
//!
//! EPSG:5936 - WGS 84 / EPSG Alaska Polar Stereographic. Polar Stereographic (Variant A):
//! natural origin at the North Pole, scale factor 0.994, central meridian -150, false
//! easting/northing 2,000,000. The tile grid matches the Esri Polar Arctic services (256x256
//! tiles). `(1 << zoom)` is an `int` shift; `1L << zoom` a `long` one.

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

// EPSG:5936 parameters (Variant A - scale factor at pole)
const k0: f64 = 0.994;
static lambda_0: LazyLock<f64> = LazyLock::new(|| -150.0 * std::f64::consts::PI / 180.0);
const FalseEasting: f64 = 2000000.0;
const FalseNorthing: f64 = 2000000.0;

/// Pre-computed factor F = sqrt((1+e)^(1+e) * (1-e)^(1-e)) (the static constructor)
static F: LazyLock<f64> = LazyLock::new(|| ((1.0 + *ecc).powf(1.0 + *ecc) * (1.0 - *ecc).powf(1.0 - *ecc)).sqrt());

// Esri Polar tiling scheme
const OriginX: f64 = -28567784.109255;
const OriginY: f64 = 32567784.109255;
const BaseResolution: f64 = 238810.81335399998;

// Geographic bounds
const MinLatitude: f64 = 50.0;
const MaxLatitude: f64 = 90.0;
const MinLongitude: f64 = -180.0;
const MaxLongitude: f64 = 180.0;

fn ComputeT(phi: f64) -> f64 {
    let sin_phi = phi.sin();
    (std::f64::consts::PI / 4.0 - phi / 2.0).tan() / ((1.0 - *ecc * sin_phi) / (1.0 + *ecc * sin_phi)).powf(*ecc / 2.0)
}

/// `1 << zoom` as C# evaluates it (`int`)
fn shl(zoom: i32) -> i32 {
    1i32.wrapping_shl(zoom as u32)
}

/// `public class EPSG5936Projection : PureProjection`
#[derive(Debug, Default)]
pub struct EPSG5936Projection {
    cache: ProjectionCache,
}

impl EPSG5936Projection {
    /// `EPSG5936Projection.Instance`
    pub fn Instance() -> &'static EPSG5936Projection {
        static INSTANCE: OnceLock<EPSG5936Projection> = OnceLock::new();
        INSTANCE.get_or_init(EPSG5936Projection::default)
    }
}

impl PureProjection for EPSG5936Projection {
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
        let mut ret = GPoint::EMPTY;
        let lat = Clip(lat, MinLatitude, MaxLatitude);
        let lng = Clip(lng, MinLongitude, MaxLongitude);
        let phi = DegreesToRadians(lat);
        let lambda = DegreesToRadians(lng);

        // Forward polar stereographic (Variant A at pole)
        let t = ComputeT(phi);
        let rho = 2.0 * SemiMajorAxis * k0 * t / *F;
        let e = FalseEasting + rho * (lambda - *lambda_0).sin();
        let n = FalseNorthing - rho * (lambda - *lambda_0).cos();

        // Projected meters -> pixels using Esri tile grid
        let resolution = BaseResolution / shl(zoom) as f64;
        let pixel_x = (e - OriginX) / resolution;
        let pixel_y = (OriginY - n) / resolution;

        let total_pixels = 1i64.wrapping_shl(zoom as u32).wrapping_mul(256);
        ret.set_x(cast_i64(Clip(pixel_x + 0.5, 0.0, total_pixels.wrapping_sub(1) as f64)));
        ret.set_y(cast_i64(Clip(pixel_y + 0.5, 0.0, total_pixels.wrapping_sub(1) as f64)));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let mut ret = PointLatLng::EMPTY;

        // Pixels -> projected meters
        let resolution = BaseResolution / shl(zoom) as f64;
        let e = x as f64 * resolution + OriginX;
        let n = OriginY - y as f64 * resolution;

        // Inverse polar stereographic
        let dx = e - FalseEasting;
        let dy = FalseNorthing - n;
        let rho = (dx * dx + dy * dy).sqrt();
        if rho < 1.0 {
            ret.set_lat(90.0);
            ret.set_lng(0.0);
            return ret;
        }

        let t = rho * *F / (2.0 * SemiMajorAxis * k0);

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
        let lambda = *lambda_0 + dx.atan2(dy);

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
        let xy = 1i64.wrapping_shl(zoom as u32);
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
    fn round_trip_and_bounds() {
        let p = EPSG5936Projection::Instance();
        // zoom 12: a pixel is small enough for a 0.01 degree round trip
        let px = p.FromLatLngToPixel(64.8, -147.7, 12);
        let back = p.FromPixelToLatLng(px.x(), px.y(), 12);
        assert!((back.lat() - 64.8).abs() < 0.01 && (back.lng() + 147.7).abs() < 0.01);
        assert_eq!(p.GetTileMatrixMaxXY(3), GSize::new(7, 7));
        assert_eq!(p.FromLatLngToPixel(20.0, 0.0, 2), p.FromLatLngToPixel(50.0, 0.0, 2));
        let max = p.FromLatLngToPixel(50.0, 30.0, 0);
        assert!(max.x() <= 255 && max.y() <= 255);
    }
}
