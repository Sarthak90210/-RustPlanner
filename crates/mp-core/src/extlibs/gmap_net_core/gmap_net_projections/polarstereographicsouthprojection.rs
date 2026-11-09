//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/PolarStereographicSouthProjection.cs`
//!
//! EPSG:3031 - WGS 84 / Antarctic Polar Stereographic. Standard parallel: -71S, central
//! meridian: 0. Used by the NASA GIBS Antarctic tile service (512x512 tiles).
//! `1L << (zoom + 1)` is a `long` shift (count masked to 6 bits).

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

// EPSG:3031 parameters (absolute values used in computation)
const phi_c_abs: f64 = 71.0; // |standard parallel| in degrees
static phi_c_rad: LazyLock<f64> = LazyLock::new(|| phi_c_abs * std::f64::consts::PI / 180.0);
const lambda_0: f64 = 0.0; // central meridian in degrees
static lambda_0_rad: LazyLock<f64> = LazyLock::new(|| lambda_0 * std::f64::consts::PI / 180.0);

// GIBS tile matrix (same extent as EPSG:3413)
const FullExtent: f64 = 8388608.0;
const OriginX: f64 = -4194304.0;
const OriginY: f64 = 4194304.0;

// Geographic bounds (southern hemisphere)
const MinLatitude: f64 = -90.0;
const MaxLatitude: f64 = -30.0;
const MinLongitude: f64 = -180.0;
const MaxLongitude: f64 = 180.0;

/// The static constructor: (m_c, t_c)
static STATIC: LazyLock<(f64, f64)> = LazyLock::new(|| {
    let sin_phi_c = phi_c_rad.sin();
    let m_c = phi_c_rad.cos() / (1.0 - *e2 * sin_phi_c * sin_phi_c).sqrt();
    let t_c = ComputeT(*phi_c_rad);
    (m_c, t_c)
});

/// Conformal latitude parameter for north pole formulation. For south pole, the latitude
/// is mirrored to positive before calling this.
fn ComputeT(phi: f64) -> f64 {
    let sin_phi = phi.sin();
    (std::f64::consts::PI / 4.0 - phi / 2.0).tan() / ((1.0 - *ecc * sin_phi) / (1.0 + *ecc * sin_phi)).powf(*ecc / 2.0)
}

/// `(1L << (zoom + 1)) * 512`
fn total_pixels(zoom: i32) -> i64 {
    1i64.wrapping_shl(zoom.wrapping_add(1) as u32).wrapping_mul(512)
}

/// `public class PolarStereographicSouthProjection : PureProjection`
#[derive(Debug, Default)]
pub struct PolarStereographicSouthProjection {
    cache: ProjectionCache,
}

impl PolarStereographicSouthProjection {
    /// `PolarStereographicSouthProjection.Instance`
    pub fn Instance() -> &'static PolarStereographicSouthProjection {
        static INSTANCE: OnceLock<PolarStereographicSouthProjection> = OnceLock::new();
        INSTANCE.get_or_init(PolarStereographicSouthProjection::default)
    }
}

impl PureProjection for PolarStereographicSouthProjection {
    fn cache(&self) -> &ProjectionCache {
        &self.cache
    }

    fn Bounds(&self) -> RectLatLng {
        RectLatLng::from_ltrb(MinLongitude, MaxLatitude, MaxLongitude, MinLatitude)
    }

    fn TileSize(&self) -> GSize {
        GSize::new(512, 512)
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

        // Mirror latitude to positive for south pole computation
        let phi_abs = DegreesToRadians(-lat);
        let lambda = DegreesToRadians(lng);

        // Forward polar stereographic using mirrored latitude
        let t = ComputeT(phi_abs);
        let rho = SemiMajorAxis * m_c * t / t_c;

        // South pole: y = +rho*cos (not -rho*cos like north)
        let x = rho * (lambda - *lambda_0_rad).sin();
        let y = rho * (lambda - *lambda_0_rad).cos();

        // Meters -> pixels
        let total = total_pixels(zoom);
        let pixel_x = (x - OriginX) / FullExtent * total as f64;
        let pixel_y = (OriginY - y) / FullExtent * total as f64;

        ret.set_x(cast_i64(Clip(pixel_x + 0.5, 0.0, total.wrapping_sub(1) as f64)));
        ret.set_y(cast_i64(Clip(pixel_y + 0.5, 0.0, total.wrapping_sub(1) as f64)));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let (m_c, t_c) = *STATIC;
        let mut ret = PointLatLng::EMPTY;
        let total = total_pixels(zoom);

        // Pixels -> meters
        let mx = x as f64 / total as f64 * FullExtent + OriginX;
        let my = OriginY - y as f64 / total as f64 * FullExtent;

        // Inverse polar stereographic: meters -> lat/lng
        let rho = (mx * mx + my * my).sqrt();
        if rho < 1.0 {
            // At the south pole
            ret.set_lat(-90.0);
            ret.set_lng(0.0);
            return ret;
        }

        let t = rho * t_c / (SemiMajorAxis * m_c);

        // Iterative inverse for mirrored (positive) latitude
        let mut phi_abs = HALF_PI - 2.0 * t.atan();
        for _ in 0..15 {
            let sin_phi = phi_abs.sin();
            let new_phi = HALF_PI - 2.0 * (t * ((1.0 - *ecc * sin_phi) / (1.0 + *ecc * sin_phi)).powf(*ecc / 2.0)).atan();
            if (new_phi - phi_abs).abs() < 1e-12 {
                break;
            }
            phi_abs = new_phi;
        }

        // South pole longitude: atan2(x, y) not atan2(x, -y)
        let lambda = *lambda_0_rad + mx.atan2(my);

        // Mirror latitude back to southern hemisphere
        ret.set_lat(Clip(-RadiansToDegrees(phi_abs), MinLatitude, MaxLatitude));
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
        // GIBS EPSG:3031 uses 2^(z+1) tiles per axis
        let xy = 1i64.wrapping_shl(zoom.wrapping_add(1) as u32);
        GSize::new(xy - 1, xy - 1)
    }

    fn GetGroundResolution(&self, zoom: i32, _latitude: f64) -> f64 {
        // Meters per pixel (true at standard parallel 71S)
        FullExtent / total_pixels(zoom) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pole_centre_and_round_trip() {
        let p = PolarStereographicSouthProjection::Instance();
        assert_eq!(p.FromLatLngToPixel(-90.0, 0.0, 0), GPoint::new(512, 512));
        let px = p.FromLatLngToPixel(-77.85, 166.67, 5);
        let back = p.FromPixelToLatLng(px.x(), px.y(), 5);
        assert!((back.lat() + 77.85).abs() < 0.01 && (back.lng() - 166.67).abs() < 0.01);
        let c = p.FromPixelToLatLng(512, 512, 0);
        assert_eq!((c.lat(), c.lng()), (-90.0, 0.0));
        // a northern latitude is clipped to -30
        assert_eq!(p.FromLatLngToPixel(10.0, 0.0, 0), p.FromLatLngToPixel(-30.0, 0.0, 0));
    }
}
