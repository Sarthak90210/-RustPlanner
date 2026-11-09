//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/PolarStereographicNorthProjection.cs`
//!
//! EPSG:3413 - WGS 84 / NSIDC Sea Ice Polar Stereographic North. Standard parallel: 70N,
//! central meridian: -45. Used by the NASA GIBS Arctic tile service (512x512 tiles).
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

// EPSG:3413 parameters
static phi_c: LazyLock<f64> = LazyLock::new(|| 70.0 * std::f64::consts::PI / 180.0); // standard parallel
static lambda_0: LazyLock<f64> = LazyLock::new(|| -45.0 * std::f64::consts::PI / 180.0); // central meridian

// GIBS tile matrix
const FullExtent: f64 = 8388608.0; // meters, total extent per axis
const OriginX: f64 = -4194304.0; // top-left X in meters
const OriginY: f64 = 4194304.0; // top-left Y in meters

// Geographic bounds
const MinLatitude: f64 = 30.0;
const MaxLatitude: f64 = 90.0;
const MinLongitude: f64 = -180.0;
const MaxLongitude: f64 = 180.0;

/// The static constructor: (sin_phi_c, m_c, t_c)
static STATIC: LazyLock<(f64, f64, f64)> = LazyLock::new(|| {
    let sin_phi_c = phi_c.sin();
    let m_c = phi_c.cos() / (1.0 - *e2 * sin_phi_c * sin_phi_c).sqrt();
    let t_c = ComputeT(*phi_c);
    (sin_phi_c, m_c, t_c)
});

fn ComputeT(phi: f64) -> f64 {
    let sin_phi = phi.sin();
    (std::f64::consts::PI / 4.0 - phi / 2.0).tan() / ((1.0 - *ecc * sin_phi) / (1.0 + *ecc * sin_phi)).powf(*ecc / 2.0)
}

/// `(1L << (zoom + 1)) * 512`
fn total_pixels(zoom: i32) -> i64 {
    1i64.wrapping_shl(zoom.wrapping_add(1) as u32).wrapping_mul(512)
}

/// `public class PolarStereographicNorthProjection : PureProjection`
#[derive(Debug, Default)]
pub struct PolarStereographicNorthProjection {
    cache: ProjectionCache,
}

impl PolarStereographicNorthProjection {
    /// `PolarStereographicNorthProjection.Instance`
    pub fn Instance() -> &'static PolarStereographicNorthProjection {
        static INSTANCE: OnceLock<PolarStereographicNorthProjection> = OnceLock::new();
        INSTANCE.get_or_init(PolarStereographicNorthProjection::default)
    }
}

impl PureProjection for PolarStereographicNorthProjection {
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
        let (_, m_c, t_c) = *STATIC;
        let mut ret = GPoint::EMPTY;
        let lat = Clip(lat, MinLatitude, MaxLatitude);
        let lng = Clip(lng, MinLongitude, MaxLongitude);
        let phi = DegreesToRadians(lat);
        let lambda = DegreesToRadians(lng);

        // Forward polar stereographic: lat/lng -> projected meters
        let t = ComputeT(phi);
        let rho = SemiMajorAxis * m_c * t / t_c;
        let x = rho * (lambda - *lambda_0).sin();
        let y = -rho * (lambda - *lambda_0).cos();

        // Meters -> pixels
        let total = total_pixels(zoom);
        let pixel_x = (x - OriginX) / FullExtent * total as f64;
        let pixel_y = (OriginY - y) / FullExtent * total as f64;

        ret.set_x(cast_i64(Clip(pixel_x + 0.5, 0.0, total.wrapping_sub(1) as f64)));
        ret.set_y(cast_i64(Clip(pixel_y + 0.5, 0.0, total.wrapping_sub(1) as f64)));
        ret
    }

    fn FromPixelToLatLng(&self, x: i64, y: i64, zoom: i32) -> PointLatLng {
        let (_, m_c, t_c) = *STATIC;
        let mut ret = PointLatLng::EMPTY;
        let total = total_pixels(zoom);

        // Pixels -> meters
        let mx = x as f64 / total as f64 * FullExtent + OriginX;
        let my = OriginY - y as f64 / total as f64 * FullExtent;

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
        let lambda = *lambda_0 + mx.atan2(-my);

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
        // GIBS EPSG:3413 uses 2^(z+1) tiles per axis
        let xy = 1i64.wrapping_shl(zoom.wrapping_add(1) as u32);
        GSize::new(xy - 1, xy - 1)
    }

    fn GetGroundResolution(&self, zoom: i32, _latitude: f64) -> f64 {
        // Meters per pixel (true at standard parallel 70N)
        FullExtent / total_pixels(zoom) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pole_centre_and_round_trip() {
        let p = PolarStereographicNorthProjection::Instance();
        // the pole is at the centre of the 1024 px zoom-0 grid
        assert_eq!(p.FromLatLngToPixel(90.0, 0.0, 0), GPoint::new(512, 512));
        assert_eq!(p.GetTileMatrixMaxXY(0), GSize::new(1, 1));
        assert_eq!(p.GetGroundResolution(0, 0.0), 8192.0);
        let px = p.FromLatLngToPixel(72.0, 10.0, 4);
        let back = p.FromPixelToLatLng(px.x(), px.y(), 4);
        assert!((back.lat() - 72.0).abs() < 0.01 && (back.lng() - 10.0).abs() < 0.01);
        let c = p.FromPixelToLatLng(512, 512, 0);
        assert_eq!((c.lat(), c.lng()), (90.0, 0.0));
    }
}
