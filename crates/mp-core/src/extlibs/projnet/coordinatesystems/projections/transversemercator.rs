//! Port of `ExtLibs/ProjNet/CoordinateSystems/Projections/TransverseMercator.cs`
//!
//! Universal (UTM) and Modified (MTM) Transverse Mercator projections (Snyder, Map
//! Projections - A Working Manual, USGS Professional Paper 1395, 1987).

use super::mapprojection::{MapProjection, MapProjectionImpl, PI};
use super::projectionparameterset::ProjectionParameter;
use crate::dotnet::Exception;
use crate::extlibs::projnet::coordinatesystems::transformations::mathtransform::MathTransform;

/// Maximum difference allowed when comparing real numbers.
const EPSILON: f64 = 1E-6;

/// Constants for the forward and inverse series of the ellipsoidal case.
const FC1: f64 = 1.00000000000000000000000; // 1/1
const FC2: f64 = 0.50000000000000000000000; // 1/2
const FC3: f64 = 0.16666666666666666666666; // 1/6
const FC4: f64 = 0.08333333333333333333333; // 1/12
const FC5: f64 = 0.05000000000000000000000; // 1/20
const FC6: f64 = 0.03333333333333333333333; // 1/30
const FC7: f64 = 0.02380952380952380952380; // 1/42
const FC8: f64 = 0.01785714285714285714285; // 1/56

/// `internal class TransverseMercator : MapProjection`
#[derive(Debug, Clone)]
pub struct TransverseMercator {
    base: MapProjection,
    /// e'^2 = (a^2-b^2)/b^2 = es/(1-es)
    esp: f64,
    /// Meridian distance at the latitude of origin.
    ml0: f64,
}

impl TransverseMercator {
    /// `new TransverseMercator(IEnumerable<ProjectionParameter> parameters)`
    pub fn new(parameters: &[ProjectionParameter]) -> Result<Self, Exception> {
        let mut base = MapProjection::new(parameters)?;
        base.name = "Transverse_Mercator".into();
        base.authority = "EPSG".into();
        base.authority_code = 9807;

        let esp = base.es / (1.0 - base.es);
        let ml0 = base.mlfn(base.lat_origin, base.lat_origin.sin(), base.lat_origin.cos());
        Ok(TransverseMercator { base, esp, ml0 })
    }

    /// `Inverse()`: the same projection running the other way. The C# links the two objects so
    /// `Invert()` on one flips both; this returns an independent copy.
    pub fn inverse(&self) -> Result<TransverseMercator, Exception> {
        let mut inv = TransverseMercator::new(&self.base.parameters.to_projection_parameter())?;
        inv.base.set_inverse(!self.base.is_inverse());
        Ok(inv)
    }

    pub fn map_projection(&self) -> &MapProjection {
        &self.base
    }
}

impl MapProjectionImpl for TransverseMercator {
    fn map(&self) -> &MapProjection {
        &self.base
    }

    /// `RadiansToMeters(double[] lonlat)`: radians to projected meters (before false
    /// easting/northing).
    fn radians_to_meters(&self, lonlat: &[f64]) -> Result<Vec<f64>, Exception> {
        let b = &self.base;
        let mut x = lonlat[0];
        x = MapProjection::adjust_lon(x - b.central_meridian);

        let mut y = lonlat[1];
        let sinphi = y.sin();
        let cosphi = y.cos();

        let mut t = if cosphi.abs() > EPSILON { sinphi / cosphi } else { 0.0 };
        t *= t;
        let mut al = cosphi * x;
        let als = al * al;
        al /= (1.0 - b.es * sinphi * sinphi).sqrt();
        let n = self.esp * cosphi * cosphi;

        // NOTE: meridinal distance at latitudeOfOrigin is always 0
        y = b.mlfn(y, sinphi, cosphi) - self.ml0
            + sinphi
                * al
                * x
                * FC2
                * (1.0
                    + FC4
                        * als
                        * (5.0 - t
                            + n * (9.0 + 4.0 * n)
                            + FC6
                                * als
                                * (61.0 + t * (t - 58.0) + n * (270.0 - 330.0 * t)
                                    + FC8 * als * (1385.0 + t * (t * (543.0 - t) - 3111.0)))));

        x = al
            * (FC1
                + FC3
                    * als
                    * (1.0 - t
                        + n
                        + FC5
                            * als
                            * (5.0 + t * (t - 18.0) + n * (14.0 - 58.0 * t) + FC7 * als * (61.0 + t * (t * (179.0 - t) - 479.0)))));

        x = b.scale_factor * b.semi_major * x;
        y = b.scale_factor * b.semi_major * y;

        Ok(if lonlat.len() == 2 { vec![x, y] } else { vec![x, y, lonlat[2]] })
    }

    /// `MetersToRadians(double[] p)`: projected meters (false easting/northing removed) to
    /// radians.
    fn meters_to_radians(&self, p: &[f64]) -> Result<Vec<f64>, Exception> {
        let b = &self.base;
        let mut x = p[0] / b.semi_major;
        let mut y = p[1] / b.semi_major;

        let phi = b.inv_mlfn(self.ml0 + y / b.scale_factor)?;

        if phi.abs() >= PI / 2.0 {
            y = if y < 0.0 { -(PI / 2.0) } else { PI / 2.0 };
            x = 0.0;
        } else {
            let sinphi = phi.sin();
            let cosphi = phi.cos();
            let mut t = if cosphi.abs() > EPSILON { sinphi / cosphi } else { 0.0 };
            let n = self.esp * cosphi * cosphi;
            let mut con = 1.0 - b.es * sinphi * sinphi;
            let d = x * con.sqrt() / b.scale_factor;
            con *= t;
            t *= t;
            let ds = d * d;

            y = phi
                - (con * ds / (1.0 - b.es))
                    * FC2
                    * (1.0
                        - ds * FC4
                            * (5.0 + t * (3.0 - 9.0 * n) + n * (1.0 - 4.0 * n)
                                - ds * FC6
                                    * (61.0 + t * (90.0 - 252.0 * n + 45.0 * t) + 46.0 * n
                                        - ds * FC8 * (1385.0 + t * (3633.0 + t * (4095.0 + 1574.0 * t))))));

            x = MapProjection::adjust_lon(
                b.central_meridian
                    + d * (FC1
                        - ds * FC3
                            * (1.0 + 2.0 * t + n
                                - ds * FC5
                                    * (5.0 + t * (28.0 + 24.0 * t + 8.0 * n) + 6.0 * n
                                        - ds * FC7 * (61.0 + t * (662.0 + t * (1320.0 + 720.0 * t))))))
                        / cosphi,
            );
        }

        Ok(if p.len() == 2 { vec![x, y] } else { vec![x, y, p[2]] })
    }
}

impl MathTransform for TransverseMercator {
    fn transform(&self, point: &[f64]) -> Result<Vec<f64>, Exception> {
        self.map_transform(point)
    }
}
