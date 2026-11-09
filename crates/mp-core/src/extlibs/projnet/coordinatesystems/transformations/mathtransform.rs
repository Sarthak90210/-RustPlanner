//! Port of `ExtLibs/ProjNet/CoordinateSystems/Transformations/MathTransform.cs`
//!
//! The abstract base's helpers that the ported projections use. The WKT/XML members and the
//! `Coordinate` overloads belong with the unported WKT reader and are not here yet.

use crate::dotnet::Exception;

/// `R2D`
pub const R2D: f64 = 180.0 / std::f64::consts::PI;

/// `D2R`
pub const D2R: f64 = std::f64::consts::PI / 180.0;

/// `Degrees2Radians(deg)`
pub fn degrees2radians(deg: f64) -> f64 {
    D2R * deg
}

/// `Radians2Degrees(rad)`
pub fn radians2degrees(rad: f64) -> f64 {
    R2D * rad
}

/// `IMathTransform`: the operations callers use on a transform.
pub trait MathTransform {
    /// `Transform(double[] point)`
    fn transform(&self, point: &[f64]) -> Result<Vec<f64>, Exception>;

    /// `TransformList(IList<double[]> points)`: each point in turn; the first failure throws.
    fn transform_list(&self, points: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, Exception> {
        let mut result = Vec::with_capacity(points.len());
        for c in points {
            result.push(self.transform(c)?);
        }
        Ok(result)
    }
}
