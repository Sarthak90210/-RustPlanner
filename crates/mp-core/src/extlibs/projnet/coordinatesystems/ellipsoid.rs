//! Port of `ExtLibs/ProjNet/CoordinateSystems/Ellipsoid.cs`
//!
//! The data and constructor logic of `Ellipsoid` and the predefined ellipsoids. The `Info`
//! base members other than the name and the WKT/XML text are not ported yet.

/// `public class Ellipsoid : Info, IEllipsoid`
#[derive(Debug, Clone, PartialEq)]
pub struct Ellipsoid {
    pub semi_major_axis: f64,
    pub semi_minor_axis: f64,
    pub inverse_flattening: f64,
    pub is_ivf_definitive: bool,
    /// `AxisUnit.MetersPerUnit` (all predefined ellipsoids use metres)
    pub axis_unit_meters_per_unit: f64,
    pub name: String,
    pub authority: String,
    pub authority_code: i64,
}

impl Ellipsoid {
    /// `internal Ellipsoid(semiMajorAxis, semiMinorAxis, inverseFlattening, isIvfDefinitive, ...)`:
    /// when the inverse flattening is definitive the semi-minor axis is derived from it.
    pub fn new(
        semi_major_axis: f64,
        semi_minor_axis: f64,
        inverse_flattening: f64,
        is_ivf_definitive: bool,
        name: &str,
        authority: &str,
        code: i64,
    ) -> Self {
        let semi_minor = if is_ivf_definitive && (inverse_flattening == 0.0 || inverse_flattening.is_infinite()) {
            semi_major_axis
        } else if is_ivf_definitive {
            (1.0 - (1.0 / inverse_flattening)) * semi_major_axis
        } else {
            semi_minor_axis
        };
        Ellipsoid {
            semi_major_axis,
            semi_minor_axis: semi_minor,
            inverse_flattening,
            is_ivf_definitive,
            axis_unit_meters_per_unit: 1.0,
            name: name.to_string(),
            authority: authority.to_string(),
            authority_code: code,
        }
    }

    /// `Ellipsoid.WGS84`
    pub fn wgs84() -> Self {
        Ellipsoid::new(6378137.0, 0.0, 298.257223563, true, "WGS 84", "EPSG", 7030)
    }

    /// `Ellipsoid.WGS72`
    pub fn wgs72() -> Self {
        Ellipsoid::new(6378135.0, 0.0, 298.26, true, "WGS 72", "EPSG", 7043)
    }

    /// `Ellipsoid.GRS80`
    pub fn grs80() -> Self {
        Ellipsoid::new(6378137.0, 0.0, 298.257222101, true, "GRS 1980", "EPSG", 7019)
    }

    /// `EqualParams(object obj)`: the numeric parameters and the axis unit.
    pub fn equal_params(&self, e: &Ellipsoid) -> bool {
        e.inverse_flattening == self.inverse_flattening
            && e.is_ivf_definitive == self.is_ivf_definitive
            && e.semi_major_axis == self.semi_major_axis
            && e.semi_minor_axis == self.semi_minor_axis
            && e.axis_unit_meters_per_unit == self.axis_unit_meters_per_unit
    }
}
