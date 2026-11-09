//! Port of `ExtLibs/ProjNet/CoordinateSystems/ProjectedCoordinateSystem.cs`
//!
//! Only the parts the ported callers reach: the `WGS84_UTM` factory and the members the
//! geographic-to-projected transformation reads (projection parameters, the geographic
//! system's ellipsoid, the linear unit). Axes, WKT/XML and `EqualParams` are not ported yet.

use super::ellipsoid::Ellipsoid;
use super::projections::projectionparameterset::ProjectionParameter;

/// `Projection`: the projection's class name, parameters and identity.
#[derive(Debug, Clone, PartialEq)]
pub struct Projection {
    pub class_name: String,
    pub parameters: Vec<ProjectionParameter>,
    pub name: String,
    pub authority: String,
    pub authority_code: i64,
}

/// `public class ProjectedCoordinateSystem : HorizontalCoordinateSystem, IProjectedCoordinateSystem`
#[derive(Debug, Clone, PartialEq)]
pub struct ProjectedCoordinateSystem {
    /// `GeographicCoordinateSystem.HorizontalDatum.Ellipsoid`
    pub ellipsoid: Ellipsoid,
    /// `LinearUnit.MetersPerUnit`
    pub linear_unit_meters_per_unit: f64,
    pub projection: Projection,
    pub name: String,
    pub authority: String,
    pub authority_code: i64,
    pub remarks: String,
}

impl ProjectedCoordinateSystem {
    /// `ProjectedCoordinateSystem.WGS84_UTM(int zone, bool zoneIsNorth)`
    pub fn wgs84_utm(zone: i32, zone_is_north: bool) -> Self {
        let p_info = vec![
            ProjectionParameter::new("latitude_of_origin", 0.0),
            ProjectionParameter::new("central_meridian", (zone.wrapping_mul(6).wrapping_sub(183)) as f64),
            ProjectionParameter::new("scale_factor", 0.9996),
            ProjectionParameter::new("false_easting", 500000.0),
            ProjectionParameter::new("false_northing", if zone_is_north { 0.0 } else { 10000000.0 }),
        ];
        let ns = if zone_is_north { "N" } else { "S" };
        let code = 32600 + zone as i64 + if zone_is_north { 0 } else { 100 };
        let proj = Projection {
            class_name: "Transverse_Mercator".into(),
            parameters: p_info,
            name: format!("UTM{zone}{ns}"),
            authority: "EPSG".into(),
            authority_code: code,
        };
        ProjectedCoordinateSystem {
            ellipsoid: Ellipsoid::wgs84(),
            linear_unit_meters_per_unit: 1.0,
            projection: proj,
            name: format!("WGS 84 / UTM zone {zone}{ns}"),
            authority: "EPSG".into(),
            authority_code: code,
            remarks: "Large and medium scale topographic mapping and engineering survey.".into(),
        }
    }
}
