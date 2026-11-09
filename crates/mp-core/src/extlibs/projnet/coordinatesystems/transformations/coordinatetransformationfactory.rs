//! Port of `ExtLibs/ProjNet/CoordinateSystems/Transformations/CoordinateTransformationFactory.cs`
//!
//! The geographic-to-projected path (`Geog2Proj`) for a source geographic system equal to the
//! target's (WGS84 to a WGS84 UTM zone, which is what Mission Planner builds), and
//! `CreateCoordinateOperation` for a projection. The datum-shift, geocentric, fitted and
//! projected-to-projected paths are not ported yet.

use crate::dotnet::Exception;
use crate::extlibs::projnet::coordinatesystems::ellipsoid::Ellipsoid;
use crate::extlibs::projnet::coordinatesystems::projectedcoordinatesystem::{ProjectedCoordinateSystem, Projection};
use crate::extlibs::projnet::coordinatesystems::projections::projectionparameterset::ProjectionParameter;
use crate::extlibs::projnet::coordinatesystems::projections::transversemercator::TransverseMercator;

/// `CreateFromCoordinateSystems(GeographicCoordinateSystem.WGS84, target)`: since the target's
/// geographic system is WGS84 too, `Geog2Proj` makes the plain map projection (degrees in,
/// target units out).
pub fn create_from_wgs84_to_projected(target: &ProjectedCoordinateSystem) -> Result<TransverseMercator, Exception> {
    create_coordinate_operation(&target.projection, &target.ellipsoid, target.linear_unit_meters_per_unit)
}

/// `CreateCoordinateOperation(IProjection projection, IEllipsoid ellipsoid, ILinearUnit unit)`:
/// the projection's parameters plus the ellipsoid axes and the unit, unless already present.
pub fn create_coordinate_operation(
    projection: &Projection,
    ellipsoid: &Ellipsoid,
    meters_per_unit: f64,
) -> Result<TransverseMercator, Exception> {
    let mut parameter_list: Vec<ProjectionParameter> = projection.parameters.clone();

    let has = |list: &[ProjectionParameter], n: &str| list.iter().any(|p| p.name.to_lowercase().replace(' ', "_") == n);
    if !has(&parameter_list, "semi_major") {
        parameter_list.push(ProjectionParameter::new("semi_major", ellipsoid.semi_major_axis));
    }
    if !has(&parameter_list, "semi_minor") {
        parameter_list.push(ProjectionParameter::new("semi_minor", ellipsoid.semi_minor_axis));
    }
    if !has(&parameter_list, "unit") {
        parameter_list.push(ProjectionParameter::new("unit", meters_per_unit));
    }

    // ProjectionsRegistry.CreateProjection(projection.ClassName, parameterList)
    match projection.class_name.to_lowercase().replace(' ', "_").as_str() {
        "transverse_mercator" => TransverseMercator::new(&parameter_list),
        _ => Err(Exception::NotSupported(format!("Projection {} is not supported.", projection.class_name))),
    }
}
