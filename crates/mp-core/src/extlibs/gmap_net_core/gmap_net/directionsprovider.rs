//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/DirectionsProvider.cs`
//!
//! The `out` parameter comes first in the returned tuple, as it does in the C# signature;
//! a null `out` value is `None`.

#![allow(non_snake_case)]

use super::gdirections::GDirections;
use super::pointlatlng::PointLatLng;
use super::statuscodes::DirectionsStatusCode;
use crate::dotnet::Exception;

/// `interface DirectionsProvider`: directions interface
#[allow(clippy::too_many_arguments)]
pub trait DirectionsProvider {
    /// `DirectionsStatusCode GetDirections(out GDirections direction, PointLatLng start, PointLatLng end, ...)`
    fn GetDirections(&self, start: PointLatLng, end: PointLatLng, avoidHighways: bool, avoidTolls: bool, walkingMode: bool, sensor: bool, metric: bool) -> Result<(Option<GDirections>, DirectionsStatusCode), Exception>;

    /// `DirectionsStatusCode GetDirections(out GDirections direction, string start, string end, ...)`
    fn GetDirectionsByName(&self, start: &str, end: &str, avoidHighways: bool, avoidTolls: bool, walkingMode: bool, sensor: bool, metric: bool) -> Result<(Option<GDirections>, DirectionsStatusCode), Exception>;

    /// `IEnumerable<GDirections> GetDirections(out DirectionsStatusCode status, string start, string end, ...)`:
    /// service may provide more than one route alternative in the response
    fn GetDirectionsAlternativesByName(&self, start: &str, end: &str, avoidHighways: bool, avoidTolls: bool, walkingMode: bool, sensor: bool, metric: bool) -> Result<(DirectionsStatusCode, Option<Vec<GDirections>>), Exception>;

    /// `IEnumerable<GDirections> GetDirections(out DirectionsStatusCode status, PointLatLng start, PointLatLng end, ...)`:
    /// service may provide more than one route alternative in the response
    fn GetDirectionsAlternatives(&self, start: PointLatLng, end: PointLatLng, avoidHighways: bool, avoidTolls: bool, walkingMode: bool, sensor: bool, metric: bool) -> Result<(DirectionsStatusCode, Option<Vec<GDirections>>), Exception>;

    /// `DirectionsStatusCode GetDirections(out GDirections direction, PointLatLng start, IEnumerable<PointLatLng> wayPoints, ...)`
    fn GetDirectionsWithWayPoints(&self, start: PointLatLng, wayPoints: &[PointLatLng], avoidHighways: bool, avoidTolls: bool, walkingMode: bool, sensor: bool, metric: bool) -> Result<(Option<GDirections>, DirectionsStatusCode), Exception>;

    /// `DirectionsStatusCode GetDirections(out GDirections direction, string start, IEnumerable<string> wayPoints, ...)`
    fn GetDirectionsWithWayPointsByName(&self, start: &str, wayPoints: &[String], avoidHighways: bool, avoidTolls: bool, walkingMode: bool, sensor: bool, metric: bool) -> Result<(Option<GDirections>, DirectionsStatusCode), Exception>;
}
