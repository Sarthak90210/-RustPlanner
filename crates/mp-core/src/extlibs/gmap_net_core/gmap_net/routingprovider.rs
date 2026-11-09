//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/RoutingProvider.cs`

#![allow(non_snake_case)]

use super::maproute::MapRoute;
use super::pointlatlng::PointLatLng;
use crate::dotnet::Exception;

/// `public interface RoutingProvider`: routing interface. A null route is `None`.
pub trait RoutingProvider {
    /// get route between two points
    fn GetRoute(&self, start: PointLatLng, end: PointLatLng, avoidHighways: bool, walkingMode: bool, Zoom: i32) -> Result<Option<MapRoute>, Exception>;

    /// get route between two points (`GetRoute(string start, string end, ...)`)
    fn GetRouteByName(&self, start: &str, end: &str, avoidHighways: bool, walkingMode: bool, Zoom: i32) -> Result<Option<MapRoute>, Exception>;
}
