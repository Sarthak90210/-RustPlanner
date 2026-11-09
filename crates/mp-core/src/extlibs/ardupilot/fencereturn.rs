//! Port of `ExtLibs/ArduPilot/FenceReturn.cs`
//!
//! (namespace `MissionPlanner.Utilities`)

use crate::extlibs::utilities::pointlatlngalt::PointLatLngAlt;

/// `public class FenceReturn`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FenceReturn {
    /// `Return` (null until set)
    pub Return: Option<PointLatLngAlt>,
}
