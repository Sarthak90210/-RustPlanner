//! Port of `ExtLibs/ArduPilot/FencePolygon.cs`
//!
//! (namespace `MissionPlanner.Utilities`)

use crate::extlibs::mavlink::MAV_CMD;
use crate::extlibs::utilities::pointlatlngalt::PointLatLngAlt;

crate::cs_enum! {
    /// `public enum PolyType`: the MAV_CMD that encodes each vertex.
    pub PolygonPolyType: i32 {
        Inclusive = MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION.0 as i32,
        Exclusive = MAV_CMD::FENCE_POLYGON_VERTEX_EXCLUSION.0 as i32,
    }
}

/// `public class FencePolygon`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FencePolygon {
    pub Points: Vec<PointLatLngAlt>,
    /// `Mode` (C# default 0, which is not a member)
    pub Mode: PolygonPolyType,
}
