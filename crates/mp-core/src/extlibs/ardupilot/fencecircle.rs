//! Port of `ExtLibs/ArduPilot/FenceCircle.cs`
//!
//! (namespace `MissionPlanner.Utilities`)

use crate::extlibs::mavlink::MAV_CMD;
use crate::extlibs::utilities::pointlatlngalt::PointLatLngAlt;

crate::cs_enum! {
    /// `public enum PolyType`: the MAV_CMD that encodes the circle.
    pub CirclePolyType: i32 {
        Inclusive = MAV_CMD::FENCE_CIRCLE_INCLUSION.0 as i32,
        Exclusive = MAV_CMD::FENCE_CIRCLE_EXCLUSION.0 as i32,
        HomeInclusive = MAV_CMD::FENCE_HOME_CIRCLE_INCLUSION.0 as i32,
    }
}

/// `public class FenceCircle`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FenceCircle {
    /// Circle centre. Not used (null) for HomeInclusive, which is always centred on the
    /// vehicle's home.
    pub Center: Option<PointLatLngAlt>,
    pub Radius: f32,
    /// `Mode` (C# default 0, which is not a member)
    pub Mode: CirclePolyType,
}
