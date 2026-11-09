//! Port of `ExtLibs/Utilities/LocationProjection.cs`

use super::pointlatlngalt::PointLatLngAlt;
use super::vector3::Vector3;
use crate::dotnet::DateTime;

/// `public class LocationProjection`
pub struct LocationProjection;

impl LocationProjection {
    /// `Project(location, velocity, locationtime, now)`: dead-reckons `location` along the
    /// velocity (x north, y east, z up, m/s) for the time elapsed since `locationtime`.
    pub fn project(location: &PointLatLngAlt, velocity: &Vector3, locationtime: DateTime, now: DateTime) -> PointLatLngAlt {
        let deltatime = now - locationtime;
        let bearing = velocity.y.atan2(velocity.x) * (180.0 / std::f64::consts::PI);
        let dist = *velocity * deltatime.total_seconds();
        let mut newpos = location.newpos(bearing, dist.length());
        newpos.Alt += dist.z;
        newpos
    }
}
