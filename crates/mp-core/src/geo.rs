//! Port of `PointLatLngAlt` math from MissionPlanner.Utilities.

pub const EARTH_RADIUS_M: f64 = 6_378_137.0;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PointLatLngAlt {
    pub lat: f64,
    pub lng: f64,
    pub alt: f64,
}

impl PointLatLngAlt {
    pub const fn new(lat: f64, lng: f64, alt: f64) -> Self {
        Self { lat, lng, alt }
    }

    pub fn is_zero(&self) -> bool {
        self.lat == 0.0 && self.lng == 0.0
    }

    /// Great-circle distance in metres (haversine), as `GetDistance` in Mission Planner.
    pub fn distance(&self, other: &PointLatLngAlt) -> f64 {
        let d_lat = (other.lat - self.lat).to_radians();
        let d_lng = (other.lng - self.lng).to_radians();
        let a = (d_lat / 2.0).sin().powi(2)
            + self.lat.to_radians().cos()
                * other.lat.to_radians().cos()
                * (d_lng / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        EARTH_RADIUS_M * c
    }

    /// Bearing in degrees 0..360 from self to other, as `GetBearing`.
    pub fn bearing(&self, other: &PointLatLngAlt) -> f64 {
        let lat1 = self.lat.to_radians();
        let lat2 = other.lat.to_radians();
        let d_lng = (other.lng - self.lng).to_radians();
        let y = d_lng.sin() * lat2.cos();
        let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * d_lng.cos();
        (y.atan2(x).to_degrees() + 360.0) % 360.0
    }

    /// New point at `distance` metres along `bearing` degrees, as `newpos`.
    pub fn newpos(&self, bearing: f64, distance: f64) -> PointLatLngAlt {
        let lat1 = self.lat.to_radians();
        let lng1 = self.lng.to_radians();
        let brng = bearing.to_radians();
        let dr = distance / EARTH_RADIUS_M;
        let lat2 = (lat1.sin() * dr.cos() + lat1.cos() * dr.sin() * brng.cos()).asin();
        let lng2 =
            lng1 + (brng.sin() * dr.sin() * lat1.cos()).atan2(dr.cos() - lat1.sin() * lat2.sin());
        PointLatLngAlt::new(
            lat2.to_degrees(),
            ((lng2.to_degrees() + 540.0) % 360.0) - 180.0,
            self.alt,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_and_bearing_roundtrip() {
        let a = PointLatLngAlt::new(-35.363261, 149.165230, 0.0);
        let b = a.newpos(45.0, 1000.0);
        assert!((a.distance(&b) - 1000.0).abs() < 0.5);
        assert!((a.bearing(&b) - 45.0).abs() < 0.01);
    }
}
