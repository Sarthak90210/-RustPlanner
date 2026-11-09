//! The app's lightweight lat/lng/alt point, as a layer over the file-by-file port of
//! `ExtLibs/Utilities/PointLatLngAlt.cs`: distance, bearing and newpos are that file's
//! `GetDistance` (haversine, 6371 km), `GetBearing` and `newpos` (6378100 m).

use crate::extlibs::utilities::pointlatlngalt::PointLatLngAlt as PortPoint;

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

    fn port(&self) -> PortPoint {
        PortPoint::new_alt(self.lat, self.lng, self.alt)
    }

    pub fn is_zero(&self) -> bool {
        self.lat == 0.0 && self.lng == 0.0
    }

    /// `GetDistance(p2)`: haversine distance in metres, radius 6371 km.
    pub fn distance(&self, other: &PointLatLngAlt) -> f64 {
        self.port().get_distance(&other.port())
    }

    /// `GetBearing(p2)`: degrees 0..360 from self to other.
    pub fn bearing(&self, other: &PointLatLngAlt) -> f64 {
        self.port().get_bearing(&other.port())
    }

    /// `newpos(bearing, distance)`: the point `distance` metres along `bearing` degrees.
    pub fn newpos(&self, bearing: f64, distance: f64) -> PointLatLngAlt {
        let p = self.port().newpos(bearing, distance);
        PointLatLngAlt::new(p.Lat, p.Lng, p.Alt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_and_bearing_roundtrip() {
        let a = PointLatLngAlt::new(-35.363261, 149.165230, 0.0);
        let b = a.newpos(45.0, 1000.0);
        // newpos moves on a 6378.1 km sphere, GetDistance measures on a 6371 km one
        assert!((a.distance(&b) - 1000.0 * 6371.0 / 6378.1).abs() < 0.01, "{}", a.distance(&b));
        assert!((a.bearing(&b) - 45.0).abs() < 0.01);
        assert_eq!(b.alt, 0.0);
    }
}
