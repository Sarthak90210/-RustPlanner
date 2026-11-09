//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/PointLatLng.cs`

use super::sizelatlng::SizeLatLng;
use crate::dotnet::{double_get_hash_code, double_to_string};

/// `public struct PointLatLng`: the point of coordinates.
#[derive(Debug, Clone, Copy, Default)]
pub struct PointLatLng {
    lat: f64,
    lng: f64,
    not_empty: bool,
}

impl PointLatLng {
    /// `PointLatLng.Empty`
    pub const EMPTY: PointLatLng = PointLatLng { lat: 0.0, lng: 0.0, not_empty: false };

    /// `new PointLatLng(lat, lng)`
    pub fn new(lat: f64, lng: f64) -> Self {
        PointLatLng { lat, lng, not_empty: true }
    }

    /// `IsEmpty`: true if the coordinates were never assigned.
    pub fn is_empty(&self) -> bool {
        !self.not_empty
    }

    /// `Lat` getter
    pub fn lat(&self) -> f64 {
        self.lat
    }

    /// `Lat` setter
    pub fn set_lat(&mut self, value: f64) {
        self.lat = value;
        self.not_empty = true;
    }

    /// `Lng` getter
    pub fn lng(&self) -> f64 {
        self.lng
    }

    /// `Lng` setter
    pub fn set_lng(&mut self, value: f64) {
        self.lng = value;
        self.not_empty = true;
    }

    /// `PointLatLng.Add(pt, sz)`
    pub fn add(pt: PointLatLng, sz: SizeLatLng) -> PointLatLng {
        PointLatLng::new(pt.lat() - sz.height_lat(), pt.lng() + sz.width_lng())
    }

    /// `PointLatLng.Subtract(pt, sz)`
    pub fn subtract(pt: PointLatLng, sz: SizeLatLng) -> PointLatLng {
        PointLatLng::new(pt.lat() + sz.height_lat(), pt.lng() - sz.width_lng())
    }

    /// `Offset(PointLatLng pos)`
    pub fn offset_point(&mut self, pos: PointLatLng) {
        self.offset(pos.lat(), pos.lng());
    }

    /// `Offset(double lat, double lng)`: note the latitude is subtracted.
    pub fn offset(&mut self, lat: f64, lng: f64) {
        let v = self.lng() + lng;
        self.set_lng(v);
        let v = self.lat() - lat;
        self.set_lat(v);
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        double_get_hash_code(self.lng()) ^ double_get_hash_code(self.lat())
    }
}

impl PartialEq for PointLatLng {
    /// `operator ==` and `Equals`: Lat and Lng only, the empty flag is ignored.
    fn eq(&self, right: &PointLatLng) -> bool {
        (self.lng() == right.lng()) && (self.lat() == right.lat())
    }
}

impl std::ops::Add<SizeLatLng> for PointLatLng {
    type Output = PointLatLng;
    fn add(self, sz: SizeLatLng) -> PointLatLng {
        PointLatLng::add(self, sz)
    }
}

impl std::ops::Sub<SizeLatLng> for PointLatLng {
    type Output = PointLatLng;
    fn sub(self, sz: SizeLatLng) -> PointLatLng {
        PointLatLng::subtract(self, sz)
    }
}

impl std::fmt::Display for PointLatLng {
    /// `ToString()`: `{Lat=.., Lng=..}` in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{{Lat={}, Lng={}}}", double_to_string(self.lat()), double_to_string(self.lng()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_and_equality() {
        let mut p = PointLatLng::default();
        assert!(p.is_empty());
        assert_eq!(p, PointLatLng::new(0.0, 0.0));
        p.offset(1.0, 2.0);
        assert!(!p.is_empty());
        assert_eq!((p.lat(), p.lng()), (-1.0, 2.0));
        let s = PointLatLng::new(10.0, 20.0) + SizeLatLng::new(1.0, 2.0);
        assert_eq!((s.lat(), s.lng()), (9.0, 22.0));
    }
}
