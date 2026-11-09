//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/SizeLatLng.cs`

use super::pointlatlng::PointLatLng;
use crate::dotnet::{double_get_hash_code, double_to_string};

/// `public struct SizeLatLng`: the size of coordinates.
#[derive(Debug, Clone, Copy, Default)]
pub struct SizeLatLng {
    height_lat: f64,
    width_lng: f64,
}

impl SizeLatLng {
    /// `SizeLatLng.Empty`
    pub const EMPTY: SizeLatLng = SizeLatLng { height_lat: 0.0, width_lng: 0.0 };

    /// `new SizeLatLng(heightLat, widthLng)`
    pub fn new(height_lat: f64, width_lng: f64) -> Self {
        SizeLatLng { height_lat, width_lng }
    }

    /// `new SizeLatLng(PointLatLng pt)`
    pub fn from_point(pt: PointLatLng) -> Self {
        SizeLatLng { height_lat: pt.lat(), width_lng: pt.lng() }
    }

    /// `IsEmpty`
    pub fn is_empty(&self) -> bool {
        (self.width_lng == 0.0) && (self.height_lat == 0.0)
    }

    /// `WidthLng`
    pub fn width_lng(&self) -> f64 {
        self.width_lng
    }

    pub fn set_width_lng(&mut self, value: f64) {
        self.width_lng = value;
    }

    /// `HeightLat`
    pub fn height_lat(&self) -> f64 {
        self.height_lat
    }

    pub fn set_height_lat(&mut self, value: f64) {
        self.height_lat = value;
    }

    /// `SizeLatLng.Add`
    pub fn add(sz1: SizeLatLng, sz2: SizeLatLng) -> SizeLatLng {
        SizeLatLng::new(sz1.height_lat() + sz2.height_lat(), sz1.width_lng() + sz2.width_lng())
    }

    /// `SizeLatLng.Subtract`
    pub fn subtract(sz1: SizeLatLng, sz2: SizeLatLng) -> SizeLatLng {
        SizeLatLng::new(sz1.height_lat() - sz2.height_lat(), sz1.width_lng() - sz2.width_lng())
    }

    /// `ToPointLatLng()` (the explicit conversion to `PointLatLng`)
    pub fn to_point_lat_lng(&self) -> PointLatLng {
        PointLatLng::new(self.height_lat(), self.width_lng())
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        if self.is_empty() {
            return 0;
        }
        double_get_hash_code(self.width_lng()) ^ double_get_hash_code(self.height_lat())
    }
}

impl PartialEq for SizeLatLng {
    fn eq(&self, sz2: &SizeLatLng) -> bool {
        (self.width_lng() == sz2.width_lng()) && (self.height_lat() == sz2.height_lat())
    }
}

impl std::ops::Add for SizeLatLng {
    type Output = SizeLatLng;
    fn add(self, o: SizeLatLng) -> SizeLatLng {
        SizeLatLng::add(self, o)
    }
}

impl std::ops::Sub for SizeLatLng {
    type Output = SizeLatLng;
    fn sub(self, o: SizeLatLng) -> SizeLatLng {
        SizeLatLng::subtract(self, o)
    }
}

impl From<SizeLatLng> for PointLatLng {
    fn from(size: SizeLatLng) -> PointLatLng {
        size.to_point_lat_lng()
    }
}

impl std::fmt::Display for SizeLatLng {
    /// `ToString()`: the C# labels the latitude "HeightLng".
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{{WidthLng={}, HeightLng={}}}", double_to_string(self.width_lng), double_to_string(self.height_lat))
    }
}
