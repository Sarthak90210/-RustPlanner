//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/RectLatLng.cs`

use super::pointlatlng::PointLatLng;
use super::sizelatlng::SizeLatLng;
use crate::dotnet::{double_get_hash_code, double_to_string, math_max, math_min};

/// `public struct RectLatLng`: a rectangle by its top-left corner and size.
#[derive(Debug, Clone, Copy, Default)]
pub struct RectLatLng {
    lng: f64,
    lat: f64,
    width_lng: f64,
    height_lat: f64,
    not_empty: bool,
}

impl RectLatLng {
    /// `RectLatLng.Empty`
    pub const EMPTY: RectLatLng = RectLatLng { lng: 0.0, lat: 0.0, width_lng: 0.0, height_lat: 0.0, not_empty: false };

    /// `new RectLatLng(lat, lng, widthLng, heightLat)`
    pub fn new(lat: f64, lng: f64, width_lng: f64, height_lat: f64) -> Self {
        RectLatLng { lng, lat, width_lng, height_lat, not_empty: true }
    }

    /// `new RectLatLng(PointLatLng location, SizeLatLng size)`
    pub fn from_location_size(location: PointLatLng, size: SizeLatLng) -> Self {
        RectLatLng { lng: location.lng(), lat: location.lat(), width_lng: size.width_lng(), height_lat: size.height_lat(), not_empty: true }
    }

    /// `RectLatLng.FromLTRB(leftLng, topLat, rightLng, bottomLat)`
    pub fn from_ltrb(left_lng: f64, top_lat: f64, right_lng: f64, bottom_lat: f64) -> Self {
        RectLatLng::new(top_lat, left_lng, right_lng - left_lng, top_lat - bottom_lat)
    }

    /// `LocationTopLeft`
    pub fn location_top_left(&self) -> PointLatLng {
        PointLatLng::new(self.lat, self.lng)
    }

    pub fn set_location_top_left(&mut self, value: PointLatLng) {
        self.lng = value.lng();
        self.lat = value.lat();
    }

    /// `LocationRightBottom`: the top-left offset by (HeightLat, WidthLng) with
    /// `PointLatLng.Offset` (lat -= height, lng += width).
    pub fn location_right_bottom(&self) -> PointLatLng {
        let mut ret = PointLatLng::new(self.lat, self.lng);
        ret.offset(self.height_lat, self.width_lng);
        ret
    }

    /// `LocationMiddle`
    pub fn location_middle(&self) -> PointLatLng {
        let mut ret = PointLatLng::new(self.lat, self.lng);
        ret.offset(self.height_lat / 2.0, self.width_lng / 2.0);
        ret
    }

    /// `Size`
    pub fn size(&self) -> SizeLatLng {
        SizeLatLng::new(self.height_lat, self.width_lng)
    }

    pub fn set_size(&mut self, value: SizeLatLng) {
        self.width_lng = value.width_lng();
        self.height_lat = value.height_lat();
    }

    pub fn lng(&self) -> f64 {
        self.lng
    }
    pub fn set_lng(&mut self, value: f64) {
        self.lng = value;
    }
    pub fn lat(&self) -> f64 {
        self.lat
    }
    pub fn set_lat(&mut self, value: f64) {
        self.lat = value;
    }
    pub fn width_lng(&self) -> f64 {
        self.width_lng
    }
    pub fn set_width_lng(&mut self, value: f64) {
        self.width_lng = value;
    }
    pub fn height_lat(&self) -> f64 {
        self.height_lat
    }
    pub fn set_height_lat(&mut self, value: f64) {
        self.height_lat = value;
    }

    /// `Left`
    pub fn left(&self) -> f64 {
        self.lng
    }
    /// `Top`
    pub fn top(&self) -> f64 {
        self.lat
    }
    /// `Right`
    pub fn right(&self) -> f64 {
        self.lng + self.width_lng
    }
    /// `Bottom`
    pub fn bottom(&self) -> f64 {
        self.lat - self.height_lat
    }

    /// `IsEmpty`: only the default (and `Empty`) value is empty.
    pub fn is_empty(&self) -> bool {
        !self.not_empty
    }

    /// `Contains(lat, lng)`
    pub fn contains(&self, lat: f64, lng: f64) -> bool {
        (self.lng <= lng) && (lng < (self.lng + self.width_lng)) && (self.lat >= lat) && (lat > (self.lat - self.height_lat))
    }

    /// `Contains(PointLatLng pt)`
    pub fn contains_point(&self, pt: PointLatLng) -> bool {
        self.contains(pt.lat(), pt.lng())
    }

    /// `Contains(RectLatLng rect)`
    pub fn contains_rect(&self, rect: &RectLatLng) -> bool {
        (self.lng <= rect.lng)
            && ((rect.lng + rect.width_lng) <= (self.lng + self.width_lng))
            && (self.lat >= rect.lat)
            && ((rect.lat - rect.height_lat) >= (self.lat - self.height_lat))
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        if self.is_empty() {
            return 0;
        }
        double_get_hash_code(self.lng) ^ double_get_hash_code(self.lat) ^ double_get_hash_code(self.width_lng) ^ double_get_hash_code(self.height_lat)
    }

    /// `Inflate(lat, lng)`
    pub fn inflate(&mut self, lat: f64, lng: f64) {
        self.lng -= lng;
        self.lat += lat;
        self.width_lng += 2.0 * lng;
        self.height_lat += 2.0 * lat;
    }

    /// `Inflate(SizeLatLng size)`
    pub fn inflate_size(&mut self, size: SizeLatLng) {
        self.inflate(size.height_lat(), size.width_lng());
    }

    /// `RectLatLng.Inflate(rect, lat, lng)`
    pub fn inflated(rect: RectLatLng, lat: f64, lng: f64) -> RectLatLng {
        let mut ef = rect;
        ef.inflate(lat, lng);
        ef
    }

    /// `Intersect(RectLatLng rect)`: keeps this rectangle's emptiness flag.
    pub fn intersect_with(&mut self, rect: RectLatLng) {
        let ef = RectLatLng::intersect(rect, *self);
        self.lng = ef.lng;
        self.lat = ef.lat;
        self.width_lng = ef.width_lng;
        self.height_lat = ef.height_lat;
    }

    /// `RectLatLng.Intersect(a, b)` (it adds HeightLat to Lat, unlike `Bottom`)
    pub fn intersect(a: RectLatLng, b: RectLatLng) -> RectLatLng {
        let lng = math_max(a.lng, b.lng);
        let num2 = math_min(a.lng + a.width_lng, b.lng + b.width_lng);
        let lat = math_max(a.lat, b.lat);
        let num4 = math_min(a.lat + a.height_lat, b.lat + b.height_lat);
        if (num2 >= lng) && (num4 >= lat) {
            return RectLatLng::new(lat, lng, num2 - lng, num4 - lat);
        }
        RectLatLng::EMPTY
    }

    /// `IntersectsWith(RectLatLng a)`
    pub fn intersects_with(&self, a: &RectLatLng) -> bool {
        self.left() < a.right() && self.top() > a.bottom() && self.right() > a.left() && self.bottom() < a.top()
    }

    /// `RectLatLng.Union(a, b)`
    pub fn union(a: RectLatLng, b: RectLatLng) -> RectLatLng {
        RectLatLng::from_ltrb(math_min(a.left(), b.left()), math_max(a.top(), b.top()), math_max(a.right(), b.right()), math_min(a.bottom(), b.bottom()))
    }

    /// `Offset(PointLatLng pos)`
    pub fn offset_point(&mut self, pos: PointLatLng) {
        self.offset(pos.lat(), pos.lng());
    }

    /// `Offset(lat, lng)`
    pub fn offset(&mut self, lat: f64, lng: f64) {
        self.lng += lng;
        self.lat -= lat;
    }
}

impl PartialEq for RectLatLng {
    /// `operator ==`: the four values, not the emptiness flag
    fn eq(&self, right: &RectLatLng) -> bool {
        (self.lng == right.lng) && (self.lat == right.lat) && (self.width_lng == right.width_lng) && (self.height_lat == right.height_lat)
    }
}

impl std::fmt::Display for RectLatLng {
    /// `ToString()`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{{Lat={},Lng={},WidthLng={},HeightLat={}}}",
            double_to_string(self.lat),
            double_to_string(self.lng),
            double_to_string(self.width_lng),
            double_to_string(self.height_lat)
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edges_contains_and_set_ops() {
        let r = RectLatLng::from_ltrb(10.0, 50.0, 20.0, 40.0);
        assert_eq!((r.left(), r.top(), r.right(), r.bottom()), (10.0, 50.0, 20.0, 40.0));
        assert_eq!(r.location_right_bottom(), PointLatLng::new(40.0, 20.0));
        assert_eq!(r.location_middle(), PointLatLng::new(45.0, 15.0));
        assert!(r.contains(45.0, 15.0) && !r.contains(40.0, 15.0) && r.contains(50.0, 10.0));
        assert!(RectLatLng::EMPTY.is_empty() && !r.is_empty());
        let u = RectLatLng::union(r, RectLatLng::from_ltrb(0.0, 60.0, 5.0, 55.0));
        assert_eq!((u.left(), u.top(), u.right(), u.bottom()), (0.0, 60.0, 20.0, 40.0));
        assert!(r.intersects_with(&RectLatLng::from_ltrb(15.0, 45.0, 30.0, 30.0)));
        // Intersect adds HeightLat to Lat (unlike Bottom)
        let i = RectLatLng::intersect(r, RectLatLng::new(50.0, 15.0, 10.0, 5.0));
        assert_eq!((i.lat(), i.lng(), i.width_lng(), i.height_lat()), (50.0, 15.0, 5.0, 5.0));
        assert_eq!(RectLatLng::intersect(r, RectLatLng::new(0.0, 100.0, 1.0, 1.0)), RectLatLng::EMPTY);
        let mut o = r;
        o.offset(1.0, 2.0);
        assert_eq!((o.lat(), o.lng()), (49.0, 12.0));
        assert_eq!(RectLatLng::EMPTY.get_hash_code(), 0);
    }
}
