//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/GPoint.cs`

use super::gsize::GSize;

/// `public struct GPoint`: the point
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct GPoint {
    x: i64,
    y: i64,
}

impl GPoint {
    /// `GPoint.Empty`
    pub const EMPTY: GPoint = GPoint { x: 0, y: 0 };

    pub fn new(x: i64, y: i64) -> Self {
        GPoint { x, y }
    }

    /// `new GPoint(GSize sz)`
    pub fn from_size(sz: GSize) -> Self {
        GPoint { x: sz.width(), y: sz.height() }
    }

    /// `IsEmpty`
    pub fn is_empty(&self) -> bool {
        self.x == 0 && self.y == 0
    }

    pub fn x(&self) -> i64 {
        self.x
    }
    pub fn set_x(&mut self, value: i64) {
        self.x = value;
    }
    pub fn y(&self) -> i64 {
        self.y
    }
    pub fn set_y(&mut self, value: i64) {
        self.y = value;
    }

    /// `explicit operator GSize(GPoint p)`
    pub fn to_size(&self) -> GSize {
        GSize::new(self.x, self.y)
    }

    /// `GPoint.Add(pt, sz)`
    pub fn add(pt: GPoint, sz: GSize) -> GPoint {
        GPoint::new(pt.x.wrapping_add(sz.width()), pt.y.wrapping_add(sz.height()))
    }

    /// `GPoint.Subtract(pt, sz)`
    pub fn subtract(pt: GPoint, sz: GSize) -> GPoint {
        GPoint::new(pt.x.wrapping_sub(sz.width()), pt.y.wrapping_sub(sz.height()))
    }

    /// `GetHashCode()`: `(int)(x ^ y)`
    pub fn get_hash_code(&self) -> i32 {
        (self.x ^ self.y) as i32
    }

    /// `Offset(dx, dy)`
    pub fn offset(&mut self, dx: i64, dy: i64) {
        self.x = self.x.wrapping_add(dx);
        self.y = self.y.wrapping_add(dy);
    }

    /// `Offset(GPoint p)`
    pub fn offset_point(&mut self, p: GPoint) {
        self.offset(p.x, p.y);
    }

    /// `OffsetNegative(GPoint p)`
    pub fn offset_negative(&mut self, p: GPoint) {
        self.offset(p.x.wrapping_neg(), p.y.wrapping_neg());
    }
}

impl std::ops::Add<GSize> for GPoint {
    type Output = GPoint;
    fn add(self, sz: GSize) -> GPoint {
        GPoint::add(self, sz)
    }
}

impl std::ops::Sub<GSize> for GPoint {
    type Output = GPoint;
    fn sub(self, sz: GSize) -> GPoint {
        GPoint::subtract(self, sz)
    }
}

impl std::fmt::Display for GPoint {
    /// `ToString()`: "{X=..,Y=..}"
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{{X={},Y={}}}", self.x, self.y)
    }
}

/// `internal class GPointComparer : IEqualityComparer<GPoint>`
#[derive(Debug, Clone, Copy, Default)]
pub struct GPointComparer;

impl GPointComparer {
    /// `Equals(x, y)`
    pub fn equals(&self, x: GPoint, y: GPoint) -> bool {
        x.x() == y.x() && x.y() == y.y()
    }

    /// `GetHashCode(obj)`
    pub fn get_hash_code(&self, obj: GPoint) -> i32 {
        obj.get_hash_code()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic_hash_and_text() {
        let mut p = GPoint::new(5, -3);
        assert_eq!(p + GSize::new(1, 1), GPoint::new(6, -2));
        assert_eq!(p - GSize::new(1, 1), GPoint::new(4, -4));
        assert_eq!(p.get_hash_code(), (5i64 ^ -3i64) as i32);
        p.offset_negative(GPoint::new(5, -3));
        assert!(p.is_empty());
        assert_eq!(GPoint::new(1, -2).to_string(), "{X=1,Y=-2}");
        assert!(GPointComparer.equals(GPoint::new(1, 2), GPoint::new(1, 2)));
        assert_eq!(GSize::new(0, 0).get_hash_code(), 0);
        assert_eq!(GSize::new(3, 4).to_string(), "{Width=3, Height=4}");
        assert_eq!(GSize::new(1 << 33, 0).get_hash_code(), 2);
    }
}
