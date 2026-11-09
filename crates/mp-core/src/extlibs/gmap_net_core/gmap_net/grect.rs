//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/GRect.cs`
//!
//! `long` arithmetic is unchecked in C#, so it wraps here. `ToString` formats each `long`
//! with `CultureInfo.CurrentCulture`; every culture Mission Planner ships uses "-" as the
//! negative sign and `long.ToString()` has no group separators, so plain formatting matches.

#![allow(non_snake_case)]

use super::gpoint::GPoint;
use super::gsize::GSize;

/// `public struct GRect`: the rect
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GRect {
    x: i64,
    y: i64,
    width: i64,
    height: i64,
}

impl GRect {
    /// `GRect.Empty`
    pub const EMPTY: GRect = GRect { x: 0, y: 0, width: 0, height: 0 };

    /// `new GRect(long x, long y, long width, long height)`
    pub fn new(x: i64, y: i64, width: i64, height: i64) -> Self {
        GRect { x, y, width, height }
    }

    /// `new GRect(GPoint location, GSize size)`
    pub fn from_location_size(location: GPoint, size: GSize) -> Self {
        GRect { x: location.x(), y: location.y(), width: size.width(), height: size.height() }
    }

    /// `GRect.FromLTRB(int left, int top, int right, int bottom)`: the differences are `int`.
    pub fn FromLTRB(left: i32, top: i32, right: i32, bottom: i32) -> GRect {
        GRect::new(left as i64, top as i64, right.wrapping_sub(left) as i64, bottom.wrapping_sub(top) as i64)
    }

    pub fn Location(&self) -> GPoint {
        GPoint::new(self.x, self.y)
    }
    pub fn set_Location(&mut self, value: GPoint) {
        self.x = value.x();
        self.y = value.y();
    }

    pub fn RightBottom(&self) -> GPoint {
        GPoint::new(self.Right(), self.Bottom())
    }

    pub fn RightTop(&self) -> GPoint {
        GPoint::new(self.Right(), self.Top())
    }

    pub fn LeftBottom(&self) -> GPoint {
        GPoint::new(self.Left(), self.Bottom())
    }

    pub fn Size(&self) -> GSize {
        GSize::new(self.width, self.height)
    }
    pub fn set_Size(&mut self, value: GSize) {
        self.width = value.width();
        self.height = value.height();
    }

    pub fn X(&self) -> i64 {
        self.x
    }
    pub fn set_X(&mut self, value: i64) {
        self.x = value;
    }
    pub fn Y(&self) -> i64 {
        self.y
    }
    pub fn set_Y(&mut self, value: i64) {
        self.y = value;
    }
    pub fn Width(&self) -> i64 {
        self.width
    }
    pub fn set_Width(&mut self, value: i64) {
        self.width = value;
    }
    pub fn Height(&self) -> i64 {
        self.height
    }
    pub fn set_Height(&mut self, value: i64) {
        self.height = value;
    }

    pub fn Left(&self) -> i64 {
        self.x
    }
    pub fn Top(&self) -> i64 {
        self.y
    }
    pub fn Right(&self) -> i64 {
        self.x.wrapping_add(self.width)
    }
    pub fn Bottom(&self) -> i64 {
        self.y.wrapping_add(self.height)
    }

    pub fn IsEmpty(&self) -> bool {
        self.height == 0 && self.width == 0 && self.x == 0 && self.y == 0
    }

    /// `Contains(long x, long y)`
    pub fn Contains(&self, x: i64, y: i64) -> bool {
        self.x <= x && x < self.x.wrapping_add(self.width) && self.y <= y && y < self.y.wrapping_add(self.height)
    }

    /// `Contains(GPoint pt)`
    pub fn ContainsPoint(&self, pt: GPoint) -> bool {
        self.Contains(pt.x(), pt.y())
    }

    /// `Contains(GRect rect)`
    pub fn ContainsRect(&self, rect: GRect) -> bool {
        (self.x <= rect.x)
            && (rect.x.wrapping_add(rect.width) <= self.x.wrapping_add(self.width))
            && (self.y <= rect.y)
            && (rect.y.wrapping_add(rect.height) <= self.y.wrapping_add(self.height))
    }

    /// `GetHashCode()`: 0 when empty, else the rotated-xor of the four `long`s cast to `int`.
    pub fn GetHashCode(&self) -> i32 {
        if self.IsEmpty() {
            return 0;
        }
        let (x, y, w, h) = (self.x, self.y, self.width, self.height);
        ((x ^ (y.wrapping_shl(13) | (y >> 0x13))) ^ (w.wrapping_shl(0x1a) | (w >> 6)) ^ (h.wrapping_shl(7) | (h >> 0x19))) as i32
    }

    /// `Inflate(long width, long height)`
    pub fn Inflate(&mut self, width: i64, height: i64) {
        self.x = self.x.wrapping_sub(width);
        self.y = self.y.wrapping_sub(height);
        self.width = self.width.wrapping_add(2i64.wrapping_mul(width));
        self.height = self.height.wrapping_add(2i64.wrapping_mul(height));
    }

    /// `Inflate(GSize size)`
    pub fn InflateSize(&mut self, size: GSize) {
        self.Inflate(size.width(), size.height());
    }

    /// `static GRect Inflate(GRect rect, long x, long y)`
    pub fn Inflated(rect: GRect, x: i64, y: i64) -> GRect {
        let mut r = rect;
        r.Inflate(x, y);
        r
    }

    /// `Intersect(GRect rect)`
    pub fn IntersectWith(&mut self, rect: GRect) {
        let result = GRect::Intersect(rect, *self);
        self.x = result.x;
        self.y = result.y;
        self.width = result.width;
        self.height = result.height;
    }

    /// `static GRect Intersect(GRect a, GRect b)`
    pub fn Intersect(a: GRect, b: GRect) -> GRect {
        let x1 = a.x.max(b.x);
        let x2 = a.x.wrapping_add(a.width).min(b.x.wrapping_add(b.width));
        let y1 = a.y.max(b.y);
        let y2 = a.y.wrapping_add(a.height).min(b.y.wrapping_add(b.height));
        if x2 >= x1 && y2 >= y1 {
            return GRect::new(x1, y1, x2.wrapping_sub(x1), y2.wrapping_sub(y1));
        }
        GRect::EMPTY
    }

    /// `IntersectsWith(GRect rect)`
    pub fn IntersectsWith(&self, rect: GRect) -> bool {
        (rect.x < self.x.wrapping_add(self.width))
            && (self.x < rect.x.wrapping_add(rect.width))
            && (rect.y < self.y.wrapping_add(self.height))
            && (self.y < rect.y.wrapping_add(rect.height))
    }

    /// `static GRect Union(GRect a, GRect b)`
    pub fn Union(a: GRect, b: GRect) -> GRect {
        let x1 = a.x.min(b.x);
        let x2 = a.x.wrapping_add(a.width).max(b.x.wrapping_add(b.width));
        let y1 = a.y.min(b.y);
        let y2 = a.y.wrapping_add(a.height).max(b.y.wrapping_add(b.height));
        GRect::new(x1, y1, x2.wrapping_sub(x1), y2.wrapping_sub(y1))
    }

    /// `Offset(GPoint pos)`
    pub fn OffsetPoint(&mut self, pos: GPoint) {
        self.Offset(pos.x(), pos.y());
    }

    /// `OffsetNegative(GPoint pos)`
    pub fn OffsetNegative(&mut self, pos: GPoint) {
        self.Offset(pos.x().wrapping_neg(), pos.y().wrapping_neg());
    }

    /// `Offset(long x, long y)`
    pub fn Offset(&mut self, x: i64, y: i64) {
        self.x = self.x.wrapping_add(x);
        self.y = self.y.wrapping_add(y);
    }
}

impl std::fmt::Display for GRect {
    /// `ToString()`: "{X=..,Y=..,Width=..,Height=..}"
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{{X={},Y={},Width={},Height={}}}", self.x, self.y, self.width, self.height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_hash_and_text() {
        let a = GRect::new(0, 0, 10, 10);
        let b = GRect::new(5, 5, 10, 10);
        assert_eq!(GRect::Intersect(a, b), GRect::new(5, 5, 5, 5));
        assert_eq!(GRect::Union(a, b), GRect::new(0, 0, 15, 15));
        assert!(a.Contains(9, 9) && !a.Contains(10, 0));
        assert!(a.IntersectsWith(b));
        // touching rectangles intersect to an empty-size rect, not GRect.Empty
        assert_eq!(GRect::Intersect(a, GRect::new(10, 0, 5, 5)), GRect::new(10, 0, 0, 5));
        assert_eq!(GRect::Intersect(a, GRect::new(20, 0, 5, 5)), GRect::EMPTY);
        let mut c = a;
        c.Inflate(1, 2);
        assert_eq!(c, GRect::new(-1, -2, 12, 14));
        assert_eq!(GRect::FromLTRB(1, 2, 4, 8), GRect::new(1, 2, 3, 6));
        assert_eq!(GRect::FromLTRB(i32::MIN, 0, i32::MAX, 0).Width(), -1);
        assert_eq!(GRect::EMPTY.GetHashCode(), 0);
        let h = GRect::new(1, 2, 3, 4).GetHashCode();
        assert_eq!(h, (1i64 ^ (2i64 << 13) ^ (3i64 << 0x1a) ^ (4i64 << 7)) as i32);
        assert_eq!(GRect::new(1, -2, 3, 4).to_string(), "{X=1,Y=-2,Width=3,Height=4}");
        assert!(a.ContainsRect(GRect::new(1, 1, 9, 9)));
    }
}
