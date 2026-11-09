//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/GSize.cs`

use super::gpoint::GPoint;

/// `public struct GSize`: the size
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct GSize {
    width: i64,
    height: i64,
}

impl GSize {
    /// `GSize.Empty`
    pub const EMPTY: GSize = GSize { width: 0, height: 0 };

    pub fn new(width: i64, height: i64) -> Self {
        GSize { width, height }
    }

    /// `new GSize(GPoint pt)`
    pub fn from_point(pt: GPoint) -> Self {
        GSize { width: pt.x(), height: pt.y() }
    }

    /// `explicit operator GPoint(GSize size)`
    pub fn to_point(&self) -> GPoint {
        GPoint::new(self.width, self.height)
    }

    /// `IsEmpty`
    pub fn is_empty(&self) -> bool {
        self.width == 0 && self.height == 0
    }

    pub fn width(&self) -> i64 {
        self.width
    }
    pub fn set_width(&mut self, value: i64) {
        self.width = value;
    }
    pub fn height(&self) -> i64 {
        self.height
    }
    pub fn set_height(&mut self, value: i64) {
        self.height = value;
    }

    /// `GSize.Add`
    pub fn add(sz1: GSize, sz2: GSize) -> GSize {
        GSize::new(sz1.width.wrapping_add(sz2.width), sz1.height.wrapping_add(sz2.height))
    }

    /// `GSize.Subtract`
    pub fn subtract(sz1: GSize, sz2: GSize) -> GSize {
        GSize::new(sz1.width.wrapping_sub(sz2.width), sz1.height.wrapping_sub(sz2.height))
    }

    /// `GetHashCode()`: 0 when empty, else `Width.GetHashCode() ^ Height.GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        if self.is_empty() {
            return 0;
        }
        let h = |v: i64| (v as i32) ^ ((v >> 32) as i32);
        h(self.width) ^ h(self.height)
    }
}

impl std::ops::Add for GSize {
    type Output = GSize;
    fn add(self, o: GSize) -> GSize {
        GSize::add(self, o)
    }
}

impl std::ops::Sub for GSize {
    type Output = GSize;
    fn sub(self, o: GSize) -> GSize {
        GSize::subtract(self, o)
    }
}

impl std::fmt::Display for GSize {
    /// `ToString()`: "{Width=.., Height=..}"
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{{Width={}, Height={}}}", self.width, self.height)
    }
}
