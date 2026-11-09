//! Port of `ExtLibs/Core/Geometry/RectangleE.cs`
//!
//! Y grows upwards: `Bottom` is `Y`, `Top` is `Y + Height`. Each property setter raises
//! `GeometryChanged`, so compound setters (`Location`, `Size`, `Center`, `Inflate`) raise one
//! event per field they assign, as in C#. `Clone`/`CloneRect` build a fresh rectangle without
//! handlers or tag.

use std::any::Any;

use super::ashapee::{AShapeE, IShapeE};
use super::calc::Calc;
use super::intervale::IntervalE;
use super::pointe::PointE;
use super::sizee::SizeE;
use crate::dotnet::convert::double_compare_to;
use crate::dotnet::drawing::{Rectangle, RectangleF};
use crate::dotnet::{cast_i32, double_to_string, Exception};

fn nre() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `public class RectangleE : AShapeE, ICloneable, IComparable<RectangleE>`
#[derive(Debug, Default)]
pub struct RectangleE {
    base: AShapeE,
    m_x: f64,
    m_y: f64,
    m_width: f64,
    m_height: f64,
}

impl Clone for RectangleE {
    /// `Clone()` / `CloneRect()`
    fn clone(&self) -> Self {
        self.clone_rect()
    }
}

impl RectangleE {
    /// `new RectangleE(double x, double y, double width, double height)`
    pub fn new(x: f64, y: f64, width: f64, height: f64) -> Self {
        RectangleE { base: AShapeE::default(), m_x: x, m_y: y, m_width: width, m_height: height }
    }

    /// `new RectangleE(PointE location, SizeE size)`
    pub fn from_location_size(location: &PointE, size: &SizeE) -> Self {
        RectangleE::new(location.x(), location.y(), size.width(), size.height())
    }

    /// `new RectangleE(IntervalE xInt, IntervalE yInt)`
    pub fn from_intervals(x_int: &IntervalE, y_int: &IntervalE) -> Self {
        RectangleE::new(x_int.minimum(), y_int.minimum(), x_int.dimension(), y_int.dimension())
    }

    /// `new RectangleE(RectangleE r)`
    pub fn from_rect(r: &RectangleE) -> Self {
        RectangleE::new(r.x(), r.y(), r.width(), r.height())
    }

    /// `X`
    pub fn x(&self) -> f64 {
        self.m_x
    }
    /// `X` setter
    pub fn set_x(&mut self, value: f64) {
        self.m_x = value;
        self.call_geometry_changed();
    }
    /// `Y`
    pub fn y(&self) -> f64 {
        self.m_y
    }
    /// `Y` setter
    pub fn set_y(&mut self, value: f64) {
        self.m_y = value;
        self.call_geometry_changed();
    }
    /// `Width`
    pub fn width(&self) -> f64 {
        self.m_width
    }
    /// `Width` setter
    pub fn set_width(&mut self, value: f64) {
        self.m_width = value;
        self.call_geometry_changed();
    }
    /// `Height`
    pub fn height(&self) -> f64 {
        self.m_height
    }
    /// `Height` setter
    pub fn set_height(&mut self, value: f64) {
        self.m_height = value;
        self.call_geometry_changed();
    }

    /// `Top`
    pub fn top(&self) -> f64 {
        self.y() + self.height()
    }
    /// `Top` setter
    pub fn set_top(&mut self, value: f64) {
        self.set_y(value - self.height());
    }
    /// `Bottom`
    pub fn bottom(&self) -> f64 {
        self.y()
    }
    /// `Bottom` setter
    pub fn set_bottom(&mut self, value: f64) {
        self.set_y(value);
    }
    /// `Left`
    pub fn left(&self) -> f64 {
        self.x()
    }
    /// `Left` setter
    pub fn set_left(&mut self, value: f64) {
        self.set_x(value);
    }
    /// `Right`
    pub fn right(&self) -> f64 {
        self.x() + self.width()
    }
    /// `Right` setter
    pub fn set_right(&mut self, value: f64) {
        self.set_x(value - self.width());
    }

    /// `Location`
    pub fn location(&self) -> PointE {
        PointE::new(self.x(), self.y())
    }
    /// `Location` setter
    pub fn set_location(&mut self, value: &PointE) {
        self.set_x(value.x());
        self.set_y(value.y());
    }

    /// `Size`
    pub fn size(&self) -> SizeE {
        SizeE::new(self.width(), self.height())
    }
    /// `Size` setter
    pub fn set_size(&mut self, value: &SizeE) {
        self.set_width(value.width());
        self.set_height(value.height());
    }

    /// `Center`
    pub fn center(&self) -> PointE {
        PointE::new(self.x() + self.width() / 2.0, self.y() + self.height() / 2.0)
    }
    /// `Center` setter
    pub fn set_center(&mut self, value: &PointE) {
        self.set_x(value.x() - self.width() / 2.0);
        self.set_y(value.y() - self.height() / 2.0);
    }

    /// `TopLeft`
    pub fn top_left(&self) -> PointE {
        PointE::new(self.x(), self.y() + self.height())
    }
    /// `TopLeft` setter
    pub fn set_top_left(&mut self, value: &PointE) {
        self.set_x(value.x());
        self.set_y(value.y() - self.height());
    }

    /// `TopRight`
    pub fn top_right(&self) -> PointE {
        PointE::new(self.x() + self.width(), self.y() + self.height())
    }
    /// `TopRight` setter
    pub fn set_top_right(&mut self, value: &PointE) {
        self.set_x(value.x() - self.width());
        self.set_y(value.y() - self.height());
    }

    /// `BottomLeft`
    pub fn bottom_left(&self) -> PointE {
        self.location()
    }
    /// `BottomLeft` setter
    pub fn set_bottom_left(&mut self, value: &PointE) {
        self.set_location(value);
    }

    /// `BottomRight`
    pub fn bottom_right(&self) -> PointE {
        PointE::new(self.x() + self.width(), self.y())
    }
    /// `BottomRight` setter
    pub fn set_bottom_right(&mut self, value: &PointE) {
        self.set_x(value.x() - self.width());
        self.set_y(value.y());
    }

    /// `XInterval`
    pub fn x_interval(&self) -> IntervalE {
        IntervalE::new(self.x(), self.right())
    }

    /// `YInterval`
    pub fn y_interval(&self) -> IntervalE {
        IntervalE::new(self.y(), self.top())
    }

    /// `IsEmpty`
    pub fn is_empty(&self) -> bool {
        self.m_width == 0.0 || self.m_height == 0.0
    }

    /// `Area`
    pub fn area(&self) -> f64 {
        self.m_width * self.m_height
    }

    /// `Union(RectangleE r)`
    pub fn union(&self, r: &RectangleE) -> RectangleE {
        if r.height() == 0.0 || r.width() == 0.0 {
            return self.clone_rect();
        }
        if self.height() == 0.0 || self.width() == 0.0 {
            return r.clone_rect();
        }
        RectangleE::from_intervals(&self.x_interval().union(&r.x_interval()), &self.y_interval().union(&r.y_interval()))
    }

    /// `Union(PointE pt)`
    pub fn union_point(&self, pt: &PointE) -> RectangleE {
        if self.contains(pt) {
            return self.clone_rect();
        }
        let x_int = self.x_interval().union_pos(pt.x());
        RectangleE::from_intervals(&x_int, &self.y_interval().union_pos(pt.y()))
    }

    /// `Contains(PointE pt)`
    pub fn contains(&self, pt: &PointE) -> bool {
        self.x_interval().contains(pt.x()) && self.y_interval().contains(pt.y())
    }

    /// `Contains(RectangleE rect)`
    pub fn contains_rect(&self, rect: &RectangleE) -> bool {
        self.x_interval().contains_interval(&rect.x_interval()) && self.y_interval().contains_interval(&rect.y_interval())
    }

    /// `Intersect(RectangleE r)`
    pub fn intersect(&self, r: &RectangleE) -> RectangleE {
        let x_intersect = self.x_interval().intersect(&r.x_interval());
        let y_intersect = self.y_interval().intersect(&r.y_interval());
        RectangleE::from_intervals(&x_intersect, &y_intersect)
    }

    /// `IntersectsWith(RectangleE r)`
    pub fn intersects_with(&self, r: &RectangleE) -> bool {
        self.x_interval().intersects_with(&r.x_interval()) && self.y_interval().intersects_with(&r.y_interval())
    }

    /// `OverlapsWith(RectangleE r)`
    pub fn overlaps_with(&self, r: &RectangleE) -> bool {
        if self.intersects_with(r) {
            return true;
        }
        if self.contains_rect(r) {
            return true;
        }
        if r.contains_rect(self) {
            return true;
        }
        false
    }

    /// `Inflate(SizeE size)`
    pub fn inflate_size(&mut self, size: &SizeE) {
        self.inflate_xy(size.width(), size.height());
    }

    /// `Inflate(double amount)`
    pub fn inflate(&mut self, amount: f64) {
        self.inflate_xy(amount, amount);
    }

    /// `Inflate(double amountX, double amountY)`
    pub fn inflate_xy(&mut self, amount_x: f64, amount_y: f64) {
        self.inflate_ltrb(amount_x, amount_x, amount_y, amount_y);
    }

    /// `Inflate(double left, double right, double top, double bottom)`: `SizeE.Inflate` doubles
    /// its arguments, so the size grows by twice `left + right` / `top + bottom`.
    pub fn inflate_ltrb(&mut self, left: f64, right: f64, top: f64, bottom: f64) {
        self.set_x(self.x() - left);
        self.set_y(self.y() - top);
        let s = self.size().inflate_xy(left + right, top + bottom);
        self.set_size(&s);
    }

    /// `ScaleCloneBy(double val)`
    pub fn scale_clone_by(&self, val: f64) -> RectangleE {
        let mut ans = RectangleE::from_rect(self);
        ans.scale_self_by(val);
        ans
    }

    /// `ScaleCloneBy(double val, PointE about)`
    pub fn scale_clone_by_about(&self, val: f64, about: &PointE) -> RectangleE {
        let mut ans = RectangleE::from_rect(self);
        ans.scale_self_by_about(val, about);
        ans
    }

    /// `ScaleSelfBy(double val)`: fields only, no events.
    pub fn scale_self_by(&mut self, val: f64) {
        self.m_x *= val;
        self.m_y *= val;
        self.m_width *= val;
        self.m_height *= val;
    }

    /// `ScaleSelfBy(double val, PointE about)`
    pub fn scale_self_by_about(&mut self, val: f64, about: &PointE) {
        self.m_width *= val;
        self.m_height *= val;
        let l = Calc::point_between(about, &self.location(), val);
        self.set_location(&l);
    }

    /// `CloneRect()`
    pub fn clone_rect(&self) -> RectangleE {
        RectangleE::new(self.x(), self.y(), self.width(), self.height())
    }

    /// `ShiftRectangle(PointE point)`
    pub fn shift_rectangle(&self, point: &PointE) -> RectangleE {
        RectangleE::from_location_size(&self.location().shift_pt(point), &self.size())
    }

    /// `CompareTo(RectangleE other)`: by area, NaN first.
    pub fn compare_to(&self, other: &RectangleE) -> i32 {
        double_compare_to(self.area(), other.area())
    }

    /// `explicit operator Rectangle(RectangleE)`: truncating casts.
    pub fn to_rectangle(&self) -> Rectangle {
        Rectangle { X: cast_i32(self.x()), Y: cast_i32(self.y()), Width: cast_i32(self.width()), Height: cast_i32(self.height()) }
    }

    /// `explicit operator RectangleF(RectangleE)`
    pub fn to_rectangle_f(&self) -> RectangleF {
        RectangleF { X: self.x() as f32, Y: self.y() as f32, Width: self.width() as f32, Height: self.height() as f32 }
    }

    /// `FromMinMax(PointE minPt, PointE maxPt)`: the size goes through the `float` cast of
    /// `(SizeE)(maxPt - minPt)`.
    pub fn from_min_max_points(min_pt: &PointE, max_pt: &PointE) -> RectangleE {
        RectangleE::from_location_size(min_pt, &(max_pt - min_pt).to_size())
    }

    /// `FromMinMax(double minX, double minY, double maxX, double maxY)`
    pub fn from_min_max(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> RectangleE {
        RectangleE::new(min_x, min_y, max_x - min_x, max_y - min_y)
    }
}

impl IShapeE for RectangleE {
    fn shape_base(&self) -> &AShapeE {
        &self.base
    }
    fn shape_base_mut(&mut self) -> &mut AShapeE {
        &mut self.base
    }
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `BoundingBox`: a copy.
    fn bounding_box(&self) -> Option<RectangleE> {
        Some(self.clone_rect())
    }

    /// `Shift(PointE point)`
    fn shift(&self, point: &PointE) -> Box<dyn IShapeE> {
        Box::new(self.shift_rectangle(point))
    }

    /// `ContainsPoint(PointE point)`
    fn contains_point(&self, point: &PointE) -> bool {
        self.contains(point)
    }

    /// `FullyContains(IShapeE shapeE)`: a shape without a bounding box (a point) throws.
    fn fully_contains(&self, shape_e: Option<&dyn IShapeE>) -> Result<bool, Exception> {
        let Some(s) = shape_e else { return Ok(false) };
        let bb = s.bounding_box().ok_or_else(nre)?;
        Ok(self.contains_rect(&bb))
    }

    /// `MostlyContains(IShapeE shapeE, double tolerance)`
    fn mostly_contains_tol(&self, shape_e: Option<&dyn IShapeE>, tolerance: f64) -> Result<bool, Exception> {
        let Some(s) = shape_e else { return Ok(false) };
        let bb = s.bounding_box().ok_or_else(nre)?;
        let intersect = self.intersect(&bb);
        Ok(intersect.area() / self.area() > tolerance)
    }
}

impl std::fmt::Display for RectangleE {
    /// `ToString()` in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RectangleE: X:{}, Y:{}, W:{}, H:{}",
            double_to_string(self.x()),
            double_to_string(self.y()),
            double_to_string(self.width()),
            double_to_string(self.height())
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[test]
    fn rect_geometry() {
        let r = RectangleE::new(0.0, 0.0, 10.0, 4.0);
        assert_eq!(r.top(), 4.0);
        assert_eq!(r.area(), 40.0);
        assert!(r.contains(&PointE::new(10.0, 4.0)));
        let u = r.union_point(&PointE::new(-2.0, 6.0));
        assert_eq!((u.x(), u.y(), u.width(), u.height()), (-2.0, 0.0, 12.0, 6.0));
        let i = r.intersect(&RectangleE::new(5.0, 2.0, 10.0, 10.0));
        assert_eq!((i.x(), i.y(), i.width(), i.height()), (5.0, 2.0, 5.0, 2.0));
        assert!(r.mostly_contains(Some(&RectangleE::new(0.0, 0.0, 10.0, 3.9))).unwrap());
        assert!(r.fully_contains(Some(&PointE::new(1.0, 1.0))).is_err());
        assert!(!r.fully_contains(None).unwrap());
        assert_eq!(r.compare_to(&RectangleE::new(0.0, 0.0, 1.0, 1.0)), 1);
        assert_eq!(r.to_rectangle(), Rectangle { X: 0, Y: 0, Width: 10, Height: 4 });
        let f = RectangleE::from_min_max_points(&PointE::new(0.0, 0.0), &PointE::new(0.1, 0.2));
        assert_eq!(f.width(), 0.1f32 as f64);
    }

    #[test]
    fn inflate_doubles_and_events() {
        let hits = Arc::new(Mutex::new(0));
        let mut r = RectangleE::new(0.0, 0.0, 10.0, 10.0);
        let h = hits.clone();
        r.shape_base().GeometryChanged.add(Arc::new(move |_: &dyn Any| *h.lock().unwrap() += 1));
        r.inflate(1.0);
        // x, y, width, height
        assert_eq!(*hits.lock().unwrap(), 4);
        assert_eq!((r.x(), r.y(), r.width(), r.height()), (-1.0, -1.0, 14.0, 14.0));
        r.scale_self_by(2.0);
        assert_eq!(*hits.lock().unwrap(), 4);
        r.scale_self_by_about(0.5, &PointE::new(0.0, 0.0));
        assert_eq!((r.x(), r.width()), (-1.0, 14.0));
        assert_eq!(*hits.lock().unwrap(), 6);
    }
}
