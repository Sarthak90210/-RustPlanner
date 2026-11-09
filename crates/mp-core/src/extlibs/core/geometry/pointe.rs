//! Port of `ExtLibs/Core/Geometry/PointE.cs`
//!
//! `PointE` is a class with events, so `Clone` builds a fresh point (the copy constructor
//! `new PointE(pt)`) with no handlers and no tag. The arithmetic operators build new points as
//! the C# operators do. `ReadXml`/`WriteXml` take and return the element text.

use std::any::Any;

use super::ashapee::{AShapeE, IShapeE};
use super::rectanglee::RectangleE;
use super::sizee::SizeE;
use crate::dotnet::convert::to_double_str;
use crate::dotnet::drawing::{Point, PointF};
use crate::dotnet::{cast_i32, double_to_string, Exception};
use crate::extlibs::core::geometry::point3d::en_us;
use crate::extlibs::core::utils::stringutils::{list_get, StringUtils};

/// `public interface IPoint`
pub trait IPoint {
    /// `double X { get; }`
    fn x(&self) -> f64;
    /// `double X { set; }`
    fn set_x(&mut self, v: f64);
    /// `double Y { get; }`
    fn y(&self) -> f64;
    /// `double Y { set; }`
    fn set_y(&mut self, v: f64);
    /// `bool Equals(IPoint test, double tolerance)`
    fn equals_tol(&self, test: &dyn IPoint, tolerance: f64) -> bool;
    /// `string Serialize()`
    fn serialize(&self) -> String;
    /// `void Deserialize(string str)`
    fn deserialize(&mut self, str: Option<&str>) -> Result<(), Exception>;
}

/// `public class PointE : AShapeE, IXmlSerializable, IPoint`
#[derive(Debug, Default)]
pub struct PointE {
    base: AShapeE,
    m_x: f64,
    m_y: f64,
}

impl Clone for PointE {
    /// `new PointE(PointE pt)`
    fn clone(&self) -> Self {
        PointE::new(self.m_x, self.m_y)
    }
}

impl PointE {
    /// `new PointE(double x, double y)`
    pub fn new(x: f64, y: f64) -> Self {
        PointE { base: AShapeE::default(), m_x: x, m_y: y }
    }

    /// `X`
    pub fn x(&self) -> f64 {
        self.m_x
    }

    /// `X` setter: raises `GeometryChanged`.
    pub fn set_x(&mut self, value: f64) {
        self.m_x = value;
        self.call_geometry_changed();
    }

    /// `Y`
    pub fn y(&self) -> f64 {
        self.m_y
    }

    /// `Y` setter: raises `GeometryChanged`.
    pub fn set_y(&mut self, value: f64) {
        self.m_y = value;
        self.call_geometry_changed();
    }

    /// `Abs`
    pub fn abs(&self) -> f64 {
        (self.x() * self.x() + self.y() * self.y()).sqrt()
    }

    /// `Magnitude`
    pub fn magnitude(&self) -> f64 {
        (self.x() * self.x() + self.y() * self.y()).sqrt()
    }

    /// `GetNormalized()`
    pub fn get_normalized(&self) -> PointE {
        let magnitude = self.magnitude();
        PointE::new(self.x() / magnitude, self.y() / magnitude)
    }

    /// `AbsDifference(PointE point)`
    pub fn abs_difference(&self, point: &PointE) -> PointE {
        PointE::new((self.x() - point.x()).abs(), (self.y() - point.y()).abs())
    }

    /// `Dist(PointE point)`
    pub fn dist(&self, point: &PointE) -> f64 {
        let x = self.x() - point.x();
        let y = self.y() - point.y();
        (x * x + y * y).sqrt()
    }

    /// `Normalize()`: two setter calls, so two `GeometryChanged` events.
    pub fn normalize(&mut self) {
        let magnitude = self.magnitude();
        self.set_x(self.x() / magnitude);
        self.set_y(self.y() / magnitude);
    }

    /// `DotProduct(PointE vector)`
    pub fn dot_product(&self, vector: &PointE) -> f64 {
        self.x() * vector.x() + self.y() * vector.y()
    }

    /// `implicit operator PointE(PointF)`
    pub fn from_point_f(p: PointF) -> PointE {
        PointE::new(p.X as f64, p.Y as f64)
    }

    /// `explicit operator PointF(PointE)`
    pub fn to_point_f(&self) -> PointF {
        PointF::new(self.x() as f32, self.y() as f32)
    }

    /// `implicit operator PointE(Point)`
    pub fn from_point(p: Point) -> PointE {
        PointE::new(p.X as f64, p.Y as f64)
    }

    /// `explicit operator Point(PointE)`: truncating casts.
    pub fn to_point(&self) -> Point {
        Point { X: cast_i32(self.x()), Y: cast_i32(self.y()) }
    }

    /// `implicit operator PointE(SizeE)`
    pub fn from_size(size: &SizeE) -> PointE {
        PointE::new(size.width(), size.height())
    }

    /// `explicit operator SizeE(PointE)`: each coordinate goes through `float`.
    pub fn to_size(&self) -> SizeE {
        SizeE::new(self.x() as f32 as f64, self.y() as f32 as f64)
    }

    /// `operator +(PointE c)`
    pub fn plus(&self) -> PointE {
        let mut ans = PointE::default();
        ans.set_x(self.x());
        ans.set_y(self.y());
        ans
    }

    /// `GetHashCode()`: `(int)X ^ (int)Y`.
    pub fn get_hash_code(&self) -> i32 {
        cast_i32(self.x()) ^ cast_i32(self.y())
    }

    /// `ReadXml(XmlReader r)` given the element content.
    pub fn read_xml(&mut self, content: &str) -> Result<(), Exception> {
        IPoint::deserialize(self, Some(content))
    }

    /// `WriteXml(XmlWriter w)`: the text written.
    pub fn write_xml(&self) -> String {
        IPoint::serialize(self)
    }

    /// `ShiftPt(PointE offset)`
    pub fn shift_pt(&self, offset: &PointE) -> PointE {
        PointE::new(self.x() + offset.x(), self.y() + offset.y())
    }

    /// `HasSamePoints(PointE nxtPt)`
    pub fn has_same_points(&self, nxt_pt: &PointE) -> bool {
        nxt_pt.x() == self.x() && nxt_pt.y() == self.y()
    }

    /// `HasSamePoints(PointE nxtPt, double tol)`
    pub fn has_same_points_tol(&self, nxt_pt: &PointE, tol: f64) -> bool {
        if self.has_same_points(nxt_pt) {
            return true;
        }
        if (nxt_pt.x() - self.x()).abs() > tol {
            return false;
        }
        if (nxt_pt.y() - self.y()).abs() > tol {
            return false;
        }
        true
    }

    /// `TransformByRotationAbout(double rotation, PointE center)`
    pub fn transform_by_rotation_about(&self, rotation: f64, center: &PointE) -> PointE {
        let t = self - center;
        let cos = rotation.cos();
        let sin = rotation.sin();
        let t = PointE::new(cos * t.x() - sin * t.y(), sin * t.x() + cos * t.y());
        &t + center
    }

    /// `UpdatePoint(PointE pointE)`: copies the fields without raising events.
    pub fn update_point(&mut self, point_e: Option<&PointE>) {
        let Some(p) = point_e else { return };
        self.m_x = p.m_x;
        self.m_y = p.m_y;
    }
}

impl IPoint for PointE {
    fn x(&self) -> f64 {
        self.m_x
    }
    fn set_x(&mut self, v: f64) {
        PointE::set_x(self, v)
    }
    fn y(&self) -> f64 {
        self.m_y
    }
    fn set_y(&mut self, v: f64) {
        PointE::set_y(self, v)
    }

    /// `virtual bool Equals(IPoint test, double tolerance)`
    fn equals_tol(&self, test: &dyn IPoint, tolerance: f64) -> bool {
        (test.x() - self.x()).abs() < tolerance && (test.y() - self.y()).abs() < tolerance
    }

    /// `virtual string Serialize()`: en-US numbers.
    fn serialize(&self) -> String {
        format!("{},{}", en_us(self.m_x), en_us(self.m_y))
    }

    /// `virtual void Deserialize(string str)`: current-culture parse; X stays set when Y fails.
    fn deserialize(&mut self, str: Option<&str>) -> Result<(), Exception> {
        let bits = StringUtils::split_to_list(str, ",")?;
        self.m_x = to_double_str(Some(list_get(&bits, 0)?))?;
        self.m_y = to_double_str(Some(list_get(&bits, 1)?))?;
        Ok(())
    }
}

impl IShapeE for PointE {
    fn shape_base(&self) -> &AShapeE {
        &self.base
    }
    fn shape_base_mut(&mut self) -> &mut AShapeE {
        &mut self.base
    }
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `BoundingBox`: null for a point.
    fn bounding_box(&self) -> Option<RectangleE> {
        None
    }

    /// `Shift(PointE offset)`
    fn shift(&self, offset: &PointE) -> Box<dyn IShapeE> {
        Box::new(self.shift_pt(offset))
    }

    /// `Equals(object obj)`: `obj.GetType()` throws for null; only an exact `PointE` with
    /// equal coordinates matches.
    fn equals(&self, obj: Option<&dyn Any>) -> Result<bool, Exception> {
        let obj = obj.ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        Ok(match obj.downcast_ref::<PointE>() {
            Some(test) => test.x() == self.x() && test.y() == self.y(),
            None => false,
        })
    }
}

impl std::ops::Neg for &PointE {
    type Output = PointE;
    /// `operator -(PointE c)`
    fn neg(self) -> PointE {
        let mut ans = PointE::default();
        ans.set_x(-self.x());
        ans.set_y(-self.y());
        ans
    }
}

impl std::ops::Add for &PointE {
    type Output = PointE;
    /// `operator +(PointE a, PointE b)`
    fn add(self, b: &PointE) -> PointE {
        PointE::new(self.x() + b.x(), self.y() + b.y())
    }
}

impl std::ops::Sub for &PointE {
    type Output = PointE;
    /// `operator -(PointE a, PointE b)`
    fn sub(self, b: &PointE) -> PointE {
        PointE::new(self.x() - b.x(), self.y() - b.y())
    }
}

impl std::ops::Mul for &PointE {
    type Output = PointE;
    /// `operator *(PointE a, PointE b)`
    fn mul(self, b: &PointE) -> PointE {
        PointE::new(self.x() * b.x(), self.y() * b.y())
    }
}

impl std::ops::Mul<f64> for &PointE {
    type Output = PointE;
    /// `operator *(PointE a, double muliplier)`
    fn mul(self, m: f64) -> PointE {
        PointE::new(self.x() * m, self.y() * m)
    }
}

impl std::ops::Div for &PointE {
    type Output = PointE;
    /// `operator /(PointE a, PointE b)`
    fn div(self, b: &PointE) -> PointE {
        PointE::new(self.x() / b.x(), self.y() / b.y())
    }
}

impl std::ops::Div<f64> for &PointE {
    type Output = PointE;
    /// `operator /(PointE a, double divisor)`
    fn div(self, d: f64) -> PointE {
        PointE::new(self.x() / d, self.y() / d)
    }
}

impl std::fmt::Display for PointE {
    /// `ToString()` in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PointE: ({}, {})", double_to_string(self.x()), double_to_string(self.y()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_math() {
        let a = PointE::new(3.0, 4.0);
        assert_eq!(a.magnitude(), 5.0);
        assert_eq!(a.dist(&PointE::new(0.0, 0.0)), 5.0);
        let n = a.get_normalized();
        assert_eq!((n.x(), n.y()), (0.6, 0.8));
        let r = PointE::new(1.0, 0.0).transform_by_rotation_about(std::f64::consts::PI, &PointE::new(0.0, 0.0));
        assert!((r.x() + 1.0).abs() < 1e-12);
        assert_eq!(PointE::new(-2.7, 3.9).get_hash_code(), -2 ^ 3);
        assert!(a.has_same_points_tol(&PointE::new(3.05, 4.05), 0.1));
        assert!(!a.has_same_points_tol(&PointE::new(3.5, 4.0), 0.1));
        // the SizeE cast goes through float
        assert_eq!(PointE::new(0.1, 0.0).to_size().width(), 0.1f32 as f64);
        assert!(a.equals_tol(&PointE::new(3.0, 4.0), 0.001));
    }

    #[test]
    fn equals_and_serialize() {
        let a = PointE::new(1.5, -2.0);
        assert!(a.equals(Some(&PointE::new(1.5, -2.0) as &dyn Any)).unwrap());
        assert!(!a.equals(Some(&5i32 as &dyn Any)).unwrap());
        assert!(a.equals(None).is_err());
        assert_eq!(IPoint::serialize(&a), "1.5,-2");
        let mut b = PointE::default();
        assert!(IPoint::deserialize(&mut b, Some("7")).is_err());
        assert_eq!(b.x(), 7.0);
        b.update_point(None);
        b.update_point(Some(&a));
        assert!(b.has_same_points(&a));
    }
}
