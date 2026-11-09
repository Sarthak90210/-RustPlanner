//! Port of `ExtLibs/Core/Geometry/AShapeE.cs`
//!
//! `IShapeE` and `IShapeEUpdateable` are traits. Every shape in Core derives from the abstract
//! `AShapeE`, so its state (the two events and `Tag`) is the `AShapeE` struct each shape
//! embeds, and its virtual members are the traits' provided methods. The C# `EventHandler`
//! sender (`this`) is passed as `&dyn Any`; `EventArgs.Empty` carries nothing and is dropped.

use std::any::Any;
use std::sync::Arc;

use super::pointe::PointE;
use super::rectanglee::RectangleE;
use crate::dotnet::Exception;
use crate::extlibs::comms::commsbase::StaticEvent;

/// `EventHandler` as `GeometryChanged` raises it: `(sender, EventArgs.Empty)`.
pub type EventHandler = dyn Fn(&dyn Any) + Send + Sync;

/// `public delegate void ShapeUpdateEventHandler(object sender, ShapeUpdateEventArgs e)`
pub type ShapeUpdateEventHandler = dyn for<'a> Fn(&dyn Any, &ShapeUpdateEventArgs<'a>) + Send + Sync;

/// `object Tag`
pub type Tag = Arc<dyn Any + Send + Sync>;

/// `public class ShapeUpdateEventArgs : EventArgs`
#[allow(non_snake_case)]
pub struct ShapeUpdateEventArgs<'a> {
    pub OldShape: &'a dyn IShapeE,
    pub NewShape: Option<&'a dyn IShapeE>,
}

impl<'a> ShapeUpdateEventArgs<'a> {
    /// `new ShapeUpdateEventArgs(IShapeE oldShape, IShapeE newShape)`
    pub fn new(old_shape: &'a dyn IShapeE, new_shape: Option<&'a dyn IShapeE>) -> Self {
        ShapeUpdateEventArgs { OldShape: old_shape, NewShape: new_shape }
    }
}

/// The fields of `public abstract class AShapeE`.
#[allow(non_snake_case)]
#[derive(Default)]
pub struct AShapeE {
    /// `public event EventHandler GeometryChanged`
    pub GeometryChanged: StaticEvent<EventHandler>,
    /// `public event ShapeUpdateEventHandler ShapeUpdated`
    pub ShapeUpdated: StaticEvent<ShapeUpdateEventHandler>,
    m_tag: Option<Tag>,
}

impl std::fmt::Debug for AShapeE {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AShapeE").field("has_tag", &self.m_tag.is_some()).finish()
    }
}

impl AShapeE {
    /// `AShapeE.Compare(IShapeE a, IShapeE b)`: `a.Equals(b)` runs first, so a null `a`
    /// throws; then shapes without a bounding box sort first; then by area, descending.
    pub fn compare(a: Option<&dyn IShapeE>, b: Option<&dyn IShapeE>) -> Result<i32, Exception> {
        let a = a.ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        if a.equals(b.map(|b| b.as_any()))? {
            return Ok(0);
        }
        let Some(a_box) = a.bounding_box() else { return Ok(-1) };
        let Some(b_box) = b.and_then(|b| b.bounding_box()) else { return Ok(1) };
        Ok(b_box.compare_to(&a_box))
    }
}

/// `public interface IShapeE`, with `AShapeE`'s implementation of each member.
pub trait IShapeE: Any {
    /// The embedded `AShapeE` state.
    fn shape_base(&self) -> &AShapeE;
    /// Mutable access to the embedded `AShapeE` state.
    fn shape_base_mut(&mut self) -> &mut AShapeE;
    /// `this` as `object`.
    fn as_any(&self) -> &dyn Any;

    /// `RectangleE BoundingBox { get; }` (abstract)
    fn bounding_box(&self) -> Option<RectangleE>;

    /// `IShapeE Shift(PointE point)` (abstract)
    fn shift(&self, point: &PointE) -> Box<dyn IShapeE>;

    /// `virtual bool ContainsPoint(PointE point)`
    fn contains_point(&self, _point: &PointE) -> bool {
        false
    }

    /// `bool MostlyContains(IShapeE shapeE)`: the 0.95 overload.
    fn mostly_contains(&self, shape_e: Option<&dyn IShapeE>) -> Result<bool, Exception> {
        self.mostly_contains_tol(shape_e, 0.95)
    }

    /// `virtual bool MostlyContains(IShapeE shapeE, double tolerance)`
    fn mostly_contains_tol(&self, _shape_e: Option<&dyn IShapeE>, _tolerance: f64) -> Result<bool, Exception> {
        Ok(false)
    }

    /// `virtual bool FullyContains(IShapeE shapeE)`
    fn fully_contains(&self, _shape_e: Option<&dyn IShapeE>) -> Result<bool, Exception> {
        Ok(false)
    }

    /// `object.Equals(object obj)`: reference equality unless overridden.
    fn equals(&self, obj: Option<&dyn Any>) -> Result<bool, Exception> {
        Ok(obj.is_some_and(|o| std::ptr::addr_eq(o as *const dyn Any, self.as_any() as *const dyn Any)))
    }

    /// `object Tag { get; }`
    fn tag(&self) -> Option<Tag> {
        self.shape_base().m_tag.clone()
    }

    /// `object Tag { set; }`
    fn set_tag(&mut self, value: Option<Tag>) {
        self.shape_base_mut().m_tag = value;
    }

    /// `protected void CallGeometryChanged()`
    fn call_geometry_changed(&self) {
        for h in self.shape_base().GeometryChanged.list() {
            h(self.as_any());
        }
    }
}

/// `public interface IShapeEUpdateable : IShapeE`
pub trait IShapeEUpdateable: IShapeE {
    /// `public void UpdateShape(IShapeE newShape)`: `ShapeUpdated(this, (this, newShape))`.
    fn update_shape(&self, new_shape: Option<&dyn IShapeE>)
    where
        Self: Sized,
    {
        let list = self.shape_base().ShapeUpdated.list();
        if list.is_empty() {
            return;
        }
        let args = ShapeUpdateEventArgs::new(self, new_shape);
        for h in list {
            h(self.as_any(), &args);
        }
    }
}

impl<T: IShapeE> IShapeEUpdateable for T {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    #[test]
    fn compare_order() {
        let p = PointE::new(1.0, 1.0);
        let small = RectangleE::new(0.0, 0.0, 1.0, 1.0);
        let big = RectangleE::new(0.0, 0.0, 5.0, 5.0);
        assert!(AShapeE::compare(None, Some(&p)).is_err());
        // PointE.Equals(null) dereferences the null argument
        assert!(AShapeE::compare(Some(&p), None).is_err());
        assert_eq!(AShapeE::compare(Some(&p), Some(&small)).unwrap(), -1);
        assert_eq!(AShapeE::compare(Some(&small), Some(&p)).unwrap(), 1);
        assert_eq!(AShapeE::compare(Some(&small), None).unwrap(), 1);
        assert_eq!(AShapeE::compare(Some(&small), Some(&small)).unwrap(), 0);
        assert_eq!(AShapeE::compare(Some(&small), Some(&big)).unwrap(), 1);
        assert_eq!(AShapeE::compare(Some(&big), Some(&small)).unwrap(), -1);
        // reference equality for rectangles: an equal copy is still compared by area
        assert_eq!(AShapeE::compare(Some(&small), Some(&small.clone_rect())).unwrap(), 0);
    }

    #[test]
    fn events_and_tag() {
        let hits = Arc::new(Mutex::new(0));
        let mut p = PointE::new(0.0, 0.0);
        let h = hits.clone();
        p.shape_base().GeometryChanged.add(Arc::new(move |s: &dyn Any| {
            assert!(s.downcast_ref::<PointE>().is_some());
            *h.lock().unwrap() += 1;
        }));
        p.set_x(2.0);
        p.set_y(3.0);
        assert_eq!(*hits.lock().unwrap(), 2);
        let seen = Arc::new(Mutex::new(false));
        let s2 = seen.clone();
        p.shape_base().ShapeUpdated.add(Arc::new(move |_: &dyn Any, e: &ShapeUpdateEventArgs<'_>| {
            *s2.lock().unwrap() = e.NewShape.is_none() && e.OldShape.bounding_box().is_none();
        }));
        p.update_shape(None);
        assert!(*seen.lock().unwrap());
        p.set_tag(Some(Arc::new(5i32)));
        assert_eq!(p.tag().unwrap().downcast_ref::<i32>(), Some(&5));
        assert!(!p.contains_point(&PointE::new(0.0, 0.0)));
        assert!(!p.mostly_contains(None).unwrap());
    }
}
