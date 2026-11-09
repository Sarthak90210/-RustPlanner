//! Port of `ExtLibs/Core/Geometry/Calc.cs`

use super::pointe::PointE;

/// `public class Calc`
pub struct Calc;

impl Calc {
    /// `PointBetween(PointE p1, PointE p2, double frac)`
    pub fn point_between(p1: &PointE, p2: &PointE, frac: f64) -> PointE {
        PointE::new(p1.x() + (p2.x() - p1.x()) * frac, p1.y() + (p2.y() - p1.y()) * frac)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn between() {
        let p = Calc::point_between(&PointE::new(0.0, 0.0), &PointE::new(10.0, -4.0), 0.25);
        assert_eq!((p.x(), p.y()), (2.5, -1.0));
    }
}
