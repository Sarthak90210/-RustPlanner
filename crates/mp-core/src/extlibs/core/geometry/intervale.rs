//! Port of `ExtLibs/Core/Geometry/IntervalE.cs`

use crate::dotnet::{double_to_string, math_max, math_min};

/// `public class IntervalE`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct IntervalE {
    m_maximum: f64,
    m_minimum: f64,
}

impl IntervalE {
    /// `new IntervalE(double min, double max)`
    pub fn new(min: f64, max: f64) -> Self {
        IntervalE { m_minimum: min, m_maximum: max }
    }

    /// `Dimension` getter
    pub fn dimension(&self) -> f64 {
        self.m_maximum - self.m_minimum
    }

    /// `Dimension` setter: moves the maximum.
    pub fn set_dimension(&mut self, value: f64) {
        self.m_maximum = self.m_minimum + value;
    }

    /// `Minimum`
    pub fn minimum(&self) -> f64 {
        self.m_minimum
    }

    /// `Minimum` setter
    pub fn set_minimum(&mut self, value: f64) {
        self.m_minimum = value;
    }

    /// `Maximum`
    pub fn maximum(&self) -> f64 {
        self.m_maximum
    }

    /// `Maximum` setter
    pub fn set_maximum(&mut self, value: f64) {
        self.m_maximum = value;
    }

    /// `IsEmpty`
    pub fn is_empty(&self) -> bool {
        self.dimension() == 0.0
    }

    /// `IntersectsWith(IntervalE b)`
    pub fn intersects_with(&self, b: &IntervalE) -> bool {
        if b.minimum() <= self.minimum() {
            b.contains(self.minimum())
        } else {
            b.contains(self.maximum())
        }
    }

    /// `Contains(double testPos)`
    pub fn contains(&self, test_pos: f64) -> bool {
        test_pos >= self.minimum() && test_pos <= self.maximum()
    }

    /// `Contains(IntervalE testInterval)`
    pub fn contains_interval(&self, test_interval: &IntervalE) -> bool {
        test_interval.minimum() >= self.minimum() && test_interval.maximum() <= self.maximum()
    }

    /// `Union(double uPos)`
    pub fn union_pos(&self, u_pos: f64) -> IntervalE {
        IntervalE::new(math_min(self.minimum(), u_pos), math_max(self.maximum(), u_pos))
    }

    /// `Union(IntervalE b)`
    pub fn union(&self, b: &IntervalE) -> IntervalE {
        IntervalE::new(math_min(self.minimum(), b.minimum()), math_max(self.maximum(), b.maximum()))
    }

    /// `Intersect(IntervalE b)`: takes the other interval's far end without checking overlap.
    pub fn intersect(&self, b: &IntervalE) -> IntervalE {
        if b.minimum() < self.minimum() {
            IntervalE::new(self.minimum(), b.maximum())
        } else {
            IntervalE::new(b.minimum(), self.maximum())
        }
    }
}

impl std::fmt::Display for IntervalE {
    /// `ToString()` in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "IntervalE ({}, {})", double_to_string(self.minimum()), double_to_string(self.maximum()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interval_ops() {
        let a = IntervalE::new(0.0, 10.0);
        let b = IntervalE::new(5.0, 20.0);
        assert!(a.intersects_with(&b));
        assert!(!a.intersects_with(&IntervalE::new(11.0, 12.0)));
        assert_eq!(a.intersect(&b), IntervalE::new(5.0, 10.0));
        // no overlap check: the result is inverted
        assert_eq!(a.intersect(&IntervalE::new(20.0, 30.0)), IntervalE::new(20.0, 10.0));
        assert_eq!(a.union(&b), IntervalE::new(0.0, 20.0));
        assert!(a.union_pos(f64::NAN).minimum().is_nan());
        let mut c = IntervalE::new(1.0, 2.0);
        c.set_dimension(5.0);
        assert_eq!(c.maximum(), 6.0);
        assert!(IntervalE::default().is_empty());
        assert!(a.contains_interval(&IntervalE::new(1.0, 2.0)));
    }
}
