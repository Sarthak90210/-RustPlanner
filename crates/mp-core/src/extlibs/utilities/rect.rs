//! Port of `ExtLibs/Utilities/Rect.cs`

/// `public struct Rect`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Rect {
    pub Top: f64,
    pub Bottom: f64,
    pub Left: f64,
    pub Right: f64,
}

#[allow(non_snake_case)]
impl Rect {
    /// `Rect(Left, Top, Width, Height)`: Right = Left + Width, Bottom = Top + Height.
    pub fn new(Left: f64, Top: f64, Width: f64, Height: f64) -> Self {
        Rect { Left, Top, Right: Left + Width, Bottom: Top + Height }
    }

    pub fn width(&self) -> f64 {
        self.Right - self.Left
    }

    pub fn height(&self) -> f64 {
        self.Top - self.Bottom
    }

    pub fn mid_width(&self) -> f64 {
        ((self.Right - self.Left) / 2.0) + self.Left
    }

    pub fn mid_height(&self) -> f64 {
        ((self.Top - self.Bottom) / 2.0) + self.Bottom
    }

    /// `DiagDistance()`: pythagoras on Width and Height.
    pub fn diag_distance(&self) -> f64 {
        (self.width().powi(2) + self.height().powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn height_is_top_minus_bottom() {
        let r = Rect::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(r.Right, 4.0);
        assert_eq!(r.Bottom, 6.0);
        assert_eq!(r.height(), -4.0);
        assert_eq!(r.diag_distance(), 5.0);
    }
}
