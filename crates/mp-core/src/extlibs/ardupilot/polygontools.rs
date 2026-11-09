//! Port of `ExtLibs/ArduPilot/PolygonTools.cs`
//!
//! Point-in-polygon by ray casting
//! (https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/).

use crate::dotnet::{math_max, math_min};

/// `public class PolygonTools`
pub struct PolygonTools;

/// `public class Point`
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }
}

impl PolygonTools {
    /// `onSegment(p, q, r)`: for colinear points, whether q lies on segment pr.
    #[allow(non_snake_case)]
    pub fn onSegment(p: &Point, q: &Point, r: &Point) -> bool {
        q.x <= math_max(p.x, r.x) && q.x >= math_min(p.x, r.x) && q.y <= math_max(p.y, r.y) && q.y >= math_min(p.y, r.y)
    }

    /// `orientation(p, q, r)`: 0 colinear, 1 clockwise, 2 counterclockwise (NaN counts as
    /// counterclockwise).
    pub fn orientation(p: &Point, q: &Point, r: &Point) -> i32 {
        let val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

        if val == 0.0 {
            return 0; // colinear
        }

        if val > 0.0 {
            1
        } else {
            2
        } // clock or counterclock wise
    }

    /// `doIntersect(p1, q1, p2, q2)`: whether segments p1q1 and p2q2 intersect.
    #[allow(non_snake_case)]
    pub fn doIntersect(p1: &Point, q1: &Point, p2: &Point, q2: &Point) -> bool {
        // Find the four orientations needed for general and special cases
        let o1 = Self::orientation(p1, q1, p2);
        let o2 = Self::orientation(p1, q1, q2);
        let o3 = Self::orientation(p2, q2, p1);
        let o4 = Self::orientation(p2, q2, q1);

        // General case
        if o1 != o2 && o3 != o4 {
            return true;
        }

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if o1 == 0 && Self::onSegment(p1, p2, q1) {
            return true;
        }

        // p1, q1 and p2 are colinear and q2 lies on segment p1q1
        if o2 == 0 && Self::onSegment(p1, q2, q1) {
            return true;
        }

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if o3 == 0 && Self::onSegment(p2, p1, q2) {
            return true;
        }

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if o4 == 0 && Self::onSegment(p2, q1, q2) {
            return true;
        }

        // Doesn't fall in any of the above cases
        false
    }

    /// `isInside(List<Point> polygon, Point p)`: ray cast to x = 360; a point on an edge
    /// counts as inside.
    #[allow(non_snake_case)]
    pub fn isInside(polygon: &[Point], p: &Point) -> bool {
        let n = polygon.len();
        // There must be at least 3 vertices in polygon[]
        if n < 3 {
            return false;
        }

        // Create a point for line segment from p to infinite
        let extreme = Point::new(360.0, p.y);

        // Count intersections of the above line with sides of polygon
        let mut count = 0;
        let mut i = 0;
        loop {
            let next = (i + 1) % n;

            // Check if the line segment from 'p' to 'extreme' intersects with the line segment
            // from 'polygon[i]' to 'polygon[next]'
            if Self::doIntersect(&polygon[i], &polygon[next], p, &extreme) {
                // If the point 'p' is colinear with line segment 'i-next', then check if it
                // lies on segment. If it lies, return true, otherwise false
                if Self::orientation(&polygon[i], p, &polygon[next]) == 0 {
                    return Self::onSegment(&polygon[i], p, &polygon[next]);
                }

                count += 1;
            }

            i = next;
            if i == 0 {
                break;
            }
        }

        // Return true if count is odd, false otherwise
        count % 2 == 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inside_square() {
        let sq = [Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(10.0, 10.0), Point::new(0.0, 10.0)];
        assert!(PolygonTools::isInside(&sq, &Point::new(5.0, 5.0)));
        assert!(!PolygonTools::isInside(&sq, &Point::new(15.0, 5.0)));
        assert!(PolygonTools::isInside(&sq, &Point::new(10.0, 5.0)));
        assert!(!PolygonTools::isInside(&sq[..2], &Point::new(5.0, 5.0)));
    }
}
