//! Port of `ExtLibs/Utilities/fitCurve.cs`
//!
//! (namespace `Simplify`)
//!
//! Douglas-Peucker polyline reduction on `System.Drawing.PointF`
//! (http://www.codeproject.com/KB/cs/Douglas-Peucker_Algorithm.aspx, Craig Selbert).

use crate::dotnet::drawing::PointF;
use crate::dotnet::Exception;

/// `public static class Douglas`
pub struct Douglas;

fn list_index_error() -> Exception {
    Exception::ArgumentOutOfRange(
        "Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into(),
    )
}

impl Douglas {
    /// `DouglasPeuckerReduction(List<PointF> Points, Double Tolerance)`: fewer than 3 points
    /// come back as they are. If every point equals the last one, the search for a distinct
    /// last point runs off the front of the list and throws, as in C#.
    pub fn douglas_peucker_reduction(points: &[PointF], tolerance: f64) -> Result<Vec<PointF>, Exception> {
        if points.len() < 3 {
            return Ok(points.to_vec());
        }

        let first_point: i32 = 0;
        let mut last_point: i32 = points.len() as i32 - 1;
        let mut point_indexs_to_keep: Vec<i32> = vec![];

        //Add the first and last index to the keepers
        point_indexs_to_keep.push(first_point);
        point_indexs_to_keep.push(last_point);

        //The first and the last point cannot be the same
        loop {
            let lp = points.get(usize::try_from(last_point).map_err(|_| list_index_error())?).ok_or_else(list_index_error)?;
            if points[first_point as usize] == *lp {
                last_point -= 1;
            } else {
                break;
            }
        }

        Self::reduce(points, first_point, last_point, tolerance, &mut point_indexs_to_keep);

        let mut return_points = vec![];
        point_indexs_to_keep.sort();
        for index in point_indexs_to_keep {
            return_points.push(points[index as usize]);
        }

        Ok(return_points)
    }

    /// The private recursive `DouglasPeuckerReduction(points, firstPoint, lastPoint, tolerance,
    /// ref pointIndexsToKeep)`.
    fn reduce(points: &[PointF], first_point: i32, last_point: i32, tolerance: f64, point_indexs_to_keep: &mut Vec<i32>) {
        let mut max_distance = 0.0;
        let mut index_farthest: i32 = 0;

        for index in first_point..last_point {
            let distance =
                Self::perpendicular_distance(points[first_point as usize], points[last_point as usize], points[index as usize]);
            if distance > max_distance {
                max_distance = distance;
                index_farthest = index;
            }
        }

        if max_distance > tolerance && index_farthest != 0 {
            //Add the largest point that exceeds the tolerance
            point_indexs_to_keep.push(index_farthest);

            Self::reduce(points, first_point, index_farthest, tolerance, point_indexs_to_keep);
            Self::reduce(points, index_farthest, last_point, tolerance, point_indexs_to_keep);
        }
    }

    /// `PerpendicularDistance(Point1, Point2, Point)`: twice the triangle area over the base.
    /// The area sum is single-precision `float` arithmetic, as in C#.
    pub fn perpendicular_distance(point1: PointF, point2: PointF, point: PointF) -> f64 {
        //Area = |(1/2)(x1y2 + x2y3 + x3y1 - x2y1 - x3y2 - x1y3)|   *Area of triangle
        //Base = v((x1-x2)²+(x1-x2)²)                               *Base of Triangle*
        //Area = .5*Base*H                                          *Solve for height
        //Height = Area/.5/Base

        let sum: f32 = point1.X * point2.Y + point2.X * point.Y + point.X * point1.Y
            - point2.X * point1.Y
            - point.X * point2.Y
            - point1.X * point.Y;
        let area = (0.5 * sum as f64).abs();
        let bottom = ((point1.X - point2.X) as f64).powf(2.0) + ((point1.Y - point2.Y) as f64).powf(2.0);
        let bottom = bottom.sqrt();
        area / bottom * 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reduces_a_straight_line() {
        let pts: Vec<PointF> = (0..5).map(|i| PointF::new(i as f32, 0.0)).collect();
        let r = Douglas::douglas_peucker_reduction(&pts, 0.1).unwrap();
        assert_eq!(r, vec![PointF::new(0.0, 0.0), PointF::new(4.0, 0.0)]);
        let mut bent = pts.clone();
        bent[2] = PointF::new(2.0, 5.0);
        // the peak at index 2 is kept, and then 1 and 3 are each ~0.93 off their sub-chords
        assert_eq!(Douglas::douglas_peucker_reduction(&bent, 0.1).unwrap().len(), 5);
        assert_eq!(Douglas::douglas_peucker_reduction(&bent, 1.0).unwrap().len(), 3);
        let same = vec![PointF::new(1.0, 1.0); 3];
        assert!(Douglas::douglas_peucker_reduction(&same, 0.1).is_err());
    }
}
