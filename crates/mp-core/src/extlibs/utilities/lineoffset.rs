//! Port of `ExtLibs/Utilities/LineOffset.cs`

use super::pointlatlngalt::PointLatLngAlt;

/// `public class LineOffset`
pub struct LineOffset;

impl LineOffset {
    /// `GetPolygon(List<PointLatLngAlt> polyline, int distm)`: a corridor polygon `distm`
    /// meters either side of the polyline; empty for 3 points or fewer, or when fewer than two
    /// points are far enough apart.
    pub fn get_polygon(polyline: &[PointLatLngAlt], distm: i32) -> Vec<PointLatLngAlt> {
        if polyline.len() <= 3 {
            return vec![];
        }

        let mut leftoffsetpoints = vec![];

        let mut rightoffsetpoints = vec![];

        let mut prevpoint = polyline[0].clone();

        // generate a point list for all points
        for point in polyline {
            // operator == (Equals: Lat, Lng, Alt, color, Tag)
            if *point == prevpoint {
                continue;
            }

            let dist = prevpoint.get_distance(point);
            if dist < (distm as f64 * 1.1) {
                continue;
            }

            let bearing = prevpoint.get_bearing(point);

            leftoffsetpoints.push(point.newpos(bearing - 90.0, distm as f64));
            rightoffsetpoints.push(point.newpos(bearing + 90.0, distm as f64));

            prevpoint = point.clone();
        }

        if leftoffsetpoints.len() <= 1 {
            return vec![];
        }

        let mut polygon_points = vec![];

        polygon_points.extend(leftoffsetpoints);

        rightoffsetpoints.reverse();

        polygon_points.extend(rightoffsetpoints);

        polygon_points
    }
}
