//! Survey (Grid) generator, a port of the lawnmower pattern in `ExtLibs/Utilities/Grid.cs`:
//! parallel lines at `spacing` metres, rotated by `angle`, clipped to the polygon, with
//! alternating direction and optional overshoot at each end.

use crate::geo::PointLatLngAlt;

const R: f64 = 6_378_137.0;

fn to_local(origin: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    let x = (p.1 - origin.1).to_radians() * R * origin.0.to_radians().cos();
    let y = (p.0 - origin.0).to_radians() * R;
    (x, y)
}

fn from_local(origin: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    let lat = origin.0 + (p.1 / R).to_degrees();
    let lng = origin.1 + (p.0 / (R * origin.0.to_radians().cos())).to_degrees();
    (lat, lng)
}

fn rot(p: (f64, f64), a: f64) -> (f64, f64) {
    let (s, c) = a.sin_cos();
    (p.0 * c - p.1 * s, p.0 * s + p.1 * c)
}

/// Returns the survey waypoints (lat, lng) in flight order.
pub fn grid(polygon: &[(f64, f64)], spacing: f64, angle_deg: f64, overshoot: f64) -> Vec<(f64, f64)> {
    if polygon.len() < 3 || spacing <= 0.1 {
        return Vec::new();
    }
    let n = polygon.len() as f64;
    let origin = (polygon.iter().map(|p| p.0).sum::<f64>() / n, polygon.iter().map(|p| p.1).sum::<f64>() / n);
    // rotate the polygon so grid lines are horizontal
    let a = -angle_deg.to_radians();
    let local: Vec<(f64, f64)> = polygon.iter().map(|p| rot(to_local(origin, *p), a)).collect();
    let (miny, maxy) = local.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.1), hi.max(p.1)));
    let mut out = Vec::new();
    let mut y = miny + spacing / 2.0;
    let mut flip = false;
    while y < maxy {
        let mut xs: Vec<f64> = Vec::new();
        for i in 0..local.len() {
            let (p1, p2) = (local[i], local[(i + 1) % local.len()]);
            if (p1.1 <= y && p2.1 > y) || (p2.1 <= y && p1.1 > y) {
                xs.push(p1.0 + (y - p1.1) / (p2.1 - p1.1) * (p2.0 - p1.0));
            }
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut segs: Vec<(f64, f64)> = xs.chunks(2).filter(|c| c.len() == 2).map(|c| (c[0] - overshoot, c[1] + overshoot)).collect();
        if flip {
            segs.reverse();
        }
        for (x0, x1) in segs {
            let (s, e) = if flip { (x1, x0) } else { (x0, x1) };
            out.push(from_local(origin, rot((s, y), -a)));
            out.push(from_local(origin, rot((e, y), -a)));
        }
        flip = !flip;
        y += spacing;
    }
    out
}

/// Polygon area in m² (shoelace in a local tangent plane).
pub fn area(polygon: &[(f64, f64)]) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    let o = polygon[0];
    let l: Vec<(f64, f64)> = polygon.iter().map(|p| to_local(o, *p)).collect();
    let mut a = 0.0;
    for i in 0..l.len() {
        let (p, q) = (l[i], l[(i + 1) % l.len()]);
        a += p.0 * q.1 - q.0 * p.1;
    }
    a.abs() / 2.0
}

/// "Create WP Circle": `points` waypoints on a circle of `radius` metres.
pub fn circle(center: (f64, f64), radius: f64, points: usize) -> Vec<(f64, f64)> {
    let c = PointLatLngAlt::new(center.0, center.1, 0.0);
    (0..points.max(3))
        .map(|i| {
            let p = c.newpos(360.0 * i as f64 / points.max(3) as f64, radius);
            (p.lat, p.lng)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_grid() {
        let c = PointLatLngAlt::new(-35.36, 149.16, 0.0);
        let a = c.newpos(0.0, 100.0);
        let b = a.newpos(90.0, 100.0);
        let d = c.newpos(90.0, 100.0);
        let poly = [(c.lat, c.lng), (a.lat, a.lng), (b.lat, b.lng), (d.lat, d.lng)];
        let wps = grid(&poly, 20.0, 0.0, 0.0);
        assert_eq!(wps.len(), 10); // 5 lines x 2 ends
        assert!((area(&poly) - 10_000.0).abs() < 100.0);
    }
}
