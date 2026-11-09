//! The app's survey helpers: the lane generator is a layer over the port of
//! `ExtLibs/Utilities/Grid.cs` (`Grid.CreateGrid`); polygon area and the "Create WP Circle"
//! points.

use crate::geo::PointLatLngAlt;

const R: f64 = 6_378_137.0;

fn to_local(origin: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    let x = (p.1 - origin.1).to_radians() * R * origin.0.to_radians().cos();
    let y = (p.0 - origin.0).to_radians() * R;
    (x, y)
}

/// The survey lanes (lat, lng) in flight order, through the port of `Grid.CreateGrid`
/// (`extlibs::utilities::grid`) with SimpleGridUI's argument pattern: lanes `spacing` metres
/// apart at bearing `angle_deg`, no trigger points, `overshoot` past both lane ends,
/// minLaneSeparation 0 and lead-ins 0, flown from the polygon's bottom-left corner. Each
/// lane gives its "S" and "E" points.
pub fn grid(polygon: &[(f64, f64)], spacing: f64, angle_deg: f64, overshoot: f64) -> Vec<(f64, f64)> {
    use crate::extlibs::utilities::grid::{Grid, StartPosition};
    use crate::extlibs::utilities::PointLatLngAlt as Plla;
    let poly: Vec<Plla> = polygon.iter().map(|&(lat, lng)| Plla::new(lat, lng)).collect();
    let home = poly.first().cloned().unwrap_or_else(Plla::zero);
    match Grid::CreateGrid(&poly, 0.0, spacing, 0.0, angle_deg, overshoot, overshoot, StartPosition::BottomLeft, false, 0.0, 0.0, 0.0, &home, true) {
        Ok(wps) => wps.iter().filter(|p| p.Tag == "S" || p.Tag == "E").map(|p| (p.Lat, p.Lng)).collect(),
        Err(e) => {
            log::error!("{e}");
            Vec::new()
        }
    }
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
        // about 5 north-south lanes x 2 ends (UTM convergence tilts the square slightly)
        assert!(wps.len() % 2 == 0 && (8..=12).contains(&wps.len()), "{}", wps.len());
        assert!((area(&poly) - 10_000.0).abs() < 100.0);
    }
}
