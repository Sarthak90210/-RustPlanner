//! Port of `ExtLibs/Utilities/Grid.cs`
//!
//! Survey grid, corridor and rotary patterns in UTM. The C# value semantics are kept where
//! they decide the result: `List.Remove`/`IndexOf` and the `Dictionary<utmpos, ...>` use
//! `utmpos.Equals` (x and y only) and `GetHashCode` (x, y, zone), `linelatlng` compares field
//! by field through `utmpos.Equals`, an `utmpos` added to a `List<PointLatLngAlt>` goes
//! through the implicit `ToLLA()` conversion, and `ReversePaths(PolyTreeToPaths(tree))`
//! reverses the tree's own contours. `addtomap` is the C# no-op debug hook.

#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]

use std::sync::{Arc, RwLock};

use super::clipper::{Clipper, ClipperOffset, EndType, IntPoint, JoinType, PolyTree};
use super::pointlatlngalt::PointLatLngAlt;
use super::rect::Rect;
use super::utmpos::{utmpos, TagToString};
use crate::dotnet::Exception;

const rad2deg: f64 = 180.0 / std::f64::consts::PI;
const deg2rad: f64 = 1.0 / rad2deg;

fn list_index() -> Exception {
    Exception::ArgumentOutOfRange("Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into())
}

fn tag(s: &str) -> Option<Arc<dyn TagToString>> {
    Some(Arc::new(s.to_string()))
}

/// `utmpos` with its `Tag` set (`new utmpos(...) { Tag = s }`).
fn tagged(mut p: utmpos, s: &str) -> utmpos {
    p.tag = tag(s);
    p
}

/// `public struct linelatlng`
#[derive(Debug, Clone, Default)]
pub struct linelatlng {
    /// start of line
    pub p1: utmpos,
    /// end of line
    pub p2: utmpos,
    /// used as a base for grid along line (initial setout)
    pub basepnt: utmpos,
}

impl linelatlng {
    /// `ValueType.Equals`: each field through `utmpos.Equals` (x and y).
    fn value_equals(&self, o: &linelatlng) -> bool {
        self.p1.equals(&o.p1) && self.p2.equals(&o.p2) && self.basepnt.equals(&o.basepnt)
    }
}

crate::cs_enum! {
    /// `public enum StartPosition`
    pub StartPosition: i32 {
        Home = 0,
        BottomLeft = 1,
        TopLeft = 2,
        BottomRight = 3,
        TopRight = 4,
        Point = 5,
    }
}

/// `public static PointLatLngAlt StartPointLatLngAlt = PointLatLngAlt.Zero`
static START_POINT: RwLock<Option<PointLatLngAlt>> = RwLock::new(None);

/// `List<T>.Remove(item)`: the first element equal to `item`.
fn remove_first<T>(list: &mut Vec<T>, eq: impl Fn(&T) -> bool) -> bool {
    match list.iter().position(eq) {
        Some(i) => {
            list.remove(i);
            true
        }
        None => false,
    }
}

/// `public class Grid`
pub struct Grid;

impl Grid {
    /// `StartPointLatLngAlt`
    pub fn StartPointLatLngAlt() -> PointLatLngAlt {
        START_POINT.read().unwrap_or_else(|e| e.into_inner()).clone().unwrap_or_else(PointLatLngAlt::zero)
    }

    pub fn set_StartPointLatLngAlt(p: PointLatLngAlt) {
        *START_POINT.write().unwrap_or_else(|e| e.into_inner()) = Some(p);
    }

    fn addtomap_line(_pos: &linelatlng) {}

    fn addtomap(_pos: &utmpos, _tag: &str) {}

    /// The polygon in UTM, all in the first point's zone.
    fn to_utm(polygon: &[PointLatLngAlt], utmzone: i32) -> Result<Vec<utmpos>, Exception> {
        utmpos::to_list(&PointLatLngAlt::to_utm_list(utmzone, polygon)?, utmzone)
    }

    /// `CreateCorridor(polygon, altitude, distance, spacing, angle, overshoot1, overshoot2,
    /// startpos, shutter, minLaneSeparation, width, leadin = 0)`
    #[allow(clippy::too_many_arguments)]
    pub fn CreateCorridor(
        polygon: &[PointLatLngAlt],
        altitude: f64,
        mut distance: f64,
        mut spacing: f64,
        _angle: f64,
        _overshoot1: f64,
        _overshoot2: f64,
        startpos: StartPosition,
        _shutter: bool,
        _minLaneSeparation: f32,
        width: f64,
        _leadin: f32,
    ) -> Result<Vec<PointLatLngAlt>, Exception> {
        if spacing < 4.0 && spacing != 0.0 {
            spacing = 4.0;
        }

        if distance < 0.1 {
            distance = 0.1;
        }

        if polygon.is_empty() {
            return Ok(Vec::new());
        }

        let mut ans: Vec<PointLatLngAlt> = Vec::new();

        // utm zone distance calcs will be done in
        let utmzone = polygon[0].get_utm_zone();

        // utm position list
        let mut utmpositions = Self::to_utm(polygon, utmzone)?;

        let lanes = width / distance;
        let start = crate::dotnet::cast_i32((lanes / 2.0) * -1.0);
        let end = start.wrapping_mul(-1);

        let mut lane = start;
        while lane <= end {
            // correct side of the line we are on because of list reversal
            let mut multi = 1;
            if (lane.wrapping_sub(start)) % 2 == 1 {
                multi = -1;
            }

            if startpos != StartPosition::Home {
                utmpositions.reverse();
            }

            for pnt in Self::GenerateOffsetPath(&utmpositions, distance * multi as f64 * lane as f64, spacing, utmzone) {
                ans.push(pnt.to_lla()?);
            }

            if startpos == StartPosition::Home {
                utmpositions.reverse();
            }
            lane += 1;
        }

        // set the altitude on all points
        for plla in ans.iter_mut() {
            plla.Alt = altitude;
        }

        Ok(ans)
    }

    fn GenerateOffsetPath(utmpositions: &[utmpos], distance: f64, spacing: f64, utmzone: i32) -> Vec<utmpos> {
        let mut ans = Vec::new();

        let mut oldpos = utmpos::zero();

        let count = utmpositions.len() as i64;
        let mut a: i64 = 0;
        while a < count - 2 {
            let au = a as usize;
            let prevCenter = &utmpositions[au];
            let currCenter = &utmpositions[au + 1];
            let nextCenter = &utmpositions[au + 2];

            let l1bearing = prevCenter.get_bearing(currCenter);
            let l2bearing = currCenter.get_bearing(nextCenter);

            let mut l1prev = Self::newpos_utm(prevCenter, l1bearing + 90.0, distance);
            let l1curr = Self::newpos_utm(currCenter, l1bearing + 90.0, distance);

            let l2curr = Self::newpos_utm(currCenter, l2bearing + 90.0, distance);
            let mut l2next = Self::newpos_utm(nextCenter, l2bearing + 90.0, distance);

            let mut l1l2center = Self::FindLineIntersectionExtension(&l1prev, &l1curr, &l2curr, &l2next);

            //start
            if a == 0 {
                // add start
                l1prev.tag = tag("S");
                ans.push(l1prev.clone());

                // add start/trigger
                l1prev.tag = tag("SM");
                ans.push(l1prev.clone());

                oldpos = l1prev.clone();
            }

            //spacing
            if spacing > 0.0 {
                let dist = oldpos.get_distance(&l1l2center);
                let mut d = crate::dotnet::cast_i32(dist % spacing);
                while (d as f64) < oldpos.get_distance(&l1l2center) {
                    let mut ax = oldpos.x;
                    let mut ay = oldpos.y;

                    Self::newpos(&mut ax, &mut ay, l1bearing, d as f64);
                    ans.push(tagged(utmpos::new(ax, ay, utmzone), "M"));
                    d = d.wrapping_add(crate::dotnet::cast_i32(spacing));
                }
            }

            //end of leg
            l1l2center.tag = tag("S");
            ans.push(l1l2center.clone());
            oldpos = l1l2center.clone();

            // last leg
            if (a + 3) == count {
                if spacing > 0.0 {
                    let mut d = crate::dotnet::cast_i32(l1l2center.get_distance(&l2next) % spacing);
                    while (d as f64) < l1l2center.get_distance(&l2next) {
                        let mut ax = l1l2center.x;
                        let mut ay = l1l2center.y;

                        Self::newpos(&mut ax, &mut ay, l2bearing, d as f64);
                        ans.push(tagged(utmpos::new(ax, ay, utmzone), "M"));
                        d = d.wrapping_add(crate::dotnet::cast_i32(spacing));
                    }
                }

                l2next.tag = tag("ME");
                ans.push(l2next.clone());

                l2next.tag = tag("E");
                ans.push(l2next.clone());
            }
            a += 1;
        }

        ans
    }

    /// `CreateRotary(polygon, altitude, distance, spacing, angle, overshoot1, overshoot2,
    /// startpos, shutter, minLaneSeparation, leadin, HomeLocation, clockwise_laps,
    /// match_spiral_perimeter, laps)`
    #[allow(clippy::too_many_arguments)]
    pub fn CreateRotary(
        polygon: &[PointLatLngAlt],
        altitude: f64,
        mut distance: f64,
        _spacing: f64,
        _angle: f64,
        _overshoot1: f64,
        _overshoot2: f64,
        startpos: StartPosition,
        _shutter: bool,
        _minLaneSeparation: f32,
        _leadin: f32,
        HomeLocation: &PointLatLngAlt,
        clockwise_laps: i32,
        match_spiral_perimeter: bool,
        laps: i32,
    ) -> Result<Vec<PointLatLngAlt>, Exception> {
        if distance < 0.1 {
            distance = 0.1;
        }

        if polygon.is_empty() {
            return Ok(Vec::new());
        }

        let mut ans: Vec<utmpos> = Vec::new();

        // utm zone distance calcs will be done in
        let utmzone = polygon[0].get_utm_zone();

        // utm position list
        let utmpositions = Self::to_utm(polygon, utmzone)?;

        // get mins/maxs of coverage area
        let area = Self::getPolyMinMax(&utmpositions);

        let maxlane = laps;

        // pick start positon based on initial point rectangle
        let mut startposutm = Self::start_position(startpos, &area, utmzone, HomeLocation)?;

        // find the closes polygon point based from our startpos selection
        startposutm = Self::findClosestPoint(&startposutm, &utmpositions);

        let mut clipperOffset = ClipperOffset::new(2.0, 0.25);

        let path: Vec<IntPoint> = utmpositions.iter().map(|a| IntPoint::from_f64(a.x * 1000.0, a.y * 1000.0)).collect();
        clipperOffset.AddPath(&path, JoinType::jtMiter, EndType::etClosedPolygon);

        for lane in 0..maxlane.max(0) {
            let mut tree = PolyTree::new();
            clipperOffset.Execute_tree(&mut tree, crate::dotnet::cast_i64(distance * 1000.0 * -(lane as f64)) as f64)?;

            if tree.ChildCount(PolyTree::ROOT) == 0 {
                break;
            }

            if lane < clockwise_laps || clockwise_laps < 0 {
                Clipper::ReversePolyTreePaths(&mut tree);
            }

            for &treeChild in tree.Childs(PolyTree::ROOT) {
                let mut ans1: Vec<utmpos> = tree.Contour(treeChild).iter().map(|a| utmpos::new(a.X as f64 / 1000.0, a.Y as f64 / 1000.0, utmzone)).collect();
                // rotate points so the start point is close to the previous
                {
                    startposutm = Self::findClosestPoint(&startposutm, &ans1);

                    // IndexOf: utmpos.Equals (x and y)
                    let startidx = ans1.iter().position(|p| p.equals(&startposutm)).ok_or_else(list_index)?;

                    let mut firsthalf = ans1[startidx..].to_vec();
                    let secondhalf = ans1[..startidx].to_vec();

                    firsthalf.extend(secondhalf);
                    ans1 = firsthalf;
                }
                if lane == 0 && clockwise_laps != 1 && match_spiral_perimeter {
                    // start at the last point of the first calculated lap
                    // to make a closed polygon on the first trip around
                    let last = ans1.last().cloned().ok_or_else(|| Exception::InvalidOperation("Sequence contains no elements".into()))?;
                    ans1.insert(0, last);
                }

                if lane == clockwise_laps.wrapping_sub(1) {
                    // revisit the first waypoint on this lap to cleanly exit the CW pattern
                    let first = ans1.first().cloned().ok_or_else(|| Exception::InvalidOperation("Sequence contains no elements".into()))?;
                    ans1.push(first);
                }

                ans.extend(ans1);
            }
        }

        // set the altitude on all points
        let mut out = Vec::with_capacity(ans.len());
        for plla in &ans {
            let mut a = plla.to_lla()?;
            a.Alt = altitude;
            a.Tag = "S".into();
            out.push(a);
        }
        Ok(out)
    }

    /// `startpos` to a UTM point (the `default:` falls into Home).
    fn start_position(startpos: StartPosition, area: &Rect, utmzone: i32, HomeLocation: &PointLatLngAlt) -> Result<utmpos, Exception> {
        Ok(match startpos {
            StartPosition::BottomLeft => utmpos::new(area.Left, area.Bottom, utmzone),
            StartPosition::BottomRight => utmpos::new(area.Right, area.Bottom, utmzone),
            StartPosition::TopLeft => utmpos::new(area.Left, area.Top, utmzone),
            StartPosition::TopRight => utmpos::new(area.Right, area.Top, utmzone),
            StartPosition::Point => utmpos::from_plla(&Self::StartPointLatLngAlt())?,
            _ => utmpos::from_plla(HomeLocation)?,
        })
    }

    #[allow(dead_code)]
    fn Centroid(poly: &[utmpos]) -> utmpos {
        let mut x = 0.0;
        let mut y = 0.0;
        let parts = poly.len() as f64;

        for a in poly {
            x += a.x / parts;
            y += a.y / parts;
        }

        utmpos::new(x, y, poly[0].zone)
    }

    /// `CreateGrid(polygon, altitude, distance, spacing, angle, overshoot1, overshoot2,
    /// startpos, shutter, minLaneSeparation, leadin1, leadin2, HomeLocation,
    /// useextendedendpoint = true)`: lines across the polygon at `distance` apart, flown
    /// boustrophedon from the start corner; `useextendedendpoint` picks the next line from
    /// the lead-out point.
    #[allow(clippy::too_many_arguments)]
    pub fn CreateGrid(
        polygon: &[PointLatLngAlt],
        altitude: f64,
        mut distance: f64,
        mut spacing: f64,
        angle: f64,
        overshoot1: f64,
        overshoot2: f64,
        startpos: StartPosition,
        _shutter: bool,
        mut minLaneSeparation: f32,
        leadin1: f32,
        leadin2: f32,
        HomeLocation: &PointLatLngAlt,
        useextendedendpoint: bool,
    ) -> Result<Vec<PointLatLngAlt>, Exception> {
        if spacing < 0.1 && spacing != 0.0 {
            spacing = 0.1;
        }

        if distance < 0.1 {
            distance = 0.1;
        }

        if polygon.is_empty() {
            return Ok(Vec::new());
        }

        // Make a non round number in case of corner cases
        if minLaneSeparation != 0.0 {
            minLaneSeparation += 0.5f32;
        }
        // Lane Separation in meters
        let minLaneSeparationINMeters = minLaneSeparation as f64 * distance;

        let mut ans: Vec<PointLatLngAlt> = Vec::new();

        // utm zone distance calcs will be done in
        let utmzone = polygon[0].get_utm_zone();

        // utm position list
        let mut utmpositions = Self::to_utm(polygon, utmzone)?;

        // close the loop if its not already (operator ==: x, y and zone)
        if utmpositions[0] != utmpositions[utmpositions.len() - 1] {
            let first = utmpositions[0].clone();
            utmpositions.push(first); // make a full loop
        }

        // get mins/maxs of coverage area
        let area = Self::getPolyMinMax(&utmpositions);

        // used to determine the size of the outer grid area
        let diagdist = area.diag_distance();

        // somewhere to store out generated lines
        let mut grid: Vec<linelatlng> = Vec::new();
        // number of lines we need
        let mut lines = 0;

        // get start point middle
        let mut x = area.mid_width();
        let mut y = area.mid_height();

        Self::addtomap(&utmpos::new(x, y, utmzone), "Base");

        // get left extent
        let mut xb1 = x;
        let mut yb1 = y;
        // to the left
        Self::newpos(&mut xb1, &mut yb1, angle - 90.0, diagdist / 2.0 + distance);
        // backwards
        Self::newpos(&mut xb1, &mut yb1, angle + 180.0, diagdist / 2.0 + distance);

        let left = utmpos::new(xb1, yb1, utmzone);

        Self::addtomap(&left, "left");

        // get right extent
        let mut xb2 = x;
        let mut yb2 = y;
        // to the right
        Self::newpos(&mut xb2, &mut yb2, angle + 90.0, diagdist / 2.0 + distance);
        // backwards
        Self::newpos(&mut xb2, &mut yb2, angle + 180.0, diagdist / 2.0 + distance);

        let right = utmpos::new(xb2, yb2, utmzone);

        Self::addtomap(&right, "right");

        // set start point to left hand side
        x = xb1;
        y = yb1;

        // draw the outergrid, this is a grid that cover the entire area of the rectangle plus more.
        while (lines as f64) < ((diagdist + distance * 2.0) / distance) {
            // copy the start point to generate the end point
            let mut nx = x;
            let mut ny = y;
            Self::newpos(&mut nx, &mut ny, angle, diagdist + distance * 2.0);

            grid.push(linelatlng { p1: utmpos::new(x, y, utmzone), p2: utmpos::new(nx, ny, utmzone), basepnt: utmpos::new(x, y, utmzone) });

            Self::newpos(&mut x, &mut y, angle + 90.0, distance);
            lines += 1;
        }

        // find intersections with our polygon

        // store lines that dont have any intersections
        let mut remove: Vec<linelatlng> = Vec::new();

        let gridno = grid.len();

        // cycle through our grid
        for a in 0..gridno {
            let mut closestdistance = f64::MAX;
            let mut farestdistance = f64::MIN;

            let mut closestpoint = utmpos::zero();
            let mut farestpoint = utmpos::zero();

            // somewhere to store our intersections
            let mut matchs: Vec<utmpos> = Vec::new();

            let mut crosses = 0;
            for b in 1..utmpositions.len() {
                let newutmpos = Self::FindLineIntersection(&utmpositions[b - 1], &utmpositions[b], &grid[a].p1, &grid[a].p2);
                if !newutmpos.is_zero() {
                    crosses += 1;
                    matchs.push(newutmpos.clone());
                    if closestdistance > grid[a].p1.get_distance(&newutmpos) {
                        closestpoint.y = newutmpos.y;
                        closestpoint.x = newutmpos.x;
                        closestpoint.zone = newutmpos.zone;
                        closestdistance = grid[a].p1.get_distance(&newutmpos);
                    }
                    if farestdistance < grid[a].p1.get_distance(&newutmpos) {
                        farestpoint.y = newutmpos.y;
                        farestpoint.x = newutmpos.x;
                        farestpoint.zone = newutmpos.zone;
                        farestdistance = grid[a].p1.get_distance(&newutmpos);
                    }
                }
            }
            if crosses == 0 {
                // outside our polygon
                if !Self::PointInPolygon(&grid[a].p1, &utmpositions) && !Self::PointInPolygon(&grid[a].p2, &utmpositions) {
                    remove.push(grid[a].clone());
                }
            } else if crosses == 1 {
                // bad - shouldnt happen
            } else if crosses == 2 {
                // simple start and finish
                grid[a].p1 = closestpoint;
                grid[a].p2 = farestpoint;
            } else {
                // multiple intersections
                let line = grid[a].clone();
                remove.push(line.clone());

                while matchs.len() > 1 {
                    let mut newline = linelatlng::default();

                    closestpoint = Self::findClosestPoint(&closestpoint, &matchs);
                    newline.p1 = closestpoint.clone();
                    remove_first(&mut matchs, |m| m.equals(&closestpoint));

                    closestpoint = Self::findClosestPoint(&closestpoint, &matchs);
                    newline.p2 = closestpoint.clone();
                    remove_first(&mut matchs, |m| m.equals(&closestpoint));

                    newline.basepnt = line.basepnt.clone();

                    grid.push(newline);
                }
            }
        }

        // cleanup and keep only lines that pass though our polygon
        for line in &remove {
            remove_first(&mut grid, |g| g.value_equals(line));
        }

        // debug
        for line in &grid {
            Self::addtomap_line(line);
        }

        if grid.is_empty() {
            return Ok(ans);
        }

        // pick start positon based on initial point rectangle
        let mut startposutm = Self::start_position(startpos, &area, utmzone, HomeLocation)?;

        // find the closes polygon point based from our startpos selection
        startposutm = Self::findClosestPoint(&startposutm, &utmpositions);

        // find closest line point to startpos
        let mut closest = Self::findClosestLine(&startposutm, &grid, 0.0 /*Lane separation does not apply to starting point*/, angle)?;

        // get the closes point from the line we picked
        let mut lastpnt = if closest.p1.get_distance(&startposutm) < closest.p2.get_distance(&startposutm) { closest.p1.clone() } else { closest.p2.clone() };

        // S =  start
        // E = end
        // ME = middle end
        // SM = start middle

        while !grid.is_empty() {
            // for each line, check which end of the line is the next closest
            if closest.p1.get_distance(&lastpnt) < closest.p2.get_distance(&lastpnt) {
                let newstart = tagged(Self::newpos_utm(&closest.p1, angle, -leadin1 as f64), "S");
                Self::addtomap(&newstart, "S");
                ans.push(newstart.to_lla()?);

                if leadin1 < 0.0 {
                    let p2 = tagged(utmpos::copy_of(&newstart), "SM");
                    Self::addtomap(&p2, "SM");
                    ans.push(p2.to_lla()?);
                } else {
                    closest.p1.tag = tag("SM");
                    Self::addtomap(&closest.p1, "SM");
                    ans.push(closest.p1.to_lla()?);
                }

                if spacing > 0.0 {
                    let mut d = spacing - (closest.basepnt.get_distance(&closest.p1) % spacing);
                    while d < closest.p1.get_distance(&closest.p2) {
                        let mut ax = closest.p1.x;
                        let mut ay = closest.p1.y;

                        Self::newpos(&mut ax, &mut ay, angle, d);
                        let utmpos1 = tagged(utmpos::new(ax, ay, utmzone), "M");
                        Self::addtomap(&utmpos1, "M");
                        ans.push(utmpos1.to_lla()?);
                        d += spacing;
                    }
                }

                let mut newend = Self::newpos_utm(&closest.p2, angle, overshoot1);

                if overshoot1 < 0.0 {
                    let p2 = tagged(utmpos::copy_of(&newend), "ME");
                    Self::addtomap(&p2, "ME");
                    ans.push(p2.to_lla()?);
                } else {
                    closest.p2.tag = tag("ME");
                    Self::addtomap(&closest.p2, "ME");
                    ans.push(closest.p2.to_lla()?);
                }

                newend.tag = tag("E");
                Self::addtomap(&newend, "E");
                ans.push(newend.to_lla()?);

                lastpnt = closest.p2.clone();

                remove_first(&mut grid, |g| g.value_equals(&closest));
                if grid.is_empty() {
                    break;
                }

                closest = if useextendedendpoint {
                    Self::findClosestLine(&newend, &grid, minLaneSeparationINMeters, angle)?
                } else {
                    Self::findClosestLine(&closest.p2, &grid, minLaneSeparationINMeters, angle)?
                };
            } else {
                let newstart = tagged(Self::newpos_utm(&closest.p2, angle, leadin2 as f64), "S");
                Self::addtomap(&newstart, "S");
                ans.push(newstart.to_lla()?);

                if leadin2 < 0.0 {
                    let p2 = tagged(utmpos::copy_of(&newstart), "SM");
                    Self::addtomap(&p2, "SM");
                    ans.push(p2.to_lla()?);
                } else {
                    closest.p2.tag = tag("SM");
                    Self::addtomap(&closest.p2, "SM");
                    ans.push(closest.p2.to_lla()?);
                }

                if spacing > 0.0 {
                    let mut d = closest.basepnt.get_distance(&closest.p2) % spacing;
                    while d < closest.p1.get_distance(&closest.p2) {
                        let mut ax = closest.p2.x;
                        let mut ay = closest.p2.y;

                        Self::newpos(&mut ax, &mut ay, angle, -d);
                        let utmpos2 = tagged(utmpos::new(ax, ay, utmzone), "M");
                        Self::addtomap(&utmpos2, "M");
                        ans.push(utmpos2.to_lla()?);
                        d += spacing;
                    }
                }

                let mut newend = Self::newpos_utm(&closest.p1, angle, -overshoot2);

                if overshoot2 < 0.0 {
                    let p2 = tagged(utmpos::copy_of(&newend), "ME");
                    Self::addtomap(&p2, "ME");
                    ans.push(p2.to_lla()?);
                } else {
                    closest.p1.tag = tag("ME");
                    Self::addtomap(&closest.p1, "ME");
                    ans.push(closest.p1.to_lla()?);
                }

                newend.tag = tag("E");
                Self::addtomap(&newend, "E");
                ans.push(newend.to_lla()?);

                lastpnt = closest.p1.clone();

                remove_first(&mut grid, |g| g.value_equals(&closest));
                if grid.is_empty() {
                    break;
                }

                closest = if useextendedendpoint {
                    Self::findClosestLine(&newend, &grid, minLaneSeparationINMeters, angle)?
                } else {
                    Self::findClosestLine(&closest.p1, &grid, minLaneSeparationINMeters, angle)?
                };
            }
        }

        // set the altitude on all points
        for plla in ans.iter_mut() {
            plla.Alt = altitude;
        }

        Ok(ans)
    }

    fn getPolyMinMax(utmpos: &[utmpos]) -> Rect {
        if utmpos.is_empty() {
            return Rect::default();
        }

        let mut minx = utmpos[0].x;
        let mut maxx = minx;
        let mut miny = utmpos[0].y;
        let mut maxy = miny;

        for pnt in utmpos {
            minx = crate::dotnet::math_min(minx, pnt.x);
            maxx = crate::dotnet::math_max(maxx, pnt.x);

            miny = crate::dotnet::math_min(miny, pnt.y);
            maxy = crate::dotnet::math_max(maxy, pnt.y);
        }

        Rect::new(minx, maxy, maxx - minx, miny - maxy)
    }

    /// polar to rectangular
    fn newpos(x: &mut f64, y: &mut f64, bearing: f64, distance: f64) {
        let mut degN = 90.0 - bearing;
        if degN < 0.0 {
            degN += 360.0;
        }
        *x += distance * (degN * deg2rad).cos();
        *y += distance * (degN * deg2rad).sin();
    }

    /// polar to rectangular
    fn newpos_utm(input: &utmpos, bearing: f64, distance: f64) -> utmpos {
        let mut degN = 90.0 - bearing;
        if degN < 0.0 {
            degN += 360.0;
        }
        let x = input.x + distance * (degN * deg2rad).cos();
        let y = input.y + distance * (degN * deg2rad).sin();

        utmpos::new(x, y, input.zone)
    }

    /// The two segments' intersection, or `utmpos.Zero` (parallel, or outside either segment).
    /// From http://stackoverflow.com/questions/1119451/how-to-tell-if-a-line-intersects-a-polygon-in-c
    pub fn FindLineIntersection(start1: &utmpos, end1: &utmpos, start2: &utmpos, end2: &utmpos) -> utmpos {
        let denom = ((end1.x - start1.x) * (end2.y - start2.y)) - ((end1.y - start1.y) * (end2.x - start2.x));
        //  AB & CD are parallel
        if denom == 0.0 {
            return utmpos::zero();
        }
        let numer = ((start1.y - start2.y) * (end2.x - start2.x)) - ((start1.x - start2.x) * (end2.y - start2.y));
        let r = numer / denom;
        let numer2 = ((start1.y - start2.y) * (end1.x - start1.x)) - ((start1.x - start2.x) * (end1.y - start1.y));
        let s = numer2 / denom;
        if (r < 0.0 || r > 1.0) || (s < 0.0 || s > 1.0) {
            return utmpos::zero();
        }
        // Find intersection point
        utmpos::new(start1.x + (r * (end1.x - start1.x)), start1.y + (r * (end1.y - start1.y)), start1.zone)
    }

    /// As `FindLineIntersection`, but the lines are extended: only parallel lines give Zero.
    pub fn FindLineIntersectionExtension(start1: &utmpos, end1: &utmpos, start2: &utmpos, end2: &utmpos) -> utmpos {
        let denom = ((end1.x - start1.x) * (end2.y - start2.y)) - ((end1.y - start1.y) * (end2.x - start2.x));
        //  AB & CD are parallel
        if denom == 0.0 {
            return utmpos::zero();
        }
        let numer = ((start1.y - start2.y) * (end2.x - start2.x)) - ((start1.x - start2.x) * (end2.y - start2.y));
        let r = numer / denom;
        // s (numer2 / denom) is worked out in C# but only feeds an empty if
        // Find intersection point
        utmpos::new(start1.x + (r * (end1.x - start1.x)), start1.y + (r * (end1.y - start1.y)), start1.zone)
    }

    fn findClosestPoint(start: &utmpos, list: &[utmpos]) -> utmpos {
        let mut answer = utmpos::zero();
        let mut currentbest = f64::MAX;

        for pnt in list {
            let dist1 = start.get_distance(pnt);

            if dist1 < currentbest {
                answer = pnt.clone();
                currentbest = dist1;
            }
        }

        answer
    }

    /// Add an angle while normalizing output in the range 0...360
    fn AddAngle(mut angle: f64, degrees: f64) -> f64 {
        angle += degrees;

        angle %= 360.0;

        while angle < 0.0 {
            angle += 360.0;
        }
        angle
    }

    fn findClosestLine(start: &utmpos, list: &[linelatlng], minDistance: f64, angle: f64) -> Result<linelatlng, Exception> {
        if minDistance == 0.0 {
            let mut answer = list.first().cloned().ok_or_else(list_index)?;
            let mut shortest = f64::MAX;

            for line in list {
                let ans1 = start.get_distance(&line.p1);
                let ans2 = start.get_distance(&line.p2);
                let shorterpnt = if ans1 < ans2 { &line.p1 } else { &line.p2 };

                if shortest > start.get_distance(shorterpnt) {
                    answer = line.clone();
                    shortest = start.get_distance(shorterpnt);
                }
            }

            return Ok(answer);
        }

        // By now, just add 5.000 km to our lines so they are long enough to allow intersection
        let METERS_TO_EXTEND = 5000.0;

        let perperndicularOrientation = Self::AddAngle(angle, 90.0);

        // Calculation of a perpendicular line to the grid lines containing the "start" point
        let start_perpendicular_line = Self::newpos_utm(start, perperndicularOrientation, -METERS_TO_EXTEND);
        let stop_perpendicular_line = Self::newpos_utm(start, perperndicularOrientation, METERS_TO_EXTEND);

        // Store one intersection point per grid line: Dictionary<utmpos, linelatlng>, keyed
        // by GetHashCode (x, y, zone) and Equals (x, y), in insertion order
        let mut intersectedPoints: Vec<(utmpos, linelatlng)> = Vec::new();
        // lets order distances from every intersected point per line with the "start" point
        let mut ordered_min_to_max: Vec<(f64, utmpos)> = Vec::new();

        for line in list {
            // Calculate intersection point
            let p = Self::FindLineIntersectionExtension(&line.p1, &line.p2, &start_perpendicular_line, &stop_perpendicular_line);

            // Store it
            match intersectedPoints.iter_mut().find(|(k, _)| k.get_hash_code() == p.get_hash_code() && k.equals(&p)) {
                Some(e) => e.1 = line.clone(),
                None => intersectedPoints.push((p.clone(), line.clone())),
            }

            // Calculate distances between interesected point and "start" (i.e. line and start)
            let distance_p = start.get_distance(&p);

            // Dictionary<double, ...>.ContainsKey: double.Equals (NaN equals NaN)
            if !ordered_min_to_max.iter().any(|(k, _)| *k == distance_p || (k.is_nan() && distance_p.is_nan())) {
                ordered_min_to_max.push((distance_p, p));
            }
        }

        // Acquire keys and sort them (double.CompareTo: NaN first).
        let mut ordered_keys: Vec<f64> = ordered_min_to_max.iter().map(|(k, _)| *k).collect();
        ordered_keys.sort_by(|a, b| match (a.is_nan(), b.is_nan()) {
            (true, true) => std::cmp::Ordering::Equal,
            (true, false) => std::cmp::Ordering::Less,
            (false, true) => std::cmp::Ordering::Greater,
            _ => a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal),
        });

        // Lets select a line that is the closest to "start" point but "mindistance" away at least.
        // If we have only one line, return that line whatever the minDistance says
        let mut key = f64::MAX;
        let mut i = 0;
        while key == f64::MAX && i < ordered_keys.len() {
            if ordered_keys[i] >= minDistance {
                key = ordered_keys[i];
            }
            i += 1;
        }

        // If no line is selected (because all of them are closer than minDistance, then get the farest one
        if key == f64::MAX {
            key = *ordered_keys.last().ok_or_else(list_index)?;
        }

        let filteredlist: Vec<linelatlng> = intersectedPoints.iter().filter(|(k, _)| k.get_distance(start) >= key).map(|(_, v)| v.clone()).collect();

        Self::findClosestLine(start, &filteredlist, 0.0, angle)
    }

    fn PointInPolygon(p: &utmpos, poly: &[utmpos]) -> bool {
        let mut inside = false;

        if poly.len() < 3 {
            return inside;
        }
        let mut oldPoint = utmpos::copy_of(&poly[poly.len() - 1]);

        for pi in poly {
            let newPoint = utmpos::copy_of(pi);

            let (p1, p2) = if newPoint.y > oldPoint.y { (&oldPoint, &newPoint) } else { (&newPoint, &oldPoint) };

            if (newPoint.y < p.y) == (p.y <= oldPoint.y) && (p.x - p1.x) * (p2.y - p1.y) < (p2.x - p1.x) * (p.y - p1.y) {
                inside = !inside;
            }
            oldPoint = newPoint;
        }
        inside
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Vec<PointLatLngAlt> {
        // about 200 m x 200 m near Canberra
        vec![
            PointLatLngAlt::new(-35.3600, 149.1600),
            PointLatLngAlt::new(-35.3600, 149.1622),
            PointLatLngAlt::new(-35.3618, 149.1622),
            PointLatLngAlt::new(-35.3618, 149.1600),
        ]
    }

    #[test]
    fn grid_lanes_alternate_and_carry_tags() {
        let home = PointLatLngAlt::new(-35.3618, 149.1600);
        let wps = Grid::CreateGrid(&square(), 50.0, 40.0, 0.0, 0.0, 0.0, 0.0, StartPosition::Home, false, 0.0, 0.0, 0.0, &home, true).unwrap();
        // each lane: S, SM, ME, E
        assert!(!wps.is_empty() && wps.len() % 4 == 0, "{}", wps.len());
        let tags: Vec<&str> = wps.iter().take(4).map(|p| p.Tag.as_str()).collect();
        assert_eq!(tags, ["S", "SM", "ME", "E"]);
        assert!(wps.iter().all(|p| p.Alt == 50.0));
        // ~200 m wide at 40 m spacing: about 5 lanes (UTM convergence tilts the square)
        assert!((4..=6).contains(&(wps.len() / 4)), "{}", wps.len() / 4);
        assert!(Grid::CreateGrid(&[], 50.0, 40.0, 0.0, 0.0, 0.0, 0.0, StartPosition::Home, false, 0.0, 0.0, 0.0, &home, true).unwrap().is_empty());
    }

    #[test]
    fn rotary_shrinks_lap_by_lap() {
        let home = PointLatLngAlt::new(-35.3618, 149.1600);
        let wps = Grid::CreateRotary(&square(), 30.0, 30.0, 0.0, 0.0, 0.0, 0.0, StartPosition::Home, false, 0.0, 0.0, &home, 0, false, 3).unwrap();
        assert!(!wps.is_empty());
        assert!(wps.iter().all(|p| p.Tag == "S" && p.Alt == 30.0));
    }

    #[test]
    fn line_intersection() {
        let z = 55;
        let p = Grid::FindLineIntersection(&utmpos::new(0.0, 0.0, z), &utmpos::new(10.0, 10.0, z), &utmpos::new(0.0, 10.0, z), &utmpos::new(10.0, 0.0, z));
        assert_eq!((p.x, p.y), (5.0, 5.0));
        assert!(Grid::FindLineIntersection(&utmpos::new(0.0, 0.0, z), &utmpos::new(1.0, 1.0, z), &utmpos::new(0.0, 10.0, z), &utmpos::new(10.0, 0.0, z)).is_zero());
        let e = Grid::FindLineIntersectionExtension(&utmpos::new(0.0, 0.0, z), &utmpos::new(1.0, 1.0, z), &utmpos::new(0.0, 10.0, z), &utmpos::new(10.0, 0.0, z));
        assert_eq!((e.x, e.y), (5.0, 5.0));
    }
}
