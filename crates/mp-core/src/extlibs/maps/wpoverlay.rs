//! Port of `ExtLibs/Maps/WPOverlay.cs`
//!
//! GMap.NET.WindowsForms (`GMapOverlay`, `GMapRoute`, `GMapPolygon`, `GMapMarkerWP`,
//! `GMarkerGoogle`, `GMapMarkerRect`, `GMapMarkerRallyPt`) and the GDI+ pens are not ported:
//! the overlay `CreateOverlay` fills is kept here as plain data, in the order the C# adds it,
//! and the app's map paints it. `srtm.getAltitude` is passed in by the caller (`None` for an
//! invalid tile).

#![allow(non_snake_case)]
// DO_SET_ROI and the non-INT frames are marked [Obsolete] in Mavlink.cs; the C# still uses them
#![allow(deprecated)]

use crate::dotnet::drawing::Color;
use crate::dotnet::{current_culture, format_fixed, Exception};
use crate::extlibs::gmap_net_core::gmap_net::PointLatLng;
use crate::extlibs::mavlink::{MAV_CMD, MAV_FRAME, MAV_MISSION_TYPE};
use crate::extlibs::utilities::locationwp::Locationwp;
use crate::extlibs::utilities::math::MathHelper;
use crate::extlibs::utilities::pointlatlngalt::PointLatLngAlt;
use crate::extlibs::utilities::spline2::{spline_segment_end_type, Spline2};

/// `System.Drawing.Drawing2D.DashStyle` (the members this file uses)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DashStyle {
    Solid,
    Dash,
    Custom,
}

/// `System.Drawing.Pen`
#[derive(Debug, Clone, PartialEq)]
pub struct Pen {
    pub color: Color,
    pub width: f32,
    pub dash_style: DashStyle,
}

impl Pen {
    /// `new Pen(color, width)`
    pub fn new(color: Color, width: f32) -> Self {
        Pen { color, width, dash_style: DashStyle::Solid }
    }
}

/// The marker classes `addpolygonmarker` and the ROI branch create.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkerKind {
    /// `GMapMarkerWP`: the numbered mission marker
    Wp,
    /// `GMarkerGoogle(GMarkerGoogleType.red)`
    GoogleRed,
    /// `GMarkerGoogle(GMarkerGoogleType.blue_dot)`
    GoogleBlueDot,
    /// `GMapMarkerRallyPt`
    RallyPt,
}

/// `MarkerTooltipMode`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkerTooltipMode {
    OnMouseOver,
    Always,
}

/// One marker as the overlay holds it.
#[derive(Debug, Clone, PartialEq)]
pub enum Marker {
    /// A marker of `kind` at (lat, lng).
    Marker { kind: MarkerKind, lat: f64, lng: f64, tag: String, tooltip: Option<(MarkerTooltipMode, String)> },
    /// `GMapMarkerRect` around the marker before it: the waypoint radius circle and colour.
    /// `Tag` "Dont draw line" marks the ROI's border.
    Rect { lat: f64, lng: f64, tag: String, wprad: f64, color: Option<Color>, fill_color: Option<Color> },
}

/// `GMapRoute`: its `Points` are `PointLatLng` (a `PointLatLngAlt` added to it loses Alt and Tag).
#[derive(Debug, Clone, PartialEq)]
pub struct GMapRoute {
    pub name: String,
    pub points: Vec<PointLatLng>,
    pub stroke: Pen,
}

/// `GMapPolygon`: `fill` is the brush colour.
#[derive(Debug, Clone, PartialEq)]
pub struct GMapPolygon {
    pub name: String,
    pub points: Vec<PointLatLngAlt>,
    pub fill: Color,
    pub stroke: Pen,
}

/// `GMapOverlay`: markers, routes and polygons in the order they were added.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GMapOverlay {
    pub id: String,
    pub markers: Vec<Marker>,
    pub routes: Vec<GMapRoute>,
    pub polygons: Vec<GMapPolygon>,
}

impl GMapOverlay {
    /// `Clear()`
    pub fn clear(&mut self) {
        self.markers.clear();
        self.routes.clear();
        self.polygons.clear();
    }
}

fn null_reference() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

fn cmd(c: MAV_CMD) -> u16 {
    c.0 as u16
}

/// `Math.Sign(double)`: NaN throws.
fn math_sign(v: f64) -> Result<i32, Exception> {
    if v.is_nan() {
        return Err(Exception::Arithmetic("Function does not accept floating point Not-a-Number values.".into()));
    }
    Ok(if v > 0.0 {
        1
    } else if v < 0.0 {
        -1
    } else {
        0
    })
}

/// `public class WPOverlay`
#[derive(Debug, Clone)]
pub struct WPOverlay {
    /// `overlay = new GMapOverlay("WPOverlay")`
    pub overlay: GMapOverlay,
    /// list of points as per the mission (`null` entries for items without a point)
    pub pointlist: Vec<Option<PointLatLngAlt>>,
    /// list of point as per mission including jump repeats
    route: Vec<Option<PointLatLngAlt>>,
}

impl Default for WPOverlay {
    fn default() -> Self {
        WPOverlay {
            overlay: GMapOverlay { id: "WPOverlay".into(), ..Default::default() },
            pointlist: vec![],
            route: vec![],
        }
    }
}

impl WPOverlay {
    pub fn new() -> Self {
        Self::default()
    }

    /// `CreateOverlay(home, missionitems, wpradius, loiterradius, altunitmultiplier, fencehome)`.
    /// `home.Tag` is set to "H" as the C# does to the caller's object.
    #[allow(clippy::too_many_arguments)]
    pub fn CreateOverlay(
        &mut self,
        home: &mut PointLatLngAlt,
        missionitems: &[Locationwp],
        wpradius: f64,
        loiterradius: f64,
        altunitmultiplier: f64,
        fencehome: Option<&PointLatLngAlt>,
        srtm_get_altitude: &mut dyn FnMut(f64, f64) -> Option<f64>,
    ) -> Result<(), Exception> {
        self.overlay.clear();

        let mut fencepoly: Option<GMapPolygon> = None;

        let home_alt = home.Alt;
        let mut gethomealt = |altmode: u8, lat: f64, lng: f64| Self::GetHomeAlt(altmode, home_alt, lat, lng, &mut *srtm_get_altitude);

        if *home != PointLatLngAlt::zero() {
            home.Tag = "H".into();
            self.pointlist.push(Some(home.clone()));
            self.route.push(self.pointlist[self.pointlist.len() - 1].clone());
            self.addpolygonmarker("H", home.Lng, home.Lat, Some(home.Alt * altunitmultiplier), None, 0.0, MAV_MISSION_TYPE::MISSION, None);
        }

        for a in 0..missionitems.len() {
            let item = &missionitems[a];
            let default_next = Locationwp::default();
            let itemnext = if a + 1 < missionitems.len() { &missionitems[a + 1] } else { &default_next };

            let command = item.id;
            let tag = (a + 1).to_string();

            // invalid locationwp
            if command == 0 {
                self.pointlist.push(None);
                continue;
            }

            // navigatable points
            if command < cmd(MAV_CMD::LAST)
                && command != cmd(MAV_CMD::RETURN_TO_LAUNCH)
                && command != cmd(MAV_CMD::CONTINUE_AND_CHANGE_ALT)
                && command != cmd(MAV_CMD::DELAY)
                && command != cmd(MAV_CMD::GUIDED_ENABLE)
                || command == cmd(MAV_CMD::DO_SET_ROI)
                || command == cmd(MAV_CMD::DO_LAND_START)
                || command == cmd(MAV_CMD::DO_RETURN_PATH_START)
            {
                let is_land = command == cmd(MAV_CMD::LAND) || command == cmd(MAV_CMD::VTOL_LAND);
                // land can be 0,0 or a lat,lng
                if is_land && item.lat == 0.0 && item.lng == 0.0 {
                    continue;
                }

                if command == cmd(MAV_CMD::DO_LAND_START) && item.lat != 0.0 && item.lng != 0.0 {
                    let alt = item.alt as f64 + gethomealt(item.frame, item.lat, item.lng);
                    self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, alt, &tag)));
                    self.route.push(self.pointlist[self.pointlist.len() - 1].clone());

                    // draw everything before
                    if !self.route.is_empty() {
                        let mut r = std::mem::take(&mut self.route);
                        self.RegenerateWPRoute(&mut r, home, false)?;
                    }

                    self.route.push(self.pointlist[self.pointlist.len() - 1].clone());
                    self.addpolygonmarker(&tag, item.lng, item.lat, Some(item.alt as f64 * altunitmultiplier), None, wpradius, MAV_MISSION_TYPE::MISSION, None);
                } else if is_land && item.lat != 0.0 && item.lng != 0.0 {
                    let alt = item.alt as f64 + gethomealt(item.frame, item.lat, item.lng);
                    self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, alt, &tag)));
                    self.route.push(self.pointlist[self.pointlist.len() - 1].clone());
                    self.addpolygonmarker(&tag, item.lng, item.lat, Some(item.alt as f64 * altunitmultiplier), None, wpradius, MAV_MISSION_TYPE::MISSION, None);

                    let mut r = std::mem::take(&mut self.route);
                    self.RegenerateWPRoute(&mut r, home, false)?;
                } else if command == cmd(MAV_CMD::DO_SET_ROI) {
                    let alt = item.alt as f64 + gethomealt(item.frame, item.lat, item.lng);
                    self.pointlist.push(Some(PointLatLngAlt {
                        color: Color::RED,
                        ..PointLatLngAlt::new_tagged(item.lat, item.lng, alt, &format!("ROI{}", a + 1))
                    }));
                    // do set roi is not a nav command. so we dont route through it
                    // check for clear roi, and hide it
                    if item.lat != 0.0 && item.lng != 0.0 {
                        // order matters
                        self.overlay.markers.push(Marker::Marker {
                            kind: MarkerKind::GoogleRed,
                            lat: item.lat,
                            lng: item.lng,
                            tag: tag.clone(),
                            tooltip: Some((MarkerTooltipMode::Always, tag.clone())),
                        });
                        self.overlay.markers.push(Marker::Rect {
                            lat: item.lat,
                            lng: item.lng,
                            tag: "Dont draw line".into(),
                            wprad: 0.0,
                            color: None,
                            fill_color: None,
                        });
                    }
                } else if command == cmd(MAV_CMD::LOITER_TIME)
                    || command == cmd(MAV_CMD::LOITER_TURNS)
                    || command == cmd(MAV_CMD::LOITER_TO_ALT)
                    || command == cmd(MAV_CMD::LOITER_UNLIM)
                {
                    if item.lat == 0.0 && item.lng == 0.0 {
                        // loiter at current location.
                        self.pointlist.push(None);
                    } else {
                        let alt = item.alt as f64 + gethomealt(item.frame, item.lat, item.lng);
                        self.pointlist.push(Some(PointLatLngAlt {
                            color: Color::LIGHT_BLUE,
                            ..PointLatLngAlt::new_tagged(item.lat, item.lng, alt, &tag)
                        }));

                        // Calculate the loiter radius and direction for this command, if one is specified
                        let mut this_loiterradius = loiterradius;
                        if command == cmd(MAV_CMD::LOITER_TURNS) || command == cmd(MAV_CMD::LOITER_UNLIM) {
                            this_loiterradius = if item.p3 != 0.0 { item.p3 as f64 } else { this_loiterradius };
                        } else if command == cmd(MAV_CMD::LOITER_TO_ALT) {
                            this_loiterradius = if item.p2 != 0.0 { item.p2 as f64 } else { this_loiterradius };
                        }
                        let loiterdirection = math_sign(this_loiterradius)?;
                        this_loiterradius = this_loiterradius.abs();

                        let last = self.pointlist[self.pointlist.len() - 1].clone();
                        // exit at tangent
                        if item.p4 == 1.0 {
                            let from = last.clone().ok_or_else(null_reference)?;
                            let to = if itemnext.lat != 0.0 && itemnext.lng != 0.0 {
                                PointLatLngAlt {
                                    Alt: itemnext.alt as f64 + gethomealt(item.frame, item.lat, item.lng),
                                    ..PointLatLngAlt::from_locationwp(itemnext)
                                }
                            } else {
                                from.clone()
                            };

                            let bearing = from.get_bearing(&to);
                            let dist = from.get_distance(&to);

                            if dist > this_loiterradius {
                                self.route.push(last);
                                let theta = (this_loiterradius / dist).acos() * MathHelper::RAD2DEG;
                                let offset = from.newpos(bearing - loiterdirection as f64 * theta, this_loiterradius);
                                self.route.push(Some(offset));
                            } else {
                                self.route.push(last);
                            }
                        } else {
                            self.route.push(last);
                        }

                        self.addpolygonmarker(
                            &tag,
                            item.lng,
                            item.lat,
                            Some(item.alt as f64 * altunitmultiplier),
                            Some(Color::LIGHT_BLUE),
                            this_loiterradius,
                            MAV_MISSION_TYPE::MISSION,
                            None,
                        );
                    }
                } else if command == cmd(MAV_CMD::SPLINE_WAYPOINT) {
                    let alt = item.alt as f64 + gethomealt(item.frame, item.lat, item.lng);
                    self.pointlist.push(Some(PointLatLngAlt { Tag2: "spline".into(), ..PointLatLngAlt::new_tagged(item.lat, item.lng, alt, &tag) }));
                    self.route.push(self.pointlist[self.pointlist.len() - 1].clone());
                    self.addpolygonmarker(&tag, item.lng, item.lat, Some(item.alt as f64 * altunitmultiplier), Some(Color::GREEN), wpradius, MAV_MISSION_TYPE::MISSION, None);
                } else if command == cmd(MAV_CMD::WAYPOINT) && item.lat == 0.0 && item.lng == 0.0 {
                    if !self.pointlist.is_empty() {
                        self.route.push(self.pointlist[self.pointlist.len() - 1].clone());
                    }
                    self.pointlist.push(None);
                } else if item.lat != 0.0 && item.lng != 0.0 {
                    let alt = item.alt as f64 + gethomealt(item.frame, item.lat, item.lng);
                    self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, alt, &tag)));
                    self.route.push(self.pointlist[self.pointlist.len() - 1].clone());
                    self.addpolygonmarker(&tag, item.lng, item.lat, Some(item.alt as f64 * altunitmultiplier), None, wpradius, MAV_MISSION_TYPE::MISSION, None);
                } else {
                    self.pointlist.push(None);
                }
                // maxlat/maxlong/minlat/minlong are computed in the C# and never read
            } else if command == cmd(MAV_CMD::DO_JUMP) {
                // fix do jumps into the future
                self.pointlist.push(None);

                // Math.Max(float, 0) keeps a NaN, which the (int) cast turns into int.MinValue
                let wpno = crate::dotnet::cast_i32(if item.p1.is_nan() { f64::NAN } else { item.p1.max(0.0) as f64 });
                let repeat = crate::dotnet::cast_i32(item.p2 as f64);

                let mut list = vec![];

                // cycle through reps
                let mut repno = repeat;
                while repno > 0 {
                    // cycle through wps
                    let mut no = wpno;
                    while no <= a as i32 {
                        let p = self.pointlist.get(no as usize).ok_or_else(list_index)?;
                        if p.is_some() {
                            list.push(p.clone());
                        }
                        no += 1;
                    }
                    repno -= 1;
                }
                self.route.extend(list);
            } else if command == cmd(MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION) || command == cmd(MAV_CMD::FENCE_POLYGON_VERTEX_EXCLUSION) {
                let inclusion = command == cmd(MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION);
                let poly = fencepoly.get_or_insert_with(|| GMapPolygon {
                    name: a.to_string(),
                    points: vec![],
                    // GMapPolygon.DefaultFill / DefaultStroke (FromArgb(155, AliceBlue), FromArgb(155,
                    // MidnightBlue) width 5) until the polygon is complete
                    fill: Color::from_argb(0x9BF0_F8FFu32 as i32),
                    stroke: Pen::new(Color::from_argb(0x9B19_1970u32 as i32), 5.0),
                });
                self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, 0.0, &tag)));
                poly.points.push(PointLatLngAlt::new_tagged(item.lat, item.lng, 0.0, &tag));
                let complete = poly.points.len() as f32 == item.p1;
                self.addpolygonmarker(&tag, item.lng, item.lat, None, Some(if inclusion { Color::BLUE } else { Color::RED }), 0.0, MAV_MISSION_TYPE::FENCE, None);
                if complete {
                    let mut p = fencepoly.take().ok_or_else(null_reference)?;
                    if inclusion {
                        p.fill = Color::TRANSPARENT;
                        p.stroke = Pen::new(Color::PINK, 1.0);
                    } else {
                        p.fill = Color::from_argb4(30, 255, 0, 0)?;
                        p.stroke = Pen::new(Color::RED, 1.0);
                    }
                    self.overlay.polygons.push(p);
                }
            } else if command == cmd(MAV_CMD::FENCE_CIRCLE_EXCLUSION) {
                self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, 0.0, &tag)));
                self.addpolygonmarker(
                    &tag,
                    item.lng,
                    item.lat,
                    None,
                    Some(Color::RED),
                    item.p1 as f64,
                    MAV_MISSION_TYPE::FENCE,
                    Some(Color::from_argb4(30, 255, 0, 0)?),
                );
            } else if command == cmd(MAV_CMD::FENCE_CIRCLE_INCLUSION) {
                self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, 0.0, &tag)));
                self.addpolygonmarker(&tag, item.lng, item.lat, None, Some(Color::BLUE), item.p1 as f64, MAV_MISSION_TYPE::FENCE, None);
            } else if command == cmd(MAV_CMD::FENCE_HOME_CIRCLE_INCLUSION) {
                // centred on home rather than the item's lat/lng, so there is no marker to drag
                self.pointlist.push(None);
                let centre: &PointLatLngAlt = fencehome.unwrap_or(&*home);
                if (centre.Lat != 0.0 || centre.Lng != 0.0) && item.p1 > 0.0 {
                    // a closed route rather than a polygon: FlightPlanner treats every fence polygon as
                    // editable vertices and adds insert markers to its edges
                    let points = (0..73).map(|i| PointLatLng::from(&centre.newpos((i * 5) as f64, item.p1 as f64))).collect();
                    let mut stroke = Pen::new(Color::BLUE, 2.0);
                    stroke.dash_style = DashStyle::Dash;
                    self.overlay.routes.push(GMapRoute { name: tag.clone(), points, stroke });
                }
            } else if command == cmd(MAV_CMD::FENCE_RETURN_POINT) {
                self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, 0.0, &tag)));
                self.addpolygonmarker(&tag, item.lng, item.lat, None, Some(Color::ORANGE), 0.0, MAV_MISSION_TYPE::FENCE, None);
            } else if command == cmd(MAV_CMD::RALLY_POINT) {
                self.pointlist.push(Some(PointLatLngAlt::new_tagged(item.lat, item.lng, 0.0, &tag)));
                self.addpolygonmarker(&tag, item.lng, item.lat, None, Some(Color::ORANGE), 0.0, MAV_MISSION_TYPE::RALLY, None);
            } else {
                self.pointlist.push(None);
            }
        }

        let mut r = std::mem::take(&mut self.route);
        let res = self.RegenerateWPRoute(&mut r, home, true);
        // the C# passes its field, which keeps the home appended to it
        self.route = r;
        res
    }

    /// `GetHomeAlt(altmode, homealt, lat, lng)`
    fn GetHomeAlt(altmode: u8, homealt: f64, lat: f64, lng: f64, srtm_get_altitude: &mut dyn FnMut(f64, f64) -> Option<f64>) -> f64 {
        let m = altmode as u32;
        if m == MAV_FRAME::GLOBAL_INT.0 as u32 || m == MAV_FRAME::GLOBAL.0 as u32 {
            return 0.0; // for absolute we dont need to add homealt
        }

        if m == MAV_FRAME::GLOBAL_TERRAIN_ALT_INT.0 as u32 || m == MAV_FRAME::GLOBAL_TERRAIN_ALT.0 as u32 {
            return match srtm_get_altitude(lat, lng) {
                None => -999.0,
                Some(alt) => alt,
            };
        }

        homealt
    }

    /// `addpolygonmarker(tag, lng, lat, alt, color, wpradius, type, fillcolor)`: the marker
    /// for the mission type, then its `GMapMarkerRect`.
    #[allow(clippy::too_many_arguments)]
    fn addpolygonmarker(
        &mut self,
        tag: &str,
        lng: f64,
        lat: f64,
        alt: Option<f64>,
        color: Option<Color>,
        wpradius: f64,
        r#type: MAV_MISSION_TYPE,
        fillcolor: Option<Color>,
    ) {
        let tooltip = alt.map(|v| (MarkerTooltipMode::OnMouseOver, format!("Alt: {}", format_fixed(v, 1, 0, current_culture()))));
        let kind = if r#type.0 == MAV_MISSION_TYPE::MISSION.0 {
            Some((MarkerKind::Wp, tooltip))
        } else if r#type.0 == MAV_MISSION_TYPE::FENCE.0 {
            Some((MarkerKind::GoogleBlueDot, None))
        } else if r#type.0 == MAV_MISSION_TYPE::RALLY.0 {
            Some((MarkerKind::RallyPt, tooltip))
        } else {
            None
        };
        // another mission type leaves the marker null: overlay.Markers.Add(null) throws inside
        // the try and nothing is added
        let Some((kind, tooltip)) = kind else { return };

        self.overlay.markers.push(Marker::Marker { kind, lat, lng, tag: tag.to_string(), tooltip });
        self.overlay.markers.push(Marker::Rect { lat, lng, tag: tag.to_string(), wprad: wpradius, color, fill_color: fillcolor });
    }

    /// `RegenerateWPRoute(fullpointlist, HomeLocation, includehomeroute)`: spline runs are
    /// expanded with `Spline2`; the yellow route and, when asked, the dashed home leg are added.
    fn RegenerateWPRoute(&mut self, fullpointlist: &mut Vec<Option<PointLatLngAlt>>, home_location: &PointLatLngAlt, includehomeroute: bool) -> Result<(), Exception> {
        if fullpointlist.is_empty() {
            return Ok(());
        }

        let mut route = GMapRoute { name: "wp route".into(), points: vec![], stroke: Pen::new(Color::YELLOW, 4.0) };
        let mut homeroute = GMapRoute { name: "home route".into(), points: vec![], stroke: Pen::new(Color::YELLOW, 2.0) };

        let mut lastpnt = fullpointlist[0].clone();
        let mut lastpnt2 = fullpointlist[0].clone();
        let mut splinepnts: Vec<Option<PointLatLngAlt>> = vec![];
        let mut wproute: Vec<PointLatLngAlt> = vec![];

        // add home - this causeszx the spline to always have a straight finish
        fullpointlist.push(fullpointlist[0].clone());

        for a in 0..fullpointlist.len() {
            let Some(pnt) = fullpointlist[a].clone() else { continue };

            if pnt.Tag2 == "spline" {
                if splinepnts.is_empty() {
                    splinepnts.push(lastpnt.clone());
                }

                splinepnts.push(Some(pnt));
            } else {
                if !splinepnts.is_empty() {
                    let mut list: Vec<PointLatLng> = vec![];

                    splinepnts.push(Some(pnt.clone()));

                    let mut sp = Spline2::new(home_location.clone());

                    let o = sp.pv_location_to_vector(lastpnt2.as_ref().ok_or_else(null_reference)?);
                    let d = sp.pv_location_to_vector(lastpnt.as_ref().ok_or_else(null_reference)?);
                    sp.set_wp_origin_and_destination(o, d);

                    sp._flags.reached_destination = true;

                    for no in 1..splinepnts.len() - 1 {
                        let mut segtype = spline_segment_end_type::SEGMENT_END_STRAIGHT;

                        if no < splinepnts.len() - 2 {
                            segtype = spline_segment_end_type::SEGMENT_END_SPLINE;
                        }

                        let dest = sp.pv_location_to_vector(splinepnts[no].as_ref().ok_or_else(null_reference)?);
                        let next = sp.pv_location_to_vector(splinepnts[no + 1].as_ref().ok_or_else(null_reference)?);
                        sp.set_spline_destination(dest, false, segtype, next)?;

                        while !sp._flags.reached_destination {
                            let t = 1.0f32;
                            sp.advance_spline_target_along_track(t)?;
                            list.push(PointLatLng::from(&sp.pv_vector_to_location(&sp.target_pos)));
                        }

                        list.push(PointLatLng::from(splinepnts[no].as_ref().ok_or_else(null_reference)?));
                    }

                    // List<PointLatLng> into List<PointLatLngAlt>: Alt 0, no Tag
                    wproute.extend(list.into_iter().map(PointLatLngAlt::from_point));

                    splinepnts.clear();
                }

                wproute.push(pnt.clone());

                lastpnt2 = lastpnt;
                lastpnt = Some(pnt);
            }
        }

        let count = wproute.len();
        let mut homepoint = PointLatLngAlt::default();
        let mut firstpoint = PointLatLngAlt::default();
        let mut lastpoint = PointLatLngAlt::default();

        if count > 2 {
            // homeroute = last, home, first
            for (i, x) in wproute.into_iter().enumerate() {
                let counter = i + 1;
                if counter == 1 && includehomeroute {
                    homepoint = x;
                    continue;
                }

                if counter == 2 {
                    firstpoint = x.clone();
                }

                if counter == count - 1 {
                    lastpoint = x.clone();
                }

                if counter == count {
                    if includehomeroute {
                        homeroute.points.push(PointLatLng::from(&lastpoint));
                        homeroute.points.push(PointLatLng::from(&homepoint));
                        homeroute.points.push(PointLatLng::from(&firstpoint));
                    }
                    continue;
                }

                route.points.push(PointLatLng::from(&x));
            }

            // if we have a large distance between home and the first/last point, it hangs on the draw of a the dashed line.
            if homepoint.get_distance(&lastpoint) < 5000.0 && homepoint.get_distance(&firstpoint) < 5000.0 {
                homeroute.stroke.dash_style = DashStyle::Dash;
            }

            if includehomeroute {
                self.overlay.routes.push(homeroute);
            }

            route.stroke.dash_style = DashStyle::Custom;
            self.overlay.routes.push(route);
        }

        Ok(())
    }
}

fn list_index() -> Exception {
    Exception::ArgumentOutOfRange(
        "Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wp(id: MAV_CMD, lat: f64, lng: f64) -> Locationwp {
        Locationwp { id: id.0 as u16, lat, lng, alt: 100.0, frame: 3, ..Default::default() }
    }

    #[test]
    fn route_and_dashed_home_leg() {
        let mut home = PointLatLngAlt::new_alt(-35.0, 149.0, 584.0);
        let items = [wp(MAV_CMD::WAYPOINT, -35.001, 149.0), wp(MAV_CMD::WAYPOINT, -35.002, 149.001), wp(MAV_CMD::WAYPOINT, -35.0, 149.002)];
        let mut o = WPOverlay::new();
        o.CreateOverlay(&mut home, &items, 2.0, 45.0, 1.0, None, &mut |_, _| None).unwrap();
        assert_eq!(home.Tag, "H");
        assert_eq!(o.overlay.routes.len(), 2);
        let homeroute = &o.overlay.routes[0];
        assert_eq!(homeroute.name, "home route");
        assert_eq!(homeroute.stroke.dash_style, DashStyle::Dash);
        let ll = |r: &GMapRoute| r.points.iter().map(|p| (p.lat(), p.lng())).collect::<Vec<_>>();
        assert_eq!(ll(homeroute), [(-35.0, 149.002), (-35.0, 149.0), (-35.001, 149.0)]);
        let route = &o.overlay.routes[1];
        assert_eq!(ll(route), [(-35.001, 149.0), (-35.002, 149.001), (-35.0, 149.002)]);
        assert_eq!(route.stroke.width, 4.0);
        // home marker, then a marker and rect per waypoint
        assert_eq!(o.overlay.markers.len(), 8);
        assert!(matches!(&o.overlay.markers[2], Marker::Marker { kind: MarkerKind::Wp, tooltip: Some((_, t)), .. } if t == "Alt: 100"));
    }

    #[test]
    fn spline_legs_are_expanded() {
        let mut home = PointLatLngAlt::new_alt(-35.0, 149.0, 0.0);
        let items = [
            wp(MAV_CMD::WAYPOINT, -35.001, 149.0),
            wp(MAV_CMD::SPLINE_WAYPOINT, -35.002, 149.001),
            wp(MAV_CMD::SPLINE_WAYPOINT, -35.001, 149.002),
            wp(MAV_CMD::WAYPOINT, -35.0, 149.003),
        ];
        let mut o = WPOverlay::new();
        o.CreateOverlay(&mut home, &items, 2.0, 45.0, 1.0, None, &mut |_, _| None).unwrap();
        let route = o.overlay.routes.iter().find(|r| r.name == "wp route").unwrap();
        assert!(route.points.len() > 6, "{}", route.points.len());
        let last = route.points.last().unwrap();
        assert_eq!((last.lat(), last.lng()), (-35.0, 149.003));
    }

    #[test]
    fn do_jump_repeats_and_fence_polygon() {
        let mut home = PointLatLngAlt::new_alt(-35.0, 149.0, 0.0);
        let mut jump = wp(MAV_CMD::DO_JUMP, 0.0, 0.0);
        jump.p1 = 1.0;
        jump.p2 = 2.0;
        let items = [wp(MAV_CMD::WAYPOINT, -35.001, 149.0), wp(MAV_CMD::WAYPOINT, -35.002, 149.0), jump];
        let mut o = WPOverlay::new();
        o.CreateOverlay(&mut home, &items, 2.0, 45.0, 1.0, None, &mut |_, _| None).unwrap();
        let route = o.overlay.routes.iter().find(|r| r.name == "wp route").unwrap();
        // H 1 2 then twice (1 2), less home and the appended home
        assert_eq!(route.points.iter().map(|p| p.lat()).collect::<Vec<_>>(), [-35.001, -35.002, -35.001, -35.002, -35.001, -35.002]);

        let v = |lat: f64, lng: f64| Locationwp { id: MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION.0 as u16, p1: 3.0, lat, lng, ..Default::default() };
        let fence = [v(1.0, 1.0), v(1.0, 2.0), v(2.0, 2.0)];
        let mut o = WPOverlay::new();
        o.CreateOverlay(&mut PointLatLngAlt::zero(), &fence, 0.0, 0.0, 1.0, None, &mut |_, _| None).unwrap();
        assert_eq!(o.overlay.polygons.len(), 1);
        assert_eq!(o.overlay.polygons[0].stroke.color, Color::PINK);
        assert!(o.overlay.routes.is_empty());
    }
}
