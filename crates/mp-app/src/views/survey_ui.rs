//! Survey (Grid) plugin window, a port of `Grid/GridUI.cs` + `GridUI.resx`.
//!
//! Layout (form client 1008x645): map 704x565 at the top left, TRK_zoom beside it, the
//! "Stats" group docked along the bottom and a 253px tab control docked right with the
//! Simple / Grid Options / Camera Config pages (the last two only with "Advanced Options").

use egui::{Color32, Pos2, Rect, Shape, Stroke, StrokeKind, Vec2};
use mp_core::mavcmd;
use mp_core::mission::Locationwp;

use super::tuning::group_box;
use crate::app::AppCtx;
use crate::{fonts, map, theme};

const CAMERAS_XML: &str = include_str!("../../assets/camerasBuiltin.xml");
const R: f64 = 6_378_137.0;

// MAV_CMD ids used by the grid output
const CONDITION_YAW: u16 = 115;
const DO_CHANGE_SPEED: u16 = 178;
const DO_SET_SERVO: u16 = 183;
const DO_REPEAT_SERVO: u16 = 184;
const DO_DIGICAM_CONTROL: u16 = 203;
const DO_SET_CAM_TRIGG_DIST: u16 = 206;

const START_POSITIONS: [&str; 6] = ["Home", "BottomLeft", "TopLeft", "BottomRight", "TopRight", "Point"];

#[derive(Clone)]
struct Camera {
    name: String,
    flen: f64,
    imgw: f64,
    imgh: f64,
    senw: f64,
    senh: f64,
}

fn cameras() -> Vec<Camera> {
    let Ok(doc) = roxmltree::Document::parse(CAMERAS_XML.trim_start_matches('\u{feff}')) else { return Vec::new() };
    doc.descendants()
        .filter(|n| n.has_tag_name("Camera"))
        .map(|c| {
            let t = |tag: &str| c.children().find(|x| x.has_tag_name(tag)).and_then(|x| x.text()).unwrap_or("").trim().to_string();
            let f = |tag: &str| t(tag).parse::<f64>().unwrap_or(0.0);
            Camera { name: t("name"), flen: f("flen"), imgw: f("imgw"), imgh: f("imgh"), senw: f("senw"), senh: f("senh") }
        })
        .collect()
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Trigger {
    Dist,
    Digicam,
    RepeatServo,
    SetServo,
}

/// A grid point with its GridUI tag: S(tart)/E(nd) of a lane or M (internal trigger point).
#[derive(Clone, Copy)]
struct GridPt {
    lat: f64,
    lng: f64,
    tag: char,
}

pub enum GridResult {
    Open,
    Cancel,
    Accept(Vec<Locationwp>),
}

pub struct GridUi {
    polygon: Vec<(f64, f64)>,
    home: (f64, f64),
    map: map::MapState,
    fitted: bool,
    tab: usize,
    cams: Vec<Camera>,
    camera: Option<usize>,
    // Simple
    alt: f64,
    angle: f64,
    camdir: bool,
    speed: f64,
    usespeed: bool,
    toandland: bool,
    toandland_rtl: bool,
    split: u32,
    // Display
    show_boundary: bool,
    show_markers: bool,
    show_grid: bool,
    show_internals: bool,
    show_footprints: bool,
    advanced: bool,
    // Grid Options
    distance: f64,
    overshoot: f64,
    overshoot2: f64,
    leadin: f64,
    leadin2: f64,
    startfrom: usize,
    overlap: f64,
    sidelap: f64,
    crossgrid: bool,
    corridor: bool,
    corridor_width: f64,
    spiral: bool,
    copter_delay: f64,
    headinghold: bool,
    heading: String,
    headinglock: bool,
    spline: bool,
    alt_lanes: f64,
    lane_dist: f64,
    optimise: bool,
    laps: f64,
    cw_laps: f64,
    match_perim: bool,
    spacing: f64,
    // Camera Config
    focal: f64,
    imgw: String,
    imgh: String,
    senw: String,
    senh: String,
    trigger: Trigger,
    stopstart: bool,
    rept_servo: f64,
    rept_pwm: f64,
    rept_time: f64,
    set_servo: f64,
    servo_low: f64,
    servo_high: f64,
    last_key: Option<[u64; 8]>,
    error: Option<String>,
}

impl GridUi {
    pub fn new(polygon: Vec<(f64, f64)>, home: (f64, f64), provider: map::Provider) -> Self {
        let cams = cameras();
        let map = map::MapState { provider, ..Default::default() };
        Self {
            polygon,
            home,
            map,
            fitted: false,
            tab: 0,
            cams,
            camera: None,
            alt: 100.0,
            angle: 0.0,
            camdir: true,
            speed: 5.0,
            usespeed: false,
            toandland: true,
            toandland_rtl: true,
            split: 1,
            show_boundary: true,
            show_markers: true,
            show_grid: true,
            show_internals: false,
            show_footprints: false,
            advanced: false,
            distance: 50.0,
            overshoot: 0.0,
            overshoot2: 0.0,
            leadin: 0.0,
            leadin2: 0.0,
            startfrom: 0,
            overlap: 50.0,
            sidelap: 60.0,
            crossgrid: false,
            corridor: false,
            corridor_width: 100.0,
            spiral: false,
            copter_delay: 0.0,
            headinghold: false,
            heading: "0".into(),
            headinglock: false,
            spline: false,
            alt_lanes: 0.0,
            lane_dist: 0.0,
            optimise: false,
            laps: 200.0,
            cw_laps: 0.0,
            match_perim: false,
            spacing: 30.0,
            focal: 5.0,
            imgw: "4608".into(),
            imgh: "3456".into(),
            senw: "6.16".into(),
            senh: "4.62".into(),
            trigger: Trigger::Dist,
            stopstart: true,
            rept_servo: 5.0,
            rept_pwm: 1100.0,
            rept_time: 2.0,
            set_servo: 5.0,
            servo_low: 1100.0,
            servo_high: 1900.0,
            last_key: None,
            error: None,
        }
    }

    /// Select a tab page (0 Simple, 1 Grid Options, 2 Camera Config); >0 turns on Advanced.
    pub fn set_tab(&mut self, t: usize) {
        self.advanced = t > 0;
        self.show_footprints = t > 0;
        self.tab = t.min(2);
    }

    fn sensor(&self) -> Option<(f64, f64, f64, f64)> {
        let p = |s: &str| s.trim().parse::<f64>().ok().filter(|v| *v > 0.0);
        Some((p(&self.imgw)?, p(&self.imgh)?, p(&self.senw)?, p(&self.senh)?))
    }

    /// getFOV: ground footprint (width, height) in metres at the survey altitude.
    fn fov(&self) -> Option<(f64, f64)> {
        let (_, _, sw, sh) = self.sensor()?;
        if self.focal <= 0.0 {
            return None;
        }
        let flscale = 1000.0 * self.alt / self.focal;
        Some((sw * flscale / 1000.0, sh * flscale / 1000.0))
    }

    fn cmpixel(&self) -> Option<f64> {
        let (_, ih, _, _) = self.sensor()?;
        let (_, vh) = self.fov()?;
        Some(vh / ih * 100.0)
    }

    /// calcfov: lane spacing and trigger distance follow overlap/sidelap whenever an input
    /// they depend on changes (they stay editable between changes, as in the original).
    fn recalc(&mut self) {
        let key = [self.alt, self.overlap, self.sidelap, self.focal, if self.camdir { 1.0 } else { 0.0 }, self.senw.parse().unwrap_or(0.0), self.senh.parse().unwrap_or(0.0), self.imgh.parse().unwrap_or(0.0)].map(f64::to_bits);
        if self.last_key == Some(key) {
            return;
        }
        self.last_key = Some(key);
        if let Some((vw, vh)) = self.fov() {
            if self.camdir {
                self.spacing = (1.0 - self.overlap / 100.0) * vh;
                self.distance = (1.0 - self.sidelap / 100.0) * vw;
            } else {
                self.spacing = (1.0 - self.overlap / 100.0) * vw;
                self.distance = (1.0 - self.sidelap / 100.0) * vh;
            }
        }
    }

    fn origin(&self) -> (f64, f64) {
        let n = self.polygon.len().max(1) as f64;
        (self.polygon.iter().map(|p| p.0).sum::<f64>() / n, self.polygon.iter().map(|p| p.1).sum::<f64>() / n)
    }

    fn lanes(&self, angle: f64, from: (f64, f64)) -> Vec<((f64, f64), (f64, f64))> {
        let raw = mp_core::survey::grid(&self.polygon, self.distance.max(0.2), angle, 0.0);
        let pairs: Vec<_> = raw.chunks(2).filter(|c| c.len() == 2).map(|c| (c[0], c[1])).collect();
        if pairs.is_empty() {
            return pairs;
        }
        // StartFrom: choose the ordering whose first point is nearest the requested corner
        let rev: Vec<_> = pairs.iter().rev().copied().collect();
        let cands = [pairs.clone(), pairs.iter().map(|(a, b)| (*b, *a)).collect(), rev.clone(), rev.iter().map(|(a, b)| (*b, *a)).collect::<Vec<_>>()];
        let o = self.origin();
        let best = cands
            .into_iter()
            .min_by(|a, b| {
                let da = dist_m(o, a[0].0, from);
                let db = dist_m(o, b[0].0, from);
                da.partial_cmp(&db).unwrap()
            })
            .unwrap();
        // overshoot / lead-in, alternating ends per lane direction
        best.into_iter()
            .enumerate()
            .map(|(i, (s, e))| {
                let (os, li) = if i % 2 == 0 { (self.overshoot, self.leadin) } else { (self.overshoot2, self.leadin2) };
                (extend(o, e, s, li), extend(o, s, e, os))
            })
            .collect()
    }

    fn start_target(&self) -> (f64, f64) {
        let (mut minlat, mut maxlat, mut minlng, mut maxlng) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for p in &self.polygon {
            minlat = minlat.min(p.0);
            maxlat = maxlat.max(p.0);
            minlng = minlng.min(p.1);
            maxlng = maxlng.max(p.1);
        }
        match self.startfrom {
            1 => (minlat, minlng),
            2 => (maxlat, minlng),
            3 => (minlat, maxlng),
            4 => (maxlat, maxlng),
            _ if self.home != (0.0, 0.0) => self.home,
            _ => (minlat, minlng),
        }
    }

    /// Grid.CreateGrid output: S/E lane ends with M trigger points between them.
    fn grid(&self) -> Vec<GridPt> {
        if self.polygon.len() < 3 {
            return Vec::new();
        }
        let o = self.origin();
        let mut lanes = self.lanes(self.angle, self.start_target());
        if self.crossgrid {
            let last = lanes.last().map(|l| l.1).unwrap_or(self.start_target());
            lanes.extend(self.lanes(self.angle + 90.0, last));
        }
        let mut out = Vec::new();
        for (s, e) in lanes {
            out.push(GridPt { lat: s.0, lng: s.1, tag: 'S' });
            let len = dist_m(o, s, e);
            if self.spacing > 0.5 {
                let n = (len / self.spacing).floor() as usize;
                for k in 1..n {
                    let f = k as f64 * self.spacing / len;
                    out.push(GridPt { lat: s.0 + (e.0 - s.0) * f, lng: s.1 + (e.1 - s.1) * f, tag: 'M' });
                }
            }
            out.push(GridPt { lat: e.0, lng: e.1, tag: 'E' });
        }
        out
    }

    fn route_m(&self, grid: &[GridPt]) -> f64 {
        let o = self.origin();
        let mut pts: Vec<(f64, f64)> = grid.iter().filter(|p| p.tag != 'M').map(|p| (p.lat, p.lng)).collect();
        if self.home != (0.0, 0.0) && !pts.is_empty() {
            pts.insert(0, self.home);
            pts.push(self.home);
        }
        pts.windows(2).map(|w| dist_m(o, w[0], w[1])).sum()
    }

    fn fit_map(&mut self, size: Vec2) {
        let (mut minlat, mut maxlat, mut minlng, mut maxlng) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for p in &self.polygon {
            minlat = minlat.min(p.0);
            maxlat = maxlat.max(p.0);
            minlng = minlng.min(p.1);
            maxlng = maxlng.max(p.1);
        }
        let (x0, y0) = map::latlng_to_world(maxlat, minlng, 0.0);
        let (x1, y1) = map::latlng_to_world(minlat, maxlng, 0.0);
        let (dx, dy) = ((x1 - x0).abs().max(1e-9), (y1 - y0).abs().max(1e-9));
        let z = ((size.x as f64 * 0.8 / dx).min(size.y as f64 * 0.8 / dy)).log2();
        self.map.zoom = z.floor().clamp(1.0, 20.0);
        self.map.center_on((minlat + maxlat) / 2.0, (minlng + maxlng) / 2.0);
    }

    fn output(&self, app: &AppCtx, grid: &[GridPt]) -> Result<Vec<Locationwp>, String> {
        if grid.is_empty() {
            return Err("Bad Grid".into());
        }
        if self.split > 1 && !self.toandland {
            return Err("You must use Land/RTL to split a mission".into());
        }
        let frame = app.settings.alt_frame;
        let cmd = |id: u16, p: [f32; 4]| Locationwp { id, p1: p[0], p2: p[1], p3: p[2], p4: p[3], frame, ..Default::default() };
        let mut out = Vec::new();
        let heading: f32 = self.heading.trim().parse().unwrap_or(0.0);
        let add_wp = |out: &mut Vec<Locationwp>, p: &GridPt| {
            if self.headinghold {
                out.push(cmd(CONDITION_YAW, [heading, 0.0, 0.0, 0.0]));
            }
            let id = if self.spline { mavcmd::SPLINE_WAYPOINT } else { mavcmd::WAYPOINT };
            out.push(Locationwp { id, p1: self.copter_delay as f32, lat: p.lat, lng: p.lng, alt: self.alt as f32, frame, ..Default::default() });
        };
        let per = (grid.len() as f64 / self.split.max(1) as f64).round().max(1.0) as usize;
        let mut start = 0;
        for split in 0..self.split.max(1) as usize {
            let mut end = if split + 1 == self.split.max(1) as usize { grid.len() } else { (per * (split + 1)).min(grid.len()) };
            while end > start && end < grid.len() && grid[end].tag != 'S' {
                end -= 1;
            }
            if self.toandland {
                out.push(Locationwp { id: mavcmd::TAKEOFF, p1: 20.0, alt: 30.0, frame, ..Default::default() });
            }
            if self.usespeed {
                out.push(cmd(DO_CHANGE_SPEED, [0.0, self.speed as f32, 0.0, 0.0]));
            }
            let mut started = false;
            for p in &grid[start..end] {
                match (p.tag, self.trigger) {
                    ('M', Trigger::Digicam) => {
                        add_wp(&mut out, p);
                        out.push(cmd(DO_DIGICAM_CONTROL, [1.0, 0.0, 0.0, 0.0]));
                    }
                    ('M', Trigger::RepeatServo) if !self.stopstart => {
                        add_wp(&mut out, p);
                        out.push(cmd(DO_REPEAT_SERVO, [self.rept_servo as f32, self.rept_pwm as f32, 1.0, self.rept_time as f32]));
                    }
                    ('M', _) => {}
                    (tag, trig) => {
                        add_wp(&mut out, p);
                        match trig {
                            Trigger::Dist if self.stopstart => out.push(cmd(DO_SET_CAM_TRIGG_DIST, [if tag == 'S' { self.spacing as f32 } else { 0.0 }, 0.0, 1.0, 0.0])),
                            Trigger::Dist if !started => out.push(cmd(DO_SET_CAM_TRIGG_DIST, [self.spacing as f32, 0.0, 1.0, 0.0])),
                            Trigger::SetServo => out.push(cmd(DO_SET_SERVO, [self.set_servo as f32, if tag == 'S' { self.servo_high } else { self.servo_low } as f32, 0.0, 0.0])),
                            Trigger::RepeatServo if self.stopstart && tag == 'S' => out.push(cmd(DO_REPEAT_SERVO, [self.rept_servo as f32, self.rept_pwm as f32, 999.0, self.rept_time as f32])),
                            _ => {}
                        }
                        started = true;
                    }
                }
            }
            if self.trigger == Trigger::Dist && !self.stopstart {
                out.push(cmd(DO_SET_CAM_TRIGG_DIST, [0.0, 0.0, 1.0, 0.0]));
            }
            if self.toandland {
                if self.toandland_rtl {
                    out.push(cmd(mavcmd::RETURN_TO_LAUNCH, [0.0; 4]));
                } else {
                    out.push(Locationwp { id: mavcmd::LAND, lat: self.home.0, lng: self.home.1, frame, ..Default::default() });
                }
            }
            start = end;
        }
        Ok(out)
    }

    pub fn show(&mut self, ctx: &egui::Context, app: &mut AppCtx) -> GridResult {
        self.recalc();
        let grid = self.grid();
        let mut result = GridResult::Open;
        let mut open = true;
        let frame = egui::Frame::window(&ctx.global_style()).fill(theme::BG).inner_margin(0.0);
        egui::Window::new("Survey (Grid)").open(&mut open).collapsible(false).resizable(false).frame(frame).fixed_size([1008.0, 645.0]).show(ctx, |ui| {
            let (full, _) = ui.allocate_exact_size(Vec2::new(1008.0, 645.0), egui::Sense::hover());
            let o = full.min;
            let p = ui.painter().clone();
            p.rect_filled(full, 0.0, theme::BG);
            self.map_area(ui, app, o, &grid);
            self.stats(&p, o, &grid);
            if let Some(r) = self.tabs(ui, app, o, &p, &grid) {
                result = r;
            }
        });
        if let Some(e) = self.error.take() {
            app.toasts.error(e);
        }
        if !open {
            result = GridResult::Cancel;
        }
        result
    }

    fn map_area(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, o: Pos2, grid: &[GridPt]) {
        let map_rect = Rect::from_min_size(o, Vec2::new(704.0, 565.0));
        if !self.fitted && self.polygon.len() >= 3 {
            self.fitted = true;
            self.fit_map(map_rect.size());
        }
        let assets = app.assets;
        let poly = self.polygon.clone();
        let (sb, sm, sg, si, sf) = (self.show_boundary, self.show_markers, self.show_grid, self.show_internals, self.show_footprints);
        let fov = self.fov();
        let camdir = self.camdir;
        let origin = self.origin();
        ui.scope_builder(egui::UiBuilder::new().max_rect(map_rect), |ui| {
            map::show(ui, &mut self.map, app.tiles, |_ui, view, _resp, painter| {
                if sf {
                    if let Some((vw, vh)) = fov {
                        let (along, across) = if camdir { (vh, vw) } else { (vw, vh) };
                        let mut a = 1usize;
                        let mut bearing = 0.0;
                        for (i, g) in grid.iter().enumerate() {
                            if g.tag == 'S' {
                                if let Some(e) = grid[i + 1..].iter().find(|x| x.tag == 'E') {
                                    let (dx, dy) = local(origin, (e.lat, e.lng), (g.lat, g.lng));
                                    bearing = dy.atan2(dx);
                                }
                            }
                            let c = local(origin, (g.lat, g.lng), origin);
                            let (s, co) = bearing.sin_cos();
                            let corners: Vec<Pos2> = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]
                                .iter()
                                .map(|(u, v)| {
                                    let (ax, ay) = (u * along / 2.0, v * across / 2.0);
                                    let (x, y) = (c.0 + ax * co - ay * s, c.1 + ax * s + ay * co);
                                    let ll = unlocal(origin, (x, y));
                                    view.to_screen(ll.0, ll.1)
                                })
                                .collect();
                            let col = Color32::from_rgb((250 - (a * 5) % 240) as u8, (250 - (a * 3) % 240) as u8, (250 - (a * 9) % 240) as u8);
                            painter.add(Shape::closed_line(corners, Stroke::new(1.0, col)));
                            a += 1;
                        }
                    }
                }
                if sg {
                    let route: Vec<Pos2> = grid.iter().filter(|g| g.tag != 'M').map(|g| view.to_screen(g.lat, g.lng)).collect();
                    painter.add(Shape::line(route, Stroke::new(4.0, Color32::YELLOW)));
                }
                if sb && poly.len() >= 2 {
                    let pts: Vec<Pos2> = poly.iter().map(|q| view.to_screen(q.0, q.1)).collect();
                    painter.add(Shape::closed_line(pts.clone(), Stroke::new(2.0, Color32::RED)));
                    for q in pts {
                        map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), q);
                    }
                }
                if si {
                    for g in grid.iter().filter(|g| g.tag == 'M') {
                        painter.circle_filled(view.to_screen(g.lat, g.lng), 3.0, Color32::from_rgb(0x00, 0xc0, 0x00));
                    }
                }
                if sm {
                    for g in grid.iter().filter(|g| g.tag != 'M') {
                        map::draw_pin(painter, assets.marker_green.id(), assets.marker_shadow.id(), view.to_screen(g.lat, g.lng));
                    }
                }
                false
            });
        });
        // TRK_zoom
        let trk = Rect::from_min_size(o + Vec2::new(710.0, 0.0), Vec2::new(45.0, 565.0));
        let mut z = self.map.zoom;
        if theme::vtrackbar(ui, trk, &mut z, 1.0, 20.0).changed() || z != self.map.zoom {
            self.map.zoom = z;
        }
    }

    fn stats(&self, p: &egui::Painter, o: Pos2, grid: &[GridPt]) {
        let g = Rect::from_min_size(o + Vec2::new(0.0, 565.0), Vec2::new(755.0, 80.0));
        group_box(p, g, "Stats");
        let f = fonts::ui_id(fonts::UI_SIZE);
        let t = |x: f32, y: f32, s: &str| {
            p.text(g.min + Vec2::new(x, y), egui::Align2::LEFT_TOP, s, f.clone(), Color32::WHITE);
        };
        let area = mp_core::survey::area(&self.polygon);
        let route = self.route_m(grid);
        let pictures = grid.len();
        let strips = grid.iter().filter(|x| x.tag == 'S').count();
        let fov = self.fov();
        let speed = self.speed.max(0.1);
        let turnrad = speed * speed / (9.808 * 45f64.to_radians().tan());
        let cm = self.cmpixel();
        for (x, y, l, v) in [
            (13.0, 20.0, "Area:", format!("{area:.0} m^2")),
            (13.0, 33.0, "Distance:", format!("{:.2} km", route / 1000.0)),
            (13.0, 46.0, "Dist between images:", format!("{:.1} m", self.spacing)),
            (13.0, 59.0, "Ground Resolution:", cm.map(|c| format!("{c:.2} cm")).unwrap_or_default()),
            (197.0, 20.0, "Pictures:", pictures.to_string()),
            (197.0, 33.0, "No of Strips:", strips.to_string()),
            (197.0, 46.0, "Footprint:", fov.map(|(w, h)| format!("{w:.1} x {h:.1} m")).unwrap_or_default()),
            (197.0, 59.0, "Dist between lines:", format!("{:.2} m", self.distance)),
            (382.0, 20.0, "Flight Time (est):", seconds_to_nice(route / (speed * 0.8))),
            (382.0, 33.0, "Photo every (est):", seconds_to_nice(self.spacing / speed)),
            (382.0, 46.0, "Turn Dia (at 45d):", format!("{:.0} m", turnrad * 2.0)),
            (382.0, 59.0, "Ground Elevation:", "0-0 m".to_string()),
            (559.0, 20.0, "Min Shutter Speed:", cm.map(|c| format!("1/{:.0}", (speed / (c * 0.01 / 2.0)).floor())).unwrap_or_default()),
        ] {
            t(x, y, l);
            let vx = match x as i32 {
                13 => 120.0,
                197 => 299.0,
                382 => 480.0,
                _ => 657.0,
            };
            t(vx, y, &v);
        }
    }

    fn tabs(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, o: Pos2, p: &egui::Painter, grid: &[GridPt]) -> Option<GridResult> {
        let tc = Rect::from_min_size(o + Vec2::new(755.0, 0.0), Vec2::new(253.0, 645.0));
        let labels: &[&str] = if self.advanced { &["Simple", "Grid Options", "Camera Config"] } else { &["Simple"] };
        if self.tab >= labels.len() {
            self.tab = 0;
        }
        ui.scope_builder(egui::UiBuilder::new().max_rect(Rect::from_min_size(tc.min, Vec2::new(253.0, 23.0))), |ui| {
            theme::tab_strip(ui, "grid_tabs", labels, &mut self.tab);
        });
        let page = Rect::from_min_size(tc.min + Vec2::new(4.0, 22.0), Vec2::new(245.0, 619.0));
        p.rect_stroke(page.expand(1.0), 0.0, Stroke::new(1.0, Color32::from_rgb(0xd9, 0xd9, 0xd9)), StrokeKind::Inside);
        let po = page.min;
        let f = fonts::ui_id(fonts::UI_SIZE);
        let txt = |x: f32, y: f32, s: &str| {
            p.text(po + Vec2::new(x, y), egui::Align2::LEFT_TOP, s, f.clone(), Color32::WHITE);
        };
        let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(po + Vec2::new(x, y), Vec2::new(w, h));
        let num = |ui: &mut egui::Ui, x: f32, y: f32, v: &mut f64, lo: f64, hi: f64, dec: usize| {
            ui.put(at(x, y, 51.0, 20.0), egui::DragValue::new(v).range(lo..=hi).max_decimals(dec).speed(if dec == 0 { 1.0 } else { 0.1 }));
        };
        let chk = |ui: &mut egui::Ui, x: f32, y: f32, w: f32, v: &mut bool, s: &str| {
            ui.put(at(x, y, w + 20.0, 17.0), egui::Checkbox::new(v, s));
        };
        let radio = |ui: &mut egui::Ui, x: f32, y: f32, w: f32, v: &mut Trigger, t: Trigger, s: &str| {
            if ui.put(at(x, y, w + 20.0, 17.0), egui::RadioButton::new(*v == t, s)).clicked() {
                *v = t;
            }
        };
        let gb = |x: f32, y: f32, w: f32, h: f32, s: &str| group_box(p, at(x, y, w, h), s);
        let mut result = None;
        match labels[self.tab] {
            "Simple" => {
                gb(6.0, 6.0, 236.0, 248.0, "Simple Options");
                let g = Vec2::new(6.0, 6.0);
                txt(g.x + 6.0, g.y + 16.0, "Camera");
                ui.put(at(g.x + 105.0, g.y + 13.0, 117.0, 21.0), |ui: &mut egui::Ui| {
                    let sel = self.camera.and_then(|i| self.cams.get(i)).map(|c| c.name.clone()).unwrap_or_default();
                    let mut pick = None;
                    let r = egui::ComboBox::from_id_salt("cmb_camera").width(117.0).wrap_mode(egui::TextWrapMode::Truncate).selected_text(sel).show_ui(ui, |ui| {
                        for (i, c) in self.cams.iter().enumerate() {
                            if ui.selectable_label(self.camera == Some(i), &c.name).clicked() {
                                pick = Some(i);
                            }
                        }
                    });
                    if let Some(i) = pick {
                        let c = self.cams[i].clone();
                        self.camera = Some(i);
                        self.focal = c.flen;
                        self.imgw = c.imgw.to_string();
                        self.imgh = c.imgh.to_string();
                        self.senw = c.senw.to_string();
                        self.senh = c.senh.to_string();
                    }
                    r.response
                });
                txt(g.x + 6.0, g.y + 42.0, "Altitude");
                num(ui, g.x + 142.0, g.y + 40.0, &mut self.alt, 1.0, 9999.0, 0);
                txt(g.x + 6.0, g.y + 68.0, "Angle [deg]");
                num(ui, g.x + 142.0, g.y + 66.0, &mut self.angle, -360.0, 360.0, 0);
                chk(ui, g.x + 43.0, g.y + 92.0, 150.0, &mut self.camdir, "Camera top facing forward");
                txt(g.x + 6.0, g.y + 117.0, "Flying Speed (est)");
                num(ui, g.x + 142.0, g.y + 115.0, &mut self.speed, 0.1, 360.0, 1);
                chk(ui, g.x + 43.0, g.y + 141.0, 148.0, &mut self.usespeed, "Use speed for this mission");
                chk(ui, g.x + 11.0, g.y + 171.0, 161.0, &mut self.toandland, "Add Takeoff and Land WP's");
                chk(ui, g.x + 29.0, g.y + 194.0, 69.0, &mut self.toandland_rtl, "Use RTL");
                txt(g.x + 7.0, g.y + 214.0, "Split into x segments");
                let mut s = self.split as f64;
                num(ui, g.x + 143.0, g.y + 212.0, &mut s, 1.0, 100.0, 0);
                self.split = s as u32;

                gb(6.0, 260.0, 236.0, 160.0, "Display");
                let g = Vec2::new(6.0, 260.0);
                chk(ui, g.x + 8.0, g.y + 20.0, 71.0, &mut self.show_boundary, "Boundary");
                chk(ui, g.x + 8.0, g.y + 43.0, 64.0, &mut self.show_markers, "Markers");
                chk(ui, g.x + 8.0, g.y + 66.0, 45.0, &mut self.show_grid, "Grid");
                chk(ui, g.x + 8.0, g.y + 89.0, 66.0, &mut self.show_internals, "Internals");
                chk(ui, g.x + 8.0, g.y + 112.0, 72.0, &mut self.show_footprints, "Footprints");
                chk(ui, g.x + 8.0, g.y + 135.0, 114.0, &mut self.advanced, "Advanced Options");
                txt(3.0, 423.0, "Control-S to save to file\nControl-O to load form file");
                let r = ui.scope_builder(egui::UiBuilder::new().max_rect(at(84.0, 587.0, 75.0, 23.0)), |ui| theme::button_sized(ui, "Accept", Vec2::new(75.0, 23.0))).inner;
                if r.clicked() {
                    match self.output(app, grid) {
                        Ok(v) => result = Some(GridResult::Accept(v)),
                        Err(e) => self.error = Some(e),
                    }
                }
            }
            "Grid Options" => {
                gb(6.0, 6.0, 233.0, 222.0, "Grid Options");
                let g = Vec2::new(6.0, 6.0);
                txt(g.x + 6.0, g.y + 16.0, "Distance between lines [m]");
                num(ui, g.x + 169.0, g.y + 14.0, &mut self.distance, 0.1, 9999.0, 2);
                txt(g.x + 7.0, g.y + 42.0, "OverShoot [m]");
                num(ui, g.x + 111.0, g.y + 40.0, &mut self.overshoot, 0.0, 999.0, 0);
                num(ui, g.x + 168.0, g.y + 40.0, &mut self.overshoot2, 0.0, 999.0, 0);
                txt(g.x + 7.0, g.y + 68.0, "LeadIn [m]");
                num(ui, g.x + 111.0, g.y + 66.0, &mut self.leadin, 0.0, 999.0, 0);
                num(ui, g.x + 168.0, g.y + 66.0, &mut self.leadin2, 0.0, 999.0, 0);
                txt(g.x + 5.0, g.y + 95.0, "StartFrom");
                ui.put(at(g.x + 112.0, g.y + 92.0, 107.0, 21.0), |ui: &mut egui::Ui| {
                    egui::ComboBox::from_id_salt("cmb_startfrom")
                        .width(107.0)
                        .selected_text(START_POSITIONS[self.startfrom])
                        .show_ui(ui, |ui| {
                            for (i, n) in START_POSITIONS.iter().enumerate() {
                                ui.selectable_value(&mut self.startfrom, i, *n);
                            }
                        })
                        .response
                });
                txt(g.x + 5.0, g.y + 121.0, "Overlap [%]");
                num(ui, g.x + 167.0, g.y + 119.0, &mut self.overlap, 0.0, 99.0, 0);
                txt(g.x + 5.0, g.y + 147.0, "Sidelap [%]");
                num(ui, g.x + 167.0, g.y + 145.0, &mut self.sidelap, 0.0, 99.0, 0);
                chk(ui, g.x + 8.0, g.y + 171.0, 74.0, &mut self.crossgrid, "Cross Grid");
                chk(ui, g.x + 92.0, g.y + 171.0, 62.0, &mut self.corridor, "Corridor");
                chk(ui, g.x + 169.0, g.y + 171.0, 52.0, &mut self.spiral, "Spiral");
                txt(g.x + 5.0, g.y + 197.0, "Corridor Width [m]");
                num(ui, g.x + 167.0, g.y + 195.0, &mut self.corridor_width, 0.0, 9999.0, 0);

                gb(6.0, 234.0, 233.0, 123.0, "Copter Options");
                let g = Vec2::new(6.0, 234.0);
                txt(g.x + 6.0, g.y + 23.0, "Delay at WP (sec)");
                num(ui, g.x + 143.0, g.y + 21.0, &mut self.copter_delay, 0.0, 999.0, 0);
                chk(ui, g.x + 9.0, g.y + 50.0, 91.0, &mut self.headinghold, "Heading Hold");
                ui.put(at(g.x + 143.0, g.y + 48.0, 31.0, 20.0), egui::TextEdit::singleline(&mut self.heading));
                for (dx, s, d) in [(180.0, "+", 1), (204.0, "--", -1)] {
                    let r = ui.scope_builder(egui::UiBuilder::new().max_rect(at(g.x + dx, g.y + 46.0, 23.0, 23.0)), |ui| theme::button_sized(ui, s, Vec2::new(23.0, 23.0))).inner;
                    if r.clicked() {
                        let h: i32 = self.heading.trim().parse().unwrap_or(0);
                        self.heading = ((h + d).rem_euclid(360)).to_string();
                    }
                }
                chk(ui, g.x + 32.0, g.y + 73.0, 103.0, &mut self.headinglock, "Unlock from grid");
                chk(ui, g.x + 9.0, g.y + 96.0, 109.0, &mut self.spline, "Spline Exit/Entrys");

                gb(6.0, 363.0, 233.0, 88.0, "Plane Options");
                let g = Vec2::new(6.0, 363.0);
                txt(g.x + 7.0, g.y + 19.0, "Alternate Lanes");
                num(ui, g.x + 143.0, g.y + 17.0, &mut self.alt_lanes, 0.0, 99.0, 0);
                txt(g.x + 26.0, g.y + 40.0, "Min Lane separation");
                num(ui, g.x + 143.0, g.y + 38.0, &mut self.lane_dist, 0.0, 999.0, 0);
                chk(ui, g.x + 10.0, g.y + 63.0, 126.0, &mut self.optimise, "Optimise for Distance");

                gb(6.0, 457.0, 233.0, 122.0, "Spiral Options");
                let g = Vec2::new(6.0, 457.0);
                txt(g.x + 7.0, g.y + 25.0, "Number of Laps");
                num(ui, g.x + 143.0, g.y + 23.0, &mut self.laps, 1.0, 9999.0, 0);
                txt(g.x + 7.0, g.y + 47.0, "Number of Clockwise Laps");
                num(ui, g.x + 143.0, g.y + 69.0, &mut self.cw_laps, -1.0, 9999.0, 0);
                txt(g.x + 18.0, g.y + 71.0, "(-1 for Clockwise Spiral)");
                chk(ui, g.x + 10.0, g.y + 94.0, 156.0, &mut self.match_perim, "Match Perimeter to Polygon");

                txt(14.0, 595.0, "Take a picture every [m]");
                num(ui, 150.0, 593.0, &mut self.spacing, 0.0, 9999.0, 2);
            }
            _ => {
                gb(6.0, 6.0, 231.0, 298.0, "Camera Options");
                let g = Vec2::new(6.0, 6.0);
                txt(g.x + 7.0, g.y + 21.0, "Focal Length [mm]");
                num(ui, g.x + 143.0, g.y + 19.0, &mut self.focal, 0.1, 999.0, 2);
                for (y, l, v) in [(43.0, "Image Width [Pixels]", &mut self.imgw), (69.0, "Image Height [Pixels]", &mut self.imgh), (95.0, "Sensor Width [mm]", &mut self.senw), (121.0, "Sensor Height [mm]", &mut self.senh)] {
                    txt(g.x + 7.0, g.y + y, l);
                    ui.put(at(g.x + 143.0, g.y + y + 2.0, 51.0, 20.0), egui::TextEdit::singleline(v));
                }
                let r = ui.scope_builder(egui::UiBuilder::new().max_rect(at(g.x + 58.0, g.y + 149.0, 110.0, 23.0)), |ui| theme::button_sized(ui, "Load Sample Photo", Vec2::new(110.0, 23.0))).inner;
                if r.clicked() {
                    self.error = Some("Load Sample Photo: EXIF reading is not available in this port yet".into());
                }
                let r = ui.scope_builder(egui::UiBuilder::new().max_rect(at(g.x + 83.0, g.y + 178.0, 64.0, 23.0)), |ui| theme::button_sized(ui, "Save", Vec2::new(64.0, 23.0))).inner;
                if r.clicked() {
                    let name = format!("Custom {:.1}mm {}x{}", self.focal, self.imgw, self.imgh);
                    if let Some((iw, ih, sw, sh)) = self.sensor() {
                        self.cams.push(Camera { name, flen: self.focal, imgw: iw, imgh: ih, senw: sw, senh: sh });
                        self.camera = Some(self.cams.len() - 1);
                    }
                }
                txt(g.x + 55.0, g.y + 204.0, "Calculated Values");
                let fov = self.fov();
                let ro = |ui: &mut egui::Ui, y: f32, s: String| {
                    let mut s = s;
                    ui.put(at(g.x + 143.0, g.y + y, 51.0, 20.0), egui::TextEdit::singleline(&mut s).interactive(false));
                };
                txt(g.x + 7.0, g.y + 224.0, "cm/pixel");
                ro(ui, 221.0, self.cmpixel().map(|c| format!("{c:.2} cm")).unwrap_or_default());
                txt(g.x + 4.0, g.y + 250.0, "Field of View Horizontal [m]");
                ro(ui, 247.0, fov.map(|f| format!("{:.1}", f.0)).unwrap_or_default());
                txt(g.x + 5.0, g.y + 277.0, "Field of View Vertical [m]");
                ro(ui, 274.0, fov.map(|f| format!("{:.1}", f.1)).unwrap_or_default());

                gb(7.0, 310.0, 231.0, 195.0, "Trigger Method");
                let g = Vec2::new(7.0, 310.0);
                radio(ui, g.x + 9.0, g.y + 19.0, 99.0, &mut self.trigger, Trigger::Dist, "CAM_TRIGG_DIST");
                chk(ui, g.x + 134.0, g.y + 19.0, 74.0, &mut self.stopstart, "Breakup starts");
                radio(ui, g.x + 9.0, g.y + 42.0, 130.0, &mut self.trigger, Trigger::Digicam, "DO_DIGICAM_CONTROL");
                radio(ui, g.x + 9.0, g.y + 65.0, 113.0, &mut self.trigger, Trigger::RepeatServo, "DO_REPEAT_SERVO");
                txt(g.x + 6.0, g.y + 85.0, "Servo");
                txt(g.x + 63.0, g.y + 85.0, "PWM");
                txt(g.x + 120.0, g.y + 85.0, "Cycle Time [s]");
                num(ui, g.x + 9.0, g.y + 101.0, &mut self.rept_servo, 1.0, 16.0, 0);
                num(ui, g.x + 66.0, g.y + 101.0, &mut self.rept_pwm, 800.0, 2200.0, 0);
                num(ui, g.x + 123.0, g.y + 101.0, &mut self.rept_time, 0.0, 999.0, 1);
                radio(ui, g.x + 9.0, g.y + 127.0, 91.0, &mut self.trigger, Trigger::SetServo, "DO_SET_SERVO");
                txt(g.x + 6.0, g.y + 148.0, "Servo");
                txt(g.x + 64.0, g.y + 147.0, "PWM L");
                txt(g.x + 120.0, g.y + 147.0, "PWM H");
                num(ui, g.x + 9.0, g.y + 164.0, &mut self.set_servo, 1.0, 16.0, 0);
                num(ui, g.x + 66.0, g.y + 164.0, &mut self.servo_low, 800.0, 2200.0, 0);
                num(ui, g.x + 123.0, g.y + 164.0, &mut self.servo_high, 800.0, 2200.0, 0);
            }
        }
        result
    }
}

fn seconds_to_nice(seconds: f64) -> String {
    if !seconds.is_finite() || seconds < 0.0 {
        return "Infinity Seconds".into();
    }
    let secs = seconds % 60.0;
    let mins = (seconds / 60.0) as i64 % 60;
    let hours = (seconds / 3600.0) as i64;
    if hours > 0 {
        format!("{hours}:{mins:02}:{:02.0} Hours", secs.floor())
    } else if mins > 0 {
        format!("{mins}:{:02.0} Minutes", secs.floor())
    } else {
        format!("{secs:.2} Seconds")
    }
}

/// Local tangent-plane metres of `p` relative to `rel`, using `origin` for the scale.
fn local(origin: (f64, f64), p: (f64, f64), rel: (f64, f64)) -> (f64, f64) {
    let x = (p.1 - rel.1).to_radians() * R * origin.0.to_radians().cos();
    let y = (p.0 - rel.0).to_radians() * R;
    (x, y)
}

fn unlocal(origin: (f64, f64), xy: (f64, f64)) -> (f64, f64) {
    (origin.0 + (xy.1 / R).to_degrees(), origin.1 + (xy.0 / (R * origin.0.to_radians().cos())).to_degrees())
}

fn dist_m(origin: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (x, y) = local(origin, b, a);
    (x * x + y * y).sqrt()
}

/// `to` moved `m` metres further along the direction from `from`.
fn extend(origin: (f64, f64), from: (f64, f64), to: (f64, f64), m: f64) -> (f64, f64) {
    if m == 0.0 {
        return to;
    }
    let (dx, dy) = local(origin, to, from);
    let l = (dx * dx + dy * dy).sqrt().max(1e-9);
    let t = local(origin, to, origin);
    unlocal(origin, (t.0 + dx / l * m, t.1 + dy / l * m))
}
