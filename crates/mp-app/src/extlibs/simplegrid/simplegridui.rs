//! Port of `ExtLibs/SimpleGrid/SimpleGridUI.cs`
//!
//! (`MissionPlanner.SimpleGrid.GridUI`)
//!
//! The form is modal (`ShowDialog`); it is drawn as a fixed-size window over the planner and
//! `show` reports whether it is still open. GMap.NET's `GMapOverlay "polygons"` is kept as
//! plain data (`Layer`) and painted over the map in the order GMap renders it: polygons, then
//! markers, then the tooltips of the markers under the mouse.

#![allow(non_snake_case)]

use egui::{Color32, Pos2, Rect, Shape, Stroke, Vec2};
use mp_core::dotnet::{cast_i32, current_culture, decimal_format_fixed_optional, decimal_parse, decimal_to_double, decimal_to_string, format_fixed, format_fixed_optional, Decimal, Exception};
use mp_core::extlibs::utilities::grid::{Grid, StartPosition};
use mp_core::extlibs::utilities::PointLatLngAlt;

use super::simplegridplugin::PluginHost;
use super::simplegridui_designer::{self as designer, Designer, DesignerEvent, CLIENT_SIZE, MAP_MAX_ZOOM, MAP_MIN_ZOOM, MAP_ZOOM, STATS_HEIGHT, TAB_WIDTH};
use crate::app::AppCtx;
use crate::map;

/// `MAVLink.MAV_CMD.WAYPOINT`
const MAV_CMD_WAYPOINT: u16 = 16;
/// `MAVLink.MAV_CMD.DO_CHANGE_SPEED`
const MAV_CMD_DO_CHANGE_SPEED: u16 = 178;

/// `GMarkerGoogleType`
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum GMarkerGoogleType {
    green,
    red,
}

/// `GMarkerGoogle` in the layer.
#[derive(Clone, Debug)]
struct GMarkerGoogle {
    lat: f64,
    lng: f64,
    kind: GMarkerGoogleType,
    /// `ToolTipText` with `ToolTipMode = MarkerTooltipMode.OnMouseOver`
    tooltip: Option<String>,
    /// `IsMouseOver`
    mouse_over: bool,
}

/// `GMapPolygon` in the layer: `Fill = Brushes.Transparent`, so only the stroke is drawn.
#[derive(Clone, Debug)]
struct GMapPolygon {
    points: Vec<(f64, f64)>,
    stroke: Color32,
    width: f32,
}

/// `layerpolygons = new GMapOverlay("polygons")`
#[derive(Clone, Debug, Default)]
struct Layer {
    polygons: Vec<GMapPolygon>,
    markers: Vec<GMarkerGoogle>,
}

/// What the dialog did this frame.
pub enum GridResult {
    Open,
    /// closed with the close box
    Cancel,
    /// `BUT_Accept_Click`: the `Host.AddWPtoList(cmd, p1, p2, p3, p4, x, y, z)` calls, in order;
    /// the caller makes them, runs writeKML, then `GridUI::savesettings` before the form closes
    Accept(Vec<(u16, [f64; 7])>),
}

/// `public partial class GridUI : Form`
pub struct GridUI {
    d: Designer,
    map: map::MapState,
    layerpolygons: Layer,
    grid: Option<Vec<PointLatLngAlt>>,
    list: Vec<PointLatLngAlt>,
    MouseDownStart: (f64, f64),
    CurrentGMapMarkerStartPos: Option<PointLatLngAlt>,
    /// `CurrentGMapMarker != null`
    CurrentGMapMarker: bool,
    CurrentGMapMarkerIndex: i32,
    isMouseDown: bool,
    isMouseDraging: bool,
    /// the last pointer position seen over the map (MouseMove fires on a change)
    last_pointer: Option<Pos2>,
    /// `CustomMessageBox.Show(text, caption)` calls waiting to be shown
    messages: Vec<(String, &'static str)>,
    /// `ClientSize`: the form is sizable; the docked controls follow it
    client: Vec2,
}

impl GridUI {
    /// `public GridUI(GridPlugin plugin)` followed by the form's `Load` (`GridUI_Load`).
    pub fn new(host: &mut PluginHost) -> Result<Self, Exception> {
        let d = designer::InitializeComponent();
        // map.MapProvider = plugin.Host.FDMapType; GMap's default Position is (0,0)
        let map = map::MapState { provider: host.FDMapType, lat: 0.0, lng: 0.0, zoom: MAP_ZOOM, ..Default::default() };
        let mut ui = GridUI {
            d,
            map,
            layerpolygons: Layer::default(),
            grid: None,
            list: Vec::new(),
            MouseDownStart: (0.0, 0.0),
            CurrentGMapMarkerStartPos: None,
            CurrentGMapMarker: false,
            CurrentGMapMarkerIndex: 0,
            isMouseDown: false,
            isMouseDraging: false,
            last_pointer: None,
            messages: Vec::new(),
            client: CLIENT_SIZE,
        };

        // CMB_startfrom.DataSource = Enum.GetNames(typeof(Utilities.Grid.StartPosition)):
        // binding selects the first item, which raises SelectedIndexChanged (list still empty)
        ui.d.CMB_startfrom.items = StartPosition::get_names().into_iter().map(String::from).collect();
        ui.d.CMB_startfrom.selected_index = 0;
        ui.domainUpDown1_ValueChanged(host)?;
        // CMB_startfrom.SelectedIndex = 0: unchanged

        // set and angle that is good
        ui.list = host.FPDrawnPolygon.iter().map(|&(lat, lng)| PointLatLngAlt::new(lat, lng)).collect();
        let area = ui.calcpolygonarea(&mut *host.FPDrawnPolygon)?;
        if area > 10000.0 {
            // (int) Math.Sqrt(area) / 10: integer division
            let v = Decimal::from(cast_i32(area.sqrt()) / 10);
            if ui.d.NUM_Distance.set_value(v)? {
                ui.domainUpDown1_ValueChanged(host)?;
            }
            if ui.d.NUM_spacing.set_value(v)? {
                ui.domainUpDown1_ValueChanged(host)?;
            }
        }

        let angle = mp_core::dotnet::decimal_from_double((ui.getAngleOfLongestSide(&ui.list)? + 360.0) % 360.0)?;
        if ui.d.NUM_angle.set_value(angle)? {
            ui.domainUpDown1_ValueChanged(host)?;
        }

        // GridUI_Load
        ui.loadsettings(host)?;
        Ok(ui)
    }

    /// `loadsettings()`: only when "simplegrid_camera" exists (nothing in this form writes it).
    fn loadsettings(&mut self, host: &mut PluginHost) -> Result<(), Exception> {
        if host.config.contains_key("simplegrid_camera") {
            for (key, which) in [("simplegrid_alt", 0), ("simplegrid_dist", 1), ("simplegrid_overshoot1", 2), ("simplegrid_overshoot2", 3)] {
                self.loadsetting(host, key, which)?;
            }
        }
        Ok(())
    }

    /// `loadsetting(key, NumericUpDown)`: `Value = decimal.Parse(config[key].ToString())`.
    fn loadsetting(&mut self, host: &mut PluginHost, key: &str, which: usize) -> Result<(), Exception> {
        let Some(text) = host.config.get(key).cloned() else { return Ok(()) };
        let v = decimal_parse(&text)?;
        let item = match which {
            0 => &mut self.d.NUM_altitude,
            1 => &mut self.d.NUM_Distance,
            2 => &mut self.d.NUM_overshoot,
            _ => &mut self.d.NUM_overshoot2,
        };
        if item.set_value(v)? {
            self.domainUpDown1_ValueChanged(host)?;
        }
        Ok(())
    }

    /// `savesettings()`: run by the caller after Accept's rows are added (C# order:
    /// AddWPtoList..., writeKML, savesettings, Close)
    pub fn savesettings(&self, host: &mut PluginHost) {
        host.config.insert("simplegrid_alt".into(), decimal_to_string(self.d.NUM_altitude.value));
        host.config.insert("simplegrid_angle".into(), decimal_to_string(self.d.NUM_angle.value));
        host.config.insert("simplegrid_dist".into(), decimal_to_string(self.d.NUM_Distance.value));
        host.config.insert("simplegrid_overshoot1".into(), decimal_to_string(self.d.NUM_overshoot.value));
        host.config.insert("simplegrid_overshoot2".into(), decimal_to_string(self.d.NUM_overshoot2.value));
    }

    /// `map_OnMarkerLeave`
    fn map_OnMarkerLeave(&mut self) {
        if !self.isMouseDown {
            // when you click the context menu this triggers and causes problems
            self.CurrentGMapMarker = false;
        }
    }

    /// `map_OnMarkerEnter`
    fn map_OnMarkerEnter(&mut self, lat: f64, lng: f64) {
        if !self.isMouseDown {
            self.CurrentGMapMarker = true;
            // a PointLatLng position converted to PointLatLngAlt
            self.CurrentGMapMarkerStartPos = Some(PointLatLngAlt::new(lat, lng));
        }
    }

    /// `map_MouseUp`
    fn map_MouseUp(&mut self, right: bool, left: bool) {
        if right {
            // ignore right clicks
            return;
        }
        if self.isMouseDown {
            // mouse down on some other object and dragged to here.
            if left {
                self.isMouseDown = false;
            }
        }
        self.isMouseDraging = false;
        self.CurrentGMapMarker = false;
        self.CurrentGMapMarkerIndex = 0;
        self.CurrentGMapMarkerStartPos = None;
    }

    /// `map_MouseDown`
    fn map_MouseDown(&mut self, point: (f64, f64), left: bool, alt: bool) {
        self.MouseDownStart = point;
        if left && !alt {
            self.isMouseDown = true;
            self.isMouseDraging = false;
            if let Some(start) = &self.CurrentGMapMarkerStartPos {
                let s = start.to_string();
                self.CurrentGMapMarkerIndex = self.list.iter().position(|c| c.to_string() == s).map(|i| i as i32).unwrap_or(-1);
            }
        }
    }

    /// `map_MouseMove`: returns the map pan (lat, lng) the left-drag asks for.
    fn map_MouseMove(&mut self, host: &mut PluginHost, point: (f64, f64), left_down: bool) -> Result<Option<(f64, f64)>, Exception> {
        if self.MouseDownStart == point {
            return Ok(None);
        }
        // dragging
        if left_down && self.isMouseDown {
            self.isMouseDraging = true;
            if self.CurrentGMapMarker {
                if self.CurrentGMapMarkerIndex == -1 {
                    self.isMouseDraging = false;
                    return Ok(None);
                }
                if let Some(slot) = self.list.get_mut(self.CurrentGMapMarkerIndex as usize) {
                    *slot = PointLatLngAlt::new(point.0, point.1);
                } else {
                    return Err(Exception::ArgumentOutOfRange("Index was out of range. Must be non-negative and less than the size of the collection.
Parameter name: index".into()));
                }
                self.domainUpDown1_ValueChanged(host)?;
            } else {
                // left click pan
                let latdif = self.MouseDownStart.0 - point.0;
                let lngdif = self.MouseDownStart.1 - point.1;
                return Ok(Some((latdif, lngdif)));
            }
        }
        Ok(None)
    }

    /// `AddDrawPolygon()`: the boundary in red (width 2) with a red marker on each corner.
    fn AddDrawPolygon(&mut self) {
        let list2: Vec<(f64, f64)> = self.list.iter().map(|x| (x.Lat, x.Lng)).collect();
        self.layerpolygons.polygons.push(GMapPolygon { points: list2, stroke: Color32::RED, width: 2.0 });
        for item in &self.list {
            self.layerpolygons.markers.push(GMarkerGoogle { lat: item.Lat, lng: item.Lng, kind: GMarkerGoogleType::red, tooltip: None, mouse_over: false });
        }
    }

    /// `getAngleOfLongestSide(list)`
    fn getAngleOfLongestSide(&self, list: &[PointLatLngAlt]) -> Result<f64, Exception> {
        let mut angle = 0.0;
        let mut maxdist = 0.0;
        let mut last = list.last().ok_or_else(|| Exception::ArgumentOutOfRange("Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into()))?;
        for item in list {
            if item.get_distance(last) > maxdist {
                angle = item.get_bearing(last);
                maxdist = item.get_distance(last);
            }
            last = item;
        }
        Ok((angle + 360.0) % 360.0)
    }

    /// `domainUpDown1_ValueChanged`: rebuild the grid and the layer, and update the stats.
    fn domainUpDown1_ValueChanged(&mut self, host: &mut PluginHost) -> Result<(), Exception> {
        let text = self.d.CMB_startfrom.text();
        let startpos = StartPosition::parse(&text).ok_or_else(|| {
            if text.trim().is_empty() {
                Exception::Argument("Must specify valid information for parsing in the string.".into())
            } else {
                Exception::Argument(format!("Requested value '{text}' was not found."))
            }
        })?;
        let grid = Grid::CreateGrid(
            &self.list,
            decimal_to_double(self.d.NUM_altitude.value),
            decimal_to_double(self.d.NUM_Distance.value),
            decimal_to_double(self.d.NUM_spacing.value),
            decimal_to_double(self.d.NUM_angle.value),
            decimal_to_double(self.d.NUM_overshoot.value),
            decimal_to_double(self.d.NUM_overshoot2.value),
            startpos,
            false,
            0.0,
            0.0,
            0.0,
            &host.PlannedHomeLocation,
            true,
        )?;

        let list2: Vec<(f64, f64)> = grid.iter().map(|x| (x.Lat, x.Lng)).collect();

        self.layerpolygons.polygons.clear();
        self.layerpolygons.markers.clear();

        if self.d.chk_boundary {
            self.AddDrawPolygon();
        }

        if grid.is_empty() {
            self.grid = Some(grid);
            self.ZoomAndCenterMarkers();
            return Ok(());
        }

        let mut strips = 0;
        let mut a = 1;
        for item in &grid {
            if item.Tag == "M" {
                if self.d.CHK_internals {
                    self.layerpolygons.markers.push(GMarkerGoogle { lat: item.Lat, lng: item.Lng, kind: GMarkerGoogleType::green, tooltip: Some(a.to_string()), mouse_over: false });
                    a += 1;
                }
            } else if item.Tag == "S" || item.Tag == "E" {
                strips += 1;
                if self.d.chk_markers {
                    self.layerpolygons.markers.push(GMarkerGoogle { lat: item.Lat, lng: item.Lng, kind: GMarkerGoogleType::green, tooltip: Some(a.to_string()), mouse_over: false });
                }
                a += 1;
            }
        }

        // add wp polygon: "Grid", yellow, width 4
        let wppoly = GMapPolygon { points: list2, stroke: Color32::YELLOW, width: 4.0 };
        // wppoly.Distance: GMapPolygon is a MapRoute
        let distance = mp_core::extlibs::gmap_net_core::gmap_net::MapRoute::with_points(
            wppoly.points.iter().map(|&(lat, lng)| mp_core::extlibs::gmap_net_core::gmap_net::PointLatLng::new(lat, lng)),
            "Grid",
        )
        .Distance();
        if self.d.chk_grid {
            self.layerpolygons.polygons.push(wppoly);
        }

        mp_core::dotnet::io::console_write_line(&format!("Poly Dist {}", mp_core::dotnet::double_to_string(distance)));

        self.d.lbl_area = format!("{} m^2", format_fixed(self.calcpolygonarea(&mut *host.FPDrawnPolygon)?, 0, 0, current_culture()));
        self.d.lbl_distance = format!("{} km", format_fixed_optional(distance, 1, 0, 2, current_culture()));
        self.d.lbl_strips = (strips / 2).to_string();
        self.d.lbl_distbetweenlines = format!("{} m", decimal_format_fixed_optional(self.d.NUM_Distance.value, 1, 0, 2, current_culture()));

        self.grid = Some(grid);
        self.ZoomAndCenterMarkers();
        Ok(())
    }

    /// `map.ZoomAndCenterMarkers("polygons")`: `GetRectOfAllMarkers` then `SetZoomToFitRect`
    /// (`GetMaxZoomToFitRect` for the 755x457 map); nothing happens without markers.
    fn ZoomAndCenterMarkers(&mut self) {
        use mp_core::extlibs::gmap_net_core::gmap_net::{PointLatLng, PureProjection, RectLatLng};
        use mp_core::extlibs::gmap_net_core::gmap_net_projections::MercatorProjection;
        // GetRectOfAllMarkers
        let markers = &self.layerpolygons.markers;
        if markers.is_empty() {
            return;
        }
        let (mut left, mut top, mut right, mut bottom) = (f64::MAX, f64::MIN, f64::MIN, f64::MAX);
        for m in markers {
            if m.lng < left {
                left = m.lng;
            }
            if m.lat > top {
                top = m.lat;
            }
            if m.lng > right {
                right = m.lng;
            }
            if m.lat < bottom {
                bottom = m.lat;
            }
        }
        let rect = RectLatLng::from_ltrb(left, top, right, bottom);
        // Core.GetMaxZoomToFitRect with the provider's (Mercator) projection
        let projection = MercatorProjection::Instance();
        // the map control's Width/Height (docked Fill)
        let (w, h) = ((self.client.x - TAB_WIDTH).max(0.0) as i64, (self.client.y - STATS_HEIGHT).max(0.0) as i64);
        let mut zoom = MAP_MIN_ZOOM;
        if rect.height_lat() == 0.0 || rect.width_lng() == 0.0 {
            zoom = MAP_MAX_ZOOM / 2;
        } else {
            for i in MAP_MIN_ZOOM..=MAP_MAX_ZOOM {
                let p1 = projection.FromLatLngToPixel_p(rect.location_top_left(), i);
                let p2 = projection.FromLatLngToPixel_p(rect.location_right_bottom(), i);
                if p2.x() - p1.x() <= w + 10 && p2.y() - p1.y() <= h + 10 {
                    zoom = i;
                } else {
                    break;
                }
            }
        }
        // SetZoomToFitRect
        if zoom > 0 {
            let center = PointLatLng::new(rect.lat() - (rect.height_lat() / 2.0), rect.lng() + (rect.width_lng() / 2.0));
            self.map.center_on(center.lat(), center.lng());
            let zoom = zoom.min(MAP_MAX_ZOOM);
            if self.map.zoom as i32 != zoom {
                self.map.zoom = zoom as f64;
            }
        }
    }

    /// `calcpolygonarea(polygon)`: shoelace over the polygon in its UTM zone, in m². The list
    /// is closed for the sum and the last point removed afterwards, so a polygon that was
    /// already closed comes back one point shorter (as the C# leaves `FPDrawnPolygon`).
    fn calcpolygonarea(&mut self, polygon: &mut Vec<(f64, f64)>) -> Result<f64, Exception> {
        use mp_core::extlibs::projnet::coordinatesystems::projectedcoordinatesystem::ProjectedCoordinateSystem;
        use mp_core::extlibs::projnet::coordinatesystems::transformations::coordinatetransformationfactory;
        use mp_core::extlibs::projnet::coordinatesystems::transformations::mathtransform::MathTransform;

        // should be a closed polygon; coords are in lat long; need utm to calc area
        if polygon.is_empty() {
            self.messages.push(("Please define a polygon!".into(), ""));
            return Ok(0.0);
        }

        // close the polygon
        if polygon[0] != polygon[polygon.len() - 1] {
            polygon.push(polygon[0]); // make a full loop
        }

        let utmzone = cast_i32((polygon[0].1 - -186.0) / 6.0);
        let utm = ProjectedCoordinateSystem::wgs84_utm(utmzone, !(polygon[0].0 < 0.0));
        let trans = coordinatetransformationfactory::create_from_wgs84_to_projected(&utm)?;

        let mut prod1 = 0.0;
        let mut prod2 = 0.0;
        for a in 0..polygon.len() - 1 {
            let pll1 = [polygon[a].1, polygon[a].0];
            let pll2 = [polygon[a + 1].1, polygon[a + 1].0];
            let p1 = trans.transform(&pll1)?;
            let p2 = trans.transform(&pll2)?;
            prod1 += p1[0] * p2[1];
            prod2 += p1[1] * p2[0];
        }

        let answer = (prod1 - prod2) / 2.0;

        if polygon[0] == polygon[polygon.len() - 1] {
            polygon.pop(); // unmake a full loop
        }

        Ok(answer.abs())
    }

    /// `BUT_Accept_Click`: `Some(calls)` closes the form.
    fn BUT_Accept_Click(&mut self) -> Option<Vec<(u16, [f64; 7])>> {
        let grid = self.grid.as_ref().filter(|g| !g.is_empty());
        let Some(grid) = grid else {
            self.messages.push(("Bad Grid".into(), "Error"));
            return None;
        };
        let mut calls = Vec::new();
        let mut lastpnt = PointLatLngAlt::zero();
        // (int)((float)NUM_UpDownFlySpeed.Value / CurrentState.multiplierspeed)
        let speed = decimal_to_double(self.d.NUM_UpDownFlySpeed.value) as f32 / crate::units::speed(1.0);
        calls.push((MAV_CMD_DO_CHANGE_SPEED, [1.0, cast_i32(speed as f64) as f64, 0.0, 0.0, 0.0, 0.0, 0.0]));
        for plla in grid {
            if plla.Tag == "M" {
                if self.d.CHK_internals {
                    calls.push((MAV_CMD_WAYPOINT, [0.0, 0.0, 0.0, 0.0, plla.Lng, plla.Lat, plla.Alt]));
                }
            } else {
                if !(plla.Lat == lastpnt.Lat && plla.Lng == lastpnt.Lng && plla.Alt == lastpnt.Alt) {
                    calls.push((MAV_CMD_WAYPOINT, [0.0, 0.0, 0.0, 0.0, plla.Lng, plla.Lat, plla.Alt]));
                }
                lastpnt = plla.clone();
            }
        }
        Some(calls)
    }

    /// Show the form for one frame.
    pub fn show(&mut self, ctx: &egui::Context, app: &mut AppCtx, host: &mut PluginHost) -> GridResult {
        let mut result = GridResult::Open;
        let mut open = true;
        let frame = egui::Frame::window(&ctx.global_style()).fill(crate::theme::bg()).inner_margin(0.0);
        let title = crate::i18n::tr(self.d.text).into_owned();
        // ShowDialog: the owner takes no input while the form is open
        let screen = ctx.content_rect();
        egui::Area::new(egui::Id::new("simplegrid_modal_block")).order(egui::Order::Middle).fixed_pos(screen.min).interactable(true).show(ctx, |ui| {
            ui.allocate_rect(screen, egui::Sense::click_and_drag());
            ui.painter().rect_filled(screen, 0.0, Color32::from_black_alpha(60));
        });
        egui::Window::new(title)
            .id(egui::Id::new("simplegrid_gridui"))
            .order(egui::Order::Foreground)
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_size(CLIENT_SIZE)
            .min_size(Vec2::new(TAB_WIDTH + 100.0, STATS_HEIGHT + 100.0))
            .frame(frame)
            .show(ctx, |ui| {
            let client = ui.available_size().max(Vec2::new(TAB_WIDTH + 100.0, STATS_HEIGHT + 100.0));
            let (full, _) = ui.allocate_exact_size(client, egui::Sense::hover());
            // GridUI_Resize: map.ZoomAndCenterMarkers("polygons")
            if client != self.client {
                self.client = client;
                self.ZoomAndCenterMarkers();
            }
            let o = full.min;
            let p = ui.painter().clone();
            p.rect_filled(full, 0.0, crate::theme::bg());
            if let Err(e) = self.map_area(ui, app, host, o) {
                self.messages.push((e.to_string(), ""));
            }
            self.d.paint_stats(&p, o, client);
            for ev in self.d.show_tabs(ui, o, &p, client) {
                match ev {
                    DesignerEvent::ValueChanged => {
                        if let Err(e) = self.domainUpDown1_ValueChanged(host) {
                            self.messages.push((e.to_string(), ""));
                        }
                    }
                    DesignerEvent::AcceptClick => {
                        if let Some(calls) = self.BUT_Accept_Click() {
                            result = GridResult::Accept(calls);
                        }
                    }
                }
            }
        });
        for (m, caption) in self.messages.drain(..) {
            app.toasts.message(m, caption);
        }
        if !open {
            result = GridResult::Cancel;
        }
        result
    }

    /// The map with the layer, and its mouse handlers.
    fn map_area(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, host: &mut PluginHost, o: Pos2) -> Result<(), Exception> {
        let map_rect = designer::layout(o, self.client).0;
        let assets = app.assets;
        let layer = self.layerpolygons.clone();
        let mut view_out = None;
        let consume = self.isMouseDown;
        ui.scope_builder(egui::UiBuilder::new().max_rect(map_rect), |ui| {
            map::show(ui, &mut self.map, app.tiles, |_ui, view, _resp, painter| {
                view_out = Some(*view);
                // GMapOverlay.OnRender: polygons, then markers, then tooltips
                for poly in &layer.polygons {
                    if poly.points.len() >= 2 {
                        let pts: Vec<Pos2> = poly.points.iter().map(|q| view.to_screen(q.0, q.1)).collect();
                        painter.add(Shape::closed_line(pts, Stroke::new(poly.width, poly.stroke)));
                    }
                }
                for m in &layer.markers {
                    let tex = match m.kind {
                        GMarkerGoogleType::green => assets.marker_green.id(),
                        GMarkerGoogleType::red => assets.marker_red.id(),
                    };
                    map::draw_pin(painter, tex, assets.marker_shadow.id(), view.to_screen(m.lat, m.lng));
                }
                for m in layer.markers.iter().filter(|m| m.mouse_over) {
                    if let Some(t) = &m.tooltip {
                        draw_tooltip(painter, view.to_screen(m.lat, m.lng), t);
                    }
                }
                // the form's own handlers pan on a left drag
                consume
            });
        });
        let Some(view) = view_out else { return Ok(()) };

        let (pointer, pressed, released, primary_down, alt) = ui.input(|i| {
            (
                i.pointer.hover_pos(),
                (i.pointer.button_pressed(egui::PointerButton::Primary), i.pointer.button_pressed(egui::PointerButton::Secondary)),
                (i.pointer.button_released(egui::PointerButton::Primary), i.pointer.button_released(egui::PointerButton::Secondary)),
                i.pointer.primary_down(),
                i.modifiers.alt,
            )
        });
        let over_map = pointer.is_some_and(|p| map_rect.contains(p)) && ui.ctx().layer_id_at(pointer.unwrap_or_default()) == Some(ui.layer_id());

        // MouseDown
        if over_map && (pressed.0 || pressed.1) {
            let pt = view.to_latlng(pointer.unwrap_or_default());
            self.map_MouseDown(pt, pressed.0, alt);
        }

        // MouseMove: the markers' enter/leave first (GMapControl.OnMouseMove), then the form's
        if let Some(p) = pointer.filter(|_| over_map || self.isMouseDown) {
            if self.last_pointer != Some(p) {
                self.last_pointer = Some(p);
                let mut events = Vec::new();
                for m in self.layerpolygons.markers.iter_mut() {
                    let sp = view.to_screen(m.lat, m.lng);
                    let area = Rect::from_min_size(sp + Vec2::new(-15.0, -31.0), Vec2::new(32.0, 32.0));
                    if area.contains(p) {
                        if !m.mouse_over {
                            m.mouse_over = true;
                            events.push(Some((m.lat, m.lng)));
                        }
                    } else if m.mouse_over {
                        m.mouse_over = false;
                        events.push(None);
                    }
                }
                for e in events {
                    match e {
                        Some((lat, lng)) => self.map_OnMarkerEnter(lat, lng),
                        None => self.map_OnMarkerLeave(),
                    }
                }
                let pt = view.to_latlng(p);
                if let Some((latdif, lngdif)) = self.map_MouseMove(host, pt, primary_down)? {
                    self.map.center_on(self.map.lat + latdif, self.map.lng + lngdif);
                }
            }
        }

        // MouseUp: the map has the capture of a press made on it
        if (released.0 || released.1) && (over_map || self.isMouseDown) {
            self.map_MouseUp(released.1 && !released.0, released.0);
        }
        Ok(())
    }
}

/// `GMapToolTip.OnRender`: a rounded box offset (14,-44) from the marker with a line back to
/// it, AliceBlue fill (alpha 222), MidnightBlue stroke, Navy bold text, 10px padding.
fn draw_tooltip(painter: &egui::Painter, p: Pos2, text: &str) {
    let font = egui::FontId::proportional(14.0);
    let galley = painter.layout_no_wrap(text.to_string(), font, Color32::from_rgb(0x00, 0x00, 0x80));
    let pad = Vec2::new(10.0, 10.0);
    let min = p + Vec2::new(14.0, -44.0);
    let r = Rect::from_min_size(min, galley.size() + pad * 2.0);
    let stroke = Stroke::new(2.0, Color32::from_rgb(0x19, 0x19, 0x70));
    painter.line_segment([p, Pos2::new(r.left(), r.center().y)], stroke);
    painter.rect_filled(r, 4.0, Color32::from_rgba_unmultiplied(0xf0, 0xf8, 0xff, 222));
    painter.rect_stroke(r, 4.0, stroke, egui::StrokeKind::Inside);
    painter.galley(r.min + pad, galley, Color32::from_rgb(0x00, 0x00, 0x80));
}
