//! Slippy-map widget (replaces GMap.NET): Web-Mercator tiles, async download with a disk
//! cache, pan/zoom (mouse, wheel, pinch) and a painter hook for overlays.

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::Arc;

use egui::{
    Color32, ColorImage, Pos2, Rect, Response, Sense, TextureHandle, TextureOptions, Ui, Vec2,
};
use parking_lot::{Condvar, Mutex};

pub const TILE: f64 = 256.0;
const MAX_TEXTURES: usize = 600;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Provider {
    GoogleSatellite,
    GoogleMap,
    GoogleHybrid,
    GoogleTerrain,
    BingSatellite,
    BingMap,
    BingHybrid,
    EsriWorldImagery,
    OpenStreetMap,
    EsriWorldStreet,
    EsriWorldTopo,
    OpenTopoMap,
    /// "Custom": tiles injected from disk (FlightPlanner Inject Custom Map), cache only
    Custom,
}

impl Provider {
    pub const ALL: [Provider; 13] = [
        Provider::GoogleSatellite,
        Provider::GoogleMap,
        Provider::GoogleHybrid,
        Provider::GoogleTerrain,
        Provider::BingSatellite,
        Provider::BingMap,
        Provider::BingHybrid,
        Provider::EsriWorldImagery,
        Provider::OpenStreetMap,
        Provider::EsriWorldStreet,
        Provider::EsriWorldTopo,
        Provider::OpenTopoMap,
        Provider::Custom,
    ];

    pub fn name(&self) -> &'static str {
        match self {
            Provider::GoogleSatellite => "GoogleSatelliteMap",
            Provider::GoogleMap => "GoogleMap",
            Provider::GoogleHybrid => "GoogleHybridMap",
            Provider::GoogleTerrain => "GoogleTerrainMap",
            Provider::BingSatellite => "BingSatelliteMap",
            Provider::BingMap => "BingMap",
            Provider::BingHybrid => "BingHybridMap",
            Provider::EsriWorldImagery => "EsriWorldImagery",
            Provider::OpenStreetMap => "OpenStreetMap",
            Provider::EsriWorldStreet => "EsriWorldStreetMap",
            Provider::EsriWorldTopo => "EsriWorldTopoMap",
            Provider::OpenTopoMap => "OpenTopoMap",
            Provider::Custom => "Custom",
        }
    }

    fn key(&self) -> &'static str {
        match self {
            Provider::GoogleSatellite => "google_sat",
            Provider::GoogleMap => "google_map",
            Provider::GoogleHybrid => "google_hyb",
            Provider::GoogleTerrain => "google_ter",
            Provider::BingSatellite => "bing_sat",
            Provider::BingMap => "bing_map",
            Provider::BingHybrid => "bing_hyb",
            Provider::EsriWorldImagery => "esri_img",
            Provider::OpenStreetMap => "osm",
            Provider::EsriWorldStreet => "esri_street",
            Provider::EsriWorldTopo => "esri_topo",
            Provider::OpenTopoMap => "opentopo",
            Provider::Custom => "custom",
        }
    }

    pub fn max_zoom(&self) -> u8 {
        match self {
            Provider::OpenTopoMap => 17,
            Provider::OpenStreetMap => 19,
            _ => 20,
        }
    }

    pub fn attribution(&self) -> &'static str {
        match self {
            Provider::OpenStreetMap => "© OpenStreetMap contributors",
            Provider::GoogleSatellite | Provider::GoogleMap | Provider::GoogleHybrid | Provider::GoogleTerrain => "©Google - Map data ©Google",
            Provider::BingSatellite | Provider::BingMap | Provider::BingHybrid => "© Microsoft Corporation",
            Provider::Custom => "Custom",
            Provider::OpenTopoMap => {
                "© OpenStreetMap contributors, SRTM | © OpenTopoMap (CC-BY-SA)"
            }
            _ => "Tiles © Esri",
        }
    }

    fn url(&self, z: u8, x: u32, y: u32) -> String {
        let s = (x + y) % 4;
        // Bing quadkey
        let quad = || -> String {
            (1..=z)
                .rev()
                .map(|i| {
                    let m = 1u32 << (i - 1);
                    char::from(b'0' + ((x & m != 0) as u8) + 2 * ((y & m != 0) as u8))
                })
                .collect()
        };
        match self {
            Provider::GoogleSatellite => format!("https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"),
            Provider::GoogleMap => format!("https://mt{s}.google.com/vt/lyrs=m&x={x}&y={y}&z={z}"),
            Provider::GoogleHybrid => format!("https://mt{s}.google.com/vt/lyrs=y&x={x}&y={y}&z={z}"),
            Provider::GoogleTerrain => format!("https://mt{s}.google.com/vt/lyrs=p&x={x}&y={y}&z={z}"),
            Provider::BingSatellite => format!("https://ecn.t{s}.tiles.virtualearth.net/tiles/a{}.jpeg?g=1", quad()),
            Provider::BingMap => format!("https://ecn.t{s}.tiles.virtualearth.net/tiles/r{}.png?g=1", quad()),
            Provider::BingHybrid => format!("https://ecn.t{s}.tiles.virtualearth.net/tiles/h{}.jpeg?g=1", quad()),
            Provider::EsriWorldImagery => format!(
                "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
            ),
            Provider::EsriWorldStreet => format!(
                "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}"
            ),
            Provider::EsriWorldTopo => format!(
                "https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}"
            ),
            Provider::OpenStreetMap => format!("https://tile.openstreetmap.org/{z}/{x}/{y}.png"),
            Provider::OpenTopoMap => format!("https://tile.opentopomap.org/{z}/{x}/{y}.png"),
            Provider::Custom => String::new(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct TileKey {
    provider: Provider,
    z: u8,
    x: u32,
    y: u32,
}

// ------------------------------------------------------------------ projection helpers

/// World pixel coordinates at zoom `z` (fractional zoom allowed).
pub fn latlng_to_world(lat: f64, lng: f64, z: f64) -> (f64, f64) {
    let lat = lat.clamp(-85.05112878, 85.05112878).to_radians();
    let scale = TILE * 2f64.powf(z);
    let x = (lng + 180.0) / 360.0 * scale;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) / 2.0 * scale;
    (x, y)
}

pub fn world_to_latlng(x: f64, y: f64, z: f64) -> (f64, f64) {
    let scale = TILE * 2f64.powf(z);
    let lng = x / scale * 360.0 - 180.0;
    let n = std::f64::consts::PI - 2.0 * std::f64::consts::PI * y / scale;
    let lat = n.sinh().atan().to_degrees();
    (lat, lng)
}

/// Snapshot of the map transform for one frame; overlays use it to project points.
#[derive(Debug, Clone, Copy)]
pub struct MapView {
    pub rect: Rect,
    pub zoom: f64,
    center_world: (f64, f64),
    rotation: egui::emath::Rot2,
}

impl MapView {
    fn new(rect: Rect, state: &MapState) -> Self {
        Self {
            rect,
            zoom: state.zoom,
            center_world: latlng_to_world(state.lat, state.lng, state.zoom),
            rotation: egui::emath::Rot2::from_angle(-(state.bearing as f32).to_radians()),
        }
    }

    fn world_to_screen(&self, x: f64, y: f64) -> Pos2 {
        self.rect.center() + self.rotation * Vec2::new(
            (x - self.center_world.0) as f32,
            (y - self.center_world.1) as f32,
        )
    }

    pub fn to_screen(&self, lat: f64, lng: f64) -> Pos2 {
        let (x, y) = latlng_to_world(lat, lng, self.zoom);
        let world_width = TILE * 2f64.powf(self.zoom);
        let dx = (x - self.center_world.0 + world_width / 2.0).rem_euclid(world_width) - world_width / 2.0;
        self.world_to_screen(self.center_world.0 + dx, y)
    }

    pub fn to_latlng(&self, p: Pos2) -> (f64, f64) {
        let d = self.rotation.inverse() * (p - self.rect.center());
        world_to_latlng(
            self.center_world.0 + d.x as f64,
            self.center_world.1 + d.y as f64,
            self.zoom,
        )
    }

    /// Metres per screen pixel at the view centre.
    pub fn metres_per_pixel(&self) -> f64 {
        let (lat, _) = self.to_latlng(self.rect.center());
        156_543.033_92 * lat.to_radians().cos() / 2f64.powf(self.zoom)
    }
}

// ------------------------------------------------------------------ state

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MapState {
    pub lat: f64,
    pub lng: f64,
    pub zoom: f64,
    pub provider: Provider,
    /// Heading at the top of the map, in degrees clockwise from north.
    #[serde(default)]
    pub bearing: f64,
}

impl Default for MapState {
    fn default() -> Self {
        Self {
            lat: -35.363261,
            lng: 149.165230,
            zoom: 16.0,
            // MainV2 default map: GoogleSatelliteMap
            provider: Provider::GoogleSatellite,
            bearing: 0.0,
        }
    }
}

impl MapState {
    pub fn center_on(&mut self, lat: f64, lng: f64) {
        self.lat = lat;
        self.lng = lng;
    }

    /// Fit the route in the current viewport, accounting for bearing and the dateline.
    pub fn fit_points(&mut self, points: &[(f64, f64)], size: Vec2) {
        let points: Vec<_> = points.iter().copied()
            .filter(|(a, b)| a.is_finite() && b.is_finite() && a.abs() <= 90.0 && b.abs() <= 180.0)
            .collect();
        let Some(&(lat, lng)) = points.first() else { return };
        let origin = latlng_to_world(lat, lng, 0.0);
        let rot = egui::emath::Rot2::from_angle(-(self.bearing as f32).to_radians());
        let mut bounds = Rect::NOTHING;
        for (lat, lng) in points {
            let (x, y) = latlng_to_world(lat, lng, 0.0);
            let dx = (x - origin.0 + TILE / 2.0).rem_euclid(TILE) - TILE / 2.0;
            bounds.extend_with(Pos2::ZERO + rot * Vec2::new(dx as f32, (y - origin.1) as f32));
        }
        let d = rot.inverse() * bounds.center().to_vec2();
        let (lat, lng) = world_to_latlng(origin.0 + d.x as f64, origin.1 + d.y as f64, 0.0);
        self.center_on(lat, (lng + 180.0).rem_euclid(360.0) - 180.0);
        let available = (size - Vec2::splat(80.0)).max(Vec2::splat(1.0));
        let scale = (available.x / bounds.width().max(1e-6)).min(available.y / bounds.height().max(1e-6));
        self.zoom = (scale as f64).log2().clamp(2.0, self.provider.max_zoom() as f64 + 2.0).min(20.0);
    }
}

// ------------------------------------------------------------------ tile loader

struct Queue {
    want: Mutex<Vec<TileKey>>,
    cv: Condvar,
}

pub struct TileCache {
    textures: HashMap<TileKey, (TextureHandle, u64)>,
    failed: HashSet<TileKey>,
    in_flight: Arc<Mutex<HashSet<TileKey>>>,
    queue: Arc<Queue>,
    done_rx: crossbeam_channel::Receiver<(TileKey, Option<ColorImage>)>,
    frame: u64,
    started: bool,
    done_tx: crossbeam_channel::Sender<(TileKey, Option<ColorImage>)>,
    cache_dir: Option<PathBuf>,
}

impl TileCache {
    #[cfg(test)]
    pub(crate) fn offline() -> Self {
        let mut cache = Self::new(None);
        cache.started = true;
        cache
    }

    pub fn new(cache_dir: Option<PathBuf>) -> Self {
        let (done_tx, done_rx) = crossbeam_channel::unbounded();
        Self {
            textures: HashMap::new(),
            failed: HashSet::new(),
            in_flight: Arc::new(Mutex::new(HashSet::new())),
            queue: Arc::new(Queue {
                want: Mutex::new(Vec::new()),
                cv: Condvar::new(),
            }),
            done_rx,
            frame: 0,
            started: false,
            done_tx,
            cache_dir,
        }
    }

    fn start_workers(&mut self, ctx: &egui::Context) {
        if self.started {
            return;
        }
        self.started = true;
        let agent: ureq::Agent = ureq::Agent::config_builder()
            .timeout_global(Some(std::time::Duration::from_secs(15)))
            .user_agent(concat!("MissionPlannerRS/", env!("CARGO_PKG_VERSION")))
            .build()
            .into();
        for i in 0..6 {
            let queue = self.queue.clone();
            let in_flight = self.in_flight.clone();
            let tx = self.done_tx.clone();
            let ctx = ctx.clone();
            let agent = agent.clone();
            let dir = self.cache_dir.clone();
            std::thread::Builder::new()
                .name(format!("tiles-{i}"))
                .spawn(move || loop {
                    let key = {
                        let mut q = queue.want.lock();
                        while q.is_empty() {
                            queue.cv.wait(&mut q);
                        }
                        let k = q.pop().unwrap();
                        in_flight.lock().insert(k);
                        k
                    };
                    let img = fetch_tile(&agent, dir.as_ref(), key);
                    in_flight.lock().remove(&key);
                    if tx.send((key, img)).is_err() {
                        return;
                    }
                    ctx.request_repaint();
                })
                .ok();
        }
    }

    fn poll(&mut self, ctx: &egui::Context) {
        while let Ok((key, img)) = self.done_rx.try_recv() {
            match img {
                Some(img) => {
                    let name = format!("tile-{}-{}-{}-{}", key.provider.key(), key.z, key.x, key.y);
                    let tex = ctx.load_texture(name, img, TextureOptions::LINEAR);
                    self.textures.insert(key, (tex, self.frame));
                }
                None => {
                    self.failed.insert(key);
                }
            }
        }
        if self.textures.len() > MAX_TEXTURES {
            let mut v: Vec<(TileKey, u64)> =
                self.textures.iter().map(|(k, (_, f))| (*k, *f)).collect();
            v.sort_by_key(|(_, f)| *f);
            for (k, _) in v.into_iter().take(self.textures.len() - MAX_TEXTURES) {
                self.textures.remove(&k);
            }
        }
    }

    fn get(&mut self, key: &TileKey) -> Option<egui::TextureId> {
        let frame = self.frame;
        self.textures.get_mut(key).map(|(t, f)| {
            *f = frame;
            t.id()
        })
    }
}

impl TileCache {
    /// GMap Prefetch: download every tile of `bounds` (min lat, min lng, max lat, max lng)
    /// for zooms z0..=z1 into the disk cache in the background. Returns the tile count.
    pub fn prefetch_area(&self, provider: Provider, bounds: (f64, f64, f64, f64), z0: u8, z1: u8) -> usize {
        let Some(dir) = self.cache_dir.clone() else { return 0 };
        let mut keys = Vec::new();
        for z in z0..=z1.min(provider.max_zoom()) {
            let (x0, y0) = latlng_to_world(bounds.2, bounds.1, z as f64);
            let (x1, y1) = latlng_to_world(bounds.0, bounds.3, z as f64);
            for x in (x0 / TILE) as u32..=(x1 / TILE) as u32 {
                for y in (y0 / TILE) as u32..=(y1 / TILE) as u32 {
                    keys.push(TileKey { provider, z, x, y });
                    if keys.len() >= 5000 {
                        break;
                    }
                }
            }
        }
        let n = keys.len();
        std::thread::spawn(move || {
            let agent: ureq::Agent = ureq::Agent::config_builder()
                .timeout_global(Some(std::time::Duration::from_secs(15)))
                .user_agent(concat!("MissionPlannerRS/", env!("CARGO_PKG_VERSION")))
                .build()
                .into();
            for key in keys {
                let p = tile_path(&dir, key);
                if p.exists() {
                    continue;
                }
                if let Ok(mut resp) = agent.get(&key.provider.url(key.z, key.x, key.y)).call() {
                    if let Ok(body) = resp.body_mut().read_to_vec() {
                        if let Some(parent) = p.parent() {
                            let _ = std::fs::create_dir_all(parent);
                        }
                        let _ = std::fs::write(p, body);
                    }
                }
            }
        });
        n
    }
}

/// BUT_InjectCustomMap: copy a z/y/x tile folder (".../Z<zoom>/<y>/<x>.jpg|png") into the
/// "Custom" provider cache. Returns tiles per zoom level.
pub fn inject_custom_tiles(cache: &std::path::Path, from: &std::path::Path) -> std::collections::BTreeMap<u8, usize> {
    let mut counts = std::collections::BTreeMap::new();
    let mut stack = vec![from.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let Ok(rd) = std::fs::read_dir(&dir) else { continue };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
                continue;
            }
            let ext = p.extension().map(|x| x.to_string_lossy().to_lowercase()).unwrap_or_default();
            if !matches!(ext.as_str(), "jpg" | "jpeg" | "png") {
                continue;
            }
            // ...\Z?<zoom>\<y>\<x>.ext
            let comps: Vec<String> = p.components().rev().take(3).map(|c| c.as_os_str().to_string_lossy().to_string()).collect();
            if comps.len() < 3 {
                continue;
            }
            let x = comps[0].split('.').next().and_then(|v| v.parse::<u32>().ok());
            let y = comps[1].parse::<u32>().ok();
            let z = comps[2].trim_start_matches(['Z', 'z']).parse::<u8>().ok();
            let (Some(x), Some(y), Some(z)) = (x, y, z) else { continue };
            let to = tile_path(&cache.to_path_buf(), TileKey { provider: Provider::Custom, z, x, y });
            if let Some(parent) = to.parent() {
                let _ = std::fs::create_dir_all(parent);
            }
            if std::fs::copy(&p, &to).is_ok() {
                *counts.entry(z).or_insert(0) += 1;
            }
        }
    }
    counts
}

fn tile_path(dir: &PathBuf, key: TileKey) -> PathBuf {
    dir.join(key.provider.key())
        .join(key.z.to_string())
        .join(key.x.to_string())
        .join(format!("{}.tile", key.y))
}

fn fetch_tile(agent: &ureq::Agent, dir: Option<&PathBuf>, key: TileKey) -> Option<ColorImage> {
    let bytes = dir
        .and_then(|d| std::fs::read(tile_path(d, key)).ok())
        .or_else(|| {
            let url = key.provider.url(key.z, key.x, key.y);
            if url.is_empty() {
                return None;
            }
            let mut resp = agent.get(&url).call().ok()?;
            let body = resp.body_mut().read_to_vec().ok()?;
            if let Some(d) = dir {
                let p = tile_path(d, key);
                if let Some(parent) = p.parent() {
                    let _ = std::fs::create_dir_all(parent);
                }
                let _ = std::fs::write(p, &body);
            }
            Some(body)
        })?;
    let img = image::load_from_memory(&bytes).ok()?.to_rgba8();
    let size = [img.width() as usize, img.height() as usize];
    Some(ColorImage::from_rgba_unmultiplied(size, img.as_raw()))
}

// ------------------------------------------------------------------ widget

pub struct MapOutput {
    pub response: Response,
    pub view: MapView,
}

/// Draw the map. `overlay` paints on top and returns `true` if it consumed the primary drag
/// (e.g. a waypoint marker is being dragged), in which case the map does not pan.
pub fn show(
    ui: &mut Ui,
    state: &mut MapState,
    cache: &mut TileCache,
    overlay: impl FnOnce(&mut Ui, &MapView, &Response, &egui::Painter) -> bool,
) -> MapOutput {
    let ctx = ui.ctx().clone();
    cache.start_workers(&ctx);
    cache.poll(&ctx);
    cache.frame += 1;

    let size = ui.available_size();
    let (response, painter) = ui.allocate_painter(size, Sense::click_and_drag());
    let rect = response.rect;
    painter.rect_filled(rect, 0.0, Color32::from_gray(30));

    let max_z = state.provider.max_zoom() as f64;
    state.zoom = state.zoom.clamp(2.0, max_z + 2.0);

    // Zoom about the pointer (wheel / pinch).
    if response.hovered() || response.contains_pointer() {
        let (scroll, pinch, hover) = ui.input(|i| {
            (
                i.smooth_scroll_delta.y,
                i.zoom_delta(),
                i.pointer.hover_pos(),
            )
        });
        let mut dz = 0.0;
        if scroll != 0.0 {
            dz += scroll as f64 / 200.0;
        }
        if pinch != 1.0 {
            dz += (pinch as f64).log2();
        }
        if dz != 0.0 {
            let anchor = hover.unwrap_or(rect.center());
            let before = MapView::new(rect, state);
            let (alat, alng) = before.to_latlng(anchor);
            state.zoom = (state.zoom + dz).clamp(2.0, max_z + 2.0);
            // keep anchor fixed
            let (ax, ay) = latlng_to_world(alat, alng, state.zoom);
            let d = before.rotation.inverse() * (anchor - rect.center());
            let cw = (ax - d.x as f64, ay - d.y as f64);
            let (lat, lng) = world_to_latlng(cw.0, cw.1, state.zoom);
            state.lat = lat;
            state.lng = lng;
        }
    }

    let view = MapView::new(rect, state);

    draw_tiles(&painter, &view, state.provider, cache);

    let painter_overlay = painter.with_clip_rect(rect);
    let consumed = overlay(ui, &view, &response, &painter_overlay);

    if !consumed && response.dragged_by(egui::PointerButton::Primary) {
        let d = view.rotation.inverse() * response.drag_delta();
        let cw = (
            view.center_world.0 - d.x as f64,
            view.center_world.1 - d.y as f64,
        );
        let (lat, lng) = world_to_latlng(cw.0, cw.1, state.zoom);
        state.lat = lat.clamp(-85.0, 85.0);
        state.lng = ((lng + 540.0) % 360.0) - 180.0;
    }

    // attribution + scale bar
    painter.text(
        rect.right_bottom() - Vec2::new(4.0, 2.0),
        egui::Align2::RIGHT_BOTTOM,
        state.provider.attribution(),
        egui::FontId::proportional(10.0),
        Color32::from_white_alpha(200),
    );
    let _ = draw_scale;

    MapOutput { response, view }
}

fn draw_tiles(painter: &egui::Painter, view: &MapView, provider: Provider, cache: &mut TileCache) {
    let max_z = provider.max_zoom();
    let z = (view.zoom.floor() as i32).clamp(0, max_z as i32) as u8;
    let scale = 2f64.powf(view.zoom - z as f64); // pixel scale of a z-level tile
    let tile_px = TILE * scale;
    let cw_z = (view.center_world.0 / scale, view.center_world.1 / scale); // centre in z-level pixels
    let mut bounds = Rect::NOTHING;
    for p in [view.rect.left_top(), view.rect.right_top(), view.rect.right_bottom(), view.rect.left_bottom()] {
        let d = view.rotation.inverse() * (p - view.rect.center());
        bounds.extend_with(Pos2::new(d.x, d.y));
    }
    let left = cw_z.0 + bounds.left() as f64 / scale;
    let top = cw_z.1 + bounds.top() as f64 / scale;
    let right = cw_z.0 + bounds.right() as f64 / scale;
    let bottom = cw_z.1 + bounds.bottom() as f64 / scale;
    let n = 1u32 << z;
    let (tx0, tx1) = ((left / TILE).floor() as i64, (right / TILE).floor() as i64);
    let (ty0, ty1) = (
        (top / TILE).floor().max(0.0) as i64,
        ((bottom / TILE).floor() as i64).min(n as i64 - 1),
    );

    let mut wanted = Vec::new();
    let uv_full = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
    let center_tile = ((cw_z.0 / TILE) as i64, (cw_z.1 / TILE) as i64);
    for ty in ty0..=ty1 {
        for tx in tx0..=tx1 {
            let wx = tx.rem_euclid(n as i64) as u32;
            let key = TileKey {
                provider,
                z,
                x: wx,
                y: ty as u32,
            };
            let sx = view.rect.center().x as f64 + (tx as f64 * TILE - cw_z.0) * scale;
            let sy = view.rect.center().y as f64 + (ty as f64 * TILE - cw_z.1) * scale;
            let r = Rect::from_min_size(
                Pos2::new(sx as f32, sy as f32),
                Vec2::splat(tile_px as f32 + 0.5),
            );
            if let Some(id) = cache.get(&key) {
                paint_tile(painter, view, id, r, uv_full);
                continue;
            }
            // Fall back to a cached ancestor, cropped and scaled.
            for up in 1..=4u8 {
                if up > z {
                    break;
                }
                let pz = z - up;
                let pk = TileKey {
                    provider,
                    z: pz,
                    x: wx >> up,
                    y: (ty as u32) >> up,
                };
                if let Some(id) = cache.get(&pk) {
                    let f = 1.0 / (1u32 << up) as f32;
                    let ox = (wx & ((1 << up) - 1)) as f32 * f;
                    let oy = ((ty as u32) & ((1 << up) - 1)) as f32 * f;
                    let uv = Rect::from_min_size(Pos2::new(ox, oy), Vec2::splat(f));
                    paint_tile(painter, view, id, r, uv);
                    break;
                }
            }
            if !cache.failed.contains(&key) && !cache.in_flight.lock().contains(&key) {
                let d = (tx - center_tile.0).abs() + (ty - center_tile.1).abs();
                wanted.push((d, key));
            }
        }
    }
    // Replace the queue: nearest-to-centre tiles are popped first (LIFO).
    wanted.sort_by_key(|(d, _)| std::cmp::Reverse(*d));
    let mut q = cache.queue.want.lock();
    q.clear();
    q.extend(wanted.into_iter().map(|(_, k)| k));
    if !q.is_empty() {
        cache.queue.cv.notify_all();
    }
}

fn paint_tile(painter: &egui::Painter, view: &MapView, id: egui::TextureId, rect: Rect, uv: Rect) {
    let mut mesh = egui::Mesh::with_texture(id);
    mesh.add_rect_with_uv(rect, uv, Color32::WHITE);
    for v in &mut mesh.vertices {
        v.pos = view.rect.center() + view.rotation * (v.pos - view.rect.center());
    }
    painter.add(mesh);
}

fn draw_scale(painter: &egui::Painter, view: &MapView) {
    let mpp = view.metres_per_pixel();
    let target = mpp * 120.0;
    let mag = 10f64.powf(target.log10().floor());
    let nice = [1.0, 2.0, 5.0, 10.0]
        .iter()
        .map(|m| m * mag)
        .filter(|v| *v <= target)
        .last()
        .unwrap_or(mag);
    let px = (nice / mpp) as f32;
    let o = view.rect.left_bottom() + Vec2::new(10.0, -14.0);
    let s = egui::Stroke::new(2.0, Color32::WHITE);
    painter.line_segment([o, o + Vec2::new(px, 0.0)], s);
    painter.line_segment([o, o + Vec2::new(0.0, -5.0)], s);
    painter.line_segment([o + Vec2::new(px, 0.0), o + Vec2::new(px, -5.0)], s);
    let label = if nice >= 1000.0 {
        format!("{} km", nice / 1000.0)
    } else {
        format!("{nice} m")
    };
    painter.text(
        o + Vec2::new(px / 2.0, -4.0),
        egui::Align2::CENTER_BOTTOM,
        label,
        egui::FontId::proportional(11.0),
        Color32::WHITE,
    );
}

// ------------------------------------------------------------------ overlay helpers

const UV1: Rect = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));

/// `GMarkerGoogle` (green/red pin): 32x32 with `Offset = (-Size.Width/2 + 1, -Size.Height + 1)`
/// and the 59x32 `msmarker.shadow` drawn first at the same origin.
pub fn draw_pin(painter: &egui::Painter, pin: egui::TextureId, shadow: egui::TextureId, p: Pos2) -> Rect {
    let tl = p + Vec2::new(-15.0, -31.0);
    painter.image(shadow, Rect::from_min_size(tl, Vec2::new(59.0, 32.0)), UV1, Color32::WHITE);
    let r = Rect::from_min_size(tl, Vec2::new(32.0, 32.0));
    painter.image(pin, r, UV1, Color32::WHITE);
    r
}

/// `GMapMarkerWP`: green pin, red ellipse when selected, number in black at (+10, +3) shown
/// when zoom > 16 or the mouse is over it.
pub fn draw_wp(painter: &egui::Painter, pin: egui::TextureId, shadow: egui::TextureId, p: Pos2, label: &str, selected: bool, zoom: f64, hover: bool) -> Rect {
    let tl = p + Vec2::new(-15.0, -31.0);
    let r = Rect::from_min_size(tl, Vec2::new(32.0, 32.0));
    if selected {
        painter.add(egui::Shape::ellipse_filled(r.center(), r.size() / 2.0, Color32::RED));
    }
    draw_pin(painter, pin, shadow, p);
    if zoom > 16.0 || hover {
        let mut x = tl.x + 10.0;
        if label.len() > 2 {
            x -= 4.0;
        }
        painter.text(Pos2::new(x, tl.y + 3.0), egui::Align2::LEFT_TOP, label, crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::BLACK);
    }
    r
}

/// `GMapMarkerQuad` heading lines (length 500): red = heading, black = COG, orange = target.
pub struct LineOpts {
    pub heading: bool,
    pub cog: bool,
    pub target: bool,
    pub nav_bearing: bool,
    pub length: f32,
}

pub fn draw_heading_lines(painter: &egui::Painter, p: Pos2, heading: f32, cog: f32, target: f32, nav: f32, o: &LineOpts) {
    let l = |deg: f32, col: Color32| {
        let a = (deg - 90.0).to_radians();
        painter.line_segment([p, p + Vec2::new(a.cos(), a.sin()) * o.length], egui::Stroke::new(2.0, col));
    };
    if o.heading {
        l(heading, Color32::RED);
    }
    if o.nav_bearing {
        l(nav, Color32::from_rgb(0, 128, 0));
    }
    if o.cog {
        l(cog, Color32::BLACK);
    }
    if o.target {
        l(target, Color32::from_rgb(255, 165, 0));
    }
}

/// `GMapMarkerQuad` frame: four motor circles, blue front arm, green body, red sysid.
pub fn draw_quad(painter: &egui::Painter, p: Pos2, heading: f32, sysid: u8) {
    let a = heading.to_radians();
    let (s, c) = a.sin_cos();
    let t = |x: f32, y: f32| Pos2::new(p.x + x * c - y * s, p.y + x * s + y * c);
    let green = Color32::from_rgb(0x8d, 0xc6, 0x3f);
    let blue = Color32::from_rgb(0x00, 0xae, 0xef);
    let gs = egui::Stroke::new(3.0, green);
    for (mx, my) in [(0.0, -23.0), (0.0, 22.0), (22.0, 0.0), (-23.0, 0.0)] {
        painter.circle_stroke(t(mx, my), 10.0, gs);
        painter.circle_stroke(t(mx, my), 2.5, gs);
    }
    painter.line_segment([t(0.0, -23.0), t(0.0, 0.0)], egui::Stroke::new(3.0, blue));
    painter.line_segment([t(0.0, 1.0), t(0.0, 22.0)], gs);
    painter.line_segment([t(22.0, 0.0), t(-23.0, 0.0)], gs);
    let body = [t(-3.0, -5.0), t(2.0, -5.0), t(2.0, 3.0), t(-3.0, 3.0)];
    painter.add(egui::Shape::convex_polygon(body.to_vec(), green, egui::Stroke::NONE));
    painter.text(p + Vec2::new(-8.0, -8.0), egui::Align2::LEFT_TOP, sysid.to_string(), egui::FontId::monospace(15.0 * 1.33), Color32::RED);
}

#[cfg(test)]
mod provider_tests {
    use super::*;

    #[test]
    fn old_map_settings_default_to_north_up() {
        let state: MapState = serde_json::from_str(r#"{"lat":10,"lng":20,"zoom":15,"provider":"OpenStreetMap"}"#).unwrap();
        assert_eq!(state.bearing, 0.0);
    }

    #[test]
    fn bearing_rotates_projection_and_preserves_inverse() {
        let rect = Rect::from_min_size(Pos2::new(15.0, 20.0), Vec2::new(800.0, 600.0));
        let state = MapState { lat: 12.0, lng: 79.0, zoom: 16.0, bearing: 90.0, ..Default::default() };
        let view = MapView::new(rect, &state);
        let east = view.to_screen(12.0, 79.001);
        assert!(east.y < rect.center().y);
        assert!((east.x - rect.center().x).abs() < 0.01);
        for bearing in [0.0, 45.0, 90.0, 180.0, 359.0] {
            let view = MapView::new(rect, &MapState { bearing, ..state.clone() });
            for p in [rect.left_top(), rect.center(), rect.right_bottom()] {
                let (a, b) = view.to_latlng(p);
                assert!(view.to_screen(a, b).distance(p) < 0.02);
            }
        }
    }

    #[test]
    fn rotated_map_keeps_wheel_anchor_and_pans_with_pointer() {
        let ctx = egui::Context::default();
        let mut cache = TileCache::offline();
        let mut state = MapState { bearing: 90.0, ..Default::default() };
        let mut frame = |state: &mut MapState, events: Vec<egui::Event>| {
            let mut view = None;
            let mut output = ctx.run_ui(egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0))),
                events, ..Default::default()
            }, |ui| { view = Some(show(ui, state, &mut cache, |_, _, _, _| false).view); });
            output.textures_delta.clear();
            view.unwrap()
        };
        let anchor = Pos2::new(300.0, 200.0);
        frame(&mut state, vec![egui::Event::PointerMoved(anchor)]);
        let before = frame(&mut state, Vec::new());
        let (lat, lng) = before.to_latlng(anchor);
        let after = frame(&mut state, vec![egui::Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point, delta: Vec2::new(0.0, 60.0), modifiers: egui::Modifiers::NONE,
            phase: egui::TouchPhase::Move, source: egui::MouseWheelSource::Unknown,
        }]);
        assert!(after.zoom > before.zoom);
        assert!(after.to_screen(lat, lng).distance(anchor) < 0.02);

        // Finish the wheel smoothing before starting a separate pan gesture.
        for _ in 0..120 { frame(&mut state, Vec::new()); }
        frame(&mut state, vec![egui::Event::PointerButton {
            pos: anchor, button: egui::PointerButton::Primary, pressed: true, modifiers: egui::Modifiers::NONE
        }]);
        let start = frame(&mut state, Vec::new()).to_latlng(anchor);
        let moved = anchor + Vec2::new(40.0, 20.0);
        frame(&mut state, vec![egui::Event::PointerMoved(moved)]);
        let view = frame(&mut state, Vec::new());
        assert!(view.to_screen(start.0, start.1).distance(moved) < 0.02);
    }

    #[test]
    fn fit_route_at_bearing_and_across_dateline() {
        let size = Vec2::new(800.0, 600.0);
        for bearing in [0.0, 45.0, 90.0, 200.0] {
            let mut state = MapState { bearing, ..Default::default() };
            for pts in [vec![(12.0, 79.0), (12.1, 79.2), (11.8, 79.1)], vec![(10.0, 179.95), (10.1, -179.95)]] {
                state.fit_points(&pts, size);
                let rect = Rect::from_min_size(Pos2::ZERO, size);
                let view = MapView::new(rect, &state);
                for (a, b) in pts {
                    assert!(rect.shrink(35.0).contains(view.to_screen(a, b)), "{state:?}");
                }
                assert!(state.zoom > 8.0);
            }
        }
    }

    #[test]
    fn rotated_tiles_match_overlay_coordinates() {
        let ctx = egui::Context::default();
        let state = MapState { lat: 0.0, lng: 0.0, zoom: 3.0, bearing: 37.0, ..Default::default() };
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0));
        let view = MapView::new(rect, &state);
        let tile = Rect::from_min_size(rect.center(), Vec2::splat(256.0));
        let mut output = ctx.run_pass(egui::RawInput::default(), |ctx| {
            let painter = ctx.layer_painter(egui::LayerId::background());
            paint_tile(&painter, &view, egui::TextureId::Managed(1), tile, UV1);
        });
        output.textures_delta.clear();
        let (lat, lng) = world_to_latlng(view.center_world.0 + 256.0, view.center_world.1 + 256.0, state.zoom);
        let expected = rect.center() + view.rotation * Vec2::splat(256.0);
        assert!(view.to_screen(lat, lng).distance(expected) < 0.01);
        let mesh = output.shapes.iter().find_map(|s| match &s.shape {
            egui::Shape::Mesh(m) => Some(m),
            _ => None,
        }).unwrap();
        assert!(mesh.vertices.iter().any(|v| v.pos.distance(view.to_screen(lat, lng)) < 0.01));
    }

    #[test]
    fn bing_quadkey() {
        // Bing Maps docs: tile (3, 5) at level 3 -> quadkey 213
        assert!(super::Provider::BingSatellite.url(3, 3, 5).contains("/a213.jpeg"));
    }
}
