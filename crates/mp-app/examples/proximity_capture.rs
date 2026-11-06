//! Content-only render fixture: same production painter and JSON data as the C# harness.
//! This does not certify the enclosing application window or cross-platform pixels.
use std::{path::PathBuf, time::{Duration, Instant}};
use egui::{Pos2, Vec2};
use mp_core::{mavlink::dialects::ardupilotmega::*, proximity::{Proximity, Reading}};

struct Capture {
    radar: mp_app::tool_windows::ProximityView,
    readings: Vec<Reading>,
    copter: bool,
    path: PathBuf,
    frames: u32,
    started: Instant,
}
impl eframe::App for Capture {
    fn ui(&mut self, ui: &mut egui::Ui, _: &mut eframe::Frame) {
        let rect = ui.max_rect();
        self.radar.paint(ui, rect, &self.readings, self.copter);
        self.frames += 1;
        let ctx = ui.ctx();
        ctx.request_repaint_after(Duration::from_millis(30));
        if self.frames == 6 {
            let path = self.path.clone();
            let copter = self.copter;
            let callback_ctx = ctx.clone();
            ctx.request_screenshot(move |image| {
                let bytes: Vec<u8> = image.pixels.iter().flat_map(|p| p.to_array()).collect();
                image::save_buffer(&path, &bytes, image.width() as u32, image.height() as u32, image::ColorType::Rgba8).expect("capture PNG");
                std::fs::write(path.with_extension("capture.json"), serde_json::to_vec_pretty(&serde_json::json!({
                    "source": "actual eframe viewport, production ProximityView::paint",
                    "fixture": "scripts/fixtures/proximity.json", "frozen_telemetry": true,
                    "width": image.width(), "height": image.height(), "copter": copter,
                    "pixels_per_point": 1, "native_form_chrome": false
                })).unwrap()).expect("capture receipt");
                callback_ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            });
        }
        if self.started.elapsed() > Duration::from_secs(15) { ctx.send_viewport_cmd(egui::ViewportCommand::Close); }
    }
}
fn main() -> eframe::Result {
    let args: Vec<_> = std::env::args().collect();
    let path = PathBuf::from(args.get(1).expect("output PNG path"));
    let width: f32 = args.get(2).map(|v| v.parse().unwrap()).unwrap_or(430.0);
    let height: f32 = args.get(3).map(|v| v.parse().unwrap()).unwrap_or(391.0);
    let copter = args.get(4).map(|s| s != "plane").unwrap_or(true);
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    let fixture: serde_json::Value = serde_json::from_str(include_str!("../../../scripts/fixtures/proximity.json")).unwrap();
    let now = Instant::now();
    let mut proximity = Proximity::default();
    for (i, cm) in fixture["sensors_cm"].as_array().unwrap().iter().enumerate() {
        let mut d = DISTANCE_SENSOR_DATA { id: i as u8, min_distance: 1, max_distance: 2000, current_distance: cm.as_u64().unwrap() as u16, ..Default::default() };
        d.orientation = [MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_45, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_90, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_135, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_180, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_225, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_270, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_315][i];
        proximity.update(1, 1, &MavMessage::DISTANCE_SENSOR(d), 0.0, now);
    }
    // Separate angles retain the four scan origins within the same reference MAVState.
    for a in fixture["arrays"].as_array().unwrap() {
        let mut d = OBSTACLE_DISTANCE_DATA { frame: MavFrame::MAV_FRAME_BODY_FRD, increment_f: a["width"].as_f64().unwrap() as f32, angle_offset: a["angle"].as_f64().unwrap() as f32, min_distance: 1, max_distance: 2000, ..Default::default() };
        d.distances.fill(u16::MAX); d.distances[0] = a["distance_cm"].as_u64().unwrap() as u16;
        proximity.update(1, 1, &MavMessage::OBSTACLE_DISTANCE(d), 0.0, now);
    }
    let readings = proximity.readings(1, 1, now);
    let radar = mp_app::tool_windows::ProximityView::with_dimensions(
        fixture["radius_cm"].as_f64().expect("fixture radius") as f32,
        fixture["vehicle_cm"].as_f64().expect("fixture vehicle size") as f32,
    ).expect("positive finite fixture dimensions");
    eframe::run_native("Proximity content fixture", eframe::NativeOptions {
        persist_window: false,
        viewport: egui::ViewportBuilder::default().with_inner_size(Vec2::new(width,height)).with_decorations(false).with_resizable(false).with_position(Pos2::new(20.0,20.0)),
        ..Default::default()
    }, Box::new(move |cc| {
        cc.egui_ctx.set_pixels_per_point(1.0);
        mp_app::fonts::install(&cc.egui_ctx);
        // App::ui root covers the full client area, with no panel margins/chrome.
        Ok(Box::new(Capture { radar, readings, copter, path, frames: 0, started: Instant::now() }))
    }))
}
