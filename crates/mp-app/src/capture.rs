//! Explicit developer capture mode. Export the renderer's actual viewport and exit.
//! This captures application pixels, not the Windows title bar or other applications.

use std::path::PathBuf;
use std::time::{Duration, Instant};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

pub struct Capture {
    path: PathBuf,
    started: Instant,
    after: Duration,
    requested: bool,
    keep_open: bool,
    finished: Arc<AtomicBool>,
}

impl Capture {
    pub fn from_env() -> Option<Self> {
        let path = PathBuf::from(std::env::var_os("MP_CAPTURE")?);
        let seconds = std::env::var("MP_CAPTURE_AFTER").ok().and_then(|v| v.parse::<f64>().ok())
            .filter(|n| n.is_finite() && *n >= 0.0 && *n <= 300.0).unwrap_or(7.0);
        Some(Self { path, started: Instant::now(), after: Duration::from_secs_f64(seconds), requested: false,
            keep_open: std::env::var_os("MP_CAPTURE_KEEP_OPEN").is_some(), finished: Arc::new(AtomicBool::new(false)) })
    }

    pub fn tick(&mut self, ctx: &egui::Context, ready: bool, runtime: serde_json::Value) {
        if self.finished.load(Ordering::Acquire) { return; }
        if self.started.elapsed() > self.after + Duration::from_secs(20) {
            log::error!("Renderer capture timed out: {}", self.path.display());
            self.finished.store(true, Ordering::Release);
            if !self.keep_open { ctx.send_viewport_cmd(egui::ViewportCommand::Close); }
            return;
        }
        ctx.request_repaint_after(Duration::from_millis(50));
        if !ready || self.requested || self.started.elapsed() < self.after { return }
        self.requested = true;
        let path = self.path.clone();
        let ctx = ctx.clone();
        let callback_ctx = ctx.clone();
        let keep_open = self.keep_open;
        let finished = self.finished.clone();
        ctx.request_screenshot(move |image| {
            let result = (|| -> Result<(), String> {
                if let Some(parent) = path.parent() { std::fs::create_dir_all(parent).map_err(|e| e.to_string())?; }
                let pixels: Vec<u8> = image.pixels.iter().flat_map(|p| p.to_array()).collect();
                image::save_buffer(&path, &pixels, image.width() as u32, image.height() as u32, image::ColorType::Rgba8).map_err(|e| e.to_string())?;
                let receipt = serde_json::json!({ "status": "ok", "width": image.width(), "height": image.height(),
                    "source": "eframe renderer viewport", "view": std::env::var("MP_VIEW").ok(),
                    "page": std::env::var("MP_PAGE").ok(), "tab": std::env::var("MP_FDTAB").ok(),
                    "autosim": std::env::var("MP_AUTOSIM").ok(), "demo": std::env::var("MP_DEMO").ok(),
                    "desktop": std::env::var("MP_DESKTOP").ok(), "runtime": runtime });
                std::fs::write(path.with_extension("capture.json"), serde_json::to_vec_pretty(&receipt).unwrap()).map_err(|e| e.to_string())
            })();
            if let Err(e) = result { log::error!("Renderer capture failed: {e}"); }
            finished.store(true, Ordering::Release);
            if !keep_open { callback_ctx.send_viewport_cmd(egui::ViewportCommand::Close); }
            callback_ctx.request_repaint();
        });
    }
}
