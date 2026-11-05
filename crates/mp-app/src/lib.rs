#![allow(deprecated)]
//! Mission Planner UI (egui/eframe). One codebase for desktop, Android and iOS.

pub mod app;
pub mod assets;
pub mod firmware_ui;
pub mod fonts;
pub mod forms;
mod demo;
pub mod hud;
pub mod joystick;
pub mod log_browse;
pub mod map;
pub mod mjpeg;
pub mod platform;
pub mod rtcm;
pub mod sitl;
pub mod speech;
pub mod srtm;
pub mod theme;
pub mod tool_windows;
pub mod tools;
pub mod views;
pub mod zedgraph;

pub use app::MissionPlannerApp;

pub const APP_NAME: &str = "Mission Planner";

/// Program.cs: `name + " " + ProductVersion + " build " + version`.
pub fn title_base() -> String {
    format!("{APP_NAME} 1.3.83 build {}-rs", env!("CARGO_PKG_VERSION"))
}

fn options() -> eframe::NativeOptions {
    eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(title_base())
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([360.0, 480.0])
            // MP_DESKTOP: start hidden, move to that virtual desktop, then show (see platform)
            .with_visible(std::env::var_os("MP_DESKTOP").is_none()),
        ..Default::default()
    }
}

/// Desktop entry point (Windows, macOS, Linux) and iOS.
pub fn run() -> eframe::Result {
    eframe::run_native(
        APP_NAME,
        options(),
        Box::new(|cc| Ok(Box::new(MissionPlannerApp::new(cc)))),
    )
}

/// Android entry point (NativeActivity).
#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(app: winit::platform::android::activity::AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default().with_max_level(log::LevelFilter::Info),
    );
    if let Some(p) = app.internal_data_path() {
        platform::set_data_dir(p);
    }
    let mut opts = options();
    opts.android_app = Some(app);
    if let Err(e) = eframe::run_native(
        APP_NAME,
        opts,
        Box::new(|cc| Ok(Box::new(MissionPlannerApp::new(cc)))),
    ) {
        log::error!("eframe: {e}");
    }
}

/// iOS entry point, called from the Xcode wrapper's `main`.
#[cfg(target_os = "ios")]
#[unsafe(no_mangle)]
pub extern "C" fn start_mission_planner() {
    if let Err(e) = run() {
        log::error!("eframe: {e}");
    }
}
