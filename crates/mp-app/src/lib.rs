//! Mission Planner UI (egui/eframe). One codebase for desktop, Android and iOS.

pub mod app;
pub mod assets;
pub mod fonts;
mod demo;
pub mod hud;
pub mod map;
pub mod platform;
pub mod theme;
pub mod views;

pub use app::MissionPlannerApp;

pub const APP_NAME: &str = "Mission Planner";

fn options() -> eframe::NativeOptions {
    eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(format!("{APP_NAME} {} (Rust)", env!("CARGO_PKG_VERSION")))
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([360.0, 480.0]),
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
