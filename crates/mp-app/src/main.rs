#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]

fn main() -> eframe::Result {
    #[cfg(not(any(target_os = "android", target_os = "ios")))]
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    mp_app::run()
}
