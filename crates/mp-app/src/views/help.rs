//! Port of `GCSViews/Help`.

use egui::RichText;

pub fn ui(ui: &mut egui::Ui) {
    ui.heading("Mission Planner (Rust)");
    ui.label(format!("Version {}", env!("CARGO_PKG_VERSION")));
    ui.label(format!(
        "Platform: {} / {}",
        std::env::consts::OS,
        std::env::consts::ARCH
    ));
    ui.add_space(8.0);
    ui.label("A Rust port of ArduPilot Mission Planner. It is cross-platform: Windows, macOS, Linux, Android and iOS.");
    ui.add_space(8.0);
    ui.hyperlink_to(
        "Mission Planner documentation",
        "https://ardupilot.org/planner/",
    );
    ui.hyperlink_to("ArduPilot", "https://ardupilot.org/");
    ui.hyperlink_to("ArduPilot Discourse", "https://discuss.ardupilot.org/");
    ui.add_space(12.0);
    ui.label(RichText::new("Licensed GPL-3.0, like the original Mission Planner.").weak());
}
