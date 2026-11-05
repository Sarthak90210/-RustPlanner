//! Port of `GCSViews/SITL`: start the built-in simulator or connect to ArduPilot SITL.

use egui::RichText;
use mp_core::comms::ConnectionSpec;

use crate::app::{AppAction, AppCtx};
use crate::theme;

pub fn ui(ui: &mut egui::Ui, app: &mut AppCtx) {
    ui.heading("Simulation");
    ui.add_space(6.0);

    ui.group(|ui| {
        ui.label(RichText::new("Built-in multirotor simulator").strong());
        ui.label("A lightweight ArduCopter-like vehicle that runs inside the app. Works on every platform, including Android and iOS.");
        ui.label(format!("UDP port {}", app.settings.sim_port));
        ui.horizontal(|ui| {
            if app.sim_running {
                if theme::button(ui, "Stop simulator").clicked() {
                    app.actions.push(AppAction::StopSim);
                }
                if theme::button(ui, "Demo flight").on_hover_text("Upload a square mission, arm, take off and fly it in Auto").clicked() {
                    app.actions.push(AppAction::DemoFlight);
                }
                ui.label(RichText::new("running").color(theme::GREEN));
            } else if theme::button(ui, "Start simulator & connect").clicked() {
                app.actions.push(AppAction::StartSim);
            }
        });
    });

    ui.add_space(10.0);
    ui.group(|ui| {
        ui.label(RichText::new("ArduPilot SITL").strong());
        ui.label("Run SITL yourself (sim_vehicle.py or a prebuilt binary), then connect:");
        ui.horizontal(|ui| {
            if theme::button(ui, "Connect TCP 127.0.0.1:5760").clicked() {
                app.actions.push(AppAction::Connect(ConnectionSpec::Tcp {
                    host: "127.0.0.1".into(),
                    port: 5760,
                }));
            }
            if theme::button(ui, "Listen UDP 14550").clicked() {
                app.actions
                    .push(AppAction::Connect(ConnectionSpec::UdpListen {
                        port: 14550,
                    }));
            }
        });
        ui.hyperlink_to(
            "SITL setup guide",
            "https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html",
        );
    });
}
