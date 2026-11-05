//! Port of `GCSViews/InitialSetup` (SETUP): BackstageView with Install Firmware,
//! ">> Mandatory Hardware" and ">> Optional Hardware" pages.

use egui::{Color32, RichText, Vec2};
use mp_core::link::{Link, LinkCommand};
use mp_core::param_meta;

use crate::app::AppCtx;
use crate::theme::{self, BsvItem};
use crate::views::config;
use crate::views::flight_data::pwm_bar;

/// (label, parent) — order and labels from InitialSetup.cs / InitialSetup.resx.
const PAGES: &[(&str, Option<usize>)] = &[
    ("Install Firmware", None),       // 0
    ("Mandatory Hardware", None),     // 1
    ("Frame Type", Some(1)),          // 2
    ("Accel Calibration", Some(1)),   // 3
    ("Compass", Some(1)),             // 4
    ("Radio Calibration", Some(1)),   // 5
    ("Servo Output", Some(1)),        // 6
    ("Serial Ports", Some(1)),        // 7
    ("ESC Calibration", Some(1)),     // 8
    ("Flight Modes", Some(1)),        // 9
    ("FailSafe", Some(1)),            // 10
    ("HW ID", Some(1)),               // 11
    ("Initial Tune Parameter", Some(1)), // 12
    ("Optional Hardware", None),      // 13
    ("Sik Radio", Some(13)),          // 14
    ("Battery Monitor", Some(13)),    // 15
    ("Battery Monitor 2", Some(13)),  // 16
    ("DroneCAN/UAVCAN", Some(13)),    // 17
    ("Joystick", Some(13)),           // 18
    ("Compass/Motor Calib", Some(13)), // 19
    ("Range Finder", Some(13)),       // 20
    ("Airspeed", Some(13)),           // 21
    ("Optical Flow", Some(13)),       // 22
    ("OSD", Some(13)),                // 23
    ("Camera Gimbal", Some(13)),      // 24
    ("Motor Test", Some(13)),         // 25
    ("Bluetooth Setup", Some(13)),    // 26
    ("Parachute", Some(13)),          // 27
    ("ESP8266 Setup", Some(13)),      // 28
];

#[derive(Default)]
pub struct SetupView {
    page: usize,
    radio_cal: Option<RadioCal>,
    motor_pct: f32,
}

struct RadioCal {
    min: [u16; 16],
    max: [u16; 16],
}

impl SetupView {
    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let items: Vec<BsvItem> = PAGES.iter().map(|(l, p)| BsvItem { label: l.to_string(), parent: *p }).collect();
        egui::Panel::left("setup_bsv")
            .resizable(false)
            .exact_size(theme::BSV_WIDTH)
            .frame(egui::Frame::new())
            .show(ui, |ui| theme::backstage(ui, &items, &mut self.page));
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::BG).inner_margin(6)).show(ui, |ui| {
            if self.page == 0 {
                install_firmware(ui);
                return;
            }
            if self.page == 18 {
                theme::label(ui, "Joystick: not available in this port yet");
                return;
            }
            let Some(link) = app.link.filter(|_| app.connected()) else {
                theme::label(ui, "Please Connect First");
                return;
            };
            match self.page {
                2 => frame_type(ui, link),
                3 => accel(ui, link),
                4 => compass(ui, link),
                5 => self.radio(ui, link),
                6 => servo_output(ui, app),
                7 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("SERIAL")),
                8 => esc_cal(ui, app),
                9 => config::flight_modes(ui, app),
                10 => failsafe(ui, app),
                11 => hw_id(ui, link),
                12 => config::friendly(ui, app, &Default::default(), |n, _| {
                    matches!(n, "MOT_THST_EXPO" | "MOT_SPIN_ARM" | "MOT_SPIN_MIN" | "MOT_BAT_VOLT_MAX" | "MOT_BAT_VOLT_MIN" | "INS_GYRO_FILTER" | "INS_ACCEL_FILTER" | "ATC_ACCEL_P_MAX" | "ATC_ACCEL_R_MAX" | "ATC_ACCEL_Y_MAX" | "ATC_THR_MIX_MAN")
                }),
                14 => {
                    theme::label(ui, "Sik Radio: not available in this port yet");
                }
                15 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("BATT_")),
                16 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("BATT2_")),
                17 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("CAN_")),
                19 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("COMPASS_MOT")),
                20 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("RNGFND")),
                21 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("ARSPD")),
                22 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("FLOW")),
                23 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("OSD")),
                24 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("MNT")),
                25 => self.motor_test(ui, link),
                26 => {
                    theme::label(ui, "Bluetooth Setup: not available in this port yet");
                }
                27 => config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("CHUTE")),
                28 => {
                    theme::label(ui, "ESP8266 Setup: not available in this port yet");
                }
                _ => {}
            }
        });
    }

    fn radio(&mut self, ui: &mut egui::Ui, link: &Link) {
        let cs = link.shared.read().cs.clone();
        let names = ["Roll", "Pitch", "Throttle", "Yaw", "Radio 5", "Radio 6", "Radio 7", "Radio 8"];
        if let Some(cal) = self.radio_cal.as_mut() {
            for i in 0..16 {
                let v = cs.ch_in[i];
                if v > 800 && v < 2200 {
                    cal.min[i] = cal.min[i].min(v);
                    cal.max[i] = cal.max[i].max(v);
                }
            }
        }
        for (i, n) in names.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.add_sized(Vec2::new(70.0, 18.0), egui::Label::new(RichText::new(*n).color(Color32::WHITE)));
                pwm_bar(ui, &(i + 1).to_string(), cs.ch_in[i]);
                if let Some(c) = &self.radio_cal {
                    theme::label(ui, format!("min {} max {}", c.min[i], c.max[i]));
                }
            });
        }
        ui.add_space(8.0);
        if self.radio_cal.is_none() {
            if theme::button(ui, "Calibrate Radio").clicked() {
                self.radio_cal = Some(RadioCal { min: [u16::MAX; 16], max: [0; 16] });
            }
        } else if theme::button(ui, "Click when Done").clicked() {
            let cal = self.radio_cal.take().unwrap();
            for i in 0..16 {
                if cal.max[i] > cal.min[i] && cal.max[i] - cal.min[i] > 200 {
                    let n = i + 1;
                    let trim = if i == 2 { cal.min[i] } else { cs.ch_in[i].clamp(cal.min[i], cal.max[i]) };
                    link.send(LinkCommand::SetParam { name: format!("RC{n}_MIN"), value: cal.min[i] as f32 });
                    link.send(LinkCommand::SetParam { name: format!("RC{n}_MAX"), value: cal.max[i] as f32 });
                    link.send(LinkCommand::SetParam { name: format!("RC{n}_TRIM"), value: trim as f32 });
                }
            }
        }
    }

    fn motor_test(&mut self, ui: &mut egui::Ui, link: &Link) {
        ui.horizontal(|ui| {
            theme::label(ui, "Throttle %");
            ui.add(egui::DragValue::new(&mut self.motor_pct).range(0.0..=100.0));
        });
        ui.horizontal_wrapped(|ui| {
            for (i, m) in ["A", "B", "C", "D", "E", "F", "G", "H"].iter().enumerate() {
                if theme::button(ui, &format!("Test motor {m}")).clicked() {
                    link.send(LinkCommand::CommandLong { command: 209, params: [(i + 1) as f32, 0.0, self.motor_pct, 2.0, 0.0, 0.0, 0.0] });
                }
            }
        });
        if theme::button(ui, "Test all motors").clicked() {
            link.send(LinkCommand::CommandLong { command: 209, params: [1.0, 0.0, self.motor_pct, 2.0, 8.0, 0.0, 0.0] });
        }
        if theme::button(ui, "Stop all motors").clicked() {
            link.send(LinkCommand::CommandLong { command: 209, params: [1.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0] });
        }
    }
}

fn install_firmware(ui: &mut egui::Ui) {
    theme::label(ui, "Firmware upload (px4 bootloader protocol and the firmware.ardupilot.org manifest) is not available in this port yet.");
    ui.hyperlink_to("firmware.ardupilot.org", "https://firmware.ardupilot.org/");
}

fn param_row(ui: &mut egui::Ui, link: &Link, name: &str) {
    let (val, fw) = {
        let s = link.shared.read();
        (s.params.get(name).map(|p| p.value), s.cs.firmware)
    };
    ui.horizontal(|ui| {
        ui.add_sized(Vec2::new(150.0, 18.0), egui::Label::new(RichText::new(name).color(Color32::WHITE)));
        match val {
            Some(v) => config::param_control(ui, link, fw, name, v, 220.0),
            None => {
                ui.label(RichText::new("not available").color(Color32::GRAY));
            }
        }
        if let Some(m) = param_meta::get(fw, name) {
            ui.label(RichText::new(&m.display_name).color(Color32::GRAY));
        }
    });
}

fn frame_type(ui: &mut egui::Ui, link: &Link) {
    param_row(ui, link, "FRAME_CLASS");
    param_row(ui, link, "FRAME_TYPE");
    theme::label(ui, "A reboot is required after changing the frame class.");
    if theme::button(ui, "Reboot").clicked() {
        link.send(LinkCommand::CommandLong { command: 246, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
    }
}

fn accel(ui: &mut egui::Ui, link: &Link) {
    ui.label(RichText::new("Accelerometer Calibration").strong().color(Color32::WHITE));
    theme::label(ui, "Place vehicle level and press Calibrate Accel, then follow the prompts.");
    ui.horizontal(|ui| {
        if theme::button(ui, "Calibrate Accel").clicked() {
            link.send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0] });
        }
        for (i, n) in ["Level", "Left", "Right", "Nose Down", "Nose Up", "Back"].iter().enumerate() {
            if theme::button(ui, n).clicked() {
                link.send(LinkCommand::CommandLong { command: 42429, params: [(i + 1) as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
            }
        }
    });
    ui.add_space(10.0);
    ui.label(RichText::new("Level Horizon").strong().color(Color32::WHITE));
    if theme::button(ui, "Calibrate Level").clicked() {
        link.send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0] });
    }
    ui.add_space(10.0);
    ui.label(RichText::new("Simple Accel Calibration").strong().color(Color32::WHITE));
    if theme::button(ui, "Simple Accel Cal").clicked() {
        link.send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0] });
    }
    messages(ui, link);
}

fn compass(ui: &mut egui::Ui, link: &Link) {
    param_row(ui, link, "COMPASS_USE");
    param_row(ui, link, "COMPASS_ORIENT");
    ui.add_space(8.0);
    ui.label(RichText::new("Onboard Mag Calibration").strong().color(Color32::WHITE));
    ui.horizontal(|ui| {
        if theme::button(ui, "Start").clicked() {
            link.send(LinkCommand::CommandLong { command: 42424, params: [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0] });
        }
        if theme::button(ui, "Accept").clicked() {
            link.send(LinkCommand::CommandLong { command: 42425, params: [0.0; 7] });
        }
        if theme::button(ui, "Cancel").clicked() {
            link.send(LinkCommand::CommandLong { command: 42426, params: [0.0; 7] });
        }
    });
    messages(ui, link);
}

fn servo_output(ui: &mut egui::Ui, app: &mut AppCtx) {
    let Some(link) = app.link else { return };
    let cs = link.shared.read().cs.clone();
    for i in 0..16 {
        pwm_bar(ui, &(i + 1).to_string(), cs.ch_out[i]);
    }
    ui.add_space(6.0);
    config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("SERVO") && n.ends_with("_FUNCTION"));
}

fn esc_cal(ui: &mut egui::Ui, app: &mut AppCtx) {
    theme::label(ui, "ESC Calibration: set ESC_CALIBRATION to 3, then follow the ESC calibration steps after a reboot.");
    config::friendly(ui, app, &Default::default(), |n, _| n == "ESC_CALIBRATION" || n == "MOT_PWM_TYPE" || n == "MOT_PWM_MIN" || n == "MOT_PWM_MAX");
}

fn failsafe(ui: &mut egui::Ui, app: &mut AppCtx) {
    if let Some(link) = app.link {
        let cs = link.shared.read().cs.clone();
        theme::label(ui, format!("Battery {:.2} V   Throttle PWM {}   Mode {}", cs.battery_voltage, cs.ch_in[2], cs.mode));
    }
    config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("FS_") || n.starts_with("BATT_FS") || n.starts_with("THR_F"));
}

fn hw_id(ui: &mut egui::Ui, link: &Link) {
    let names: Vec<String> = link.shared.read().params.keys().filter(|k| k.ends_with("_DEV_ID") || k.ends_with("_ID")).cloned().collect();
    for n in names {
        let v = link.shared.read().params.get(&n).map(|p| p.value).unwrap_or(0.0);
        theme::label(ui, format!("{n}: {}", v as i64));
    }
}

fn messages(ui: &mut egui::Ui, link: &Link) {
    ui.add_space(8.0);
    egui::ScrollArea::vertical().max_height(220.0).stick_to_bottom(true).show(ui, |ui| {
        let s = link.shared.read();
        for m in s.messages.iter().rev().take(30).rev() {
            theme::label(ui, format!("{} {}", m.time, m.text));
        }
    });
}
