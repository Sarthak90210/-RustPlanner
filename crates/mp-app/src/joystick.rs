//! Joystick support (`Joystick/JoystickSetup.cs` + `ExtLibs/ArduPilot/Joystick`): map game
//! controller axes to RC channels with expo / reverse and stream RC_CHANNELS_OVERRIDE while
//! enabled. Desktop only (gilrs); mobile builds show the page without devices.

use std::time::{Duration, Instant};

use egui::{Color32, Rect, Sense, Vec2};
use mp_core::link::{Link, LinkCommand};
use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, RC_CHANNELS_OVERRIDE_DATA};

use crate::theme;

/// `joystickaxis` names, in the original order.
pub const AXES: &[&str] = &[
    "None", "Pass", "ARx", "ARy", "ARz", "AX", "AY", "AZ", "FRx", "FRy", "FRz", "FX", "FY", "FZ", "Rx", "Ry", "Rz", "VRx", "VRy", "VRz", "VX", "VY", "VZ", "X", "Y", "Z", "Slider1", "Slider2", "Hatud1", "Hatlr2", "Custom1", "Custom2",
];

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct ChannelCfg {
    pub axis: String,
    pub expo: i32,
    pub reverse: bool,
}

#[derive(serde::Serialize, serde::Deserialize, Default)]
struct Saved {
    channels: Vec<ChannelCfg>,
    elevons: bool,
    manual_control: bool,
}

pub struct Joystick {
    #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
    gilrs: Option<gilrs::Gilrs>,
    devices: Vec<(usize, String)>,
    pub device: Option<usize>,
    pub enabled: bool,
    pub channels: Vec<ChannelCfg>,
    pub elevons: bool,
    pub manual_control: bool,
    /// last output per channel (0 = released / no override)
    pub output: [u16; 16],
    last_send: Instant,
    detect: Option<(usize, Instant, Vec<f32>)>,
    pub window_open: bool,
}

fn default_channels() -> Vec<ChannelCfg> {
    // JoystickBase defaults: roll / pitch on the right stick, throttle / yaw on the left
    let map = ["Rx", "Ry", "Y", "X"];
    (0..16).map(|i| ChannelCfg { axis: map.get(i).unwrap_or(&"None").to_string(), expo: 0, reverse: i == 2 || i == 1 }).collect()
}

impl Default for Joystick {
    fn default() -> Self {
        let mut j = Self {
            #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
            gilrs: gilrs::Gilrs::new().ok(),
            devices: Vec::new(),
            device: None,
            enabled: false,
            channels: default_channels(),
            elevons: false,
            manual_control: false,
            output: [0; 16],
            last_send: Instant::now(),
            detect: None,
            window_open: false,
        };
        j.load();
        j
    }
}

fn config_path() -> std::path::PathBuf {
    crate::platform::data_dir().join("joystick.json")
}

impl Joystick {
    fn load(&mut self) {
        if let Some(s) = std::fs::read_to_string(config_path()).ok().and_then(|t| serde_json::from_str::<Saved>(&t).ok()) {
            if s.channels.len() == 16 {
                self.channels = s.channels;
            }
            self.elevons = s.elevons;
            self.manual_control = s.manual_control;
        }
    }

    fn save_to(&self, path: &std::path::Path) -> std::io::Result<()> {
        let s = Saved { channels: self.channels.clone(), elevons: self.elevons, manual_control: self.manual_control };
        std::fs::write(path, serde_json::to_string_pretty(&s).unwrap_or_default())
    }

    /// Raw axis value in -1..1 for a `joystickaxis` name.
    #[allow(unused_variables)]
    fn axis_value(&self, name: &str) -> Option<f32> {
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        {
            use gilrs::Axis;
            let g = self.gilrs.as_ref()?;
            let id = self.device?;
            let pad = g.gamepads().find(|(gid, _)| usize::from(*gid) == id)?.1;
            let ax = match name {
                "X" => Axis::LeftStickX,
                "Y" => Axis::LeftStickY,
                "Z" => Axis::LeftZ,
                "Rx" => Axis::RightStickX,
                "Ry" => Axis::RightStickY,
                "Rz" => Axis::RightZ,
                "Hatlr2" => Axis::DPadX,
                "Hatud1" => Axis::DPadY,
                _ => return None,
            };
            return Some(pad.value(ax));
        }
        #[allow(unreachable_code)]
        None
    }

    fn all_axes(&self) -> Vec<f32> {
        ["X", "Y", "Z", "Rx", "Ry", "Rz", "Hatlr2", "Hatud1"].iter().map(|a| self.axis_value(a).unwrap_or(0.0)).collect()
    }

    /// JoystickBase.getValueForChannel: axis -> expo -> reverse -> PWM 1000..2000 (0 = none).
    fn channel_value(&self, i: usize) -> u16 {
        let c = &self.channels[i];
        if c.axis == "None" {
            return 0;
        }
        let Some(mut v) = self.axis_value(&c.axis) else { return 0 };
        // gilrs reports up as +1; RC wants low PWM at the bottom
        if c.expo != 0 {
            let e = c.expo as f32 / 100.0;
            v = v * (1.0 - e) + v.powi(3) * e;
        }
        if c.reverse {
            v = -v;
        }
        (1500.0 + v.clamp(-1.0, 1.0) * 500.0) as u16
    }

    /// Poll the controller, refresh the device list and stream RC overrides while enabled.
    pub fn tick(&mut self, link: Option<&Link>) {
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        if let Some(g) = self.gilrs.as_mut() {
            while g.next_event().is_some() {}
            self.devices = g.gamepads().map(|(id, p)| (usize::from(id), p.name().to_string())).collect();
            if self.device.is_none() {
                self.device = self.devices.first().map(|d| d.0);
            }
        }
        for i in 0..16 {
            self.output[i] = self.channel_value(i);
        }
        if self.elevons {
            // mix ch1/ch2 into elevons
            let (a, b) = (self.output[0] as i32, self.output[1] as i32);
            if a != 0 && b != 0 {
                self.output[0] = ((a + b) / 2).clamp(1000, 2000) as u16;
                self.output[1] = ((a - b) / 2 + 1500).clamp(1000, 2000) as u16;
            }
        }
        if !self.enabled || self.last_send.elapsed() < Duration::from_millis(50) {
            return;
        }
        self.last_send = Instant::now();
        let Some(l) = link else { return };
        let (sys, comp) = {
            let s = l.shared.read();
            (s.cs.sysid, s.cs.compid)
        };
        let o = self.output;
        l.send(LinkCommand::Send(Box::new(MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
            chan1_raw: o[0],
            chan2_raw: o[1],
            chan3_raw: o[2],
            chan4_raw: o[3],
            chan5_raw: o[4],
            chan6_raw: o[5],
            chan7_raw: o[6],
            chan8_raw: o[7],
            target_system: sys,
            target_component: comp,
            chan9_raw: o[8],
            chan10_raw: o[9],
            chan11_raw: o[10],
            chan12_raw: o[11],
            chan13_raw: o[12],
            chan14_raw: o[13],
            chan15_raw: o[14],
            chan16_raw: o[15],
            ..Default::default()
        }))));
    }

    /// JoystickSetup control (702 wide): device row, axis grid and options.
    pub fn setup_ui(&mut self, ui: &mut egui::Ui, firmware: &str, toasts: &mut crate::app::Toasts) {
        let o = ui.available_rect_before_wrap().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let p = ui.painter().clone();
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let txt = |x: f32, y: f32, s: &str| {
            p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, s, f.clone(), Color32::WHITE);
        };
        let btn = |ui: &mut egui::Ui, rr: Rect, t: &str| ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, t, rr.size())).clicked();
        txt(19.0, 15.0, "Joystick");
        let dev_name = self.device.and_then(|d| self.devices.iter().find(|x| x.0 == d)).map(|x| x.1.clone()).unwrap_or_default();
        ui.put(r(72.0, 12.0, 202.0, 21.0), |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_joysticks")
                .width(194.0)
                .selected_text(dev_name)
                .show_ui(ui, |ui| {
                    for (id, n) in self.devices.clone() {
                        ui.selectable_value(&mut self.device, Some(id), n);
                    }
                })
                .response
        });
        if btn(ui, r(280.0, 12.0, 58.0, 23.0), if self.enabled { "Disable" } else { "Enable" }) {
            if !self.enabled && self.device.is_none() {
                toasts.error("No joystick found");
            } else {
                self.enabled = !self.enabled;
                if !self.enabled {
                    self.output = [0; 16];
                }
            }
        }
        if btn(ui, r(344.0, 12.0, 44.0, 23.0), "Save") {
            if let Err(e) = self.save_to(&config_path()) {
                toasts.error(e.to_string());
            }
        }
        if btn(ui, r(394.0, 12.0, 49.0, 23.0), "Export") {
            #[cfg(not(any(target_os = "android", target_os = "ios")))]
            if let Some(p) = rfd::FileDialog::new().add_filter("Joystick config", &["json"]).set_file_name("joystick.json").save_file() {
                if let Err(e) = self.save_to(&p) {
                    toasts.error(e.to_string());
                }
            }
        }
        if btn(ui, r(449.0, 12.0, 49.0, 23.0), "Import") {
            #[cfg(not(any(target_os = "android", target_os = "ios")))]
            if let Some(p) = rfd::FileDialog::new().add_filter("Joystick config", &["json"]).pick_file() {
                match std::fs::read_to_string(&p).ok().and_then(|t| serde_json::from_str::<Saved>(&t).ok()) {
                    Some(s) if s.channels.len() == 16 => {
                        self.channels = s.channels;
                        self.elevons = s.elevons;
                        self.manual_control = s.manual_control;
                    }
                    _ => toasts.error("Invalid joystick config"),
                }
            }
        }
        txt(500.0, 17.0, &format!("Loaded Config for {firmware}"));
        txt(69.0, 47.0, "Controller Axis");
        txt(197.0, 47.0, "Output");
        txt(307.0, 47.0, "Expo");
        txt(411.0, 47.0, "Reverse");
        ui.put(r(464.0, 47.0, 117.0, 17.0), egui::Checkbox::new(&mut self.manual_control, "Manual Control"));
        ui.put(r(434.0, 81.0, 84.0, 17.0), egui::Checkbox::new(&mut self.elevons, "Elevons"));
        // detection: the axis that moves most within 3 s
        if let Some((ch, start, base)) = &self.detect {
            let now = self.all_axes();
            let names = ["X", "Y", "Z", "Rx", "Ry", "Rz", "Hatlr2", "Hatud1"];
            if let Some((k, d)) = now.iter().zip(base.iter()).map(|(a, b)| (a - b).abs()).enumerate().max_by(|a, b| a.1.total_cmp(&b.1)) {
                if d > 0.5 {
                    self.channels[*ch].axis = names[k].to_string();
                    self.detect = None;
                } else if start.elapsed() > Duration::from_secs(3) {
                    self.detect = None;
                }
            }
            ui.ctx().request_repaint();
        }
        let mut y = 60.0;
        for i in 0..16 {
            let row = |x: f32, yy: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y + yy), Vec2::new(w, h));
            p.text(o + Vec2::new(2.0, y + 8.0), egui::Align2::LEFT_TOP, format!("RC {}", i + 1), f.clone(), Color32::WHITE);
            let cur = self.channels[i].axis.clone();
            ui.put(row(65.0, 3.0, 70.0, 21.0), |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt(("joy_axis", i))
                    .width(62.0)
                    .selected_text(cur)
                    .show_ui(ui, |ui| {
                        for a in AXES {
                            ui.selectable_value(&mut self.channels[i].axis, a.to_string(), *a);
                        }
                    })
                    .response
            });
            if btn(ui, row(141.0, 3.0, 45.0, 23.0), "Auto Detect") {
                self.detect = Some((i, Instant::now(), self.all_axes()));
            }
            let pr = row(192.0, 2.0, 100.0, 23.0);
            let v = self.output[i];
            ui.put(pr, |ui: &mut egui::Ui| {
                theme::progress(ui, if v == 0 { 0.0 } else { (v as f32 - 1000.0) / 1000.0 }, pr.size(), &v.to_string());
                ui.allocate_response(Vec2::ZERO, Sense::hover())
            });
            let mut e = self.channels[i].expo.to_string();
            if ui.put(row(300.0, 4.0, 100.0, 18.0), egui::TextEdit::singleline(&mut e)).changed() {
                self.channels[i].expo = e.trim().parse().unwrap_or(0).clamp(-100, 100);
            }
            ui.put(row(406.0, 6.0, 15.0, 14.0), egui::Checkbox::without_text(&mut self.channels[i].reverse));
            y += 28.0;
        }
        ui.allocate_rect(r(0.0, 0.0, 702.0, y + 6.0), Sense::hover());
    }

    /// Stand-alone JoystickSetup window (Actions tab / Planner "Joystick" buttons).
    pub fn window(&mut self, ctx: &egui::Context, firmware: &str, toasts: &mut crate::app::Toasts) {
        if !self.window_open {
            return;
        }
        let mut open = true;
        egui::Window::new("Joystick").open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            egui::ScrollArea::vertical().max_height(560.0).show(ui, |ui| self.setup_ui(ui, firmware, toasts));
        });
        if !open {
            self.window_open = false;
        }
    }
}
