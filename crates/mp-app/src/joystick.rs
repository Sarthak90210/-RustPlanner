//! Joystick support (`Joystick/JoystickSetup.cs` + `ExtLibs/ArduPilot/Joystick`): map game
//! controller axes to RC channels with expo / reverse and stream RC_CHANNELS_OVERRIDE while
//! enabled. Desktop only (gilrs); mobile builds show the page without devices.

use std::time::{Duration, Instant};

use egui::{Color32, Rect, Sense, Vec2};
use mp_core::link::{Link, LinkCommand};
use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, RC_CHANNELS_OVERRIDE_DATA};

use crate::theme;

mod buttons;
use buttons::{ButtonCfg, Function};

/// `joystickaxis` names, in the original order.
pub const AXES: &[&str] = &[
    "None", "Pass", "ARx", "ARy", "ARz", "AX", "AY", "AZ", "FRx", "FRy", "FRz", "FX", "FY", "FZ",
    "Rx", "Ry", "Rz", "VRx", "VRy", "VRz", "VX", "VY", "VZ", "X", "Y", "Z", "Slider1", "Slider2",
    "Hatud1", "Hatlr2", "Custom1", "Custom2",
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
    #[serde(default = "buttons::defaults")]
    buttons: [ButtonCfg; 16],
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
    buttons: [ButtonCfg; 16],
    button_runtime: buttons::Runtime,
    button_detect: Option<(usize, Instant)>,
    button_settings: Option<usize>,
    was_active: bool,
    was_manual: bool,
    last_device: Option<usize>,
    active_buttons: [ButtonCfg; 16],
    #[cfg(test)]
    test_buttons: Option<Vec<bool>>,
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
    (0..16)
        .map(|i| ChannelCfg {
            axis: map.get(i).unwrap_or(&"None").to_string(),
            expo: 0,
            reverse: i == 2 || i == 1,
        })
        .collect()
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
            buttons: buttons::defaults(),
            button_runtime: Default::default(),
            button_detect: None,
            button_settings: None,
            was_active: false,
            was_manual: false,
            last_device: None,
            active_buttons: buttons::defaults(),
            #[cfg(test)]
            test_buttons: None,
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
    /// Release RC before replacing a link or selecting another vehicle.
    pub fn stop(&mut self, link: Option<&Link>) {
        if let Some(l) = link.filter(|_| self.was_active) {
            l.send(LinkCommand::CancelCommands);
            for command in self.button_runtime.release_actions(&self.active_buttons) {
                l.send(command);
            }
        }
        if self.was_active && !self.was_manual {
            if let Some(l) = link {
                let (sys, comp) = {
                    let s = l.shared.read();
                    (s.cs.sysid, s.cs.compid)
                };
                l.send(LinkCommand::Send(Box::new(control_message(
                    [0; 16], sys, comp, false,
                ))));
            }
        }
        self.enabled = false;
        self.was_active = false;
        self.button_runtime.sync(&self.buttons, &[]);
    }

    fn load(&mut self) {
        if let Some(s) = std::fs::read_to_string(config_path())
            .ok()
            .and_then(|t| serde_json::from_str::<Saved>(&t).ok())
        {
            if s.channels.len() == 16 {
                self.channels = s.channels;
            }
            self.buttons = s.buttons;
            self.elevons = s.elevons;
            self.manual_control = s.manual_control;
        }
    }

    fn save_to(&self, path: &std::path::Path) -> std::io::Result<()> {
        let s = Saved {
            channels: self.channels.clone(),
            buttons: self.buttons.clone(),
            elevons: self.elevons,
            manual_control: self.manual_control,
        };
        std::fs::write(path, serde_json::to_string_pretty(&s).unwrap_or_default())
    }

    /// Raw axis value in -1..1 for a `joystickaxis` name.
    #[allow(unused_variables)]
    fn axis_value(&self, name: &str) -> Option<f32> {
        match name {
            "Pass" => return Some(0.0),
            "Custom1" | "Custom2" => {
                return self.button_runtime.custom[usize::from(name == "Custom2")]
                    .map(|p| (p as f32 - 1500.0) / 500.0)
            }
            _ => {}
        }
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
        ["X", "Y", "Z", "Rx", "Ry", "Rz", "Hatlr2", "Hatud1"]
            .iter()
            .map(|a| self.axis_value(a).unwrap_or(0.0))
            .collect()
    }

    /// JoystickBase.getValueForChannel: axis -> expo -> reverse -> PWM 1000..2000 (0 = none).
    fn channel_value(&self, i: usize) -> u16 {
        let c = &self.channels[i];
        if c.axis == "None" {
            return 0;
        }
        let Some(mut v) = self.axis_value(&c.axis) else {
            return 0;
        };
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

    fn button_states(&self) -> Vec<bool> {
        #[cfg(test)]
        if let Some(states) = &self.test_buttons {
            return states.clone();
        }
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        {
            use gilrs::Button::*;
            let names = [
                South,
                East,
                North,
                West,
                C,
                Z,
                LeftTrigger,
                LeftTrigger2,
                RightTrigger,
                RightTrigger2,
                Select,
                Start,
                Mode,
                LeftThumb,
                RightThumb,
                DPadUp,
                DPadDown,
                DPadLeft,
                DPadRight,
            ];
            if let (Some(g), Some(id)) = (&self.gilrs, self.device) {
                if let Some((_, pad)) = g.gamepads().find(|(gid, _)| usize::from(*gid) == id) {
                    return names.iter().map(|&b| pad.is_pressed(b)).collect();
                }
            }
        }
        vec![false; 19]
    }

    /// Poll button edges independently of the 20 Hz axis stream.
    pub fn tick(&mut self, link: Option<&Link>, readonly: bool) {
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        if let Some(g) = self.gilrs.as_mut() {
            while g.next_event().is_some() {}
            self.devices = g
                .gamepads()
                .map(|(id, p)| (usize::from(id), p.name().to_string()))
                .collect();
            if self
                .device
                .is_some_and(|id| !self.devices.iter().any(|d| d.0 == id))
            {
                self.enabled = false;
                self.device = None;
            }
            if self.device.is_none() {
                self.device = self.devices.first().map(|d| d.0);
            }
        }
        let states = self.button_states();
        if let Some((row, start)) = self.button_detect {
            if let Some(button) = states.iter().position(|s| *s) {
                self.buttons[row].buttonno = button as i16;
                self.button_detect = None;
            } else if start.elapsed() > Duration::from_secs(5) {
                self.button_detect = None;
            }
        }
        // A different controller or imported/remapped buttons must be enabled again.
        if self.was_active
            && (self.device != self.last_device || self.buttons != self.active_buttons)
        {
            self.enabled = false;
        }
        let active = self.enabled
            && self.device.is_some()
            && !readonly
            && link.is_some_and(|l| l.status() == mp_core::link::LinkStatus::Connected);
        if self.was_active && !active {
            if let Some(l) = link {
                l.send(LinkCommand::CancelCommands);
                for command in self.button_runtime.release_actions(&self.active_buttons) {
                    l.send(command);
                }
            }
        }
        if active {
            let l = link.unwrap();
            let (firmware, pan, mode) = {
                let s = l.shared.read();
                (
                    s.cs.firmware,
                    s.params.get("MNT_STAB_PAN").map(|p| p.value),
                    s.cs.custom_mode,
                )
            };
            if !self.was_active {
                self.button_runtime.sync(&self.buttons, &states);
            }
            for command in self
                .button_runtime
                .poll(&self.buttons, &states, firmware, pan)
            {
                l.send(command);
            }
            for command in self.button_runtime.mode_commands(mode, Instant::now()) {
                l.send(command);
            }
            self.active_buttons = self.buttons.clone();
        } else {
            self.button_runtime.sync(&self.buttons, &states);
        }
        for i in 0..16 {
            self.output[i] = self.channel_value(i);
        }
        if self.elevons {
            let (a, b) = (self.output[0] as i32, self.output[1] as i32);
            if a != 0 && b != 0 {
                self.output[0] = ((a + b) / 2).clamp(1000, 2000) as u16;
                self.output[1] = ((a - b) / 2 + 1500).clamp(1000, 2000) as u16;
            }
        }
        if let Some(l) = link {
            let (sys, comp) = {
                let s = l.shared.read();
                (s.cs.sysid, s.cs.compid)
            };
            if self.was_active && !self.was_manual && (!active || self.manual_control) {
                l.send(LinkCommand::Send(Box::new(control_message(
                    [0; 16], sys, comp, false,
                ))));
            }
            if active && self.last_send.elapsed() >= Duration::from_millis(50) {
                self.last_send = Instant::now();
                l.send(LinkCommand::Send(Box::new(control_message(
                    self.output,
                    sys,
                    comp,
                    self.manual_control,
                ))));
            }
        }
        self.was_active = active;
        self.was_manual = self.manual_control;
        self.last_device = self.device;
    }

    pub fn setup_ui(&mut self, ui: &mut egui::Ui, firmware: &str, toasts: &mut crate::app::Toasts) {
        egui::ScrollArea::both()
            .auto_shrink([false, false])
            .show(ui, |ui| self.setup_contents(ui, firmware, toasts));
    }

    /// JoystickSetup: axes at x=0, button rows at axis.Right + 100, as timer1_Tick.
    fn setup_contents(
        &mut self,
        ui: &mut egui::Ui,
        firmware: &str,
        toasts: &mut crate::app::Toasts,
    ) {
        let o = ui.available_rect_before_wrap().min;
        let r = |x: f32, y: f32, w: f32, h: f32| {
            Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h))
        };
        let p = ui.painter().clone();
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let txt = |x: f32, y: f32, s: &str| {
            p.text(
                o + Vec2::new(x, y),
                egui::Align2::LEFT_TOP,
                s,
                f.clone(),
                Color32::WHITE,
            );
        };
        let btn = |ui: &mut egui::Ui, rr: Rect, t: &str| {
            ui.put(rr, |ui: &mut egui::Ui| {
                theme::button_sized(ui, t, rr.size())
            })
            .clicked()
        };
        txt(19.0, 15.0, "Joystick");
        let dev_name = self
            .device
            .and_then(|d| self.devices.iter().find(|x| x.0 == d))
            .map(|x| x.1.clone())
            .unwrap_or_default();
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
        if btn(
            ui,
            r(280.0, 12.0, 58.0, 23.0),
            if self.enabled { "Disable" } else { "Enable" },
        ) {
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
            if let Some(p) = crate::platform::save_file_as(
                "Export Joystick Config",
                "joystick.json",
                "Joystick config",
                &["json"],
            ) {
                if let Err(e) = self.save_to(&p) {
                    toasts.error(e.to_string());
                }
            }
        }
        if btn(ui, r(449.0, 12.0, 49.0, 23.0), "Import") {
            if let Some(p) = crate::platform::pick_file(
                "Import Joystick Config",
                "Joystick config",
                &["json"],
            ) {
                match std::fs::read_to_string(&p)
                    .ok()
                    .and_then(|t| serde_json::from_str::<Saved>(&t).ok())
                {
                    Some(s) if s.channels.len() == 16 => {
                        self.channels = s.channels;
                        self.buttons = s.buttons;
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
        ui.put(
            r(464.0, 47.0, 117.0, 17.0),
            egui::Checkbox::new(&mut self.manual_control, crate::i18n::t("Manual Control")),
        );
        ui.put(
            r(434.0, 81.0, 84.0, 17.0),
            egui::Checkbox::new(&mut self.elevons, crate::i18n::t("Elevons")),
        );
        // detection: the axis that moves most within 3 s
        if let Some((ch, start, base)) = &self.detect {
            let now = self.all_axes();
            let names = ["X", "Y", "Z", "Rx", "Ry", "Rz", "Hatlr2", "Hatud1"];
            if let Some((k, d)) = now
                .iter()
                .zip(base.iter())
                .map(|(a, b)| (a - b).abs())
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(&b.1))
            {
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
            let row = |x: f32, yy: f32, w: f32, h: f32| {
                Rect::from_min_size(o + Vec2::new(x, y + yy), Vec2::new(w, h))
            };
            p.text(
                o + Vec2::new(2.0, y + 8.0),
                egui::Align2::LEFT_TOP,
                format!("RC {}", i + 1),
                f.clone(),
                Color32::WHITE,
            );
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
                theme::progress(
                    ui,
                    if v == 0 {
                        0.0
                    } else {
                        (v as f32 - 1000.0) / 1000.0
                    },
                    pr.size(),
                    &v.to_string(),
                );
                ui.allocate_response(Vec2::ZERO, Sense::hover())
            });
            let mut e = self.channels[i].expo.to_string();
            if ui
                .put(
                    row(300.0, 4.0, 100.0, 18.0),
                    egui::TextEdit::singleline(&mut e),
                )
                .changed()
            {
                self.channels[i].expo = e.trim().parse().unwrap_or(0).clamp(-100, 100);
            }
            ui.put(
                row(406.0, 6.0, 15.0, 14.0),
                egui::Checkbox::without_text(&mut self.channels[i].reverse),
            );
            y += 28.0;
        }
        self.button_rows(ui, o + Vec2::new(527.0, 0.0), 60.0, firmware);
        ui.allocate_rect(r(0.0, 0.0, 1100.0, y + 6.0), Sense::hover());
    }

    fn button_rows(
        &mut self,
        ui: &mut egui::Ui,
        origin: egui::Pos2,
        mut y: f32,
        firmware: &str,
    ) -> f32 {
        let states = self.button_states();
        for i in 0..16 {
            ui.push_id(("joy_button", i), |ui| {
                let r = |x, w| Rect::from_min_size(origin + Vec2::new(x, y), Vec2::new(w, 23.0));
                ui.put(r(0.0, 47.0), egui::Label::new(format!("But {}", i + 1)));
                ui.put(r(47.0, 70.0), |ui: &mut egui::Ui| {
                    egui::ComboBox::from_id_salt("number")
                        .width(62.0)
                        .selected_text(self.buttons[i].buttonno.to_string())
                        .show_ui(ui, |ui| {
                            for n in -1..19 {
                                ui.selectable_value(
                                    &mut self.buttons[i].buttonno,
                                    n,
                                    n.to_string(),
                                );
                            }
                        })
                        .response
                });
                if ui
                    .put(
                        r(120.0, 65.0),
                        egui::Button::new(if self.button_detect.is_some_and(|(row, _)| row == i) {
                            "Press..."
                        } else {
                            "Detect"
                        }),
                    )
                    .clicked()
                {
                    self.button_detect = Some((i, Instant::now()));
                }
                let down = usize::try_from(self.buttons[i].buttonno)
                    .ok()
                    .and_then(|k| states.get(k))
                    .copied()
                    .unwrap_or(false);
                ui.put(
                    r(192.0, 100.0),
                    egui::ProgressBar::new(if down { 1.0 } else { 0.0 }).text(if down {
                        "Pressed"
                    } else {
                        "Released"
                    }),
                );
                ui.put(r(300.0, 170.0), |ui: &mut egui::Ui| {
                    egui::ComboBox::from_id_salt("function")
                        .width(162.0)
                        .selected_text(format!("{:?}", self.buttons[i].function))
                        .show_ui(ui, |ui| {
                            for f in Function::ALL {
                                if ui
                                    .selectable_value(
                                        &mut self.buttons[i].function,
                                        f,
                                        format!("{f:?}"),
                                    )
                                    .changed()
                                    && matches!(f, Function::Button_axis0 | Function::Button_axis1)
                                {
                                    self.buttons[i].params = [1000.0, 2000.0, 0.0, 0.0];
                                }
                            }
                        })
                        .response
                });
                if ui
                    .put(r(478.0, 75.0), egui::Button::new("Settings"))
                    .clicked()
                {
                    self.button_settings = Some(i);
                }
            });
            y += 28.0;
        }
        if let Some(i) = self.button_settings {
            let mut open = true;
            let cfg = &mut self.buttons[i];
            egui::Window::new(format!("Button {}: {:?}", i + 1, cfg.function))
                .id(egui::Id::new("joy_button_settings"))
                .open(&mut open)
                .resizable(false)
                .show(ui.ctx(), |ui| {
                    if cfg.function == Function::ChangeMode {
                        let fw = match firmware {
                            "ArduPlane" => mp_core::firmware::Firmware::ArduPlane,
                            "ArduRover" => mp_core::firmware::Firmware::ArduRover,
                            "ArduSub" => mp_core::firmware::Firmware::ArduSub,
                            "ArduTracker" => mp_core::firmware::Firmware::ArduTracker,
                            "Px4" => mp_core::firmware::Firmware::Px4,
                            _ => mp_core::firmware::Firmware::ArduCopter2,
                        };
                        egui::ComboBox::from_id_salt("button_mode")
                            .selected_text(&cfg.mode)
                            .show_ui(ui, |ui| {
                                for (_, name) in fw.modes() {
                                    ui.selectable_value(&mut cfg.mode, name.clone(), name);
                                }
                            });
                    } else {
                        for (k, label) in cfg.function.fields().iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(*label);
                                ui.add(egui::DragValue::new(&mut cfg.params[k]).speed(1.0));
                            });
                        }
                        if cfg.function.fields().is_empty() {
                            ui.label(crate::i18n::t("No settings to set"));
                        }
                    }
                });
            if !open {
                self.button_settings = None;
            }
        }
        y
    }

    /// Stand-alone JoystickSetup window (Actions tab / Planner "Joystick" buttons).
    pub fn window(&mut self, ctx: &egui::Context, firmware: &str, toasts: &mut crate::app::Toasts) {
        if !self.window_open {
            return;
        }
        let mut open = true;
        egui::Window::new(crate::i18n::t("Joystick"))
            .open(&mut open)
            .collapsible(false)
            .default_size([1100.0, 560.0])
            .show(ctx, |ui| {
                self.setup_ui(ui, firmware, toasts);
            });
        if !open {
            self.window_open = false;
        }
    }
}

/// Same four-axis Manual Control mapping as MainV2; MAVLink target is the system ID.
fn control_message(o: [u16; 16], sys: u8, comp: u8, manual: bool) -> MavMessage {
    if manual {
        let axis = |i| {
            if o[i] == 0 {
                i16::MAX
            } else {
                ((o[i] as i32 - 1500) * 2).clamp(-1000, 1000) as i16
            }
        };
        return MavMessage::MANUAL_CONTROL(
            mp_core::mavlink::dialects::ardupilotmega::MANUAL_CONTROL_DATA {
                target: sys,
                x: axis(0),
                y: axis(1),
                z: axis(2),
                r: axis(3),
                ..Default::default()
            },
        );
    }
    // Extension channels use 65534 to release; zero there means 'ignore'.
    let ext = |i| if o[i] == 0 { 65534 } else { o[i] };
    MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
        target_system: sys,
        target_component: comp,
        chan1_raw: o[0],
        chan2_raw: o[1],
        chan3_raw: o[2],
        chan4_raw: o[3],
        chan5_raw: o[4],
        chan6_raw: o[5],
        chan7_raw: o[6],
        chan8_raw: o[7],
        chan9_raw: ext(8),
        chan10_raw: ext(9),
        chan11_raw: ext(10),
        chan12_raw: ext(11),
        chan13_raw: ext(12),
        chan14_raw: ext(13),
        chan15_raw: ext(14),
        chan16_raw: ext(15),
        ..Default::default()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manual_control_uses_normalized_axes_and_vehicle_system_id() {
        let mut axes = [0; 16];
        axes[..4].copy_from_slice(&[1000, 1500, 2000, 0]);
        let MavMessage::MANUAL_CONTROL(m) = control_message(axes, 42, 7, true) else {
            panic!("Wrong protocol")
        };
        assert_eq!(
            (m.target, m.x, m.y, m.z, m.r),
            (42, -1000, 0, 1000, i16::MAX)
        );
    }

    #[test]
    fn release_packet_releases_extension_channels_instead_of_ignoring_them() {
        let MavMessage::RC_CHANNELS_OVERRIDE(m) = control_message([0; 16], 42, 7, false) else {
            panic!("Wrong protocol")
        };
        assert_eq!((m.target_system, m.target_component), (42, 7));
        assert_eq!((m.chan1_raw, m.chan8_raw), (0, 0));
        assert_eq!((m.chan9_raw, m.chan16_raw), (65534, 65534));
    }

    #[test]
    fn legacy_axis_configs_load_and_button_settings_roundtrip() {
        let old = Saved {
            channels: default_channels(),
            elevons: true,
            manual_control: false,
            ..Default::default()
        };
        let mut value = serde_json::to_value(old).unwrap();
        value.as_object_mut().unwrap().remove("buttons");
        let mut saved: Saved = serde_json::from_value(value).unwrap();
        assert!(saved.buttons.iter().all(|c| c.buttonno == -1));
        saved.buttons[0] = ButtonCfg {
            buttonno: 3,
            function: Function::Do_Repeat_Servo,
            params: [6.0, 1700.0, 3.0, 0.5],
            ..Default::default()
        };
        let saved: Saved = serde_json::from_str(&serde_json::to_string(&saved).unwrap()).unwrap();
        assert_eq!(saved.buttons[0].params, [6.0, 1700.0, 3.0, 0.5]);
        assert_eq!(saved.buttons[0].function, Function::Do_Repeat_Servo);
    }

    #[test]
    fn stopping_joystick_sends_release_over_actual_link() {
        use std::net::UdpSocket;
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let link = Link::connect_with_params(
            mp_core::comms::ConnectionSpec::UdpClient {
                host: "127.0.0.1".into(),
                port: socket.local_addr().unwrap().port(),
            },
            None,
            false,
        );
        {
            let mut s = link.shared.write();
            s.cs.sysid = 42;
            s.cs.compid = 7;
        }
        let mut joystick = Joystick::default();
        joystick.was_active = true;
        joystick.enabled = false;
        joystick.tick(Some(&link), false);
        let mut bytes = [0; 4096];
        let end = Instant::now() + Duration::from_secs(3);
        loop {
            assert!(Instant::now() < end, "No release packet");
            let (n, _) = socket.recv_from(&mut bytes).unwrap();
            let (_, message) = mp_core::mavlink::MavlinkReader::new(&bytes[..n])
                .read_any_message::<MavMessage>()
                .unwrap();
            if let MavMessage::RC_CHANNELS_OVERRIDE(m) = message {
                assert_eq!(
                    (
                        m.target_system,
                        m.target_component,
                        m.chan1_raw,
                        m.chan16_raw
                    ),
                    (42, 7, 0, 65534)
                );
                break;
            }
        }
        assert!(!joystick.was_active);
    }

    #[test]
    fn controller_and_mapping_changes_cannot_create_an_arm_edge() {
        use std::net::UdpSocket;
        for remap in [false, true] {
            let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
            socket
                .set_read_timeout(Some(Duration::from_millis(30)))
                .unwrap();
            let link = Link::connect_with_params(
                mp_core::comms::ConnectionSpec::UdpClient {
                    host: "127.0.0.1".into(),
                    port: socket.local_addr().unwrap().port(),
                },
                None,
                false,
            );
            {
                let mut s = link.shared.write();
                s.status = mp_core::link::LinkStatus::Connected;
                s.cs.sysid = 42;
                s.cs.compid = 1;
            }
            let mut j = Joystick::default();
            #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
            {
                j.gilrs = None;
            }
            j.enabled = true;
            j.was_active = true;
            j.last_device = Some(1);
            j.device = Some(if remap { 1 } else { 2 });
            j.test_buttons = Some(vec![true, true]);
            j.buttons[0] = ButtonCfg {
                buttonno: if remap { 1 } else { 0 },
                function: Function::Arm,
                ..Default::default()
            };
            j.active_buttons[0] = ButtonCfg {
                buttonno: 0,
                function: Function::Arm,
                ..Default::default()
            };
            j.tick(Some(&link), false);
            assert!(!j.enabled, "Changed input source remained enabled");
            let end = Instant::now() + Duration::from_millis(150);
            let mut bytes = [0; 4096];
            let mut released = false;
            while Instant::now() < end {
                if let Ok((n, _)) = socket.recv_from(&mut bytes) {
                    let (_, message) = mp_core::mavlink::MavlinkReader::new(&bytes[..n])
                        .read_any_message::<MavMessage>()
                        .unwrap();
                    assert!(
                        !matches!(message, MavMessage::COMMAND_LONG(_)),
                        "Held button generated an action after source change"
                    );
                    if let MavMessage::RC_CHANNELS_OVERRIDE(m) = message {
                        released = m.chan1_raw == 0 && m.chan16_raw == 65534;
                    }
                }
            }
            assert!(released);
        }
    }
}
