//! Port of `GCSViews/InitialSetup` (SETUP): BackstageView with Install Firmware,
//! ">> Mandatory Hardware" and ">> Optional Hardware" pages.

use egui::{Color32, Pos2, Rect, RichText, Sense, Vec2};
use std::io::{Read, Write};

use mp_core::comms::ConnectionSpec;
use mp_core::link::{Link, LinkCommand};

use crate::app::AppCtx;
use crate::theme::{self, BsvItem};
use crate::views::config;

/// (label, parent, needs a connection) — order and labels from InitialSetup.cs.
/// When a page is listed (InitialSetup.HardwareConfig_Load conditions).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Vis {
    Always,
    /// isConnected
    Conn,
    /// isConnected && gotAllParams
    Params,
    /// isDisConnected
    Disc,
    /// connected, parameters still arriving (ConfigParamLoading)
    Loading,
    /// MAV_TYPE_HELICOPTER
    Heli,
    /// ArduTracker firmware
    Tracker,
}

// BackstageView draws each parent's children in the order they were added, so ADSB
// (added after Sik Radio with parent "Mandatory Hardware") sits at the end of that group.
const PAGES: &[(&str, Option<usize>, Vis)] = &[
    ("Loading", None, Vis::Loading),                  // 0
    ("Install Firmware", None, Vis::Always),          // 1
    ("Install Firmware Legacy", None, Vis::Disc),     // 2
    ("Secure", None, Vis::Disc),                      // 3
    ("Mandatory Hardware", None, Vis::Params),        // 4
    ("Heli Setup", Some(4), Vis::Heli),               // 5
    ("Frame Type", Some(4), Vis::Params),             // 6
    ("Accel Calibration", Some(4), Vis::Params),      // 7
    ("Compass", Some(4), Vis::Params),                // 8
    ("Radio Calibration", Some(4), Vis::Params),      // 9
    ("Servo Output", Some(4), Vis::Params),           // 10
    ("Serial Ports", Some(4), Vis::Params),           // 11
    ("ESC Calibration", Some(4), Vis::Params),        // 12
    ("Flight Modes", Some(4), Vis::Params),           // 13
    ("FailSafe", Some(4), Vis::Params),               // 14
    ("Initial Tune Parameter", Some(4), Vis::Params), // 15
    ("HW ID", Some(4), Vis::Params),                  // 16
    ("ADSB", Some(4), Vis::Params),                   // 17
    ("Optional Hardware", None, Vis::Always),         // 18
    ("RTK/GPS Inject", Some(18), Vis::Always),        // 19
    ("CubeID Update", Some(18), Vis::Conn),           // 20
    ("Sik Radio", Some(18), Vis::Always),             // 21
    ("CAN GPS Order", Some(18), Vis::Params),         // 22
    ("Battery Monitor", Some(18), Vis::Params),       // 23
    ("Battery Monitor 2", Some(18), Vis::Params),     // 24
    ("DroneCAN/UAVCAN", Some(18), Vis::Always),       // 25
    ("Joystick", Some(18), Vis::Always),              // 26
    ("Compass/Motor Calib", Some(18), Vis::Params),   // 27
    ("Range Finder", Some(18), Vis::Params),          // 28
    ("Airspeed", Some(18), Vis::Params),              // 29
    ("PX4Flow", Some(18), Vis::Always),               // 30
    ("Optical Flow", Some(18), Vis::Params),          // 31
    ("OSD", Some(18), Vis::Params),                   // 32
    ("Camera Gimbal", Some(18), Vis::Params),         // 33
    ("Antenna tracker", Some(18), Vis::Tracker),      // 34
    ("Motor Test", Some(18), Vis::Params),            // 35
    ("Bluetooth Setup", Some(18), Vis::Always),       // 36
    ("Parachute", Some(18), Vis::Params),             // 37
    ("ESP8266 Setup", Some(18), Vis::Params),         // 38
    ("Antenna Tracker", Some(18), Vis::Always),       // 39
    ("FFT Setup", Some(18), Vis::Params),             // 40
    ("Advanced", None, Vis::Always),                  // 41
    ("Terminal", Some(41), Vis::Always),              // 42
    ("Script REPL", Some(41), Vis::Conn),             // 43
];

fn page_visible(v: Vis, app: &AppCtx) -> bool {
    let link = app.link.filter(|_| app.connected());
    let all = link.map(crate::views::setup_pages::got_all_params).unwrap_or(false);
    match v {
        Vis::Always => true,
        Vis::Conn => link.is_some(),
        Vis::Params => all,
        Vis::Disc => link.is_none(),
        Vis::Loading => link.is_some() && !all,
        Vis::Heli => all && link.map(|l| l.shared.read().cs.mavtype == 4).unwrap_or(false),
        Vis::Tracker => link.map(|l| l.shared.read().cs.firmware == mp_core::firmware::Firmware::ArduTracker).unwrap_or(false),
    }
}

pub struct SetupView {
    page: usize,
    forms: std::collections::HashMap<&'static str, crate::forms::FormState>,
    param_compare: Option<Vec<(String, Option<f64>, f64, bool)>>,
    uarts: std::sync::Arc<std::sync::Mutex<Option<std::collections::BTreeMap<u32, String>>>>,
    uarts_requested: bool,
    serial_bitmask: Option<(String, u32)>,
    /// Sikradio background job result and the settings last read from the radio
    sik_done: std::sync::Arc<std::sync::Mutex<Option<SikDone>>>,
    sik_loaded: Option<mp_core::sik::SikSettings>,
    radio_cal: Option<RadioCal>,
    radio_done: bool,
    motor_pct: f32,
    accel_running: bool,
    accel_label: String,
    accel_user: String,
    level_label: String,
    simple_label: String,
    inject: crate::rtcm::Injector,
    inject_port: String,
    inject_baud: u32,
    inject_url: String,
    term: Option<Terminal>,
    term_port: String,
    term_baud: u32,
    term_input: String,
    firmware: crate::firmware_ui::FirmwareInstaller,
    extra: crate::views::setup_pages::ExtraPages,
    dev: crate::views::device_pages::DevicePages,
}

struct Terminal {
    rx: crossbeam_channel::Receiver<Vec<u8>>,
    tx: crossbeam_channel::Sender<Vec<u8>>,
    text: String,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

impl Drop for Terminal {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
    }
}

impl Default for SetupView {
    fn default() -> Self {
        Self {
            page: std::env::var("MP_PAGE").ok().and_then(|v| v.parse().ok()).unwrap_or(0),
            forms: Default::default(),
            param_compare: None,
            uarts: Default::default(),
            uarts_requested: false,
            serial_bitmask: None,
            sik_done: Default::default(),
            sik_loaded: None,
            radio_cal: None,
            radio_done: false,
            motor_pct: 0.0,
            accel_running: false,
            accel_label: "Calibrate Accel".into(),
            accel_user: String::new(),
            level_label: "Calibrate Level".into(),
            simple_label: "Simple Accel Cal".into(),
            inject: Default::default(),
            inject_port: "NTRIP".into(),
            inject_baud: 115200,
            inject_url: "ntrip://user:pass@host:2101/MOUNT".into(),
            term: None,
            term_port: String::new(),
            term_baud: 115200,
            term_input: String::new(),
            firmware: Default::default(),
            extra: Default::default(),
            dev: Default::default(),
        }
    }
}

struct RadioCal {
    min: [u16; 16],
    max: [u16; 16],
}

impl SetupView {
    /// Select a page by its BackstageView label.
    pub fn goto(&mut self, name: &str) {
        if let Some(i) = PAGES.iter().position(|p| p.0 == name) {
            self.page = i;
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        // map visible pages to BackstageView items (parents keep their indices)
        let visible: Vec<usize> = (0..PAGES.len()).filter(|i| page_visible(PAGES[*i].2, app)).collect();
        let items: Vec<BsvItem> = visible
            .iter()
            .map(|i| BsvItem { label: PAGES[*i].0.to_string(), parent: PAGES[*i].1.and_then(|p| visible.iter().position(|v| *v == p)) })
            .collect();
        // a page that is hidden while disconnected stays selected and returns on reconnect
        let shown = if visible.contains(&self.page) { self.page } else { visible.first().copied().unwrap_or(1) };
        let before = visible.iter().position(|v| *v == shown).unwrap_or(0);
        let mut sel = before;
        egui::Panel::left("setup_bsv")
            .resizable(false)
            .exact_size(theme::BSV_WIDTH)
            .frame(egui::Frame::new())
            .show(ui, |ui| theme::backstage(ui, &items, &mut sel));
        if sel != before {
            self.page = visible[sel];
        }
        let name = PAGES[if sel != before { self.page } else { shown }].0;
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg()).inner_margin(6)).show(ui, |ui| {
            match name {
                "Install Firmware" if app.connected() => return self.extra.fw_disabled(ui, app),
                "Install Firmware" => return self.firmware.ui(ui, app),
                "Install Firmware Legacy" => return self.firmware.ui_legacy(ui, app),
                "Secure" => return self.extra.secure(ui, app),
                "Loading" => return self.extra.param_loading(ui, app),
                "Antenna Tracker" => return self.extra.tracker(ui, app),
                "RTK/GPS Inject" => return self.rtk_inject(ui, app),
                "Terminal" => return self.terminal(ui, app),
                "Advanced" => return advanced(ui, app),
                "Joystick" => {
                    let fw = app.link.map(|l| format!("{:?}", l.shared.read().cs.firmware)).unwrap_or_else(|| "ArduCopter2".into());
                    return app.joystick.setup_ui(ui, &fw, app.toasts);
                }
                _ => {}
            }
            if let Some(form) = form_for(name) {
                if PAGES.iter().find(|p| p.0 == name).map(|p| matches!(p.2, Vis::Conn | Vis::Params)).unwrap_or(false) && !app.connected() {
                    theme::label(ui, "Please Connect First");
                    return;
                }
                return self.form_page(ui, app, name, form);
            }
            let Some(link) = app.link.filter(|_| app.connected()) else {
                theme::label(ui, "Please Connect First");
                return;
            };
            let fr = |ui: &mut egui::Ui, app: &mut AppCtx, pre: &'static [&'static str]| config::friendly(ui, app, &Default::default(), |n, _| pre.iter().any(|p| n.starts_with(p)));
            match name {
                "Heli Setup" => self.extra.heli(ui, app, link),
                "Frame Type" => frame_type(ui, app, link),
                "Accel Calibration" => accel(ui, self, link),
                "Compass" => compass(ui, app, link),
                "Radio Calibration" => self.radio(ui, app, link),
                "Servo Output" => servo_output(ui, app),
                "Serial Ports" => self.serial_ports(ui, app, link),
                "ESC Calibration" => esc_cal(ui, app, link),
                "Flight Modes" => config::flight_modes(ui, app),
                "FailSafe" => failsafe(ui, app, link),
                "Initial Tune Parameter" => config::friendly(ui, app, &Default::default(), |n, _| {
                    matches!(n, "MOT_THST_EXPO" | "MOT_SPIN_ARM" | "MOT_SPIN_MIN" | "MOT_BAT_VOLT_MAX" | "MOT_BAT_VOLT_MIN" | "INS_GYRO_FILTER" | "INS_ACCEL_FILTER" | "ATC_ACCEL_P_MAX" | "ATC_ACCEL_R_MAX" | "ATC_ACCEL_Y_MAX" | "ATC_THR_MIX_MAN")
                }),
                "HW ID" => hw_id(ui, link),
                "ADSB" => self.adsb_page(ui, app, link),
                "CAN GPS Order" => self.gps_order_page(ui, app, link),
                "Battery Monitor" => battery_monitor(ui, app, link, "BATT_"),
                "Battery Monitor 2" => battery_monitor(ui, app, link, "BATT2_"),
                "Compass/Motor Calib" => fr(ui, app, &["COMPASS_MOT"]),
                "Range Finder" => fr(ui, app, &["RNGFND"]),
                "Airspeed" => fr(ui, app, &["ARSPD"]),
                "PX4Flow" | "Optical Flow" => fr(ui, app, &["FLOW"]),
                "OSD" => self.hw_osd_page(ui, app, link),
                "Camera Gimbal" => fr(ui, app, &["MNT"]),
                "Motor Test" => self.motor_test(ui, link),
                "Parachute" => fr(ui, app, &["CHUTE"]),
                "FFT Setup" => self.fft_page(ui, app),
                _ => {
                    theme::label(ui, name);
                }
            }
        });
    }

    /// ConfigSerialInjectGPS: CMB_serialport / CMB_baudrate / Connect, bps and counters.
    fn rtk_inject(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.max_rect().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let st = self.inject.status.lock().clone();
        crate::views::tuning::group_box(ui.painter(), r(3.0, 3.0, 330.0, 110.0), "");
        let mut opts: Vec<String> = mp_core::comms::serial_ports();
        opts.extend(["TCP Client".into(), "UDP Host".into(), "NTRIP".into()]);
        ui.put(r(9.0, 12.0, 121.0, 21.0), |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_serialport")
                .width(113.0)
                .selected_text(self.inject_port.clone())
                .show_ui(ui, |ui| {
                    for o in &opts {
                        ui.selectable_value(&mut self.inject_port, o.clone(), o);
                    }
                })
                .response
        });
        ui.put(r(9.0, 39.0, 121.0, 21.0), |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("CMB_baudrate")
                .width(113.0)
                .selected_text(self.inject_baud.to_string())
                .show_ui(ui, |ui| {
                    for b in mp_core::comms::BAUD_RATES {
                        ui.selectable_value(&mut self.inject_baud, *b, b.to_string());
                    }
                })
                .response
        });
        if matches!(self.inject_port.as_str(), "NTRIP" | "TCP Client" | "UDP Host") {
            ui.put(r(136.0, 39.0, 190.0, 21.0), egui::TextEdit::singleline(&mut self.inject_url));
        }
        let br = r(136.0, 10.0, 75.0, 23.0);
        let label = if st.running { "Disconnect" } else { "Connect" };
        if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, label, br.size())).clicked() {
            if st.running {
                self.inject.stop();
            } else if let Some(link) = app.link {
                let src = match self.inject_port.as_str() {
                    "NTRIP" => crate::rtcm::Source::Ntrip(self.inject_url.clone()),
                    "TCP Client" => {
                        let (h, p) = self.inject_url.rsplit_once(':').unwrap_or((self.inject_url.as_str(), "2101"));
                        crate::rtcm::Source::Spec(ConnectionSpec::Tcp { host: h.trim_start_matches("tcp://").into(), port: p.parse().unwrap_or(2101) })
                    }
                    "UDP Host" => crate::rtcm::Source::Spec(ConnectionSpec::UdpListen { port: self.inject_url.parse().unwrap_or(5000) }),
                    p => crate::rtcm::Source::Spec(ConnectionSpec::Serial { port: p.into(), baud: self.inject_baud }),
                };
                self.inject.start(src, link.sender());
            } else {
                app.toasts.error("Please Connect First");
            }
        }
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let p = ui.painter();
        p.text(o + Vec2::new(9.0, 70.0), egui::Align2::LEFT_TOP, format!("{} bps", st.bps), f.clone(), Color32::WHITE);
        p.text(o + Vec2::new(9.0, 88.0), egui::Align2::LEFT_TOP, format!("Total {} bytes  Messages {}", st.total, st.msgs), f.clone(), Color32::WHITE);
        if let Some(e) = &st.error {
            p.text(o + Vec2::new(9.0, 120.0), egui::Align2::LEFT_TOP, e, f, Color32::RED);
        }
        if st.running {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
        }
        ui.allocate_rect(r(0.0, 0.0, 340.0, 140.0), Sense::hover());
    }

    /// ConfigTerminal: raw serial terminal to the autopilot / a radio.
    fn terminal(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        ui.horizontal(|ui| {
            let ports = mp_core::comms::serial_ports();
            egui::ComboBox::from_id_salt("term_port").width(110.0).selected_text(self.term_port.clone()).show_ui(ui, |ui| {
                for p in &ports {
                    ui.selectable_value(&mut self.term_port, p.clone(), p);
                }
            });
            egui::ComboBox::from_id_salt("term_baud").width(90.0).selected_text(self.term_baud.to_string()).show_ui(ui, |ui| {
                for b in mp_core::comms::BAUD_RATES {
                    ui.selectable_value(&mut self.term_baud, *b, b.to_string());
                }
            });
            let label = if self.term.is_some() { "Disconnect" } else { "Connect" };
            if theme::button(ui, label).clicked() {
                if self.term.is_some() {
                    self.term = None;
                } else {
                    match mp_core::comms::open(&ConnectionSpec::Serial { port: self.term_port.clone(), baud: self.term_baud }) {
                        Ok(mut t) => {
                            let (otx, orx) = crossbeam_channel::unbounded::<Vec<u8>>();
                            let (itx, irx) = crossbeam_channel::unbounded::<Vec<u8>>();
                            let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
                            let st = stop.clone();
                            let ctx = ui.ctx().clone();
                            std::thread::spawn(move || {
                                let mut buf = [0u8; 512];
                                while !st.load(std::sync::atomic::Ordering::Relaxed) {
                                    while let Ok(d) = irx.try_recv() {
                                        let _ = t.write_all(&d);
                                        let _ = t.flush();
                                    }
                                    if let Ok(n) = t.read(&mut buf) {
                                        if n > 0 {
                                            let _ = otx.send(buf[..n].to_vec());
                                            ctx.request_repaint();
                                        }
                                    }
                                }
                            });
                            self.term = Some(Terminal { rx: orx, tx: itx, text: String::new(), stop });
                        }
                        Err(e) => app.toasts.error(format!("Error: {e}")),
                    }
                }
            }
        });
        if let Some(t) = self.term.as_mut() {
            while let Ok(d) = t.rx.try_recv() {
                t.text.push_str(&String::from_utf8_lossy(&d));
            }
        }
        let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), (ui.available_height() - 30.0).max(100.0)), Sense::hover());
        ui.painter().rect_filled(rr, 0.0, Color32::BLACK);
        let text = self.term.as_ref().map(|t| t.text.clone()).unwrap_or_default();
        ui.scope_builder(egui::UiBuilder::new().max_rect(rr.shrink(4.0)), |ui| {
            egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
                ui.label(RichText::new(text).monospace().color(Color32::WHITE));
            });
        });
        let resp = ui.add(egui::TextEdit::singleline(&mut self.term_input).desired_width(ui.available_width()));
        if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            if let Some(t) = &self.term {
                let _ = t.tx.send(format!("{}\r\n", self.term_input).into_bytes());
            }
            self.term_input.clear();
            resp.request_focus();
        }
    }

    /// ConfigRadioInput: Roll/Yaw horizontal, Pitch/Throttle vertical, Radio 5-16, reverse,
    /// Calibrate Radio with red min/max lines, Spektrum bind, Elevon config (plane).
    fn radio(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
        let (cs, params) = {
            let s = link.shared.read();
            (s.cs.clone(), s.params.clone())
        };
        if let Some(cal) = self.radio_cal.as_mut() {
            for i in 0..16 {
                let v = cs.ch_in[i];
                if v > 800 && v < 2200 {
                    cal.min[i] = cal.min[i].min(v);
                    cal.max[i] = cal.max[i].max(v);
                }
            }
        }
        let o = ui.max_rect().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let p = ui.painter().clone();
        // RCMAP: roll/pitch/throttle/yaw channel numbers
        let map = |n: &str, d: usize| params.get(n).map(|p| p.value as usize).filter(|v| (1..=16).contains(v)).unwrap_or(d);
        let (ch_r, ch_p, ch_t, ch_y) = (map("RCMAP_ROLL", 1), map("RCMAP_PITCH", 2), map("RCMAP_THROTTLE", 3), map("RCMAP_YAW", 4));
        let mm = |ch: usize| self.radio_cal.as_ref().map(|c| (c.min[ch - 1], c.max[ch - 1]));
        rc_bar(&p, r(12.0, 13.0, 288.0, 23.0), "Roll", cs.ch_in[ch_r - 1], mm(ch_r), false);
        rc_bar(&p, r(12.0, 64.0, 47.0, 211.0), "Pitch", cs.ch_in[ch_p - 1], mm(ch_p), true);
        rc_bar(&p, r(251.0, 64.0, 47.0, 211.0), "Throttle", cs.ch_in[ch_t - 1], mm(ch_t), true);
        rc_bar(&p, r(12.0, 307.0, 288.0, 23.0), "Yaw", cs.ch_in[ch_y - 1], mm(ch_y), false);
        let others = [(5, 384.0, 13.0), (6, 384.0, 68.0), (7, 384.0, 123.0), (8, 384.0, 178.0), (9, 384.0, 234.0), (10, 510.0, 13.0), (11, 510.0, 68.0), (12, 510.0, 123.0), (13, 510.0, 178.0), (14, 510.0, 234.0), (15, 384.0, 290.0), (16, 510.0, 290.0)];
        for (ch, x, y) in others {
            rc_bar(&p, r(x, y, 118.0, 25.0), &format!("Radio {ch}"), cs.ch_in[ch - 1], mm(ch), false);
        }
        // CHK_revch1..4 bound to RCn_REVERSED
        for (ch, x, y) in [(ch_r, 306.0, 19.0), (ch_p, 65.0, 161.0), (ch_t, 304.0, 161.0), (ch_y, 306.0, 313.0)] {
            let name = format!("RC{ch}_REVERSED");
            if let Some(pv) = params.get(&name) {
                let mut b = pv.value != 0.0;
                if ui.put(r(x, y, 70.0, 17.0), egui::Checkbox::new(&mut b, "Reverse")).changed() && !app.readonly {
                    link.send(LinkCommand::SetParam { name, value: if b { 1.0 } else { 0.0 } });
                }
            }
        }
        let cal_r = r(491.0, 327.0, 134.0, 23.0);
        let label = if self.radio_cal.is_some() { "Click when Done" } else if self.radio_done { "Completed" } else { "Calibrate Radio" };
        if ui.put(cal_r, |ui: &mut egui::Ui| theme::button_sized(ui, label, cal_r.size())).clicked() {
            if self.radio_cal.is_none() {
                app.toasts.error("Ensure your transmitter is on and receiver is powered and connected\nEnsure your motor does not have power/no props!!!");
                app.toasts.error("Click OK and move all RC sticks and switches to their\nextreme positions so the red bars hit the limits.");
                self.radio_cal = Some(RadioCal { min: [u16::MAX; 16], max: [0; 16] });
            } else if let Some(cal) = self.radio_cal.take() {
                let mut summary = String::from("Here are the detected radio options\nNOTE Channels not connected are displayed as 1500 +-2\nNormal values are around 1100 | 1900\nChannel:Min | Max \n");
                for i in 0..16 {
                    if cal.max[i] > cal.min[i] && cal.max[i] - cal.min[i] > 200 {
                        let n = i + 1;
                        let trim = if n == ch_t { cal.min[i] } else { cs.ch_in[i].clamp(cal.min[i], cal.max[i]) };
                        if !app.readonly {
                            link.send(LinkCommand::SetParam { name: format!("RC{n}_MIN"), value: cal.min[i] as f32 });
                            link.send(LinkCommand::SetParam { name: format!("RC{n}_MAX"), value: cal.max[i] as f32 });
                            link.send(LinkCommand::SetParam { name: format!("RC{n}_TRIM"), value: trim as f32 });
                        }
                        summary.push_str(&format!("CH{n} {} | {}\n", cal.min[i], cal.max[i]));
                    }
                }
                app.toasts.error(summary);
                self.radio_done = true;
            }
        }
        // groupBoxElevons (plane only) and Spektrum Bind
        if cs.firmware == mp_core::firmware::Firmware::ArduPlane {
            let g = r(3.0, 356.0, 409.0, 42.0);
            crate::views::tuning::group_box(&p, g, "Elevon Config");
            for (x, w, t) in [(13.0, 64.0, "Elevons"), (83.0, 87.0, "Elevons Rev"), (176.0, 111.0, "Elevons CH1 Rev"), (293.0, 111.0, "Elevons CH2 Rev")] {
                let mut b = false;
                ui.put(Rect::from_min_size(g.min + Vec2::new(x, 19.0), Vec2::new(w, 17.0)), egui::Checkbox::new(&mut b, t));
            }
        }
        let g = r(418.0, 356.0, 207.0, 42.0);
        crate::views::tuning::group_box(&p, g, "Spektrum Bind");
        for (x, w, t, sub) in [(6.0, 63.0, "Bind DSM2", 0.0), (71.0, 65.0, "Bind DSMX", 1.0), (138.0, 65.0, "Bind DSM8", 2.0)] {
            let br = Rect::from_min_size(g.min + Vec2::new(x, 15.0), Vec2::new(w, 23.0));
            if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, t, br.size())).clicked() {
                // MAV_CMD_START_RX_PAIR: spektrum, sub type
                link.send(LinkCommand::CommandLong { command: 500, params: [0.0, sub, 0.0, 0.0, 0.0, 0.0, 0.0] });
            }
        }
        ui.allocate_rect(r(0.0, 0.0, 628.0, 406.0), Sense::hover());
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


/// HorizontalProgressBar2 / VerticalProgressBar2 as themed: ControlBG background, #94C11F value,
/// "Label  value" caption, red min/max lines while calibrating.
fn rc_bar(p: &egui::Painter, r: Rect, label: &str, value: u16, minmax: Option<(u16, u16)>, vertical: bool) {
    let (lo, hi) = (800.0f32, 2200.0f32);
    let f = ((value as f32 - lo) / (hi - lo)).clamp(0.0, 1.0);
    p.rect_filled(r, 0.0, theme::control_bg());
    let fill = if vertical {
        Rect::from_min_max(egui::pos2(r.left(), r.bottom() - r.height() * f), r.max)
    } else {
        Rect::from_min_size(r.min, Vec2::new(r.width() * f, r.height()))
    };
    p.rect_filled(fill, 0.0, theme::green());
    p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::from_gray(120)), egui::StrokeKind::Inside);
    let font = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    if let Some((mn, mx)) = minmax.filter(|(a, b)| b > a) {
        let red = egui::Stroke::new(2.0, Color32::RED);
        for (v, txt_col) in [(mn, Color32::from_rgb(0x40, 0x57, 0x04)), (mx, Color32::WHITE)] {
            let t = ((v as f32 - lo) / (hi - lo)).clamp(0.0, 1.0);
            if vertical {
                let y = r.bottom() - r.height() * t;
                p.line_segment([egui::pos2(r.left(), y), egui::pos2(r.right(), y)], red);
                p.text(egui::pos2(r.left() + 5.0, y + 2.0), egui::Align2::LEFT_TOP, v.to_string(), font.clone(), txt_col);
            } else {
                let x = r.left() + r.width() * t;
                p.line_segment([egui::pos2(x, r.top()), egui::pos2(x, r.bottom())], red);
                p.text(egui::pos2(x, r.top() + 5.0), egui::Align2::LEFT_TOP, v.to_string(), font.clone(), txt_col);
            }
        }
    }
    let caption = format!("{label}  {value}");
    if vertical {
        let g = p.layout_no_wrap(caption, font, Color32::WHITE);
        let pos = egui::pos2(r.center().x - g.size().y / 2.0, r.center().y + g.size().x / 2.0);
        p.add(egui::epaint::TextShape::new(pos, g, Color32::WHITE).with_angle(-std::f32::consts::FRAC_PI_2));
    } else {
        p.text(r.center(), egui::Align2::CENTER_CENTER, caption, font, Color32::WHITE);
    }
}

/// ConfigAccelerometerCalibration: one button that walks the vehicle's ACCELCAL_VEHICLE_POS prompts.
fn accel(ui: &mut egui::Ui, view: &mut SetupView, link: &Link) {
    let (pos, msgs) = {
        let s = link.shared.read();
        (s.accelcal_pos, s.messages.iter().rev().take(5).map(|m| m.text.clone()).collect::<Vec<_>>())
    };
    let o = ui.max_rect().min;
    let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
    let p = ui.painter().clone();
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    p.text(o + Vec2::new(3.0, 8.0), egui::Align2::LEFT_TOP, "Accelerometer Calibration", crate::fonts::bold_id(16.0), Color32::WHITE);
    p.line_segment([o + Vec2::new(7.0, 30.0), o + Vec2::new(467.0, 30.0)], egui::Stroke::new(1.0, Color32::GRAY));
    p.text(o + Vec2::new(83.0, 45.0), egui::Align2::LEFT_TOP, "Level your Autopilot to set default accelerometer Min/Max (3 axis).\nThis will ask you to place your autopilot on each edge.", f.clone(), Color32::WHITE);
    // vehicle prompts: COMMAND_LONG ACCELCAL_VEHICLE_POS and STATUSTEXT
    let names = |p: u32| match p {
        1 => "LEVEL",
        2 => "LEFT",
        3 => "RIGHT",
        4 => "NOSEDOWN",
        5 => "NOSEUP",
        6 => "BACK",
        16777215 => "SUCCESS",
        16777216 => "FAILED",
        _ => "",
    };
    if view.accel_running {
        let user = msgs.iter().find(|m| m.to_lowercase().contains("place vehicle") || m.to_lowercase().contains("calibration")).cloned();
        let text = match pos {
            Some(16777215) => {
                view.accel_running = false;
                view.accel_label = "Done".into();
                "Calibration successful".to_string()
            }
            Some(16777216) => {
                view.accel_running = false;
                view.accel_label = "Calibrate Accel".into();
                "Calibration FAILED".to_string()
            }
            Some(p) => format!("Please place vehicle {}", names(p)),
            None => user.unwrap_or_default(),
        };
        view.accel_user = text;
    }
    p.text(o + Vec2::new(7.0, 96.0), egui::Align2::LEFT_TOP, &view.accel_user, crate::fonts::bold_id(14.0), Color32::WHITE);
    let br = r(188.0, 72.0, 102.0, 21.0);
    if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, &view.accel_label, br.size())).clicked() {
        if !view.accel_running {
            link.shared.write().accelcal_pos = None;
            link.send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0] });
            view.accel_running = true;
            view.accel_label = "Click when Done".into();
        } else if let Some(p) = pos {
            link.send(LinkCommand::CommandLong { command: 42429, params: [p as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
        }
    }
    p.text(o + Vec2::new(60.0, 133.0), egui::Align2::LEFT_TOP, "Level your Autopilot to set default accelerometer offsets (1 axis/AHRS trims).\nThis requires you to place your autopilot flat and level.", f.clone(), Color32::WHITE);
    let lr = r(188.0, 168.0, 102.0, 21.0);
    if ui.put(lr, |ui: &mut egui::Ui| theme::button_sized(ui, &view.level_label, lr.size())).clicked() {
        link.send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0] });
        view.level_label = "Completed".into();
    }
    p.text(o + Vec2::new(60.0, 220.0), egui::Align2::LEFT_TOP, "Level your Autopilot to set default accelerometer scale factors for level flight (1 axis).\nThis requires you to place your autopilot flat and level.", f, Color32::WHITE);
    let sr = r(188.0, 249.0, 102.0, 21.0);
    if ui.put(sr, |ui: &mut egui::Ui| theme::button_sized(ui, &view.simple_label, sr.size())).clicked() {
        link.send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0] });
        view.simple_label = "Completed".into();
    }
    ui.allocate_rect(r(0.0, 0.0, 880.0, 280.0), Sense::hover());
}

const ADVANCED: &[(&str, f32, &str)] = &[
    ("Warning Manager", 5.0, "Enable custom warnings based on a set of conditions"),
    ("MAVLink Inspector", 45.0, "View decoded mavlink data being sent and received"),
    ("Proximity", 85.0, "View the data from a 360 lidar"),
    ("Mavlink Signing", 125.0, "Enable mavlink signing to secure communication with the MAV"),
    ("Mavlink Mirror", 165.0, "Mavlink mirror to an external location. For Monitoring or control"),
    ("NMEA", 205.0, "Output the MAV location as a NMEA string"),
    ("Follow Me", 245.0, "Use an external NMEA gps and send guided mode waypoints to the MAV based on that location"),
    ("Param gen", 285.0, "Regenerage the param info used inside mp"),
    ("Moving Base", 325.0, "Show an extra icon on the map of your current location."),
    ("Anon Log", 365.0, "Scramble lat/lng in bin or tlog"),
    ("FFT", 405.0, "Plot a FFT from a log"),
    ("Spectrogram", 445.0, "Plot a FFT from a log"),
    ("Support Proxy", 485.0, "Share connection with support engineer"),
];

/// ConfigAdvanced: buttons for the advanced tools with their descriptions.
fn advanced(ui: &mut egui::Ui, app: &mut AppCtx) {
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    p.text(o + Vec2::new(16.0, 11.0), egui::Align2::LEFT_TOP, "The following pages are for advanced configutation only. use with caution", f.clone(), Color32::WHITE);
    let t = o + Vec2::new(73.0, 27.0);
    for (b, y, l) in ADVANCED {
        let r = Rect::from_min_size(t + Vec2::new(5.0, *y), Vec2::new(84.0, 30.0));
        let resp = ui.scope_builder(egui::UiBuilder::new().max_rect(r), |ui| theme::button_sized(ui, b, r.size())).inner;
        let g = p.layout(l.to_string(), f.clone(), Color32::WHITE, 304.0);
        p.galley(t + Vec2::new(97.0, *y - 5.0 + 9.0), g, Color32::WHITE);
        if resp.clicked() {
            if *b == "MAVLink Inspector" {
                app.actions.push(crate::app::AppAction::OpenInspector);
            } else {
                app.actions.push(crate::app::AppAction::Tool(b.to_string()));
            }
        }
    }
}

// ------------------------------------------------------------------ designer-driven pages

/// Setup page label -> exported form name.
fn form_for(page: &str) -> Option<&'static str> {
    Some(match page {
        "Range Finder" => "rangefinder",
        "Airspeed" => "airspeed",
        "PX4Flow" | "Optical Flow" => "optflow",
        "Camera Gimbal" => "mount",
        "Parachute" => "parachute",
        "Antenna tracker" => "tracker",
        "Bluetooth Setup" => "bluetooth",
        "ESP8266 Setup" => "esp8266",
        "CubeID Update" => "cubeid",
        "DroneCAN/UAVCAN" => "dronecan",
        "Script REPL" => "repl",
        "Initial Tune Parameter" => "initialparams",
        "Compass/Motor Calib" => "compassmot",
        "Sik Radio" => "sikradio",
        _ => return None,
    })
}

enum SikDone {
    Busy(String),
    Loaded(mp_core::sik::SikSettings),
    Log(String),
    Err(String),
}

/// Sikradio combo choices per register (values as the radio reports them).
fn sik_choices(name: &str) -> Vec<&'static str> {
    match name.trim_start_matches('R') {
        "SERIAL_SPEED" => vec!["1", "2", "4", "9", "19", "38", "57", "115", "230", "460", "1000"],
        "AIR_SPEED" => vec!["4", "8", "16", "19", "24", "32", "48", "64", "96", "128", "192", "250"],
        "TXPOWER" => vec!["1", "2", "5", "8", "11", "14", "17", "20", "27", "30"],
        "MAVLINK" => vec!["0", "1", "2"],
        "MAX_WINDOW" => vec!["20", "33", "50", "80", "131"],
        "NUM_CHANNELS" => vec!["1", "2", "5", "10", "20", "30", "40", "50"],
        "DUTY_CYCLE" => vec!["10", "20", "30", "40", "50", "60", "70", "80", "90", "100"],
        "LBT_RSSI" => vec!["0", "25", "50", "100", "150", "200"],
        "MIN_FREQ" => vec!["414000", "433050", "868000", "902000", "915000"],
        "MAX_FREQ" => vec!["434790", "464000", "869000", "915000", "928000"],
        "ENCRYPTION_LEVEL" => vec!["0", "1"],
        "ANT_MODE" => vec!["0", "1", "2", "3"],
        "RATE_FREQBAND" => vec!["0", "1", "2"],
        _ => Vec::new(),
    }
}

impl SetupView {
    fn sik_busy(&self) -> bool {
        matches!(&*self.sik_done.lock().unwrap(), Some(SikDone::Busy(_)))
    }

    /// Run `job` against the radio on the selected COM port, in the background.
    fn sik_run(&self, app: &mut AppCtx, label: &str, job: impl FnOnce(&mut mp_core::sik::Sik) -> Result<SikDone, String> + Send + 'static) {
        if self.sik_busy() {
            return;
        }
        if app.connected() {
            app.toasts.error("Please disconnect from the autopilot first.\nThe radio is configured on its own COM port.");
            return;
        }
        let (port, baud) = (app.settings.port.clone(), app.settings.baud);
        if port.is_empty() || port == "AUTO" || port == "TCP" || port == "UDP" || port == "UDPCl" {
            app.toasts.error("Select the radio's COM port and baud rate (top right) first.");
            return;
        }
        *self.sik_done.lock().unwrap() = Some(SikDone::Busy(label.to_string()));
        let slot = self.sik_done.clone();
        std::thread::spawn(move || {
            let r = mp_core::sik::Sik::open(&port, baud)
                .map_err(|e| format!("{port}: {e}"))
                .and_then(|mut sik| {
                    if !sik.enter_command_mode(std::time::Duration::from_millis(1100)).map_err(|e| e.to_string())? {
                        return Err("Failed to enter command mode".into());
                    }
                    let r = job(&mut sik);
                    let _ = sik.exit();
                    r
                });
            *slot.lock().unwrap() = Some(r.unwrap_or_else(SikDone::Err));
        });
    }

    fn sik_poll(&mut self, st: &mut crate::forms::FormState, app: &mut AppCtx) {
        let done = {
            let mut g = self.sik_done.lock().unwrap();
            match &*g {
                Some(SikDone::Busy(l)) => {
                    st.set("lbl_status", l.clone());
                    None
                }
                _ => g.take(),
            }
        };
        match done {
            Some(SikDone::Loaded(cfg)) => {
                st.set("ATI", cfg.version.clone());
                st.set("ATI2", cfg.board.clone());
                st.set("ATI3", cfg.freq.clone());
                st.set("RSSI", cfg.rssi.clone());
                st.set("RTI", cfg.remote_version.clone());
                for (prefix, list) in [("", &cfg.local), ("R", &cfg.remote)] {
                    for p in list.iter() {
                        let key = format!("{prefix}{}", p.name);
                        let mut items: Vec<String> = sik_choices(&key).iter().map(|s| s.to_string()).collect();
                        if !items.contains(&p.value) {
                            items.push(p.value.clone());
                        }
                        st.items.insert(key.clone(), items);
                        st.checked.insert(key.clone(), p.value != "0");
                        st.set(&key, p.value.clone());
                    }
                }
                st.set("lbl_status", "Done");
                self.sik_loaded = Some(cfg);
            }
            Some(SikDone::Log(l)) => st.set("lbl_status", l),
            Some(SikDone::Err(e)) => {
                st.set("lbl_status", "Error");
                app.toasts.error(e);
            }
            _ => {}
        }
    }

    /// Current form values that differ from what was read: (local, remote).
    fn sik_changes(&self, st: &crate::forms::FormState) -> (Vec<mp_core::sik::SikParam>, Vec<mp_core::sik::SikParam>) {
        let Some(cfg) = &self.sik_loaded else { return (Vec::new(), Vec::new()) };
        let pick = |prefix: &str, list: &[mp_core::sik::SikParam]| -> Vec<mp_core::sik::SikParam> {
            list.iter()
                .filter_map(|p| {
                    let key = format!("{prefix}{}", p.name);
                    let v = match st.checked.get(&key) {
                        Some(on) if matches!(p.value.as_str(), "0" | "1") && !st.items.get(&key).map(|i| i.len() > 2).unwrap_or(false) => if *on { "1".to_string() } else { "0".to_string() },
                        _ => st.get(&key).to_string(),
                    };
                    (!v.is_empty() && v != p.value).then(|| mp_core::sik::SikParam { value: v, ..p.clone() })
                })
                .collect()
        };
        (pick("", &cfg.local), pick("R", &cfg.remote))
    }

    /// Sikradio button handlers; true when `b` was one of them.
    fn sik_button(&mut self, b: &str, app: &mut AppCtx, st: &mut crate::forms::FormState) -> bool {
        match b {
            "BUT_getcurrent" => self.sik_run(app, "Doing Command ATI & RTI", |s| s.load().map(SikDone::Loaded).map_err(|e| e.to_string())),
            "BUT_savesettings" => {
                if self.sik_loaded.is_none() {
                    app.toasts.error("Please Load Settings first");
                } else {
                    let (local, remote) = self.sik_changes(st);
                    self.sik_run(app, "Saving settings", move |s| s.save(&local, &remote).map(|l| SikDone::Log(format!("Saved: {}", l.join("; ")))).map_err(|e| e.to_string()));
                }
            }
            "BUT_resettodefault" => self.sik_run(app, "Resetting to defaults", |s| s.reset_defaults().map(|_| SikDone::Log("Reset to defaults - reload settings".into())).map_err(|e| e.to_string())),
            "BUT_SetPPMFailSafe" | "BUT_SetPPMFailSafeRemote" => {
                let c = if b.ends_with("Remote") { "RT&R" } else { "AT&R" };
                self.sik_run(app, "Setting PPM fail safe", move |s| s.cmd(c).map(|r| SikDone::Log(format!("{c} {r}"))).map_err(|e| e.to_string()));
            }
            "BUT_Syncoptions" => {
                // the settings both radios must share for a link
                for k in ["AIR_SPEED", "NETID", "ECC", "MAVLINK", "MIN_FREQ", "MAX_FREQ", "NUM_CHANNELS", "MAX_WINDOW", "AESKEY"] {
                    let v = st.get(k).to_string();
                    st.set(&format!("R{k}"), v);
                    if let Some(c) = st.checked.get(k).copied() {
                        st.checked.insert(format!("R{k}"), c);
                    }
                }
            }
            "btnRandom" => {
                let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_nanos();
                let key: String = (0..32).map(|i| format!("{:X}", ((seed >> (i % 16 * 4)) as u64).wrapping_mul(6364136223846793005u64.wrapping_add(i as u64)) >> 60)).collect();
                st.set("AESKEY", key);
            }
            "btnSaveToFile" | "btnRemoteSaveToFile" => {
                let prefix = if b.starts_with("btnRemote") { "R" } else { "" };
                let list = self.sik_loaded.as_ref().map(|c| if prefix == "R" { c.remote.clone() } else { c.local.clone() }).unwrap_or_default();
                let text: String = list.iter().map(|p| format!("{}={}\r\n", p.name, st.get(&format!("{prefix}{}", p.name)))).collect();
                if let Some(Err(e)) = crate::platform::save_text("Save to File...", "sikradio.txt", "Text", &["txt"], &text) {
                    app.toasts.error(e);
                }
            }
            "btnLoadFromFile" | "btnRemoteLoadFromFile" => {
                let prefix = if b.starts_with("btnRemote") { "R" } else { "" };
                if let Some((_, text)) = crate::platform::open_text("Load from File...", "Text", &["txt"]) {
                    for l in text.lines() {
                        if let Some((k, v)) = l.split_once('=') {
                            st.set(&format!("{prefix}{}", k.trim()), v.trim().to_string());
                            st.checked.insert(format!("{prefix}{}", k.trim()), v.trim() != "0");
                        }
                    }
                }
            }
            _ => return false,
        }
        true
    }
}

/// ConfigInitialParams.calc_values: suggested parameters for a prop size and battery.
fn initial_params(prop: f64, cells: f64, cmax: f64, cmin: f64, tmotor: bool, suggested: bool, params: &std::collections::BTreeMap<String, mp_core::link::Param>, plane: bool) -> Vec<(String, f64)> {
    let round_to = |v: f64, p: i32| -> f64 {
        if p >= 0 {
            let m = 10f64.powi(p);
            (v * m).round() / m
        } else {
            let q = 10f64.powi(-p);
            let v = v + 5.0 * q / 10.0;
            (v - v.rem_euclid(q)).round()
        }
    };
    let acc_y = (round_to(-900.0 * prop + 36000.0, -2)).max(8000.0);
    let acro_yaw_p = 0.5 * acc_y / 4500.0;
    let acc_p = round_to(-2.613267 * prop.powi(3) + 343.39216 * prop.powi(2) - 15083.7121 * prop + 235771.0, -2).max(10000.0);
    let gyro = (289.22 * prop.powf(-0.838)).round().max(20.0);
    let flt = (gyro / 2.0).max(10.0);
    let mut expo = (0.15686 * prop.ln() + 0.23693).min(0.80);
    expo = (expo * 100.0).round() / 100.0;
    if tmotor {
        expo = 0.2;
    }
    let (atc, mot) = if plane { ("Q_A", "Q_M") } else { ("ATC", "MOT") };
    let mut out: Vec<(String, f64)> = vec![("ACRO_YAW_P".into(), acro_yaw_p)];
    if params.contains_key(&format!("{atc}_ACCEL_P_MAX")) {
        out.push((format!("{atc}_ACCEL_P_MAX"), acc_p));
        out.push((format!("{atc}_ACCEL_R_MAX"), acc_p));
        out.push((format!("{atc}_ACCEL_Y_MAX"), acc_y));
    } else {
        out.push((format!("{atc}_ACC_P_MAX"), acc_p / 100.0));
        out.push((format!("{atc}_ACC_R_MAX"), acc_p / 100.0));
        out.push((format!("{atc}_ACC_Y_MAX"), acc_y / 100.0));
    }
    for (k, v) in [("RAT_PIT_FLTD", flt), ("RAT_PIT_FLTE", 0.0), ("RAT_PIT_FLTT", flt), ("RAT_RLL_FLTD", flt), ("RAT_RLL_FLTE", 0.0), ("RAT_RLL_FLTT", flt), ("RAT_YAW_FLTD", 0.0), ("RAT_YAW_FLTE", 2.0), ("RAT_YAW_FLTT", flt), ("THR_MIX_MAN", 0.1)] {
        out.push((format!("{atc}_{k}"), v));
    }
    out.push(("INS_ACCEL_FILTER".into(), 10.0));
    out.push(("INS_GYRO_FILTER".into(), gyro));
    out.push((format!("{mot}_THST_EXPO"), expo));
    out.push((format!("{mot}_THST_HOVER"), 0.2));
    out.push(("BATT_ARM_VOLT".into(), (cells - 1.0) * 0.1 + (cmin + 0.3) * cells));
    out.push(("BATT_CRT_VOLT".into(), (cmin + 0.2) * cells));
    out.push(("BATT_LOW_VOLT".into(), (cmin + 0.3) * cells));
    out.push((format!("{mot}_BAT_VOLT_MAX"), cmax * cells));
    out.push((format!("{mot}_BAT_VOLT_MIN"), cmin * cells));
    if tmotor {
        out.push((format!("{mot}_PWM_MIN"), 1100.0));
        out.push((format!("{mot}_PWM_MAX"), 1940.0));
    }
    if suggested && !plane {
        for (k, v) in [("BATT_FS_CRT_ACT", 1.0), ("BATT_FS_LOW_ACT", 2.0), ("FENCE_ACTION", 3.0), ("FENCE_ALT_MAX", 120.0), ("FENCE_ENABLE", 1.0), ("FENCE_RADIUS", 150.0), ("FENCE_TYPE", 7.0)] {
            out.push((k.into(), v));
        }
    }
    out
}

impl SetupView {
    /// Pages rendered from the exported designer layouts, with their button handlers.
    fn form_page(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, page: &str, form: &'static str) {
        let mut st_owned = self.forms.remove(form).unwrap_or_default();
        let st = &mut st_owned;
        if form == "initialparams" && st.text.is_empty() {
            st.items.insert("cmb_batterytype".into(), vec!["LiPo".into(), "LiPoHV".into(), "LiIon".into()]);
            st.set("cmb_batterytype", "LiPo");
            st.set("t_prop", "9");
            st.set("t_cellcount", "4");
        }
        if form == "compassmot" {
            if let Some(l) = app.link {
                let s = l.shared.read();
                if let Some(cm) = s.inspector.values().find_map(|m| match &m.msg {
                    mp_core::mavlink::dialects::ardupilotmega::MavMessage::COMPASSMOT_STATUS(c) => Some(c.clone()),
                    _ => None,
                }) {
                    st.set("lbl_status", format!("Throttle {}%  Current {:.1}A  Interference {}%  Comp {:.0},{:.0},{:.0}", cm.throttle / 10, cm.current, cm.interference, cm.CompensationX, cm.CompensationY, cm.CompensationZ));
                }
            }
        }
        if form == "sikradio" {
            self.sik_poll(st, app);
            if self.sik_busy() {
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
            }
        }
        self.dev.pre(ui.ctx(), form, st, app);
        let clicked = crate::forms::show(ui, app, form, st);
        let params = app.link.map(|l| l.shared.read().params.clone()).unwrap_or_default();
        for b in clicked {
            match (form, b.as_str()) {
                ("initialparams", "cmb_batterytype") => {
                    let (mx, mn) = match st.get("cmb_batterytype") {
                        "LiPoHV" => ("4.35", "3.3"),
                        "LiIon" => ("4.1", "2.8"),
                        _ => ("4.2", "3.3"),
                    };
                    st.set("t_cellmax", mx);
                    st.set("t_cellmin", mn);
                }
                ("initialparams", "btn_docalc") => {
                    let num = |k: &str| st.get(k).trim().parse::<f64>().unwrap_or(0.0);
                    let (prop, cells) = (num("t_prop"), num("t_cellcount"));
                    if prop <= 0.0 {
                        app.toasts.error("Prop size must be larger than zero.");
                    } else if cells < 1.0 {
                        app.toasts.error("Battery cell count must be at least 1.");
                    } else {
                        let plane = app.link.map(|l| l.shared.read().cs.firmware == mp_core::firmware::Firmware::ArduPlane).unwrap_or(false);
                        let list = initial_params(prop, cells, num("t_cellmax"), num("t_cellmin"), *st.checked.get("cb_tmotor").unwrap_or(&false), *st.checked.get("cb_suggested").unwrap_or(&false), &params, plane);
                        self.param_compare = Some(list.into_iter().map(|(k, v)| {
                            let cur = params.get(&k).map(|p| p.value as f64);
                            (k, cur, v, true)
                        }).collect());
                    }
                }
                ("compassmot", "BUT_compassmot") => match app.link {
                    // PREFLIGHT_CALIBRATION param6 = 1: compass / motor interference
                    Some(l) => l.send(LinkCommand::CommandLong { command: 241, params: [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0] }),
                    None => app.toasts.error("Please Connect First"),
                },
                ("tracker", "BUT_test_pitch") | ("tracker", "BUT_test_yaw") => match app.link {
                    Some(l) => l.send(LinkCommand::CommandLong { command: 205, params: [if b.ends_with("pitch") { 30.0 } else { 0.0 }, 0.0, if b.ends_with("yaw") { 45.0 } else { 0.0 }, 0.0, 0.0, 0.0, 2.0] }),
                    None => app.toasts.error("Please Connect First"),
                },
                ("sikradio", b) if self.sik_button(b, app, st) => {}
                (f, b) if self.dev.button(f, b, st, app) => {}
                ("dronecan", other) => {
                    let label = crate::forms::form(form).controls.iter().find(|c| c.name == other).and_then(|c| c.text.clone()).unwrap_or_default();
                    if !label.is_empty() {
                        app.toasts.error(format!("{page}: {label}\nNo DroneCAN interface. Connect to an autopilot with CAN, then press Connect (SLCAN over MAVLink)."));
                    }
                }
                _ => {}
            }
        }
        self.forms.insert(form, st_owned);
        self.param_compare_ui(ui.ctx(), app);
    }

    /// ParamCompare dialog ("Write to FC") used by the initial parameter calculator.
    fn param_compare_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some(list) = self.param_compare.as_mut() else { return };
        let mut done = None;
        egui::Window::new("Param Compare").collapsible(false).default_size([520.0, 420.0]).show(ctx, |ui| {
            egui::ScrollArea::vertical().max_height(360.0).show(ui, |ui| {
                egui::Grid::new("pcmp").num_columns(4).striped(true).show(ui, |ui| {
                    for h in ["Command", "Value", "New Value", "Use"] {
                        ui.label(RichText::new(h).strong().color(Color32::WHITE));
                    }
                    ui.end_row();
                    for (k, cur, new, use_it) in list.iter_mut() {
                        theme::label(ui, k.as_str());
                        theme::label(ui, cur.map(|c| format!("{c}")).unwrap_or_else(|| "-".into()));
                        theme::label(ui, format!("{:.4}", new).trim_end_matches('0').trim_end_matches('.').to_string());
                        ui.checkbox(use_it, "");
                        ui.end_row();
                    }
                });
            });
            ui.horizontal(|ui| {
                if theme::button(ui, "Write to FC").clicked() {
                    done = Some(true);
                }
                if theme::button(ui, "Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        if let Some(write) = done {
            let list = self.param_compare.take().unwrap_or_default();
            if write {
                match app.link {
                    Some(l) if !app.readonly => {
                        for (k, cur, v, use_it) in list {
                            if use_it && cur.is_some() {
                                l.send(LinkCommand::SetParam { name: k, value: v as f32 });
                            }
                        }
                        app.toasts.error("Initial Parameters successfully updated.\nCheck parameters before flight!\n\nAfter test flight (land and disarm first):\n\tSet ATC_THR_MIX_MAN to 0.5\n\tSet PSC_ACCZ_P to MOT_THST_HOVER\n\tSet PSC_ACCZ_I to 2*MOT_THST_HOVER\n\nHappy flying!");
                    }
                    _ => app.toasts.error("Please Connect First"),
                }
            }
        }
    }
}

impl SetupView {
    /// ConfigSerial: one row per SERIALn (152 / 114 / 132 / 307 px columns): port name with
    /// the board's UART name from @SYS/uarts.txt, baud, protocol, options summary, Set Bitmask.
    fn serial_ports(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
        // _gotUARTNames: fetch @SYS/uarts.txt once over MAVFtp
        if !self.uarts_requested {
            self.uarts_requested = true;
            let mut ftp = mp_core::ftp::Ftp::new(link);
            let slot = self.uarts.clone();
            std::thread::spawn(move || {
                let names: std::collections::BTreeMap<u32, String> = ftp
                    .get("@SYS/uarts.txt", |_, _| {})
                    .map(|b| {
                        String::from_utf8_lossy(&b)
                            .lines()
                            .filter_map(|l| {
                                let mut it = l.split_whitespace();
                                let port = it.next()?;
                                let n: u32 = port.strip_prefix("SERIAL")?.parse().ok()?;
                                Some((n, it.next().unwrap_or("").to_string()))
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                *slot.lock().unwrap() = Some(names);
            });
        }
        let uarts = self.uarts.lock().unwrap().clone().unwrap_or_default();
        let (params, fw) = {
            let s = link.shared.read();
            (s.params.clone(), s.cs.firmware)
        };
        let o = ui.max_rect().min;
        let p = ui.painter().clone();
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let bold = crate::fonts::bold_id(crate::fonts::UI_SIZE);
        let row_h = 44.0;
        let cols = [152.0f32, 114.0, 132.0, 307.0];
        let mut y = 0.0;
        for i in 0..10u32 {
            let port = format!("SERIAL{i}");
            let baud = format!("{port}_BAUD");
            if !params.contains_key(&baud) {
                continue;
            }
            let x0 = o.x;
            let cell = |c: usize| {
                let x: f32 = cols[..c].iter().sum();
                Rect::from_min_size(Pos2::new(x0 + x, o.y + y), Vec2::new(cols[c], row_h))
            };
            let name = uarts.get(&i).cloned().unwrap_or_default();
            let lr = cell(0);
            p.text(lr.center(), egui::Align2::CENTER_CENTER, format!("SERIAL PORT {i}\n{name}"), bold.clone(), Color32::WHITE);
            for (c, pname) in [(1usize, baud.clone()), (2, format!("{port}_PROTOCOL"))] {
                if params.contains_key(&pname) {
                    let r = cell(c);
                    let cr = Rect::from_min_size(Pos2::new(r.left() + 3.0, r.center().y - 11.0), Vec2::new(r.width() - 6.0, 22.0));
                    ui.scope_builder(egui::UiBuilder::new().max_rect(cr), |ui| config::param_control(ui, link, fw, &pname, params[&pname].value, cr.width() - 8.0));
                }
            }
            let opt = format!("{port}_OPTIONS");
            if let Some(pv) = params.get(&opt) {
                let bits = mp_core::param_meta::get(fw, &opt).map(|m| m.bitmask.clone()).unwrap_or_default();
                let v = pv.value as u32;
                let names: Vec<String> = bits.iter().filter(|(b, _)| v & (1 << b) != 0).map(|(_, n)| n.clone()).collect();
                let summary = if names.is_empty() { "".to_string() } else { names.join(", ") };
                let r = cell(3);
                let g = p.layout(summary, f.clone(), Color32::WHITE, 300.0 - 90.0);
                p.galley(Pos2::new(r.left() + 3.0, r.center().y - g.size().y / 2.0), g, Color32::WHITE);
                let br = Rect::from_min_size(Pos2::new(r.right() - 85.0, r.center().y - 12.0), Vec2::new(82.0, 24.0));
                if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, "Set Bitmask", br.size())).clicked() {
                    self.serial_bitmask = Some((opt.clone(), v));
                }
            }
            y += row_h;
        }
        if y == 0.0 {
            theme::label(ui, "No serial port parameters on this vehicle");
        }
        p.text(o + Vec2::new(0.0, y + 8.0), egui::Align2::LEFT_TOP, "Note: changes to serial port settings require a reboot of the autopilot.", f.clone(), Color32::WHITE);
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(cols.iter().sum(), y + 30.0)), Sense::hover());

        // MavlinkCheckBoxBitMask popup
        if let Some((name, mut v)) = self.serial_bitmask.clone() {
            let bits = mp_core::param_meta::get(fw, &name).map(|m| m.bitmask.clone()).unwrap_or_default();
            let mut done = None;
            egui::Window::new(name.as_str()).collapsible(false).resizable(false).show(ui.ctx(), |ui| {
                egui::Grid::new("serial_bits").num_columns(2).show(ui, |ui| {
                    for (k, (b, n)) in bits.iter().enumerate() {
                        let mut on = v & (1 << b) != 0;
                        if ui.checkbox(&mut on, n.as_str()).changed() {
                            if on {
                                v |= 1 << b;
                            } else {
                                v &= !(1 << b);
                            }
                        }
                        if k % 2 == 1 {
                            ui.end_row();
                        }
                    }
                });
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() {
                        done = Some(true);
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        done = Some(false);
                    }
                });
            });
            match done {
                Some(true) => {
                    if !app.readonly {
                        link.send(LinkCommand::SetParam { name, value: v as f32 });
                    }
                    self.serial_bitmask = None;
                }
                Some(false) => self.serial_bitmask = None,
                None => self.serial_bitmask = Some((name, v)),
            }
        }
    }
}

impl SetupView {
    /// ConfigADSB: Write / Refresh / Find buttons, flight id and registration row, then the
    /// ADSB_* parameters as friendly-param controls.
    fn adsb_page(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
        let mut st = self.forms.remove("adsb").unwrap_or_default();
        let clicked = crate::forms::show_fixed(ui, app, "adsb", &mut st, 67.0);
        for b in clicked {
            match b.as_str() {
                "BUT_rerequestparams" => link.send(LinkCommand::RequestParams),
                "BUT_writePIDS" => app.toasts.error("Parameters are written as they are changed."),
                "BUT_Find" => st.set("_find_open", "1"),
                // MP's flight id / registration save buttons are commented out (ConfigADSB.cs:673)
                _ => {}
            }
        }
        if st.get("_find_open") == "1" {
            let mut q = st.get("_find").to_string();
            let mut done = None;
            egui::Window::new("Search For").collapsible(false).resizable(false).show(ui.ctx(), |ui| {
                theme::label(ui, "Enter a single word to search for");
                ui.text_edit_singleline(&mut q);
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() {
                        done = Some(true);
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        done = Some(false);
                    }
                });
            });
            st.set("_find", q.clone());
            if let Some(ok) = done {
                st.set("_find_open", "");
                if !ok {
                    st.set("_find", "");
                }
            }
        }
        let q = st.get("_find").to_lowercase();
        self.forms.insert("adsb", st);
        config::friendly(ui, app, &Default::default(), move |n, m| {
            n.starts_with("ADSB_") && (q.is_empty() || n.to_lowercase().contains(&q) || m.map(|m| m.description.to_lowercase().contains(&q) || m.display_name.to_lowercase().contains(&q)).unwrap_or(false))
        });
    }

    /// ConfigGPSOrder: GPS override / detected node ids with Override 1 / Override 2 buttons.
    fn gps_order_page(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
        let params = link.shared.read().params.clone();
        let mut st = self.forms.remove("gpsorder").unwrap_or_default();
        st.set("myDataGridView1", "");
        let o = ui.max_rect().min;
        crate::forms::show(ui, app, "gpsorder", &mut st);
        self.forms.insert("gpsorder", st);
        if !params.contains_key("GPS1_CAN_OVRIDE") {
            ui.painter().text(o + Vec2::new(8.0, 60.0), egui::Align2::LEFT_TOP, "This vehicle has no DroneCAN GPS override parameters (GPS1_CAN_OVRIDE)", crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::WHITE);
            return;
        }
        let v = |n: &str| params.get(n).map(|p| p.value as i64).unwrap_or(0);
        let mut rows = Vec::new();
        if v("GPS1_CAN_OVRIDE") != 0 {
            rows.push((1, v("GPS1_CAN_OVRIDE"), "GPS Override 1"));
        }
        if v("GPS2_CAN_OVRIDE") != 0 {
            rows.push((2, v("GPS2_CAN_OVRIDE"), "GPS Override 2"));
        }
        if v("GPS_CAN_NODEID1") != 0 {
            rows.push((98, v("GPS_CAN_NODEID1"), "GPS Detect 1"));
        }
        if v("GPS_CAN_NODEID2") != 0 {
            rows.push((99, v("GPS_CAN_NODEID2"), "GPS Detect 2"));
        }
        let p = ui.painter().clone();
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let cols = [("Order", 60.0f32), ("NodeID", 80.0), ("Name", 140.0), ("GPS1", 110.0), ("GPS2", 110.0)];
        let mut x = o.x + 3.0;
        for (h, w) in cols {
            let r = Rect::from_min_size(Pos2::new(x, o.y + 49.0), Vec2::new(w, 21.0));
            p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::from_gray(160)), egui::StrokeKind::Inside);
            p.text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, h, f.clone(), Color32::WHITE);
            x += w;
        }
        for (i, (order, node, name)) in rows.iter().enumerate() {
            let y = o.y + 70.0 + i as f32 * 22.0;
            let mut x = o.x + 3.0;
            for (k, (_, w)) in cols.iter().enumerate() {
                let r = Rect::from_min_size(Pos2::new(x, y), Vec2::new(*w, 22.0));
                p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::from_gray(160)), egui::StrokeKind::Inside);
                match k {
                    0 => {
                        p.text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, order.to_string(), f.clone(), Color32::WHITE);
                    }
                    1 => {
                        p.text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, node.to_string(), f.clone(), Color32::WHITE);
                    }
                    2 => {
                        p.text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, *name, f.clone(), Color32::WHITE);
                    }
                    _ => {
                        let label = if k == 3 { "Override 1" } else { "Override 2" };
                        let br = r.shrink(1.0);
                        if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, label, br.size())).clicked() && !app.readonly {
                            link.send(LinkCommand::SetParam { name: if k == 3 { "GPS1_CAN_OVRIDE".into() } else { "GPS2_CAN_OVRIDE".into() }, value: *node as f32 });
                        }
                    }
                }
                x += w;
            }
        }
    }

    /// ConfigHWOSD: MinimOSD picture and "Enable Telemetry" (streams the SRn rates the OSD needs).
    fn hw_osd_page(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
        let mut st = self.forms.remove("hwosd").unwrap_or_default();
        let clicked = crate::forms::show(ui, app, "hwosd", &mut st);
        self.forms.insert("hwosd", st);
        if clicked.iter().any(|b| b == "BUT_osdrates") && !app.readonly {
            for sr in ["SR0", "SR1", "SR3"] {
                for s in ["EXT_STAT", "EXTRA1", "EXTRA2", "EXTRA3", "POSITION", "RAW_CTRL", "RAW_SENS", "RC_CHAN"] {
                    let n = format!("{sr}_{s}");
                    if link.shared.read().params.contains_key(&n) {
                        link.send(LinkCommand::SetParam { name: n, value: 2.0 });
                    }
                }
            }
            app.toasts.error("Set telemetry rates to 2hz");
        }
    }

    /// ConfigFFT: the FFT_* parameters with the FFT button (opens the log FFT tool).
    fn fft_page(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.max_rect().min;
        let br = Rect::from_min_size(o + Vec2::new(629.0, 13.0), Vec2::new(75.0, 23.0));
        if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, "FFT", br.size())).clicked() {
            app.actions.push(crate::app::AppAction::Tool("FFT".into()));
        }
        config::friendly(ui, app, &Default::default(), |n, _| n.starts_with("FFT_") || n == "INS_LOG_BAT_CNT" || n == "INS_LOG_BAT_MASK" || n == "INS_LOG_BAT_OPT");
    }
}

/// MAVLink-bound NumericUpDown at a fixed rect: hidden when the param is missing.
fn mav_num(ui: &mut egui::Ui, app: &AppCtx, link: &Link, name: &str, r: Rect, range: (f64, f64), inc: f64) -> bool {
    let Some(cur) = link.shared.read().params.get(name).map(|p| p.value) else { return false };
    let mut v = cur as f64;
    let dec = if inc >= 1.0 { 0 } else { (-inc.log10()).ceil() as usize };
    let resp = ui.put(r, egui::DragValue::new(&mut v).range(range.0..=range.1).speed(inc).max_decimals(dec).min_decimals(dec));
    if (resp.drag_stopped() || resp.lost_focus() || (resp.changed() && !resp.dragged() && !resp.has_focus())) && (v as f32) != cur && !app.readonly {
        link.send(LinkCommand::SetParam { name: name.into(), value: v as f32 });
    }
    true
}

/// MAVLink-bound ComboBox (options from the parameter metadata) at a fixed rect.
fn mav_combo(ui: &mut egui::Ui, link: &Link, name: &str, r: Rect) -> bool {
    let (val, fw) = {
        let s = link.shared.read();
        let Some(v) = s.params.get(name).map(|p| p.value) else { return false };
        (v, s.cs.firmware)
    };
    ui.scope_builder(egui::UiBuilder::new().max_rect(r), |ui| config::param_control(ui, link, fw, name, val, r.width() - 8.0));
    true
}

/// MAVLink-bound CheckBox (non-zero = checked).
fn mav_check(ui: &mut egui::Ui, app: &AppCtx, link: &Link, name: &str, r: Rect, text: &str) -> bool {
    let Some(cur) = link.shared.read().params.get(name).map(|p| p.value) else { return false };
    let mut on = cur != 0.0;
    if ui.put(r, egui::Checkbox::new(&mut on, text)).changed() && !app.readonly {
        link.send(LinkCommand::SetParam { name: name.into(), value: if on { 1.0 } else { 0.0 } });
    }
    true
}

/// ConfigFailSafe: radio in / servo out bars, big mode/armed/GPS labels and the
/// Battery / Radio / GCS failsafe groups.
fn failsafe(ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
    let txt = |x: f32, y: f32, s: &str| {
        p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, s, f.clone(), Color32::WHITE);
    };
    let cs = link.shared.read().cs.clone();
    txt(84.0, 5.0, "Radio IN");
    txt(342.0, 5.0, "Servo/Motor OUT");
    p.text(o + Vec2::new(657.0, 5.0), egui::Align2::LEFT_TOP, "Wiki", f.clone(), Color32::from_rgb(0x00, 0x66, 0xcc));
    let ys = [19.0, 70.0, 121.0, 172.0, 222.0, 273.0, 324.0, 375.0];
    for (i, y) in ys.iter().enumerate() {
        rc_bar(&p, at(15.0, *y, 170.0, 23.0), &format!("Radio {}", i + 1), cs.ch_in[i], None, false);
        rc_bar(&p, at(297.0, *y, 170.0, 23.0), &format!("Radio {}", i + 1), cs.ch_out[i], None, false);
    }
    let big = egui::FontId::proportional(35.0);
    let gps = match cs.gpsstatus {
        0 | 1 => "No Fix",
        2 => "2D Fix",
        3 => "3D Fix",
        4 => "3D dgps",
        5 => "3D rtk float",
        6 => "3D rtk Fixed",
        _ => "No Lock",
    };
    for (y, s) in [(19.0, cs.mode.as_str()), (60.0, if cs.armed { "Armed" } else { "Disarmed" }), (102.0, gps)] {
        let g = p.layout_no_wrap(s.to_string(), big.clone(), Color32::WHITE);
        p.galley(o + Vec2::new(473.0, y - 4.0), g, Color32::WHITE);
    }

    // groupBox4: Battery
    let gb = at(477.0, 146.0, 208.0, 121.0);
    crate::views::tuning::group_box(&p, gb, "Battery");
    let g = gb.min;
    let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(g + Vec2::new(x, y), Vec2::new(w, h));
    let rows: [(f32, &str, [&str; 2], (f64, f64), f64); 3] = [
        (18.0, "Low Battery", ["BATT_LOW_VOLT", "FS_BATT_VOLTAGE"], (0.0, 1000.0), 0.01),
        (43.0, "Reserved MAH", ["BATT_LOW_MAH", "FS_BATT_MAH"], (0.0, 100000.0), 1.0),
        (68.0, "Low Timer", ["BATT_LOW_TIMER", "BATT_LOW_TIMER"], (0.0, 120.0), 1.0),
    ];
    for (y, l, names, rng, inc) in rows {
        let has = { link.shared.read().params.keys().find(|k| names.contains(&k.as_str())).cloned() };
        if let Some(n) = has {
            p.text(g + Vec2::new(10.0, y + 3.0), egui::Align2::LEFT_TOP, l, f.clone(), Color32::WHITE);
            mav_num(ui, app, link, &n, r(120.0, y, 85.0, 19.0), rng, inc);
        }
    }
    for n in ["BATT_FS_LOW_ACT", "FS_BATT_ENABLE"] {
        if mav_combo(ui, link, n, r(6.0, 92.0, 199.0, 20.0)) {
            break;
        }
    }

    // groupBox2: Radio
    let gr = at(477.0, 273.0, 208.0, 96.0);
    crate::views::tuning::group_box(&p, gr, "Radio");
    let g = gr.min;
    let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(g + Vec2::new(x, y), Vec2::new(w, h));
    mav_combo(ui, link, "FS_THR_ENABLE", r(6.0, 13.0, 199.0, 20.0));
    let has_thr = { let s = link.shared.read(); s.params.contains_key("FS_THR_VALUE") || s.params.contains_key("THR_FS_VALUE") };
    if has_thr {
        p.text(g + Vec2::new(6.0, 38.0), egui::Align2::LEFT_TOP, "FS Pwm", f.clone(), Color32::WHITE);
        mav_num(ui, app, link, "FS_THR_VALUE", r(102.0, 35.0, 103.0, 19.0), (800.0, 1200.0), 1.0);
        mav_num(ui, app, link, "THR_FS_VALUE", r(102.0, 35.0, 103.0, 19.0), (800.0, 1200.0), 1.0);
    }
    mav_check(ui, app, link, "THR_FAILSAFE", r(6.0, 60.0, 110.0, 16.0), "Throttle FailSafe");
    mav_check(ui, app, link, "THR_FS_ACTION", r(6.0, 77.0, 146.0, 16.0), "Throttle Failsafe Action");

    // groupBox3: GCS
    let gg = at(477.0, 375.0, 208.0, 73.0);
    crate::views::tuning::group_box(&p, gg, "GCS");
    let g = gg.min;
    let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(g + Vec2::new(x, y), Vec2::new(w, h));
    mav_check(ui, app, link, "FS_GCS_ENABLE", r(6.0, 14.0, 103.0, 16.0), "GCS FS Enable");
    mav_check(ui, app, link, "FS_GCS_ENABL", r(6.0, 29.0, 93.0, 16.0), "GCS FailSafe");
    mav_check(ui, app, link, "FS_SHORT_ACTN", r(6.0, 43.0, 137.0, 16.0), "FailSafe Short (1 sec)");
    mav_check(ui, app, link, "FS_LONG_ACTN", r(6.0, 58.0, 140.0, 16.0), "FailSafe Long (20 sec)");
}

const ESC_STEPS: &str = "Remove Props!
After pushing this button:
-Disconnect USB and battery
-Plug in battery
-when LEDs flash, push Saftey Switch (if present)
-ESCs should beep as they are calibrated
- restart flight controller normally";

/// ConfigESCCalibration (AC3.3+): Calibrate ESCs button and the MOT_* output range.
fn esc_cal(ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
    let txt = |x: f32, y: f32, s: &str| {
        p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, s, f.clone(), Color32::WHITE);
    };
    p.text(o + Vec2::new(7.0, 5.0), egui::Align2::LEFT_TOP, "ESC Calibration (AC3.3+)", egui::FontId::proportional(16.0), Color32::WHITE);
    for y in [23.0, 147.0, 316.0] {
        p.line_segment([o + Vec2::new(3.0, y + 2.0), o + Vec2::new(592.0, y + 2.0)], egui::Stroke::new(1.0, Color32::from_rgb(0xd5, 0xdf, 0xe5)));
    }
    let id = ui.scope_id().with("esc_started");
    let started: bool = ui.ctx().data(|d| d.get_temp(id)).unwrap_or(false);
    let r = ui.scope_builder(egui::UiBuilder::new().max_rect(at(11.0, 42.0, 93.0, 25.0)), |ui| {
        ui.add_enabled_ui(!started, |ui| theme::button_sized(ui, "Calibrate ESCs", Vec2::new(93.0, 25.0))).inner
    });
    if r.inner.clicked() {
        let ok = link.shared.read().params.contains_key("ESC_CALIBRATION");
        if ok && !app.readonly {
            link.send(LinkCommand::SetParam { name: "ESC_CALIBRATION".into(), value: 3.0 });
            ui.ctx().data_mut(|d| d.insert_temp(id, true));
        } else {
            app.toasts.error("Set param error. Please ensure your version is AC3.3+.");
        }
    }
    let g = p.layout(ESC_STEPS.to_string(), f.clone(), Color32::WHITE, 239.0);
    p.galley(o + Vec2::new(110.0, 42.0), g, Color32::WHITE);
    txt(11.0, 165.0, "ESC Type:");
    mav_combo(ui, link, "MOT_PWM_TYPE", at(113.0, 158.0, 120.0, 21.0));
    let rows: [(f32, &str, &str, &str, (f64, f64), f64); 5] = [
        (186.0, "Output PWM Min", "MOT_PWM_MIN", "Leave as 0 to use RX input range", (0.0, 1500.0), 1.0),
        (212.0, "Output PWM Max", "MOT_PWM_MAX", "Leave as 0 to use RX input range", (0.0, 2200.0), 1.0),
        (238.0, "Spin when Armed", "MOT_SPIN_ARM", "speed when motors are armed but throttle is at zero (idle)", (0.0, 1.0), 0.01),
        (264.0, "Spin minimum", "MOT_SPIN_MIN", "minimum speed of motors while in flight (slightly higher than \"Spin when Armed\")", (0.0, 1.0), 0.01),
        (290.0, "Spin Maximum", "MOT_SPIN_MAX", "maximum speed of motors while in flight (almost all escs have a deadzone at the top)", (0.0, 1.0), 0.01),
    ];
    for (y, l, n, hint, rng, inc) in rows {
        txt(11.0, y + 7.0, l);
        mav_num(ui, app, link, n, at(113.0, y, 62.0, 20.0), rng, inc);
        txt(181.0, y + 7.0, hint);
    }
}

/// ConfigBatteryMonitoring: monitor/sensor/HW version presets and the voltage/current
/// calibration group.
fn battery_monitor(ui: &mut egui::Ui, app: &mut AppCtx, link: &Link, prefix: &str) {
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
    let txt = |x: f32, y: f32, s: &str| {
        p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, s, f.clone(), Color32::WHITE);
    };
    let (params, cs) = {
        let s = link.shared.read();
        (s.params.clone(), s.cs.clone())
    };
    let pv = |n: &str| params.get(&format!("{prefix}{n}")).map(|p| p.value);
    let readonly = app.readonly;
    let set = |n: &str, v: f32| {
        if !readonly {
            link.send(LinkCommand::SetParam { name: format!("{prefix}{n}"), value: v });
        }
    };
    // pictureBox5: battery icon
    let pic = at(3.0, 41.0, 97.0, 75.0);
    let body = Rect::from_min_size(pic.min + Vec2::new(14.0, 18.0), Vec2::new(64.0, 40.0));
    p.rect_filled(body, 4.0, Color32::from_gray(0x50));
    p.rect_stroke(body, 4.0, egui::Stroke::new(2.0, Color32::from_gray(0xc0)), egui::StrokeKind::Inside);
    p.rect_filled(Rect::from_min_size(body.right_top() + Vec2::new(0.0, 12.0), Vec2::new(6.0, 16.0)), 1.0, Color32::from_gray(0xc0));
    p.rect_filled(Rect::from_min_size(body.min + Vec2::new(5.0, 5.0), Vec2::new(40.0, 30.0)), 2.0, theme::green());

    txt(106.0, 45.0, "Monitor");
    let mon = pv("MONITOR").unwrap_or(0.0) as i32;
    let mons = [(0, "0: Disabled"), (3, "3: Battery Volts"), (4, "4: Voltage and Current")];
    ui.put(at(160.0, 41.0, 162.0, 21.0), |ui: &mut egui::Ui| {
        egui::ComboBox::from_id_salt("batmontype")
            .width(154.0)
            .selected_text(mons.iter().find(|m| m.0 == mon).map(|m| m.1.to_string()).unwrap_or_else(|| mon.to_string()))
            .show_ui(ui, |ui| {
                for (v, s) in mons {
                    if ui.selectable_label(v == mon, s).clicked() {
                        set("MONITOR", v as f32);
                    }
                }
            })
            .response
    });
    txt(330.0, 47.0, "Battery Capacity");
    mav_num(ui, app, link, &format!("{prefix}CAPACITY"), at(425.0, 45.0, 50.0, 19.0), (0.0, 1_000_000.0), 1.0);
    txt(481.0, 47.0, "mAh");
    let speech_id = ui.scope_id().with("speechbatt");
    let mut speech: bool = ui.ctx().data(|d| d.get_temp(speech_id)).unwrap_or(false);
    ui.put(at(333.0, 67.0, 151.0, 16.0), egui::Checkbox::new(&mut speech, "MP Alert on Low Battery"));
    ui.ctx().data_mut(|d| d.insert_temp(speech_id, speech));

    // sensor presets: (label, volt mult, amp per volt)
    let sensors: [(&str, f32, f32); 10] = [
        ("0: Other", 0.0, 0.0),
        ("1: AttoPilot 45A", 12.02, 17.0),
        ("2: AttoPilot 90A", 15.70, 27.32),
        ("3: AttoPilot 180A", 15.70, 54.64),
        ("4: 3DR Power Module", 10.1, 17.0),
        ("5: 3DR 4 in 1 ESC", 12.02, 17.0),
        ("6: 3DR HV Power Module APM", 12.02, 39.877),
        ("7: Cube HV Power Module", 12.02, 39.877),
        ("8: CUAV HV PM", 18.0, 24.0),
        ("9: Holybro Power Module", 18.18, 36.36),
    ];
    let (vm, apv) = (pv("VOLT_MULT").unwrap_or(0.0), pv("AMP_PERVLT").or(pv("AMP_PERVOL")).unwrap_or(0.0));
    let cur_sensor = sensors.iter().position(|s| (s.1 - vm).abs() < 0.01 && (s.2 - apv).abs() < 0.01).unwrap_or(0);
    txt(106.0, 71.0, "Sensor");
    ui.put(at(160.0, 68.0, 162.0, 20.0), |ui: &mut egui::Ui| {
        egui::ComboBox::from_id_salt("batmonsensor")
            .width(154.0)
            .selected_text(sensors[cur_sensor].0)
            .show_ui(ui, |ui| {
                for (i, s) in sensors.iter().enumerate().skip(1) {
                    if ui.selectable_label(i == cur_sensor, s.0).clicked() {
                        set("VOLT_MULT", s.1);
                        set("AMP_PERVLT", s.2);
                    }
                }
            })
            .response
    });
    // HW version: (label, volt pin, curr pin)
    let hw: [(&str, f32, f32); 11] = [
        ("0: CUAV V5/Pixhawk4 or APM1", 0.0, 1.0),
        ("1: APM2 - 2.5 non 3DR", 1.0, 2.0),
        ("2: APM2.5+/ZealotF427 - 3DR Power Module", 13.0, 12.0),
        ("3: PX4", 100.0, 101.0),
        ("4: The Cube or Pixhawk", 2.0, 3.0),
        ("5: VR Brain 4.5 - 5", 10.0, 11.0),
        ("6: VR Micro Brain 5", 10.0, -1.0),
        ("7: VR Brain 4", 6.0, 7.0),
        ("8: Cube Orange", 14.0, 15.0),
        ("9: Durandal/ZealotH743", 16.0, 17.0),
        ("10: Pixhawk 6C/Pix32 v6", 8.0, 4.0),
    ];
    let (vp, cp) = (pv("VOLT_PIN").unwrap_or(-2.0), pv("CURR_PIN").unwrap_or(-2.0));
    let cur_hw = hw.iter().position(|h| h.1 == vp && h.2 == cp);
    txt(106.0, 98.0, "HW Ver");
    ui.put(at(160.0, 95.0, 162.0, 20.0), |ui: &mut egui::Ui| {
        egui::ComboBox::from_id_salt("batmonhw")
            .width(154.0)
            .wrap_mode(egui::TextWrapMode::Truncate)
            .selected_text(cur_hw.map(|i| hw[i].0).unwrap_or(""))
            .show_ui(ui, |ui| {
                for (i, h) in hw.iter().enumerate() {
                    if ui.selectable_label(Some(i) == cur_hw, h.0).clicked() {
                        set("VOLT_PIN", h.1);
                        set("CURR_PIN", h.2);
                    }
                }
            })
            .response
    });

    // groupBox4: Calibration
    let gc = at(19.0, 142.0, 276.0, 164.0);
    crate::views::tuning::group_box(&p, gc, "Calibration");
    let g = gc.min;
    let r = |y: f32| Rect::from_min_size(g + Vec2::new(183.0, y), Vec2::new(76.0, 19.0));
    let gt = |x: f32, y: f32, s: &str| {
        p.text(g + Vec2::new(x, y), egui::Align2::LEFT_TOP, s, f.clone(), Color32::WHITE);
    };
    let mid = ui.scope_id().with("measvolt");
    let cid = ui.scope_id().with("meascurr");
    let mut meas_v: String = ui.ctx().data(|d| d.get_temp(mid)).unwrap_or_default();
    let mut meas_c: String = ui.ctx().data(|d| d.get_temp(cid)).unwrap_or_default();
    gt(5.0, 16.0, "1. Measured battery voltage:");
    if ui.put(r(13.0), egui::TextEdit::singleline(&mut meas_v)).lost_focus() {
        // new divider = measured / (calced / divider)
        if let Ok(m) = meas_v.trim().parse::<f32>() {
            if cs.battery_voltage > 0.0 && vm > 0.0 {
                set("VOLT_MULT", m * vm / cs.battery_voltage);
            }
        }
    }
    gt(5.0, 38.0, "2. Battery voltage (Calced):");
    let mut s = format!("{:.2}", cs.battery_voltage);
    ui.put(r(35.0), egui::TextEdit::singleline(&mut s).interactive(false));
    gt(5.0, 59.0, "3. Voltage divider (Calced):");
    let mut s = format!("{vm}");
    ui.put(r(56.0), egui::TextEdit::singleline(&mut s).interactive(false));
    gt(6.0, 80.0, "4. Measured current:");
    if ui.put(r(77.0), egui::TextEdit::singleline(&mut meas_c)).lost_focus() {
        if let Ok(m) = meas_c.trim().parse::<f32>() {
            if cs.current > 0.0 && apv > 0.0 {
                set("AMP_PERVLT", m * apv / cs.current);
            }
        }
    }
    gt(6.0, 101.0, "5. Current (Calced)");
    let mut s = format!("{:.2}", cs.current);
    ui.put(r(98.0), egui::TextEdit::singleline(&mut s).interactive(false));
    gt(6.0, 122.0, "6. Amperes per volt:");
    let mut s = format!("{apv}");
    ui.put(r(119.0), egui::TextEdit::singleline(&mut s).interactive(false));
    ui.ctx().data_mut(|d| {
        d.insert_temp(mid, meas_v);
        d.insert_temp(cid, meas_c);
    });
}

/// ConfigFrameClassType: Frame Class radio tiles and Frame Type pictures.
fn frame_type(ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
    let params = link.shared.read().params.clone();
    let (Some(fc), Some(ft)) = (params.get("FRAME_CLASS").map(|p| p.value as i32), params.get("FRAME_TYPE").map(|p| p.value as i32)) else {
        theme::label(ui, "Frame Class and Frame Type not available on this vehicle");
        return;
    };
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    let a = app.assets;
    let uv = Rect::from_min_max(egui::Pos2::ZERO, egui::Pos2::new(1.0, 1.0));
    let set = |name: &str, v: i32| {
        if !app.readonly {
            link.send(LinkCommand::SetParam { name: name.into(), value: v as f32 });
        }
    };
    // groupBox3: Frame Class
    let g3 = Rect::from_min_size(o + Vec2::new(3.0, 3.0), Vec2::new(652.0, 100.0));
    crate::views::tuning::group_box(&p, g3, "Frame Class");
    let classes: [(i32, &str, Option<egui::TextureId>); 8] = [
        (0, "Undefined", None),
        (1, "Quad", Some(a.fc_quad.id())),
        (2, "Hexa", Some(a.fc_hexa.id())),
        (3, "Octa", Some(a.fc_octa.id())),
        (4, "OctaQuad", Some(a.fc_octaquad.id())),
        (5, "Y6", Some(a.fc_y6.id())),
        (6, "Heli", Some(a.fc_heli.id())),
        (7, "Tri", Some(a.fc_tri.id())),
    ];
    for (i, (val, name, tex)) in classes.iter().enumerate() {
        let r = Rect::from_min_size(g3.min + Vec2::new(6.0 + i as f32 * 80.0, 18.0), Vec2::new(74.0, 74.0));
        let resp = ui.interact(r, ui.scope_id().with(("fc", i)), Sense::click());
        if fc == *val {
            p.rect_filled(r, 0.0, theme::green().gamma_multiply(0.35));
        }
        if let Some(t) = tex {
            p.image(*t, Rect::from_min_max(r.min + Vec2::new(8.0, 2.0), r.max - Vec2::new(8.0, 16.0)), uv, Color32::WHITE);
        }
        let mut chk = fc == *val;
        ui.put(Rect::from_min_size(Pos2::new(r.left(), r.bottom() - 15.0), Vec2::new(74.0, 15.0)), egui::RadioButton::new(chk, *name));
        if resp.clicked() {
            chk = true;
            set("FRAME_CLASS", *val);
        }
        let _ = chk;
    }
    // groupBox2: Frame Type (only types valid for the class are enabled)
    let valid: &[i32] = match fc {
        1 => &[0, 1, 2, 3, 4],
        2 => &[0, 1, 3],
        3 | 4 => &[0, 1, 2, 3],
        5 => &[1, 10],
        _ => &[],
    };
    let g2 = Rect::from_min_size(o + Vec2::new(3.0, 109.0), Vec2::new(504.0, 420.0));
    crate::views::tuning::group_box(&p, g2, "Frame Type");
    let types: [(i32, &str, (f32, f32), (f32, f32), (f32, f32, f32, f32), Option<egui::TextureId>); 6] = [
        (0, "'Plus'", (25.0, 38.0), (62.0, 38.0), (83.0, 9.0, 248.0, 75.0), Some(a.ft_plus.id())),
        (1, "'X' , 'Y6A'", (8.0, 118.0), (62.0, 118.0), (83.0, 88.0, 406.0, 78.0), Some(a.ft_x.id())),
        (2, "'V'", (38.0, 203.0), (62.0, 203.0), (83.0, 173.0, 111.0, 78.0), Some(a.ft_v.id())),
        (3, "'H'", (38.0, 287.0), (62.0, 287.0), (83.0, 257.0, 111.0, 78.0), Some(a.ft_h.id())),
        (10, "'Y6B'", (19.0, 371.0), (62.0, 371.0), (83.0, 341.0, 111.0, 77.0), Some(a.ft_y6b.id())),
        (4, "'V Tail'", (313.0, 203.0), (357.0, 203.0), (378.0, 173.0, 111.0, 78.0), None),
    ];
    for (val, name, lp, rp, (px, py, pw, ph), tex) in types {
        let ok = valid.contains(&val);
        p.text(g2.min + Vec2::new(lp.0, lp.1), egui::Align2::LEFT_TOP, name, f.clone(), if ok { Color32::WHITE } else { Color32::GRAY });
        let pr = Rect::from_min_size(g2.min + Vec2::new(px, py), Vec2::new(pw, ph));
        if let Some(t) = tex {
            // PictureBoxWithPseudoOpacity: disabled types are faded
            p.image(t, pr, uv, if ok { Color32::WHITE } else { Color32::from_white_alpha(60) });
        }
        let rr = Rect::from_min_size(g2.min + Vec2::new(rp.0, rp.1), Vec2::new(16.0, 14.0));
        let r = ui.add_enabled_ui(ok, |ui| ui.put(rr, egui::RadioButton::new(ft == val, ""))).inner;
        let pic = ui.interact(pr, ui.scope_id().with(("ft", val)), Sense::click());
        if ok && (r.clicked() || pic.clicked()) {
            set("FRAME_TYPE", val);
        }
    }
    p.text(g2.min + Vec2::new(200.0, 272.0), egui::Align2::LEFT_TOP, "NOTE: X and H are NOT interchangable\nthe prop rotation changes", f.clone(), Color32::WHITE);
    p.text(g2.min + Vec2::new(200.0, 356.0), egui::Align2::LEFT_TOP, "NOTE: This is the Y6B and prop rotation\nchanges from the old Y6A", f.clone(), Color32::WHITE);
    ui.allocate_rect(Rect::from_min_size(o, Vec2::new(787.0, 532.0)), Sense::hover());
}

/// ConfigHWCompass2: priority table, use-compass checkboxes, onboard mag calibration.
fn compass(ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
    let (params, progress, report, fw) = {
        let s = link.shared.read();
        (s.params.clone(), s.mag_progress, s.mag_report.clone(), s.cs.firmware)
    };
    let o = ui.max_rect().min;
    let p = ui.painter().clone();
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
    p.text(o + Vec2::new(3.0, 0.0), egui::Align2::LEFT_TOP, "Compass Priority", crate::fonts::bold_id(15.0), Color32::WHITE);
    p.line_segment([o + Vec2::new(0.0, 22.0), o + Vec2::new(ui.max_rect().width(), 22.0)], egui::Stroke::new(1.0, Color32::GRAY));
    p.text(o + Vec2::new(3.0, 33.0), egui::Align2::LEFT_TOP, "Set the Compass Priority by reordering the compasses in the table below (Highest at the top)", f.clone(), Color32::WHITE);
    // myDataGridView1
    let cols = [("Priority", 50.0), ("DevID", 50.0), ("BusType", 60.0), ("Bus", 50.0), ("Address", 50.0), ("DevType", 150.0), ("Missing", 50.0), ("External", 50.0), ("Orientation", 100.0), ("Up", 40.0), ("Down", 40.0)];
    let grid = r(3.0, 49.0, 672.0, 209.0);
    p.rect_filled(grid, 0.0, theme::bg());
    let mut x = grid.left() + 41.0; // row headers
    for (h, w) in cols {
        let c = Rect::from_min_size(Pos2::new(x, grid.top()), Vec2::new(w, 21.0));
        p.rect_stroke(c, 0.0, egui::Stroke::new(1.0, Color32::from_gray(160)), egui::StrokeKind::Inside);
        p.text(c.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, h, f.clone(), Color32::WHITE);
        x += w;
    }
    let id_names = |i: usize| -> [String; 3] {
        let n = if i == 0 { String::new() } else { (i + 1).to_string() };
        [format!("COMPASS_PRIO{}_ID", i + 1), format!("COMPASS_DEV_ID{n}"), format!("COMPASS_EXTERN{}", if i == 0 { "AL".to_string() } else { n.clone() })]
    };
    let orient_opts = mp_core::param_meta::options(fw, "COMPASS_ORIENT");
    let prio_vals: Vec<f32> = (0..3).map(|i| params.get(&format!("COMPASS_PRIO{}_ID", i + 1)).map(|p| p.value).unwrap_or(0.0)).collect();
    let mut swap: Option<(usize, usize)> = None;
    for i in 0..3 {
        let [prio, dev, ext] = id_names(i);
        let Some(raw) = params.get(&prio).or(params.get(&dev)).map(|p| p.value as u32) else { continue };
        if raw == 0 {
            continue;
        }
        let d = mp_core::devid::decode(raw);
        let y = grid.top() + 21.0 + i as f32 * 22.0;
        let ext_v = params.get(&ext).map(|p| p.value).unwrap_or(0.0);
        let orient_name = if i == 0 { "COMPASS_ORIENT".to_string() } else { format!("COMPASS_ORIENT{}", i + 1) };
        let orient = params.get(&orient_name).map(|p| p.value as i64).unwrap_or(0);
        let missing = !params.contains_key(&dev) || params.get(&dev).map(|p| p.value as u32) != Some(raw);
        let cells = [
            (i + 1).to_string(),
            raw.to_string(),
            d.bus_type.to_string(),
            d.bus.to_string(),
            d.address.to_string(),
            mp_core::devid::compass_devtype(d.devtype),
            if missing { "True".into() } else { "False".into() },
            if ext_v != 0.0 { "True".into() } else { "False".into() },
            orient_opts.iter().find(|(k, _)| *k == orient).map(|(_, s)| s.clone()).unwrap_or(orient.to_string()),
            String::new(),
            String::new(),
        ];
        let hdr = Rect::from_min_size(Pos2::new(grid.left(), y), Vec2::new(41.0, 22.0));
        p.rect_filled(hdr, 0.0, theme::bg());
        p.rect_stroke(hdr, 0.0, egui::Stroke::new(1.0, Color32::from_gray(160)), egui::StrokeKind::Inside);
        let mut x = grid.left() + 41.0;
        for (k, (_, w)) in cols.iter().enumerate() {
            let c = Rect::from_min_size(Pos2::new(x, y), Vec2::new(*w, 22.0));
            p.rect_filled(c, 0.0, if i % 2 == 0 { theme::control_bg() } else { theme::bg() });
            p.rect_stroke(c, 0.0, egui::Stroke::new(1.0, Color32::from_gray(160)), egui::StrokeKind::Inside);
            if k >= 9 {
                // Up / Down image buttons
                let tex = if k == 9 { app.assets.up.id() } else { app.assets.down.id() };
                let ir = Rect::from_center_size(c.center(), Vec2::new(16.0, 16.0));
                p.image(tex, ir, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
                if ui.interact(c, ui.scope_id().with(("cmp_mv", i, k)), Sense::click()).clicked() {
                    if k == 9 && i > 0 {
                        swap = Some((i, i - 1));
                    } else if k == 10 && i < 2 {
                        swap = Some((i, i + 1));
                    }
                }
            } else {
                p.with_clip_rect(c).text(c.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, &cells[k], f.clone(), Color32::WHITE);
            }
            x += w;
        }
    }
    // UpdateFirst3: rewrite COMPASS_PRIOn_ID in the new order
    if let Some((a, b)) = swap {
        if !app.readonly && prio_vals[a] != 0.0 && prio_vals[b] != 0.0 {
            link.send(LinkCommand::SetParam { name: format!("COMPASS_PRIO{}_ID", a + 1), value: prio_vals[b] });
            link.send(LinkCommand::SetParam { name: format!("COMPASS_PRIO{}_ID", b + 1), value: prio_vals[a] });
        }
    }
    let rm = r(560.0, 261.0, 115.0, 23.0);
    if ui.put(rm, |ui: &mut egui::Ui| theme::button_sized(ui, "Remove Missing", rm.size())).clicked() && !app.readonly {
        for i in 0..3 {
            let [prio, dev, _] = id_names(i);
            let pv = params.get(&prio).map(|p| p.value as u32).unwrap_or(0);
            let present = (0..3).any(|j| params.get(&id_names(j)[1]).map(|p| p.value as u32) == Some(pv));
            if pv != 0 && !present {
                link.send(LinkCommand::SetParam { name: prio, value: 0.0 });
            }
            let _ = dev;
        }
    }
    p.text(o + Vec2::new(0.0, 261.0), egui::Align2::LEFT_TOP, "Do you want to disable any of the first 3 compasses?", f.clone(), Color32::WHITE);
    for (k, (x, name)) in [(3.0, "COMPASS_USE"), (109.0, "COMPASS_USE2"), (215.0, "COMPASS_USE3")].iter().enumerate() {
        if let Some(pv) = params.get(*name) {
            let mut b = pv.value != 0.0;
            if ui.put(r(*x, 277.0, 105.0, 17.0), egui::Checkbox::new(&mut b, format!("Use Compass {}", k + 1))).changed() && !app.readonly {
                link.send(LinkCommand::SetParam { name: name.to_string(), value: if b { 1.0 } else { 0.0 } });
            }
        }
    }
    if let Some(pv) = params.get("COMPASS_LEARN") {
        let mut b = pv.value != 0.0;
        if ui.put(r(414.0, 277.0, 170.0, 17.0), egui::Checkbox::new(&mut b, "Automatically learn offsets")).changed() && !app.readonly {
            link.send(LinkCommand::SetParam { name: "COMPASS_LEARN".into(), value: if b { 1.0 } else { 0.0 } });
        }
    }
    p.text(o + Vec2::new(0.0, 297.0), egui::Align2::LEFT_TOP, "A reboot is required to adjust the ordering.", f.clone(), Color32::WHITE);
    let rb = r(3.0, 313.0, 75.0, 23.0);
    if ui.put(rb, |ui: &mut egui::Ui| theme::button_sized(ui, "Reboot", rb.size())).clicked() {
        link.send(LinkCommand::CommandLong { command: 246, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
    }
    p.text(o + Vec2::new(0.0, 339.0), egui::Align2::LEFT_TOP, "A mag calibration is required to remap the above changes.", f.clone(), Color32::WHITE);
    // groupBoxonboardcalib
    let g = r(3.0, 355.0, 600.0, 162.0);
    crate::views::tuning::group_box(&p, g, "Onboard Mag Calibration");
    let gr = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(g.min + Vec2::new(x, y), Vec2::new(w, h));
    for (x, t, cmd) in [(44.0, "Start", 42424u16), (125.0, "Accept", 42425), (206.0, "Cancel", 42426)] {
        let br = gr(x, 20.0, 75.0, 23.0);
        if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, t, br.size())).clicked() {
            if cmd == 42424 {
                let mut s = link.shared.write();
                s.mag_progress = [0; 3];
                s.mag_report = Default::default();
            }
            // DO_START_MAG_CAL: all compasses, retry, autosave
            let params = if cmd == 42424 { [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0] } else { [0.0; 7] };
            link.send(LinkCommand::CommandLong { command: cmd, params });
        }
    }
    for i in 0..3 {
        let y = 49.0 + i as f32 * 29.0;
        p.text(g.min + Vec2::new(6.0, y + 10.0), egui::Align2::LEFT_TOP, format!("Mag {}", i + 1), f.clone(), Color32::WHITE);
        let pr = gr(57.0, y, 258.0, 23.0);
        ui.put(pr, |ui: &mut egui::Ui| {
            theme::progress(ui, progress[i] as f32 / 100.0, pr.size(), "");
            ui.allocate_response(Vec2::ZERO, Sense::hover())
        });
    }
    let mut result = String::new();
    for (i, rep) in report.iter().enumerate() {
        if let Some((st, fit, ofs)) = rep {
            result.push_str(&format!("Mag {} {} Fitness {:.2}\nOffsets {:.0},{:.0},{:.0}\n", i + 1, st, fit, ofs[0], ofs[1], ofs[2]));
        }
    }
    p.text(g.min + Vec2::new(350.0, 20.0), egui::Align2::LEFT_TOP, result, f.clone(), Color32::WHITE);
    p.text(g.min + Vec2::new(7.0, 139.0), egui::Align2::LEFT_TOP, "Fitness", f.clone(), Color32::WHITE);
    if let Some(pv) = params.get("COMPASS_CAL_FIT") {
        let opts = [(4.0, "Strict"), (8.0, "Default"), (16.0, "Relaxed"), (32.0, "Very Relaxed")];
        let cur = opts.iter().find(|(v, _)| (*v - pv.value).abs() < 0.01).map(|(_, n)| n.to_string()).unwrap_or(pv.value.to_string());
        ui.put(gr(57.0, 135.0, 140.0, 21.0), |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("mavlinkComboBoxfitness")
                .width(132.0)
                .selected_text(cur)
                .show_ui(ui, |ui| {
                    for (v, n) in opts {
                        if ui.selectable_label(false, n).clicked() && !app.readonly {
                            link.send(LinkCommand::SetParam { name: "COMPASS_CAL_FIT".into(), value: v });
                        }
                    }
                })
                .response
        });
    }
    p.text(g.min + Vec2::new(203.0, 139.0), egui::Align2::LEFT_TOP, "Relax fitness if calibration fails", f.clone(), Color32::WHITE);
    let lb = r(3.0, 523.0, 120.0, 23.0);
    if ui.put(lb, |ui: &mut egui::Ui| theme::button_sized(ui, "Large Vehicle MagCal", lb.size())).clicked() {
        // MAV_CMD_FIXED_MAG_CAL_YAW with the current heading
        let yaw = link.shared.read().cs.yaw;
        link.send(LinkCommand::CommandLong { command: 42006, params: [yaw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
    }
    if progress.iter().any(|p| *p > 0 && *p < 100) {
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
    }
    ui.allocate_rect(r(0.0, 0.0, 680.0, 550.0), Sense::hover());
}

/// ConfigHWIDs: every *_ID / *_DEVID param decoded into bus/address/type.
fn hw_id(ui: &mut egui::Ui, link: &Link) {
    let params = link.shared.read().params.clone();
    let rows: Vec<(String, u32)> = params
        .iter()
        .filter(|(k, p)| (k.ends_with("_ID") || k.ends_with("DEVID") || k.contains("_ID") && k.starts_with("INS_")) && p.value != 0.0 && p.value.fract() == 0.0 && p.value > 255.0)
        .map(|(k, p)| (k.clone(), p.value as u32))
        .collect();
    egui::Grid::new("hwids").num_columns(6).striped(true).show(ui, |ui| {
        for h in ["Param", "DevID", "BusType", "Bus", "Address", "DevType"] {
            ui.label(RichText::new(h).strong().color(Color32::WHITE));
        }
        ui.end_row();
        for (n, raw) in rows {
            let d = mp_core::devid::decode(raw);
            theme::label(ui, n.as_str());
            theme::label(ui, raw.to_string());
            theme::label(ui, d.bus_type);
            theme::label(ui, d.bus.to_string());
            theme::label(ui, format!("0x{:02X}", d.address));
            theme::label(ui, mp_core::devid::devtype_for(&n, d.devtype));
            ui.end_row();
        }
    });
}

/// ConfigRadioOutput ("Servo Output"): # | Position | Reverse | Function | Min | Trim | Max.
fn servo_output(ui: &mut egui::Ui, app: &mut AppCtx) {
    let Some(link) = app.link else { return };
    let (cs, params, fw) = {
        let s = link.shared.read();
        (s.cs.clone(), s.params.clone(), s.cs.firmware)
    };
    let funcs = mp_core::param_meta::options(fw, "SERVO1_FUNCTION");
    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
    egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        let heads = [("#", 25.0), ("Position", 200.0), ("Reverse", 55.0), ("Function", 170.0), ("Min", 60.0), ("Trim", 60.0), ("Max", 60.0)];
        ui.horizontal(|ui| {
            for (h, w) in heads {
                ui.add_sized(Vec2::new(w, 18.0), egui::Label::new(RichText::new(h).strong().color(Color32::WHITE)));
            }
        });
        for ch in 1..=16usize {
            let pre = format!("SERVO{ch}_");
            if !params.contains_key(&format!("{pre}FUNCTION")) {
                continue;
            }
            ui.horizontal(|ui| {
                ui.add_sized(Vec2::new(25.0, 20.0), egui::Label::new(RichText::new(ch.to_string()).color(Color32::WHITE)));
                let pwm = cs.ch_out.get(ch - 1).copied().unwrap_or(0);
                let (br, _) = ui.allocate_exact_size(Vec2::new(200.0, 20.0), Sense::hover());
                let frac = ((pwm as f32 - 800.0) / 1400.0).clamp(0.0, 1.0);
                ui.painter().rect_filled(br, 0.0, theme::control_bg());
                ui.painter().rect_filled(Rect::from_min_size(br.min, Vec2::new(br.width() * frac, br.height())), 0.0, theme::green());
                ui.painter().text(br.center(), egui::Align2::CENTER_CENTER, pwm.to_string(), f.clone(), Color32::WHITE);
                let rev = format!("{pre}REVERSED");
                let mut b = params.get(&rev).map(|p| p.value != 0.0).unwrap_or(false);
                if ui.add_sized(Vec2::new(55.0, 20.0), egui::Checkbox::without_text(&mut b)).changed() && !app.readonly {
                    link.send(LinkCommand::SetParam { name: rev, value: if b { 1.0 } else { 0.0 } });
                }
                let fname = format!("{pre}FUNCTION");
                let cur = params.get(&fname).map(|p| p.value as i64).unwrap_or(0);
                let label = funcs.iter().find(|(k, _)| *k == cur).map(|(_, s)| s.clone()).unwrap_or(cur.to_string());
                egui::ComboBox::from_id_salt(("sfunc", ch)).width(162.0).wrap_mode(egui::TextWrapMode::Truncate).selected_text(label).show_ui(ui, |ui| {
                    for (k, s) in &funcs {
                        if ui.selectable_label(*k == cur, s).clicked() && !app.readonly {
                            link.send(LinkCommand::SetParam { name: fname.clone(), value: *k as f32 });
                        }
                    }
                });
                for suf in ["MIN", "TRIM", "MAX"] {
                    let n = format!("{pre}{suf}");
                    let v0 = params.get(&n).map(|p| p.value).unwrap_or(0.0);
                    let mut v = v0;
                    let r = ui.add_sized(Vec2::new(60.0, 20.0), egui::DragValue::new(&mut v).range(500.0..=2500.0));
                    if (r.drag_stopped() || r.lost_focus()) && v != v0 && !app.readonly {
                        link.send(LinkCommand::SetParam { name: n, value: v });
                    }
                }
            });
        }
    });
}

