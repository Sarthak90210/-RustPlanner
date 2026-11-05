//! FlightData tabs: Transponder (uAvionix ADS-B OUT control), Aux Function (AuxOptions rows
//! sending DO_AUX_FUNCTION), Scripts (run a script file as a child process) and Payload
//! Control (gimbal tilt / pan / roll trackbars sending MOUNT_CONTROL). Layouts follow
//! FlightData.resx; each tab page is 290x175.

use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};

use egui::{Color32, Rect, Sense, Vec2};
use mp_core::link::{Link, LinkCommand};
use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, UavionixAdsbOutControlState, MOUNT_CONTROL_DATA, UAVIONIX_ADSB_OUT_CONTROL_DATA};

use crate::app::AppCtx;
use crate::views::tuning::group_box;
use crate::{fonts, theme};

const MODES: [&str; 4] = ["Mode A", "Mode C", "Mode S", "1090ES ADS-B OUT"];
const FAULTS: [&str; 5] = ["Maint. Req.", "GPS Unavail.", "GPS No Fix", "TX Sys. Fail.", "On Ground"];

pub struct ExtraTabs {
    squawk: f64,
    flight_id: String,
    nic: String,
    nacp: String,
    modes: [bool; 4],
    xpdr_state: usize,
    aux_fn: [i64; 7],
    tilt: f64,
    pan: f64,
    roll: f64,
    script: Option<std::path::PathBuf>,
    child: Option<Child>,
    output: Arc<Mutex<Vec<String>>>,
    redirect: bool,
}

impl Default for ExtraTabs {
    fn default() -> Self {
        Self {
            squawk: 1200.0,
            flight_id: String::new(),
            nic: String::new(),
            nacp: String::new(),
            modes: [false; 4],
            xpdr_state: 0,
            aux_fn: [0; 7],
            tilt: 0.0,
            pan: 0.0,
            roll: 0.0,
            script: None,
            child: None,
            output: Default::default(),
            redirect: true,
        }
    }
}

fn put_btn(ui: &mut egui::Ui, r: Rect, t: &str) -> bool {
    ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, t, r.size())).clicked()
}

impl ExtraTabs {
    fn send_xpdr(&self, link: &Link, ident: bool) {
        let mut state = 0u8;
        for (i, on) in self.modes.iter().enumerate() {
            if *on {
                state |= 16 << i;
            }
        }
        if ident {
            state |= 8;
        }
        let mut id = [b' '; 8];
        for (i, b) in self.flight_id.bytes().take(8).enumerate() {
            id[i] = b;
        }
        let mut d = UAVIONIX_ADSB_OUT_CONTROL_DATA::DEFAULT;
        d.baroAltMSL = i32::MAX;
        d.squawk = self.squawk as u16;
        d.state = UavionixAdsbOutControlState::from_bits_truncate(state);
        d.flight_id = id.into();
        link.send(LinkCommand::Send(Box::new(MavMessage::UAVIONIX_ADSB_OUT_CONTROL(d))));
    }

    /// tabTransponder
    pub fn transponder(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.available_rect_before_wrap().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let p = ui.painter().clone();
        let f = fonts::ui_id(fonts::UI_SIZE);
        let link = app.link;
        for (i, (y, h, t)) in [(6.0, 23.0, "STBY"), (35.0, 22.0, "ON"), (63.0, 23.0, "ALT")].into_iter().enumerate() {
            let rr = r(6.0, y, 55.0, h);
            let clicked = ui.put(rr, |ui: &mut egui::Ui| {
                let resp = theme::button_sized(ui, "", rr.size());
                let font = if self.xpdr_state == i { fonts::bold_id(fonts::UI_SIZE) } else { f.clone() };
                ui.painter().text(rr.center(), egui::Align2::CENTER_CENTER, t, font, theme::BUT_TEXT);
                resp
            });
            if clicked.clicked() {
                self.xpdr_state = i;
                self.modes = match i {
                    0 => [false; 4],
                    1 => [true, false, true, true],
                    _ => [true; 4],
                };
                if let Some(l) = link {
                    self.send_xpdr(l, false);
                }
            }
        }
        if put_btn(ui, r(6.0, 92.0, 55.0, 45.0), "IDENT") {
            if let Some(l) = link {
                self.send_xpdr(l, true);
            }
        }
        p.text(o + Vec2::new(64.0, 11.0), egui::Align2::LEFT_TOP, "FlightID", f.clone(), Color32::WHITE);
        ui.put(r(113.0, 6.0, 77.0, 20.0), egui::TextEdit::singleline(&mut self.flight_id).char_limit(8));
        p.text(o + Vec2::new(64.0, 37.0), egui::Align2::LEFT_TOP, "Squawk", f.clone(), Color32::WHITE);
        ui.put(r(113.0, 31.0, 77.0, 26.0), egui::DragValue::new(&mut self.squawk).range(0.0..=7777.0).speed(1.0));
        p.text(o + Vec2::new(64.0, 63.0), egui::Align2::LEFT_TOP, "NIC", f.clone(), Color32::WHITE);
        ui.put(r(113.0, 63.0, 77.0, 20.0), egui::TextEdit::singleline(&mut self.nic).interactive(false));
        p.text(o + Vec2::new(64.0, 89.0), egui::Align2::LEFT_TOP, "NACp", f.clone(), Color32::WHITE);
        ui.put(r(113.0, 89.0, 77.0, 20.0), egui::TextEdit::singleline(&mut self.nacp).interactive(false));
        // fault_clb: status from UAVIONIX_ADSB_OUT_STATUS
        let faults: u8 = link
            .and_then(|l| l.shared.read().inspector.values().find_map(|s| match &s.msg {
                MavMessage::UAVIONIX_ADSB_OUT_STATUS(d) => Some(d.fault.bits()),
                _ => None,
            }))
            .unwrap_or(0);
        let fr = r(196.0, 6.0, 87.0, 103.0);
        p.rect_filled(fr, 0.0, theme::CONTROL_BG);
        for (i, t) in FAULTS.iter().enumerate() {
            let mut on = faults & (1 << i) != 0;
            ui.put(Rect::from_min_size(fr.min + Vec2::new(2.0, 2.0 + i as f32 * 16.0), Vec2::new(85.0, 16.0)), egui::Checkbox::new(&mut on, *t));
        }
        if put_btn(ui, r(67.0, 114.0, 216.0, 23.0), "Connect to Transponder") {
            match link {
                Some(l) => self.send_xpdr(l, false),
                None => app.toasts.error("Please Connect First"),
            }
        }
        let mr = r(289.0, 6.0, 130.0, 131.0);
        p.rect_filled(mr, 0.0, theme::CONTROL_BG);
        for (i, t) in MODES.iter().enumerate() {
            let rr = Rect::from_min_size(mr.min + Vec2::new(2.0, 2.0 + i as f32 * 16.0), Vec2::new(126.0, 16.0));
            if ui.put(rr, egui::Checkbox::new(&mut self.modes[i], *t)).changed() {
                if let Some(l) = link {
                    self.send_xpdr(l, false);
                }
            }
        }
        ui.allocate_rect(r(0.0, 0.0, 420.0, 140.0), Sense::hover());
    }

    /// tabAuxFunction: seven AuxOptions rows (function combo + Low / Mid / High).
    pub fn aux(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.available_rect_before_wrap().min;
        let fw = app.link.map(|l| l.shared.read().cs.firmware).unwrap_or_default();
        let opts = mp_core::param_meta::options(fw, "RC1_OPTION");
        for row in 0..7 {
            let base = o + Vec2::new(3.0, 3.0 + row as f32 * 32.0);
            let cur = self.aux_fn[row];
            let name = opts.iter().find(|(k, _)| *k == cur).map(|(_, s)| s.clone()).unwrap_or_else(|| cur.to_string());
            ui.put(Rect::from_min_size(base + Vec2::new(1.0, 1.0), Vec2::new(121.0, 21.0)), |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt(("aux_fn", row))
                    .width(113.0)
                    .wrap_mode(egui::TextWrapMode::Truncate)
                    .selected_text(name)
                    .show_ui(ui, |ui| {
                        for (k, s) in &opts {
                            ui.selectable_value(&mut self.aux_fn[row], *k, s);
                        }
                    })
                    .response
            });
            for (x, w, t, pos) in [(128.0, 27.0, "Low", 0.0f32), (161.0, 28.0, "Mid", 1.0), (195.0, 31.0, "High", 2.0)] {
                if put_btn(ui, Rect::from_min_size(base + Vec2::new(x, 1.0), Vec2::new(w, 21.0)), t) {
                    match app.link {
                        Some(l) => l.send(LinkCommand::CommandLong { command: 218, params: [self.aux_fn[row] as f32, pos, 0.0, 0.0, 0.0, 0.0, 0.0] }),
                        None => app.toasts.error("Please Connect First"),
                    }
                }
            }
        }
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(284.0, 7.0 * 32.0)), Sense::hover());
    }

    /// tabScripts: select / run / abort / edit a script file; output goes to the Messages tab.
    pub fn scripts(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.available_rect_before_wrap().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let running = match self.child.as_mut().map(|c| c.try_wait()) {
            Some(Ok(None)) => true,
            Some(_) => {
                self.child = None;
                false
            }
            None => false,
        };
        // forward captured output to the Messages list
        let lines: Vec<String> = std::mem::take(&mut *self.output.lock().unwrap());
        if let Some(l) = app.link {
            if !lines.is_empty() {
                let mut s = l.shared.write();
                for t in lines {
                    s.messages.push(mp_core::state::StatusMessage { time: mp_core::link::now_hms(), severity: 6, text: t });
                }
            }
        }
        let p = ui.painter().clone();
        let f = fonts::ui_id(fonts::UI_SIZE);
        p.text(o + Vec2::new(19.0, 10.0), egui::Align2::LEFT_TOP, if running { "Script Status: Running" } else { "Script Status: No Script Running" }, f.clone(), Color32::WHITE);
        let sel = self.script.as_ref().and_then(|s| s.file_name()).map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "None".into());
        p.text(o + Vec2::new(19.0, 29.0), egui::Align2::LEFT_TOP, format!("Selected Script: {sel}"), f, Color32::WHITE);
        ui.put(r(19.0, 50.0, 140.0, 17.0), egui::Checkbox::new(&mut self.redirect, "Redirect Output"));
        if put_btn(ui, r(16.0, 80.0, 71.0, 23.0), "Select Script") {
            #[cfg(not(any(target_os = "android", target_os = "ios")))]
            {
                self.script = rfd::FileDialog::new().add_filter("Script", &["py", "lua", "sh", "bat"]).pick_file();
            }
        }
        if put_btn(ui, r(92.0, 80.0, 71.0, 23.0), "Run Script") {
            match &self.script {
                Some(path) if !running => {
                    let ext = path.extension().map(|e| e.to_string_lossy().to_lowercase()).unwrap_or_default();
                    let mut cmd = match ext.as_str() {
                        "py" => {
                            let mut c = Command::new(if cfg!(windows) { "python" } else { "python3" });
                            c.arg(path);
                            c
                        }
                        "bat" => {
                            let mut c = Command::new("cmd");
                            c.arg("/C").arg(path);
                            c
                        }
                        "sh" => {
                            let mut c = Command::new("sh");
                            c.arg(path);
                            c
                        }
                        _ => Command::new(path),
                    };
                    if self.redirect {
                        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());
                    }
                    match cmd.spawn() {
                        Ok(mut child) => {
                            for pipe in [child.stdout.take().map(|s| Box::new(s) as Box<dyn std::io::Read + Send>), child.stderr.take().map(|s| Box::new(s) as Box<dyn std::io::Read + Send>)].into_iter().flatten() {
                                let out = self.output.clone();
                                std::thread::spawn(move || {
                                    use std::io::BufRead;
                                    for line in std::io::BufReader::new(pipe).lines().map_while(Result::ok) {
                                        out.lock().unwrap().push(line);
                                    }
                                });
                            }
                            self.child = Some(child);
                        }
                        Err(e) => app.toasts.error(format!("Failed to run script\n{e}")),
                    }
                }
                Some(_) => {}
                None => app.toasts.error("Please select a script first"),
            }
        }
        if put_btn(ui, r(169.0, 80.0, 80.0, 23.0), "Abort Running Script") {
            if let Some(mut c) = self.child.take() {
                let _ = c.kill();
            }
        }
        if put_btn(ui, r(255.0, 79.0, 80.0, 23.0), "Edit Selected Script") {
            if let Some(path) = &self.script {
                let _ = if cfg!(windows) {
                    Command::new("notepad").arg(path).spawn()
                } else if cfg!(target_os = "macos") {
                    Command::new("open").arg("-t").arg(path).spawn()
                } else {
                    Command::new("xdg-open").arg(path).spawn()
                };
            }
        }
        if running {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
        }
        ui.allocate_rect(r(0.0, 0.0, 340.0, 110.0), Sense::hover());
    }

    fn send_mount(&self, link: &Link) {
        let (sys, comp) = {
            let s = link.shared.read();
            (s.cs.sysid, s.cs.compid)
        };
        link.send(LinkCommand::Send(Box::new(MavMessage::MOUNT_CONTROL(MOUNT_CONTROL_DATA {
            input_a: (self.tilt * 100.0) as i32,
            input_b: (self.roll * 100.0) as i32,
            input_c: (self.pan * 100.0) as i32,
            target_system: sys,
            target_component: comp,
            save_position: 0,
            ..Default::default()
        }))));
    }

    /// tabPayload: gimbal Tilt (vertical), Pan and Roll trackbars.
    pub fn payload(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let o = ui.available_rect_before_wrap().min;
        let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let p = ui.painter().clone();
        let mut changed = false;
        let gp = r(10.0, 6.0, 119.0, 124.0);
        group_box(&p, gp, "Tilt");
        let mut s = format!("{:.0}", self.tilt);
        ui.put(Rect::from_min_size(gp.min + Vec2::new(6.0, 18.0), Vec2::new(50.0, 20.0)), egui::TextEdit::singleline(&mut s).interactive(false));
        changed |= theme::vtrackbar(ui, Rect::from_min_size(gp.min + Vec2::new(62.0, 11.0), Vec2::new(45.0, 104.0)), &mut self.tilt, -90.0, 90.0).drag_stopped();
        for (title, gr, val, lo, hi) in [("Pan", r(144.0, 6.0, 178.0, 76.0), &mut self.pan, -180.0, 180.0), ("Roll", r(144.0, 82.0, 178.0, 76.0), &mut self.roll, -90.0, 90.0)] {
            group_box(&p, gr, title);
            let mut s = format!("{:.0}", *val);
            ui.put(Rect::from_min_size(gr.min + Vec2::new(6.0, 18.0), Vec2::new(51.0, 20.0)), egui::TextEdit::singleline(&mut s).interactive(false));
            let resp = theme::htrackbar(ui, Rect::from_min_size(gr.min + Vec2::new(63.0, 18.0), Vec2::new(104.0, 45.0)), val, lo, hi, 10);
            changed |= resp.drag_stopped() || resp.clicked();
        }
        if put_btn(ui, r(10.0, 134.0, 56.0, 23.0), "Reset Position") {
            self.tilt = 0.0;
            self.pan = 0.0;
            self.roll = 0.0;
            changed = true;
        }
        if put_btn(ui, r(73.0, 134.0, 56.0, 23.0), "Video Control") {
            app.toasts.error("No gimbal video stream is configured.\nSet one up under Planner > Video.");
        }
        if changed {
            match app.link {
                Some(l) => self.send_mount(l),
                None => app.toasts.error("Please Connect First"),
            }
        }
        ui.allocate_rect(r(0.0, 0.0, 330.0, 160.0), Sense::hover());
    }
}
