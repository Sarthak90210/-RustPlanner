//! Setup pages added from MP's InitialSetup tree: Install Firmware while connected
//! (ConfigFirmwareDisabled), Loading (ConfigParamLoading), Secure (ConfigSecureAP),
//! Heli Setup (ConfigTradHeli4) and Antenna Tracker (Antenna.TrackerUI).

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use egui::{Color32, Rect, Vec2};
use mp_core::link::{Link, LinkCommand};
use mp_core::{param_meta, signedfw, tracker};
use parking_lot::Mutex;

use crate::app::AppCtx;
use crate::forms::FormState;
use crate::{fonts, theme};

#[derive(Default)]
pub struct ExtraPages {
    fwdisabled: FormState,
    bl_confirm: u8,
    loading: FormState,
    secure: FormState,
    key: Option<signedfw::SigningKey>,
    heli: FormState,
    trk: FormState,
    trk_init: bool,
    trk_run: Option<Arc<AtomicBool>>,
    /// latest (pan, tilt) target for the tracker thread
    trk_target: Arc<Mutex<(f64, f64)>>,
    trk_cfg: Arc<Mutex<tracker::TrackerConfig>>,
    trk_find: Option<Arc<Mutex<Option<String>>>>,
}

fn open_file(filter: &str, ext: &[&str]) -> Option<std::path::PathBuf> {
    crate::platform::pick_file("Select File", filter, ext)
}

fn save_file(name: &str) -> Option<std::path::PathBuf> {
    crate::platform::save_file_as("Save File", name, "*.pem", &["pem"])
}

/// "x.bin" -> "x-signed.bin" next to it.
fn signed_name(p: &std::path::Path) -> std::path::PathBuf {
    let stem = p.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
    let ext = p.extension().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
    p.with_file_name(format!("{stem}-signed.{ext}"))
}

pub fn got_all_params(link: &Link) -> bool {
    let s = link.shared.read();
    s.params_complete || (s.param_count > 0 && s.params.len() >= s.param_count as usize)
}

impl ExtraPages {
    /// ConfigFirmwareDisabled: no flashing over MAVLink; bootloader update via the firmware.
    pub fn fw_disabled(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let clicked = crate::forms::show(ui, app, "fwdisabled", &mut self.fwdisabled);
        if clicked.iter().any(|b| b == "but_bootloaderupdate") && app.link.is_some() {
            self.bl_confirm = 1;
        }
        if self.bl_confirm == 0 {
            return;
        }
        let text = if self.bl_confirm == 1 {
            "Are you sure you want to upgrade the bootloader? This can brick your board"
        } else {
            "Are you sure you want to upgrade the bootloader? This can brick your board, Please allow 5 mins for this process"
        };
        let mut answer = None;
        egui::Modal::new(egui::Id::new("bl_update")).show(ui.ctx(), |ui| {
            ui.label(egui::RichText::new("BL Update").strong().color(Color32::WHITE));
            theme::label(ui, text);
            ui.horizontal(|ui| {
                if theme::button(ui, "Yes").clicked() {
                    answer = Some(true);
                }
                if theme::button(ui, "No").clicked() {
                    answer = Some(false);
                }
            });
        });
        match answer {
            Some(true) if self.bl_confirm == 1 => self.bl_confirm = 2,
            Some(true) => {
                self.bl_confirm = 0;
                if let Some(l) = app.link {
                    // MAV_CMD_FLASH_BOOTLOADER, param5 = magic 290876
                    l.send(LinkCommand::CommandLong { command: 42650, params: [0.0, 0.0, 0.0, 0.0, 290876.0, 0.0, 0.0] });
                    app.toasts.error("Bootloader update sent. The result is reported as a COMMAND_ACK (see Messages).");
                }
            }
            Some(false) => self.bl_confirm = 0,
            None => {}
        }
    }

    /// ConfigParamLoading: shown until every parameter has arrived.
    pub fn param_loading(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        if let Some(l) = app.link {
            let s = l.shared.read();
            self.loading.set("label1", format!("Paramaters are still loading. Many screens will not work untill all Parameters are loaded. \n{} / {}", s.params.len(), s.param_count));
        }
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
        if crate::forms::show(ui, app, "paramloading", &mut self.loading).iter().any(|b| b == "but_forceparams") {
            if let Some(l) = app.link {
                l.send(LinkCommand::RequestParams);
            }
        }
    }

    /// ConfigSecureAP: Ed25519 key, signed bootloader, signed .apj.
    pub fn secure(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        for b in crate::forms::show(ui, app, "secureap", &mut self.secure) {
            match b.as_str() {
                "but_generatekey" => {
                    let k = signedfw::generate_key();
                    if let Some(p) = save_file("secure.pem") {
                        let s = p.to_string_lossy().to_string();
                        let r = std::fs::write(&p, signedfw::private_pem(&k))
                            .and_then(|_| std::fs::write(s.replace(".pem", "_private_key.dat"), signedfw::private_dat(&k)))
                            .and_then(|_| std::fs::write(s.replace(".pem", "_public_key.dat"), signedfw::public_dat(&k)));
                        match r {
                            Ok(()) => {
                                self.secure.set("txt_pubkey", signedfw::public_b64(&k));
                                self.key = Some(k);
                                app.toasts.error("Protect your private key, if lost there is no method to get it back.");
                            }
                            Err(e) => app.toasts.error(e.to_string()),
                        }
                    }
                }
                "but_privkey" => {
                    if let Some(p) = open_file("*.pem;*.dat", &["pem", "dat"]) {
                        match std::fs::read_to_string(&p).map_err(|e| e.to_string()).and_then(|t| signedfw::load_key(&t)) {
                            Ok(k) => {
                                self.secure.set("txt_pubkey", signedfw::public_b64(&k));
                                self.key = Some(k);
                            }
                            Err(e) => app.toasts.error(e),
                        }
                    }
                }
                "but_bootloader" | "but_firmware" => {
                    let Some(k) = self.key.clone() else {
                        app.toasts.error("Load or generate a private key first.");
                        continue;
                    };
                    let bl = b == "but_bootloader";
                    let Some(p) = (if bl { open_file("*.bin", &["bin"]) } else { open_file("*.apj", &["apj"]) }) else { continue };
                    self.secure.set(if bl { "txt_bl" } else { "txt_fwapj" }, p.display().to_string());
                    let r = if bl {
                        std::fs::read(&p).map_err(|e| e.to_string()).and_then(|d| signedfw::sign_bootloader(&k, &d))
                    } else {
                        std::fs::read_to_string(&p).map_err(|e| e.to_string()).and_then(|d| signedfw::sign_apj(&k, &d))
                    };
                    match r.and_then(|out| std::fs::write(signed_name(&p), out).map_err(|e| e.to_string())) {
                        Ok(()) => app.toasts.error(format!("Saved {}", signed_name(&p).display())),
                        Err(e) => app.toasts.error(e),
                    }
                }
                _ => {}
            }
        }
    }

    /// ConfigTradHeli4: servo table (designer) plus the four runtime parameter tables.
    pub fn heli(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, link: &Link) {
        let o = ui.max_rect().min;
        crate::forms::show_fixed(ui, app, "tradheli", &mut self.heli, 262.0);
        const SWASH: &[&str] = &["H_SV_MAN", "H_SW_TYPE", "H_SW_COL_DIR", "H_SW_LIN_SVO", "H_FLYBAR_MODE", "H_CYC_MAX", "H_COL_MAX", "H_COL_MID", "H_COL_MIN", "H_COL_ANG_MIN", "H_COL_ANG_MAX", "H_COL_ZERO_THRST", "H_COL_LAND_MIN"];
        const THROT: &[&str] = &["H_RSC_MODE", "H_RSC_CRITICAL", "H_RSC_RAMP_TIME", "H_RSC_RUNUP_TIME", "H_RSC_CLDWN_TIME", "H_RSC_SETPOINT", "H_RSC_IDLE", "H_RSC_THRCRV_0", "H_RSC_THRCRV_25", "H_RSC_THRCRV_50", "H_RSC_THRCRV_75", "H_RSC_THRCRV_100"];
        const GOV: &[&str] = &["H_RSC_GOV_COMP", "H_RSC_GOV_SETPNT", "H_RSC_GOV_DISGAG", "H_RSC_GOV_DROOP", "H_RSC_GOV_FF", "H_RSC_GOV_TCGAIN", "H_RSC_GOV_RANGE", "H_RSC_GOV_RPM", "H_RSC_GOV_TORQUE"];
        const MISC: &[&str] = &["IM_STB_COL_1", "IM_STB_COL_2", "IM_STB_COL_3", "IM_STB_COL_4", "H_TAIL_TYPE", "H_TAIL_SPEED", "H_GYR_GAIN", "H_GYR_GAIN_ACRO", "H_COLYAW"];
        let (params, fw) = {
            let s = link.shared.read();
            (s.params.clone(), s.cs.firmware)
        };
        let f = fonts::ui_id(fonts::UI_SIZE);
        // FlowLayoutPanel: groups left to right, wrapping at the page width
        let width = ui.max_rect().width().max(640.0);
        let (gw, row) = (300.0, 27.0);
        let (mut x, mut y, mut line_h) = (3.0, 265.0, 0.0f32);
        for (title, list) in [("Swashplate Setup", SWASH), ("Throttle Settings", THROT), ("Governor Settings", GOV), ("Misc Settings", MISC)] {
            let h = 16.0 + list.len() as f32 * row + 6.0;
            if x + gw > width {
                x = 3.0;
                y += line_h + 6.0;
                line_h = 0.0;
            }
            let g = Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(gw, h));
            crate::views::tuning::group_box(ui.painter(), g, title);
            for (i, n) in list.iter().enumerate() {
                let ry = g.top() + 16.0 + i as f32 * row;
                let meta = param_meta::get(fw, n);
                let name = meta.as_ref().map(|m| m.display_name.clone()).filter(|s| !s.is_empty()).unwrap_or_else(|| n.to_string());
                let unit = meta.as_ref().map(|m| m.units.clone()).unwrap_or_default();
                let label = if unit.is_empty() { name } else { format!("{name} ({unit})") };
                let lr = Rect::from_min_size(egui::pos2(g.left() + 6.0, ry + 4.0), Vec2::new(170.0, 18.0));
                ui.painter().with_clip_rect(lr).text(lr.min, egui::Align2::LEFT_TOP, &label, f.clone(), Color32::WHITE);
                let tip = meta.as_ref().map(|m| m.description.clone()).unwrap_or_default();
                ui.interact(lr, ui.id().with(("heli_lbl", *n)), egui::Sense::hover()).on_hover_text(if tip.is_empty() { n.to_string() } else { tip });
                let cr = Rect::from_min_size(egui::pos2(g.left() + 180.0, ry + 2.0), Vec2::new(gw - 190.0, 21.0));
                match params.get(*n) {
                    Some(p) => {
                        ui.scope_builder(egui::UiBuilder::new().max_rect(cr), |ui| crate::views::config::param_control(ui, link, fw, n, p.value, cr.width() - 8.0));
                    }
                    None => {
                        let mut s = String::new();
                        ui.put(cr, |ui: &mut egui::Ui| ui.add_enabled(false, egui::TextEdit::singleline(&mut s)));
                    }
                }
            }
            line_h = line_h.max(h);
            x += gw + 6.0;
        }
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(10.0, y + line_h + 10.0)), egui::Sense::hover());
    }

    /// Antenna.TrackerUI / TrackerGeneric.
    pub fn tracker(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let st = &mut self.trk;
        if !self.trk_init {
            self.trk_init = true;
            st.items.insert("CMB_interface".into(), tracker::Interface::ALL.iter().map(|s| s.to_string()).collect());
            st.items.insert("CMB_baudrate".into(), ["4800", "9600", "14400", "19200", "28800", "38400", "57600", "115200"].iter().map(|s| s.to_string()).collect());
            st.set("CMB_interface", "Maestro");
            st.set("CMB_baudrate", "57600");
            st.progress.insert("TRK_pantrim".into(), 0.5);
            st.progress.insert("TRK_tilttrim".into(), 0.5);
            st.set("LBL_pantrim", "0");
            st.set("LBL_tilttrim", "0");
            // saved Tracker_* settings
            for (k, v) in &app.settings.tracker_ui {
                if k.starts_with("CHK_") {
                    st.checked.insert(k.clone(), v == "True");
                } else {
                    st.set(k, v.clone());
                }
            }
        }
        st.items.insert("CMB_serialport".into(), mp_core::comms::serial_ports());
        let running = self.trk_run.is_some();
        st.set("BUT_connect", if running { "Disconnect" } else { "Connect" });
        let maestro = st.get("CMB_interface") == "Maestro";
        for c in ["TXT_panspeed", "TXT_panaccel", "TXT_tiltspeed", "TXT_tiltaccel"] {
            if maestro && !running {
                st.disabled.remove(c);
            } else {
                st.disabled.insert(c.to_string());
            }
        }
        for c in crate::forms::form("trackerui").controls.iter().filter(|c| c.name.starts_with("TXT_") || c.name.starts_with("CMB_")) {
            if running {
                st.disabled.insert(c.name.clone());
            } else if !c.name.ends_with("speed") && !c.name.ends_with("accel") {
                st.disabled.remove(&c.name);
            }
        }
        // trim track bars: -range/2 .. range/2 (pan is always 360)
        let tilt_half = (st.get("TXT_tiltrange").trim().parse::<i32>().unwrap_or(90) / 2).max(1) as f64;
        st.ranges.insert("TRK_pantrim".into(), (-180.0, 180.0));
        st.ranges.insert("TRK_tilttrim".into(), (-tilt_half, tilt_half));
        for (trk, lbl) in [("TRK_pantrim", "LBL_pantrim"), ("TRK_tilttrim", "LBL_tilttrim")] {
            if !st.text.contains_key(trk) {
                let v = st.get(lbl).to_string();
                st.set(trk, if v.is_empty() { "0".to_string() } else { v });
            }
        }
        let clicked = crate::forms::show(ui, app, "trackerui", st);
        for (trk, lbl) in [("TRK_pantrim", "LBL_pantrim"), ("TRK_tilttrim", "LBL_tilttrim")] {
            if clicked.iter().any(|c| c == trk) {
                let v = st.get(trk).to_string();
                st.set(lbl, v.clone());
                let v: f64 = v.parse().unwrap_or(0.0);
                let mut c = self.trk_cfg.lock();
                if trk == "TRK_pantrim" {
                    c.trim_pan = v;
                } else {
                    c.trim_tilt = v;
                }
            }
        }
        // AZToMAV / ELToMAV for the running thread
        if let (Some(_), Some(l)) = (&self.trk_run, app.link) {
            let s = l.shared.read();
            let home = s.cs.home.as_ref().map(|h| (h.lat, h.lng, h.alt as f64));
            let t = app.settings.tracker_home.map(|(a, b)| (a, b, home.map(|h| h.2).unwrap_or(0.0))).or(home);
            if let Some(t) = t {
                *self.trk_target.lock() = tracker::az_el(t, (s.cs.lat, s.cs.lng, s.cs.altasl as f64));
            }
        }
        let found = self.trk_find.as_ref().and_then(|f| f.lock().take());
        {
            if let Some(msg) = found {
                let a = format!("{:.0}", self.trk_cfg.lock().trim_pan);
                self.trk.set("LBL_pantrim", a.clone());
                self.trk.set("TRK_pantrim", a);
                app.toasts.error(msg);
                self.trk_find = None;
            }
        }
        for b in clicked {
            match b.as_str() {
                "BUT_connect" => self.tracker_connect(app),
                "BUT_find" => self.tracker_find(app),
                _ => {}
            }
        }
    }

    fn tracker_save(&self, app: &mut AppCtx) {
        let st = &self.trk;
        let mut out = std::collections::BTreeMap::new();
        for c in &crate::forms::form("trackerui").controls {
            match c.kind.as_str() {
                "text" | "combo" => {
                    out.insert(c.name.clone(), st.get(&c.name).to_string());
                }
                "check" => {
                    out.insert(c.name.clone(), if st.checked.get(&c.name).copied().unwrap_or(false) { "True" } else { "False" }.to_string());
                }
                _ => {}
            }
        }
        for l in ["LBL_pantrim", "LBL_tilttrim"] {
            out.insert(l.to_string(), st.get(l).to_string());
        }
        app.settings.tracker_ui = out;
    }

    fn tracker_connect(&mut self, app: &mut AppCtx) {
        self.tracker_save(app);
        if let Some(run) = self.trk_run.take() {
            run.store(false, Ordering::Relaxed);
            return;
        }
        let st = &self.trk;
        let n = |k: &str| st.get(k).trim().parse::<i32>();
        let cfg = (|| -> Result<tracker::TrackerConfig, std::num::ParseIntError> {
            Ok(tracker::TrackerConfig {
                interface: tracker::Interface::from_name(st.get("CMB_interface")),
                pan_range: n("TXT_panrange")?,
                tilt_range: n("TXT_tiltrange")?,
                trim_pan: st.get("LBL_pantrim").parse().unwrap_or(0.0),
                trim_tilt: st.get("LBL_tilttrim").parse().unwrap_or(0.0),
                pan_reverse: st.checked.get("CHK_revpan").copied().unwrap_or(false),
                tilt_reverse: st.checked.get("CHK_revtilt").copied().unwrap_or(false),
                pan_pwm_range: n("TXT_pwmrangepan")?,
                tilt_pwm_range: n("TXT_pwmrangetilt")?,
                pan_pwm_center: n("TXT_centerpan")?,
                tilt_pwm_center: n("TXT_centertilt")?,
                pan_speed: n("TXT_panspeed")?,
                pan_accel: n("TXT_panaccel")?,
                tilt_speed: n("TXT_tiltspeed")?,
                tilt_accel: n("TXT_tiltaccel")?,
            })
        })();
        let cfg = match cfg {
            Ok(c) => c,
            Err(e) => {
                app.toasts.error(format!("Invalid number entered\n{e}"));
                return;
            }
        };
        if let Err(e) = cfg.validate() {
            app.toasts.error(e);
            return;
        }
        let baud: u32 = st.get("CMB_baudrate").parse().unwrap_or(57600);
        let mut port = match mp_core::comms::open_serial_port(st.get("CMB_serialport"), baud) {
            Ok(p) => p,
            Err(e) => {
                app.toasts.error(format!("Error Connecting\n{e}"));
                return;
            }
        };
        if port.write_all(&cfg.setup_bytes()).and_then(|_| port.write_all(&cfg.pan_and_tilt(0.0, 0.0))).is_err() {
            app.toasts.error("Failed to set initial pan and tilt");
            return;
        }
        *self.trk_cfg.lock() = cfg;
        let run = Arc::new(AtomicBool::new(true));
        let (r, target, cfg) = (run.clone(), self.trk_target.clone(), self.trk_cfg.clone());
        std::thread::Builder::new()
            .name("Antenna Tracker".into())
            .spawn(move || {
                while r.load(Ordering::Relaxed) {
                    let (pan, tilt) = *target.lock();
                    let bytes = cfg.lock().pan_and_tilt(pan, tilt);
                    if port.write_all(&bytes).is_err() {
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(100));
                }
            })
            .ok();
        self.trk_run = Some(run);
    }

    /// BUT_find: sweep the pan trim for the best SiK local SNR (30, 5, then 1 degree steps).
    fn tracker_find(&mut self, app: &mut AppCtx) {
        let Some(l) = app.link else {
            app.toasts.error("No valid sik radio");
            return;
        };
        let snr = |s: &mp_core::link::LinkShared| s.cs.rssi as f32 - s.cs.noise as f32;
        if snr(&l.shared.read()) == 0.0 {
            app.toasts.error("No valid sik radio");
            return;
        }
        let done = Arc::new(Mutex::new(None));
        self.trk_find = Some(done.clone());
        let shared = l.shared.clone();
        let cfg = self.trk_cfg.clone();
        let range: f32 = self.trk.get("TXT_panrange").parse().unwrap_or(360.0);
        let start: f32 = self.trk.get("LBL_pantrim").parse().unwrap_or(0.0);
        std::thread::spawn(move || {
            let set = |v: f32| cfg.lock().trim_pan = v as f64;
            let check = |a: f32, b: f32, step: f32| {
                let (mut best, mut last) = (a, 0.0f32);
                set(a);
                std::thread::sleep(std::time::Duration::from_secs(4));
                let mut n = a;
                while n < b {
                    set(n);
                    std::thread::sleep(std::time::Duration::from_secs(2));
                    let s = snr(&shared.read());
                    if s > last {
                        best = n;
                        last = s;
                    }
                    n += step;
                }
                best
            };
            let a = check(start - range / 4.0, start + range / 4.0 - 1.0, 30.0);
            let a = check(a - 30.0, a + 30.0, 5.0);
            let a = check(a - 5.0, a + 5.0, 1.0);
            set(a);
            *done.lock() = Some(format!("Best pan trim {a:.0} degrees"));
        });
    }
}
