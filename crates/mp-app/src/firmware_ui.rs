//! Install Firmware (ConfigFirmwareManifest): vehicle tiles, manifest lookup for the board in
//! the bootloader, .apj download and upload via the px4 bootloader protocol.

use std::sync::Arc;
use std::time::Duration;

use egui::{Color32, Pos2, Rect, Sense, Vec2};
use mp_core::fwupload::{self, BoardInfo, FirmwareInfo};
use parking_lot::Mutex;

use crate::app::AppCtx;
use crate::{fonts, theme};

const MANIFEST_URL: &str = "https://firmware.ardupilot.org/manifest.json.gz";

#[derive(Default, Clone)]
pub struct FwStatus {
    pub busy: bool,
    pub text: String,
    pub progress: f32,
    pub board: Option<BoardInfo>,
    pub error: Option<String>,
    pub done: bool,
}

pub struct FirmwareInstaller {
    manifest: Arc<Mutex<Option<Result<Vec<FirmwareInfo>, String>>>>,
    loading: bool,
    pub status: Arc<Mutex<FwStatus>>,
    confirm: Option<(String, &'static str)>,
    beta: bool,
    choices: Option<Vec<FirmwareInfo>>,
}

impl Default for FirmwareInstaller {
    fn default() -> Self {
        Self {
            manifest: Arc::new(Mutex::new(None)),
            loading: false,
            status: Arc::new(Mutex::new(FwStatus { text: "Status".into(), ..Default::default() })),
            confirm: None,
            beta: false,
            choices: None,
        }
    }
}

fn agent() -> ureq::Agent {
    ureq::Agent::config_builder().timeout_global(Some(Duration::from_secs(120))).user_agent(concat!("MissionPlannerRS/", env!("CARGO_PKG_VERSION"))).build().into()
}

pub(crate) fn get(url: &str) -> Result<Vec<u8>, String> {
    let mut r = agent().get(url).call().map_err(|e| format!("{url}: {e}"))?;
    r.body_mut().with_config().limit(100 * 1024 * 1024).read_to_vec().map_err(|e| e.to_string())
}

impl FirmwareInstaller {
    fn ensure_manifest(&mut self, ctx: &egui::Context) {
        if self.loading {
            return;
        }
        self.loading = true;
        let m = self.manifest.clone();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let cache = crate::platform::data_dir().join("manifest.json.gz");
            let fresh = std::fs::metadata(&cache).and_then(|m| m.modified()).map(|t| t.elapsed().unwrap_or_default() < Duration::from_secs(86400)).unwrap_or(false);
            let gz = if fresh {
                std::fs::read(&cache).map_err(|e| e.to_string())
            } else {
                get(MANIFEST_URL).inspect(|b| {
                    let _ = std::fs::create_dir_all(cache.parent().unwrap());
                    let _ = std::fs::write(&cache, b);
                })
            };
            let r = gz.and_then(|g| fwupload::gunzip(&g)).and_then(|j| fwupload::parse_manifest(&j));
            *m.lock() = Some(r);
            ctx.request_repaint();
        });
    }

    fn label_for(&self, mav: &str) -> String {
        let m = self.manifest.lock();
        let rel = if self.beta { "BETA" } else { "OFFICIAL" };
        match m.as_ref() {
            Some(Ok(list)) => list
                .iter()
                .find(|f| fwupload::matches_type(f, mav) && f.version_type == rel)
                .map(|f| format!("{} {} {}", f.vehicletype, f.version_str.clone().or(f.version.clone()).unwrap_or_default(), f.version_type))
                .unwrap_or_default(),
            _ => String::new(),
        }
    }

    /// Download `url` (or use `local`), wait for the bootloader, flash.
    fn flash(&self, url: Option<String>, local: Option<Vec<u8>>, ctx: &egui::Context) {
        let st = self.status.clone();
        let ctx = ctx.clone();
        *st.lock() = FwStatus { busy: true, text: "Downloading from Internet".into(), ..Default::default() };
        std::thread::spawn(move || {
            let set = |t: &str, p: f32| {
                let mut s = st.lock();
                s.text = t.to_string();
                s.progress = p;
                ctx.request_repaint();
            };
            let res = (|| -> Result<(), String> {
                let bytes = match (local, url) {
                    (Some(b), _) => b,
                    (None, Some(u)) => get(&u)?,
                    _ => return Err("No firmware".into()),
                };
                let apj = fwupload::parse_apj(&String::from_utf8_lossy(&bytes))?;
                set("Please unplug the board, and then plug back in (looking for the bootloader)", 0.0);
                let board = fwupload::find_bootloader(Duration::from_secs(30)).ok_or("No board found in bootloader mode")?;
                st.lock().board = Some(board.clone());
                set(&format!("Found board type {} brdrev {} blrev {} fwmax {} on {}", board.board_id, board.board_rev, board.bl_rev, board.fw_maxsize, board.port), 0.0);
                let mut up = fwupload::Uploader::open(&board.port).map_err(|e| e.to_string())?;
                up.identify().map_err(|e| e.to_string())?;
                up.upload(&apj, |p, t| set(t, p)).map_err(|e| e.to_string())?;
                set("Upload Done", 1.0);
                Ok(())
            })();
            let mut s = st.lock();
            s.busy = false;
            match res {
                Ok(()) => s.done = true,
                Err(e) => {
                    s.text = e.clone();
                    s.error = Some(e);
                }
            }
            ctx.request_repaint();
        });
    }

    fn pick(&mut self, mav: &str, all: bool) -> Option<Vec<FirmwareInfo>> {
        let m = self.manifest.lock();
        let Some(Ok(list)) = m.as_ref() else { return None };
        let rel = if self.beta { "BETA" } else { "OFFICIAL" };
        let board = self.status.lock().board.as_ref().map(|b| b.board_id);
        let mut v: Vec<FirmwareInfo> = list
            .iter()
            .filter(|f| all || (fwupload::matches_type(f, mav) && f.version_type == rel))
            .filter(|f| board.map(|b| f.board_id == Some(b)).unwrap_or(true))
            .cloned()
            .collect();
        v.sort_by(|a, b| b.latest.cmp(&a.latest).then(a.platform.cmp(&b.platform)));
        Some(v)
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        self.ui_impl(ui, app, false)
    }

    /// "Install Firmware Legacy" (ConfigFirmware): the older tile layout with the
    /// licence text, firmware history and the same download / upload path.
    pub fn ui_legacy(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        self.ui_impl(ui, app, true)
    }

    fn ui_impl(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, legacy: bool) {
        self.ensure_manifest(ui.ctx());
        let o = ui.max_rect().min;
        let f = fonts::ui_id(fonts::UI_SIZE);
        let a = app.assets;
        let legacy_tiles: [(&str, (f32, f32), egui::TextureId, &'static str); 12] = [
            ("Rover", (30.0, 0.0), a.fw_rover.id(), "GROUND_ROVER"),
            ("Plane", (186.0, 0.0), a.fw_plane.id(), "FIXED_WING"),
            ("Quad", (342.0, 0.0), a.fw_quad.id(), "Copter"),
            ("Hexa", (498.0, 0.0), a.fw_hexa.id(), "Copter"),
            ("OctaQuad", (654.0, 0.0), a.fw_octaquad.id(), "Copter"),
            ("Sub", (810.0, 0.0), a.fw_sub.id(), "SUBMARINE"),
            ("Antenna Tracker", (30.0, 176.0), a.fw_tracker.id(), "ANTENNA_TRACKER"),
            ("Heli", (186.0, 176.0), a.fw_heli.id(), "HELICOPTER"),
            ("Tri", (342.0, 176.0), a.fw_tri.id(), "Copter"),
            ("Y6", (498.0, 176.0), a.fw_y6.id(), "Copter"),
            ("Octa", (654.0, 176.0), a.fw_octa.id(), "Copter"),
            ("Cube", (810.0, 176.0), a.fw_cube.id(), "Copter"),
        ];
        let manifest_tiles: [(&str, (f32, f32), egui::TextureId, &'static str); 12] = [
            ("Rover", (3.0, 3.0), a.fw_rover.id(), "GROUND_ROVER"),
            ("Plane", (159.0, 3.0), a.fw_plane.id(), "FIXED_WING"),
            ("Quad", (315.0, 3.0), a.fw_quad.id(), "Copter"),
            ("Hexa", (471.0, 3.0), a.fw_hexa.id(), "Copter"),
            ("OctaQuad", (627.0, 3.0), a.fw_octaquad.id(), "Copter"),
            ("Sub", (783.0, 3.0), a.fw_sub.id(), "SUBMARINE"),
            ("Antenna Tracker", (3.0, 159.0), a.fw_tracker.id(), "ANTENNA_TRACKER"),
            ("Heli", (159.0, 159.0), a.fw_heli.id(), "HELICOPTER"),
            ("Tri", (315.0, 159.0), a.fw_tri.id(), "Copter"),
            ("Y6", (471.0, 159.0), a.fw_y6.id(), "Copter"),
            ("Octa", (627.0, 159.0), a.fw_octa.id(), "Copter"),
            ("Cube", (783.0, 159.0), a.fw_cube.id(), "Copter"),
        ];
        let tiles = if legacy { legacy_tiles } else { manifest_tiles };
        let busy = self.status.lock().busy;
        for (name, (x, y), tex, mav) in tiles {
            let r = Rect::from_min_size(o + Vec2::new(x, y), Vec2::splat(150.0));
            let resp = ui.interact(r, ui.scope_id().with(("fw", name)), Sense::click());
            if resp.hovered() {
                ui.painter().rect_filled(r, 0.0, Color32::from_white_alpha(18));
            }
            // ImageLabel: picture scaled into the top, caption underneath
            let img_r = Rect::from_min_max(r.min + Vec2::new(10.0, 4.0), r.max - Vec2::new(10.0, 30.0));
            let size = ui.ctx().tex_manager().read().meta(tex).map(|m| m.size).unwrap_or([150, 150]);
            let scale = (img_r.width() / size[0] as f32).min(img_r.height() / size[1] as f32);
            let ir = Rect::from_center_size(img_r.center(), Vec2::new(size[0] as f32 * scale, size[1] as f32 * scale));
            ui.painter().image(tex, ir, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
            let label = self.label_for(mav);
            let mut job = egui::text::LayoutJob::simple(label.clone(), f.clone(), Color32::WHITE, 146.0);
            job.halign = egui::Align::Center;
            let g = ui.painter().layout_job(job);
            ui.painter().galley(Pos2::new(r.center().x, r.bottom() - 28.0), g, Color32::WHITE);
            if resp.clicked() && !busy {
                self.confirm = Some((if label.is_empty() { name.to_string() } else { label }, mav));
            }
        }
        // progress / status / links
        let st = self.status.lock().clone();
        if legacy {
            self.legacy_footer(ui, app, o, &st);
            self.dialogs(ui);
            return;
        }
        let pr = Rect::from_min_size(o + Vec2::new(3.0, 315.0), Vec2::new(940.0, 23.0));
        ui.put(pr, |ui: &mut egui::Ui| {
            theme::progress(ui, st.progress, pr.size(), "");
            ui.allocate_response(Vec2::ZERO, Sense::hover())
        });
        let manifest_state = match self.manifest.lock().as_ref() {
            None => " (loading firmware list...)".to_string(),
            Some(Err(e)) => format!(" (firmware list: {e})"),
            Some(Ok(_)) => String::new(),
        };
        let p = ui.painter().clone();
        p.with_clip_rect(Rect::from_min_size(o + Vec2::new(3.0, 341.0), Vec2::new(450.0, 34.0)))
            .text(o + Vec2::new(3.0, 341.0), egui::Align2::LEFT_TOP, format!("{}{manifest_state}", st.text), f.clone(), if st.error.is_some() { Color32::RED } else { Color32::WHITE });
        let link = |ui: &mut egui::Ui, x: f32, w: f32, t: &str| ui.put(Rect::from_min_size(o + Vec2::new(x, 341.0), Vec2::new(w, 13.0)), egui::Link::new(t)).clicked();
        if link(ui, 459.0, 57.0, "All Options") {
            self.choices = self.pick("", true);
        }
        if link(ui, 522.0, 96.0, "Bootloader Update") || link(ui, 624.0, 88.0, "Force Bootloader") {
            // ConfigFirmwareManifest: the bootloader is updated from the connected page
            app.toasts.error("Connect to the autopilot, then use Install Firmware > Bootloader Update.\nThe new bootloader is flashed by the running firmware (MAV_CMD_FLASH_BOOTLOADER).");
        }
        if link(ui, 736.0, 110.0, "Load custom firmware") {
            if let Some(path) = custom_file() {
                match std::fs::read(&path) {
                    Ok(b) => self.flash(None, Some(b), ui.ctx()),
                    Err(e) => app.toasts.error(e.to_string()),
                }
            }
        }
        if link(ui, 867.0, 76.0, if self.beta { "Stable firmwares" } else { "Beta firmwares" }) {
            self.beta = !self.beta;
        }
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(946.0, 380.0)), Sense::hover());
        self.dialogs(ui);
    }

    /// ConfigFirmware's lower half: hint, licence, links, history combo, progress, status.
    fn legacy_footer(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, o: Pos2, st: &FwStatus) {
        let f = fonts::ui_id(fonts::UI_SIZE);
        let p = ui.painter().clone();
        let at = |x: f32, y: f32| o + Vec2::new(x, y);
        p.text(at(397.0, 330.0), egui::Align2::LEFT_TOP, "Please click the images above for \"Flight versions\"", f.clone(), Color32::WHITE);
        if ui.put(Rect::from_min_size(at(348.0, 349.0), Vec2::new(383.0, 13.0)), egui::Link::new("Ardupilot is free software. Click here to read the licence")).clicked() {
            ui.ctx().open_url(egui::OpenUrl::new_tab("https://ardupilot.org/copter/docs/common-license-gpl.html"));
        }
        if ui.put(Rect::from_min_size(at(708.0, 0.0), Vec2::new(96.0, 13.0)), egui::Link::new("Copter motor setup")).clicked() {
            ui.ctx().open_url(egui::OpenUrl::new_tab("https://ardupilot.org/copter/docs/connect-escs-and-motors.html"));
        }
        let link = |ui: &mut egui::Ui, x: f32, y: f32, w: f32, t: &str| ui.put(Rect::from_min_size(at(x, y), Vec2::new(w, 13.0)), egui::Link::new(t)).clicked();
        if link(ui, 738.0, 373.0, 102.0, "Download firmwares") {
            ui.ctx().open_url(egui::OpenUrl::new_tab("https://firmware.ardupilot.org/"));
        }
        if link(ui, 862.0, 373.0, 110.0, "Load custom firmware") {
            if let Some(path) = custom_file() {
                match std::fs::read(&path) {
                    Ok(b) => self.flash(None, Some(b), ui.ctx()),
                    Err(e) => app.toasts.error(e.to_string()),
                }
            }
        }
        if link(ui, 630.0, 393.0, 76.0, if self.beta { "Stable firmwares" } else { "Beta firmwares" }) {
            self.beta = !self.beta;
        }
        if link(ui, 738.0, 393.0, 88.0, "Force Bootloader") {
            app.toasts.error("Connect to the autopilot, then use Install Firmware > Bootloader Update.");
        }
        // CMB_history_label -> every firmware in the manifest
        if link(ui, 862.0, 393.0, 113.0, "Pick previous firmware") {
            self.choices = self.pick("", true);
        }
        let pr = Rect::from_min_size(at(3.0, 417.0), Vec2::new(980.0, 23.0));
        ui.put(pr, |ui: &mut egui::Ui| {
            theme::progress(ui, st.progress, pr.size(), "");
            ui.allocate_response(Vec2::ZERO, Sense::hover())
        });
        p.text(at(3.0, 443.0), egui::Align2::LEFT_TOP, &st.text, f.clone(), if st.error.is_some() { Color32::RED } else { Color32::WHITE });
        p.text(at(862.0, 443.0), egui::Align2::LEFT_TOP, "Images by Max Levine", f, Color32::WHITE);
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(986.0, 462.0)), Sense::hover());
    }

    fn dialogs(&mut self, ui: &mut egui::Ui) {
        // CustomMessageBox: AreYouSureYouWantToUpload
        if let Some((label, mav)) = self.confirm.clone() {
            let mut close = false;
            egui::Modal::new(egui::Id::unique("fw_confirm")).show(ui.ctx(), |ui| {
                ui.label(egui::RichText::new("Continue").strong().color(Color32::WHITE));
                theme::label(ui, format!("Are you sure you want to upload {label}?"));
                ui.horizontal(|ui| {
                    if theme::button(ui, "Yes").clicked() {
                        close = true;
                        match self.pick(mav, false) {
                            Some(v) if v.len() == 1 => self.flash(Some(v[0].url.clone()), None, ui.ctx()),
                            Some(v) if !v.is_empty() => self.choices = Some(v),
                            Some(_) => self.status.lock().text = "No firmware available for this board".into(),
                            None => self.status.lock().text = "Firmware list not loaded yet".into(),
                        }
                    }
                    if theme::button(ui, "No").clicked() {
                        close = true;
                    }
                });
            });
            if close {
                self.confirm = None;
            }
        }
        // FirmwareSelectionForm
        if let Some(list) = self.choices.clone() {
            let mut open = true;
            let mut chosen = None;
            egui::Window::new("Firmware Selection").open(&mut open).default_size(Vec2::new(620.0, 420.0)).show(ui.ctx(), |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for f in list.iter().take(500) {
                        let t = format!("{} {} {} {} ({})", f.platform, f.vehicletype, f.version_str.clone().unwrap_or_default(), f.version_type, f.board_id.unwrap_or(0));
                        if ui.selectable_label(false, t).clicked() {
                            chosen = Some(f.url.clone());
                        }
                    }
                });
            });
            if let Some(u) = chosen {
                self.choices = None;
                self.flash(Some(u), None, ui.ctx());
            } else if !open {
                self.choices = None;
            }
        }
    }
}

#[cfg(not(any(target_os = "android", target_os = "ios")))]
fn custom_file() -> Option<std::path::PathBuf> {
    rfd::FileDialog::new().add_filter("Firmware", &["apj", "px4"]).pick_file()
}

#[cfg(any(target_os = "android", target_os = "ios"))]
fn custom_file() -> Option<std::path::PathBuf> {
    None
}
