//! Georefimage form (DATA > DataFlash Logs > Geo Reference Images): pick the log and the
//! picture folder, Pre-process (time offset or CAM messages), then Location Kml and
//! GeoTag Images.

use mp_core::georef;

use crate::app::AppCtx;
use crate::forms::FormState;

#[derive(Default)]
pub struct GeorefWindow {
    pub open: bool,
    st: FormState,
    matches: Vec<georef::Match>,
    init: bool,
}

impl GeorefWindow {
    fn log(&mut self, s: impl Into<String>) {
        let mut t = self.st.get("TXT_outputlog").to_string();
        t.push_str(&s.into());
        t.push_str("\r\n");
        self.st.set("TXT_outputlog", t);
    }

    pub fn ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if !self.open {
            return;
        }
        if !self.init {
            self.init = true;
            self.st.checked.insert("RDIO_TimeOffset".into(), true);
            self.st.set("num_dropfromstart", "0");
            self.st.set("num_dropend", "0");
            self.st.set("num_minshutter", "0");
            self.st.set("num_vfov", "40");
            self.st.set("num_hfov", "60");
            self.st.set("num_camerarotation", "0");
        }
        let mut open = true;
        let mut clicked = Vec::new();
        egui::Window::new("Geo Ref Images").open(&mut open).collapsible(false).resizable(false).fixed_size([1085.0, 595.0]).show(ctx, |ui| {
            clicked = crate::forms::show(ui, app, "georef", &mut self.st);
        });
        if !open {
            self.open = false;
        }
        for b in clicked {
            // the three sync modes are radio buttons
            if b.starts_with("RDIO_") {
                for r in ["RDIO_TimeOffset", "RDIO_CAMMsgSynchro", "RDIO_trigmsg"] {
                    self.st.checked.insert(r.into(), r == b);
                }
                continue;
            }
            self.button(&b, app);
        }
    }

    fn button(&mut self, b: &str, app: &mut AppCtx) {
        match b {
            "BUT_browselog" => {
                #[cfg(not(any(target_os = "android", target_os = "ios")))]
                if let Some(p) = rfd::FileDialog::new().add_filter("Logs", &["tlog", "bin", "BIN"]).set_directory(crate::platform::data_dir().join("logs")).pick_file() {
                    self.st.set("TXT_logfile", p.display().to_string());
                }
            }
            "BUT_browsedir" => {
                #[cfg(not(any(target_os = "android", target_os = "ios")))]
                if let Some(p) = rfd::FileDialog::new().pick_folder() {
                    self.st.set("TXT_jpgdir", p.display().to_string());
                }
            }
            "BUT_estoffset" | "BUT_doit" => {
                let log = std::path::PathBuf::from(self.st.get("TXT_logfile"));
                let dir = std::path::PathBuf::from(self.st.get("TXT_jpgdir"));
                if !log.exists() {
                    app.toasts.error("Please select a log file");
                    return;
                }
                if !dir.is_dir() {
                    app.toasts.error("Please select a picture folder");
                    return;
                }
                let (fixes, cams) = match georef::read_log(&log) {
                    Ok(v) => v,
                    Err(e) => {
                        app.toasts.error(e);
                        return;
                    }
                };
                let photos = georef::photos(&dir);
                self.log(format!("Read {} positions, {} CAM messages, {} photos", fixes.len(), cams.len(), photos.len()));
                if b == "BUT_estoffset" {
                    match georef::estimate_offset(&photos, &fixes, &cams) {
                        Some(o) => {
                            self.st.set("TXT_offsetseconds", format!("{o:.0}"));
                            self.log(format!("Estimated offset {o:.0} s"));
                        }
                        None => self.log("Could not estimate the offset (no photo times or log positions)"),
                    }
                    return;
                }
                let n = |k: &str| self.st.get(k).trim().parse::<f64>().unwrap_or(0.0);
                let cam_mode = self.st.checked.get("RDIO_CAMMsgSynchro").copied().unwrap_or(false) || self.st.checked.get("RDIO_trigmsg").copied().unwrap_or(false);
                self.matches = if cam_mode {
                    georef::match_by_cam(&photos, &cams, n("num_dropfromstart") as usize, n("num_dropend") as usize)
                } else {
                    let lag = n("TXT_shutterLag") / 1000.0;
                    georef::match_by_offset(&photos, &fixes, n("TXT_offsetseconds") + lag)
                };
                let base = n("txt_basealt");
                if base != 0.0 {
                    for m in &mut self.matches {
                        m.fix.alt += base;
                    }
                }
                for m in self.matches.clone() {
                    self.log(format!("{} {:.7} {:.7} {:.1}", m.file.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default(), m.fix.lat, m.fix.lng, m.fix.alt));
                }
                self.log(format!("Matched {} of {} photos", self.matches.len(), photos.len()));
                // CreateReportFiles -> httpGeoRefKML: /georefnetwork.kml and the photos it links
                crate::httpserver::set_georef(georef::location_kml(&self.matches), std::path::PathBuf::from(self.st.get("TXT_jpgdir")));
            }
            "BUT_networklinkgeoref" => {
                if self.matches.is_empty() {
                    app.toasts.error("Pre-process first");
                    return;
                }
                let dir = std::path::PathBuf::from(self.st.get("TXT_jpgdir"));
                let r = std::fs::write(dir.join("location.txt"), georef::location_txt(&self.matches)).and_then(|_| std::fs::write(dir.join("location.kml"), georef::location_kml(&self.matches)));
                match r {
                    Ok(()) => self.log(format!("Wrote {} and location.txt", dir.join("location.kml").display())),
                    Err(e) => app.toasts.error(e.to_string()),
                }
            }
            "BUT_Geotagimages" => {
                if self.matches.is_empty() {
                    app.toasts.error("Pre-process first");
                    return;
                }
                let dir = std::path::PathBuf::from(self.st.get("TXT_jpgdir")).join("geotagged");
                let _ = std::fs::create_dir_all(&dir);
                let mut ok = 0;
                for m in self.matches.clone() {
                    let Ok(bytes) = std::fs::read(&m.file) else { continue };
                    if let Some(out) = georef::geotag(&bytes, m.fix.lat, m.fix.lng, m.fix.alt) {
                        if let Some(name) = m.file.file_name() {
                            if std::fs::write(dir.join(name), out).is_ok() {
                                ok += 1;
                            }
                        }
                    }
                }
                self.log(format!("GeoTagged {ok} images into {}", dir.display()));
            }
            _ => {}
        }
    }
}
