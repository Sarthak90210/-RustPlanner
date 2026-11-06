//! ConfigSerialInjectGPS (SETUP > Optional Hardware > RTK/GPS Inject) on its designer layout:
//! source and baud, Connect, link status (data rates, messages seen), the RTCM base /
//! constellation lights and base position, receiver auto-configuration (u-blox survey-in,
//! Septentrio, Unicore UM982) and the saved base position list.

use std::sync::Arc;

use egui::{Color32, Rect, Sense, Vec2};
use mp_core::comms::ConnectionSpec;
use mp_core::rtk;
use parking_lot::Mutex;

use crate::app::AppCtx;
use crate::forms::FormState;
use crate::rtcm::{InjectOptions, Injector, Source};
use crate::theme;

const FORM: &str = "rtkinject";
const TYPES: [&str; 3] = ["UBlox M8P/F9P", "Septentrio", "Unicore UM982"];

#[derive(Default)]
pub struct RtkPage {
    st: FormState,
    init: bool,
    inject: Injector,
    gga: Arc<Mutex<Option<(f64, f64, f64)>>>,
    /// InputBox for the connection details or a base position name: (key, title, value)
    prompt: Option<(&'static str, &'static str, String)>,
}

/// Saved base positions (lat, lng, alt, name), Settings "SerialInjectGPS_BasePosList".
fn base_list(pl: &std::collections::BTreeMap<String, String>) -> Vec<(f64, f64, f64, String)> {
    pl.get("SerialInjectGPS_BasePosList")
        .map(|s| {
            s.split(';')
                .filter_map(|e| {
                    let v: Vec<&str> = e.splitn(4, ',').collect();
                    (v.len() == 4).then(|| (v[0].parse().ok(), v[1].parse().ok(), v[2].parse().ok(), v[3].to_string())).and_then(|(a, b, c, d)| Some((a?, b?, c?, d)))
                })
                .collect()
        })
        .unwrap_or_default()
}

fn save_base_list(pl: &mut std::collections::BTreeMap<String, String>, list: &[(f64, f64, f64, String)]) {
    let s: Vec<String> = list.iter().map(|(a, b, c, d)| format!("{a},{b},{c},{}", d.replace([';', ','], " "))).collect();
    pl.insert("SerialInjectGPS_BasePosList".into(), s.join(";"));
}

impl RtkPage {
    fn load(&mut self, app: &mut AppCtx) {
        let pl = &app.settings.planner;
        let st = &mut self.st;
        let mut ports = mp_core::comms::serial_ports();
        ports.extend(["UDP Host", "UDP Client", "TCP Client", "NTRIP"].map(String::from));
        st.items.insert("CMB_serialport".into(), ports);
        st.items.insert("CMB_baudrate".into(), ["4800", "9600", "14400", "19200", "28800", "38400", "57600", "115200", "230400", "460800", "921600"].map(String::from).to_vec());
        st.items.insert("comboBoxConfigType".into(), TYPES.map(String::from).to_vec());
        st.items.insert("cmb_septentriortcmamount".into(), ["Lite", "Basic", "Full"].map(String::from).to_vec());
        let get = |k: &str, d: &str| pl.get(k).cloned().unwrap_or_else(|| d.to_string());
        st.set("CMB_serialport", get("SerialInjectGPS_port", "NTRIP"));
        st.set("CMB_baudrate", get("SerialInjectGPS_baud", "115200"));
        st.set("comboBoxConfigType", get("SerialInjectGPS_AutoConfigType", TYPES[0]));
        st.set("txt_surveyinAcc", get("SerialInjectGPS_SIAcc", "2.00"));
        st.set("txt_surveyinDur", get("SerialInjectGPS_SIDur", "60"));
        st.set("cmb_septentriortcmamount", "Full");
        st.set("input_septentriortcminterval", "1.0");
        for (k, d) in [("chk_autoconfig", false), ("chk_m8p_130p", false), ("chk_sendgga", true), ("check_sendntripv1", false), ("chk_septentriogps", true), ("chk_septentrioglonass", true), ("chk_septentriobeidou", true), ("chk_septentriogalileo", true)] {
            let v = pl.get(&format!("SerialInjectGPS_{k}")).map(|v| v == "True").unwrap_or(d);
            st.checked.insert(k.into(), v);
        }
    }

    fn save(&self, app: &mut AppCtx) {
        let pl = &mut app.settings.planner;
        for (k, c) in [("SerialInjectGPS_port", "CMB_serialport"), ("SerialInjectGPS_baud", "CMB_baudrate"), ("SerialInjectGPS_AutoConfigType", "comboBoxConfigType"), ("SerialInjectGPS_SIAcc", "txt_surveyinAcc"), ("SerialInjectGPS_SIDur", "txt_surveyinDur")] {
            pl.insert(k.into(), self.st.get(c).to_string());
        }
        for (k, v) in &self.st.checked {
            pl.insert(format!("SerialInjectGPS_{k}"), if *v { "True" } else { "False" }.into());
        }
    }

    fn survey(&self) -> (u32, f64) {
        (self.st.get("txt_surveyinDur").trim().parse().unwrap_or(60), self.st.get("txt_surveyinAcc").trim().parse().unwrap_or(2.0))
    }

    /// The configuration the receiver gets on Connect when "Automatically Configure Receiver" is on.
    fn init_bytes(&self, app: &AppCtx) -> Vec<Vec<u8>> {
        let st = &self.st;
        let on = |k: &str| st.checked.get(k).copied().unwrap_or(false);
        let base = app.settings.planner.get("SerialInjectGPS_base_pos").and_then(|s| {
            let v: Vec<f64> = s.split(',').take(3).filter_map(|x| x.parse().ok()).collect();
            (v.len() == 3).then(|| (v[0], v[1], v[2]))
        });
        match st.get("comboBoxConfigType") {
            "Septentrio" => {
                let lvl = match st.get("cmb_septentriortcmamount") {
                    "Lite" => 3,
                    "Basic" => 4,
                    _ => 7,
                };
                let mut msgs = String::from("RTCM1006+RTCM1033+RTCM1230");
                for (k, n) in [("chk_septentriogps", "107"), ("chk_septentrioglonass", "108"), ("chk_septentriogalileo", "109"), ("chk_septentriobeidou", "112")] {
                    if on(k) {
                        msgs.push_str(&format!("+RTCM{n}{lvl}"));
                    }
                }
                let mut v = vec!["setPVTMode,Static,All,Auto\n".to_string(), "setDataInOut,USB1+USB2+COM1+COM2+COM3,Auto,RTCMv3\n".into(), format!("setRTCMv3Output,COM1+COM2+COM3+USB1+USB2,{msgs}\n")];
                v.push(self.septentrio_position());
                v.into_iter().map(String::into_bytes).collect()
            }
            "Unicore UM982" => ["GPGGA COM3 1\r\n", "mode base time 60 2 2.5\r\n", "rtcm1006 com3 1\r\n", "rtcm1033 com3 1\r\n", "rtcm1074 com3 1\r\n", "rtcm1124 com3 1\r\n", "rtcm1084 com3 1\r\n", "rtcm1094 com3 1\r\n", "saveconfig\r\n"].iter().map(|s| s.as_bytes().to_vec()).collect(),
            _ => {
                let mut v = rtk::setup_m8p(on("chk_m8p_130p"));
                let (dur, acc) = self.survey();
                v.push(match base {
                    Some((a, b, c)) => rtk::tmode3_fixed(a, b, c),
                    None => rtk::tmode3_survey(dur, acc),
                });
                v.push(rtk::ubx(0x06, 0x71, &[]));
                v
            }
        }
    }

    fn septentrio_position(&self) -> String {
        let st = &self.st;
        if st.checked.get("chk_septentriofixedposition").copied().unwrap_or(false) {
            let f = |k: &str| st.get(k).trim().parse::<f64>().unwrap_or(0.0);
            format!("setStaticPosGeodetic,Geodetic1,{:.9},{:.9},{:.4},WGS84\nsetPVTMode,Static,,Geodetic1\n", f("input_septentriofixedatitude"), f("input_septentriofixedlongitude"), f("input_septentriofixedaltitude"))
        } else {
            "setPVTMode,Static,,auto\n".into()
        }
    }

    fn connect(&mut self, app: &mut AppCtx) {
        if self.inject.running() {
            self.inject.stop();
            return;
        }
        let Some(link) = app.link else {
            app.toasts.error("Please Connect First");
            return;
        };
        let pl = &app.settings.planner;
        let port = self.st.get("CMB_serialport").to_string();
        let src = match port.as_str() {
            "NTRIP" => match pl.get("SerialInjectGPS_ntrip") {
                Some(u) => Source::Ntrip(u.clone()),
                None => {
                    self.prompt = Some(("SerialInjectGPS_ntrip", "Enter the NTRIP url (ntrip://user:pass@host:2101/mount)", "ntrip://".into()));
                    return;
                }
            },
            "UDP Host" => Source::Spec(ConnectionSpec::UdpListen { port: pl.get("SerialInjectGPS_udpport").and_then(|p| p.parse().ok()).unwrap_or(14550) }),
            "UDP Client" | "TCP Client" => {
                let key = if port == "TCP Client" { "SerialInjectGPS_tcp" } else { "SerialInjectGPS_udpcl" };
                match pl.get(key).and_then(|h| h.rsplit_once(':').and_then(|(a, b)| b.parse::<u16>().ok().map(|p| (a.to_string(), p)))) {
                    Some((host, p)) if port == "TCP Client" => Source::Spec(ConnectionSpec::Tcp { host, port: p }),
                    Some((host, p)) => Source::Spec(ConnectionSpec::UdpClient { host, port: p }),
                    None => {
                        self.prompt = Some((if port == "TCP Client" { "SerialInjectGPS_tcp" } else { "SerialInjectGPS_udpcl" }, "Enter host:port", "127.0.0.1:2101".into()));
                        return;
                    }
                }
            }
            p => Source::Spec(ConnectionSpec::Serial { port: p.to_string(), baud: self.st.get("CMB_baudrate").parse().unwrap_or(115200) }),
        };
        let serial = matches!(src, Source::Spec(ConnectionSpec::Serial { .. }));
        let init = if serial && self.st.checked.get("chk_autoconfig").copied().unwrap_or(false) { self.init_bytes(app) } else { Vec::new() };
        let gga = self.st.checked.get("chk_sendgga").copied().unwrap_or(false).then(|| self.gga.clone());
        self.inject.start(src, link.sender(), InjectOptions { init, gga });
        self.save(app);
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        if !self.init {
            self.init = true;
            self.load(app);
        }
        if let Some(l) = app.link {
            let s = l.shared.read();
            *self.gga.lock() = (s.cs.lat != 0.0).then_some((s.cs.lat, s.cs.lng, s.cs.altasl as f64));
        }
        let status = self.inject.status.lock().clone();
        let st = &mut self.st;
        let auto = st.checked.get("chk_autoconfig").copied().unwrap_or(false);
        let kind = st.get("comboBoxConfigType").to_string();
        st.visible.insert("comboBoxConfigType".into(), auto);
        st.visible.insert("groupBox_autoconfig".into(), auto);
        st.visible.insert("panel_ubloxoptions".into(), kind == TYPES[0]);
        st.visible.insert("panel_septentrio".into(), kind == TYPES[1]);
        st.visible.insert("panel_um982".into(), kind == TYPES[2]);
        st.set("BUT_connect", if status.running { "Disconnect" } else { "Connect" });
        st.set("lbl_status1", format!("{:>10} bps", status.bps));
        st.set("lbl_status2", format!("{:>10} bps sent", status.bps_sent));
        st.set("lbl_status3", status.base.map(|(a, b, c)| format!("{a:.9} {b:.9} {c:.4}")).unwrap_or_default());
        st.set("labelmsgseen", status.seen.iter().map(|(k, v)| format!("{k}={v} ")).collect::<String>());
        if let Some(e) = &status.error {
            st.set("labelmsgseen", e.clone());
        }
        // groupBox1 "Survey In": NAV-SVIN while surveying, TMODE3 when a fixed base is in use
        let svin_labels = ["lbl_svin", "label7", "label8", "label9", "label10"];
        for l in svin_labels {
            st.visible.insert(l.into(), status.svin.is_some() || status.tmode.map(|t| t & 0xff == 2).unwrap_or(false));
        }
        if let Some(sv) = status.svin {
            st.set("lbl_svin", if sv.valid { "Postion is valid" } else { "Position is invalid" });
            if sv.valid {
                let (a, b, c) = rtk::ecef_to_lla(sv.mean_ecef);
                st.set("label7", format!("Lat/X: {a}"));
                st.set("label8", format!("Lng/Y: {b}"));
                st.set("label9", format!("Alt/Z: {c}"));
            } else {
                st.set("label7", if sv.active { "In Progress" } else { "Complete" });
                st.set("label8", format!("Duration: {}", sv.dur));
                st.set("label9", format!("Observations: {}", sv.obs));
            }
            st.set("label10", format!("Current Acc: {}", sv.mean_acc));
        } else if let Some(t) = status.tmode {
            st.set("lbl_svin", format!("Using {}", match t { 0 => "Disabled", 1 => "SurveyIn", 2 => "FixedECEF", 258 => "FixedLLA", _ => "Unknown" }));
        }
        if status.running {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
        }

        let o = ui.max_rect().min;
        let clicked = crate::forms::show(ui, app, FORM, &mut self.st);

        // RTCM status lights: green while that message type keeps arriving
        let p = ui.painter().clone();
        for (ctl, light) in [("labelbase", "base"), ("labelgps", "gps"), ("labelglonass", "glonass"), ("label14BDS", "beidou"), ("labelGall", "galileo")] {
            if let Some((off, size)) = crate::forms::ctl_rect(FORM, ctl) {
                let r = Rect::from_min_size(o + off, size);
                p.rect_filled(r, 0.0, if status.light(light) { Color32::from_rgb(0, 128, 0) } else { Color32::RED });
            }
        }
        if let Some(sv) = status.svin {
            if let Some((off, size)) = crate::forms::ctl_rect(FORM, "lbl_svin") {
                let r = Rect::from_min_size(o + off, size);
                p.rect_filled(r, 0.0, if sv.valid { Color32::from_rgb(0, 128, 0) } else { Color32::RED });
                p.text(r.left_center() + Vec2::new(2.0, 0.0), egui::Align2::LEFT_CENTER, self.st.get("lbl_svin"), crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::WHITE);
            }
        }
        if auto && kind == TYPES[0] {
            self.base_grid(ui, app, o);
        }

        for b in clicked {
            match b.as_str() {
                "BUT_connect" => self.connect(app),
                "chk_autoconfig" | "chk_m8p_130p" | "chk_sendgga" | "check_sendntripv1" | "comboBoxConfigType" => self.save(app),
                "but_restartsvin" => {
                    // SetupBasePos(PointLatLngAlt.Zero): survey in again
                    app.settings.planner.remove("SerialInjectGPS_base_pos");
                    let (d, a) = self.survey();
                    self.inject.write(rtk::tmode3_survey(d, a));
                    self.inject.write(rtk::ubx(0x06, 0x71, &[]));
                }
                "but_save_basepos" => match status.base {
                    Some(_) => self.prompt = Some(("basepos_name", "Enter a friendly name for this location.", String::new())),
                    None => app.toasts.error("No valid base position determined by gps yet"),
                },
                "button_septentriosetposition" => self.inject.write(self.septentrio_position().into_bytes()),
                "button_septentriortcminterval" => {
                    let i: f64 = self.st.get("input_septentriortcminterval").trim().parse().unwrap_or(1.0);
                    self.inject.write(format!("setRTCMv3Interval,MSM3+MSM4+MSM7+RTCM1005|6+RTCM1033+RTCM1230,{i:.1}\n").into_bytes());
                }
                _ => {}
            }
        }
        self.prompt_ui(ui.ctx(), app, status.base);
    }

    /// dg_basepos: Lat, Long, Alt, BaseName, Use, Delete.
    fn base_grid(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, o: egui::Pos2) {
        let Some((off, size)) = crate::forms::ctl_rect(FORM, "dg_basepos") else { return };
        let area = Rect::from_min_size(o + off, size);
        let p = ui.painter().clone();
        p.rect_filled(area, 0.0, theme::control_bg());
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let cols = [("Lat", 110.0), ("Long", 110.0), ("Alt", 70.0), ("BaseName", 120.0), ("Use", 60.0), ("Delete", 60.0)];
        let row_h = 20.0;
        let mut x = area.left();
        for (h, w) in cols {
            let r = Rect::from_min_size(egui::pos2(x, area.top()), Vec2::new(w, row_h));
            p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);
            p.text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, h, f.clone(), Color32::WHITE);
            x += w;
        }
        let mut list = base_list(&app.settings.planner);
        let mut action = None;
        for (i, (la, ln, al, name)) in list.iter().enumerate() {
            let y = area.top() + row_h * (i + 1) as f32;
            if y + row_h > area.bottom() {
                break;
            }
            let mut x = area.left();
            for (k, (_, w)) in cols.iter().enumerate() {
                let r = Rect::from_min_size(egui::pos2(x, y), Vec2::new(*w, row_h));
                p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);
                match k {
                    0..=3 => {
                        let t = [la.to_string(), ln.to_string(), al.to_string(), name.clone()][k].clone();
                        p.with_clip_rect(r).text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, t, f.clone(), Color32::WHITE);
                    }
                    _ => {
                        let label = if k == 4 { "Use" } else { "Delete" };
                        let br = r.shrink(1.0);
                        if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, label, br.size())).clicked() {
                            action = Some((k, i));
                        }
                    }
                }
                x += w;
            }
        }
        ui.interact(area, ui.id().with("dg_basepos"), Sense::hover());
        match action {
            Some((4, i)) => {
                let (a, b, c, n) = list[i].clone();
                app.settings.planner.insert("SerialInjectGPS_base_pos".into(), format!("{a},{b},{c},{n}"));
                self.inject.write(rtk::tmode3_fixed(a, b, c));
                self.inject.write(rtk::ubx(0x06, 0x71, &[]));
            }
            Some((_, i)) => {
                list.remove(i);
                save_base_list(&mut app.settings.planner, &list);
            }
            None => {}
        }
    }

    fn prompt_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx, base: Option<(f64, f64, f64)>) {
        let Some((_, title, value)) = self.prompt.as_mut() else { return };
        let mut done = None;
        egui::Modal::new(egui::Id::new("rtk_prompt")).show(ctx, |ui| {
            theme::label(ui, *title);
            let r = ui.add(egui::TextEdit::singleline(value).desired_width(320.0));
            let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            ui.horizontal(|ui| {
                if theme::button(ui, "OK").clicked() || enter {
                    done = Some(true);
                }
                if theme::button(ui, "Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        let Some(ok) = done else { return };
        let (key, _, value) = self.prompt.take().unwrap_or(("", "", String::new()));
        if !ok {
            return;
        }
        if key == "basepos_name" {
            if let Some((a, b, c)) = base {
                let mut list = base_list(&app.settings.planner);
                list.push((a, b, c, value.clone()));
                save_base_list(&mut app.settings.planner, &list);
                app.settings.planner.insert("SerialInjectGPS_base_pos".into(), format!("{a},{b},{c},{value}"));
            }
            return;
        }
        app.settings.planner.insert(key.to_string(), value);
        self.connect(app);
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn base_list_round_trip() {
        let mut pl = std::collections::BTreeMap::new();
        super::save_base_list(&mut pl, &[(-35.1, 149.2, 584.5, "field;one".into())]);
        let l = super::base_list(&pl);
        assert_eq!(l.len(), 1);
        assert_eq!(l[0].3, "field one");
        assert_eq!(l[0].2, 584.5);
    }
}
