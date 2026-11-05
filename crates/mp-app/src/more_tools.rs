//! More temp-form (`temp.cs`) tools: Swarm / Follow the leader / Sequence Swarm / Follow Swarm
//! (FormationControl), QNH, adjust aircraft baro height, DEM, Manage Command List,
//! grab threads.txt, map logs, Shp to Poly, Inject GE, vlc, opticalflow calib, gimbal test,
//! and MAVSerial pass (TCP port 500 to the GPS over SERIAL_CONTROL).

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use egui::{Color32, Stroke, Vec2};
use mp_core::link::{Link, LinkCommand};
use mp_core::mavlink::dialects::ardupilotmega::*;

use crate::app::Toasts;
use crate::theme;

fn cmd_to(l: &Link, sys: u8, comp: u8, command: MavCmd, p: [f32; 7]) {
    l.send(LinkCommand::Send(Box::new(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: p[0],
        param2: p[1],
        param3: p[2],
        param4: p[3],
        param5: p[4],
        param6: p[5],
        param7: p[6],
        command,
        target_system: sys,
        target_component: comp,
        confirmation: 0,
    }))));
}

fn mode_to(l: &Link, sys: u8, mode: u32) {
    l.send(LinkCommand::Send(Box::new(MavMessage::SET_MODE(SET_MODE_DATA { custom_mode: mode, target_system: sys, base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED }))));
}

fn guided_to(l: &Link, sys: u8, comp: u8, lat: f64, lng: f64, alt: f32) {
    l.send(LinkCommand::Send(Box::new(MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
        lat_int: (lat * 1e7) as i32,
        lon_int: (lng * 1e7) as i32,
        alt,
        target_system: sys,
        target_component: comp,
        coordinate_frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask: PositionTargetTypemask::from_bits_truncate(0b0000_1111_1111_1000),
        ..Default::default()
    }))));
}

#[derive(Default)]
pub struct MoreTools {
    swarm_open: Option<&'static str>,
    leader: Option<(u8, u8)>,
    follow: bool,
    follow_last: Option<Instant>,
    takeoff_alt: String,
    spacing: String,
    qnh_open: bool,
    qnh: String,
    baro_prompt: Option<String>,
    text: Option<(String, String)>,
    cmds_open: bool,
    flow_open: bool,
    gimbal: bool,
    serial_pass: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
}

impl MoreTools {
    pub fn activate(&mut self, name: &str, link: Option<&Link>, toasts: &mut Toasts, settings: &mut crate::app::Settings) -> bool {
        match name {
            "Swarm" | "Follow the leader" | "Sequence Swarm" | "Follow Swarm" => {
                self.swarm_open = Some(match name {
                    "Swarm" => "Swarm",
                    "Follow the leader" => "Follow the leader",
                    "Sequence Swarm" => "Sequence Swarm",
                    _ => "Follow Swarm",
                });
                self.follow = name != "Swarm" && name != "Sequence Swarm";
                if self.takeoff_alt.is_empty() {
                    self.takeoff_alt = "10".into();
                    self.spacing = "10".into();
                }
            }
            "QNH" => {
                self.qnh_open = true;
                if self.qnh.is_empty() {
                    self.qnh = "1013.25".into();
                }
            }
            "adjust aircraft baro height" => self.baro_prompt = Some("0".into()),
            "DEM" => {
                let dir = crate::platform::data_dir().join("srtm");
                let mut lines: Vec<String> = std::fs::read_dir(&dir)
                    .map(|rd| rd.flatten().map(|e| format!("{}  {:.1} MB", e.file_name().to_string_lossy(), e.metadata().map(|m| m.len()).unwrap_or(0) as f64 / 1e6)).collect())
                    .unwrap_or_default();
                lines.sort();
                if lines.is_empty() {
                    lines.push("No DEM tiles loaded".into());
                }
                self.text = Some(("DEM".into(), format!("{}\n\n{}", dir.display(), lines.join("\n"))));
            }
            "Manage Command List" => self.cmds_open = true,
            "grab threads.txt" => match link {
                Some(l) => {
                    let mut ftp = mp_core::ftp::Ftp::new(l);
                    match ftp.get("@SYS/threads.txt", |_, _| {}) {
                        Ok(b) => {
                            let p = crate::platform::data_dir().join("threads.txt");
                            let _ = std::fs::write(&p, &b);
                            self.text = Some(("threads.txt".into(), String::from_utf8_lossy(&b).to_string()));
                        }
                        Err(e) => toasts.error(format!("Failed to get threads.txt\n{e}")),
                    }
                }
                None => toasts.error("Please connect first"),
            },
            "map logs" => self.map_logs(toasts),
            "Shp to Poly" => {
                #[cfg(not(any(target_os = "android", target_os = "ios")))]
                if let Some(p) = rfd::FileDialog::new().add_filter("Shape file", &["shp"]).pick_file() {
                    let shapes = std::fs::read(&p).map(|b| crate::views::flight_plan::menu::read_shp(&b)).unwrap_or_default();
                    let mut n = 0;
                    for (i, s) in shapes.iter().enumerate() {
                        let mut text = String::from("#saved by Mission Planner\r\n");
                        for (a, b) in s {
                            text.push_str(&format!("{a} {b}\r\n"));
                        }
                        let to = p.with_file_name(format!("{}-{i}.poly", p.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default()));
                        if std::fs::write(&to, text).is_ok() {
                            n += 1;
                        }
                    }
                    self.text = Some(("Shp to Poly".into(), format!("Wrote {n} polygon files next to {}", p.display())));
                }
            }
            "Inject GE" => {
                #[cfg(not(any(target_os = "android", target_os = "ios")))]
                if let Some(dir) = rfd::FileDialog::new().set_title("Inject GE").pick_folder() {
                    let counts = crate::map::inject_custom_tiles(&crate::platform::tile_cache_dir(), &dir);
                    let total: usize = counts.values().sum();
                    if total > 0 {
                        settings.fp_map.provider = crate::map::Provider::Custom;
                        settings.fd_map.provider = crate::map::Provider::Custom;
                    }
                    self.text = Some(("Inject GE".into(), format!("{total} tiles added to the Custom map")));
                }
            }
            "vlc" => {
                let url = settings.mjpeg_url.clone();
                if let Err(e) = std::process::Command::new("vlc").arg(&url).spawn() {
                    toasts.error(format!("Failed to start vlc\n{e}"));
                }
            }
            "opticalflow calib" => self.flow_open = true,
            "gimbal test" => self.gimbal = true,
            "MAVSerial pass" => match link {
                Some(l) => self.start_serial_pass(l, toasts),
                None => toasts.error("Please connect first"),
            },
            _ => return false,
        }
        true
    }

    fn map_logs(&mut self, toasts: &mut Toasts) {
        #[cfg(not(any(target_os = "android", target_os = "ios")))]
        {
            let Some(dir) = rfd::FileDialog::new().set_title("map logs").pick_folder() else { return };
            let mut n = 0;
            for f in crate::platform::list_files_in(&dir, &["tlog"]) {
                let Ok(recs) = mp_core::tlog::read(&f) else { continue };
                let pts: Vec<(f64, f64)> = recs
                    .iter()
                    .filter_map(|r| match &r.msg {
                        MavMessage::GLOBAL_POSITION_INT(g) if g.lat != 0 => Some((g.lat as f64 * 1e-7, g.lon as f64 * 1e-7)),
                        _ => None,
                    })
                    .collect();
                if pts.len() < 2 {
                    continue;
                }
                let (mut a0, mut a1, mut b0, mut b1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
                for (a, b) in &pts {
                    a0 = a0.min(*a);
                    a1 = a1.max(*a);
                    b0 = b0.min(*b);
                    b1 = b1.max(*b);
                }
                let (w, h) = (800u32, 800u32);
                let mut img = image::RgbImage::from_pixel(w, h, image::Rgb([0x26, 0x27, 0x28]));
                let span = (a1 - a0).max((b1 - b0) * a0.to_radians().cos()).max(1e-6);
                let px = |a: f64, b: f64| (((b - b0) * a0.to_radians().cos() / span * 760.0 + 20.0) as i32, ((a1 - a) / span * 760.0 + 20.0) as i32);
                for win in pts.windows(2) {
                    let (p0, p1) = (px(win[0].0, win[0].1), px(win[1].0, win[1].1));
                    let steps = (p1.0 - p0.0).abs().max((p1.1 - p0.1).abs()).max(1);
                    for k in 0..=steps {
                        let x = p0.0 + (p1.0 - p0.0) * k / steps;
                        let y = p0.1 + (p1.1 - p0.1) * k / steps;
                        if x >= 0 && y >= 0 && (x as u32) < w && (y as u32) < h {
                            img.put_pixel(x as u32, y as u32, image::Rgb([0xff, 0x00, 0xff]));
                        }
                    }
                }
                if img.save(f.with_extension("jpg")).is_ok() {
                    n += 1;
                }
            }
            self.text = Some(("map logs".into(), format!("Created {n} map images in {}", dir.display())));
        }
        let _ = toasts;
    }

    /// MAVSerial pass: TCP port 500 <-> SERIAL_CONTROL (GPS1, exclusive).
    fn start_serial_pass(&mut self, l: &Link, toasts: &mut Toasts) {
        if self.serial_pass.is_some() {
            toasts.error("MAVSerial pass is already running on TCP port 500");
            return;
        }
        let listener = match std::net::TcpListener::bind("0.0.0.0:500") {
            Ok(s) => s,
            Err(e) => {
                toasts.error(format!("Failed to listen on port 500\n{e}"));
                return;
            }
        };
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        self.serial_pass = Some(stop.clone());
        let (stx, srx) = crossbeam_channel::unbounded();
        l.send(LinkCommand::Subscribe { msgid: 126, tx: stx });
        let tx = l.sender();
        let (sys, comp) = {
            let s = l.shared.read();
            (s.cs.sysid, s.cs.compid)
        };
        std::thread::spawn(move || {
            let send = |data: &[u8]| {
                let mut d = [0u8; 70];
                d[..data.len()].copy_from_slice(data);
                let _ = tx.send(LinkCommand::Send(Box::new(MavMessage::SERIAL_CONTROL(SERIAL_CONTROL_DATA {
                    baudrate: 0,
                    timeout: 10,
                    device: SerialControlDev::SERIAL_CONTROL_DEV_GPS1,
                    flags: SerialControlFlag::SERIAL_CONTROL_FLAG_EXCLUSIVE | SerialControlFlag::SERIAL_CONTROL_FLAG_RESPOND | SerialControlFlag::SERIAL_CONTROL_FLAG_MULTI,
                    count: data.len() as u8,
                    data: d,
                    target_system: sys,
                    target_component: comp,
                    ..Default::default()
                }))));
            };
            let _ = listener.set_nonblocking(true);
            while !stop.load(std::sync::atomic::Ordering::Relaxed) {
                let Ok((mut sock, _)) = listener.accept() else {
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                };
                let _ = sock.set_read_timeout(Some(Duration::from_millis(20)));
                let mut buf = [0u8; 70];
                let mut last_poll = Instant::now();
                loop {
                    match sock.read(&mut buf) {
                        Ok(0) => break,
                        Ok(n) => send(&buf[..n]),
                        Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {}
                        Err(_) => break,
                    }
                    if last_poll.elapsed() > Duration::from_millis(100) {
                        last_poll = Instant::now();
                        send(&[]);
                    }
                    while let Ok(MavMessage::SERIAL_CONTROL(r)) = srx.try_recv() {
                        if sock.write_all(&r.data[..(r.count as usize).min(70)]).is_err() {
                            break;
                        }
                    }
                    if stop.load(std::sync::atomic::Ordering::Relaxed) {
                        break;
                    }
                }
            }
        });
        self.text = Some(("MAVSerial pass".into(), "Listening on TCP port 500 (GPS1 over SERIAL_CONTROL).\nConnect u-center or similar to this PC, port 500.".into()));
    }

    pub fn ui(&mut self, ctx: &egui::Context, link: Option<&Link>, toasts: &mut Toasts) {
        if let Some((t, body)) = &self.text {
            let mut open = true;
            egui::Window::new(t.as_str()).id(egui::Id::new("more_tools_text")).open(&mut open).collapsible(false).show(ctx, |ui| {
                egui::ScrollArea::vertical().max_height(420.0).show(ui, |ui| {
                    ui.label(egui::RichText::new(body.as_str()).monospace().color(Color32::WHITE));
                });
            });
            if !open {
                self.text = None;
            }
        }
        self.swarm_ui(ctx, link);
        self.qnh_ui(ctx, link);
        self.baro_ui(ctx, link, toasts);
        self.cmds_ui(ctx);
        self.flow_ui(ctx, link);
        self.gimbal_ui(ctx, link);
    }

    /// FormationControl: every vehicle on the link with group actions; Follow the leader
    /// keeps the others on a line behind the leader.
    fn swarm_ui(&mut self, ctx: &egui::Context, link: Option<&Link>) {
        let Some(title) = self.swarm_open else { return };
        let Some(l) = link else { return };
        let mavs: Vec<((u8, u8), mp_core::link::MavSummary)> = l.shared.read().mavs.iter().map(|(k, v)| (*k, v.clone())).collect();
        let mut open = true;
        egui::Window::new(title).open(&mut open).collapsible(false).default_size([520.0, 320.0]).show(ctx, |ui| {
            egui::Grid::new("swarm_grid").num_columns(8).striped(true).show(ui, |ui| {
                for h in ["Sysid", "Type", "Mode", "Armed", "Alt", "Leader", "", ""] {
                    theme::label(ui, h);
                }
                ui.end_row();
                for ((sys, comp), m) in &mavs {
                    theme::label(ui, format!("{sys}-{comp}"));
                    theme::label(ui, format!("{:?}", m.firmware));
                    let mode = m.firmware.modes().into_iter().find(|(n, _)| *n == m.custom_mode).map(|(_, s)| s).unwrap_or_else(|| m.custom_mode.to_string());
                    theme::label(ui, mode);
                    theme::label(ui, if m.armed { "Armed" } else { "Disarmed" });
                    theme::label(ui, format!("{:.1}", m.alt));
                    let mut is_leader = self.leader == Some((*sys, *comp));
                    if ui.radio(is_leader, "").clicked() {
                        is_leader = true;
                        self.leader = Some((*sys, *comp));
                    }
                    let _ = is_leader;
                    if theme::button(ui, "Arm").clicked() {
                        cmd_to(l, *sys, *comp, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                    }
                    if theme::button(ui, "Disarm").clicked() {
                        cmd_to(l, *sys, *comp, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [0.0; 7]);
                    }
                    ui.end_row();
                }
            });
            ui.separator();
            ui.horizontal(|ui| {
                theme::label(ui, "Takeoff alt");
                ui.add(egui::TextEdit::singleline(&mut self.takeoff_alt).desired_width(40.0));
                theme::label(ui, "Spacing (m)");
                ui.add(egui::TextEdit::singleline(&mut self.spacing).desired_width(40.0));
            });
            let alt: f32 = self.takeoff_alt.trim().parse().unwrap_or(10.0);
            ui.horizontal(|ui| {
                let all = |f: &dyn Fn(u8, u8, &mp_core::link::MavSummary)| {
                    for ((s, c), m) in &mavs {
                        f(*s, *c, m);
                    }
                };
                if theme::button(ui, "Arm All").clicked() {
                    all(&|s, c, _| cmd_to(l, s, c, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
                }
                if theme::button(ui, "Guided").clicked() {
                    all(&|s, _, m| {
                        if let Some((n, _)) = m.firmware.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case("guided")) {
                            mode_to(l, s, n);
                        }
                    });
                }
                if theme::button(ui, "Takeoff").clicked() {
                    all(&|s, c, _| cmd_to(l, s, c, MavCmd::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt]));
                }
                if theme::button(ui, "RTL").clicked() {
                    all(&|s, _, m| {
                        if let Some((n, _)) = m.firmware.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case("rtl")) {
                            mode_to(l, s, n);
                        }
                    });
                }
                if theme::button(ui, "Land").clicked() {
                    all(&|s, _, m| {
                        if let Some((n, _)) = m.firmware.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case("land")) {
                            mode_to(l, s, n);
                        }
                    });
                }
                ui.checkbox(&mut self.follow, "Follow the leader");
            });
            if self.follow && self.leader.is_none() {
                theme::label(ui, "Pick a leader with the radio button");
            }
        });
        if !open {
            self.swarm_open = None;
            self.follow = false;
        }
        // follower positions: a line behind the leader, `spacing` metres apart
        if self.follow && self.follow_last.map(|t| t.elapsed() > Duration::from_secs(1)).unwrap_or(true) {
            self.follow_last = Some(Instant::now());
            if let Some(lead) = self.leader {
                if let Some((_, lm)) = mavs.iter().find(|(k, _)| *k == lead) {
                    let sp: f64 = self.spacing.trim().parse().unwrap_or(10.0);
                    let back = (lm.heading as f64 + 180.0).to_radians();
                    for (k, ((s, c), _)) in mavs.iter().filter(|(k, _)| *k != lead).enumerate() {
                        let d = sp * (k + 1) as f64;
                        let lat = lm.lat + (d * back.cos() / 6_378_137.0).to_degrees();
                        let lng = lm.lng + (d * back.sin() / (6_378_137.0 * lm.lat.to_radians().cos())).to_degrees();
                        guided_to(l, *s, *c, lat, lng, lm.alt.max(5.0));
                    }
                }
            }
        }
    }

    fn qnh_ui(&mut self, ctx: &egui::Context, link: Option<&Link>) {
        if !self.qnh_open {
            return;
        }
        let press = link.and_then(|l| {
            l.shared.read().inspector.values().find_map(|m| match &m.msg {
                MavMessage::SCALED_PRESSURE(p) => Some(p.press_abs),
                _ => None,
            })
        });
        let mut open = true;
        egui::Window::new("QNH").open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
            theme::label(ui, "QNH (hPa)");
            ui.text_edit_singleline(&mut self.qnh);
            match (press, self.qnh.trim().parse::<f32>()) {
                (Some(p), Ok(q)) if q > 0.0 => {
                    let alt = 44330.0 * (1.0 - (p / q).powf(1.0 / 5.255));
                    theme::label(ui, format!("Pressure {p:.2} hPa\nAltitude AMSL (QNH) {alt:.1} m"));
                }
                (None, _) => {
                    theme::label(ui, "No SCALED_PRESSURE from the vehicle");
                }
                _ => {}
            }
        });
        if !open {
            self.qnh_open = false;
        }
    }

    /// modify the baro altitude reference: BARO_ALT_OFFSET so the current alt reads `h`.
    fn baro_ui(&mut self, ctx: &egui::Context, link: Option<&Link>, toasts: &mut Toasts) {
        let Some(v) = self.baro_prompt.as_mut() else { return };
        let mut done = None;
        egui::Window::new("adjust aircraft baro height").collapsible(false).resizable(false).show(ctx, |ui| {
            theme::label(ui, "Current height above home (m)");
            ui.text_edit_singleline(v);
            ui.horizontal(|ui| {
                if theme::button(ui, "OK").clicked() {
                    done = Some(true);
                }
                if theme::button(ui, "Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        let Some(ok) = done else { return };
        let v = self.baro_prompt.take().unwrap_or_default();
        if !ok {
            return;
        }
        match (link, v.trim().parse::<f32>()) {
            (Some(l), Ok(h)) => {
                let (alt, cur) = {
                    let s = l.shared.read();
                    (s.cs.alt, s.params.get("BARO_ALT_OFFSET").map(|p| p.value).unwrap_or(0.0))
                };
                l.send(LinkCommand::SetParam { name: "BARO_ALT_OFFSET".into(), value: cur + (h - alt) });
            }
            _ => toasts.error("Please connect first"),
        }
    }

    fn cmds_ui(&mut self, ctx: &egui::Context) {
        if !self.cmds_open {
            return;
        }
        let mut open = true;
        egui::Window::new("Manage Command List").open(&mut open).default_size([560.0, 420.0]).show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                egui::Grid::new("cmdlist").num_columns(8).striped(true).show(ui, |ui| {
                    for h in ["Command", "P1", "P2", "P3", "P4", "X", "Y", "Z"] {
                        theme::label(ui, h);
                    }
                    ui.end_row();
                    for c in mp_core::mavcmd::commands_for(mp_core::firmware::Firmware::ArduCopter2) {
                        theme::label(ui, &c.name);
                        for k in 0..7 {
                            theme::label(ui, c.labels[k].as_str());
                        }
                        ui.end_row();
                    }
                });
            });
        });
        if !open {
            self.cmds_open = false;
        }
    }

    fn flow_ui(&mut self, ctx: &egui::Context, link: Option<&Link>) {
        if !self.flow_open {
            return;
        }
        let flow = link.and_then(|l| {
            l.shared.read().inspector.values().find_map(|m| match &m.msg {
                MavMessage::OPTICAL_FLOW(f) => Some(f.clone()),
                _ => None,
            })
        });
        let mut open = true;
        egui::Window::new("opticalflow calib").open(&mut open).collapsible(false).show(ctx, |ui| match &flow {
            Some(f) => {
                let (r, _) = ui.allocate_exact_size(Vec2::splat(200.0), egui::Sense::hover());
                let p = ui.painter();
                p.rect_filled(r, 0.0, Color32::BLACK);
                let c = r.center();
                p.line_segment([c, c + Vec2::new(f.flow_x as f32, f.flow_y as f32) * 0.5], Stroke::new(2.0, Color32::GREEN));
                p.circle_stroke(c, 4.0, Stroke::new(1.0, Color32::WHITE));
                theme::label(ui, format!("flow x {} y {}  quality {}  ground {:.2} m", f.flow_x, f.flow_y, f.quality, f.ground_distance));
            }
            None => {
                theme::label(ui, "No OPTICAL_FLOW messages from the vehicle");
            }
        });
        if !open {
            self.flow_open = false;
        }
    }

    /// gimbal test: point the mount at home from the vehicle position (MOUNT_CONTROL).
    fn gimbal_ui(&mut self, ctx: &egui::Context, link: Option<&Link>) {
        if !self.gimbal {
            return;
        }
        let Some(l) = link else { return };
        let cs = l.shared.read().cs.clone();
        let mut open = true;
        egui::Window::new("gimbal test").open(&mut open).collapsible(false).show(ctx, |ui| match cs.home {
            Some(h) => {
                let dn = (h.lat - cs.lat).to_radians() * 6_378_137.0;
                let de = (h.lng - cs.lng).to_radians() * 6_378_137.0 * cs.lat.to_radians().cos();
                let dist = (dn * dn + de * de).sqrt();
                let bearing = de.atan2(dn).to_degrees() as f32;
                let pitch = (-(cs.alt as f64)).atan2(dist.max(0.1)).to_degrees() as f32;
                let yaw = (bearing - cs.yaw + 540.0).rem_euclid(360.0) - 180.0;
                theme::label(ui, format!("Target: home  dist {dist:.0} m\npitch {pitch:.1} deg  yaw {yaw:.1} deg (body)"));
                if theme::button(ui, "Point").clicked() {
                    l.send(LinkCommand::Send(Box::new(MavMessage::MOUNT_CONTROL(MOUNT_CONTROL_DATA { input_a: (pitch * 100.0) as i32, input_b: 0, input_c: (yaw * 100.0) as i32, target_system: cs.sysid, target_component: cs.compid, save_position: 0, ..Default::default() }))));
                }
            }
            None => {
                theme::label(ui, "Home is not set");
            }
        });
        if !open {
            self.gimbal = false;
        }
    }
}

