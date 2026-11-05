//! Developer/advanced tool windows: the MAVLink Inspector (`Controls/MAVLinkInspector.cs`)
//! and the Ctrl+F "temp" form (`temp.cs`).

use std::collections::BTreeSet;

use egui::{Color32, Rect, RichText, Vec2};
use mp_core::link::{Link, LinkCommand};

use crate::theme;

/// Split a `{:?}` dump of a MAVLink message into (field, value) pairs.
pub(crate) fn fields(msg: &mp_core::mavlink::dialects::ardupilotmega::MavMessage) -> Vec<(String, String)> {
    let dbg = format!("{msg:?}");
    let Some(a) = dbg.find("{ ") else { return Vec::new() };
    let inner = dbg[a + 2..].trim_end_matches(')').trim_end().trim_end_matches('}').trim_end();
    let mut out = Vec::new();
    let mut depth = 0i32;
    let mut cur = String::new();
    for c in inner.chars() {
        match c {
            '[' | '(' | '{' => depth += 1,
            ']' | ')' | '}' => depth -= 1,
            _ => {}
        }
        if c == ',' && depth == 0 {
            out.push(std::mem::take(&mut cur));
        } else {
            cur.push(c);
        }
    }
    if !cur.trim().is_empty() {
        out.push(cur);
    }
    out.into_iter()
        .filter_map(|f| {
            let (k, v) = f.split_once(':')?;
            Some((k.trim().to_string(), v.trim().to_string()))
        })
        .collect()
}

fn component_name(id: u8) -> String {
    match id {
        0 => "MAV_COMP_ID_ALL".into(),
        1 => "MAV_COMP_ID_AUTOPILOT1".into(),
        25..=86 => format!("MAV_COMP_ID_USER{}", id - 24),
        100..=105 => format!("MAV_COMP_ID_CAMERA{}", if id == 100 { String::new() } else { (id - 99).to_string() }),
        154 => "MAV_COMP_ID_GIMBAL".into(),
        190 => "MAV_COMP_ID_MISSIONPLANNER".into(),
        191 => "MAV_COMP_ID_ONBOARD_COMPUTER".into(),
        220 => "MAV_COMP_ID_GPS".into(),
        240 => "MAV_COMP_ID_UDP_BRIDGE".into(),
        241 => "MAV_COMP_ID_UART_BRIDGE".into(),
        250 => "MAV_COMP_ID_SYSTEM_CONTROL".into(),
        _ => id.to_string(),
    }
}

#[derive(Default)]
pub struct Inspector {
    pub open: bool,
    gcs_traffic: bool,
    expanded: BTreeSet<(u8, u8, u32)>,
    selected: Option<(u8, u8, u32, String)>,
    /// "Graph It" windows: (sysid, compid, msgid, field) and the sampled series
    graphs: Vec<((u8, u8, u32, String), std::collections::VecDeque<(f64, f64)>)>,
    start: Option<std::time::Instant>,
}

impl Inspector {
    pub fn new(open: bool) -> Self {
        Self { open, ..Default::default() }
    }

    pub fn show(&mut self, ctx: &egui::Context, link: Option<&Link>, toasts: &mut crate::app::Toasts) {
        if !self.open {
            return;
        }
        let mut open = true;
        egui::Window::new("Mavlink Inspector").open(&mut open).collapsible(false).default_size([526.0, 311.0]).show(ctx, |ui| {
            ui.horizontal(|ui| {
                if theme::button_sized(ui, "Graph It", Vec2::new(75.0, 23.0)).clicked() {
                    match &self.selected {
                        Some(sel) if !self.graphs.iter().any(|g| &g.0 == sel) => self.graphs.push((sel.clone(), Default::default())),
                        Some(_) => {}
                        None => toasts.error("Please select a field to graph"),
                    }
                }
                ui.checkbox(&mut self.gcs_traffic, "Show GCS Traffic");
            });
            let Some(link) = link else {
                theme::label(ui, "Not connected");
                return;
            };
            let rows: Vec<_> = {
                let s = link.shared.read();
                s.inspector.iter().filter(|(k, _)| self.gcs_traffic || k.0 != 255).map(|(k, v)| (*k, v.clone())).collect()
            };
            // sample graphed fields (numbers only)
            let t = self.start.get_or_insert_with(std::time::Instant::now).elapsed().as_secs_f64();
            for ((sys, comp, id, field), q) in &mut self.graphs {
                if let Some((_, st)) = rows.iter().find(|(k, _)| *k == (*sys, *comp, *id)) {
                    if let Some(v) = fields(&st.msg).into_iter().find(|(n, _)| n == field).and_then(|(_, v)| v.parse::<f64>().ok()) {
                        if q.back().map(|(t0, _)| t - t0 >= 0.05).unwrap_or(true) {
                            q.push_back((t, v));
                        }
                        while q.front().map(|(t0, _)| t - t0 > 20.0).unwrap_or(false) {
                            q.pop_front();
                        }
                    }
                }
            }
            let mono = egui::FontId::monospace(11.0);
            egui::Frame::new().fill(Color32::from_rgb(0x26, 0x27, 0x28)).stroke(egui::Stroke::new(1.0, Color32::from_gray(0x80))).show(ui, |ui| {
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                    let mut last_sys = None;
                    let mut last_comp = None;
                    for (k, st) in rows {
                        if last_sys != Some(k.0) {
                            ui.label(RichText::new(format!("System {}", k.0)).font(mono.clone()).color(Color32::WHITE));
                            last_sys = Some(k.0);
                            last_comp = None;
                        }
                        if last_comp != Some(k.1) {
                            ui.label(RichText::new(format!("    Comp {} {}", k.1, component_name(k.1))).font(mono.clone()).color(Color32::WHITE));
                            last_comp = Some(k.1);
                        }
                        let age = st.last.elapsed().as_secs_f32();
                        let rate = if age > 2.0 { 0.0 } else { st.rate };
                        let exp = self.expanded.contains(&k);
                        let head = format!("        {} {} ({:.1} Hz, #{}) {}Bps", if exp { "-" } else { "+" }, st.name, rate, k.2, (rate * 30.0).round());
                        if ui.add(egui::Label::new(RichText::new(head).font(mono.clone()).color(Color32::WHITE)).sense(egui::Sense::click())).clicked() {
                            if exp {
                                self.expanded.remove(&k);
                            } else {
                                self.expanded.insert(k);
                            }
                        }
                        if exp {
                            for (name, value) in fields(&st.msg) {
                                let sel = self.selected.as_ref().map(|s| s.0 == k.0 && s.1 == k.1 && s.2 == k.2 && s.3 == name).unwrap_or(false);
                                let text = format!("            {name:<32} {value:>20}");
                                let r = ui.add(egui::Label::new(RichText::new(text).font(mono.clone()).color(Color32::WHITE).background_color(if sel { Color32::from_rgb(0x33, 0x99, 0xff) } else { Color32::TRANSPARENT })).sense(egui::Sense::click()));
                                if r.clicked() {
                                    self.selected = Some((k.0, k.1, k.2, name.clone()));
                                }
                            }
                        }
                    }
                });
            });
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
        });
        if !open {
            self.open = false;
        }
        let t = self.start.map(|s| s.elapsed().as_secs_f64()).unwrap_or(0.0);
        let mut close = None;
        for (i, ((_, _, id, field), q)) in self.graphs.iter().enumerate() {
            let name = format!("{field}");
            let mut gopen = true;
            egui::Window::new(format!("Graph {id} {field}")).id(egui::Id::new(("inspgraph", i))).open(&mut gopen).default_size([500.0, 300.0]).show(ctx, |ui| {
                let size = ui.available_size().max(Vec2::new(300.0, 200.0));
                crate::zedgraph::plot(ui, size, &name, &[(name.as_str(), q)], t);
            });
            if !gopen {
                close = Some(i);
            }
        }
        if let Some(i) = close {
            self.graphs.remove(i);
        }
    }
}

/// temp.cs: (button, description) per row; left and right table halves.
const TEMP_LEFT: &[(&str, &str)] = &[
    ("Geo ref images", "moved to dataflash tab"),
    ("Warning Manager", "Create custom audio warnings"),
    ("Follow Me", "use a nmea gps to follow me"),
    ("NMEA", "outputs the mav location in nmea"),
    ("MicroDrone", "outputs the mav location in microdrone format"),
    ("Mavlink", "mirrors the mavlink stream received by mp"),
    ("Param gen", "regenerate the param info used inside mp"),
    ("Lang Edit", "translation language editor"),
    ("OSDVideo", "overlay the hud into your recorded videos"),
    ("Moving Base", "show an extra icon on the map of your current location."),
    ("Shp to Poly", "convert shp file to a polygon file"),
    ("Anon Log", ""),
    ("Swarm", "multi mav swarm interface"),
    ("Follow the leader", "follow the leader swarm"),
    ("MAVSerial pass", "create a exclusive passthrough to the gps (port 500)"),
    ("Start Remote df logger", ""),
    ("Sort TLogs", "Sort TLogs into their type and sysid directories"),
    ("rip all fw", "download all current fw's"),
    ("Inject GE", "add custom imagery to mp"),
    ("Clear Custom Maps", "wipe custom imagery"),
    ("structtest", "struct conversion speed test"),
    ("DashWare", "Create dashware date input file"),
    ("arm and takeoff", "quad: arm and takeoff"),
    ("gimbal test", "run the gimbal pointing algo"),
    ("map logs", "create map JPGs for all TLogs in a dir"),
    ("logindex", "tlog browser"),
    ("opticalflow calib", "display the image data from the px4 optical flow sensor"),
    ("APJ Tool", ""),
    ("mag calb log", "get mag offsets from a log"),
    ("CoT", "Outputs Cursor-on-Target"),
    ("Force Accel Cal", "Mark accel as cal'd after param restore"),
    ("Force Compass Cal", "Mark mag as cal'd after param restore"),
];

const TEMP_RIGHT: &[(&str, &str)] = &[
    ("hex Mavlink decode", ""),
    ("driver clean", "remove installed drivers"),
    ("Toggle Safety Switch", "virtual press the safety button"),
    ("Message Interval", "set custom message intervals for messages"),
    ("MAVLink Inspector", "Inspect all mavlink packets being transmitted"),
    ("Bootloader Upgrade", "update the bootloader"),
    ("3D Map", "3d map testing"),
    ("decode HWIDs", "display info about a hardware id typed in"),
    ("parse packet bytes", "debug a hex string mavlink packet"),
    ("adjust aircraft baro height", "modify baro alt reference alt"),
    ("Lockup MAV", "cause the autopilot to lockup"),
    ("DEM", "display information about the currently loaded DEMs"),
    ("logdownload scp", "logdownload via scp - ssh (apsync)"),
    ("ReSort All logs", "resort all the logs in the MP logging folder"),
    ("Custom GDAL", "load a custom map tile source via GDAL"),
    ("sitl streamcombiner", ""),
    ("Param Restore", "...."),
    ("FFT", "...."),
    ("grab threads.txt", "..."),
    ("reboot pixhawk", "reboot the autopilot"),
    ("QNH", "adjust the qnh"),
    ("Sequence Swarm", "label49"),
    ("vlc", "display video stream via vlc - USE Gstream instead"),
    ("Age Map Data", "remove image tiles older than 30 days"),
    ("Split DFLog", "split dflog into x pieces"),
    ("signing", "mavlink2 signing configuration"),
    ("extract gps_inject", "extract rtcm data from tlog"),
    ("Proximity", "display the proximity ui"),
    ("Follow Swarm", "swarm style"),
    ("Manage Command List", "Manage Planner's Command List"),
    ("DFU Mode", "DFU Mode"),
];

/// What a temp-form button asks the app to do.
pub enum TempAction {
    Inspector,
    HwIds,
    LogBrowse,
    Unavailable(&'static str),
}

/// temp form (Ctrl+F): 1044x628 client area, TableLayoutPanel of 19px rows at (12,12).
pub fn temp_window(ctx: &egui::Context, open: &mut bool, link: Option<&Link>, toasts: &mut crate::app::Toasts) -> Option<TempAction> {
    if !*open {
        return None;
    }
    let mut act = None;
    let frame = egui::Frame::window(&ctx.global_style()).fill(theme::bg());
    egui::Window::new("temp").open(open).collapsible(false).resizable(false).frame(frame).fixed_size([1044.0, 628.0]).show(ctx, |ui| {
        let (full, _) = ui.allocate_exact_size(Vec2::new(1044.0, 628.0), egui::Sense::hover());
        let o = full.min + Vec2::new(12.0, 12.0);
        let p = ui.painter().clone();
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        for (col, list) in [(0.0f32, TEMP_LEFT), (400.0, TEMP_RIGHT)] {
            let bw = if col == 0.0 { 125.0 } else { 148.0 };
            for (i, (b, l)) in list.iter().enumerate() {
                let y = 3.0 + i as f32 * 19.0;
                let r = Rect::from_min_size(o + Vec2::new(col + 3.0, y), Vec2::new(bw, 16.0));
                let resp = ui.scope_builder(egui::UiBuilder::new().max_rect(r), |ui| theme::button_sized(ui, b, r.size())).inner;
                p.text(o + Vec2::new(col + 134.0 + if col == 0.0 { 0.0 } else { 23.0 }, y), egui::Align2::LEFT_TOP, *l, f.clone(), Color32::WHITE);
                if resp.clicked() {
                    act = Some(match *b {
                        "MAVLink Inspector" => TempAction::Inspector,
                        "decode HWIDs" => TempAction::HwIds,
                        "logindex" => TempAction::LogBrowse,
                        "reboot pixhawk" => {
                            if let Some(l) = link {
                                l.send(LinkCommand::CommandLong { command: 246, params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                                toasts.info("Rebooting");
                            }
                            continue;
                        }
                        other => TempAction::Unavailable(other),
                    });
                }
            }
        }
    });
    act
}
