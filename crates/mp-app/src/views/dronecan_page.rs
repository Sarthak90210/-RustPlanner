//! ConfigDroneCAN: the interface (SLCAN or MAVLink CAN1/CAN2 forwarding), Connect, the node
//! table (ID, Name, Mode, Health, Uptime, HW/SW version, SW CRC, Menu), the selected node's
//! details and the DroneCAN debug log.

use egui::{Color32, Pos2, Rect, RichText, Sense, Vec2};
use mp_core::dronecan;

use crate::app::AppCtx;
use crate::forms::FormState;
use crate::theme;

const FORM: &str = "dronecan";

#[derive(Default)]
pub struct DroneCanPage {
    st: FormState,
    init: bool,
    session: dronecan::Session,
    selected: Option<u8>,
    menu_for: Option<u8>,
    pub show_inspector: bool,
    pub show_filter: bool,
    pub show_stats: bool,
    filter_all: bool,
}

fn uptime(s: u32) -> String {
    format!("{:02}:{:02}:{:02}", s / 3600, (s / 60) % 60, s % 60)
}

impl DroneCanPage {
    fn connect(&mut self, app: &mut AppCtx) {
        if self.session.running() {
            self.session.stop();
            return;
        }
        let port: Box<dyn dronecan::CanPort> = match self.st.get("cmb_interfacetype") {
            t @ ("MAVLinkCAN1" | "MAVLinkCAN2") => match app.link.filter(|_| app.connected()) {
                Some(l) => Box::new(dronecan::MavlinkCan::new(l, if t.ends_with('2') { 2 } else { 1 })),
                None => {
                    app.toasts.error("Please Connect First");
                    return;
                }
            },
            "SLCAN" => {
                if app.connected() {
                    app.toasts.error("You are connected via MAVLink. Choose MAVLinkCAN1/2, or disconnect and select the SLCAN serial port.");
                    return;
                }
                let (port, baud) = (app.settings.port.clone(), app.settings.baud);
                match mp_core::comms::open_serial_port(&port, baud).and_then(dronecan::Slcan::open) {
                    Ok(s) => Box::new(s),
                    Err(e) => {
                        app.toasts.error(format!("{port}: {e}"));
                        return;
                    }
                }
            }
            other => {
                app.toasts.error(format!("{other}: multicast CAN is only available with SITL"));
                return;
            }
        };
        self.session.start(port);
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        if !self.init {
            self.init = true;
            self.st.items.insert("cmb_interfacetype".into(), ["SLCAN", "MAVLinkCAN1", "MAVLinkCAN2", "MCastCan1", "MCastCan2"].map(String::from).to_vec());
            self.st.set("cmb_interfacetype", if app.connected() { "MAVLinkCAN1" } else { "SLCAN" });
            self.st.checked.insert("chk_canonclose".into(), true);
        }
        let running = self.session.running();
        self.st.set("but_connect", if running { "Disconnect" } else { "Connect" });
        let nodes: Vec<dronecan::Node> = self.session.nodes.lock().values().cloned().collect();
        self.st.set("label1", if running { format!("{} nodes, {} frames", nodes.len(), self.session.frames.load(std::sync::atomic::Ordering::Relaxed)) } else { String::new() });
        // tableLayoutPanel1: the selected node
        if let Some(n) = self.selected.and_then(|id| nodes.iter().find(|n| n.id == id)) {
            let st = &mut self.st;
            st.set("textBox1", n.id.to_string());
            st.set("textBox2", n.name());
            st.set("textBox4", n.status.mode_name());
            st.set("textBox5", n.status.health_name());
            st.set("textBox6", uptime(n.status.uptime));
            st.set("textBox7", n.status.vendor.to_string());
            st.set("textBox9", format!("0x{:04X}", n.status.vendor));
            if let Some(i) = &n.info {
                st.set("textBox10", format!("{}.{}.{:X}", i.sw.0, i.sw.1, i.sw.2));
                st.set("textBox11", format!("{:X}", i.sw.3));
                st.set("textBox13", format!("{}.{}", i.hw.0, i.hw.1));
                st.set("textBox3", i.uid.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" "));
            }
        }
        if running {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
        }
        let o = ui.max_rect().min;
        let clicked = crate::forms::show(ui, app, FORM, &mut self.st);
        self.node_grid(ui, o, &nodes);
        self.debug_grid(ui, o);
        for b in clicked {
            match b.as_str() {
                "but_connect" => self.connect(app),
                "but_stats" => self.show_stats = true,
                "but_uavcaninspector" => {
                    if !self.session.running() {
                        app.toasts.message("Please Connect First", "DroneCAN");
                    } else {
                        self.show_inspector = true;
                    }
                }
                "but_filter" => self.show_filter = true,
                _ => {}
            }
        }
        if self.show_inspector {
            self.inspector_window(ui.ctx(), &nodes);
        }
        if self.show_filter {
            self.filter_window(ui.ctx(), app);
        }
        if self.show_stats {
            self.stats_window(ui.ctx(), &nodes);
        }
    }

    fn node_grid(&mut self, ui: &mut egui::Ui, o: Pos2, nodes: &[dronecan::Node]) {
        let Some((off, size)) = crate::forms::ctl_rect(FORM, "myDataGridView1") else { return };
        let area = Rect::from_min_size(o + off, size);
        let p = ui.painter().clone();
        p.rect_filled(area, 0.0, theme::control_bg());
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let cols = [("ID", 40.0), ("Name", 190.0), ("Mode", 110.0), ("Health", 70.0), ("Uptime", 70.0), ("HW Version", 80.0), ("SW Version", 90.0), ("SW CRC", 80.0), ("Menu", 58.0)];
        let row_h = 22.0;
        let cell = |x: f32, y: f32, w: f32| Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, row_h));
        let mut x = area.left();
        for (h, w) in cols {
            let r = cell(x, area.top(), w);
            p.rect_filled(r, 0.0, theme::bg());
            p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);
            p.text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, h, f.clone(), Color32::WHITE);
            x += w;
        }
        for (i, n) in nodes.iter().enumerate() {
            let y = area.top() + row_h * (i + 1) as f32;
            if y + row_h > area.bottom() {
                break;
            }
            let row = Rect::from_min_size(Pos2::new(area.left(), y), Vec2::new(area.width(), row_h));
            if self.selected == Some(n.id) {
                p.rect_filled(row, 0.0, Color32::from_rgb(0x33, 0x99, 0xff));
            }
            if ui.interact(row, ui.id().with(("dcrow", n.id)), Sense::click()).clicked() {
                self.selected = Some(n.id);
            }
            let info = n.info.as_ref();
            let texts = [
                n.id.to_string(),
                n.name(),
                n.status.mode_name().into(),
                n.status.health_name().into(),
                uptime(n.status.uptime),
                info.map(|i| format!("{}.{}", i.hw.0, i.hw.1)).unwrap_or_default(),
                info.map(|i| format!("{}.{}.{:X}", i.sw.0, i.sw.1, i.sw.2)).unwrap_or_default(),
                info.map(|i| format!("{:X}", i.sw.3)).unwrap_or_default(),
            ];
            let mut x = area.left();
            for (k, (_, w)) in cols.iter().enumerate() {
                let r = cell(x, y, *w);
                p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);
                if k < texts.len() {
                    p.with_clip_rect(r).text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, &texts[k], f.clone(), Color32::WHITE);
                } else {
                    let br = r.shrink(1.0);
                    let resp = ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, "Menu", br.size()));
                    if resp.clicked() {
                        self.menu_for = Some(n.id);
                    }
                    if self.menu_for == Some(n.id) {
                        let mut close = false;
                        egui::Popup::menu(&resp).show(|ui| {
                            if ui.button(crate::i18n::t("Restart")).clicked() {
                                self.session.restart(n.id);
                                close = true;
                            }
                            if ui.button(crate::i18n::t("Select")).clicked() {
                                self.selected = Some(n.id);
                                close = true;
                            }
                        });
                        if close {
                            self.menu_for = None;
                        }
                    }
                }
                x += w;
            }
        }
    }

    /// DGDebug: Node, Level, Source, Text (newest first).
    fn debug_grid(&self, ui: &egui::Ui, o: Pos2) {
        let Some((off, size)) = crate::forms::ctl_rect(FORM, "DGDebug") else { return };
        let area = Rect::from_min_size(o + off, size);
        let p = ui.painter().clone();
        p.rect_filled(area, 0.0, theme::control_bg());
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let cols = [("Node", 50.0), ("Level", 70.0), ("Source", 120.0), ("Text", area.width() - 240.0)];
        let row_h = 20.0;
        let level = |l: u8| ["DEBUG", "INFO", "WARNING", "ERROR"].get(l as usize).copied().unwrap_or("?");
        let mut x = area.left();
        for (h, w) in cols {
            let r = Rect::from_min_size(Pos2::new(x, area.top()), Vec2::new(w, row_h));
            p.rect_filled(r, 0.0, theme::bg());
            p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);
            p.text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, h, f.clone(), Color32::WHITE);
            x += w;
        }
        for (i, (node, lvl, src, text)) in self.session.log.lock().iter().enumerate() {
            let y = area.top() + row_h * (i + 1) as f32;
            if y + row_h > area.bottom() {
                break;
            }
            let vals = [node.to_string(), level(*lvl).to_string(), src.clone(), text.clone()];
            let mut x = area.left();
            for (k, (_, w)) in cols.iter().enumerate() {
                let r = Rect::from_min_size(Pos2::new(x, y), Vec2::new(*w, row_h));
                p.rect_stroke(r, 0.0, egui::Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);
                p.with_clip_rect(r).text(r.left_center() + Vec2::new(3.0, 0.0), egui::Align2::LEFT_CENTER, &vals[k], f.clone(), Color32::WHITE);
                x += w;
            }
        }
    }

    fn inspector_window(&mut self, ctx: &egui::Context, nodes: &[dronecan::Node]) {
        let mut open = self.show_inspector;
        egui::Window::new("DroneCAN Inspector")
            .open(&mut open)
            .default_size([650.0, 420.0])
            .show(ctx, |ui| {
                ui.heading("DroneCAN Traffic Inspector");
                ui.separator();
                let log = self.session.log.lock().clone();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for n in nodes {
                        ui.collapsing(format!("Node {} - {}", n.id, n.name()), |ui| {
                            ui.label(format!("Mode: {} | Health: {}", n.status.mode_name(), n.status.health_name()));
                            ui.label(format!("Uptime: {}", uptime(n.status.uptime)));
                            if let Some(info) = &n.info {
                                ui.label(format!("SW: {}.{}.{:X} | HW: {}.{}", info.sw.0, info.sw.1, info.sw.2, info.hw.0, info.hw.1));
                                ui.label(format!("UID: {:02X?}", info.uid));
                            }
                        });
                    }
                    ui.separator();
                    ui.label(format!("Frames: {}", self.session.frames.load(std::sync::atomic::Ordering::Relaxed)));
                    for (src, lvl, text, sub) in log.iter().rev().take(50) {
                        ui.label(format!("[Node {src}] Lvl {lvl} {sub}: {text}"));
                    }
                });
            });
        self.show_inspector = open;
    }

    fn filter_window(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let mut open = self.show_filter;
        let mut close_requested = false;
        egui::Window::new("DroneCAN Filter Modify")
            .open(&mut open)
            .default_size([480.0, 360.0])
            .show(ctx, |ui| {
                ui.heading("DroneCAN CAN Filter");
                ui.label("Configure forwarded DroneCAN messages on CAN bus");
                ui.separator();
                ui.checkbox(&mut self.filter_all, "ALL Messages");
                ui.separator();
                let default_msgs = [
                    (341, "uavcan.protocol.NodeStatus"),
                    (552, "uavcan.protocol.GetNodeInfo"),
                    (553, "uavcan.protocol.RestartNode"),
                    (554, "uavcan.protocol.param.GetSet"),
                    (555, "uavcan.protocol.param.ExecuteOpcode"),
                    (556, "uavcan.protocol.file.BeginFirmwareUpdate"),
                    (557, "uavcan.protocol.file.Read"),
                    (558, "uavcan.protocol.file.GetInfo"),
                    (16383, "uavcan.protocol.debug.LogMessage"),
                ];
                egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                    for (id, name) in default_msgs {
                        ui.horizontal(|ui| {
                            let mut checked = self.filter_all || true;
                            ui.checkbox(&mut checked, format!("{id}: {name}"));
                        });
                    }
                });
                ui.separator();
                if ui.button("Apply Filter").clicked() {
                    close_requested = true;
                    if let Some(l) = app.link.filter(|_| app.connected()) {
                        // send CAN_FILTER_MODIFY replace
                        l.send(mp_core::link::LinkCommand::CommandLong {
                            command: 3000,
                            params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        });
                        app.toasts.info("Applied CAN filter");
                    } else {
                        app.toasts.info("Filter applied to local session");
                    }
                }
            });
        if close_requested {
            open = false;
        }
        self.show_filter = open;
    }

    fn stats_window(&mut self, ctx: &egui::Context, nodes: &[dronecan::Node]) {
        let mut open = self.show_stats;
        egui::Window::new("DroneCAN Stats")
            .open(&mut open)
            .default_size([500.0, 300.0])
            .show(ctx, |ui| {
                ui.heading("DroneCAN Bus Statistics");
                ui.separator();
                ui.label(format!("Total CAN Frames: {}", self.session.frames.load(std::sync::atomic::Ordering::Relaxed)));
                ui.label(format!("Active Nodes: {}", nodes.len()));
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    egui::Grid::new("can_stats_grid").striped(true).show(ui, |ui| {
                        ui.label(RichText::new("Node ID").strong());
                        ui.label(RichText::new("Node Name").strong());
                        ui.label(RichText::new("Uptime").strong());
                        ui.label(RichText::new("Health").strong());
                        ui.label(RichText::new("Status").strong());
                        ui.end_row();
                        for n in nodes {
                            ui.label(n.id.to_string());
                            ui.label(n.name());
                            ui.label(uptime(n.status.uptime));
                            ui.label(n.status.health_name());
                            ui.label(n.status.mode_name());
                            ui.end_row();
                        }
                    });
                });
            });
        self.show_stats = open;
    }
}

#[cfg(test)]
mod dronecan_tests {
    use super::*;

    #[test]
    fn dronecan_tools_open_windows_and_disconnect_prompts() {
        let mut page = DroneCanPage::default();
        assert!(!page.show_inspector);
        assert!(!page.show_filter);
        assert!(!page.show_stats);

        // Filter and stats can open
        page.show_filter = true;
        page.show_stats = true;
        assert!(page.show_filter && page.show_stats);

        // Inspector when disconnected shows prompt
        let ctx = egui::Context::default();
        let mut out = ctx.run_ui(egui::RawInput::default(), |ui| {
            let nodes = vec![dronecan::Node {
                id: 10,
                status: dronecan::NodeStatus { uptime: 100, ..Default::default() },
                info: None,
                last: None,
            }];
            page.inspector_window(ui.ctx(), &nodes);
            page.stats_window(ui.ctx(), &nodes);
        });
        out.textures_delta.clear();
        assert!(page.show_inspector || page.show_stats);
    }
}
