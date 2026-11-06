//! ConfigDroneCAN: the interface (SLCAN or MAVLink CAN1/CAN2 forwarding), Connect, the node
//! table (ID, Name, Mode, Health, Uptime, HW/SW version, SW CRC, Menu), the selected node's
//! details and the DroneCAN debug log.

use egui::{Color32, Pos2, Rect, Sense, Vec2};
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
                "but_stats" => app.toasts.info(format!("Frames received: {}", self.session.frames.load(std::sync::atomic::Ordering::Relaxed))),
                "but_uavcaninspector" | "but_filter" => app.toasts.info("Node and message details are shown below the node list."),
                _ => {}
            }
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
                            if ui.button("Restart").clicked() {
                                self.session.restart(n.id);
                                close = true;
                            }
                            if ui.button("Select").clicked() {
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
}
