//! Windows behind MainV2.ProcessCmdKey: Ctrl+P PluginUI, Ctrl+X GMAPCache, Ctrl+J DevopsUI.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use egui::{Color32, RichText};
use mp_core::link::{Link, LinkCommand};
use mp_core::mavlink::dialects::ardupilotmega::*;

use crate::app::{AppCtx, Toasts};
use crate::forms::FormState;
use crate::theme;

#[derive(Default)]
pub struct HotkeyTools {
    pub plugins_open: bool,
    pub cache_open: bool,
    pub devops_open: bool,
    cache_rows: Option<Vec<(String, u64, u64)>>,
    devops: FormState,
    devops_init: bool,
    devops_rx: Option<crossbeam_channel::Receiver<MavMessage>>,
    request_id: u32,
}

fn dir_size(p: &Path) -> (u64, u64) {
    let mut out = (0, 0);
    if let Ok(rd) = std::fs::read_dir(p) {
        for e in rd.flatten() {
            let path = e.path();
            if path.is_dir() {
                let (b, n) = dir_size(&path);
                out.0 += b;
                out.1 += n;
            } else if let Ok(m) = e.metadata() {
                out.0 += m.len();
                out.1 += 1;
            }
        }
    }
    out
}

/// Remove files under `p` older than `age` (or every file when `age` is None).
fn remove_older(p: &Path, age: Option<Duration>) -> u64 {
    let mut n = 0;
    if let Ok(rd) = std::fs::read_dir(p) {
        for e in rd.flatten() {
            let path = e.path();
            if path.is_dir() {
                n += remove_older(&path, age);
                let _ = std::fs::remove_dir(&path);
            } else {
                let old = match age {
                    None => true,
                    Some(a) => e
                        .metadata()
                        .and_then(|m| m.modified())
                        .ok()
                        .and_then(|t| SystemTime::now().duration_since(t).ok())
                        .map(|d| d > a)
                        .unwrap_or(false),
                };
                if old && std::fs::remove_file(&path).is_ok() {
                    n += 1;
                }
            }
        }
    }
    n
}

impl HotkeyTools {
    pub fn ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        self.plugins_ui(ctx);
        self.cache_ui(ctx, app.toasts);
        self.devops_ui(ctx, app);
    }

    /// PluginUI: the plugins folder and what was loaded from it.
    fn plugins_ui(&mut self, ctx: &egui::Context) {
        if !self.plugins_open {
            return;
        }
        let dir = crate::platform::data_dir().join("plugins");
        let _ = std::fs::create_dir_all(&dir);
        let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
            .map(|r| r.flatten().map(|e| e.path()).collect())
            .unwrap_or_default();
        files.sort();
        let mut open = true;
        egui::Window::new("Plugins").open(&mut open).default_size([520.0, 300.0]).show(ctx, |ui| {
            egui::Grid::new("plugins").num_columns(4).striped(true).show(ui, |ui| {
                for h in ["Name", "Author", "Version", "Enabled"] {
                    ui.label(RichText::new(h).strong().color(Color32::WHITE));
                }
                ui.end_row();
                for f in &files {
                    let name = f.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
                    theme::label(ui, &name);
                    theme::label(ui, "");
                    theme::label(ui, "");
                    // .NET plugins (.dll / .cs) are not loadable by this build
                    ui.add_enabled(false, egui::Checkbox::new(&mut false, ""));
                    ui.end_row();
                }
            });
            if files.is_empty() {
                theme::label(ui, "No plugins found.");
            }
            ui.add_space(6.0);
            theme::label(ui, format!("Plugin folder: {}\nMission Planner .NET plugins (.dll / .cs) cannot be loaded by this build; use the Scripts tab for Python and Lua automation.", dir.display()));
            if theme::button(ui, "Open Folder").clicked() {
                ui.ctx().open_url(egui::OpenUrl::new_tab(format!("file://{}", dir.display())));
            }
        });
        self.plugins_open = open;
    }

    /// GMAPCache: size and tile count per map provider, Remove 30+days / Remove All.
    fn cache_ui(&mut self, ctx: &egui::Context, toasts: &mut Toasts) {
        if !self.cache_open {
            self.cache_rows = None;
            return;
        }
        let root = crate::platform::tile_cache_dir();
        if self.cache_rows.is_none() {
            let mut rows: Vec<(String, u64, u64)> = std::fs::read_dir(&root)
                .map(|r| {
                    r.flatten()
                        .filter(|e| e.path().is_dir())
                        .map(|e| {
                            let (b, n) = dir_size(&e.path());
                            (e.file_name().to_string_lossy().to_string(), b, n)
                        })
                        .collect()
                })
                .unwrap_or_default();
            rows.sort();
            let (b, n) = rows.iter().fold((0, 0), |a, r| (a.0 + r.1, a.1 + r.2));
            rows.push(("Total".into(), b, n));
            self.cache_rows = Some(rows);
        }
        let rows = self.cache_rows.clone().unwrap_or_default();
        let mut open = true;
        let mut action = None;
        egui::Window::new("Map Cache")
            .open(&mut open)
            .default_size([520.0, 300.0])
            .show(ctx, |ui| {
                egui::Grid::new("gmapcache")
                    .num_columns(5)
                    .striped(true)
                    .show(ui, |ui| {
                        for h in ["Name", "Size", "Count", "", ""] {
                            ui.label(RichText::new(h).strong().color(Color32::WHITE));
                        }
                        ui.end_row();
                        for (name, bytes, count) in &rows {
                            theme::label(ui, name);
                            theme::label(
                                ui,
                                format!("{}MB", (*bytes as f64 / 1024.0 / 1024.0).round()),
                            );
                            theme::label(ui, count.to_string());
                            if theme::button(ui, "Remove 30+days").clicked() {
                                action =
                                    Some((name.clone(), Some(Duration::from_secs(30 * 86400))));
                            }
                            if theme::button(ui, "Remove All").clicked() {
                                action = Some((name.clone(), None));
                            }
                            ui.end_row();
                        }
                    });
            });
        if let Some((name, age)) = action {
            let p = if name == "Total" {
                root.clone()
            } else {
                root.join(&name)
            };
            let n = remove_older(&p, age);
            toasts.info(format!("Removed {n} tiles"));
            self.cache_rows = None;
        }
        self.cache_open = open;
    }

    /// DevopsUI: DEVICE_OP_READ / DEVICE_OP_WRITE against a SPI or I2C device.
    fn devops_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if !self.devops_open {
            return;
        }
        if !self.devops_init {
            self.devops_init = true;
            let st = &mut self.devops;
            for (k, v) in [
                ("num_sysid", "1"),
                ("num_compid", "1"),
                ("num_busno", "0"),
                ("num_address", "0"),
                ("num_regstart", "0"),
                ("num_count", "1"),
            ] {
                st.set(k, v);
            }
            st.items
                .insert("dom_bustype".into(), vec!["SPI".into(), "I2C".into()]);
        }
        let mut disconnected = false;
        if let Some(rx) = &self.devops_rx {
            loop {
                let m = match rx.try_recv() {
                    Ok(m) => m,
                    Err(crossbeam_channel::TryRecvError::Empty) => break,
                    Err(crossbeam_channel::TryRecvError::Disconnected) => {
                        disconnected = true;
                        break;
                    }
                };
                let Some(line) = reply_text(m, self.request_id) else {
                    continue;
                };
                let mut t = self.devops.get("textBox1").to_string();
                t.push_str(&line);
                t.push_str("\r\n");
                self.devops.set("textBox1", t);
            }
        }
        if disconnected {
            self.devops_rx = None;
        }
        let spi = self.devops.get("dom_bustype") != "I2C";
        for (c, on) in [
            ("txt_spiname", spi),
            ("num_busno", !spi),
            ("num_address", !spi),
        ] {
            if on {
                self.devops.disabled.remove(c);
            } else {
                self.devops.disabled.insert(c.into());
            }
        }
        let mut open = true;
        let mut clicked = Vec::new();
        egui::Window::new("Devops")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .fixed_size([605.0, 215.0])
            .show(ctx, |ui| {
                clicked = crate::forms::show(ui, app, "devops", &mut self.devops);
            });
        self.devops_open = open;
        for b in clicked {
            let Some(l) = app.link else {
                app.toasts.error("Please Connect First");
                continue;
            };
            if self.devops_rx.is_none() {
                let (tx, rx) = crossbeam_channel::unbounded();
                l.send(LinkCommand::Subscribe {
                    msgid: 11001,
                    tx: tx.clone(),
                });
                l.send(LinkCommand::Subscribe { msgid: 11003, tx });
                self.devops_rx = Some(rx);
            }
            let st = &self.devops;
            let n = |k: &str| st.get(k).trim().parse::<u32>().unwrap_or(0);
            let bustype = if spi {
                DeviceOpBustype::DEVICE_OP_BUSTYPE_SPI
            } else {
                DeviceOpBustype::DEVICE_OP_BUSTYPE_I2C
            };
            let name = st.get("txt_spiname").to_string();
            match b.as_str() {
                "but_doit" => {
                    self.request_id += 1;
                    device_read(
                        l,
                        n("num_sysid") as u8,
                        n("num_compid") as u8,
                        self.request_id,
                        bustype,
                        &name,
                        n("num_busno") as u8,
                        n("num_address") as u8,
                        n("num_regstart") as u8,
                        n("num_count") as u8,
                    );
                }
                "but_test" => {
                    // write 0x72 0x00 at 0xff, then read two bytes back
                    self.request_id += 1;
                    let mut data = [0u8; 128];
                    data[0] = 0x72;
                    l.send(LinkCommand::Send(Box::new(MavMessage::DEVICE_OP_WRITE(
                        DEVICE_OP_WRITE_DATA {
                            request_id: self.request_id,
                            target_system: 1,
                            target_component: 1,
                            bustype: DeviceOpBustype::DEVICE_OP_BUSTYPE_SPI,
                            bus: 0,
                            address: 0,
                            busname: name.as_str().into(),
                            regstart: 0xff,
                            count: 2,
                            data,
                            bank: 0,
                        },
                    ))));
                    self.request_id += 1;
                    device_read(
                        l,
                        1,
                        1,
                        self.request_id,
                        DeviceOpBustype::DEVICE_OP_BUSTYPE_SPI,
                        &name,
                        0,
                        0,
                        0xff,
                        2,
                    );
                }
                _ => {}
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn device_read(
    l: &Link,
    sys: u8,
    comp: u8,
    request_id: u32,
    bustype: DeviceOpBustype,
    name: &str,
    bus: u8,
    address: u8,
    regstart: u8,
    count: u8,
) {
    l.send(LinkCommand::Send(Box::new(MavMessage::DEVICE_OP_READ(
        DEVICE_OP_READ_DATA {
            request_id,
            target_system: sys,
            target_component: comp,
            bustype,
            bus,
            address,
            busname: name.into(),
            regstart,
            count: count.min(128),
            bank: 0,
        },
    ))));
}

fn reply_text(message: MavMessage, request_id: u32) -> Option<String> {
    match message {
        MavMessage::DEVICE_OP_READ_REPLY(r) if r.request_id == request_id => {
            Some(if r.result == 0 {
                r.data[..(r.count as usize).min(r.data.len())]
                    .iter()
                    .map(|b| format!("{b:02X}"))
                    .collect()
            } else {
                format!("No Response - {}", r.result)
            })
        }
        MavMessage::DEVICE_OP_WRITE_REPLY(r) if r.request_id == request_id => {
            Some(format!("Write result {}", r.result))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keyboard_forms_are_registered() {
        let form = crate::forms::form("devops");
        assert!(form.controls.iter().any(|c| c.name == "but_doit"));
        assert_eq!(crate::forms::form("gmapcache").name, "gmapcache");
    }

    #[test]
    fn device_reply_checks_request_and_bounds_remote_count() {
        let reply = DEVICE_OP_READ_REPLY_DATA {
            request_id: 42,
            count: 255,
            data: [0xAB; 128],
            ..Default::default()
        };
        assert!(reply_text(MavMessage::DEVICE_OP_READ_REPLY(reply.clone()), 41).is_none());
        let text = reply_text(MavMessage::DEVICE_OP_READ_REPLY(reply), 42).unwrap();
        assert_eq!(text.len(), 256);
        assert!(text.starts_with("ABAB"));
    }
}
