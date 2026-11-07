//! CONFIG > MAVFtp (`Controls/MavFTPUI.cs`): directory tree (208 px) beside a details list
//! (Name / Type / Size / Date modified), the right-click file menu and a status strip.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};

use egui::{Color32, RichText, Sense, Vec2};
use mp_core::ftp::{Entry, Ftp};
use mp_core::link::Link;

use crate::app::AppCtx;
use crate::theme;

enum Done {
    Listing(String, Vec<Entry>),
    Refresh(String),
    Downloaded(String, Vec<u8>),
    Crc(String, u32),
    Error(String),
}

#[derive(Default)]
struct Shared {
    status: String,
    progress: Option<(u64, u64)>,
    done: Vec<Done>,
    busy: bool,
}

enum Prompt {
    Rename(String, String),
    NewFolder(String),
    Delete(String, bool),
}

pub struct MavFtpUi {
    cwd: String,
    tree: BTreeMap<String, Vec<String>>,
    expanded: BTreeSet<String>,
    entries: Vec<Entry>,
    selected: Option<usize>,
    shared: Arc<Mutex<Shared>>,
    prompt: Option<Prompt>,
    started: bool,
}

impl Default for MavFtpUi {
    fn default() -> Self {
        Self { cwd: "/".into(), tree: BTreeMap::new(), expanded: BTreeSet::from(["/".to_string()]), entries: Vec::new(), selected: None, shared: Default::default(), prompt: None, started: false }
    }
}

fn join(dir: &str, name: &str) -> String {
    if dir.ends_with('/') {
        format!("{dir}{name}")
    } else {
        format!("{dir}/{name}")
    }
}

/// Left-aligned list cell, optionally with a folder / file glyph (ImageList icons).
fn cell(ui: &mut egui::Ui, w: f32, text: &str, icon: Option<bool>, bg: Color32) -> egui::Response {
    let (r, resp) = ui.allocate_exact_size(Vec2::new(w, 18.0), Sense::click());
    let p = ui.painter();
    if bg != Color32::TRANSPARENT {
        p.rect_filled(r, 0.0, bg);
    }
    let mut x = r.left() + 2.0;
    if let Some(dir) = icon {
        let ir = egui::Rect::from_min_size(egui::pos2(x, r.top() + 3.0), Vec2::new(14.0, 12.0));
        if dir {
            p.rect_filled(egui::Rect::from_min_size(ir.min, Vec2::new(6.0, 3.0)), 1.0, Color32::from_rgb(0xe8, 0xb8, 0x4a));
            p.rect_filled(egui::Rect::from_min_max(ir.min + Vec2::new(0.0, 2.0), ir.max), 1.0, Color32::from_rgb(0xf4, 0xcf, 0x6a));
        } else {
            let fr = egui::Rect::from_min_size(ir.min + Vec2::new(2.0, -1.0), Vec2::new(10.0, 13.0));
            p.rect_filled(fr, 0.0, Color32::WHITE);
            p.rect_stroke(fr, 0.0, egui::Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);
        }
        x += 18.0;
    }
    p.with_clip_rect(r).text(egui::pos2(x, r.center().y), egui::Align2::LEFT_CENTER, text, crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::WHITE);
    resp
}

fn human(n: u64) -> String {
    if n >= 1 << 20 {
        format!("{:.1} MB", n as f64 / (1 << 20) as f64)
    } else if n >= 1024 {
        format!("{:.1} KB", n as f64 / 1024.0)
    } else {
        format!("{n} B")
    }
}

impl MavFtpUi {
    /// Run `job` on a worker thread with a fresh FTP session.
    fn run(&self, link: &Link, label: &str, job: impl FnOnce(&mut Ftp, &Arc<Mutex<Shared>>) -> Done + Send + 'static) {
        let mut ftp = Ftp::new(link);
        let sh = self.shared.clone();
        {
            let mut s = sh.lock().unwrap();
            if s.busy {
                return;
            }
            s.busy = true;
            s.status = label.to_string();
            s.progress = None;
        }
        std::thread::spawn(move || {
            let d = job(&mut ftp, &sh);
            let mut s = sh.lock().unwrap();
            s.busy = false;
            s.progress = None;
            s.done.push(d);
        });
    }

    fn list(&self, link: &Link, dir: String) {
        self.run(link, &format!("Listing {dir}"), move |f, _| match f.list(&dir) {
            Ok(v) => Done::Listing(dir, v),
            Err(e) => Done::Error(format!("{dir}: {e}")),
        });
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let Some(link) = app.link.filter(|_| app.connected()) else {
            theme::label(ui, "Please Connect First");
            return;
        };
        if !self.started {
            self.started = true;
            let l = link;
            self.run(l, "Reset sessions", |f, _| {
                let _ = f.reset();
                Done::Refresh("/".into())
            });
        }
        // results from the worker
        let done: Vec<Done> = std::mem::take(&mut self.shared.lock().unwrap().done);
        for d in done {
            match d {
                Done::Listing(dir, v) => {
                    self.tree.insert(dir.clone(), v.iter().filter(|e| e.is_dir).map(|e| e.name.clone()).collect());
                    if dir == self.cwd {
                        self.entries = v;
                        self.entries.sort_by(|a, b| b.is_dir.cmp(&a.is_dir).then(a.name.cmp(&b.name)));
                        self.selected = None;
                    }
                    self.shared.lock().unwrap().status = format!("{dir}: {} items", self.entries.len());
                }
                Done::Refresh(dir) => self.list(link, dir),
                Done::Downloaded(name, bytes) => {
                    let n = bytes.len();
                    let target = crate::platform::save_file_as("Save File", &name, "All Files", &["*"]);
                    if let Some(p) = target {
                        match std::fs::write(&p, bytes) {
                            Ok(()) => self.shared.lock().unwrap().status = format!("Downloaded {name} ({}) to {}", human(n as u64), p.display()),
                            Err(e) => app.toasts.error(e.to_string()),
                        }
                    }
                }
                Done::Crc(name, crc) => app.toasts.error(format!("{name}\nCRC32: 0x{crc:08X}")),
                Done::Error(e) => {
                    self.shared.lock().unwrap().status = e.clone();
                    app.toasts.error(e);
                }
            }
        }
        let (status, progress, busy) = {
            let s = self.shared.lock().unwrap();
            (s.status.clone(), s.progress, s.busy)
        };
        if busy {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));
        }

        // statusStrip1 (22 px) docked bottom
        egui::Panel::bottom("ftp_status").exact_size(22.0).frame(egui::Frame::new().fill(theme::control_bg())).show(ui, |ui| {
            ui.horizontal_centered(|ui| {
                theme::label(ui, &status);
                if let Some((a, b)) = progress {
                    theme::progress(ui, if b > 0 { a as f32 / b as f32 } else { 0.0 }, Vec2::new(160.0, 14.0), "");
                    theme::label(ui, format!("{} / {}", human(a), human(b)));
                }
            });
        });
        // splitContainer1.Panel1: treeView1
        let mut open_dir: Option<String> = None;
        egui::Panel::left("ftp_tree").resizable(true).default_size(208.0).frame(egui::Frame::new().fill(theme::control_bg()).inner_margin(3)).show(ui, |ui| {
            egui::ScrollArea::vertical().id_salt("ftptree").auto_shrink([false, false]).show(ui, |ui| {
                self.tree_node(ui, "/", 0, &mut open_dir);
            });
        });
        // splitContainer1.Panel2: listView1 (details)
        let mut action: Option<&'static str> = None;
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::bg())).show(ui, |ui| {
            let cols = [("Name", 220.0), ("Type", 90.0), ("Size", 90.0), ("Date modified", 120.0)];
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 0.0;
                for (h, w) in cols {
                    cell(ui, w, h, None, theme::control_bg());
                }
            });
            ui.separator();
            let resp = egui::ScrollArea::vertical().id_salt("ftplist").auto_shrink([false, false]).show(ui, |ui| {
                if self.cwd != "/" {
                    let r = ui.add(egui::Label::new(RichText::new("..").color(Color32::WHITE)).sense(Sense::click()));
                    if r.double_clicked() {
                        let parent = self.cwd.rsplit_once('/').map(|(a, _)| if a.is_empty() { "/".to_string() } else { a.to_string() }).unwrap_or_else(|| "/".into());
                        open_dir = Some(if self.cwd.starts_with('@') { "/".into() } else { parent });
                    }
                }
                for (i, e) in self.entries.iter().enumerate() {
                    let sel = self.selected == Some(i);
                    let row = ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 0.0;
                        let bg = if sel { Color32::from_rgb(0x33, 0x99, 0xff) } else { Color32::TRANSPARENT };
                        cell(ui, 220.0, &e.name, Some(e.is_dir), bg);
                        cell(ui, 90.0, if e.is_dir { "Directory" } else { "File" }, None, Color32::TRANSPARENT);
                        cell(ui, 90.0, &if e.is_dir { String::new() } else { human(e.size) }, None, Color32::TRANSPARENT);
                        cell(ui, 120.0, "", None, Color32::TRANSPARENT);
                    });
                    let r = ui.interact(row.response.rect, ui.scope_id().with(("ftprow", i)), Sense::click());
                    if r.clicked() || r.secondary_clicked() {
                        self.selected = Some(i);
                    }
                    if r.double_clicked() && e.is_dir {
                        open_dir = Some(join(&self.cwd, &e.name));
                    }
                }
                ui.allocate_space(ui.available_size());
            });
            let area = ui.interact(resp.inner_rect, ui.scope_id().with("ftparea"), Sense::click());
            area.context_menu(|ui| {
                let has_file = self.selected.and_then(|i| self.entries.get(i)).map(|e| !e.is_dir).unwrap_or(false);
                let has_sel = self.selected.is_some();
                for (t, en) in [("Download Burst", has_file), ("Download", has_file), ("Upload", true), ("Delete", has_sel), ("Rename", has_sel), ("New Folder", true), ("GetCRC32", has_file)] {
                    if ui.add_enabled(en && !busy, egui::Button::new(t)).clicked() {
                        action = Some(t);
                        ui.close();
                    }
                }
            });
        });
        if let Some(d) = open_dir {
            self.cwd = d.clone();
            self.expanded.insert(d.clone());
            self.list(link, d);
        }
        let sel = self.selected.and_then(|i| self.entries.get(i)).cloned();
        match (action, sel) {
            (Some("Download" | "Download Burst"), Some(e)) => {
                let path = join(&self.cwd, &e.name);
                let name = e.name.clone();
                self.run(link, &format!("Downloading {path}"), move |f, sh| {
                    let sh2 = sh.clone();
                    match f.get(&path, |a, b| sh2.lock().unwrap().progress = Some((a, b))) {
                        Ok(b) => Done::Downloaded(name, b),
                        Err(e) => Done::Error(format!("{path}: {e}")),
                    }
                });
            }
            (Some("Upload"), _) => {
                if let Some(p) = crate::platform::pick_file("Upload File", "All Files", &["*"]) {
                    if let Ok(bytes) = std::fs::read(&p) {
                        let name = p.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
                        let path = join(&self.cwd, &name);
                        let dir = self.cwd.clone();
                        self.run(link, &format!("Uploading {path}"), move |f, sh| {
                            let sh2 = sh.clone();
                            match f.put(&path, &bytes, |a, b| sh2.lock().unwrap().progress = Some((a, b))) {
                                Ok(()) => Done::Refresh(dir),
                                Err(e) => Done::Error(format!("{path}: {e}")),
                            }
                        });
                    }
                }
            }
            (Some("Delete"), Some(e)) => self.prompt = Some(Prompt::Delete(join(&self.cwd, &e.name), e.is_dir)),
            (Some("Rename"), Some(e)) => self.prompt = Some(Prompt::Rename(join(&self.cwd, &e.name), e.name.clone())),
            (Some("New Folder"), _) => self.prompt = Some(Prompt::NewFolder("New Folder".into())),
            (Some("GetCRC32"), Some(e)) => {
                let path = join(&self.cwd, &e.name);
                self.run(link, &format!("CRC32 {path}"), move |f, _| match f.crc32(&path) {
                    Ok(c) => Done::Crc(path, c),
                    Err(e) => Done::Error(format!("{path}: {e}")),
                });
            }
            _ => {}
        }
        self.prompt_ui(ui.ctx(), link);
    }

    fn tree_node(&mut self, ui: &mut egui::Ui, path: &str, depth: usize, open_dir: &mut Option<String>) {
        let name = if path == "/" { "/".to_string() } else { path.rsplit('/').next().unwrap_or(path).to_string() };
        let expanded = self.expanded.contains(path);
        let sel = self.cwd == path;
        let r = ui.horizontal(|ui| {
            ui.add_space(depth as f32 * 14.0);
            let t = cell(ui, 12.0, if expanded { "-" } else { "+" }, None, Color32::TRANSPARENT);
            let w = 24.0 + name.len() as f32 * 7.0;
            let l = cell(ui, w, &name, Some(true), if sel { Color32::from_rgb(0x33, 0x99, 0xff) } else { Color32::TRANSPARENT });
            (t.clicked(), l.clicked())
        });
        let (toggle, select) = r.inner;
        if toggle {
            if expanded {
                self.expanded.remove(path);
            } else {
                self.expanded.insert(path.to_string());
                if !self.tree.contains_key(path) {
                    *open_dir = Some(path.to_string());
                }
            }
        }
        if select {
            *open_dir = Some(path.to_string());
        }
        if self.expanded.contains(path) {
            let children = self.tree.get(path).cloned().unwrap_or_default();
            for c in children {
                let child = if c.starts_with('@') { c.clone() } else { join(path, &c) };
                self.tree_node(ui, &child, depth + 1, open_dir);
            }
        }
    }

    fn prompt_ui(&mut self, ctx: &egui::Context, link: &Link) {
        let Some(p) = self.prompt.as_mut() else { return };
        let mut done = None;
        let title = match p {
            Prompt::Rename(..) => "Rename",
            Prompt::NewFolder(_) => "New Folder",
            Prompt::Delete(..) => "Delete",
        };
        egui::Window::new(title).collapsible(false).resizable(false).show(ctx, |ui| {
            match p {
                Prompt::Rename(_, n) => {
                    theme::label(ui, "New name");
                    ui.text_edit_singleline(n);
                }
                Prompt::NewFolder(n) => {
                    theme::label(ui, "Folder name");
                    ui.text_edit_singleline(n);
                }
                Prompt::Delete(path, _) => {
                    theme::label(ui, format!("Delete {path}?"));
                }
            }
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
        let p = self.prompt.take().unwrap();
        if !ok {
            return;
        }
        let dir = self.cwd.clone();
        match p {
            Prompt::Rename(from, to) => {
                let to = join(&dir, &to);
                self.run(link, "Renaming", move |f, _| match f.rename(&from, &to) {
                    Ok(()) => Done::Refresh(dir),
                    Err(e) => Done::Error(e),
                });
            }
            Prompt::NewFolder(n) => {
                let path = join(&dir, &n);
                self.run(link, "Creating folder", move |f, _| match f.mkdir(&path) {
                    Ok(()) => Done::Refresh(dir),
                    Err(e) => Done::Error(e),
                });
            }
            Prompt::Delete(path, is_dir) => {
                self.run(link, "Deleting", move |f, _| match if is_dir { f.rmdir(&path) } else { f.remove(&path) } {
                    Ok(()) => Done::Refresh(dir),
                    Err(e) => Done::Error(e),
                });
            }
        }
    }
}
