//! Platform glue: data directory and file open/save.
//!
//! Desktop uses native dialogs (rfd). Android and iOS have no blocking file dialog, so files
//! are kept in the app's data directory and chosen through the in-app file browser.

use std::path::PathBuf;
use std::sync::OnceLock;

static DATA_DIR: OnceLock<PathBuf> = OnceLock::new();

pub fn set_data_dir(p: PathBuf) {
    let _ = DATA_DIR.set(p);
}

/// Move this process's top-level window to the Windows virtual desktop with id `guid`
/// (IVirtualDesktopManager only moves windows owned by the calling process).
#[cfg(windows)]
pub fn move_to_virtual_desktop(cc: &eframe::CreationContext<'_>, guid: &str) -> Result<(), String> {
    use raw_window_handle::{HasWindowHandle, RawWindowHandle};
    use windows::Win32::Foundation::HWND;
    use windows::Win32::System::Com::{CoCreateInstance, CoInitializeEx, CLSCTX_ALL, COINIT_APARTMENTTHREADED};
    use windows::Win32::UI::Shell::{IVirtualDesktopManager, VirtualDesktopManager};
    let hex: String = guid.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    let id = windows::core::GUID::from_u128(u128::from_str_radix(&hex, 16).map_err(|e| e.to_string())?);
    let RawWindowHandle::Win32(h) = cc.window_handle().map_err(|e| e.to_string())?.as_raw() else { return Err("not a Win32 window".into()) };
    unsafe {
        let _ = CoInitializeEx(None, COINIT_APARTMENTTHREADED);
        let vdm: IVirtualDesktopManager = CoCreateInstance(&VirtualDesktopManager, None, CLSCTX_ALL).map_err(|e| e.to_string())?;
        vdm.MoveWindowToDesktop(HWND(h.hwnd.get() as *mut _), &id).map_err(|e| e.to_string())
    }
}

#[cfg(not(windows))]
pub fn move_to_virtual_desktop(_cc: &eframe::CreationContext<'_>, _guid: &str) -> Result<(), String> {
    Ok(())
}

pub fn data_dir() -> PathBuf {
    DATA_DIR
        .get_or_init(|| {
            #[cfg(not(any(target_os = "android", target_os = "ios")))]
            {
                if let Some(d) = eframe::storage_dir("MissionPlannerRS") {
                    return d;
                }
            }
            #[cfg(target_os = "ios")]
            if let Some(home) = std::env::var_os("HOME") {
                return ios_data_dir(std::path::Path::new(&home));
            }
            std::env::temp_dir().join("MissionPlannerRS")
        })
        .clone()
}

// iOS may purge tmp while the app is suspended. Settings, missions and tile caches
// belong in the application's persistent sandbox instead.
#[cfg(target_os = "ios")]
fn ios_data_dir(home: &std::path::Path) -> PathBuf {
    home.join("Library").join("Application Support").join("MissionPlannerRS")
}

pub fn files_dir() -> PathBuf {
    #[cfg(target_os = "ios")]
    let d = std::env::var_os("HOME")
        .map(|home| PathBuf::from(home).join("Documents"))
        .unwrap_or_else(|| data_dir().join("files"));
    #[cfg(not(target_os = "ios"))]
    let d = data_dir().join("files");
    let _ = std::fs::create_dir_all(&d);
    d
}

pub fn tile_cache_dir() -> PathBuf {
    data_dir().join("tiles")
}

/// Available storage roots for document browsing across desktop and mobile targets.
pub fn document_roots() -> Vec<(String, PathBuf)> {
    #[allow(unused_mut)]
    let mut roots = vec![("App Storage".into(), files_dir())];
    #[cfg(target_os = "android")]
    {
        for (label, p) in [
            ("Downloads", "/sdcard/Download"),
            ("Documents", "/sdcard/Documents"),
            ("Internal Storage", "/storage/emulated/0"),
        ] {
            let path = PathBuf::from(p);
            if path.exists() {
                roots.push((label.into(), path));
            }
        }
    }
    #[cfg(target_os = "ios")]
    {
        if let Ok(home) = std::env::var("HOME") {
            let docs = PathBuf::from(home).join("Documents");
            if docs.exists() {
                roots.push(("Documents".into(), docs));
            }
        }
    }
    roots
}

pub const HAS_NATIVE_DIALOGS: bool = cfg!(not(any(target_os = "android", target_os = "ios")));

/// Desktop OS dialog, or a deferred browser of the mobile app's files.
pub struct FilePicker {
    id: egui::Id,
    title: String,
    root: PathBuf,
    pub current_dir: PathBuf,
    pub roots: Vec<(String, PathBuf)>,
    exts: Vec<String>,
    name: String,
    save: bool,
    overwrite: bool,
    ready: Option<Option<Vec<PathBuf>>>,
    multiple: bool,
    selected_names: std::collections::BTreeSet<String>,
}

impl FilePicker {
    pub fn new(title: &str, exts: &[&str], save_name: Option<&str>) -> Self {
        let mut picker = Self::native_or_browser(title, files_dir(), exts, save_name, false);
        picker.roots = document_roots();
        picker
    }

    pub fn open_from(title: &str, root: PathBuf, exts: &[&str], multiple: bool) -> Self {
        let mut picker = Self::native_or_browser(title, root, exts, None, multiple);
        picker.roots = document_roots();
        picker
    }

    fn native_or_browser(title: &str, root: PathBuf, exts: &[&str], save_name: Option<&str>, multiple: bool) -> Self {
        let mut picker = Self::in_directory(title, root, exts, save_name);
        picker.multiple = multiple;
        #[cfg(not(any(target_os = "android", target_os = "ios")))]
        {
            let dialog = rfd::FileDialog::new().set_title(title).set_directory(&picker.root).add_filter(title, exts);
            picker.ready = Some(if let Some(name) = save_name { dialog.set_file_name(name).save_file().map(|p| vec![p]) }
                else if multiple { dialog.pick_files() } else { dialog.pick_file().map(|p| vec![p]) });
        }
        picker
    }

    pub(crate) fn in_directory(title: &str, root: PathBuf, exts: &[&str], save_name: Option<&str>) -> Self {
        static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
        let current_dir = root.clone();
        let roots = vec![("Storage".into(), root.clone())];
        Self {
            id: egui::Id::new(("file_picker", NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed))),
            title: title.into(),
            root,
            current_dir,
            roots,
            exts: exts.iter().map(|s| s.to_string()).collect(),
            name: save_name.unwrap_or_default().into(),
            save: save_name.is_some(),
            overwrite: false,
            ready: None,
            multiple: false,
            selected_names: Default::default(),
        }
    }

    fn selected_path(&self) -> Option<PathBuf> {
        let name = if self.save { self.name.trim() } else { self.name.as_str() };
        if name.is_empty() || name == "." || name == ".." || name.chars().any(|c| matches!(c, '/' | '\\' | ':' | '\0')) { return None; }
        let mut path = self.current_dir.join(name);
        if self.save && !self.exts.is_empty() {
            match path.extension().and_then(|s| s.to_str()).filter(|s| !s.is_empty()) {
                None => { path.set_extension(&self.exts[0]); }
                Some(ext) if !self.exts.iter().any(|allowed| ext.eq_ignore_ascii_case(allowed)) => return None,
                _ => {}
            }
        }
        if !self.save && !self.exts.is_empty() && !self.exts.iter().any(|e| e == "*" || path.extension().is_some_and(|x| x.eq_ignore_ascii_case(e))) { return None; }
        Some(path)
    }

    /// Some(None) means Cancel; None means the browser is still open.
    pub fn show(&mut self, ctx: &egui::Context) -> Option<Option<PathBuf>> {
        self.show_many(ctx).map(|paths| paths.and_then(|v| v.into_iter().next()))
    }

    pub fn show_many(&mut self, ctx: &egui::Context) -> Option<Option<Vec<PathBuf>>> {
        if let Some(done) = self.ready.take() { return Some(done); }
        let mut done = None;
        let modal = egui::Modal::new(self.id).show(ctx, |ui| {
            ui.set_min_width(300.0);
            crate::theme::label(ui, &self.title);
            if self.roots.len() > 1 {
                ui.horizontal(|ui| {
                    for (lbl, p) in &self.roots {
                        let is_sel = &self.current_dir == p || self.current_dir.starts_with(p);
                        if ui.selectable_label(is_sel, lbl).clicked() {
                            self.current_dir = p.clone();
                            self.name.clear();
                            self.selected_names.clear();
                        }
                    }
                });
            }
            let exts: Vec<_> = self.exts.iter().map(String::as_str).collect();
            let (dirs, files) = list_entries_in(&self.current_dir, &exts);

            // Up button if not at storage root
            if let Some(parent) = self.current_dir.parent() {
                let at_root = self.roots.iter().any(|(_, r)| &self.current_dir == r) || &self.current_dir == &self.root;
                if !at_root && parent.exists() {
                    if ui.button("📁 .. (Up)").clicked() {
                        self.current_dir = parent.to_path_buf();
                        self.name.clear();
                        self.selected_names.clear();
                    }
                }
            }

            egui::ScrollArea::vertical().max_height(220.0).show(ui, |ui| {
                if dirs.is_empty() && files.is_empty() { crate::theme::label(ui, "No files"); }
                for dir in dirs {
                    let dirname = dir.file_name().unwrap_or_default().to_string_lossy().into_owned();
                    if ui.selectable_label(false, format!("📁 {dirname}")).clicked() {
                        self.current_dir = dir;
                        self.name.clear();
                        self.selected_names.clear();
                    }
                }
                for path in files {
                    let name = path.file_name().unwrap_or_default().to_string_lossy().into_owned();
                    let selected = if self.multiple { self.selected_names.contains(&name) } else { self.name == name };
                    if ui.selectable_label(selected, &name).clicked() {
                        if self.multiple { if !self.selected_names.remove(&name) { self.selected_names.insert(name); } }
                        else { self.name = name; }
                        self.overwrite = false;
                    }
                }
            });
            if self.save && ui.text_edit_singleline(&mut self.name).changed() { self.overwrite = false; }
            let path = self.selected_path();
            let paths: Vec<PathBuf> = if self.multiple {
                let exts: Vec<_> = self.exts.iter().map(String::as_str).collect();
                list_files_in(&self.current_dir, &exts).into_iter().filter(|p| p.file_name().is_some_and(|n| self.selected_names.contains(n.to_string_lossy().as_ref()))).collect()
            } else { path.clone().filter(|p| self.save || p.is_file()).into_iter().collect() };
            let valid = !paths.is_empty();
            ui.horizontal(|ui| {
                let text = if self.overwrite { "Overwrite" } else if self.save { "Save" } else { "Open" };
                if ui.add_enabled(valid, egui::Button::new(text)).clicked() {
                    if self.save { if let Some(p) = &path { self.name = p.file_name().unwrap_or_default().to_string_lossy().into_owned(); } }
                    if self.save && path.as_ref().is_some_and(|p| p.exists()) && !self.overwrite { self.overwrite = true; }
                    else { done = Some(Some(paths)); }
                }
                if crate::theme::button(ui, "Cancel").clicked() { done = Some(None); }
            });
        });
        if modal.is_top_modal && !modal.any_popup_open && ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Escape)) { done = Some(None); }
        done
    }
}

#[cfg(test)]
mod file_picker_tests {
    use super::*;
    #[test]
    fn explicit_multi_selection_filters_directories_and_does_not_choose_the_last_file() {
        let root = picker_test_support::root();
        for name in ["a.tlog", "z.TLOG", "ignore.txt"] { std::fs::write(root.join(name), b"fixture").unwrap(); }
        std::fs::create_dir(root.join("directory.tlog")).unwrap();
        assert_eq!(list_files_in(&root, &["tlog"]), [root.join("a.tlog"), root.join("z.TLOG")]);
        let ctx = egui::Context::default();
        let mut picker = FilePicker::in_directory("Logs", root.clone(), &["tlog"], None);
        picker.multiple = true;
        let mut result = None;
        {
            let mut draw = |ctx: &egui::Context| { if let Some(done) = picker.show_many(ctx) { result = Some(done); } };
            picker_test_support::frame(&ctx, &mut draw, Vec::new());
        }
        assert!(result.is_none(), "opening a picker must not select a file");
        {
            let mut draw = |ctx: &egui::Context| { if let Some(done) = picker.show_many(ctx) { result = Some(done); } };
            picker_test_support::click(&ctx, &mut draw, "a.tlog");
            picker_test_support::click(&ctx, &mut draw, "z.TLOG");
            picker_test_support::click(&ctx, &mut draw, "Open");
        }
        assert_eq!(result, Some(Some(vec![root.join("a.tlog"), root.join("z.TLOG")])));
        for name in ["a.tlog", "z.TLOG", "ignore.txt"] { std::fs::remove_file(root.join(name)).unwrap(); }
        std::fs::remove_dir(root.join("directory.tlog")).unwrap(); std::fs::remove_dir(root).unwrap();
    }
    #[test]
    fn opening_preserves_the_exact_selected_filename_including_leading_space() {
        let root = picker_test_support::root();
        std::fs::write(root.join("a.bin"), b"wrong file").unwrap();
        std::fs::write(root.join(" a.bin"), b"selected file").unwrap();
        let ctx = egui::Context::default();
        let mut picker = FilePicker::in_directory("Logs", root.clone(), &["bin"], None);
        let mut result = None;
        { let mut draw = |ctx: &egui::Context| { if let Some(done) = picker.show(ctx) { result = Some(done); } };
          picker_test_support::click(&ctx, &mut draw, " a.bin"); picker_test_support::click(&ctx, &mut draw, "Open"); }
        let path = result.unwrap().unwrap();
        assert_eq!(std::fs::read(path).unwrap(), b"selected file");
        std::fs::remove_file(root.join("a.bin")).unwrap(); std::fs::remove_file(root.join(" a.bin")).unwrap(); std::fs::remove_dir(root).unwrap();
    }
    #[test]
    fn extensionless_mobile_sequence_save_is_visible_to_load() {
        let id = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos();
        let root = std::env::temp_dir().join(format!("mp-picker-{}-{id}", std::process::id()));
        std::fs::create_dir(&root).unwrap();
        let mut picker = FilePicker::in_directory("Save sequence", root.clone(), &["txt"], Some("formation"));
        let path = picker.selected_path().unwrap();
        assert_eq!(path.file_name().unwrap(), "formation.txt");
        let seq = crate::swarm::Sequence::default();
        seq.save(&path).unwrap();
        assert_eq!(list_files_in(&root, &["txt", "json"]), [path.clone()]);
        assert_eq!(crate::swarm::Sequence::load(&path), Some(seq));
        picker.name = "formation.bad".into(); assert!(picker.selected_path().is_none());
        picker.name = "formation.TXT".into(); assert!(picker.selected_path().is_some());
        std::fs::remove_file(path).unwrap();
        std::fs::remove_dir(root).unwrap();
    }
    #[test]
    fn app_browser_keeps_names_in_its_directory_and_cancel_returns_no_file() {
        let root = std::env::temp_dir().join("mp-picker-no-files");
        let mut picker = FilePicker::in_directory("Save sequence", root.clone(), &["txt"], Some("sequence.txt"));
        assert_eq!(picker.selected_path(), Some(root.join("sequence.txt")));
        for bad in ["", ".", "..", "../other.txt", "a/b.txt", "a\\b.txt", "C:other.txt"] {
            picker.name = bad.into(); assert!(picker.selected_path().is_none(), "{bad}");
        }
        let ctx = egui::Context::default();
        let mut done = None;
        for _ in 0..3 {
            let mut out = ctx.run_ui(egui::RawInput::default(), |ui| { done = picker.show(ui.ctx()); });
            out.textures_delta.clear();
        }
        let mut out = ctx.run_ui(egui::RawInput { events: vec![egui::Event::Key { key: egui::Key::Escape, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() }], ..Default::default() }, |ui| { done = picker.show(ui.ctx()); });
        out.textures_delta.clear();
        assert_eq!(done, Some(None));
    }
    #[test]
    fn escape_cancels_only_the_top_picker_when_requests_overlap() {
        let root = picker_test_support::root();
        let mut first = FilePicker::in_directory("First", root.clone(), &["bin"], None);
        let mut second = FilePicker::in_directory("Second", root.clone(), &["bin"], None);
        assert_ne!(first.id, second.id);
        let ctx = egui::Context::default();
        let (mut a, mut b) = (None, None);
        for _ in 0..3 { picker_test_support::frame(&ctx, &mut |ctx| { a = first.show(ctx); b = second.show(ctx); }, Vec::new()); }
        picker_test_support::frame(&ctx, &mut |ctx| { a = first.show(ctx); b = second.show(ctx); }, vec![egui::Event::Key { key: egui::Key::Escape, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() }]);
        assert!(a.is_none()); assert_eq!(b, Some(None));
        std::fs::remove_dir(root).unwrap();
    }
    #[test]
    fn mobile_document_roots_and_subdirectory_navigation() {
        let root = picker_test_support::root();
        let sub = root.join("missions");
        std::fs::create_dir(&sub).unwrap();
        std::fs::write(sub.join("survey.waypoints"), b"QGC WPL 110\n").unwrap();
        let (dirs, files) = list_entries_in(&root, &["waypoints"]);
        assert_eq!(dirs, vec![sub.clone()]);
        assert!(files.is_empty());

        let mut picker = FilePicker::in_directory("Open Mission", root.clone(), &["waypoints"], None);
        assert_eq!(picker.current_dir, root);
        picker.current_dir = sub.clone();
        picker.name = "survey.waypoints".into();
        assert_eq!(picker.selected_path(), Some(sub.join("survey.waypoints")));

        let roots = document_roots();
        assert!(!roots.is_empty());
        assert_eq!(roots[0].0, "App Storage");

        std::fs::remove_file(sub.join("survey.waypoints")).unwrap();
        std::fs::remove_dir(sub).unwrap();
        std::fs::remove_dir(root).unwrap();
    }
}

/// Returns (file name, contents).
#[cfg(not(any(target_os = "android", target_os = "ios")))]
pub fn open_text(title: &str, filter: &str, exts: &[&str]) -> Option<(String, String)> {
    let path = rfd::FileDialog::new()
        .set_title(title)
        .add_filter(filter, exts)
        .add_filter("All Files", &["*"])
        .pick_file()?;
    let text = std::fs::read_to_string(&path).ok()?;
    Some((path.file_name()?.to_string_lossy().to_string(), text))
}

#[cfg(any(target_os = "android", target_os = "ios"))]
pub fn open_text(_title: &str, _filter: &str, _exts: &[&str]) -> Option<(String, String)> {
    None
}

/// Returns the saved path on success.
#[cfg(not(any(target_os = "android", target_os = "ios")))]
pub fn save_text(
    title: &str,
    default_name: &str,
    filter: &str,
    exts: &[&str],
    contents: &str,
) -> Option<Result<String, String>> {
    let path = rfd::FileDialog::new()
        .set_title(title)
        .set_file_name(default_name)
        .add_filter(filter, exts)
        .save_file()?;
    Some(
        std::fs::write(&path, contents)
            .map(|_| path.display().to_string())
            .map_err(|e| e.to_string()),
    )
}

#[cfg(any(target_os = "android", target_os = "ios"))]
pub fn save_text(
    _title: &str,
    default_name: &str,
    _filter: &str,
    _exts: &[&str],
    contents: &str,
) -> Option<Result<String, String>> {
    let path = files_dir().join(default_name);
    Some(
        std::fs::write(&path, contents)
            .map(|_| path.display().to_string())
            .map_err(|e| e.to_string()),
    )
}

/// Files in the app's directory with an allowed extension.
pub fn list_files(exts: &[&str]) -> Vec<PathBuf> { list_files_in(&files_dir(), exts) }

/// Entries in `dir` (directories and files), sorted.
pub fn list_entries_in(dir: &std::path::Path, exts: &[&str]) -> (Vec<PathBuf>, Vec<PathBuf>) {
    let mut dirs = Vec::new();
    let mut files = Vec::new();
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                if !p.file_name().unwrap_or_default().to_string_lossy().starts_with('.') {
                    dirs.push(p);
                }
            } else if p.is_file() && (exts.is_empty() || exts.contains(&"*") || p.extension().is_some_and(|e| exts.iter().any(|x| e.eq_ignore_ascii_case(x)))) {
                files.push(p);
            }
        }
    }
    dirs.sort();
    files.sort();
    (dirs, files)
}

/// Files in `dir`, sorted. Directories with file-like names are not selectable.
pub fn list_files_in(dir: &std::path::Path, exts: &[&str]) -> Vec<PathBuf> {
    let mut paths: Vec<_> = std::fs::read_dir(dir).into_iter().flatten().filter_map(Result::ok).map(|e| e.path())
        .filter(|p| p.is_file() && (exts.is_empty() || exts.contains(&"*") || p.extension().is_some_and(|e| exts.iter().any(|x| e.eq_ignore_ascii_case(x)))))
        .collect();
    paths.sort(); paths
}

/// Consume a completed deferred action once. Cancel clears it without dispatch.
pub fn poll_file_action<A>(pending: &mut Option<(A, FilePicker)>, ctx: &egui::Context) -> Option<(A, PathBuf)> {
    let result = pending.as_mut()?.1.show(ctx)?;
    let (action, _) = pending.take()?;
    result.map(|path| (action, path))
}

pub fn binary_log_picker() -> FilePicker {
    FilePicker::open_from("Load a Log", data_dir().join("logs"), &["bin"], false)
}

#[cfg(test)]
pub(crate) mod picker_test_support {
    pub fn frame(ctx: &egui::Context, draw: &mut impl FnMut(&egui::Context), events: Vec<egui::Event>) -> Vec<egui::epaint::ClippedShape> {
        let input = egui::RawInput { screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::Vec2::new(1400.0, 900.0))), events, ..Default::default() };
        let mut out = ctx.run_ui(input, |ui| draw(ui.ctx()));
        out.textures_delta.clear(); out.shapes
    }
    pub fn click(ctx: &egui::Context, draw: &mut impl FnMut(&egui::Context), label: &str) {
        let mut shapes = Vec::new();
        for _ in 0..3 { shapes = frame(ctx, draw, Vec::new()); }
        let point = shapes.iter().find_map(|s| match &s.shape {
            egui::Shape::Text(t) if t.galley.job.text == label => Some(t.pos + t.galley.size() / 2.0), _ => None,
        }).unwrap_or_else(|| panic!("visible picker label missing: {label}"));
        for pressed in [true, false] {
            frame(ctx, draw, vec![egui::Event::PointerMoved(point), egui::Event::PointerButton { pos: point, button: egui::PointerButton::Primary, pressed, modifiers: Default::default() }]);
        }
    }
    pub fn root() -> std::path::PathBuf {
        let id = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos();
        let root = std::env::temp_dir().join(format!("mp-file-workflow-{}-{id}", std::process::id()));
        std::fs::create_dir(&root).unwrap(); root
    }
}
