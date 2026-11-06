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
            std::env::temp_dir().join("MissionPlannerRS")
        })
        .clone()
}

pub fn files_dir() -> PathBuf {
    let d = data_dir().join("files");
    let _ = std::fs::create_dir_all(&d);
    d
}

pub fn tile_cache_dir() -> PathBuf {
    data_dir().join("tiles")
}

pub const HAS_NATIVE_DIALOGS: bool = cfg!(not(any(target_os = "android", target_os = "ios")));

/// Desktop OS dialog, or a deferred browser of the mobile app's files.
pub struct FilePicker {
    title: String,
    root: PathBuf,
    exts: Vec<String>,
    name: String,
    save: bool,
    overwrite: bool,
    ready: Option<Option<PathBuf>>,
}

impl FilePicker {
    pub fn new(title: &str, exts: &[&str], save_name: Option<&str>) -> Self {
        let mut picker = Self::in_directory(title, files_dir(), exts, save_name);
        #[cfg(not(any(target_os = "android", target_os = "ios")))]
        {
            let dialog = rfd::FileDialog::new().set_title(title).add_filter(title, exts);
            picker.ready = Some(if let Some(name) = save_name { dialog.set_file_name(name).save_file() } else { dialog.pick_file() });
        }
        picker
    }

    fn in_directory(title: &str, root: PathBuf, exts: &[&str], save_name: Option<&str>) -> Self {
        Self { title: title.into(), root, exts: exts.iter().map(|s| s.to_string()).collect(), name: save_name.unwrap_or_default().into(), save: save_name.is_some(), overwrite: false, ready: None }
    }

    fn selected_path(&self) -> Option<PathBuf> {
        let name = self.name.trim();
        if name.is_empty() || name == "." || name == ".." || name.chars().any(|c| matches!(c, '/' | '\\' | ':' | '\0')) { return None; }
        Some(self.root.join(name))
    }

    /// Some(None) means Cancel; None means the browser is still open.
    pub fn show(&mut self, ctx: &egui::Context) -> Option<Option<PathBuf>> {
        if let Some(done) = self.ready.take() { return Some(done); }
        let mut done = None;
        egui::Modal::new(egui::Id::new("app_file_picker")).show(ctx, |ui| {
            ui.set_min_width(280.0);
            crate::theme::label(ui, &self.title);
            let exts: Vec<_> = self.exts.iter().map(String::as_str).collect();
            egui::ScrollArea::vertical().max_height(220.0).show(ui, |ui| {
                let files = list_files_in(&self.root, &exts);
                if files.is_empty() { crate::theme::label(ui, "No files"); }
                for path in files {
                    let name = path.file_name().unwrap_or_default().to_string_lossy().into_owned();
                    if ui.selectable_label(self.name == name, &name).clicked() { self.name = name; self.overwrite = false; }
                }
            });
            if self.save && ui.text_edit_singleline(&mut self.name).changed() { self.overwrite = false; }
            let path = self.selected_path();
            let valid = path.as_ref().is_some_and(|p| self.save || p.is_file());
            ui.horizontal(|ui| {
                let text = if self.overwrite { "Overwrite" } else if self.save { "Save" } else { "Open" };
                if ui.add_enabled(valid, egui::Button::new(text)).clicked() {
                    if self.save && path.as_ref().is_some_and(|p| p.exists()) && !self.overwrite { self.overwrite = true; }
                    else { done = Some(path); }
                }
                if crate::theme::button(ui, "Cancel").clicked() { done = Some(None); }
            });
        });
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) { done = Some(None); }
        done
    }
}

#[cfg(test)]
mod file_picker_tests {
    use super::*;
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
        let mut out = ctx.run_ui(egui::RawInput { events: vec![egui::Event::Key { key: egui::Key::Escape, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() }], ..Default::default() }, |ui| { done = picker.show(ui.ctx()); });
        out.textures_delta.clear();
        assert_eq!(done, Some(None));
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

/// Files in the app's files directory with one of the given extensions (for the in-app browser).
pub fn list_files(exts: &[&str]) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(files_dir())
        .map(|rd| {
            rd.filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| {
                    p.extension()
                        .map(|e| exts.iter().any(|x| e.eq_ignore_ascii_case(x)))
                        .unwrap_or(false)
                })
                .collect()
        })
        .unwrap_or_default();
    v.sort();
    v
}

/// Files in `dir` with one of the given extensions, sorted.
pub fn list_files_in(dir: &std::path::Path, exts: &[&str]) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().map(|e| exts.iter().any(|x| e.eq_ignore_ascii_case(x))).unwrap_or(false))
                .collect()
        })
        .unwrap_or_default();
    v.sort();
    v
}
