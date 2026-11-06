//! `Plugin/PluginLoader.cs` for native plugins (see the `mp-plugin` crate): LoadAll from the
//! `plugins` folder beside the program, Init then Loaded, Loop on the plugin thread at each
//! plugin's loopratehz, Exit at shutdown, DisabledPlugins and ErrorInfo for PluginUI.
use mp_plugin::{Host, Plugin, API_VERSION, ENTRY};
use std::collections::{BTreeMap, HashMap};
use std::ffi::{c_char, c_void, CStr};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

/// What the program and the plugins exchange between frames.
#[derive(Default)]
pub struct HostState {
    pub fields: HashMap<String, f64>,
    pub config: BTreeMap<String, String>,
    pub config_changes: Vec<(String, String)>,
    pub commands: Vec<(u16, [f32; 7])>,
    pub waypoints: Vec<(u16, [f64; 7])>,
    /// (menu, label, plugin index, id)
    pub menus: Vec<(u32, String, usize, u32)>,
    pub planner_rows: usize,
    pub log: Vec<String>,
}

struct Ctx {
    state: Arc<Mutex<HostState>>,
    plugin: usize,
}

fn ctx<'a>(c: *mut c_void) -> &'a Ctx {
    unsafe { &*(c as *const Ctx) }
}

fn text(p: *const c_char) -> String {
    if p.is_null() {
        String::new()
    } else {
        unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned()
    }
}

extern "C" fn cs_field(c: *mut c_void, name: *const c_char, out: *mut f64) -> bool {
    match ctx(c).state.lock().unwrap().fields.get(&text(name)) {
        Some(v) if !out.is_null() => {
            unsafe { *out = *v };
            true
        }
        _ => false,
    }
}

extern "C" fn config_get(c: *mut c_void, key: *const c_char, buf: *mut c_char, len: usize) -> isize {
    let s = ctx(c).state.lock().unwrap();
    let Some(v) = s.config.get(&text(key)) else { return -1 };
    let bytes = v.as_bytes();
    if !buf.is_null() && len > 0 {
        let n = bytes.len().min(len - 1);
        unsafe {
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), buf as *mut u8, n);
            *buf.add(n) = 0;
        }
    }
    bytes.len() as isize
}

extern "C" fn config_set(c: *mut c_void, key: *const c_char, value: *const c_char) {
    let (k, v) = (text(key), text(value));
    let mut s = ctx(c).state.lock().unwrap();
    s.config.insert(k.clone(), v.clone());
    s.config_changes.push((k, v));
}

extern "C" fn do_command(c: *mut c_void, command: u16, params: *const f32) {
    let mut p = [0f32; 7];
    if !params.is_null() {
        p.copy_from_slice(unsafe { std::slice::from_raw_parts(params, 7) });
    }
    ctx(c).state.lock().unwrap().commands.push((command, p));
}

extern "C" fn add_wp(c: *mut c_void, command: u16, p1: f64, p2: f64, p3: f64, p4: f64, x: f64, y: f64, z: f64) -> i32 {
    let mut s = ctx(c).state.lock().unwrap();
    s.waypoints.push((command, [p1, p2, p3, p4, x, y, z]));
    s.planner_rows += 1;
    s.planner_rows as i32 - 1
}

extern "C" fn add_menu(c: *mut c_void, menu: u32, label: *const c_char, id: u32) {
    let c = ctx(c);
    c.state.lock().unwrap().menus.push((menu, text(label), c.plugin, id));
}

extern "C" fn log_message(c: *mut c_void, message: *const c_char) {
    let m = text(message);
    log::info!("plugin: {m}");
    ctx(c).state.lock().unwrap().log.push(m);
}

pub struct Loaded {
    pub file: String,
    pub name: String,
    pub version: String,
    pub author: String,
    api: Plugin,
    next_run: Instant,
    _host: Box<Host>,
    _ctx: Box<Ctx>,
    _lib: libloading::Library,
}

#[derive(Default)]
pub struct Loader {
    pub plugins: Vec<Loaded>,
    pub disabled: Vec<String>,
    pub errors: BTreeMap<String, String>,
    pub restart_required: bool,
    pub dir: PathBuf,
    pub state: Arc<Mutex<HostState>>,
}

fn is_library(p: &Path) -> bool {
    p.extension().and_then(|e| e.to_str()).is_some_and(|e| ["dll", "so", "dylib"].contains(&e.to_ascii_lowercase().as_str()))
}

impl Loader {
    /// PluginLoader.Load: one library, Init, and its place in the list.
    fn load(&mut self, path: &Path) {
        let file = path.file_name().map(|f| f.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
        if self.disabled.iter().any(|d| d.eq_ignore_ascii_case(&file)) {
            return;
        }
        let lib = match unsafe { libloading::Library::new(path) } {
            Ok(l) => l,
            Err(e) => {
                self.errors.insert(file, e.to_string());
                return;
            }
        };
        let entry: mp_plugin::EntryFn = match unsafe { lib.get::<mp_plugin::EntryFn>(ENTRY) } {
            Ok(f) => *f,
            Err(_) => {
                self.errors.insert(file, "Not a Mission Planner RS plugin (no mp_plugin_entry). .NET plugin assemblies cannot be loaded by this build.".into());
                return;
            }
        };
        let index = self.plugins.len();
        let ctx = Box::new(Ctx { state: self.state.clone(), plugin: index });
        let host = Box::new(Host { ctx: &*ctx as *const Ctx as *mut c_void, cs_field, config_get, config_set, do_command, add_wp, add_menu, log: log_message });
        let mut api = std::mem::MaybeUninit::<Plugin>::uninit();
        if !unsafe { entry(API_VERSION, &*host, api.as_mut_ptr()) } {
            self.errors.insert(file, format!("mp_plugin_entry refused API version {API_VERSION}"));
            return;
        }
        let api = unsafe { api.assume_init() };
        // InitPlugin: only plugins whose Init returns true are kept
        if !(api.init)(api.instance) {
            return;
        }
        let s = |f: extern "C" fn(*mut c_void) -> *const c_char| text(f(api.instance));
        self.plugins.push(Loaded { file, name: s(api.name), version: s(api.version), author: s(api.author), api, next_run: Instant::now(), _host: host, _ctx: ctx, _lib: lib });
    }

    /// LoadAll: every library in the folder, then PluginInit (Loaded); .cs sources are reported.
    pub fn load_all(dir: &Path, disabled: Vec<String>) -> Self {
        let mut l = Loader::default();
        l.disabled = disabled;
        l.dir = dir.to_path_buf();
        let mut files: Vec<PathBuf> = std::fs::read_dir(dir).map(|r| r.flatten().map(|e| e.path()).collect()).unwrap_or_default();
        files.sort();
        for f in &files {
            if f.extension().is_some_and(|e| e.eq_ignore_ascii_case("cs")) {
                let name = f.file_name().unwrap().to_string_lossy().to_ascii_lowercase();
                if !l.disabled.contains(&name) {
                    l.errors.insert(name, "C# source plugins are compiled by Mission Planner's .NET runtime (CodeGenRoslyn) and cannot be built by this program.".into());
                }
            } else if is_library(f) {
                l.load(f);
            }
        }
        // PluginInit: a plugin whose Loaded fails is not kept
        let mut kept = Vec::new();
        for p in std::mem::take(&mut l.plugins) {
            if (p.api.loaded)(p.api.instance) {
                kept.push(p);
            }
        }
        l.plugins = kept;
        l
    }

    /// The plugin thread's pass: Loop for each plugin due at its loopratehz.
    pub fn run_loops(&mut self) -> Instant {
        let now = Instant::now();
        let mut next = now + Duration::from_millis(100);
        for p in &mut self.plugins {
            let hz = (p.api.loop_rate_hz)(p.api.instance);
            if hz > 0.0 && now >= p.next_run {
                (p.api.loop_)(p.api.instance);
                p.next_run = now + Duration::from_secs_f32(1.0 / hz);
            }
            if hz > 0.0 {
                next = next.min(p.next_run);
            }
        }
        next
    }

    pub fn click(&self, plugin: usize, id: u32, lat: f64, lng: f64) {
        if let Some(p) = self.plugins.get(plugin) {
            (p.api.menu_clicked)(p.api.instance, id, lat, lng);
        }
    }

    /// MainV2 closing: Exit for every plugin.
    pub fn exit_all(&mut self) {
        for p in self.plugins.drain(..) {
            (p.api.exit)(p.api.instance);
        }
    }
}

impl Drop for Loader {
    fn drop(&mut self) {
        self.exit_all();
    }
}

/// The program's loader and its plugin thread.
static LOADER: OnceLock<Arc<Mutex<Loader>>> = OnceLock::new();
static STOP: AtomicBool = AtomicBool::new(false);

/// Settings.GetRunningDirectory() + "plugins"
pub fn plugin_dir() -> PathBuf {
    std::env::var_os("MP_PLUGIN_DIR")
        .map(PathBuf::from)
        .or_else(|| std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("plugins"))))
        .unwrap_or_else(|| PathBuf::from("plugins"))
}

/// Settings "DisabledPlugins" (SetList: comma separated).
pub fn disabled_from(planner: &BTreeMap<String, String>) -> Vec<String> {
    planner.get("DisabledPlugins").map(|v| v.split(',').map(|s| s.trim().to_ascii_lowercase()).filter(|s| !s.is_empty()).collect()).unwrap_or_default()
}

pub fn start(planner: &BTreeMap<String, String>) {
    if LOADER.get().is_some() {
        return;
    }
    let loader = Arc::new(Mutex::new(Loader::load_all(&plugin_dir(), disabled_from(planner))));
    let _ = LOADER.set(loader.clone());
    std::thread::Builder::new()
        .name("plugin thread".into())
        .spawn(move || {
            while !STOP.load(Ordering::Relaxed) {
                let next = loader.lock().unwrap().run_loops();
                std::thread::sleep(next.saturating_duration_since(Instant::now()).clamp(Duration::from_millis(1), Duration::from_millis(100)));
            }
        })
        .ok();
}

pub fn with<R>(f: impl FnOnce(&mut Loader) -> R) -> Option<R> {
    LOADER.get().map(|l| f(&mut l.lock().unwrap()))
}

/// Per frame: publish CurrentState and config, apply what plugins asked for.
pub fn sync(fields: &[(&'static str, f64)], planner: &mut BTreeMap<String, String>, planner_rows: usize) -> (Vec<(u16, [f32; 7])>, Vec<(u16, [f64; 7])>) {
    let Some(state) = with(|l| l.state.clone()) else { return Default::default() };
    let mut s = state.lock().unwrap();
    s.fields = fields.iter().map(|(k, v)| ((*k).to_string(), *v)).collect();
    for (k, v) in s.config_changes.drain(..) {
        planner.insert(k, v);
    }
    s.config = planner.clone();
    s.planner_rows = planner_rows;
    (std::mem::take(&mut s.commands), std::mem::take(&mut s.waypoints))
}

/// The ToolStripMenuItems plugins added to a context menu: (label, plugin, id).
pub fn menu_items(menu: u32) -> Vec<(String, usize, u32)> {
    with(|l| l.state.lock().unwrap().menus.iter().filter(|m| m.0 == menu).map(|m| (m.1.clone(), m.2, m.3)).collect()).unwrap_or_default()
}

pub fn click(plugin: usize, id: u32, lat: f64, lng: f64) {
    with(|l| l.click(plugin, id, lat, lng));
}

pub fn shutdown() {
    STOP.store(true, Ordering::Relaxed);
    with(|l| l.exit_all());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The example plugin built by cargo beside the test binary.
    fn example() -> Option<PathBuf> {
        let deps = std::env::current_exe().ok()?.parent()?.to_path_buf();
        let name = if cfg!(windows) { "mp_plugin_example.dll" } else if cfg!(target_os = "macos") { "libmp_plugin_example.dylib" } else { "libmp_plugin_example.so" };
        [deps.join(name), deps.parent()?.join(name)].into_iter().find(|p| p.exists())
    }

    #[test]
    fn lifecycle_loop_rate_services_menus_and_exit() {
        let Some(lib) = example() else {
            panic!("build the example first: cargo build -p mp-plugin-example");
        };
        let dir = std::env::temp_dir().join(format!("mp-plugins-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::copy(&lib, dir.join(lib.file_name().unwrap())).unwrap();
        std::fs::write(dir.join("Legacy.cs"), "class X {}").unwrap();
        std::fs::write(dir.join("notaplugin.dll"), b"not a library").unwrap();
        let mut l = Loader::load_all(&dir, vec![]);
        assert_eq!(l.plugins.len(), 1, "{:?}", l.errors);
        let p = &l.plugins[0];
        assert_eq!((p.name.as_str(), p.version.as_str(), p.author.as_str()), ("Example", "0.1", "Mission Planner RS"));
        assert!(l.errors.contains_key("legacy.cs") && l.errors.contains_key("notaplugin.dll"));
        assert_eq!(l.state.lock().unwrap().log, ["example plugin init"]);
        // Loaded added its PLAN map menu item
        assert_eq!(l.state.lock().unwrap().menus, [(mp_plugin::MENU_FP_MAP, "Example: Add WP Here".to_string(), 0, 7)]);
        // Loop runs at 10 Hz with the host's CurrentState
        l.state.lock().unwrap().fields.insert("alt".into(), 42.5);
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(550) {
            let next = l.run_loops();
            std::thread::sleep(next.saturating_duration_since(Instant::now()));
        }
        let loops: u32 = l.state.lock().unwrap().config["example_loops"].parse().unwrap();
        assert!((4..=8).contains(&loops), "{loops} loops in 550 ms at 10 Hz");
        assert_eq!(l.state.lock().unwrap().config["example_alt"], "42.5");
        // the menu click adds a waypoint and sends a command
        l.click(0, 7, -35.1, 149.2);
        {
            let s = l.state.lock().unwrap();
            assert_eq!(s.waypoints, [(16, [0.0, 0.0, 0.0, 0.0, -35.1, 149.2, 25.0])]);
            assert_eq!(s.commands, [(512, [242.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]);
        }
        let state = l.state.clone();
        l.exit_all();
        assert_eq!(state.lock().unwrap().config["example_exit"], "True");
        // a disabled plugin is not loaded
        let l2 = Loader::load_all(&dir, vec![lib.file_name().unwrap().to_string_lossy().to_ascii_lowercase()]);
        assert!(l2.plugins.is_empty());
        drop(l2);
        std::fs::remove_dir_all(&dir).ok();
    }
}
