//! Native plugins for Mission Planner RS: the `MissionPlanner.Plugin.Plugin` lifecycle over a C
//! ABI, so plugins can be written in Rust, C or anything that builds a shared library.
//!
//! A plugin library in the `plugins` folder beside the program exports
//! `mp_plugin_entry(api_version, host, out) -> bool`. It fills `out` with its instance and
//! callbacks and keeps `host` for the PluginHost services. The host then calls, as
//! PluginLoader does: `init` (a plugin returning false is dropped), `loaded` once every plugin
//! is initialised, `loop_` at `loop_rate_hz` on the plugin thread (0 disables it), and `exit`
//! when the program closes. `menu_clicked` reports clicks on map-menu items the plugin added.
//!
//! Every callback may run on the UI thread or the plugin thread; host services are thread-safe.
//! Strings returned by `name`/`version`/`author` must stay valid while the plugin is loaded.
use std::ffi::{c_char, c_void};

pub const API_VERSION: u32 = 1;
/// The exported entry point's name.
pub const ENTRY: &[u8] = b"mp_plugin_entry\0";

/// PluginHost.FDMenuMap / FDMenuHud / FPMenuMap
pub const MENU_FD_MAP: u32 = 0;
pub const MENU_FD_HUD: u32 = 1;
pub const MENU_FP_MAP: u32 = 2;

/// PluginHost: services the program gives every plugin.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Host {
    pub ctx: *mut c_void,
    /// Host.cs.<name>: false when the field does not exist.
    pub cs_field: extern "C" fn(ctx: *mut c_void, name: *const c_char, out: *mut f64) -> bool,
    /// Host.config[key] into `buf`: the full length, or -1 when the key is unset.
    pub config_get: extern "C" fn(ctx: *mut c_void, key: *const c_char, buf: *mut c_char, len: usize) -> isize,
    /// Host.config[key] = value
    pub config_set: extern "C" fn(ctx: *mut c_void, key: *const c_char, value: *const c_char),
    /// Host.comPort.doCommand(cmd, p1..p7) on the current vehicle.
    pub do_command: extern "C" fn(ctx: *mut c_void, command: u16, params: *const f32),
    /// Host.AddWPtoList(cmd, p1, p2, p3, p4, x, y, z): the planner row index, or -1.
    pub add_wp: extern "C" fn(ctx: *mut c_void, command: u16, p1: f64, p2: f64, p3: f64, p4: f64, x: f64, y: f64, z: f64) -> i32,
    /// Add a ToolStripMenuItem to one of the MENU_* context menus; clicks call `menu_clicked(id)`.
    pub add_menu: extern "C" fn(ctx: *mut c_void, menu: u32, label: *const c_char, id: u32),
    /// log.Info
    pub log: extern "C" fn(ctx: *mut c_void, message: *const c_char),
}

unsafe impl Send for Host {}
unsafe impl Sync for Host {}

/// A loaded plugin: its instance and the Plugin members.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Plugin {
    pub instance: *mut c_void,
    pub name: extern "C" fn(instance: *mut c_void) -> *const c_char,
    pub version: extern "C" fn(instance: *mut c_void) -> *const c_char,
    pub author: extern "C" fn(instance: *mut c_void) -> *const c_char,
    pub init: extern "C" fn(instance: *mut c_void) -> bool,
    pub loaded: extern "C" fn(instance: *mut c_void) -> bool,
    pub loop_: extern "C" fn(instance: *mut c_void) -> bool,
    pub loop_rate_hz: extern "C" fn(instance: *mut c_void) -> f32,
    pub exit: extern "C" fn(instance: *mut c_void) -> bool,
    /// A map-menu item was clicked at (lat, lng): MouseDownStart / MouseDownEnd.
    pub menu_clicked: extern "C" fn(instance: *mut c_void, id: u32, lat: f64, lng: f64),
}

unsafe impl Send for Plugin {}
unsafe impl Sync for Plugin {}

pub type EntryFn = unsafe extern "C" fn(api_version: u32, host: *const Host, out: *mut Plugin) -> bool;
