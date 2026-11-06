//! An example plugin. It counts its Loop calls in config "example_loops", records the
//! altitude it saw in "example_alt", and adds a "Example: Add WP Here" item to the PLAN map menu
//! that inserts a waypoint and asks the vehicle for its home position.
use mp_plugin::{Host, Plugin, API_VERSION, MENU_FP_MAP};
use std::ffi::{c_char, c_void, CString};

struct Example {
    host: Host,
    loops: u32,
}

fn cstr(s: &str) -> CString {
    CString::new(s).unwrap()
}

extern "C" fn name(_: *mut c_void) -> *const c_char {
    c"Example".as_ptr()
}
extern "C" fn version(_: *mut c_void) -> *const c_char {
    c"0.1".as_ptr()
}
extern "C" fn author(_: *mut c_void) -> *const c_char {
    c"Mission Planner RS".as_ptr()
}

fn me<'a>(i: *mut c_void) -> &'a mut Example {
    unsafe { &mut *(i as *mut Example) }
}

extern "C" fn init(i: *mut c_void) -> bool {
    let e = me(i);
    (e.host.log)(e.host.ctx, c"example plugin init".as_ptr());
    true
}

extern "C" fn loaded(i: *mut c_void) -> bool {
    let e = me(i);
    (e.host.add_menu)(e.host.ctx, MENU_FP_MAP, c"Example: Add WP Here".as_ptr(), 7);
    true
}

extern "C" fn loop_(i: *mut c_void) -> bool {
    let e = me(i);
    e.loops += 1;
    let mut alt = 0.0;
    if (e.host.cs_field)(e.host.ctx, c"alt".as_ptr(), &mut alt) {
        (e.host.config_set)(e.host.ctx, c"example_alt".as_ptr(), cstr(&alt.to_string()).as_ptr());
    }
    (e.host.config_set)(e.host.ctx, c"example_loops".as_ptr(), cstr(&e.loops.to_string()).as_ptr());
    true
}

extern "C" fn rate(_: *mut c_void) -> f32 {
    10.0
}

extern "C" fn exit(i: *mut c_void) -> bool {
    let e = me(i);
    (e.host.config_set)(e.host.ctx, c"example_exit".as_ptr(), c"True".as_ptr());
    unsafe { drop(Box::from_raw(i as *mut Example)) };
    true
}

extern "C" fn menu_clicked(i: *mut c_void, id: u32, lat: f64, lng: f64) {
    let e = me(i);
    if id == 7 {
        (e.host.add_wp)(e.host.ctx, 16, 0.0, 0.0, 0.0, 0.0, lat, lng, 25.0);
        // MAV_CMD_REQUEST_MESSAGE HOME_POSITION
        let params = [242.0f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        (e.host.do_command)(e.host.ctx, 512, params.as_ptr());
    }
}

/// # Safety
/// Called by the host with valid `host` and `out` pointers.
#[no_mangle]
pub unsafe extern "C" fn mp_plugin_entry(api_version: u32, host: *const Host, out: *mut Plugin) -> bool {
    if api_version != API_VERSION || host.is_null() || out.is_null() {
        return false;
    }
    let instance = Box::into_raw(Box::new(Example { host: *host, loops: 0 })) as *mut c_void;
    *out = Plugin { instance, name, version, author, init, loaded, loop_, loop_rate_hz: rate, exit, menu_clicked };
    true
}
