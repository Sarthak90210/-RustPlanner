//! `ExtLibs/Utilities/GStreamer.cs`: the HUD's GStreamer video. Like the original, GStreamer is
//! found at runtime (LookForGstreamer), loaded dynamically, and driven through an appsink named
//! `outsink`; nothing links against it at build time. DownloadGStreamer fetches the same runtime
//! zip into the data directory when none is installed.
use std::ffi::{c_char, c_int, c_uint, c_void, CStr, CString};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// FlightData "Set GStreamer Source" default (Settings "gstreamer_url").
pub const DEFAULT_URL: &str = "videotestsrc ! video/x-raw, width=1280, height=720, framerate=30/1 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink";

/// HereLinkVideoToolStripMenuItem_Click pipeline for an IP.
pub fn herelink_url(ip: &str) -> String {
    format!("rtspsrc location=rtsp://{ip}:8554/fpv_stream latency=1 udp-reconnect=1 timeout=0 do-retransmission=false ! application/x-rtp ! rtph264depay ! h264parse ! queue ! avdec_h264 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRx ! appsink name=outsink")
}

fn library_names() -> &'static [&'static str] {
    if cfg!(target_os = "android") {
        &["libgstreamer_android.so"]
    } else if cfg!(windows) {
        &["libgstreamer-1.0-0.dll"]
    } else if cfg!(target_os = "macos") {
        &["libgstreamer-1.0.0.dylib"]
    } else {
        &["libgstreamer-1.0.so.0"]
    }
}

fn find_in(dir: &Path, depth: usize, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for e in entries.flatten() {
        let p = e.path();
        if p.is_dir() {
            if depth > 0 {
                find_in(&p, depth - 1, out);
            }
        } else if let Some(name) = p.file_name().and_then(|n| n.to_str()) {
            let lower = name.to_ascii_lowercase();
            if library_names().iter().any(|l| lower.contains(l)) {
                out.push(p);
            }
        }
    }
}

/// LookForGstreamer: the directories the original searches, newest match first by the
/// process bitness rule (`_64` in the path for 64-bit Windows builds).
pub fn look_for_gstreamer(data_dir: &Path) -> Option<PathBuf> {
    let mut dirs: Vec<PathBuf> = vec!["/usr/lib/x86_64-linux-gnu".into(), "/usr/lib/arm-linux-gnueabihf".into(), "/usr/lib/aarch64-linux-gnu".into()];
    if let Some(exe) = std::env::current_exe().ok().and_then(|p| p.parent().map(Path::to_path_buf)) {
        dirs.push(exe);
    }
    dirs.push(data_dir.to_path_buf());
    // developer switch for captures: another directory holding a runtime
    if let Some(extra) = std::env::var_os("MP_GST_DIR") {
        dirs.push(extra.into());
    }
    if cfg!(target_os = "macos") {
        dirs.push("/Library/Frameworks/GStreamer.framework/Versions/1.0/lib".into());
    }
    if cfg!(windows) {
        for drive in b'C'..=b'Z' {
            let root = format!("{}:\\", drive as char);
            if Path::new(&root).exists() {
                dirs.push(format!("{root}gstreamer").into());
                dirs.push(format!("{root}Program Files\\gstreamer").into());
                dirs.push(format!("{root}Program Files (x86)\\gstreamer").into());
            }
        }
    }
    let is64 = cfg!(target_pointer_width = "64");
    for dir in dirs.iter().filter(|d| d.is_dir()) {
        let mut found = Vec::new();
        find_in(dir, 6, &mut found);
        found.retain(|p| {
            let s = p.to_string_lossy().to_ascii_lowercase();
            if !cfg!(windows) {
                return true;
            }
            (!is64 && !s.contains("_64")) || (is64 && s.contains("_64"))
        });
        if let Some(p) = found.into_iter().next() {
            return Some(p);
        }
    }
    None
}

/// SetGSTPath: put the runtime's bin/lib first on PATH and point the plugin scanner at it.
fn set_gst_path(lib: &Path) {
    let Some(root) = lib.parent().and_then(Path::parent) else { return };
    let (bin, libdir) = (root.join("bin"), root.join("lib"));
    let sep = if cfg!(windows) { ";" } else { ":" };
    let path = std::env::var("PATH").unwrap_or_default();
    if !path.contains(&*root.to_string_lossy()) {
        std::env::set_var("PATH", format!("{}{sep}{}{sep}{path}", bin.display(), libdir.display()));
    }
    std::env::set_var("GSTREAMER_ROOT", root);
    std::env::set_var("GSTREAMER_1_0_ROOT_X86_64", root);
    if libdir.join("gstreamer-1.0").is_dir() {
        std::env::set_var("GST_PLUGIN_PATH", libdir.join("gstreamer-1.0"));
    } else {
        std::env::set_var("GST_PLUGIN_PATH", &libdir);
    }
}

/// DownloadGStreamer: the original's runtime zip, extracted into the data directory.
pub fn download(data_dir: &Path, status: &dyn Fn(i32, &str)) -> Result<(), String> {
    download_cancellable(data_dir, status, &AtomicBool::new(false))
}

/// DownloadGStreamer with the progress dialog's Cancel ("User Request").
pub fn download_cancellable(data_dir: &Path, status: &dyn Fn(i32, &str), cancel: &AtomicBool) -> Result<(), String> {
    if cancel.load(Ordering::Relaxed) {
        status(-1, "User Request");
        return Err("User Request".into());
    }
    if cfg!(any(target_arch = "arm", target_arch = "aarch64")) {
        return Err("No GStreamer download for this architecture".into());
    }
    let (name, url) = if cfg!(target_pointer_width = "64") {
        ("gstreamer-1.0-x86_64-1.14.4.zip", "https://firmware.ardupilot.org/MissionPlanner/gstreamer/gstreamer-1.0-x86_64-1.14.4.zip")
    } else {
        ("gstreamer-1.0-x86-1.14.4.zip", "https://firmware.ardupilot.org/MissionPlanner/gstreamer/gstreamer-1.0-x86-1.14.4.zip")
    };
    std::fs::create_dir_all(data_dir).map_err(|e| e.to_string())?;
    let output = data_dir.join(name);
    status(0, "Downloading..");
    let mut last = String::new();
    for _ in 0..3 {
        let attempt = (|| -> Result<(), String> {
            if cancel.load(Ordering::Relaxed) {
                return Err("User Request".into());
            }
            let resp = ureq::get(url).call().map_err(|e| e.to_string())?;
            let total = resp.headers().get("content-length").and_then(|v| v.to_str().ok()).and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
            let mut reader = resp.into_body().into_reader();
            let mut file = std::fs::File::create(&output).map_err(|e| e.to_string())?;
            let mut buf = vec![0u8; 1 << 16];
            let mut done = 0u64;
            loop {
                if cancel.load(Ordering::Relaxed) {
                    drop(file);
                    let _ = std::fs::remove_file(&output);
                    return Err("User Request".into());
                }
                let n = std::io::Read::read(&mut reader, &mut buf).map_err(|e| e.to_string())?;
                if n == 0 {
                    break;
                }
                std::io::Write::write_all(&mut file, &buf[..n]).map_err(|e| e.to_string())?;
                done += n as u64;
                if total > 0 {
                    status((done * 50 / total) as i32, "Downloading..");
                }
            }
            status(50, "Extracting..");
            let zip = std::fs::File::open(&output).map_err(|e| e.to_string())?;
            zip::ZipArchive::new(zip).and_then(|mut a| a.extract(data_dir)).map_err(|e| e.to_string())?;
            status(100, "Done.");
            Ok(())
        })();
        match attempt {
            Ok(()) => return Ok(()),
            Err(e) if e == "User Request" => {
                status(-1, &e);
                return Err(e);
            }
            Err(e) => {
                status(-1, &format!("Error downloading file {e}"));
                let _ = std::fs::remove_file(&output);
                last = e;
            }
        }
    }
    Err(last)
}

#[repr(C)]
struct GError {
    domain: u32,
    code: c_int,
    message: *const c_char,
}

#[repr(C)]
struct GstMapInfo {
    memory: *mut c_void,
    flags: c_int,
    data: *mut u8,
    size: usize,
    maxsize: usize,
    user_data: [*mut c_void; 4],
    reserved: [*mut c_void; 4],
}

type P = *mut c_void;
const GST_STATE_NULL: c_int = 1;
const GST_STATE_PLAYING: c_int = 4;
const GST_STATE_CHANGE_FAILURE: c_int = 0;
const GST_MESSAGE_EOS: c_int = 1 << 0;
const GST_MESSAGE_ERROR: c_int = 1 << 1;
const GST_SECOND: u64 = 1_000_000_000;

struct Api {
    _core: libloading::Library,
    _app: Option<libloading::Library>,
    init: unsafe extern "C" fn(*mut c_int, *mut c_void),
    version: unsafe extern "C" fn(*mut c_uint, *mut c_uint, *mut c_uint, *mut c_uint),
    parse_launch: unsafe extern "C" fn(*const c_char, *mut *mut GError) -> P,
    bin_get_by_name: unsafe extern "C" fn(P, *const c_char) -> P,
    set_state: unsafe extern "C" fn(P, c_int) -> c_int,
    get_bus: unsafe extern "C" fn(P) -> P,
    bus_pop: unsafe extern "C" fn(P, u64, c_int) -> P,
    object_unref: unsafe extern "C" fn(P),
    mini_object_unref: unsafe extern "C" fn(P),
    sample_get_caps: unsafe extern "C" fn(P) -> P,
    sample_get_buffer: unsafe extern "C" fn(P) -> P,
    caps_get_structure: unsafe extern "C" fn(P, c_uint) -> P,
    structure_get_int: unsafe extern "C" fn(P, *const c_char, *mut c_int) -> c_int,
    buffer_map: unsafe extern "C" fn(P, *mut GstMapInfo, c_int) -> c_int,
    buffer_unmap: unsafe extern "C" fn(P, *mut GstMapInfo),
    sink_set_drop: unsafe extern "C" fn(P, c_int),
    sink_set_max_buffers: unsafe extern "C" fn(P, c_uint),
    sink_is_eos: unsafe extern "C" fn(P) -> c_int,
    sink_try_pull_sample: unsafe extern "C" fn(P, u64) -> P,
}

impl Api {
    /// The core library and libgstapp beside it (Android bundles both in one library).
    unsafe fn load(lib: &Path) -> Result<Self, String> {
        set_gst_path(lib);
        let core = libloading::Library::new(lib).map_err(|e| format!("The file was not found at {}\nPlease verify permissions {e}", lib.display()))?;
        let dir = lib.parent().unwrap_or(Path::new("."));
        let app_name = if cfg!(windows) {
            "libgstapp-1.0-0.dll"
        } else if cfg!(target_os = "macos") {
            "libgstapp-1.0.0.dylib"
        } else {
            "libgstapp-1.0.so.0"
        };
        let app = if cfg!(target_os = "android") { None } else { Some(libloading::Library::new(dir.join(app_name)).or_else(|_| libloading::Library::new(app_name)).map_err(|e| e.to_string())?) };
        macro_rules! sym {
            ($lib:expr, $name:literal) => {
                *$lib.get(concat!($name, "\0").as_bytes()).map_err(|e| format!("{}: {e}", $name))?
            };
        }
        let applib = app.as_ref().unwrap_or(&core);
        Ok(Self {
            init: sym!(core, "gst_init"),
            version: sym!(core, "gst_version"),
            parse_launch: sym!(core, "gst_parse_launch"),
            bin_get_by_name: sym!(core, "gst_bin_get_by_name"),
            set_state: sym!(core, "gst_element_set_state"),
            get_bus: sym!(core, "gst_element_get_bus"),
            bus_pop: sym!(core, "gst_bus_timed_pop_filtered"),
            object_unref: sym!(core, "gst_object_unref"),
            mini_object_unref: sym!(core, "gst_mini_object_unref"),
            sample_get_caps: sym!(core, "gst_sample_get_caps"),
            sample_get_buffer: sym!(core, "gst_sample_get_buffer"),
            caps_get_structure: sym!(core, "gst_caps_get_structure"),
            structure_get_int: sym!(core, "gst_structure_get_int"),
            buffer_map: sym!(core, "gst_buffer_map"),
            buffer_unmap: sym!(core, "gst_buffer_unmap"),
            sink_set_drop: sym!(applib, "gst_app_sink_set_drop"),
            sink_set_max_buffers: sym!(applib, "gst_app_sink_set_max_buffers"),
            sink_is_eos: sym!(applib, "gst_app_sink_is_eos"),
            sink_try_pull_sample: sym!(applib, "gst_app_sink_try_pull_sample"),
            _core: core,
            _app: app,
        })
    }
}

/// BGRA/BGRx appsink frames as RGBA (BGRx's padding byte becomes opaque alpha).
pub fn bgra_to_rgba(data: &[u8], width: usize, height: usize) -> Option<egui::ColorImage> {
    if width == 0 || height == 0 || data.len() < width * height * 4 {
        return None;
    }
    let mut rgba = Vec::with_capacity(width * height * 4);
    for px in data[..width * height * 4].chunks_exact(4) {
        rgba.extend_from_slice(&[px[2], px[1], px[0], 255]);
    }
    Some(egui::ColorImage::from_rgba_unmultiplied([width, height], &rgba))
}

/// GStreamer.Start / ThreadStart: run `pipeline` on a worker until `stop`, publishing frames.
pub fn spawn(lib: PathBuf, pipeline: String, frame: Arc<Mutex<Option<egui::ColorImage>>>, stop: Arc<AtomicBool>, error: Arc<Mutex<Option<String>>>, ctx: egui::Context) {
    std::thread::Builder::new()
        .name("gstreamer".into())
        .spawn(move || {
            if let Err(e) = unsafe { run(&lib, &pipeline, &frame, &stop, &ctx) } {
                *error.lock().unwrap() = Some(e);
                ctx.request_repaint();
            }
        })
        .ok();
}

unsafe fn run(lib: &Path, pipeline: &str, frame: &Mutex<Option<egui::ColorImage>>, stop: &AtomicBool, ctx: &egui::Context) -> Result<(), String> {
    std::env::set_var("GST_DEBUG", "*:4");
    let api = Api::load(lib)?;
    (api.init)(std::ptr::null_mut(), std::ptr::null_mut());
    let (mut a, mut b, mut c, mut d) = (0, 0, 0, 0);
    (api.version)(&mut a, &mut b, &mut c, &mut d);
    log::info!("GStreamer {a}.{b}.{c}.{d}");
    let desc = CString::new(pipeline).map_err(|e| e.to_string())?;
    let mut err: *mut GError = std::ptr::null_mut();
    let pipe = (api.parse_launch)(desc.as_ptr(), &mut err);
    if !err.is_null() {
        let msg = CStr::from_ptr((*err).message).to_string_lossy().to_string();
        return Err(format!("gst_parse_launch: {msg}"));
    }
    if pipe.is_null() {
        return Err("gst_parse_launch failed".into());
    }
    let sink = (api.bin_get_by_name)(pipe, c"outsink".as_ptr());
    if !sink.is_null() {
        (api.sink_set_drop)(sink, 1);
        (api.sink_set_max_buffers)(sink, 1);
    }
    let running = (api.set_state)(pipe, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE;
    let bus = (api.get_bus)(pipe);
    let mut run = running;
    if sink.is_null() {
        // no appsink: the pipeline renders itself; wait for error or EOS
        while running && !stop.load(Ordering::Relaxed) {
            let msg = (api.bus_pop)(bus, GST_SECOND / 5, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
            if !msg.is_null() {
                (api.mini_object_unref)(msg);
                break;
            }
        }
        run = false;
    } else {
        let msg = (api.bus_pop)(bus, 0, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
        if !msg.is_null() {
            (api.mini_object_unref)(msg);
            run = false;
        }
    }
    let mut failed = None;
    while run && !stop.load(Ordering::Relaxed) && (api.sink_is_eos)(sink) == 0 {
        let sample = (api.sink_try_pull_sample)(sink, GST_SECOND / 2);
        if sample.is_null() {
            // an error on the bus ends the stream (the original waits out its retries)
            let msg = (api.bus_pop)(bus, 0, GST_MESSAGE_ERROR);
            if !msg.is_null() {
                (api.mini_object_unref)(msg);
                failed = Some("GStreamer pipeline error".to_string());
                break;
            }
            continue;
        }
        let caps = (api.sample_get_caps)(sample);
        let s = (api.caps_get_structure)(caps, 0);
        let (mut w, mut h) = (0, 0);
        (api.structure_get_int)(s, c"width".as_ptr(), &mut w);
        (api.structure_get_int)(s, c"height".as_ptr(), &mut h);
        let buffer = (api.sample_get_buffer)(sample);
        if !buffer.is_null() {
            let mut info: GstMapInfo = std::mem::zeroed();
            if (api.buffer_map)(buffer, &mut info, 1) != 0 {
                let data = std::slice::from_raw_parts(info.data, info.size);
                if let Some(img) = bgra_to_rgba(data, w as usize, h as usize) {
                    *frame.lock().unwrap() = Some(img);
                    ctx.request_repaint();
                }
                (api.buffer_unmap)(buffer, &mut info);
            }
        }
        (api.mini_object_unref)(sample);
    }
    if sink.is_null() || (api.sink_is_eos)(sink) == 0 {
        (api.set_state)(pipe, GST_STATE_NULL);
    }
    if !bus.is_null() {
        (api.object_unref)(bus);
    }
    if !sink.is_null() {
        (api.object_unref)(sink);
    }
    (api.object_unref)(pipe);
    match failed {
        Some(e) => Err(e),
        None => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// End to end with a real runtime: MP_GST_DATA names a data directory; the original's
    /// runtime zip is downloaded there when absent. `cargo test -- --ignored real_runtime`
    #[test]
    #[ignore]
    fn real_runtime_plays_the_default_test_pipeline() {
        let data = PathBuf::from(std::env::var("MP_GST_DATA").expect("MP_GST_DATA"));
        if look_for_gstreamer(&data).is_none() {
            download(&data, &|p, t| eprintln!("{p} {t}")).expect("download");
        }
        let lib = look_for_gstreamer(&data).expect("runtime");
        let frame = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let error = Arc::new(Mutex::new(None));
        spawn(lib, DEFAULT_URL.into(), frame.clone(), stop.clone(), error.clone(), egui::Context::default());
        let end = std::time::Instant::now() + std::time::Duration::from_secs(30);
        let img = loop {
            if let Some(img) = frame.lock().unwrap().take() {
                break img;
            }
            assert!(error.lock().unwrap().is_none(), "{:?}", error.lock().unwrap());
            assert!(std::time::Instant::now() < end, "no frame");
            std::thread::sleep(std::time::Duration::from_millis(50));
        };
        assert_eq!(img.size, [1280, 720]);
        // videotestsrc's first bar is white
        assert_eq!(img.pixels[10], egui::Color32::WHITE, "{:?}", img.pixels[10]);
        stop.store(true, Ordering::Relaxed);
        // a broken pipeline reports its parse error
        let error2 = Arc::new(Mutex::new(None));
        spawn(look_for_gstreamer(&data).unwrap(), "nosuchelement ! appsink name=outsink".into(), Arc::new(Mutex::new(None)), Arc::new(AtomicBool::new(false)), error2.clone(), egui::Context::default());
        let end = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while error2.lock().unwrap().is_none() {
            assert!(std::time::Instant::now() < end, "no parse error");
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        assert!(error2.lock().unwrap().as_ref().unwrap().contains("gst_parse_launch"));
    }

    #[test]
    fn a_cancelled_download_stops_with_user_request() {
        let dir = std::env::temp_dir().join(format!("mp-gst-cancel-{}", std::process::id()));
        let seen = Mutex::new(Vec::new());
        let r = download_cancellable(&dir, &|p, t| seen.lock().unwrap().push((p, t.to_string())), &AtomicBool::new(true));
        assert_eq!(r, Err("User Request".to_string()));
        assert_eq!(seen.lock().unwrap().last(), Some(&(-1, "User Request".to_string())));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn bgrx_frames_become_opaque_rgba() {
        let img = bgra_to_rgba(&[10, 20, 30, 0, 1, 2, 3, 4], 2, 1).unwrap();
        assert_eq!(img.pixels[0], egui::Color32::from_rgba_unmultiplied(30, 20, 10, 255));
        assert_eq!(img.pixels[1], egui::Color32::from_rgb(3, 2, 1));
        assert!(bgra_to_rgba(&[0; 4], 2, 1).is_none());
    }

    #[test]
    fn discovery_finds_a_runtime_in_the_data_directory() {
        let dir = std::env::temp_dir().join(format!("mp-gst-{}", std::process::id()));
        let arch = if cfg!(target_pointer_width = "64") { "x86_64" } else { "x86" };
        let bin = dir.join("gstreamer").join("1.0").join(arch).join("bin");
        std::fs::create_dir_all(&bin).unwrap();
        let name = library_names()[0];
        std::fs::write(bin.join(name), b"").unwrap();
        let found = look_for_gstreamer(&dir).expect("found");
        assert!(found.ends_with(name));
        std::fs::remove_dir_all(&dir).ok();
        assert!(herelink_url("10.0.0.5").starts_with("rtspsrc location=rtsp://10.0.0.5:8554/fpv_stream"));
    }
}
