//! Mission Planner's embedded web service (`Utilities/httpserver.cs`), started with the
//! application on port 56781: Google Earth network links, MAVLink and state websockets,
//! MJPEG HUD/map streams, guided commands, Mavelous JSON/files and the georef overlay.
use mp_core::{
    link::{LinkCommand, LinkShared, LinkStatus},
    mavlink::dialects::ardupilotmega::MavMessage,
    mission::{AltMode, Locationwp},
};
use parking_lot::{Mutex, RwLock};
use std::{
    collections::HashMap,
    fmt::Write as _,
    io::{self, Read, Write},
    net::{IpAddr, SocketAddr, TcpListener, TcpStream},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc, LazyLock,
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

pub const PORT: u16 = 56781;
const KML_TYPE: &str = "application/vnd.google-earth.kml+xml";
const MODEL: &[u8] = include_bytes!("../assets/block_plane_0.dae");
const HUD_HTML: &[u8] = include_bytes!("../assets/hud.html");
const OPEN: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>";
const CLOSE: &str = "</Document></kml>";
const ROOT_OPEN: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\">";
const MAX_CONNECTIONS: usize = 200;
const MAX_PER_IP: usize = 20;
/// The state and MJPEG streams send at 5 Hz.
const STREAM_PERIOD: Duration = Duration::from_millis(200);
const INDEX: &str = "
                <a href=/mav/>Mavelous</a>
<a href=/mavlink/>Mavelous traffic</a>
<a href=/hud.jpg>Hud image</a>
<a href=/map.jpg>Map image </a>
<a href=/both.jpg>Map & hud image</a>
<a href=/hud.html>hud html5</a>
<a href=/network.kml>network kml</a>
<a href=/georefnetwork.kml>georef kml</a>

";

#[derive(Clone, Default)]
pub struct Snapshot {
    pub home: Locationwp,
    pub mission: Vec<Locationwp>,
    /// Active link first, then the other MainV2.Comports.
    pub links: Vec<Arc<RwLock<LinkShared>>>,
    /// MainV2.comPort, for guided targets and raw MAVLink from websocket clients.
    pub command: Option<crossbeam_channel::Sender<LinkCommand>>,
    /// CurrentState.rateattitude, rateposition, ratestatus, raterc, ratesensors
    pub rates: [u16; 5],
}

/// httpserver.georefkml / georefimagepath, set by the georef tool's pre-process.
static GEOREF: LazyLock<RwLock<(String, PathBuf)>> = LazyLock::new(Default::default);

pub fn set_georef(kml: String, image_dir: PathBuf) {
    *GEOREF.write() = (kml, image_dir);
}

/// The latest JPEG frames of the DATA HUD and map for hud.jpg / map.jpg / both.jpg.
#[derive(Default)]
struct Frames {
    hud: Option<Arc<Vec<u8>>>,
    map: Option<Arc<Vec<u8>>>,
    both: Option<Arc<Vec<u8>>>,
    /// a stream is open: the UI keeps capturing until then
    wanted_until: Option<Instant>,
}

static FRAMES: LazyLock<Mutex<Frames>> = LazyLock::new(Default::default);

/// Where the DATA HUD and map were drawn this frame (points), for the image streams.
static REGIONS: LazyLock<Mutex<(Option<egui::Rect>, Option<egui::Rect>, Instant)>> =
    LazyLock::new(|| Mutex::new((None, None, Instant::now())));

pub fn set_hud_region(r: egui::Rect) {
    let mut g = REGIONS.lock();
    g.0 = Some(r);
    g.2 = Instant::now();
}

pub fn set_map_region(r: egui::Rect) {
    let mut g = REGIONS.lock();
    g.1 = Some(r);
    g.2 = Instant::now();
}

/// GetControlJpeg for myhud / mymap: crop the rendered viewport and encode JPEG frames.
/// Called with a renderer screenshot while a stream client is connected.
pub fn publish_screenshot(image: &egui::ColorImage, pixels_per_point: f32) {
    let (hud, map, at) = *REGIONS.lock();
    // regions only count while the DATA view is drawing them
    if at.elapsed() > Duration::from_secs(1) {
        return;
    }
    let crop = |r: egui::Rect| -> Option<image::RgbImage> {
        let [w, h] = image.size;
        let x0 = ((r.min.x * pixels_per_point).round().max(0.0) as usize).min(w);
        let y0 = ((r.min.y * pixels_per_point).round().max(0.0) as usize).min(h);
        let x1 = ((r.max.x * pixels_per_point).round().max(0.0) as usize).min(w);
        let y1 = ((r.max.y * pixels_per_point).round().max(0.0) as usize).min(h);
        (x1 > x0 && y1 > y0).then(|| {
            image::RgbImage::from_fn((x1 - x0) as u32, (y1 - y0) as u32, |x, y| {
                let c = image.pixels[(y0 + y as usize) * w + x0 + x as usize];
                image::Rgb([c.r(), c.g(), c.b()])
            })
        })
    };
    let encode = |img: &image::RgbImage| {
        let mut out = Vec::new();
        img.write_to(&mut io::Cursor::new(&mut out), image::ImageFormat::Jpeg).ok().map(|_| out)
    };
    let hud = hud.and_then(crop);
    let map = map.and_then(crop);
    // both.jpg: HUD at the left, map beside it, as tall as the taller one
    let both = match (&hud, &map) {
        (Some(a), Some(b)) => {
            let mut out = image::RgbImage::new(a.width() + b.width(), a.height().max(b.height()));
            image::imageops::replace(&mut out, a, 0, 0);
            image::imageops::replace(&mut out, b, a.width() as i64, 0);
            Some(out)
        }
        _ => None,
    };
    set_frames(hud.as_ref().and_then(encode), map.as_ref().and_then(encode), both.as_ref().and_then(encode));
}

/// Whether an MJPEG client wants new frames (myhud.streamjpgenable).
pub fn frames_wanted() -> bool {
    FRAMES.lock().wanted_until.is_some_and(|t| t > Instant::now())
}

/// Store new JPEG frames for the HUD and map.
pub fn set_frames(hud: Option<Vec<u8>>, map: Option<Vec<u8>>, both: Option<Vec<u8>>) {
    let mut f = FRAMES.lock();
    if let Some(h) = hud {
        f.hud = Some(Arc::new(h));
    }
    if let Some(m) = map {
        f.map = Some(Arc::new(m));
    }
    if let Some(b) = both {
        f.both = Some(Arc::new(b));
    }
}

struct Limits {
    active: AtomicUsize,
    per_ip: Mutex<HashMap<IpAddr, usize>>,
}

impl Limits {
    /// TryAcquireConnectionSlot: 200 connections in all, 20 per address.
    fn acquire(&self, ip: IpAddr) -> bool {
        if self.active.fetch_add(1, Ordering::SeqCst) >= MAX_CONNECTIONS {
            self.active.fetch_sub(1, Ordering::SeqCst);
            return false;
        }
        let mut per = self.per_ip.lock();
        let n = per.entry(ip).or_insert(0);
        if *n >= MAX_PER_IP {
            self.active.fetch_sub(1, Ordering::SeqCst);
            return false;
        }
        *n += 1;
        true
    }

    fn release(&self, ip: IpAddr) {
        self.active.fetch_sub(1, Ordering::SeqCst);
        let mut per = self.per_ip.lock();
        if let Some(n) = per.get_mut(&ip) {
            *n -= 1;
            if *n == 0 {
                per.remove(&ip);
            }
        }
    }
}

struct Shared {
    port: u16,
    snapshot: RwLock<Snapshot>,
    stop: AtomicBool,
    limits: Limits,
    mavlink_index: AtomicUsize,
}

pub struct HttpServer {
    addr: SocketAddr,
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

/// The application's server (MainV2's httpthread).
static SERVER: LazyLock<Mutex<Option<HttpServer>>> = LazyLock::new(Default::default);

/// MainV2: "start http" on every interface. A second instance fails to bind, as the
/// original logs "Possible multiple instances of planner?".
pub fn start() {
    let mut s = SERVER.lock();
    if s.is_none() {
        match HttpServer::bind(([0, 0, 0, 0], PORT).into(), Snapshot::default()) {
            Ok(server) => *s = Some(server),
            Err(e) => log::error!("Exception starting listener. Possible multiple instances of planner? {e}"),
        }
    }
}

/// Publish the current mission, links and settings to the running server.
pub fn update(snapshot: Snapshot) {
    if let Some(s) = SERVER.lock().as_ref() {
        s.update(snapshot);
    }
}

impl HttpServer {
    pub fn bind(addr: SocketAddr, snapshot: Snapshot) -> io::Result<Self> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        let addr = listener.local_addr()?;
        let shared = Arc::new(Shared {
            port: addr.port(),
            snapshot: RwLock::new(snapshot),
            stop: AtomicBool::new(false),
            limits: Limits { active: AtomicUsize::new(0), per_ip: Mutex::new(HashMap::new()) },
            mavlink_index: AtomicUsize::new(0),
        });
        let data = shared.clone();
        let thread = std::thread::Builder::new().name("motion jpg stream-network kml".into()).spawn(move || {
            while !data.stop.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, peer)) => {
                        if !data.limits.acquire(peer.ip()) {
                            continue;
                        }
                        let conn = data.clone();
                        let spawned = std::thread::Builder::new().name("httpserver client".into()).spawn(move || {
                            let mut stream = stream;
                            let _ = process(&mut stream, peer, &conn);
                            conn.limits.release(peer.ip());
                        });
                        if spawned.is_err() {
                            data.limits.release(peer.ip());
                        }
                    }
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => std::thread::sleep(Duration::from_millis(20)),
                    Err(_) => break,
                }
            }
        })?;
        Ok(Self { addr, shared, thread: Some(thread) })
    }

    pub fn network_url(&self) -> String {
        format!("http://127.0.0.1:{}/network.kml", self.addr.port())
    }

    pub fn update(&self, snapshot: Snapshot) {
        *self.shared.snapshot.write() = snapshot;
    }
}

impl Drop for HttpServer {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// One request header (and any body that arrived with it), like the original's 4 KiB read.
fn read_request(stream: &mut TcpStream) -> io::Result<Option<String>> {
    let mut request = Vec::new();
    let mut buf = [0; 4096];
    while !request.windows(4).any(|b| b == b"\r\n\r\n") {
        let n = match stream.read(&mut buf) {
            Ok(n) => n,
            Err(e) if mp_core::comms::is_timeout(&e) => return Ok(None),
            Err(e) => return Err(e),
        };
        if n == 0 {
            return Ok(None);
        }
        request.extend_from_slice(&buf[..n]);
        if request.len() > 16 * 1024 {
            return Ok(None);
        }
    }
    let text = String::from_utf8_lossy(&request).to_string();
    // a POST body announced by Content-Length
    let header_end = text.find("\r\n\r\n").map(|i| i + 4).unwrap_or(text.len());
    let want = header_value(&text, "Content-Length").and_then(|v| v.parse::<usize>().ok()).unwrap_or(0).min(4096);
    let mut body = request.len() - header_end;
    while body < want {
        match stream.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(n) => {
                request.extend_from_slice(&buf[..n]);
                body += n;
            }
        }
    }
    Ok(Some(String::from_utf8_lossy(&request).to_string()))
}

fn header_value<'a>(head: &'a str, name: &str) -> Option<&'a str> {
    head.lines().skip(1).take_while(|l| !l.is_empty()).find_map(|l| {
        let (k, v) = l.split_once(':')?;
        k.trim().eq_ignore_ascii_case(name).then(|| v.trim())
    })
}

fn is_loopback_origin(value: &str) -> bool {
    let rest = value.split_once("://").map(|(_, r)| r).unwrap_or("");
    let host = rest.split(['/', '?', '#']).next().unwrap_or("");
    let host = if let Some(h) = host.strip_prefix('[') { h.split(']').next().unwrap_or("") } else { host.rsplit_once(':').map(|(h, _)| h).unwrap_or(host) };
    host.eq_ignore_ascii_case("localhost") || host.parse::<IpAddr>().is_ok_and(|a| a.is_loopback())
}

/// IsCrossSiteRequest: a browser request carrying a non-loopback Origin or Referer.
fn is_cross_site(head: &str) -> bool {
    ["Origin", "Referer"].iter().any(|h| header_value(head, h).is_some_and(|v| !v.is_empty() && !is_loopback_origin(v)))
}

fn process(stream: &mut TcpStream, peer: SocketAddr, shared: &Shared) -> io::Result<()> {
    // Accepted sockets inherit the listener's non-blocking mode on Windows.
    stream.set_nonblocking(false)?;
    // 5 seconds - default for httpd 2.2+
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    stream.set_write_timeout(Some(Duration::from_secs(5)))?;
    // keep-alive routes return true to read the next request ("goto again")
    while let Some(head) = read_request(stream)? {
        if !route(stream, peer, shared, &head)? {
            break;
        }
    }
    Ok(())
}

fn route(stream: &mut TcpStream, peer: SocketAddr, shared: &Shared, head: &str) -> io::Result<bool> {
    let line = head.lines().next().unwrap_or("");
    let mut parts = line.split_whitespace();
    let method = parts.next().unwrap_or("").to_string();
    let target = parts.next().unwrap_or("").to_string();
    let path = target.split('?').next().unwrap_or("").to_string();
    let lower = path.to_ascii_lowercase();
    let is_head = method == "HEAD";
    let upgrade = head.lines().any(|l| l.eq_ignore_ascii_case("Upgrade: websocket"));
    let state = shared.snapshot.read().clone();
    let port = shared.port;
    match () {
        _ if path == "/websocket/server" => websocket_server(stream, head, shared).map(|_| false),
        _ if path == "/websocket/raw" || (path == "/" && upgrade) => websocket_raw(stream, head, &state).map(|_| false),
        _ if path == "/georefnetwork.kml" => {
            let kml = GEOREF.read().0.clone();
            keep_alive(stream, KML_TYPE, kml.as_bytes(), is_head)
        }
        _ if path == "/location.kml" => keep_alive(stream, KML_TYPE, location(&state).as_bytes(), is_head),
        _ if path == "/network.kml" => keep_alive(stream, KML_TYPE, network(port).as_bytes(), is_head),
        _ if path == "/wps.kml" => keep_alive(stream, KML_TYPE, waypoints(&state).as_bytes(), is_head),
        _ if path == "/block_plane_0.dae" => keep_alive(stream, "model/vnd.collada+xml", MODEL, is_head),
        _ if path == "/hud.html" => {
            write!(stream, "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n", HUD_HTML.len())?;
            if !is_head {
                stream.write_all(HUD_HTML)?;
            }
            Ok(false)
        }
        _ if matches!(lower.as_str(), "/hud.jpg" | "/map.jpg" | "/both.jpg") => mjpeg(stream, &lower, shared).map(|_| false),
        _ if path == "/guided" && target.contains('?') => guided(stream, peer, head, &state, GuidedSource::Query(&target)).map(|_| false),
        _ if method.eq_ignore_ascii_case("POST") && lower.starts_with("/guide") => guided(stream, peer, head, &state, GuidedSource::Body).map(|_| false),
        _ if matches!(lower.as_str(), "/command_long" | "/rcoverride" | "/get_mission") => {
            stream.write_all(b"HTTP/1.1 404 not found\r\nContent-Type: image/jpg\r\n\r\n")?;
            Ok(false)
        }
        _ if lower.starts_with("/mavlink/") => {
            let index = shared.mavlink_index.fetch_add(1, Ordering::Relaxed);
            let body = mavelous_json(&state, index);
            write!(stream, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{body}", body.len())?;
            stream.flush()?;
            Ok(true)
        }
        _ if lower.starts_with("/mav/") => mavelous_file(stream, head, &target),
        _ if lower.contains(".jpg") => georef_image(stream, &path).map(|_| false),
        _ if path == "/" => {
            stream.write_all(b"HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html\r\n\r\n")?;
            stream.write_all(INDEX.as_bytes())?;
            Ok(false)
        }
        _ => {
            stream.write_all(b"HTTP/1.1 404 not found\r\nContent-Type: text/plain\r\n\r\n")?;
            Ok(false)
        }
    }
}

/// The KML routes' keep-alive response.
fn keep_alive(stream: &mut TcpStream, content_type: &str, body: &[u8], head: bool) -> io::Result<bool> {
    write!(stream, "HTTP/1.1 200 OK\r\nServer: here\r\nKeep-Alive: timeout=15, max=100\r\nConnection: Keep-Alive\r\nCache-Control: no-cache\r\nContent-Type: {content_type}\r\nX-Pad: avoid browser bug\r\nContent-Length: {}\r\n\r\n", body.len())?;
    if !head {
        stream.write_all(body)?;
    }
    stream.flush()?;
    Ok(true)
}

/// ComputeWebSocketHandshakeSecurityHash09
fn websocket_accept(head: &str) -> String {
    use base64::Engine;
    let key = header_value(head, "Sec-WebSocket-Key").unwrap_or("");
    let digest = sha1(format!("{key}258EAFA5-E914-47DA-95CA-C5AB0DC85B11").as_bytes());
    base64::engine::general_purpose::STANDARD.encode(digest)
}

/// SHA-1 (FIPS 180-4), only for the RFC 6455 handshake.
fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0];
    let mut msg = data.to_vec();
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&((data.len() as u64) * 8).to_be_bytes());
    for chunk in msg.chunks(64) {
        let mut w = [0u32; 80];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([chunk[4 * i], chunk[4 * i + 1], chunk[4 * i + 2], chunk[4 * i + 3]]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let [mut a, mut b, mut c, mut d, mut e] = h;
        for (i, wi) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | (!b & d), 0x5A827999),
                20..=39 => (b ^ c ^ d, 0x6ED9EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1BBCDC),
                _ => (b ^ c ^ d, 0xCA62C1D6),
            };
            let t = a.rotate_left(5).wrapping_add(f).wrapping_add(e).wrapping_add(k).wrapping_add(*wi);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = t;
        }
        for (x, v) in h.iter_mut().zip([a, b, c, d, e]) {
            *x = x.wrapping_add(v);
        }
    }
    let mut out = [0u8; 20];
    for (i, v) in h.iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&v.to_be_bytes());
    }
    out
}

fn ws_frame(opcode: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = vec![0x80 | opcode];
    match payload.len() {
        n if n <= 125 => out.push(n as u8),
        n if n <= 0xffff => {
            out.push(126);
            out.extend_from_slice(&(n as u16).to_be_bytes());
        }
        n => {
            out.push(127);
            out.extend_from_slice(&(n as u64).to_be_bytes());
        }
    }
    out.extend_from_slice(payload);
    out
}

/// Read one client frame: (opcode, unmasked payload). None when nothing has arrived.
fn ws_read(stream: &mut TcpStream) -> io::Result<Option<(u8, Vec<u8>)>> {
    let mut h = [0u8; 2];
    match stream.read(&mut h[..1]) {
        Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
        Ok(_) => {}
        Err(e) if mp_core::comms::is_timeout(&e) => return Ok(None),
        Err(e) => return Err(e),
    }
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    stream.read_exact(&mut h[1..])?;
    let mut len = (h[1] & 0x7f) as u64;
    if len == 126 {
        let mut b = [0u8; 2];
        stream.read_exact(&mut b)?;
        len = u16::from_be_bytes(b) as u64;
    } else if len == 127 {
        let mut b = [0u8; 8];
        stream.read_exact(&mut b)?;
        len = u64::from_be_bytes(b);
    }
    if len > 1 << 20 {
        return Err(io::ErrorKind::InvalidData.into());
    }
    let mut mask = [0u8; 4];
    if h[1] & 0x80 != 0 {
        stream.read_exact(&mut mask)?;
    }
    let mut payload = vec![0u8; len as usize];
    stream.read_exact(&mut payload)?;
    for (i, b) in payload.iter_mut().enumerate() {
        *b ^= mask[i % 4];
    }
    Ok(Some((h[0] & 0x0f, payload)))
}

/// CurrentState as JSON, by Mission Planner's property names.
fn state_json(state: &Snapshot) -> String {
    let Some(shared) = state.links.first() else { return "{}".into() };
    let s = shared.read();
    let cs = &s.cs;
    let mut map = serde_json::Map::new();
    for (name, value) in cs.fields() {
        map.insert(name.into(), serde_json::json!(value));
    }
    for (name, rate) in ["rateattitude", "rateposition", "ratestatus", "raterc", "ratesensors"].iter().zip(state.rates) {
        map.insert((*name).into(), rate.into());
    }
    map.insert("mode".into(), cs.mode.clone().into());
    // ter_alt: TERRAIN_REPORT terrain height, read by hud.html
    let ter_alt = s.inspector.get(&(cs.sysid, cs.compid, 136)).and_then(|m| if let MavMessage::TERRAIN_REPORT(t) = &m.msg { Some(t.terrain_height) } else { None }).unwrap_or(0.0);
    map.insert("ter_alt".into(), serde_json::json!(crate::units::alt(ter_alt)));
    map.insert("HomeAlt".into(), serde_json::json!(state.home.alt));
    map.insert("HomeLocation".into(), serde_json::json!({ "Lat": state.home.lat, "Lng": state.home.lng, "Alt": state.home.alt }));
    serde_json::Value::Object(map).to_string()
}

/// MAV.wps: the mission keyed by sequence, as mavlink_mission_item_int_t fields.
fn wps_json(state: &Snapshot) -> String {
    let mut map = serde_json::Map::new();
    for (i, w) in state.mission.iter().enumerate() {
        let seq = i + 1;
        map.insert(seq.to_string(), serde_json::json!({
            "param1": w.p1, "param2": w.p2, "param3": w.p3, "param4": w.p4,
            "x": (w.lat * 1e7) as i32, "y": (w.lng * 1e7) as i32, "z": w.alt,
            "seq": seq, "command": w.id, "target_system": 0, "target_component": 0,
            "frame": w.frame, "current": 0, "autocontinue": 1, "mission_type": 0,
        }));
    }
    serde_json::Value::Object(map).to_string()
}

fn websocket_server(stream: &mut TcpStream, head: &str, shared: &Shared) -> io::Result<()> {
    write!(stream, "HTTP/1.1 101 WebSocket Protocol Handshake\r\nUpgrade: WebSocket\r\nConnection: Upgrade\r\nWebSocket-Location: ws://localhost:56781/websocket/server\r\nSec-WebSocket-Accept: {}\r\nServer: Mission Planner\r\n\r\n", websocket_accept(head))?;
    stream.flush()?;
    stream.set_read_timeout(Some(Duration::from_millis(1)))?;
    while !shared.stop.load(Ordering::Relaxed) {
        // a close frame from the client ends the stream
        let mut b = [0u8; 256];
        match stream.read(&mut b) {
            Ok(0) => return Ok(()),
            Ok(n) if b[..n].contains(&0x88) => return Ok(()),
            Ok(_) => {}
            Err(e) if mp_core::comms::is_timeout(&e) => {}
            Err(e) => return Err(e),
        }
        let snapshot = shared.snapshot.read().clone();
        for text in [state_json(&snapshot), wps_json(&snapshot)] {
            stream.write_all(&ws_frame(0x1, text.as_bytes()))?;
        }
        stream.flush()?;
        std::thread::sleep(STREAM_PERIOD);
    }
    Ok(())
}

fn websocket_raw(stream: &mut TcpStream, head: &str, state: &Snapshot) -> io::Result<()> {
    let date = httpdate(std::time::SystemTime::now());
    write!(stream, "HTTP/1.1 101 WebSocket Protocol Handshake\r\nUpgrade: WebSocket\r\nConnection: upgrade\r\nDate: {date}\r\nSec-WebSocket-Accept: {}\r\n", websocket_accept(head))?;
    if head.contains("Sec-WebSocket-Protocol:") {
        stream.write_all(b"Sec-WebSocket-Protocol: binary\r\n")?;
    }
    stream.write_all(b"Server: Mission Planner\r\n\r\n")?;
    stream.flush()?;
    let Some(command) = state.command.clone() else { return Ok(()) };
    // MainV2.comPort.OnPacketReceived
    let (tx, rx) = crossbeam_channel::unbounded();
    command.send(LinkCommand::TapFrames(tx)).map_err(|_| io::Error::from(io::ErrorKind::NotConnected))?;
    stream.set_read_timeout(Some(Duration::from_millis(1)))?;
    loop {
        while let Ok(frame) = rx.try_recv() {
            stream.write_all(&ws_frame(0x2, &frame))?;
        }
        stream.flush()?;
        loop {
            match ws_read(stream)? {
                Some((0x8, _)) => return Ok(()),
                Some((_, payload)) => {
                    // a whole MAVLink frame from the client goes to the vehicle
                    if matches!(payload.first(), Some(0xfe | 0xfd)) && payload.len() >= 8 {
                        let _ = command.send(LinkCommand::SendRaw(payload));
                    }
                    stream.set_read_timeout(Some(Duration::from_millis(1)))?;
                }
                None => break,
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// RFC 1123 date as .NET's ToString("R").
fn httpdate(t: std::time::SystemTime) -> String {
    let secs = t.duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0) as i64;
    let days = secs.div_euclid(86400);
    let rem = secs.rem_euclid(86400);
    // civil-from-days (Howard Hinnant)
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + if m <= 2 { 1 } else { 0 };
    let wd = ["Thu", "Fri", "Sat", "Sun", "Mon", "Tue", "Wed"][days.rem_euclid(7) as usize];
    let mon = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"][(m - 1) as usize];
    format!("{wd}, {d:02} {mon} {y} {:02}:{:02}:{:02} GMT", rem / 3600, rem / 60 % 60, rem % 60)
}

fn mjpeg(stream: &mut TcpStream, which: &str, shared: &Shared) -> io::Result<()> {
    stream.write_all(b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=PLANNER\r\n\r\n--PLANNER\r\n")?;
    while !shared.stop.load(Ordering::Relaxed) {
        std::thread::sleep(STREAM_PERIOD); // 5hz
        let data = {
            let mut f = FRAMES.lock();
            f.wanted_until = Some(Instant::now() + Duration::from_secs(2));
            match which {
                "/hud.jpg" => f.hud.clone(),
                "/map.jpg" => f.map.clone(),
                _ => f.both.clone(),
            }
        };
        let Some(data) = data else { continue };
        write!(stream, "Content-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n", data.len())?;
        stream.write_all(&data)?;
        stream.write_all(b"\r\n--PLANNER\r\n")?;
        stream.flush()?;
    }
    Ok(())
}

enum GuidedSource<'a> {
    Query(&'a str),
    Body,
}

/// /guided?lat=..&lng=..&alt=.. and POST /guide {"lat":..,"lon":..,"alt":..}: setGuidedModeWP.
fn guided(stream: &mut TcpStream, peer: SocketAddr, head: &str, state: &Snapshot, source: GuidedSource) -> io::Result<()> {
    if !peer.ip().is_loopback() || is_cross_site(head) {
        stream.write_all(b"HTTP/1.1 403 Forbidden\r\n\r\nForbidden")?;
        return Ok(());
    }
    let number = |text: &str, key: &str, signed: bool| -> Option<f64> {
        let at = text.to_ascii_lowercase().find(&key.to_ascii_lowercase())? + key.len();
        let v: String = text[at..].chars().take_while(|c| c.is_ascii_digit() || *c == '.' || (signed && *c == '-')).collect();
        v.parse().ok()
    };
    let parsed = match source {
        GuidedSource::Query(q) => number(q, "lat=", true).zip(number(q, "&lng=", true)).zip(number(q, "&alt=", false)),
        GuidedSource::Body => number(head, "lat\":", true).zip(number(head, ",\"lon\":", true)).zip(number(head, ",\"alt\":", false)),
    };
    match parsed {
        Some(((lat, lng), alt)) => {
            if let Some(c) = &state.command {
                let _ = c.send(LinkCommand::Guided { lat, lng, alt: alt as f32 });
            }
            stream.write_all(b"HTTP/1.1 200 OK\r\n\r\nSent Guide Mode Wp")
        }
        None => stream.write_all(b"HTTP/1.1 200 OK\r\n\r\nFailed Guide Mode Wp"),
    }
}

/// Newtonsoft writes byte[] as base64.
fn b64(bytes: &[u8]) -> String {
    use base64::Engine;
    base64::engine::general_purpose::STANDARD.encode(bytes)
}

fn message_json(msg: &MavMessage) -> Option<serde_json::Value> {
    use serde_json::json;
    Some(match msg {
        MavMessage::ATTITUDE(m) => json!({"time_boot_ms": m.time_boot_ms, "roll": m.roll, "pitch": m.pitch, "yaw": m.yaw, "rollspeed": m.rollspeed, "pitchspeed": m.pitchspeed, "yawspeed": m.yawspeed}),
        MavMessage::VFR_HUD(m) => json!({"airspeed": m.airspeed, "groundspeed": m.groundspeed, "alt": m.alt, "climb": m.climb, "heading": m.heading, "throttle": m.throttle}),
        MavMessage::NAV_CONTROLLER_OUTPUT(m) => json!({"nav_roll": m.nav_roll, "nav_pitch": m.nav_pitch, "alt_error": m.alt_error, "aspd_error": m.aspd_error, "xtrack_error": m.xtrack_error, "nav_bearing": m.nav_bearing, "target_bearing": m.target_bearing, "wp_dist": m.wp_dist}),
        MavMessage::GPS_RAW_INT(m) => json!({"time_usec": m.time_usec, "lat": m.lat, "lon": m.lon, "alt": m.alt, "eph": m.eph, "epv": m.epv, "vel": m.vel, "cog": m.cog, "fix_type": m.fix_type as u8, "satellites_visible": m.satellites_visible, "alt_ellipsoid": m.alt_ellipsoid, "h_acc": m.h_acc, "v_acc": m.v_acc, "vel_acc": m.vel_acc, "hdg_acc": m.hdg_acc, "yaw": m.yaw}),
        MavMessage::HEARTBEAT(m) => json!({"custom_mode": m.custom_mode, "type": m.mavtype as u8, "autopilot": m.autopilot as u8, "base_mode": m.base_mode.bits(), "system_status": m.system_status as u8, "mavlink_version": m.mavlink_version}),
        MavMessage::GPS_STATUS(m) => json!({"satellites_visible": m.satellites_visible, "satellite_prn": b64(&m.satellite_prn), "satellite_used": b64(&m.satellite_used), "satellite_elevation": b64(&m.satellite_elevation), "satellite_azimuth": b64(&m.satellite_azimuth), "satellite_snr": b64(&m.satellite_snr)}),
        MavMessage::STATUSTEXT(m) => json!({"severity": m.severity as u8, "text": b64(&m.text[..]), "id": m.id, "chunk_seq": m.chunk_seq}),
        _ => return None,
    })
}

/// /mavlink/: the Mavelous message set from the active vehicle's last packets.
fn mavelous_json(state: &Snapshot, index: usize) -> String {
    let mut out = serde_json::Map::new();
    let mut quality = serde_json::json!({"msg": null, "index": index, "time_usec": 0});
    if let Some(shared) = state.links.first() {
        let s = shared.read();
        let (sys, comp) = (s.cs.sysid, s.cs.compid);
        for (name, id) in [("VFR_HUD", 74u32), ("STATUSTEXT", 253), ("SYS_STATUS", 1), ("ATTITUDE", 30), ("GPS_RAW_INT", 24), ("HEARTBEAT", 0), ("GPS_STATUS", 25), ("NAV_CONTROLLER_OUTPUT", 62)] {
            let value = s.inspector.get(&(sys, comp, id)).and_then(|m| message_json(&m.msg)).map(|msg| serde_json::json!({"msg": msg, "index": 1, "time_usec": 0}));
            out.insert(name.into(), value.unwrap_or(serde_json::Value::Null));
        }
        quality["msg"] = serde_json::json!({
            "master_in": s.cs.packets_received.saturating_sub(s.cs.packets_lost) as i64,
            "mav_loss": 0, "mavpackettype": "META_LINKQUALITY",
            "master_out": s.cs.packets_received as i64,
            "packet_loss": 100.0 - s.cs.linkqualitygcs as f64,
        });
    }
    // the original assigns META_LINKQUALITY to SYS_STATUS as well
    out.insert("SYS_STATUS".into(), quality.clone());
    out.insert("META_LINKQUALITY".into(), quality);
    serde_json::Value::Object(out).to_string()
}

/// Settings.GetRunningDirectory() + "mavelous_web"
fn mavelous_dir() -> PathBuf {
    std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("mavelous_web"))).unwrap_or_else(|| PathBuf::from("mavelous_web"))
}

fn url_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        let hex = (b[i] == b'%' && i + 2 < b.len())
            .then(|| std::str::from_utf8(&b[i + 1..i + 3]).ok().and_then(|h| u8::from_str_radix(h, 16).ok()))
            .flatten();
        match (hex, b[i]) {
            (Some(v), _) => {
                out.push(v);
                i += 3;
                continue;
            }
            (None, b'+') => out.push(b' '),
            (None, c) => out.push(c),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).to_string()
}

fn mavelous_file(stream: &mut TcpStream, head: &str, target: &str) -> io::Result<bool> {
    let mut file = url_decode(target).replacen("/mav/", "", 1);
    if file.is_empty() || file == "/" {
        file = "index.html".into();
    }
    let base = mavelous_dir();
    let base = base.canonicalize().unwrap_or(base);
    let full = base.join(&file);
    let resolved = full.canonicalize().ok();
    // path traversal stays inside mavelous_web
    if resolved.as_ref().is_some_and(|p| !p.starts_with(&base)) || file.split(['/', '\\']).any(|c| c == "..") {
        stream.write_all(b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n\r\n")?;
        stream.flush()?;
        return Ok(true);
    }
    let Some(path) = resolved.filter(|p| p.is_file()) else {
        stream.write_all(b"HTTP/1.1 404 not found\r\nConnection: Keep-Alive\r\nContent-Length: 0\r\nContent -Type: text/plain\r\n\r\n")?;
        stream.flush()?;
        return Ok(true);
    };
    let kind = if file.contains(".htm") { "text/html" } else if file.contains(".js") { "application/x-javascript" } else if file.contains(".css") { "text/css" } else { "text/plain" };
    let modified = std::fs::metadata(&path).and_then(|m| m.modified()).map(httpdate).unwrap_or_default();
    // LastWriteTimeUtc.ToString("ddd, dd MMM yyyy HH:mm:ss") + " GMT"
    if header_value(head, "If-Modified-Since").is_some_and(|v| v.eq_ignore_ascii_case(&modified)) {
        stream.write_all(b"HTTP/1.1 304 not modified\r\nConnection: Keep-Alive\r\n\r\n")?;
        stream.flush()?;
        return Ok(true);
    }
    let body = std::fs::read(&path)?;
    write!(stream, "HTTP/1.1 200 OK\r\nContent-Type: {kind}\r\nConnection: keep-alive\r\nLast-Modified: {modified}\r\nContent-Length: {}\r\n\r\n", body.len())?;
    stream.write_all(&body)?;
    stream.flush()?;
    Ok(true)
}

/// A georeferenced photo from georefimagepath, resized to 640x480.
fn georef_image(stream: &mut TcpStream, path: &str) -> io::Result<()> {
    let dir = GEOREF.read().1.clone();
    let name = url_decode(path.trim_start_matches('/'));
    let jpeg = (!dir.as_os_str().is_empty() && !name.split(['/', '\\']).any(|c| c == ".."))
        .then(|| image::open(dir.join(&name)).ok())
        .flatten()
        .and_then(|img| {
            let mut out = Vec::new();
            // ResizeImage(orig, 640x480) keeps the aspect ratio
            let percent = (640.0 / img.width() as f32).min(480.0 / img.height() as f32);
            let (w, h) = (((img.width() as f32 * percent) as u32).max(1), ((img.height() as f32 * percent) as u32).max(1));
            img.resize_exact(w, h, image::imageops::FilterType::CatmullRom).to_rgb8().write_to(&mut io::Cursor::new(&mut out), image::ImageFormat::Jpeg).ok()?;
            Some(out)
        });
    match jpeg {
        Some(data) => {
            write!(stream, "HTTP/1.1 200 OK\r\nServer: here\r\nKeep-Alive: timeout=15, max=100\r\nConnection: Keep-Alive\r\nContent-Type: image/jpg\r\nX-Pad: avoid browser bug\r\nContent-Length: {}\r\n\r\n", data.len())?;
            stream.write_all(&data)
        }
        None => stream.write_all(b"HTTP/1.1 404 not found\r\nContent-Type: image/jpg\r\n\r\n"),
    }
}

fn network(port: u16) -> String {
    format!("{ROOT_OPEN}<Folder><name>Network Links</name><open>1</open><NetworkLink><name>View Centered Placemark</name><open>1</open><refreshVisibility>0</refreshVisibility><flyToView>1</flyToView><Link><href>http://127.0.0.1:{port}/location.kml</href><refreshMode>onInterval</refreshMode><refreshInterval>1</refreshInterval><viewRefreshTime>1</viewRefreshTime></Link></NetworkLink><NetworkLink><name>View Centered Placemark</name><open>1</open><refreshVisibility>0</refreshVisibility><flyToView>0</flyToView><Link><href>http://127.0.0.1:{port}/wps.kml</href></Link></NetworkLink></Folder></kml>")
}

fn waypoints(state: &Snapshot) -> String {
    let mut points = Vec::new();
    if state.home.lat != 0.0 || state.home.lng != 0.0 {
        points.push(("H".to_string(), state.home.lat, state.home.lng, state.home.alt as f64));
    }
    for (i, wp) in state.mission.iter().enumerate() {
        if !wp.has_location() || (wp.lat == 0.0 && wp.lng == 0.0) {
            continue;
        }
        if !wp.lat.is_finite() || !wp.lng.is_finite() || !wp.alt.is_finite() {
            continue;
        }
        let ground = match AltMode::from_frame(wp.frame) {
            AltMode::Absolute => 0.0,
            AltMode::Relative => state.home.alt as f64,
            AltMode::Terrain => crate::srtm::altitude(wp.lat, wp.lng).unwrap_or(0.0),
        };
        points.push(((i + 1).to_string(), wp.lat, wp.lng, wp.alt as f64 + ground));
    }
    let mut out = OPEN.to_string();
    let mut coords = String::new();
    for (tag, lat, lng, alt) in points {
        let _ = write!(out, "<Placemark><name>WP {tag} Alt: {alt}</name><Point><altitudeMode>absolute</altitudeMode><coordinates>{lng},{lat},{alt}</coordinates></Point></Placemark>");
        let _ = write!(coords, "{lng},{lat},{alt} ");
    }
    for (name, altitude, color) in [("WPs", "absolute", "ff00ffff"), ("onground", "clampToGround", "7f00ffff")] {
        let _ = write!(out, "<Placemark><name>{name}</name><Style><LineStyle><color>{color}</color><width>4</width></LineStyle></Style><LineString><extrude>0</extrude><tessellate>1</tessellate><altitudeMode>{altitude}</altitudeMode><coordinates>{coords}</coordinates></LineString></Placemark>");
    }
    out.push_str(CLOSE);
    out
}

fn location(state: &Snapshot) -> String {
    let mut out = OPEN.to_string();
    for (index, shared) in state.links.iter().enumerate() {
        let shared = shared.read();
        if shared.status != LinkStatus::Connected {
            continue;
        }
        if index == 0 && shared.cs.lat != 0.0 && shared.cs.lng != 0.0 {
            let cs = &shared.cs;
            let _ = write!(out, "<LookAt><longitude>{}</longitude><latitude>{}</latitude><altitude>{}</altitude><heading>{}</heading><tilt>80</tilt><range>50</range><altitudeMode>absolute</altitudeMode></LookAt>", cs.lng, cs.lat, cs.altasl, cs.yaw);
        }
        let mut vehicles: Vec<(u8, u8)> = shared.mavs.keys().copied().collect();
        if vehicles.is_empty() {
            vehicles.push((shared.cs.sysid, shared.cs.compid));
        }
        for (sys, comp) in vehicles {
            let (lat, lng, alt, roll, pitch, yaw) = if (sys, comp) == (shared.cs.sysid, shared.cs.compid) {
                let c = &shared.cs;
                (c.lat, c.lng, c.altasl as f64, c.roll as f64, c.pitch as f64, c.yaw as f64)
            } else {
                let Some(position) = shared.inspector.get(&(sys, comp, 33)).and_then(|s| if let MavMessage::GLOBAL_POSITION_INT(p) = &s.msg { Some(p) } else { None }) else {
                    continue;
                };
                let attitude = shared.inspector.get(&(sys, comp, 30)).and_then(|s| if let MavMessage::ATTITUDE(a) = &s.msg { Some(a) } else { None });
                (
                    position.lat as f64 / 1e7,
                    position.lon as f64 / 1e7,
                    position.alt as f64 / 1000.0,
                    attitude.map(|a| a.roll as f64).unwrap_or(0.0).to_degrees(),
                    attitude.map(|a| a.pitch as f64).unwrap_or(0.0).to_degrees(),
                    attitude.map(|a| a.yaw as f64).unwrap_or(0.0).to_degrees().rem_euclid(360.0),
                )
            };
            if lat == 0.0 && lng == 0.0 {
                continue;
            }
            let alt = if alt < 0.0 { 0.01 } else { alt };
            let _ = write!(out, "<Placemark><name>P/Q {alt}</name><visibility>1</visibility><Model><altitudeMode>absolute</altitudeMode><Location><longitude>{lng}</longitude><latitude>{lat}</latitude><altitude>{alt}</altitude></Location><Orientation><heading>{yaw}</heading><tilt>{}</tilt><roll>{}</roll></Orientation><Scale><x>2</x><y>2</y><z>2</z></Scale><Link><href>block_plane_0.dae</href></Link></Model></Placemark>", -pitch, -roll);
        }
    }
    out.push_str(CLOSE);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_core::mavlink::dialects::ardupilotmega::ATTITUDE_DATA;

    fn request(addr: SocketAddr, raw: &str) -> (String, Vec<u8>) {
        let mut stream = TcpStream::connect(addr).unwrap();
        stream.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        stream.write_all(raw.as_bytes()).unwrap();
        let mut data = Vec::new();
        let mut buf = [0u8; 65536];
        // keep-alive routes leave the socket open: stop once the announced body arrived
        loop {
            match stream.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => data.extend_from_slice(&buf[..n]),
            }
            if let Some(split) = data.windows(4).position(|b| b == b"\r\n\r\n") {
                let head = String::from_utf8_lossy(&data[..split]).to_string();
                if let Some(len) = header_value(&format!("X\r\n{head}"), "Content-Length").and_then(|v| v.parse::<usize>().ok()) {
                    if data.len() >= split + 4 + len {
                        break;
                    }
                }
            }
        }
        let split = data.windows(4).position(|b| b == b"\r\n\r\n").map(|i| i + 4).unwrap_or(data.len());
        (String::from_utf8_lossy(&data[..split]).to_string(), data[split..].to_vec())
    }

    fn get(addr: SocketAddr, method: &str, path: &str) -> (String, Vec<u8>) {
        request(addr, &format!("{method} {path} HTTP/1.1\r\nHost: localhost\r\n\r\n"))
    }

    fn server(snapshot: Snapshot) -> (HttpServer, SocketAddr) {
        let s = HttpServer::bind(([127, 0, 0, 1], 0).into(), snapshot).unwrap();
        let addr = s.addr;
        (s, addr)
    }

    #[test]
    fn network_kml_serves_live_mission_aircraft_model_and_updates() {
        let shared = Arc::new(RwLock::new(LinkShared::default()));
        {
            let mut s = shared.write();
            s.status = LinkStatus::Connected;
            s.cs.lat = 12.0;
            s.cs.lng = 79.0;
            s.cs.altasl = 110.0;
            s.cs.roll = 5.0;
            s.cs.pitch = 10.0;
            s.cs.yaw = 90.0;
        }
        let mut state = Snapshot {
            home: Locationwp::waypoint(12.0, 79.0, 100.0, 0),
            mission: vec![Locationwp::waypoint(12.1, 79.1, 20.0, 3), Locationwp::waypoint(12.2, 79.2, 150.0, 0)],
            links: vec![shared.clone()],
            ..Default::default()
        };
        let (server, addr) = server(state.clone());
        let (head, body) = get(addr, "GET", "/network.kml");
        assert!(head.starts_with("HTTP/1.1 200") && head.contains(KML_TYPE) && head.contains("Keep-Alive"));
        let xml = String::from_utf8(body).unwrap();
        let doc = roxmltree::Document::parse(&xml).unwrap();
        assert_eq!(doc.descendants().filter(|n| n.has_tag_name("NetworkLink")).count(), 2);
        assert!(xml.contains(&format!("http://127.0.0.1:{}/wps.kml", addr.port())));
        let xml = String::from_utf8(get(addr, "GET", "/wps.kml").1).unwrap();
        roxmltree::Document::parse(&xml).unwrap();
        assert!(xml.contains("79.1,12.1,120") && xml.contains("79.2,12.2,150") && xml.contains("clampToGround"));
        state.mission[0].lat = 13.0;
        server.update(state);
        assert!(String::from_utf8(get(addr, "GET", "/wps.kml?refresh=1").1).unwrap().contains("79.1,13,120"));
        let xml = String::from_utf8(get(addr, "GET", "/location.kml").1).unwrap();
        roxmltree::Document::parse(&xml).unwrap();
        assert!(xml.contains("<heading>90</heading>") && xml.contains("<roll>-5</roll>") && xml.contains("<tilt>-10</tilt>"));
        shared.write().cs.lat = 14.0;
        assert!(String::from_utf8(get(addr, "GET", "/location.kml").1).unwrap().contains("<latitude>14</latitude>"));
        assert_eq!(get(addr, "GET", "/block_plane_0.dae").1, MODEL);
        let (head, body) = get(addr, "HEAD", "/wps.kml");
        assert!(head.starts_with("HTTP/1.1 200") && body.is_empty());
        drop(server);
        assert!(TcpStream::connect_timeout(&addr, Duration::from_millis(100)).is_err());
    }

    #[test]
    fn index_hud_page_stubs_and_missing_routes_match_the_original() {
        let (_server, addr) = server(Snapshot::default());
        let (head, body) = get(addr, "GET", "/");
        assert!(head.starts_with("HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html"));
        let body = String::from_utf8(body).unwrap();
        for link in ["/mav/", "/mavlink/", "/hud.jpg", "/map.jpg", "/both.jpg", "/hud.html", "/network.kml", "/georefnetwork.kml"] {
            assert!(body.contains(&format!("href={link}>")), "{link}");
        }
        let (head, body) = get(addr, "GET", "/hud.html");
        assert!(head.contains("Content-Type: text/html") && body == HUD_HTML);
        assert!(String::from_utf8_lossy(&body).contains(":56781/websocket/server"));
        for stub in ["/command_long", "/rcoverride", "/get_mission"] {
            assert_eq!(get(addr, "GET", stub).0, "HTTP/1.1 404 not found\r\nContent-Type: image/jpg\r\n\r\n", "{stub}");
        }
        assert!(get(addr, "GET", "/nothing").0.starts_with("HTTP/1.1 404 not found\r\nContent-Type: text/plain"));
        // /mav/ never leaves mavelous_web
        let (head, body) = get(addr, "GET", "/mav/../../Cargo.toml");
        assert!(head.starts_with("HTTP/1.1 403") || head.starts_with("HTTP/1.1 404"), "{head}");
        assert!(body.is_empty());
        assert!(get(addr, "GET", "/mav/").0.starts_with("HTTP/1.1 404 not found"));
        // georef overlay published by the georef tool
        set_georef("<kml>georef</kml>".into(), PathBuf::new());
        let (head, body) = get(addr, "GET", "/georefnetwork.kml");
        assert!(head.contains(KML_TYPE) && body == b"<kml>georef</kml>");
        assert!(get(addr, "GET", "/photo.jpg").0.starts_with("HTTP/1.1 404 not found"));
    }

    #[test]
    fn guided_requests_need_loopback_same_site_and_valid_numbers() {
        let (tx, rx) = crossbeam_channel::unbounded();
        let (_server, addr) = server(Snapshot { command: Some(tx), ..Default::default() });
        let (_, body) = get(addr, "GET", "/guided?lat=-34&lng=117.8&alt=30");
        assert_eq!(body, b"Sent Guide Mode Wp");
        assert!(matches!(rx.try_recv(), Ok(LinkCommand::Guided { lat, lng, alt }) if lat == -34.0 && lng == 117.8 && alt == 30.0));
        let (_, body) = get(addr, "GET", "/guided?lat=x&lng=1&alt=2");
        assert_eq!(body, b"Failed Guide Mode Wp");
        let (head, _) = request(addr, "GET /guided?lat=1&lng=2&alt=3 HTTP/1.1\r\nOrigin: http://example.com\r\n\r\n");
        assert!(head.starts_with("HTTP/1.1 403 Forbidden"));
        assert!(rx.try_recv().is_err(), "a cross-site request sends nothing");
        let (_, body) = request(addr, "GET /guided?lat=1&lng=2&alt=3 HTTP/1.1\r\nReferer: http://localhost:56781/hud.html\r\n\r\n");
        assert_eq!(body, b"Sent Guide Mode Wp");
        let json = "{\"lat\":-35.5,\"lon\":149.1,\"alt\":40}";
        let (_, body) = request(addr, &format!("POST /guide HTTP/1.1\r\nContent-Length: {}\r\n\r\n{json}", json.len()));
        assert_eq!(body, b"Sent Guide Mode Wp");
        let sent: Vec<_> = rx.try_iter().collect();
        assert!(sent.iter().any(|c| matches!(c, LinkCommand::Guided { lat, lng, alt } if *lat == -35.5 && *lng == 149.1 && *alt == 40.0)));
    }

    fn ws_connect(addr: SocketAddr, path: &str) -> (TcpStream, String) {
        let mut s = TcpStream::connect(addr).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        write!(s, "GET {path} HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n").unwrap();
        let mut head = Vec::new();
        let mut b = [0u8; 1];
        while !head.ends_with(b"\r\n\r\n") {
            s.read_exact(&mut b).unwrap();
            head.push(b[0]);
        }
        (s, String::from_utf8(head).unwrap())
    }

    fn ws_recv(s: &mut TcpStream) -> (u8, Vec<u8>) {
        let mut h = [0u8; 2];
        s.read_exact(&mut h).unwrap();
        let mut len = (h[1] & 0x7f) as usize;
        if len == 126 {
            let mut b = [0u8; 2];
            s.read_exact(&mut b).unwrap();
            len = u16::from_be_bytes(b) as usize;
        } else if len == 127 {
            let mut b = [0u8; 8];
            s.read_exact(&mut b).unwrap();
            len = u64::from_be_bytes(b) as usize;
        }
        let mut payload = vec![0u8; len];
        s.read_exact(&mut payload).unwrap();
        (h[0], payload)
    }

    #[test]
    fn state_websocket_streams_current_state_and_waypoints() {
        let shared = Arc::new(RwLock::new(LinkShared::default()));
        shared.write().cs.roll = 12.5;
        let (server, addr) = server(Snapshot {
            links: vec![shared.clone()],
            mission: vec![Locationwp::waypoint(1.5, 2.5, 30.0, 3)],
            rates: [4, 2, 2, 2, 2],
            ..Default::default()
        });
        let (mut s, head) = ws_connect(addr, "/websocket/server");
        // RFC 6455 section 1.3 example
        assert!(head.starts_with("HTTP/1.1 101 WebSocket Protocol Handshake\r\n"), "{head}");
        assert!(head.contains("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n"));
        let (op, cs) = ws_recv(&mut s);
        assert_eq!(op, 0x81);
        let cs: serde_json::Value = serde_json::from_slice(&cs).unwrap();
        assert_eq!(cs["rateattitude"], 4);
        assert_eq!(cs["roll"], 12.5);
        let (_, wps) = ws_recv(&mut s);
        let wps: serde_json::Value = serde_json::from_slice(&wps).unwrap();
        assert_eq!(wps["1"]["x"], 15_000_000);
        shared.write().cs.roll = -3.0;
        server.update(Snapshot { links: vec![shared.clone()], ..Default::default() });
        let mut seen = false;
        for _ in 0..6 {
            let (_, data) = ws_recv(&mut s);
            if serde_json::from_slice::<serde_json::Value>(&data).unwrap().get("roll") == Some(&serde_json::json!(-3.0)) {
                seen = true;
                break;
            }
        }
        assert!(seen, "the stream follows live state");
        s.write_all(&[0x88, 0x80, 0, 0, 0, 0]).unwrap();
    }

    #[test]
    fn raw_websocket_relays_frames_both_ways() {
        let (tx, rx) = crossbeam_channel::unbounded();
        let (_server, addr) = server(Snapshot { command: Some(tx), ..Default::default() });
        let (mut s, head) = ws_connect(addr, "/websocket/raw");
        assert!(head.contains("Connection: upgrade\r\nDate: ") && head.contains("Server: Mission Planner"));
        let tap = loop {
            match rx.recv_timeout(Duration::from_secs(2)).unwrap() {
                LinkCommand::TapFrames(t) => break t,
                _ => continue,
            }
        };
        let frame = vec![0xfd, 9, 0, 0, 1, 1, 1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xaa, 0xbb];
        tap.send(frame.clone()).unwrap();
        assert_eq!(ws_recv(&mut s), (0x82, frame.clone()));
        // a masked client frame carrying MAVLink goes to the vehicle
        let mask = [1u8, 2, 3, 4];
        let mut out = vec![0x82, 0x80 | frame.len() as u8];
        out.extend_from_slice(&mask);
        out.extend(frame.iter().enumerate().map(|(i, b)| b ^ mask[i % 4]));
        s.write_all(&out).unwrap();
        let raw = loop {
            match rx.recv_timeout(Duration::from_secs(2)).unwrap() {
                LinkCommand::SendRaw(r) => break r,
                _ => continue,
            }
        };
        assert_eq!(raw, frame);
        s.write_all(&[0x88, 0x80, 0, 0, 0, 0]).unwrap();
    }

    #[test]
    fn mavelous_json_reports_last_packets_and_link_quality() {
        let shared = Arc::new(RwLock::new(LinkShared::default()));
        {
            let mut s = shared.write();
            s.cs.sysid = 1;
            s.cs.compid = 1;
            s.cs.packets_received = 100;
            s.cs.packets_lost = 4;
            s.cs.linkqualitygcs = 96.0;
            let msg = MavMessage::ATTITUDE(ATTITUDE_DATA { time_boot_ms: 7, roll: 0.25, ..Default::default() });
            s.inspector.insert((1, 1, 30), mp_core::link::MsgStat { name: "ATTITUDE", count: 1, rate: 1.0, last: Instant::now(), msg });
        }
        let (_server, addr) = server(Snapshot { links: vec![shared], ..Default::default() });
        let (head, body) = get(addr, "GET", "/mavlink/ATTITUDE+VFR_HUD?_=1");
        assert!(head.contains("Content-Type: application/json"));
        let v: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(v["ATTITUDE"]["msg"]["roll"], 0.25);
        assert_eq!(v["ATTITUDE"]["msg"]["time_boot_ms"], 7);
        assert!(v["VFR_HUD"].is_null());
        assert_eq!(v["META_LINKQUALITY"]["msg"]["master_in"], 96);
        assert_eq!(v["META_LINKQUALITY"]["msg"]["packet_loss"], 4.0);
        assert_eq!(v["SYS_STATUS"], v["META_LINKQUALITY"], "the original overwrites SYS_STATUS");
        let next: serde_json::Value = serde_json::from_slice(&get(addr, "GET", "/mavlink/").1).unwrap();
        assert_eq!(next["META_LINKQUALITY"]["index"].as_u64().unwrap(), v["META_LINKQUALITY"]["index"].as_u64().unwrap() + 1);
    }

    #[test]
    fn mjpeg_streams_requested_frames() {
        let (_server, addr) = server(Snapshot::default());
        let jpeg = vec![0xff, 0xd8, 1, 2, 3, 0xff, 0xd9];
        set_frames(Some(jpeg.clone()), None, None);
        let mut s = TcpStream::connect(addr).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        s.write_all(b"GET /hud.jpg HTTP/1.1\r\n\r\n").unwrap();
        let mut data = Vec::new();
        let mut buf = [0u8; 4096];
        let end = Instant::now() + Duration::from_secs(3);
        while Instant::now() < end && !data.windows(jpeg.len()).any(|w| w == jpeg.as_slice()) {
            let n = s.read(&mut buf).unwrap();
            data.extend_from_slice(&buf[..n]);
        }
        let text = String::from_utf8_lossy(&data);
        assert!(text.starts_with("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=PLANNER\r\n\r\n--PLANNER\r\n"));
        assert!(text.contains(&format!("Content-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n", jpeg.len())));
        assert!(frames_wanted(), "an open stream asks the UI for frames");
    }

    #[test]
    fn sha1_and_http_dates_match_references() {
        let hex: String = sha1(b"abc").iter().map(|b| format!("{b:02x}")).collect();
        assert_eq!(hex, "a9993e364706816aba3e25717850c26c9cd0d89d");
        assert_eq!(httpdate(std::time::UNIX_EPOCH + Duration::from_secs(784111777)), "Sun, 06 Nov 1994 08:49:37 GMT");
        assert!(is_loopback_origin("http://127.0.0.1:56781/x") && is_loopback_origin("http://[::1]:8080") && !is_loopback_origin("https://example.com"));
    }
}
