//! Mission Planner's Google Earth network links (`Utilities/httpserver.cs`).
//! The loopback server serves live mission/aircraft snapshots and the bundled model.
use mp_core::{
    link::{LinkShared, LinkStatus},
    mavlink::dialects::ardupilotmega::MavMessage,
    mission::{AltMode, Locationwp},
};
use parking_lot::RwLock;
use std::{
    fmt::Write as _,
    io::{self, Read, Write},
    net::{SocketAddr, TcpListener, TcpStream},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::JoinHandle,
    time::Duration,
};

const KML_TYPE: &str = "application/vnd.google-earth.kml+xml";
const ROOT_OPEN: &str =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\">";
const MODEL: &[u8] = include_bytes!("../assets/block_plane_0.dae");
const OPEN: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>";
const CLOSE: &str = "</Document></kml>";

#[derive(Clone, Default)]
pub struct Snapshot {
    pub home: Locationwp,
    pub mission: Vec<Locationwp>,
    /// Active link first, then the other MainV2.Comports.
    pub links: Vec<Arc<RwLock<LinkShared>>>,
}

pub struct KmlServer {
    addr: SocketAddr,
    snapshot: Arc<RwLock<Snapshot>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl KmlServer {
    pub fn bind(port: u16, snapshot: Snapshot) -> io::Result<Self> {
        let listener = TcpListener::bind(("127.0.0.1", port))?;
        listener.set_nonblocking(true)?;
        let addr = listener.local_addr()?;
        let snapshot = Arc::new(RwLock::new(snapshot));
        let stop = Arc::new(AtomicBool::new(false));
        let data = snapshot.clone();
        let stopping = stop.clone();
        let thread = std::thread::Builder::new()
            .name("mission-kml".into())
            .spawn(move || {
                while !stopping.load(Ordering::Relaxed) {
                    match listener.accept() {
                        Ok((mut stream, _)) => {
                            let _ = serve(&mut stream, addr, &data);
                        }
                        Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                            std::thread::sleep(Duration::from_millis(20))
                        }
                        Err(_) => break,
                    }
                }
            })?;
        Ok(Self {
            addr,
            snapshot,
            stop,
            thread: Some(thread),
        })
    }

    pub fn network_url(&self) -> String {
        format!("http://{}/network.kml", self.addr)
    }
    pub fn update(&self, snapshot: Snapshot) {
        *self.snapshot.write() = snapshot;
    }
}

impl Drop for KmlServer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn serve(stream: &mut TcpStream, addr: SocketAddr, snapshot: &RwLock<Snapshot>) -> io::Result<()> {
    // Accepted sockets inherit the listener's non-blocking mode on Windows.
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(Duration::from_millis(200)))?;
    stream.set_write_timeout(Some(Duration::from_millis(200)))?;
    let mut request = Vec::new();
    let mut buf = [0; 1024];
    while !request.windows(4).any(|b| b == b"\r\n\r\n") {
        let n = stream.read(&mut buf)?;
        if n == 0 {
            return Ok(());
        }
        request.extend_from_slice(&buf[..n]);
        if request.len() > 8192 {
            return response(
                stream,
                "431 Request Header Fields Too Large",
                "text/plain",
                b"",
                false,
            );
        }
    }
    let text = String::from_utf8_lossy(&request);
    let mut line = text.lines().next().unwrap_or("").split_whitespace();
    let method = line.next().unwrap_or("");
    let path = line.next().unwrap_or("").split('?').next().unwrap_or("");
    if !matches!(method, "GET" | "HEAD") {
        return response(stream, "405 Method Not Allowed", "text/plain", b"", false);
    }
    let head = method == "HEAD";
    let state = snapshot.read().clone();
    match path {
        "/network.kml" => response(stream, "200 OK", KML_TYPE, network(addr).as_bytes(), head),
        "/wps.kml" => response(
            stream,
            "200 OK",
            KML_TYPE,
            waypoints(&state).as_bytes(),
            head,
        ),
        "/location.kml" => response(
            stream,
            "200 OK",
            KML_TYPE,
            location(&state).as_bytes(),
            head,
        ),
        "/block_plane_0.dae" => response(stream, "200 OK", "model/vnd.collada+xml", MODEL, head),
        _ => response(stream, "404 Not Found", "text/plain", b"Not Found", head),
    }
}

fn response(
    stream: &mut TcpStream,
    status: &str,
    content_type: &str,
    body: &[u8],
    head: bool,
) -> io::Result<()> {
    write!(stream, "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n", body.len())?;
    if !head {
        stream.write_all(body)?;
    }
    stream.flush()
}

fn network(addr: SocketAddr) -> String {
    format!("{ROOT_OPEN}<Folder><name>Network Links</name><open>1</open><NetworkLink><name>View Centered Placemark</name><open>1</open><refreshVisibility>0</refreshVisibility><flyToView>1</flyToView><Link><href>http://{addr}/location.kml</href><refreshMode>onInterval</refreshMode><refreshInterval>1</refreshInterval><viewRefreshTime>1</viewRefreshTime></Link></NetworkLink><NetworkLink><name>View Centered Placemark</name><open>1</open><refreshVisibility>0</refreshVisibility><flyToView>0</flyToView><Link><href>http://{addr}/wps.kml</href></Link></NetworkLink></Folder></kml>")
}

fn waypoints(state: &Snapshot) -> String {
    let mut points = Vec::new();
    if state.home.lat != 0.0 || state.home.lng != 0.0 {
        points.push((
            "H".to_string(),
            state.home.lat,
            state.home.lng,
            state.home.alt as f64,
        ));
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
    for (name, altitude, color) in [
        ("WPs", "absolute", "ff00ffff"),
        ("onground", "clampToGround", "7f00ffff"),
    ] {
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
            let (lat, lng, alt, roll, pitch, yaw) =
                if (sys, comp) == (shared.cs.sysid, shared.cs.compid) {
                    let c = &shared.cs;
                    (
                        c.lat,
                        c.lng,
                        c.altasl as f64,
                        c.roll as f64,
                        c.pitch as f64,
                        c.yaw as f64,
                    )
                } else {
                    let Some(position) = shared.inspector.get(&(sys, comp, 33)).and_then(|s| {
                        if let MavMessage::GLOBAL_POSITION_INT(p) = &s.msg {
                            Some(p)
                        } else {
                            None
                        }
                    }) else {
                        continue;
                    };
                    let attitude = shared.inspector.get(&(sys, comp, 30)).and_then(|s| {
                        if let MavMessage::ATTITUDE(a) = &s.msg {
                            Some(a)
                        } else {
                            None
                        }
                    });
                    (
                        position.lat as f64 / 1e7,
                        position.lon as f64 / 1e7,
                        position.alt as f64 / 1000.0,
                        attitude.map(|a| a.roll as f64).unwrap_or(0.0).to_degrees(),
                        attitude.map(|a| a.pitch as f64).unwrap_or(0.0).to_degrees(),
                        attitude
                            .map(|a| a.yaw as f64)
                            .unwrap_or(0.0)
                            .to_degrees()
                            .rem_euclid(360.0),
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
    fn get(addr: SocketAddr, method: &str, path: &str) -> (String, Vec<u8>) {
        let mut stream = TcpStream::connect(addr).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        write!(
            stream,
            "{method} {path} HTTP/1.1\r\nHost: localhost\r\n\r\n"
        )
        .unwrap();
        let mut data = Vec::new();
        stream.read_to_end(&mut data).unwrap();
        let split = data.windows(4).position(|b| b == b"\r\n\r\n").unwrap() + 4;
        (
            String::from_utf8(data[..split].to_vec()).unwrap(),
            data[split..].to_vec(),
        )
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
            mission: vec![
                Locationwp::waypoint(12.1, 79.1, 20.0, 3),
                Locationwp::waypoint(12.2, 79.2, 150.0, 0),
            ],
            links: vec![shared.clone()],
        };
        let server = KmlServer::bind(0, state.clone()).unwrap();
        let addr = server.addr;
        let (head, body) = get(addr, "GET", "/network.kml");
        assert!(head.starts_with("HTTP/1.1 200") && head.contains(KML_TYPE));
        let xml = String::from_utf8(body).unwrap();
        let doc = roxmltree::Document::parse(&xml).unwrap();
        assert_eq!(
            doc.descendants()
                .filter(|n| n.has_tag_name("NetworkLink"))
                .count(),
            2
        );
        assert!(xml.contains(&format!("http://{addr}/wps.kml")));
        let (_, body) = get(addr, "GET", "/wps.kml");
        let xml = String::from_utf8(body).unwrap();
        roxmltree::Document::parse(&xml).unwrap();
        assert!(xml.contains("79.1,12.1,120") && xml.contains("79.2,12.2,150"));
        assert!(xml.contains("clampToGround"));
        state.mission[0].lat = 13.0;
        server.update(state);
        let (_, body) = get(addr, "GET", "/wps.kml?refresh=1");
        assert!(String::from_utf8(body).unwrap().contains("79.1,13,120"));
        let (_, body) = get(addr, "GET", "/location.kml");
        let xml = String::from_utf8(body).unwrap();
        roxmltree::Document::parse(&xml).unwrap();
        assert!(
            xml.contains("<heading>90</heading>")
                && xml.contains("<roll>-5</roll>")
                && xml.contains("<tilt>-10</tilt>")
        );
        shared.write().cs.lat = 14.0;
        assert!(String::from_utf8(get(addr, "GET", "/location.kml").1)
            .unwrap()
            .contains("<latitude>14</latitude>"));
        let (_, model) = get(addr, "GET", "/block_plane_0.dae");
        assert_eq!(model, MODEL);
        let (head, body) = get(addr, "HEAD", "/wps.kml");
        assert!(head.starts_with("HTTP/1.1 200") && body.is_empty());
        assert!(get(addr, "GET", "/missing").0.starts_with("HTTP/1.1 404"));
        drop(server);
        assert!(TcpStream::connect_timeout(&addr, Duration::from_millis(100)).is_err());
    }
}
