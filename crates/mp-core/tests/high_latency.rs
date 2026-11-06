//! Source-derived HIGH_LATENCY fixtures sent through the production byte transport.
#![allow(deprecated)]
use mp_core::{comms::ConnectionSpec, link::{Link, LinkEvent, LinkStatus}, mavlink};
use mavlink::dialects::ardupilotmega::*;
use std::{io::Write, net::{TcpListener,TcpStream}, time::{Duration,Instant}};

struct Peer { stream: TcpStream, seq: u8 }
impl Peer {
    fn connect() -> (Link, Self) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let link = Link::connect_with_params(ConnectionSpec::Tcp { host: "127.0.0.1".into(), port: listener.local_addr().unwrap().port() }, None, false);
        let (stream, _) = listener.accept().unwrap();
        stream.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        stream.set_nodelay(true).unwrap();
        let mut reader = mavlink::MavlinkReader::new(stream.try_clone().unwrap());
        loop { if matches!(reader.read_any_message::<MavMessage>().unwrap().1, MavMessage::HEARTBEAT(_)) { break; } }
        (link, Self { stream, seq: 0 })
    }
    fn send(&mut self, component: u8, msg: MavMessage) {
        let mut bytes = Vec::new();
        mavlink::write_versioned_msg(&mut bytes, mavlink::MavlinkVersion::V2,
            mavlink::MavHeader { system_id: 42, component_id: component, sequence: self.seq }, &msg).unwrap();
        self.seq = self.seq.wrapping_add(1);
        self.stream.write_all(&bytes).unwrap();
    }
}
fn packet() -> HIGH_LATENCY2_DATA {
    HIGH_LATENCY2_DATA { mavtype: MavType::MAV_TYPE_FIXED_WING, autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        custom_mode: 15, latitude: -351000000, longitude: 1492000000, altitude: 600, target_altitude: 120,
        target_distance: 40, wp_num: 8, heading: 90, target_heading: 100, airspeed: 125, airspeed_sp: 150,
        groundspeed: 100, windspeed: 20, wind_heading: 45, climb_rate: -2, battery: 75, ..Default::default() }
}
fn wait(link: &Link, mut check: impl FnMut(&mp_core::link::LinkShared)->bool) {
    let end = Instant::now() + Duration::from_secs(3);
    while !check(&link.shared.read()) {
        assert!(Instant::now() < end, "wire state did not arrive");
        std::thread::sleep(Duration::from_millis(5));
    }
}
fn connected(link: &Link) {
    let end = Instant::now() + Duration::from_secs(3);
    loop {
        let left = end.saturating_duration_since(Instant::now());
        assert!(!left.is_zero(), "no Connected event");
        if matches!(link.events.recv_timeout(left).unwrap(), LinkEvent::Connected) { break; }
    }
}

#[test]
fn high_latency2_connects_without_heartbeat_and_populates_state_before_event() {
    let (link, mut peer) = Peer::connect();
    let mut gcs = packet(); gcs.mavtype = MavType::MAV_TYPE_GCS;
    for _ in 0..4 { peer.send(1, MavMessage::HIGH_LATENCY2(gcs.clone())); peer.send(0, MavMessage::HIGH_LATENCY2(packet())); }
    peer.send(1, MavMessage::HIGH_LATENCY2(packet()));
    wait(&link, |s| s.inspector.contains_key(&(42,1,235)));
    std::thread::sleep(Duration::from_millis(100));
    assert_ne!(link.status(), LinkStatus::Connected);
    peer.send(1, MavMessage::HIGH_LATENCY2(packet()));
    connected(&link);
    {
        let s = link.shared.read();
        assert_eq!((s.cs.sysid,s.cs.compid), (42,1));
        assert!(s.is_high_latency()); assert_eq!(s.cs.mode, "Guided");
        assert_eq!(s.cs.firmware, mp_core::firmware::Firmware::ArduPlane);
        assert_eq!((s.cs.lat,s.cs.lng,s.cs.alt), (-35.1,149.2,600.0));
        assert_eq!(s.cs.field("climbrate"), Some(-20.0)); assert_eq!(s.cs.track, [(-35.1,149.2)]);
        assert_eq!(s.mavs[&(42,1)].heading, 180.0);
    }
    peer.send(1, MavMessage::HOME_POSITION(HOME_POSITION_DATA { latitude: -351000000, longitude: 1492000000, altitude: 500000, ..Default::default() }));
    peer.send(1, MavMessage::HIGH_LATENCY2(packet()));
    wait(&link, |s| s.cs.alt == 100.0);
    let mut other = packet(); other.altitude = 999;
    peer.send(2, MavMessage::HIGH_LATENCY2(other));
    wait(&link, |s| s.inspector.contains_key(&(42,2,235)));
    assert_eq!(link.shared.read().cs.alt, 100.0);
    peer.send(1, MavMessage::HIGH_LATENCY(HIGH_LATENCY_DATA { base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode: 10, latitude: -352000000, longitude: 1493000000, altitude_amsl: 650, altitude_sp: 170,
        roll: 2500, heading: 18000, gps_fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX, gps_nsat: 10, ..Default::default() }));
    wait(&link, |s| s.cs.alt == 150.0);
    let s = link.shared.read();
    assert_eq!(s.cs.mode, "Auto"); assert_eq!(s.cs.roll,25.0); assert_eq!(s.cs.gpsstatus,3);
    assert_eq!(s.cs.field("alt_error"),Some(20.0)); assert_eq!(s.cs.track.last(),Some(&(-35.2,149.3)));
}

#[test]
fn high_latency2_non_autopilot_component_waits_for_four_identity_packets() {
    let (link, mut peer) = Peer::connect();
    for _ in 0..3 { peer.send(9, MavMessage::HIGH_LATENCY2(packet())); }
    wait(&link, |s| s.inspector.get(&(42,9,235)).is_some_and(|m| m.count == 3));
    assert_ne!(link.status(),LinkStatus::Connected);
    peer.send(9, MavMessage::HIGH_LATENCY2(packet()));
    connected(&link);
    let s=link.shared.read(); assert_eq!(s.cs.compid,9); assert_eq!(s.cs.mode,"Guided");
}
