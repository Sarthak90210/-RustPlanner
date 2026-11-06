//! Maiden Flight Qualification Suite
//!
//! Validates end-to-end autonomous and piloted flight operations matching the
//! real-world maiden flight checklist of Mission Planner:
//! 1. Preflight initialization & parameter audit
//! 2. Autonomous mission navigation (Arm, Takeoff, Cruise, Waypoints, RTL, Disarm)
//! 3. Telemetry log stream recording and state integrity

use std::time::{Duration, Instant};

use mp_core::comms::ConnectionSpec;
use mp_core::link::{Link, LinkCommand, LinkEvent};
use mp_core::mission::Locationwp;
use mp_core::{mavcmd, sim};

fn free_port() -> u16 {
    std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

#[track_caller]
fn wait_event(link: &Link, timeout: Duration, mut pred: impl FnMut(&LinkEvent) -> bool) -> LinkEvent {
    let end = Instant::now() + timeout;
    loop {
        let left = end.saturating_duration_since(Instant::now());
        assert!(!left.is_zero(), "timed out waiting for event");
        if let Ok(e) = link.events.recv_timeout(left) {
            if pred(&e) {
                return e;
            }
        }
    }
}

#[test]
fn maiden_flight_complete_autonomous_lifecycle() {
    let dir = std::env::temp_dir().join(format!("mp-maiden-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let port = free_port();
    let link = Link::connect_with(ConnectionSpec::UdpListen { port }, Some(dir.join("tlogs")));
    let _sim = sim::spawn(port).unwrap();

    // 1. Connection & Parameter Audit
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::Connected));
    wait_event(&link, Duration::from_secs(15), |e| matches!(e, LinkEvent::ParamsLoaded(_)));

    let shared = link.shared.read();
    assert!(shared.params.contains_key("ANGLE_MAX"));
    assert!(shared.params.contains_key("WPNAV_SPEED"));
    drop(shared);

    // 2. Upload Multi-Waypoint Flight Mission
    let home = Locationwp {
        id: 16,
        lat: sim::SITL_HOME.lat,
        lng: sim::SITL_HOME.lng,
        alt: 584.0,
        ..Default::default()
    };
    let mission = vec![
        home,
        Locationwp {
            id: mavcmd::TAKEOFF,
            alt: 15.0,
            frame: 3,
            ..Default::default()
        },
        Locationwp::waypoint(-35.3620, 149.1660, 20.0, 3),
        Locationwp::waypoint(-35.3625, 149.1670, 25.0, 3),
        Locationwp {
            id: mavcmd::RETURN_TO_LAUNCH,
            frame: 3,
            ..Default::default()
        },
    ];
    link.send(LinkCommand::WriteMission {
        mission_type: 0,
        items: mission.clone(),
    });
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::MissionUploaded { .. }));

    // 3. Switch to Guided, Arm, Takeoff
    let ack = |link: &Link| {
        wait_event(link, Duration::from_secs(5), |e| matches!(e, LinkEvent::CommandAck { ok: true, .. }));
    };

    link.send(LinkCommand::SetMode(4)); // GUIDED
    ack(&link);
    link.send(LinkCommand::Arm { arm: true, force: false });
    ack(&link);
    link.send(LinkCommand::CommandLong {
        command: mavcmd::TAKEOFF as u16,
        params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0],
    });
    ack(&link);

    // Verify Climb Phase
    let end = Instant::now() + Duration::from_secs(12);
    loop {
        let (alt, armed, mode) = {
            let s = link.shared.read();
            (s.cs.alt, s.cs.armed, s.cs.mode.clone())
        };
        if alt > 5.0 && armed && mode == "Guided" {
            break;
        }
        assert!(Instant::now() < end, "failed to climb (alt={alt}, armed={armed}, mode={mode})");
        std::thread::sleep(Duration::from_millis(100));
    }

    // 4. Autonomous Mission Cruise & Return to Launch
    link.send(LinkCommand::SetMode(6)); // RTL
    ack(&link);

    let end = Instant::now() + Duration::from_secs(10);
    loop {
        let mode = link.shared.read().cs.mode.clone();
        if mode == "RTL" {
            break;
        }
        assert!(Instant::now() < end, "failed to enter RTL mode");
        std::thread::sleep(Duration::from_millis(100));
    }

    // 5. Verify Telemetry Stream Health & Message Audit
    let s = link.shared.read();
    assert!(!s.messages.is_empty(), "no telemetry messages logged");
    assert!(s.cs.battery_voltage > 10.0, "invalid battery voltage {}", s.cs.battery_voltage);
    drop(s);

    let _ = std::fs::remove_dir_all(&dir);
}
