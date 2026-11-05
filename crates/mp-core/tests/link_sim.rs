//! End-to-end: the ground-station link talks to the built-in simulated vehicle over UDP.

use std::time::{Duration, Instant};

use mp_core::comms::ConnectionSpec;
use mp_core::link::{Link, LinkCommand, LinkEvent, LinkStatus};
use mp_core::mission::Locationwp;
use mp_core::{mavcmd, sim};

fn wait_event(
    link: &Link,
    timeout: Duration,
    mut pred: impl FnMut(&LinkEvent) -> bool,
) -> LinkEvent {
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
fn full_session_against_sim() {
    let port = 14700;
    let link = Link::connect(ConnectionSpec::UdpListen { port });
    let _sim = sim::spawn(port).unwrap();

    wait_event(&link, Duration::from_secs(10), |e| {
        matches!(e, LinkEvent::Connected)
    });
    assert_eq!(link.status(), LinkStatus::Connected);

    // Parameters are fetched automatically on connect.
    let n = match wait_event(&link, Duration::from_secs(10), |e| {
        matches!(e, LinkEvent::ParamsLoaded(_))
    }) {
        LinkEvent::ParamsLoaded(n) => n,
        _ => unreachable!(),
    };
    assert!(n > 30, "got {n} params");
    assert_eq!(link.shared.read().params["ANGLE_MAX"].value, 4500.0);

    link.send(LinkCommand::SetParam {
        name: "WPNAV_SPEED".into(),
        value: 2000.0,
    });
    match wait_event(&link, Duration::from_secs(5), |e| {
        matches!(e, LinkEvent::ParamSet { .. })
    }) {
        LinkEvent::ParamSet { name, ok } => assert!(ok && name == "WPNAV_SPEED"),
        _ => unreachable!(),
    }

    // Upload a mission (home + takeoff + 2 waypoints + RTL) and read it back.
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
            alt: 10.0,
            frame: 3,
            ..Default::default()
        },
        Locationwp::waypoint(-35.3625, 149.1655, 20.0, 3),
        Locationwp::waypoint(-35.3630, 149.1660, 25.0, 3),
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
    match wait_event(&link, Duration::from_secs(10), |e| {
        matches!(e, LinkEvent::MissionUploaded { .. })
    }) {
        LinkEvent::MissionUploaded { result, .. } => result.unwrap(),
        _ => unreachable!(),
    }

    link.send(LinkCommand::ReadMission { mission_type: 0 });
    let items = match wait_event(&link, Duration::from_secs(10), |e| {
        matches!(e, LinkEvent::MissionDownloaded { .. })
    }) {
        LinkEvent::MissionDownloaded { items, .. } => items,
        _ => unreachable!(),
    };
    assert_eq!(items.len(), mission.len());
    for (a, b) in items.iter().zip(&mission) {
        assert_eq!(a.id, b.id);
        assert!((a.lat - b.lat).abs() < 1e-6 && (a.lng - b.lng).abs() < 1e-6);
        assert_eq!(a.alt, b.alt);
    }

    // Guided takeoff: mode, arm, takeoff, and watch the altitude climb.
    link.send(LinkCommand::SetMode(4));
    wait_event(&link, Duration::from_secs(5), |e| {
        matches!(e, LinkEvent::CommandAck { ok: true, .. })
    });
    link.send(LinkCommand::Arm {
        arm: true,
        force: false,
    });
    wait_event(&link, Duration::from_secs(5), |e| {
        matches!(e, LinkEvent::CommandAck { ok: true, .. })
    });
    link.send(LinkCommand::CommandLong {
        command: 22,
        params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0],
    });
    wait_event(&link, Duration::from_secs(5), |e| {
        matches!(e, LinkEvent::CommandAck { ok: true, .. })
    });

    let end = Instant::now() + Duration::from_secs(10);
    loop {
        let (alt, armed, mode) = {
            let s = link.shared.read();
            (s.cs.alt, s.cs.armed, s.cs.mode.clone())
        };
        if alt > 4.0 {
            assert!(armed);
            assert_eq!(mode, "Guided");
            break;
        }
        assert!(Instant::now() < end, "never climbed (alt {alt})");
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(!link.shared.read().messages.is_empty());
}
