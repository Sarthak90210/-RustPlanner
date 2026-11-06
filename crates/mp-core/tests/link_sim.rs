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
fn connect_without_parameters_then_request_explicitly() {
    // A separate port per process also allows concurrent workspace test runs.
    let port = std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let link = Link::connect_with_params(ConnectionSpec::UdpListen { port }, None, false);
    assert!(link.shared.read().skip_params);
    let _sim = sim::spawn(port).unwrap();
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::Connected));
    // Observe actual traffic beyond connection setup, while the default session test
    // separately verifies the normal automatic parameter fetch.
    let (tx, rx) = crossbeam_channel::unbounded();
    link.send(LinkCommand::Subscribe { msgid: 0, tx });
    rx.recv_timeout(Duration::from_secs(5)).unwrap();
    rx.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(link.shared.read().params.is_empty());
    link.send(LinkCommand::RequestParams);
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::ParamsLoaded(_)));
    assert!(link.shared.read().params.contains_key("ANGLE_MAX"));
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
        if alt > 4.0 && armed {
            assert_eq!(mode, "Guided");
            break;
        }
        assert!(Instant::now() < end, "never climbed (alt {alt})");
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(!link.shared.read().messages.is_empty());
}

#[test]
fn tlog_logs_and_playback() {
    let dir = std::env::temp_dir().join(format!("mprs-test-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let port = 14710;
    let link = Link::connect_with(ConnectionSpec::UdpListen { port }, Some(dir.join("tlogs")));
    let _sim = sim::spawn(port).unwrap();
    wait_event(&link, Duration::from_secs(30), |e| matches!(e, LinkEvent::ParamsLoaded(_)));

    // DataFlash log list + download over MAVLink
    link.send(LinkCommand::LogList);
    wait_event(&link, Duration::from_secs(5), |e| matches!(e, LinkEvent::LogList(2)));
    let entries = link.shared.read().log_entries.clone();
    assert_eq!(entries.len(), 2);
    let out = dir.join("log2.bin");
    let size = mp_core::dataflash::sample_log().len() as u32;
    assert_eq!(entries[1].1, size);
    link.send(LinkCommand::LogDownload { id: 2, size, path: out.clone() });
    match wait_event(&link, Duration::from_secs(15), |e| matches!(e, LinkEvent::LogDownloaded { .. })) {
        LinkEvent::LogDownloaded { result, .. } => result.unwrap(),
        _ => unreachable!(),
    }
    let data = std::fs::read(&out).unwrap();
    assert_eq!(data, mp_core::dataflash::sample_log());
    let parsed = mp_core::dataflash::parse(&data);
    assert_eq!(parsed.gps_track().len(), 200);

    // the tlog was recorded and parses
    let tlog = link.shared.read().tlog_path.clone().unwrap();
    std::thread::sleep(Duration::from_millis(1500));
    drop(link);
    let recs = mp_core::tlog::read(&tlog).unwrap();
    assert!(recs.len() > 50, "only {} records", recs.len());

    // playback feeds CurrentState
    let pb = Link::playback(&tlog).unwrap();
    pb.send(LinkCommand::Playback(mp_core::tlog::PlaybackCommand::Speed(20.0)));
    pb.send(LinkCommand::Playback(mp_core::tlog::PlaybackCommand::Play(true)));
    let end = Instant::now() + Duration::from_secs(5);
    loop {
        let s = pb.shared.read();
        if s.cs.lat != 0.0 && s.cs.mode != "Unknown" {
            assert_eq!(s.cs.sysid, 1);
            break;
        }
        drop(s);
        assert!(Instant::now() < end, "playback never produced a position");
        std::thread::sleep(Duration::from_millis(50));
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn mavftp_against_sim() {
    let port = 14720;
    let link = Link::connect(ConnectionSpec::UdpListen { port });
    let _sim = sim::spawn(port).unwrap();
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::Connected));

    let mut ftp = mp_core::ftp::Ftp::new(&link);
    ftp.reset().unwrap();
    let root = ftp.list("/").unwrap();
    assert!(root.iter().any(|e| e.is_dir && e.name == "APM"), "{root:?}");
    let logs = ftp.list("/APM/LOGS").unwrap();
    let bin = logs.iter().find(|e| e.name == "00000001.BIN").expect("log listed");
    let data = ftp.get("/APM/LOGS/00000001.BIN", |_, _| {}).unwrap();
    assert_eq!(data.len() as u64, bin.size);
    assert_eq!(data, mp_core::dataflash::sample_log());
    assert_eq!(ftp.crc32("/APM/LOGS/00000001.BIN").unwrap(), mp_core::ftp::crc32(0, &data));

    let payload: Vec<u8> = (0..1000u32).map(|i| (i % 251) as u8).collect();
    ftp.mkdir("/APM/test").unwrap();
    ftp.put("/APM/test/a.bin", &payload, |_, _| {}).unwrap();
    ftp.rename("/APM/test/a.bin", "/APM/test/b.bin").unwrap();
    assert_eq!(ftp.get("/APM/test/b.bin", |_, _| {}).unwrap(), payload);
    ftp.remove("/APM/test/b.bin").unwrap();
    ftp.rmdir("/APM/test").unwrap();
    assert!(ftp.list("/APM").unwrap().iter().all(|e| e.name != "test"));

    // packed parameter list with defaults (MAVLinkInterface.getParamList over FTP)
    let pck = ftp.get("@PARAM/param.pck?withdefaults=1", |_, _| {}).unwrap();
    let list = mp_core::parampck::unpack(&pck).expect("valid param.pck");
    assert!(list.len() > 500, "{} params", list.len());
    let rtl = list.iter().find(|(n, _, _)| n == "RTL_ALT").expect("RTL_ALT present");
    assert_eq!(rtl.1, 1500.0);
    assert_eq!(rtl.2, Some(1500.0));
}

#[test]
fn swarm_three_vehicles_one_link() {
    use mp_core::mavlink::dialects::ardupilotmega::{MavCmd, MavMessage, COMMAND_LONG_DATA};
    let port = 14740;
    let link = Link::connect(ConnectionSpec::UdpListen { port });
    let _sims: Vec<_> = (1..=3u8)
        .map(|i| sim::spawn_with(sim::SimOptions { gcs_port: port, sysid: i, kind: sim::SimKind::Copter, offset: (0.0, i as f64 * 10.0) }).unwrap())
        .collect();
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::Connected));
    let end = Instant::now() + Duration::from_secs(10);
    while link.shared.read().mavs.len() < 3 {
        assert!(Instant::now() < end, "only {} vehicles seen", link.shared.read().mavs.len());
        std::thread::sleep(Duration::from_millis(100));
    }
    // arm sysid 2 only (FormationControl "Arm"); resent each second like doCommand's retries,
    // since a single UDP datagram can be dropped while the whole suite runs
    let arm = || {
        link.send(LinkCommand::Send(Box::new(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 1.0,
            command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            target_system: 2,
            target_component: 1,
            ..Default::default()
        }))))
    };
    arm();
    let end = Instant::now() + Duration::from_secs(10);
    let mut next = Instant::now() + Duration::from_secs(1);
    loop {
        let armed: Vec<bool> = link.shared.read().mavs.values().map(|m| m.armed).collect();
        if armed == vec![false, true, false] {
            break;
        }
        assert!(Instant::now() < end, "armed states {armed:?}");
        if Instant::now() >= next {
            arm();
            next = Instant::now() + Duration::from_secs(1);
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

#[test]
fn lua_repl_against_sim() {
    let port = 14750;
    let link = Link::connect(ConnectionSpec::UdpListen { port });
    let _sim = sim::spawn(port).unwrap();
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::Connected));
    let repl = mp_core::repl::Repl::start(&link);
    repl.write(b"print(1+2*3)\n");
    let mut text = String::new();
    let end = std::time::Instant::now() + Duration::from_secs(15);
    while std::time::Instant::now() < end && !text.contains("> 7\n") {
        if let Ok(s) = repl.output.recv_timeout(Duration::from_millis(200)) {
            text.push_str(&s);
        }
    }
    assert!(text.contains("Hello World!!"), "{text:?}");
    assert!(text.contains("> 7\n"), "{text:?}");
}

#[test]
fn mission_over_mavftp() {
    let port = 14760;
    let link = Link::connect(ConnectionSpec::UdpListen { port });
    let _sim = sim::spawn(port).unwrap();
    wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::Connected));
    let home = mp_core::mission::Locationwp::waypoint(-35.363, 149.165, 584.0, 0);
    let wps = vec![home, mp_core::mission::Locationwp::waypoint(-35.36, 149.16, 40.0, 3), mp_core::mission::Locationwp::waypoint(-35.361, 149.161, 60.0, 3)];
    mp_core::missionpck::ftp_write(&link, 0, &wps).unwrap();
    // the vehicle now reports the same mission over the MAVLink protocol
    link.send(LinkCommand::ReadMission { mission_type: 0 });
    let got = wait_event(&link, Duration::from_secs(10), |e| matches!(e, LinkEvent::MissionDownloaded { mission_type: 0, .. }));
    let LinkEvent::MissionDownloaded { items, .. } = got else { unreachable!() };
    assert_eq!(items.len(), 3);
    assert!((items[2].lat - -35.361).abs() < 1e-6 && items[2].alt == 60.0);
    let back = mp_core::missionpck::ftp_read(&link, 0).unwrap();
    assert_eq!(back.len(), 3);
    assert_eq!(back[1].alt, 40.0);
}
