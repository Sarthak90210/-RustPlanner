//! Inspect production-link fields using an independent byte-stream peer.
//! TCP keeps delivery deterministic while ACKs are deliberately withheld to test
//! retries and timeouts. The simulator suite separately exercises UDP.
#![allow(deprecated)] // Mission Planner's altitude-only operation uses MISSION_ITEM.
use mavlink::dialects::ardupilotmega::*;
use mp_core::{
    comms::ConnectionSpec,
    link::{Link, LinkCommand, LinkEvent},
    mission::Locationwp,
    mavlink,
};
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::Mutex;
use std::time::{Duration, Instant};

struct Peer {
    reader: Mutex<mavlink::MavlinkReader<TcpStream>>,
    writer: Mutex<TcpStream>,
}
impl Peer {
    fn connect() -> (Link, Self) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let link = Link::connect_with_params(
            ConnectionSpec::Tcp {
                host: "127.0.0.1".into(),
                port: listener.local_addr().unwrap().port(),
            },
            None,
            false,
        );
        let (stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_millis(40)))
            .unwrap();
        stream.set_nodelay(true).unwrap();
        let peer = Self {
            writer: Mutex::new(stream.try_clone().unwrap()),
            reader: Mutex::new(mavlink::MavlinkReader::new(stream)),
        };
        receive(&peer, Duration::from_secs(3), |m| {
            matches!(m, MavMessage::HEARTBEAT(_))
        });
        send(
            &peer,
            1,
            MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                mavtype: MavType::MAV_TYPE_FIXED_WING,
                autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode: MavModeFlag::empty(),
                ..Default::default()
            }),
        );
        loop {
            if matches!(
                link.events.recv_timeout(Duration::from_secs(3)).unwrap(),
                LinkEvent::Connected
            ) {
                break;
            }
        }
        (link, peer)
    }
    fn message(&self) -> Option<MavMessage> {
        match self.reader.lock().unwrap().read_any_message::<MavMessage>() {
            Ok((_, message)) => Some(message),
            Err(mavlink::error::MessageReadError::Io(e))
                if mp_core::comms::is_timeout(&e) =>
            {
                None
            }
            Err(e) => panic!("Peer read failed: {e:?}"),
        }
    }
}
fn receive(
    peer: &Peer,
    timeout: Duration,
    mut predicate: impl FnMut(&MavMessage) -> bool,
) -> MavMessage {
    let end = Instant::now() + timeout;
    while Instant::now() < end {
        if let Some(message) = peer.message() {
            if predicate(&message) {
                return message;
            }
        }
    }
    panic!("No expected packet before deadline");
}
fn send(peer: &Peer, component: u8, message: MavMessage) {
    let mut bytes = Vec::new();
    mavlink::write_versioned_msg(
        &mut bytes,
        mavlink::MavlinkVersion::V2,
        mavlink::MavHeader {
            system_id: 42,
            component_id: component,
            sequence: 0,
        },
        &message,
    )
    .unwrap();
    peer.writer.lock().unwrap().write_all(&bytes).unwrap();
}

#[test]
fn guided_cache_observes_legacy_items_and_mission_type_guards_on_the_wire() {
    use mavlink::Message;
    let (link, peer) = Peer::connect();
    let key = (42, 1);
    link.shared.write().guided_targets.insert(key, Locationwp::waypoint(-35.0, 149.0, 50.0, 3));
    for mission_type in [MavMissionType::MAV_MISSION_TYPE_FENCE, MavMissionType::MAV_MISSION_TYPE_RALLY] {
        for msg in [
            MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA { current: 2, target_system: 42, target_component: 1, mission_type, z: 999.0, ..Default::default() }),
            MavMessage::MISSION_ITEM(MISSION_ITEM_DATA { current: 2, target_system: 42, target_component: 1, mission_type, z: 999.0, ..Default::default() }),
        ] {
            let id = msg.message_id(); link.send(LinkCommand::Send(Box::new(msg)));
            receive(&peer, Duration::from_secs(3), |m| m.message_id() == id);
            assert_eq!(link.shared.read().guided_target().alt, 50.0);
        }
    }
    link.send(LinkCommand::Send(Box::new(MavMessage::MISSION_ITEM(MISSION_ITEM_DATA { current: 2, target_system: 42, target_component: 1, mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION, x: -34.5, y: 148.5, z: 80.0, frame: MavFrame::MAV_FRAME_GLOBAL, ..Default::default() }))));
    receive(&peer, Duration::from_secs(3), |m| matches!(m, MavMessage::MISSION_ITEM(_)));
    let target = link.shared.read().guided_target();
    assert_eq!((target.lat, target.lng, target.alt, target.frame), (-34.5,148.5,80.0,0));
    link.send(LinkCommand::Send(Box::new(MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
        target_system: 42, target_component: 1, lat_int: -330000000, lon_int: 1470000000, alt: 100.0,
        coordinate_frame: MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT, type_mask: PositionTargetTypemask::from_bits_truncate(7), ..Default::default()
    }))));
    receive(&peer, Duration::from_secs(3), |m| matches!(m, MavMessage::SET_POSITION_TARGET_GLOBAL_INT(_)));
    assert_eq!(link.shared.read().guided_target(), Locationwp::waypoint(-33.0,147.0,100.0,10));
}

#[test]
fn guided_cache_follows_vehicle_selection_and_raw_position_commands() {
    let (link, peer) = Peer::connect();
    link.send(LinkCommand::Guided { lat: -35.1, lng: 149.2, alt: 55.0, frame: 10 });
    receive(&peer, Duration::from_secs(3), |m| matches!(m, MavMessage::MISSION_ITEM_INT(_)));
    let first = link.shared.read().guided_target();
    assert_eq!((first.alt, first.frame), (55.0, 10));
    // Formation and other callers can send targets for a nonselected MAV.
    link.send(LinkCommand::Send(Box::new(MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
        target_system: 43, target_component: 2, lat_int: -340000000, lon_int: 1480000000,
        alt: 80.0, coordinate_frame: MavFrame::MAV_FRAME_GLOBAL,
        type_mask: PositionTargetTypemask::from_bits_truncate(0x0df8), ..Default::default()
    }))));
    receive(&peer, Duration::from_secs(3), |m| matches!(m, MavMessage::SET_POSITION_TARGET_GLOBAL_INT(p) if p.target_system == 43));
    assert_eq!(link.shared.read().guided_target(), first);
    for (sysid, compid) in [(43, 2), (44, 1), (42, 1)] {
        link.send(LinkCommand::SetTarget { sysid, compid });
        let until = Instant::now() + Duration::from_secs(2);
        while { let s = link.shared.read(); (s.cs.sysid, s.cs.compid) } != (sysid, compid) && Instant::now() < until { std::thread::sleep(Duration::from_millis(5)); }
        let selected = link.shared.read().guided_target();
        match sysid {
            43 => { assert_eq!((selected.lat, selected.lng, selected.alt, selected.frame), (-34.0, 148.0, 80.0, 0)); }
            44 => { assert_eq!(selected.alt, 0.0); link.shared.write().set_guided_altitude(12.0, 3); }
            42 => assert_eq!(selected, first),
            _ => unreachable!(),
        }
    }
    let (other, _peer) = Peer::connect();
    assert_eq!(other.shared.read().guided_target().alt, 0.0);
    assert_eq!(link.shared.read().guided_target(), first);
}

#[test]
fn guided_frames_survive_the_production_link_for_plane_and_copter() {
    for mavtype in [MavType::MAV_TYPE_FIXED_WING, MavType::MAV_TYPE_QUADROTOR] {
        let (link, peer) = Peer::connect();
        send(&peer, 1, MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype, autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            custom_mode: if mavtype == MavType::MAV_TYPE_FIXED_WING { 15 } else { 4 },
            ..Default::default()
        }));
        let until = Instant::now() + Duration::from_secs(2);
        while link.shared.read().cs.mavtype != mavtype as u32 && Instant::now() < until { std::thread::sleep(Duration::from_millis(5)); }
        for (frame, expected) in [(0, MavFrame::MAV_FRAME_GLOBAL), (3, MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT), (10, MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT)] {
            link.send(LinkCommand::Guided { lat: -35.1, lng: 149.2, alt: 55.0, frame });
            let msg = receive(&peer, Duration::from_secs(3), |m| matches!(m, MavMessage::MISSION_ITEM_INT(_) | MavMessage::SET_POSITION_TARGET_GLOBAL_INT(_)));
            match msg {
                MavMessage::MISSION_ITEM_INT(wp) => { assert_eq!(wp.frame, expected); assert_eq!(wp.current, 2); assert_eq!(wp.z, 55.0); }
                MavMessage::SET_POSITION_TARGET_GLOBAL_INT(wp) => { assert_eq!(wp.coordinate_frame, expected); assert_eq!(wp.alt, 55.0); }
                _ => unreachable!(),
            }
            assert_eq!(link.shared.read().guided_targets[&(42,1)].frame, frame);
        }
    }
}

#[test]
fn parameter_clear_then_reassign_waits_for_each_readback_and_cancels_on_target_change() {
    let (link, peer) = Peer::connect();
    for value in [0.0, 10.0] { link.send(LinkCommand::SetParam { name: "SERVO9_FUNCTION".into(), value }); }
    let next = || {
        let MavMessage::PARAM_SET(p) = receive(&peer, Duration::from_secs(3), |m| matches!(m, MavMessage::PARAM_SET(_))) else { unreachable!() };
        assert_eq!((p.target_system, p.target_component), (42, 1));
        p
    };
    let first = next();
    assert_eq!(first.param_value, 0.0);
    let end = Instant::now() + Duration::from_millis(150);
    while Instant::now() < end { assert!(!matches!(peer.message(), Some(MavMessage::PARAM_SET(_))), "A second value overtook its pending readback"); }
    let ack = |p: PARAM_SET_DATA, value: f32| {
        send(&peer, 1, MavMessage::PARAM_VALUE(PARAM_VALUE_DATA { param_id: p.param_id, param_value: value, param_type: MavParamType::MAV_PARAM_TYPE_REAL32, param_count: 1, param_index: 0 }));
    };
    let result = || {
        loop {
            if let LinkEvent::ParamSet { name, ok } = link.events.recv_timeout(Duration::from_secs(3)).unwrap() {
                assert_eq!(name, "SERVO9_FUNCTION");
                return ok;
            }
        }
    };
    ack(first, 0.0);
    assert!(result());
    let second = next();
    assert_eq!(second.param_value, 10.0);
    ack(second, 10.0);
    assert!(result());
    assert_eq!(link.shared.read().params["SERVO9_FUNCTION"].value, 10.0);

    for value in [0.0, 10.0] { link.send(LinkCommand::SetParam { name: "SERVO9_FUNCTION".into(), value }); }
    let first = next();
    ack(first, 3.0); // The autopilot rejects the requested value.
    assert!(!result());
    let second = next();
    assert_eq!(second.param_value, 10.0);
    ack(second, 10.0);
    assert!(result());

    for value in [0.0, 10.0] { link.send(LinkCommand::SetParam { name: "SERVO9_FUNCTION".into(), value }); }
    assert_eq!(next().param_value, 0.0);
    link.send(LinkCommand::SetTarget { sysid: 43, compid: 1 });
    assert!(!result());
    let end = Instant::now() + Duration::from_millis(850);
    while Instant::now() < end { assert!(!matches!(peer.message(), Some(MavMessage::PARAM_SET(_))), "An old parameter write survived the vehicle switch"); }
}

#[test]
fn parameter_writes_across_different_names_follow_ack_order() {
    let (link, peer) = Peer::connect();
    let writes = [("SERVO9_FUNCTION", 0.0), ("SERVO9_FUNCTION", 10.0), ("CAM_TRIGG_TYPE", 0.0), ("SERVO7_FUNCTION", 7.0)];
    for (name, value) in writes { link.send(LinkCommand::SetParam { name: name.into(), value }); }
    for (name, value) in writes {
        let MavMessage::PARAM_SET(p) = receive(&peer, Duration::from_secs(3), |m| matches!(m, MavMessage::PARAM_SET(_))) else { unreachable!() };
        let actual = String::from_utf8_lossy(&p.param_id[..]).trim_end_matches('\0').to_string();
        assert_eq!((actual.as_str(), p.param_value), (name, value));
        let end = Instant::now() + Duration::from_millis(100);
        while Instant::now() < end { assert!(!matches!(peer.message(), Some(MavMessage::PARAM_SET(_))), "A different parameter overtook the previous readback"); }
        send(&peer, 1, MavMessage::PARAM_VALUE(PARAM_VALUE_DATA { param_id: p.param_id, param_value: value, param_type: MavParamType::MAV_PARAM_TYPE_REAL32, param_count: 3, param_index: 0 }));
    }
}
fn wait_result(link: &Link) -> Result<(), String> {
    let end = Instant::now() + Duration::from_secs(7);
    loop {
        if let LinkEvent::AltitudeChanged(result) = link
            .events
            .recv_timeout(end.saturating_duration_since(Instant::now()))
            .unwrap()
        {
            return result;
        }
    }
}

#[test]
fn altitude_change_wire_ack_rejection_and_timeout() {
    let (link, socket) = Peer::connect();
    let ack = |result| {
        MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: 255,
            target_component: 190,
            mavtype: result,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            ..Default::default()
        })
    };

    link.send(LinkCommand::ChangeAltitude { alt: 32.5 });
    let message = receive(&socket, Duration::from_secs(3), |m| {
        matches!(m, MavMessage::MISSION_ITEM(_))
    });
    let MavMessage::MISSION_ITEM(item) = message else {
        unreachable!()
    };
    assert_eq!((item.current, item.x, item.y, item.seq), (3, 0.0, 0.0, 0));
    assert_eq!(
        (item.target_system, item.target_component, item.z),
        (42, 1, 32.5)
    );
    assert_eq!(item.frame, MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert_eq!(item.command, MavCmd::MAV_CMD_NAV_WAYPOINT);
    // A camera component on the same system cannot acknowledge an autopilot operation.
    send(&socket, 100, ack(MavMissionResult::MAV_MISSION_ACCEPTED));
    assert!(link
        .events
        .recv_timeout(Duration::from_millis(100))
        .is_err());
    send(&socket, 1, ack(MavMissionResult::MAV_MISSION_ACCEPTED));
    let accepted = wait_result(&link);
    assert!(
        accepted.is_ok(),
        "{accepted:?}, received ACKs {:?}",
        link.shared
            .read()
            .inspector
            .iter()
            .filter(|(k, _)| k.2 == 47)
            .collect::<Vec<_>>()
    );

    link.send(LinkCommand::ChangeAltitude { alt: 40.0 });
    receive(&socket, Duration::from_secs(3), |m| {
        matches!(m, MavMessage::MISSION_ITEM(_))
    });
    send(&socket, 1, ack(MavMissionResult::MAV_MISSION_DENIED));
    assert!(wait_result(&link).unwrap_err().contains("DENIED"));

    link.send(LinkCommand::ChangeAltitude { alt: 50.0 });
    for _ in 0..3 {
        receive(&socket, Duration::from_secs(3), |m| {
            matches!(m, MavMessage::MISSION_ITEM(_))
        });
    }
    assert!(wait_result(&link).unwrap_err().contains("timed out"));

    // Selecting a different vehicle cancels the old operation before retries/ACKs.
    link.send(LinkCommand::ChangeAltitude { alt: 60.0 });
    receive(&socket, Duration::from_secs(3), |m| {
        matches!(m, MavMessage::MISSION_ITEM(_))
    });
    link.send(LinkCommand::SetTarget {
        sysid: 43,
        compid: 1,
    });
    assert!(wait_result(&link)
        .unwrap_err()
        .contains("selected vehicle changed"));
    let end = Instant::now() + Duration::from_millis(1800);
    while Instant::now() < end {
        if let Some(message) = socket.message() {
            assert!(
                !matches!(message, MavMessage::MISSION_ITEM(_)),
                "Altitude retry survived the target switch"
            );
        }
    }
}

#[test]
fn command_queue_retries_each_action_and_cancellation_stops_old_work() {
    let (link, socket) = Peer::connect();
    let ack = |command, result| {
        MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command,
            result,
            target_system: 255,
            target_component: 190,
            ..Default::default()
        })
    };
    let command = |code, p1, p2| LinkCommand::CommandLong {
        command: code,
        params: [p1, p2, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    let next = || {
        let m = receive(
            &socket,
            Duration::from_secs(3),
            |m| matches!(m, MavMessage::COMMAND_LONG(c) if matches!(c.command as u16, 178 | 181 | 183 | 400)),
        );
        let MavMessage::COMMAND_LONG(c) = m else {
            unreachable!()
        };
        c
    };
    link.send(command(178, 0.0, 10.0));
    link.send(command(183, 6.0, 1700.0));
    let first = next();
    assert_eq!(first.command, MavCmd::MAV_CMD_DO_CHANGE_SPEED);
    let retry = next(); // Drop the first packet, so its own retry must precede the servo.
    assert_eq!(retry.command, MavCmd::MAV_CMD_DO_CHANGE_SPEED);
    assert!(retry.confirmation >= 1);
    send(
        &socket,
        1,
        ack(first.command, MavResult::MAV_RESULT_ACCEPTED),
    );
    let servo = next();
    assert_eq!(
        (servo.command, servo.param1, servo.param2),
        (MavCmd::MAV_CMD_DO_SET_SERVO, 6.0, 1700.0)
    );
    link.send(command(183, 7.0, 1800.0));
    send(
        &socket,
        1,
        ack(servo.command, MavResult::MAV_RESULT_IN_PROGRESS),
    );
    let end = Instant::now() + Duration::from_millis(150);
    while Instant::now() < end {
        if let Some(m) = socket.message() {
            assert!(
                !matches!(m, MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_DO_SET_SERVO && c.param1 == 7.0)
            );
        }
    }
    send(
        &socket,
        1,
        ack(servo.command, MavResult::MAV_RESULT_ACCEPTED),
    );
    assert_eq!(next().param1, 7.0);
    send(
        &socket,
        1,
        ack(servo.command, MavResult::MAV_RESULT_ACCEPTED),
    );

    link.send(command(178, 0.0, 12.0));
    assert_eq!(next().command, MavCmd::MAV_CMD_DO_CHANGE_SPEED);
    link.send(command(400, 1.0, 0.0));
    link.send(command(181, 3.0, 0.0));
    // Relay OFF reaches the wire immediately while speed is still waiting for an ACK.
    let relay = next();
    assert_eq!(
        (relay.command, relay.param1, relay.param2),
        (MavCmd::MAV_CMD_DO_SET_RELAY, 3.0, 0.0)
    );
    link.send(LinkCommand::CancelCommands);
    let end = Instant::now() + Duration::from_millis(1800);
    while Instant::now() < end {
        if let Some(m) = socket.message() {
            assert!(
                !matches!(m, MavMessage::COMMAND_LONG(c) if matches!(c.command as u16, 178 | 181 | 183 | 400)),
                "A cancelled action was sent or retried"
            );
        }
    }
}
