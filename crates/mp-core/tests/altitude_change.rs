//! Inspect production-link fields using an independent byte-stream peer.
//! TCP keeps delivery deterministic while ACKs are deliberately withheld to test
//! retries and timeouts. The simulator suite separately exercises UDP.
#![allow(deprecated)] // Mission Planner's altitude-only operation uses MISSION_ITEM.
use mavlink::dialects::ardupilotmega::*;
use mp_core::{
    comms::ConnectionSpec,
    link::{Link, LinkCommand, LinkEvent},
    mavlink,
};
use std::io::{self, Write};
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
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
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
