//! Exercise the production link against an independent UDP peer, inspecting wire fields.
use mavlink::dialects::ardupilotmega::*;
use mp_core::{
    comms::ConnectionSpec,
    link::{Link, LinkCommand, LinkEvent},
    mavlink,
};
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

fn receive(
    socket: &UdpSocket,
    timeout: Duration,
    mut predicate: impl FnMut(&MavMessage) -> bool,
) -> (SocketAddr, MavMessage) {
    let end = Instant::now() + timeout;
    let mut bytes = [0; 4096];
    while Instant::now() < end {
        if let Ok((n, peer)) = socket.recv_from(&mut bytes) {
            if let Ok((_, message)) =
                mavlink::MavlinkReader::new(&bytes[..n]).read_any_message::<MavMessage>()
            {
                if predicate(&message) {
                    return (peer, message);
                }
            }
        }
    }
    panic!("No expected packet before deadline");
}

fn send(socket: &UdpSocket, peer: SocketAddr, component: u8, message: MavMessage) {
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
    socket.send_to(&bytes, peer).unwrap();
}

fn wait_result(link: &Link) -> Result<(), String> {
    let end = Instant::now() + Duration::from_secs(7);
    loop {
        match link
            .events
            .recv_timeout(end.saturating_duration_since(Instant::now()))
            .unwrap()
        {
            LinkEvent::AltitudeChanged(result) => return result,
            _ => {}
        }
    }
}

#[test]
fn altitude_change_wire_ack_rejection_and_timeout() {
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket
        .set_read_timeout(Some(Duration::from_millis(50)))
        .unwrap();
    let link = Link::connect_with_params(
        ConnectionSpec::UdpClient {
            host: "127.0.0.1".into(),
            port: socket.local_addr().unwrap().port(),
        },
        None,
        false,
    );
    let (peer, _) = receive(&socket, Duration::from_secs(3), |m| {
        matches!(m, MavMessage::HEARTBEAT(_))
    });
    send(
        &socket,
        peer,
        1,
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype: MavType::MAV_TYPE_FIXED_WING,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
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
    let (_, message) = receive(&socket, Duration::from_secs(3), |m| {
        matches!(m, MavMessage::MISSION_ITEM_INT(_))
    });
    let MavMessage::MISSION_ITEM_INT(item) = message else {
        unreachable!()
    };
    assert_eq!((item.current, item.x, item.y, item.seq), (3, 0, 0, 0));
    assert_eq!(
        (item.target_system, item.target_component, item.z),
        (42, 1, 32.5)
    );
    assert_eq!(item.frame, MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert_eq!(item.command, MavCmd::MAV_CMD_NAV_WAYPOINT);
    // A camera component on the same system cannot acknowledge an autopilot operation.
    send(
        &socket,
        peer,
        100,
        ack(MavMissionResult::MAV_MISSION_ACCEPTED),
    );
    assert!(link
        .events
        .recv_timeout(Duration::from_millis(100))
        .is_err());
    send(
        &socket,
        peer,
        1,
        ack(MavMissionResult::MAV_MISSION_ACCEPTED),
    );
    assert!(wait_result(&link).is_ok());

    link.send(LinkCommand::ChangeAltitude { alt: 40.0 });
    receive(&socket, Duration::from_secs(3), |m| {
        matches!(m, MavMessage::MISSION_ITEM_INT(_))
    });
    send(&socket, peer, 1, ack(MavMissionResult::MAV_MISSION_DENIED));
    assert!(wait_result(&link).unwrap_err().contains("DENIED"));

    link.send(LinkCommand::ChangeAltitude { alt: 50.0 });
    for _ in 0..3 {
        receive(&socket, Duration::from_secs(3), |m| {
            matches!(m, MavMessage::MISSION_ITEM_INT(_))
        });
    }
    assert!(wait_result(&link).unwrap_err().contains("timed out"));
}
