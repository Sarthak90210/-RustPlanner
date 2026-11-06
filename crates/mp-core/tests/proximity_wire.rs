//! Sensor messages traverse the production transport and link dispatch before radar use.
use std::{io::Write, net::TcpListener, time::{Duration, Instant}};
use mp_core::{comms::ConnectionSpec, link::Link, mavlink::{self, dialects::ardupilotmega::*}};

#[test]
fn radar_keeps_simultaneous_sensor_sectors_on_the_actual_link() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let link = Link::connect_with_params(ConnectionSpec::Tcp { host: "127.0.0.1".into(), port: listener.local_addr().unwrap().port() }, None, false);
    let (mut stream, _) = listener.accept().unwrap();
    let mut sequence = 0u8;
    let mut send = |component_id: u8, msg: MavMessage| {
        let mut bytes = Vec::new();
        mavlink::write_versioned_msg(&mut bytes, mavlink::MavlinkVersion::V2, mavlink::MavHeader { system_id: 42, component_id, sequence }, &msg).unwrap();
        sequence = sequence.wrapping_add(1);
        stream.write_all(&bytes).unwrap();
    };
    for _ in 0..2 {
        send(1, MavMessage::HEARTBEAT(HEARTBEAT_DATA { mavtype: MavType::MAV_TYPE_QUADROTOR, autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA, ..Default::default() }));
    }
    for (id, orientation, distance) in [(0, MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, 100), (1, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_90, 200)] {
        send(1, MavMessage::DISTANCE_SENSOR(DISTANCE_SENSOR_DATA { id, orientation, current_distance: distance, min_distance: 10, max_distance: 1000, ..Default::default() }));
    }
    send(1, MavMessage::ATTITUDE(ATTITUDE_DATA { yaw: std::f32::consts::FRAC_PI_2, ..Default::default() }));
    let mut obstacles = OBSTACLE_DISTANCE_DATA { frame: MavFrame::MAV_FRAME_GLOBAL, increment: 5, min_distance: 10, max_distance: 1000, ..Default::default() };
    obstacles.distances.fill(u16::MAX);
    obstacles.distances[0] = 300;
    send(1, MavMessage::OBSTACLE_DISTANCE(obstacles));
    send(2, MavMessage::DISTANCE_SENSOR(DISTANCE_SENSOR_DATA { id: 0, orientation: MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, current_distance: 900, min_distance: 10, max_distance: 1000, ..Default::default() }));
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let state = link.shared.read();
        let readings = state.proximity.readings(state.cs.sysid, state.cs.compid, Instant::now());
        if readings.len() == 3 && !state.proximity.readings(42, 2, Instant::now()).is_empty() {
            assert_eq!((readings[0].angle_deg, readings[0].distance_m), (0.0, 1.0));
            assert_eq!((readings[1].angle_deg, readings[1].distance_m), (90.0, 2.0));
            assert_eq!((readings[2].angle_deg, readings[2].distance_m), (90.0, 3.0));
            assert_eq!(state.proximity.readings(42, 2, Instant::now())[0].distance_m, 9.0, "other component must not replace the selected MAVState sensor");
            break;
        }
        drop(state);
        assert!(Instant::now() < deadline, "sensor sectors did not reach radar state");
        std::thread::sleep(Duration::from_millis(5));
    }
}
