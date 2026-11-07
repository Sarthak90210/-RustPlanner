//! Hardware-in-the-loop (HIL) telemetry benchmarks for BLE and WinUSB transports.
//! Exercises high-throughput MAVLink streaming, MTU fragmentation/reassembly, and RF channel noise.

use std::io::{Read, Write};
use std::time::Instant;

use mp_core::comms::{BleTransport, WinUsbTransport};
use mp_core::mavlink::dialects::ardupilotmega::*;
use mp_core::mavlink::{self, MavHeader};

fn make_heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

fn make_attitude(time_boot_ms: u32, roll: f32, pitch: f32, yaw: f32) -> MavMessage {
    MavMessage::ATTITUDE(ATTITUDE_DATA {
        time_boot_ms,
        roll,
        pitch,
        yaw,
        rollspeed: 0.01,
        pitchspeed: 0.02,
        yawspeed: 0.03,
    })
}

fn make_global_pos(lat: i32, lon: i32, alt: i32) -> MavMessage {
    MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
        time_boot_ms: 1000,
        lat,
        lon,
        alt,
        relative_alt: alt - 100,
        vx: 120,
        vy: 40,
        vz: -10,
        hdg: 18000,
    })
}

fn serialize_msg(header: &MavHeader, msg: &MavMessage) -> Vec<u8> {
    let mut buf = Vec::new();
    mavlink::write_v2_msg(&mut buf, *header, msg).expect("mavlink serialization failed");
    buf
}

#[test]
fn ble_telemetry_mtu_chunking_and_reassembly() {
    let (mut tx_dev, rx_dev) = BleTransport::loopback_pair();
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };

    // Construct telemetry burst: HEARTBEAT, ATTITUDE, and GLOBAL_POSITION_INT
    let msgs = vec![
        make_heartbeat(),
        make_attitude(100, 0.05, -0.02, 1.57),
        make_global_pos(-353632610, 1491652300, 600000),
    ];

    let mut raw_stream = Vec::new();
    for (i, msg) in msgs.iter().enumerate() {
        let mut h = header;
        h.sequence = i as u8;
        raw_stream.extend(serialize_msg(&h, msg));
    }

    assert!(raw_stream.len() > 60, "stream should contain multiple serialized frames");

    // Transmit over BLE TX
    let written = tx_dev.write(&raw_stream).expect("BLE write failed");
    assert_eq!(written, raw_stream.len());

    let tx_bytes = tx_dev.take_tx();
    assert_eq!(tx_bytes, raw_stream);

    // Simulate BLE 4.0 ATT MTU slicing (20-byte notification chunks)
    const BLE_ATT_MTU: usize = 20;
    for chunk in tx_bytes.chunks(BLE_ATT_MTU) {
        rx_dev.inject_rx(chunk);
    }

    // Read back through MAVLink parser
    let mut rx_stream = rx_dev;
    let mut read_buf = Vec::new();
    let mut scratch = [0u8; 128];
    while let Ok(n) = rx_stream.read(&mut scratch) {
        if n == 0 {
            break;
        }
        read_buf.extend_from_slice(&scratch[..n]);
    }

    assert_eq!(read_buf.len(), raw_stream.len());

    let mut reader = mavlink::MavlinkReader::new(std::io::Cursor::new(read_buf));
    let mut decoded = Vec::new();
    while let Ok((_, msg)) = reader.read_any_message::<MavMessage>() {
        decoded.push(msg);
    }

    assert_eq!(decoded.len(), msgs.len());
    match &decoded[0] {
        MavMessage::HEARTBEAT(h) => {
            assert_eq!(h.autopilot, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA);
            assert_eq!(h.mavtype, MavType::MAV_TYPE_QUADROTOR);
        }
        _ => panic!("expected HEARTBEAT"),
    }
    match &decoded[1] {
        MavMessage::ATTITUDE(a) => {
            assert!((a.roll - 0.05).abs() < 1e-4);
        }
        _ => panic!("expected ATTITUDE"),
    }
    match &decoded[2] {
        MavMessage::GLOBAL_POSITION_INT(g) => {
            assert_eq!(g.lat, -353632610);
            assert_eq!(g.lon, 1491652300);
        }
        _ => panic!("expected GLOBAL_POSITION_INT"),
    }
}

#[test]
fn winusb_bulk_endpoint_high_throughput_benchmark() {
    let (mut tx_dev, rx_dev) = WinUsbTransport::loopback_pair();
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };

    let packet_count = 500;
    let start_time = Instant::now();
    let mut total_bytes = 0;

    for i in 0..packet_count {
        let mut h = header;
        h.sequence = (i % 256) as u8;
        let msg = make_attitude(i as u32 * 20, 0.1 * (i as f32).sin(), 0.05, 3.14);
        let bytes = serialize_msg(&h, &msg);
        total_bytes += bytes.len();
        tx_dev.write_all(&bytes).expect("WinUSB write failed");
    }

    let duration_write = start_time.elapsed();
    assert_eq!(tx_dev.flush().is_ok(), true);

    let outbound = tx_dev.take_tx();
    assert_eq!(outbound.len(), total_bytes);

    // Simulate bulk-in endpoint transfers in 64-byte USB packets
    const USB_BULK_PKT_SIZE: usize = 64;
    for chunk in outbound.chunks(USB_BULK_PKT_SIZE) {
        rx_dev.inject_rx(chunk);
    }

    let mut rx_stream = rx_dev;
    let mut collected = Vec::with_capacity(total_bytes);
    let mut buf = [0u8; 512];
    while let Ok(n) = rx_stream.read(&mut buf) {
        if n == 0 {
            break;
        }
        collected.extend_from_slice(&buf[..n]);
    }

    assert_eq!(collected.len(), total_bytes);

    let mut reader = mavlink::MavlinkReader::new(std::io::Cursor::new(collected));
    let mut decoded_count = 0;
    while let Ok((_, _msg)) = reader.read_any_message::<MavMessage>() {
        decoded_count += 1;
    }

    assert_eq!(decoded_count, packet_count);

    let throughput_kb_sec = (total_bytes as f64 / 1024.0) / duration_write.as_secs_f64().max(0.0001);
    eprintln!(
        "WinUSB HIL Benchmark: {packet_count} packets ({total_bytes} bytes) transferred in {:?}, throughput: {:.2} KB/s",
        duration_write, throughput_kb_sec
    );
}

#[test]
fn rf_telemetry_jitter_and_framing_resynchronization() {
    let (mut tx_dev, rx_dev) = BleTransport::loopback_pair();
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    };

    let valid_msg = make_heartbeat();
    let valid_bytes = serialize_msg(&header, &valid_msg);

    // Inject corrupted noise / garbage preceding the valid frame
    let garbage = [0xFF, 0x00, 0x55, 0xAA, 0x12, 0x34, 0x56];
    rx_dev.inject_rx(&garbage);

    // Write valid packet
    tx_dev.write_all(&valid_bytes).unwrap();
    let tx_bytes = tx_dev.take_tx();
    rx_dev.inject_rx(&tx_bytes);

    // Also inject corrupted trailing noise
    let trailing_noise = [0xDE, 0xAD, 0xBE, 0xEF];
    rx_dev.inject_rx(&trailing_noise);

    // Verify parser skips noise and locks onto the valid MAVLink frame
    let mut rx_stream = rx_dev;
    let mut accumulated = Vec::new();
    let mut tmp = [0u8; 256];
    while let Ok(n) = rx_stream.read(&mut tmp) {
        accumulated.extend_from_slice(&tmp[..n]);
    }

    let mut reader = mavlink::MavlinkReader::new(std::io::Cursor::new(accumulated));
    let mut decoded_msgs = Vec::new();
    while let Ok((_, msg)) = reader.read_any_message::<MavMessage>() {
        decoded_msgs.push(msg);
    }

    assert_eq!(decoded_msgs.len(), 1, "MAVLink parser must resynchronize and recover valid frame");
    match &decoded_msgs[0] {
        MavMessage::HEARTBEAT(h) => {
            assert_eq!(h.system_status, MavState::MAV_STATE_ACTIVE);
        }
        _ => panic!("expected recovered HEARTBEAT"),
    }
}
