//! Exercise the production transport with an independent MAVLink byte-stream peer.
use mp_core::comms::ConnectionSpec;
use mp_core::link::{Link, LinkCommand, LinkEvent};
use mp_core::mavlink::{self, dialects::ardupilotmega::*, MavHeader, MavlinkVersion};
use mp_core::mavserial::MavSerialPort;
use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

const TELEM: SerialControlDev = SerialControlDev::SERIAL_CONTROL_DEV_TELEM1;

struct Peer {
    reader: mavlink::MavlinkReader<TcpStream>,
    writer: TcpStream,
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
            .set_read_timeout(Some(Duration::from_millis(20)))
            .unwrap();
        stream.set_nodelay(true).unwrap();
        let mut peer = Self {
            writer: stream.try_clone().unwrap(),
            reader: mavlink::MavlinkReader::new(stream),
        };
        peer.send(
            42,
            1,
            MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                ..Default::default()
            }),
        );
        while !matches!(
            link.events.recv_timeout(Duration::from_secs(3)).unwrap(),
            LinkEvent::Connected
        ) {}
        (link, peer)
    }

    fn send(&mut self, sys: u32, comp: u8, message: MavMessage) {
        let mut bytes = Vec::new();
        mavlink::write_versioned_msg(
            &mut bytes,
            MavlinkVersion::V2,
            MavHeader {
                system_id: sys,
                component_id: comp,
                sequence: 0,
            },
            &message,
        )
        .unwrap();
        self.writer.write_all(&bytes).unwrap();
    }

    fn next(&mut self) -> Option<MavMessage> {
        match self.reader.read_any_message::<MavMessage>() {
            Ok((_, m)) => Some(m),
            Err(mavlink::error::MessageReadError::Io(e))
                if mp_core::comms::is_timeout(&e) =>
            {
                None
            }
            Err(e) => panic!("Peer read failed: {e:?}"),
        }
    }

    fn serial(
        &mut self,
        mut predicate: impl FnMut(&SERIAL_CONTROL_DATA) -> bool,
    ) -> SERIAL_CONTROL_DATA {
        let end = Instant::now() + Duration::from_secs(3);
        while Instant::now() < end {
            if let Some(MavMessage::SERIAL_CONTROL(s)) = self.next() {
                if predicate(&s) {
                    return s;
                }
            }
        }
        panic!("No matching SERIAL_CONTROL before deadline");
    }
}

fn reply(device: SerialControlDev, data: &[u8]) -> SERIAL_CONTROL_DATA {
    let mut s = SERIAL_CONTROL_DATA {
        device,
        flags: SerialControlFlag::SERIAL_CONTROL_FLAG_REPLY,
        count: data.len() as u8,
        ..Default::default()
    };
    s.data[..data.len()].copy_from_slice(data);
    s
}

#[test]
fn serial_wire_target_chunking_filters_polling_and_release() {
    let (link, mut peer) = Peer::connect();
    let mut port = MavSerialPort::new(&link, TELEM, 57600).unwrap();
    port.read_timeout = Duration::from_millis(600);
    let open = peer.serial(|_| true);
    assert_eq!(
        (open.target_system, open.target_component, open.device),
        (42, 1, TELEM)
    );
    assert_eq!((open.baudrate, open.timeout, open.count), (57600, 10, 0));
    assert!(open
        .flags
        .contains(SerialControlFlag::SERIAL_CONTROL_FLAG_EXCLUSIVE));

    // A UI target switch must not redirect writes, polls, replies or release.
    link.send(LinkCommand::SetTarget {
        sysid: 43,
        compid: 1,
    });
    let payload: Vec<u8> = (0..151).map(|v| v as u8).collect();
    port.write_all(&payload).unwrap();
    let mut received = Vec::new();
    for n in [70, 70, 11] {
        let frame = peer.serial(|s| s.count > 0);
        assert_eq!(
            (frame.count, frame.target_system, frame.target_component),
            (n, 42, 1)
        );
        assert_eq!(frame.timeout, 0);
        received.extend_from_slice(&frame.data[..n as usize]);
    }
    assert_eq!(received, payload);

    peer.send(
        43,
        1,
        MavMessage::SERIAL_CONTROL(reply(TELEM, b"wrong vehicle")),
    );
    peer.send(
        42,
        100,
        MavMessage::SERIAL_CONTROL(reply(TELEM, b"wrong component")),
    );
    peer.send(
        42,
        1,
        MavMessage::SERIAL_CONTROL(reply(
            SerialControlDev::SERIAL_CONTROL_DEV_GPS1,
            b"wrong device",
        )),
    );
    let mut s = reply(TELEM, b"not a reply");
    s.flags = SerialControlFlag::empty();
    peer.send(42, 1, MavMessage::SERIAL_CONTROL(s));
    let mut s = reply(TELEM, b"other GCS");
    s.target_system = 254;
    peer.send(42, 1, MavMessage::SERIAL_CONTROL(s));
    let mut s = reply(TELEM, b"other GCS component");
    s.target_component = 191;
    peer.send(42, 1, MavMessage::SERIAL_CONTROL(s));
    let mut s = reply(TELEM, b"");
    s.count = 255; // Malformed input must neither panic nor leak trailing bytes.
    peer.send(42, 1, MavMessage::SERIAL_CONTROL(s));
    peer.send(42, 1, MavMessage::SERIAL_CONTROL(reply(TELEM, b"abc")));
    let mut s = reply(TELEM, b"def");
    s.target_system = 255;
    s.target_component = 190;
    peer.send(42, 1, MavMessage::SERIAL_CONTROL(s));
    let mut result = [0; 6];
    port.read_exact(&mut result[..2]).unwrap();
    port.read_exact(&mut result[2..]).unwrap();
    assert_eq!(&result, b"abcdef");
    assert_eq!(port.read(&mut []).unwrap(), 0);
    assert_eq!(port.write(&[]).unwrap(), 0);

    port.read_timeout = Duration::from_millis(120);
    assert_eq!(
        port.read(&mut [0]).unwrap_err().kind(),
        io::ErrorKind::TimedOut
    );
    let poll = peer.serial(|s| s.count == 0);
    assert_eq!(
        (poll.target_system, poll.target_component, poll.device),
        (42, 1, TELEM)
    );
    assert!(poll
        .flags
        .contains(SerialControlFlag::SERIAL_CONTROL_FLAG_RESPOND));

    port.close().unwrap();
    let release = peer.serial(|s| s.flags.is_empty());
    assert_eq!(
        (
            release.target_system,
            release.target_component,
            release.device
        ),
        (42, 1, TELEM)
    );
    assert_eq!(
        (release.count, release.timeout, release.baudrate),
        (0, 0, 0)
    );
    port.close().unwrap();
    assert_eq!(
        port.write(b"closed").unwrap_err().kind(),
        io::ErrorKind::BrokenPipe
    );
    assert_eq!(
        port.read(&mut [0]).unwrap_err().kind(),
        io::ErrorKind::BrokenPipe
    );
    let mut sik = mp_core::sik::Sik::new(Box::new(port));
    assert_eq!(sik.load().unwrap_err().kind(), io::ErrorKind::BrokenPipe);
    drop(sik);
    let end = Instant::now() + Duration::from_millis(100);
    while Instant::now() < end {
        assert!(
            !matches!(peer.next(), Some(MavMessage::SERIAL_CONTROL(_))),
            "close/drop released twice"
        );
    }
}

#[test]
fn serial_devices_on_one_link_do_not_cross_talk() {
    let (link, mut peer) = Peer::connect();
    let shell_device = SerialControlDev::SERIAL_CONTROL_DEV_SHELL;
    let mut telem = MavSerialPort::new(&link, TELEM, 0).unwrap();
    let mut shell = MavSerialPort::new(&link, shell_device, 0).unwrap();
    telem.read_timeout = Duration::from_millis(500);
    shell.read_timeout = Duration::from_millis(500);
    peer.serial(|s| s.device == TELEM);
    peer.serial(|s| s.device == shell_device);
    shell.write_all(b"help\r").unwrap();
    let command = peer.serial(|s| s.device == shell_device && s.count > 0);
    assert_eq!(&command.data[..command.count as usize], b"help\r");
    peer.send(
        42,
        1,
        MavMessage::SERIAL_CONTROL(reply(shell_device, b"nsh> ")),
    );
    peer.send(42, 1, MavMessage::SERIAL_CONTROL(reply(TELEM, b"OK\r\n")));
    let mut data = [0; 5];
    telem.read_exact(&mut data[..4]).unwrap();
    assert_eq!(&data[..4], b"OK\r\n");
    shell.read_exact(&mut data).unwrap();
    assert_eq!(&data, b"nsh> ");
    drop(shell);
    assert_eq!(peer.serial(|s| s.flags.is_empty()).device, shell_device);
    telem.write_all(b"AT\r").unwrap();
    peer.serial(|s| s.device == TELEM && s.count == 3);
}

#[test]
fn serial_reports_link_disconnect_and_drop_releases_port() {
    let (link, mut peer) = Peer::connect();
    let port = MavSerialPort::new(&link, TELEM, 0).unwrap();
    peer.serial(|_| true);
    drop(port);
    peer.serial(|s| s.flags.is_empty());
    let mut port = MavSerialPort::new(&link, TELEM, 0).unwrap();
    peer.serial(|_| true);
    link.send(LinkCommand::Disconnect);
    while !matches!(
        link.events.recv_timeout(Duration::from_secs(3)).unwrap(),
        LinkEvent::Disconnected(_)
    ) {}
    assert_eq!(
        port.write(b"AT").unwrap_err().kind(),
        io::ErrorKind::BrokenPipe
    );
    assert_eq!(
        port.read(&mut [0]).unwrap_err().kind(),
        io::ErrorKind::BrokenPipe
    );
    assert!(
        matches!(MavSerialPort::new(&link, TELEM, 0), Err(e) if e.kind() == io::ErrorKind::NotConnected)
    );
}

#[test]
fn sik_load_and_save_through_production_link() {
    let (link, mut peer) = Peer::connect();
    let mut port = MavSerialPort::new(&link, TELEM, 0).unwrap();
    port.read_timeout = Duration::from_millis(20);
    let worker = std::thread::spawn(move || {
        // This radio double runs on the other side of actual MAVLink encoding,
        // production link IO, source filtering and serial buffering.
        let mut radio = mp_core::sik::FakeRadio::default();
        let mut commands = Vec::new();
        let end = Instant::now() + Duration::from_secs(20);
        let mut released = false;
        while Instant::now() < end {
            let Some(MavMessage::SERIAL_CONTROL(s)) = peer.next() else {
                continue;
            };
            assert_eq!(
                (s.target_system, s.target_component, s.device),
                (42, 1, TELEM)
            );
            if s.flags.is_empty() {
                released = true;
                break;
            }
            if s.count > 0 {
                commands.extend_from_slice(&s.data[..s.count as usize]);
                radio.write_all(&s.data[..s.count as usize]).unwrap();
            }
            let mut buf = [0; 70];
            while let Ok(n) = radio.read(&mut buf) {
                if n == 0 {
                    break;
                }
                peer.send(42, 1, MavMessage::SERIAL_CONTROL(reply(TELEM, &buf[..n])));
            }
        }
        assert!(released, "Radio session never released TELEM1");
        String::from_utf8(commands).unwrap()
    });
    let mut sik = mp_core::sik::Sik::new(Box::new(port));
    assert!(sik.enter_command_mode(Duration::from_millis(10)).unwrap());
    let cfg = sik.load().unwrap();
    assert_eq!(cfg.version, "SiK 2.2 on HM-TRP");
    assert_eq!(cfg.local.len(), 16);
    assert_eq!(cfg.remote.len(), 16);
    let netid = cfg.local.iter().find(|p| p.name == "NETID").unwrap();
    let change = mp_core::sik::SikParam {
        value: "42".into(),
        ..netid.clone()
    };
    sik.save(&[change.clone()], &[change]).unwrap();
    let reloaded = sik.load().unwrap();
    assert_eq!(
        reloaded
            .local
            .iter()
            .find(|p| p.name == "NETID")
            .unwrap()
            .value,
        "42"
    );
    sik.exit().unwrap();
    drop(sik);
    let commands = worker.join().unwrap();
    for expected in [
        "+++",
        "RTS3=42\r\n",
        "ATS3=42\r\n",
        "RT&W\r\n",
        "AT&W\r\n",
        "ATO\r\n",
    ] {
        assert!(
            commands.contains(expected),
            "Missing {expected:?}: {commands}"
        );
    }
}
