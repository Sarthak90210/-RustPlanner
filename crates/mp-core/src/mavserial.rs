//! `MAVLinkSerialPort`: a serial device on the selected autopilot reached through
//! SERIAL_CONTROL. Closing (including drop) releases exclusive use of that device.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};

use crate::link::{Link, LinkCommand, LinkStatus, GCS_COMPID, GCS_SYSID};
use crate::mavlink::dialects::ardupilotmega::*;

const POLL_INTERVAL: Duration = Duration::from_millis(50);

pub struct MavSerialPort {
    tx: Sender<LinkCommand>,
    rx: Receiver<MavMessage>,
    target: (u8, u8),
    device: SerialControlDev,
    baud: u32,
    buf: VecDeque<u8>,
    pub read_timeout: Duration,
    last_poll: Instant,
    closed: bool,
}

impl MavSerialPort {
    /// `baud` 0 preserves the device's current rate. The destination is captured
    /// when opened so changing the UI vehicle cannot redirect an existing session.
    pub fn new(link: &Link, device: SerialControlDev, baud: u32) -> io::Result<Self> {
        let target = {
            let s = link.shared.read();
            if s.status != LinkStatus::Connected || link.is_playback() {
                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Connect to an autopilot first",
                ));
            }
            (s.cs.sysid, s.cs.compid)
        };
        let tx = link.sender();
        let (stx, rx) = crossbeam_channel::unbounded();
        tx.send(LinkCommand::SubscribeFrom {
            msgid: 126,
            sysid: target.0,
            compid: target.1,
            tx: stx,
        })
        .map_err(|_| disconnected())?;
        let p = Self {
            tx,
            rx,
            target,
            device,
            baud,
            buf: VecDeque::new(),
            read_timeout: Duration::from_secs(4),
            last_poll: Instant::now(),
            closed: false,
        };
        p.send(&[], Self::flags(), 10, baud)?;
        Ok(p)
    }

    fn send(
        &self,
        data: &[u8],
        flags: SerialControlFlag,
        timeout: u16,
        baud: u32,
    ) -> io::Result<()> {
        let mut d = [0u8; 70];
        d[..data.len()].copy_from_slice(data);
        self.tx
            .send(LinkCommand::Send(Box::new(MavMessage::SERIAL_CONTROL(
                SERIAL_CONTROL_DATA {
                    baudrate: baud,
                    timeout,
                    device: self.device,
                    flags,
                    count: data.len() as u8,
                    data: d,
                    target_system: self.target.0,
                    target_component: self.target.1,
                },
            ))))
            .map_err(|_| disconnected())
    }

    fn flags() -> SerialControlFlag {
        SerialControlFlag::SERIAL_CONTROL_FLAG_EXCLUSIVE
            | SerialControlFlag::SERIAL_CONTROL_FLAG_RESPOND
            | SerialControlFlag::SERIAL_CONTROL_FLAG_MULTI
    }

    fn accept(&mut self, msg: MavMessage) {
        if let MavMessage::SERIAL_CONTROL(s) = msg {
            if s.device == self.device
                && s.flags
                    .contains(SerialControlFlag::SERIAL_CONTROL_FLAG_REPLY)
                && (s.target_system == 0 || s.target_system == GCS_SYSID)
                && (s.target_component == 0 || s.target_component == GCS_COMPID)
                && s.count as usize <= s.data.len()
            {
                self.buf.extend(&s.data[..s.count as usize]);
            }
        }
    }

    fn check_open(&self) -> io::Result<()> {
        if self.closed {
            Err(disconnected())
        } else {
            Ok(())
        }
    }

    /// Release the port without changing its baud rate. Safe to call more than once.
    pub fn close(&mut self) -> io::Result<()> {
        if self.closed {
            return Ok(());
        }
        self.closed = true;
        self.buf.clear();
        self.send(&[], SerialControlFlag::empty(), 0, 0)
    }
}

fn disconnected() -> io::Error {
    io::Error::new(io::ErrorKind::BrokenPipe, "SERIAL_CONTROL session closed")
}

impl Read for MavSerialPort {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        if out.is_empty() {
            return Ok(0);
        }
        self.check_open()?;
        let end = Instant::now() + self.read_timeout;
        while self.buf.is_empty() {
            // Consume queued data even with a zero read timeout.
            match self.rx.try_recv() {
                Ok(msg) => self.accept(msg),
                Err(crossbeam_channel::TryRecvError::Disconnected) => return Err(disconnected()),
                Err(crossbeam_channel::TryRecvError::Empty) => {}
            }
            if !self.buf.is_empty() {
                break;
            }
            let now = Instant::now();
            if now >= end {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "SERIAL_CONTROL read timeout",
                ));
            }
            if self.last_poll.elapsed() >= POLL_INTERVAL {
                self.send(&[], Self::flags(), 10, self.baud)?;
                self.last_poll = now;
            }
            let wait = end
                .saturating_duration_since(now)
                .min(POLL_INTERVAL.saturating_sub(self.last_poll.elapsed()));
            match self.rx.recv_timeout(wait) {
                Ok(msg) => self.accept(msg),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Err(disconnected()),
            }
        }
        let n = out.len().min(self.buf.len());
        for (dst, b) in out.iter_mut().zip(self.buf.drain(..n)) {
            *dst = b;
        }
        Ok(n)
    }
}

impl Write for MavSerialPort {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        if data.is_empty() {
            return Ok(0);
        }
        self.check_open()?;
        // write_all handles subsequent 70-byte frames and reports an interrupted transfer.
        let chunk = &data[..data.len().min(70)];
        self.send(chunk, Self::flags(), 0, self.baud)?;
        Ok(chunk.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        // There is no remote flush ACK in SERIAL_CONTROL.
        self.check_open()
    }
}

impl Drop for MavSerialPort {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
