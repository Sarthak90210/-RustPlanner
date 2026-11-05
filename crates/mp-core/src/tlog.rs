//! Telemetry logs (`.tlog`), as written by Mission Planner: each record is an 8-byte big-endian
//! timestamp (µs since the Unix epoch) followed by one raw MAVLink frame. Includes the writer
//! used by the live link, a reader, and a playback driver that feeds a [`LinkShared`] so every
//! screen replays the flight.

use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink::{MavHeader, Message};
use parking_lot::RwLock;

use crate::link::{LinkShared, LinkStatus};
use crate::state::StatusMessage;

pub fn now_us() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_micros() as u64).unwrap_or(0)
}

pub struct TlogWriter {
    out: BufWriter<File>,
    pub path: PathBuf,
}

impl TlogWriter {
    /// `dir/yyyy-MM-dd HH-mm-ss.tlog`, the Mission Planner naming.
    pub fn create(dir: &Path) -> io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let name = chrono::Local::now().format("%Y-%m-%d %H-%M-%S.tlog").to_string();
        let path = dir.join(name);
        Ok(Self { out: BufWriter::new(File::create(&path)?), path })
    }

    pub fn write_frame(&mut self, frame: &[u8]) {
        let _ = self.out.write_all(&now_us().to_be_bytes());
        let _ = self.out.write_all(frame);
    }

    pub fn flush(&mut self) {
        let _ = self.out.flush();
    }
}

impl Drop for TlogWriter {
    fn drop(&mut self) {
        self.flush();
    }
}

/// One decoded tlog record.
pub struct Record {
    pub time_us: u64,
    pub header: MavHeader,
    pub msg: MavMessage,
}

/// Read every decodable record from a tlog.
pub fn read(path: &Path) -> io::Result<Vec<Record>> {
    let mut bytes = Vec::new();
    File::open(path)?.read_to_end(&mut bytes)?;
    let mut out = Vec::new();
    let mut i = 0usize;
    while i + 9 < bytes.len() {
        let ts = u64::from_be_bytes(bytes[i..i + 8].try_into().unwrap());
        let start = i + 8;
        let magic = bytes[start];
        let frame_len = match magic {
            0xFE if start + 1 < bytes.len() => 6 + bytes[start + 1] as usize + 2,
            0xFD if start + 9 < bytes.len() => {
                let signed = bytes[start + 2] & 0x01 != 0;
                10 + bytes[start + 1] as usize + 2 + if signed { 13 } else { 0 }
            }
            _ => {
                i += 1; // resync
                continue;
            }
        };
        if start + frame_len > bytes.len() {
            break;
        }
        let mut r = mavlink::MavlinkReader::new(&bytes[start..start + frame_len]);
        if let Ok((header, msg)) = r.read_any_message::<MavMessage>() {
            out.push(Record { time_us: ts, header, msg });
            i = start + frame_len;
        } else {
            i += 1;
        }
    }
    Ok(out)
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PlaybackInfo {
    pub position: f32,
    pub playing: bool,
    pub speed: f32,
    pub duration_s: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum PlaybackCommand {
    Play(bool),
    Speed(f32),
    Seek(f32),
    Stop,
}

/// Replay a tlog into `shared` on a background thread. Returns the control channel.
pub fn playback(path: &Path, shared: Arc<RwLock<LinkShared>>) -> io::Result<crossbeam_channel::Sender<PlaybackCommand>> {
    let recs = read(path)?;
    if recs.is_empty() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "no MAVLink messages in log"));
    }
    let t0 = recs[0].time_us;
    let dur = (recs.last().unwrap().time_us - t0) as f32 / 1e6;
    {
        let mut s = shared.write();
        s.status = LinkStatus::Connected;
        s.progress = None;
        s.playback = Some(PlaybackInfo { position: 0.0, playing: false, speed: 1.0, duration_s: dur });
    }
    let (tx, rx) = crossbeam_channel::unbounded();
    std::thread::Builder::new()
        .name("tlog-playback".into())
        .spawn(move || {
            let mut idx = 0usize;
            let mut playing = false;
            let mut speed = 1.0f32;
            let mut log_t = t0;
            let mut wall = Instant::now();
            let mut target: Option<(u32, u8)> = None;
            loop {
                while let Ok(c) = rx.try_recv() {
                    match c {
                        PlaybackCommand::Play(p) => {
                            playing = p;
                            wall = Instant::now();
                        }
                        PlaybackCommand::Speed(s) => speed = s.max(0.05),
                        PlaybackCommand::Seek(f) => {
                            let want = t0 + ((recs.last().unwrap().time_us - t0) as f64 * f.clamp(0.0, 1.0) as f64) as u64;
                            idx = recs.partition_point(|r| r.time_us < want);
                            log_t = want;
                            shared.write().cs.track.clear();
                        }
                        PlaybackCommand::Stop => return,
                    }
                }
                if playing {
                    let el = wall.elapsed().as_secs_f64() * speed as f64;
                    wall = Instant::now();
                    log_t += (el * 1e6) as u64;
                    let mut s = shared.write();
                    while idx < recs.len() && recs[idx].time_us <= log_t {
                        let r = &recs[idx];
                        if target.is_none() {
                            if let MavMessage::HEARTBEAT(hb) = &r.msg {
                                use mavlink::dialects::ardupilotmega::{MavAutopilot, MavType};
                                if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID && hb.mavtype != MavType::MAV_TYPE_GCS {
                                    target = Some((r.header.system_id, r.header.component_id));
                                    s.cs.sysid = r.header.system_id as u8;
                                    s.cs.compid = r.header.component_id;
                                }
                            }
                        }
                        if target == Some((r.header.system_id, r.header.component_id)) {
                            s.cs.update(&r.msg);
                            if let MavMessage::STATUSTEXT(t) = &r.msg {
                                let text = t.text.to_str().unwrap_or("").trim_end_matches('\0').to_string();
                                let secs = r.time_us / 1_000_000;
                                let time = format!("{:02}:{:02}:{:02}", (secs / 3600) % 24, (secs / 60) % 60, secs % 60);
                                s.messages.push(StatusMessage { time, severity: t.severity as u8, text });
                            }
                        }
                        let _ = r.msg.message_id();
                        idx += 1;
                    }
                    s.last_packet = Some(Instant::now());
                    if idx >= recs.len() {
                        playing = false;
                    }
                    if let Some(p) = s.playback.as_mut() {
                        p.position = (log_t.saturating_sub(t0)) as f32 / 1e6 / dur.max(0.001);
                        p.playing = playing;
                        p.speed = speed;
                    }
                } else if let Some(p) = shared.write().playback.as_mut() {
                    p.playing = false;
                    p.speed = speed;
                    p.position = (log_t.saturating_sub(t0)) as f32 / 1e6 / dur.max(0.001);
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        })?;
    Ok(tx)
}
