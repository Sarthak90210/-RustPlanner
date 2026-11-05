//! AP_REPL: the ArduPilot Lua REPL. MAV_CMD_SCRIPTING starts it; commands are appended
//! to `repl/in` and responses read from `repl/out` over MAVFtp, polled once a second.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};

use crate::ftp::Ftp;
use crate::link::{Link, LinkCommand};
use crate::mavlink::dialects::ardupilotmega::MavMessage;

const MAV_CMD_SCRIPTING: u16 = 42701;

pub struct Repl {
    input: Sender<Vec<u8>>,
    pub output: Receiver<String>,
    stop: Arc<AtomicBool>,
}

impl Drop for Repl {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// Send MAV_CMD_SCRIPTING and wait for its COMMAND_ACK.
fn scripting(link_tx: &Sender<LinkCommand>, acks: &Receiver<MavMessage>, start: bool) -> bool {
    let _ = link_tx.send(LinkCommand::CommandLong { command: MAV_CMD_SCRIPTING, params: [if start { 0.0 } else { 1.0 }, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
    let end = Instant::now() + Duration::from_secs(3);
    while let Some(left) = end.checked_duration_since(Instant::now()) {
        match acks.recv_timeout(left) {
            Ok(MavMessage::COMMAND_ACK(a)) if a.command as u32 == MAV_CMD_SCRIPTING as u32 => {
                return a.result as u32 == 0;
            }
            Ok(_) => {}
            Err(_) => break,
        }
    }
    false
}

impl Repl {
    pub fn start(link: &Link) -> Self {
        let (itx, irx) = crossbeam_channel::unbounded::<Vec<u8>>();
        let (otx, orx) = crossbeam_channel::unbounded::<String>();
        let stop = Arc::new(AtomicBool::new(false));
        let (atx, arx) = crossbeam_channel::unbounded();
        link.send(LinkCommand::Subscribe { msgid: 77, tx: atx });
        let mut ftp = Ftp::new(link);
        let tx = link.sender();
        let st = stop.clone();
        std::thread::Builder::new()
            .name("REPL".into())
            .spawn(move || {
                if !scripting(&tx, &arx, true) {
                    let _ = otx.send("Failed to start REPL\n".into());
                    return;
                }
                let mut base = "repl/".to_string();
                let root_has = ftp.list("/").map(|l| l.iter().any(|e| e.name == "repl")).unwrap_or(false);
                if !root_has && ftp.list("/APM/").map(|l| l.iter().any(|e| e.name == "repl")).unwrap_or(false) {
                    base = "/APM/repl/".into();
                    let _ = otx.send("Base dir changed to /APM/repl/\n".into());
                }
                let mut pending: Vec<u8> = b"\"Hello World!!\"\n".to_vec();
                let mut outskip = 0usize;
                let mut next_poll = Instant::now();
                while !st.load(Ordering::Relaxed) {
                    while let Ok(b) = irx.try_recv() {
                        pending.extend(b);
                    }
                    if !pending.is_empty() {
                        // SendLine: append at the current size of `in`
                        let at = ftp.list(base.trim_end_matches('/')).ok().and_then(|l| l.into_iter().find(|e| e.name == "in").map(|e| e.size as u32)).unwrap_or(0);
                        if ftp.write_at(&format!("{base}in"), &pending, at).is_ok() {
                            pending.clear();
                        }
                        next_poll = Instant::now() + Duration::from_millis(200);
                    }
                    if Instant::now() >= next_poll {
                        next_poll = Instant::now() + Duration::from_secs(1);
                        if let Ok(out) = ftp.get(&format!("{base}out"), |_, _| {}) {
                            if out.len() > outskip {
                                let _ = otx.send(String::from_utf8_lossy(&out[outskip..]).to_string());
                                outskip = out.len();
                            }
                        }
                    }
                    if let Ok(b) = irx.recv_timeout(Duration::from_millis(50)) {
                        pending.extend(b);
                    }
                }
                if !scripting(&tx, &arx, false) {
                    let _ = otx.send("Failed to stop REPL\n".into());
                }
            })
            .ok();
        Self { input: itx, output: orx, stop }
    }

    /// Queue bytes for `repl/in` ("+++" is sent as is, other commands end in "\n").
    pub fn write(&self, bytes: &[u8]) {
        let _ = self.input.send(bytes.to_vec());
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}
