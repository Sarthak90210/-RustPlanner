//! Script (MissionPlanner/Script.cs) for Python run from the Scripts tab: the script gets
//! `cs`, `Script`, `MAV`, `mavutil` and `Ports` like MP's IronPython scope. A prelude talks to
//! this host over a local TCP socket, one JSON request per line.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::time::{Duration, Instant};

use mp_core::link::{LinkCommand, LinkShared};
use mp_core::mavlink::dialects::ardupilotmega::*;
use parking_lot::RwLock;
use serde_json::{json, Value};

/// The Python side of the object model, written next to the script run.
pub const PRELUDE: &str = r#"# Mission Planner script scope: cs, Script, MAV, mavutil, Ports
import json, os, socket, sys, time

class _Host:
    def __init__(self, port):
        self.s = socket.create_connection(("127.0.0.1", port))
        self.f = self.s.makefile("rw", encoding="utf-8", newline="\n")
    def call(self, **req):
        self.f.write(json.dumps(req) + "\n")
        self.f.flush()
        r = json.loads(self.f.readline())
        if "error" in r:
            raise RuntimeError(r["error"])
        return r.get("v")

_host = _Host(int(os.environ["MP_SCRIPT_PORT"]))

class _CurrentState(object):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return _host.call(op="cs", field=name)
    def __dir__(self):
        return _host.call(op="fields")

class _Script(object):
    def Sleep(self, ms):
        time.sleep(ms / 1000.0)
    def ChangeParam(self, param, value):
        return _host.call(op="param_set", name=param, value=float(value))
    def GetParam(self, param):
        return _host.call(op="param_get", name=param)
    def ChangeMode(self, mode):
        return _host.call(op="mode", mode=str(mode))
    def WaitFor(self, message, timeout):
        end = time.time() + timeout / 1000.0
        while time.time() < end:
            if any(message in m for m in _host.call(op="messages")):
                return True
            time.sleep(0.05)
        return False
    def SendRC(self, channel, pwm, sendnow):
        return _host.call(op="rc", channel=int(channel), pwm=int(pwm), send=bool(sendnow))
    def runScript(self, filename):
        exec(compile(open(filename).read(), filename, "exec"), globals())

class _MAV(object):
    def doARM(self, arm, force=False):
        return _host.call(op="arm", arm=bool(arm), force=bool(force))
    def setMode(self, mode):
        return _host.call(op="mode", mode=str(mode))
    def setParam(self, name, value):
        return _host.call(op="param_set", name=name, value=float(value))
    def doCommand(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        return _host.call(op="command", command=int(command), params=[p1, p2, p3, p4, p5, p6, p7])
    def setGuidedModeWP(self, lat, lng, alt):
        return _host.call(op="guided", lat=float(lat), lng=float(lng), alt=float(alt))
    @property
    def cs(self):
        return cs

cs = _CurrentState()
Script = _Script()
mavutil = Script
MAV = _MAV()
Ports = [MAV]

print("hello world from python")
print(cs.roll)
if len(sys.argv) > 1:
    _f = sys.argv[1]
    sys.argv = sys.argv[1:]
    exec(compile(open(_f).read(), _f, "exec"), globals())
"#;

/// Serve one script's requests until it disconnects.
fn serve(stream: TcpStream, shared: Arc<RwLock<LinkShared>>, tx: crossbeam_channel::Sender<LinkCommand>) {
    let mut out = stream.try_clone().expect("socket clone");
    let mut rc = [0u16; 8];
    for line in BufReader::new(stream).lines() {
        let Ok(line) = line else { break };
        let reply = match serde_json::from_str::<Value>(&line) {
            Ok(req) => handle(&req, &shared, &tx, &mut rc),
            Err(e) => json!({ "error": e.to_string() }),
        };
        if writeln!(out, "{reply}").is_err() {
            break;
        }
    }
}

fn handle(req: &Value, shared: &Arc<RwLock<LinkShared>>, tx: &crossbeam_channel::Sender<LinkCommand>, rc: &mut [u16; 8]) -> Value {
    let s = |k: &str| req.get(k).and_then(|v| v.as_str()).unwrap_or("").to_string();
    let n = |k: &str| req.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0);
    match req.get("op").and_then(|v| v.as_str()).unwrap_or("") {
        "cs" => {
            let f = s("field");
            let st = shared.read();
            match st.cs.field(&f) {
                Some(v) => json!({ "v": v }),
                None => match f.as_str() {
                    "mode" => json!({ "v": st.cs.firmware.modes().into_iter().find(|(k, _)| *k == st.cs.custom_mode).map(|(_, n)| n).unwrap_or_default() }),
                    "connected" => json!({ "v": st.status == mp_core::link::LinkStatus::Connected }),
                    "firmware" => json!({ "v": format!("{:?}", st.cs.firmware) }),
                    _ => json!({ "error": format!("'CurrentState' object has no attribute '{f}'") }),
                },
            }
        }
        "fields" => json!({ "v": shared.read().cs.fields().into_iter().map(|(k, _)| k).collect::<Vec<_>>() }),
        "messages" => json!({ "v": shared.read().messages.iter().map(|m| m.text.clone()).collect::<Vec<_>>() }),
        "param_get" => match shared.read().params.get(&s("name")) {
            Some(p) => json!({ "v": p.value }),
            None => json!({ "error": format!("param not found {}", s("name")) }),
        },
        "param_set" => {
            let _ = tx.send(LinkCommand::SetParam { name: s("name"), value: n("value") as f32 });
            json!({ "v": true })
        }
        "mode" => {
            let m = s("mode");
            let k = shared.read().cs.firmware.modes().into_iter().find(|(_, name)| name.eq_ignore_ascii_case(&m)).map(|(k, _)| k);
            match k {
                Some(k) => {
                    let _ = tx.send(LinkCommand::SetMode(k));
                    json!({ "v": true })
                }
                None => json!({ "v": false }),
            }
        }
        "arm" => {
            let _ = tx.send(LinkCommand::CommandLong { command: 400, params: [if req["arm"].as_bool().unwrap_or(false) { 1.0 } else { 0.0 }, if req["force"].as_bool().unwrap_or(false) { 21196.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0, 0.0] });
            json!({ "v": true })
        }
        "command" => {
            let p: Vec<f32> = req["params"].as_array().map(|a| a.iter().map(|v| v.as_f64().unwrap_or(0.0) as f32).collect()).unwrap_or_default();
            let mut params = [0f32; 7];
            for (i, v) in p.iter().take(7).enumerate() {
                params[i] = *v;
            }
            let _ = tx.send(LinkCommand::CommandLong { command: n("command") as u16, params });
            json!({ "v": true })
        }
        "guided" => {
            let _ = tx.send(LinkCommand::Guided { lat: n("lat"), lng: n("lng"), alt: n("alt") as f32 });
            json!({ "v": true })
        }
        "rc" => {
            let ch = n("channel") as usize;
            if (1..=8).contains(&ch) {
                rc[ch - 1] = n("pwm") as u16;
            }
            if req["send"].as_bool().unwrap_or(false) {
                let (sys, comp) = {
                    let st = shared.read();
                    (st.cs.sysid, st.cs.compid)
                };
                let msg = MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
                    chan1_raw: rc[0],
                    chan2_raw: rc[1],
                    chan3_raw: rc[2],
                    chan4_raw: rc[3],
                    chan5_raw: rc[4],
                    chan6_raw: rc[5],
                    chan7_raw: rc[6],
                    chan8_raw: rc[7],
                    target_system: sys,
                    target_component: comp,
                    ..Default::default()
                });
                let _ = tx.send(LinkCommand::Send(Box::new(msg.clone())));
                std::thread::sleep(Duration::from_millis(20));
                let _ = tx.send(LinkCommand::Send(Box::new(msg)));
            }
            json!({ "v": true })
        }
        op => json!({ "error": format!("unknown request {op}") }),
    }
}

/// Listen on a free local port for one script run; returns the port.
pub fn start(shared: Arc<RwLock<LinkShared>>, tx: crossbeam_channel::Sender<LinkCommand>) -> std::io::Result<u16> {
    let l = TcpListener::bind("127.0.0.1:0")?;
    let port = l.local_addr()?.port();
    l.set_nonblocking(true)?;
    std::thread::spawn(move || {
        // the script connects straight away; give up after a minute without one
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(60) {
            match l.accept() {
                Ok((s, _)) => {
                    let _ = s.set_nonblocking(false);
                    serve(s, shared, tx);
                    return;
                }
                Err(_) => std::thread::sleep(Duration::from_millis(50)),
            }
        }
    });
    Ok(port)
}

/// Write the prelude to the data directory; the script path is passed as its argument.
pub fn prelude_path() -> std::io::Result<std::path::PathBuf> {
    let p = crate::platform::data_dir().join("scripts").join("mp_scope.py");
    std::fs::create_dir_all(p.parent().unwrap_or(std::path::Path::new(".")))?;
    std::fs::write(&p, PRELUDE)?;
    Ok(p)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests() {
        let shared: Arc<RwLock<LinkShared>> = Default::default();
        shared.write().cs.roll = 12.5;
        let (tx, rx) = crossbeam_channel::unbounded();
        let mut rc = [0u16; 8];
        assert_eq!(handle(&json!({"op":"cs","field":"roll"}), &shared, &tx, &mut rc)["v"], 12.5);
        assert!(handle(&json!({"op":"cs","field":"nope"}), &shared, &tx, &mut rc).get("error").is_some());
        handle(&json!({"op":"param_set","name":"RTL_ALT","value":1500.0}), &shared, &tx, &mut rc);
        assert!(matches!(rx.try_recv(), Ok(LinkCommand::SetParam { .. })));
        handle(&json!({"op":"rc","channel":3,"pwm":1600,"send":true}), &shared, &tx, &mut rc);
        assert_eq!(rc[2], 1600);
        assert!(matches!(rx.try_recv(), Ok(LinkCommand::Send(_))));
    }

    #[test]
    fn python_prelude_end_to_end() {
        let py = if cfg!(windows) { "python" } else { "python3" };
        if std::process::Command::new(py).arg("--version").output().is_err() {
            return;
        }
        let shared: Arc<RwLock<LinkShared>> = Default::default();
        shared.write().cs.roll = 3.5;
        shared.write().cs.alt = 42.0;
        let (tx, rx) = crossbeam_channel::unbounded();
        let port = start(shared, tx).unwrap();
        let dir = std::env::temp_dir().join(format!("mp_script_test_{port}"));
        std::fs::create_dir_all(&dir).unwrap();
        let prelude = dir.join("mp_scope.py");
        std::fs::write(&prelude, PRELUDE).unwrap();
        let script = dir.join("t.py");
        std::fs::write(&script, "print('alt', cs.alt)
Script.ChangeParam('RTL_ALT', 1500)
Script.SendRC(3, 1500, True)
").unwrap();
        let out = std::process::Command::new(py).env("MP_SCRIPT_PORT", port.to_string()).arg(&prelude).arg(&script).output().unwrap();
        let text = String::from_utf8_lossy(&out.stdout);
        assert!(out.status.success(), "{}{}", text, String::from_utf8_lossy(&out.stderr));
        assert!(text.contains("hello world from python") && text.contains("3.5") && text.contains("alt 42"), "{text}");
        assert!(matches!(rx.try_recv(), Ok(LinkCommand::SetParam { .. })));
    }
}
