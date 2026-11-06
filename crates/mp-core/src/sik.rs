//! SiK telemetry radio configuration (`Radio/Sikradio.cs`): Hayes AT command mode over the
//! radio's serial port. `ATI5` / `RTI5` list the local / remote parameters as
//! `S<n>:<NAME>=<value>`; changes are written with `ATS<n>=v` / `RTS<n>=v`, saved with
//! `AT&W` / `RT&W` and applied with `ATZ` / `RTZ`.

use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use crate::comms::Transport;

/// One radio parameter from ATI5 / RTI5.
#[derive(Debug, Clone, PartialEq)]
pub struct SikParam {
    pub reg: u32,
    pub name: String,
    pub value: String,
}

#[derive(Debug, Clone, Default)]
pub struct SikSettings {
    /// ATI (firmware version), ATI2 (board id), ATI3 (frequency band), ATI7 (RSSI report)
    pub version: String,
    pub board: String,
    pub freq: String,
    pub rssi: String,
    pub local: Vec<SikParam>,
    pub remote_version: String,
    pub remote: Vec<SikParam>,
}

pub struct Sik {
    port: Box<dyn Transport>,
}

/// Parse ATI5 output ("S1:SERIAL_SPEED=57"), dropping a multipoint "[n]" prefix.
pub fn parse_params(text: &str) -> Vec<SikParam> {
    text.lines()
        .filter_map(|l| {
            let l = l.trim();
            let l = if l.starts_with('[') { l.split_once(']').map(|(_, r)| r.trim()).unwrap_or(l) } else { l };
            let rest = l.strip_prefix('S')?;
            let (reg, kv) = rest.split_once(':')?;
            let (name, value) = kv.split_once('=')?;
            Some(SikParam { reg: reg.trim().parse().ok()?, name: name.trim().to_string(), value: value.trim().to_string() })
        })
        .collect()
}

impl Sik {
    pub fn new(port: Box<dyn Transport>) -> Self {
        Self { port }
    }

    /// Open the radio's serial port.
    pub fn open(port: &str, baud: u32) -> io::Result<Self> {
        Ok(Self::new(crate::comms::open_serial_port(port, baud)?))
    }

    fn read_for(&mut self, d: Duration, until: Option<&str>) -> io::Result<String> {
        let end = Instant::now() + d;
        let mut out = Vec::new();
        let mut buf = [0u8; 256];
        let mut last_data: Option<Instant> = None;
        while Instant::now() < end {
            // a reply is complete once the radio has been quiet for 150 ms
            if last_data.map(|t| t.elapsed() > Duration::from_millis(150)).unwrap_or(false) {
                break;
            }
            match self.port.read(&mut buf) {
                Ok(0) => std::thread::sleep(Duration::from_millis(5)),
                Ok(n) => {
                    last_data = Some(Instant::now());
                    out.extend_from_slice(&buf[..n]);
                    if let Some(u) = until {
                        if String::from_utf8_lossy(&out).contains(u) {
                            break;
                        }
                    }
                }
                Err(e) if crate::comms::is_timeout(&e) => std::thread::sleep(Duration::from_millis(5)),
                Err(e) => return Err(e),
            }
        }
        Ok(String::from_utf8_lossy(&out).to_string())
    }

    /// "+++" with the 1 s guard times; true when the radio answers OK.
    pub fn enter_command_mode(&mut self, guard: Duration) -> io::Result<bool> {
        std::thread::sleep(guard);
        self.port.write_all(b"+++")?;
        self.port.flush()?;
        let r = self.read_for(guard + Duration::from_millis(500), Some("OK"))?;
        if r.contains("OK") {
            return Ok(true);
        }
        // already in command mode?
        Ok(self.cmd("AT")?.contains("OK"))
    }

    /// doCommand: send one command, return the reply without the echo.
    pub fn cmd(&mut self, c: &str) -> io::Result<String> {
        self.read_for(Duration::from_millis(20), None)?;
        self.port.write_all(format!("{c}\r\n").as_bytes())?;
        self.port.flush()?;
        let r = self.read_for(Duration::from_millis(700), None)?;
        Ok(r.lines().filter(|l| l.trim() != c).collect::<Vec<_>>().join("\n").trim().to_string())
    }

    /// BUT_getcurrent: read everything the form shows.
    pub fn load(&mut self) -> io::Result<SikSettings> {
        let mut s = SikSettings {
            version: self.cmd("ATI")?,
            board: self.cmd("ATI2")?,
            freq: self.cmd("ATI3")?,
            rssi: self.cmd("ATI7")?,
            local: parse_params(&self.cmd("ATI5")?),
            ..Default::default()
        };
        s.remote_version = self.cmd("RTI")?;
        if !s.remote_version.is_empty() {
            s.remote = parse_params(&self.cmd("RTI5")?);
        }
        Ok(s)
    }

    /// BUT_savesettings: write changed registers, save and reboot each side.
    pub fn save(&mut self, local: &[SikParam], remote: &[SikParam]) -> io::Result<Vec<String>> {
        let mut log = Vec::new();
        if !remote.is_empty() {
            for p in remote {
                let r = self.cmd(&format!("RTS{}={}", p.reg, p.value))?;
                log.push(format!("Remote {} = {} : {r}", p.name, p.value));
            }
            log.push(format!("RT&W {}", self.cmd("RT&W")?));
            self.cmd("RTZ")?;
        }
        for p in local {
            let r = self.cmd(&format!("ATS{}={}", p.reg, p.value))?;
            log.push(format!("Local {} = {} : {r}", p.name, p.value));
        }
        log.push(format!("AT&W {}", self.cmd("AT&W")?));
        self.cmd("ATZ")?;
        Ok(log)
    }

    /// BUT_resettodefault: AT&F, save, reboot.
    pub fn reset_defaults(&mut self) -> io::Result<()> {
        self.cmd("AT&F")?;
        self.cmd("AT&W")?;
        self.cmd("ATZ")?;
        Ok(())
    }

    pub fn exit(&mut self) -> io::Result<()> {
        self.cmd("ATO").map(|_| ())
    }
}

/// In-memory SiK radio used by the tests (and as a reference for the command set).
pub struct FakeRadio {
    pub params: Vec<SikParam>,
    pub saved: bool,
    out: Vec<u8>,
    line: String,
}

impl Default for FakeRadio {
    fn default() -> Self {
        let p = |reg: u32, name: &str, value: &str| SikParam { reg, name: name.into(), value: value.into() };
        Self {
            params: vec![p(0, "FORMAT", "25"), p(1, "SERIAL_SPEED", "57"), p(2, "AIR_SPEED", "64"), p(3, "NETID", "25"), p(4, "TXPOWER", "20"), p(5, "ECC", "0"), p(6, "MAVLINK", "1"), p(7, "OPPRESEND", "0"), p(8, "MIN_FREQ", "915000"), p(9, "MAX_FREQ", "928000"), p(10, "NUM_CHANNELS", "50"), p(11, "DUTY_CYCLE", "100"), p(12, "LBT_RSSI", "0"), p(13, "MANCHESTER", "0"), p(14, "RTSCTS", "0"), p(15, "MAX_WINDOW", "131")],
            saved: false,
            out: Vec::new(),
            line: String::new(),
        }
    }
}

impl Read for FakeRadio {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.out.is_empty() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        let n = buf.len().min(self.out.len());
        buf[..n].copy_from_slice(&self.out[..n]);
        self.out.drain(..n);
        Ok(n)
    }
}

impl Write for FakeRadio {
    fn write(&mut self, b: &[u8]) -> io::Result<usize> {
        let s = String::from_utf8_lossy(b).to_string();
        if s == "+++" {
            self.out.extend_from_slice(b"OK\r\n");
            return Ok(b.len());
        }
        self.line.push_str(&s);
        while let Some(i) = self.line.find('\n') {
            let c = self.line[..i].trim().to_string();
            self.line.drain(..=i);
            if c.is_empty() {
                continue;
            }
            let reply = match c.as_str() {
                "AT" => "OK".to_string(),
                "ATI" => "SiK 2.2 on HM-TRP".to_string(),
                "ATI2" => "130".to_string(),
                "ATI3" => "915".to_string(),
                "ATI7" => "L/R RSSI: 210/205  L/R noise: 40/38 pkts: 120  txe=0 rxe=0 stx=0 srx=0 ecc=0/0 temp=37 dco=0".to_string(),
                "RTI" => "SiK 2.2 on HM-TRP".to_string(),
                "ATI5" | "RTI5" => self.params.iter().map(|p| format!("S{}:{}={}", p.reg, p.name, p.value)).collect::<Vec<_>>().join("\r\n"),
                "AT&W" | "RT&W" => {
                    self.saved = true;
                    "OK".to_string()
                }
                c if c.starts_with("ATS") || c.starts_with("RTS") => {
                    let (reg, v) = c[3..].split_once('=').unwrap_or(("", ""));
                    match self.params.iter_mut().find(|p| p.reg.to_string() == reg) {
                        Some(p) => {
                            p.value = v.to_string();
                            "OK".to_string()
                        }
                        None => "ERROR".to_string(),
                    }
                }
                _ => "OK".to_string(),
            };
            self.out.extend_from_slice(format!("{c}\r\n{reply}\r\n").as_bytes());
        }
        Ok(b.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_and_save_against_fake_radio() {
        let mut s = Sik::new(Box::new(FakeRadio::default()));
        assert!(s.enter_command_mode(Duration::from_millis(10)).unwrap());
        let cfg = s.load().unwrap();
        assert_eq!(cfg.version, "SiK 2.2 on HM-TRP");
        let netid = cfg.local.iter().find(|p| p.name == "NETID").unwrap();
        assert_eq!(netid.value, "25");
        assert_eq!(cfg.remote.len(), cfg.local.len());
        let change = SikParam { value: "42".into(), ..netid.clone() };
        let log = s.save(&[change], &[]).unwrap();
        assert!(log.iter().any(|l| l.contains("NETID = 42 : OK")), "{log:?}");
        let again = s.load().unwrap();
        assert_eq!(again.local.iter().find(|p| p.name == "NETID").unwrap().value, "42");
    }

    #[test]
    fn multipoint_prefix() {
        let p = parse_params("[1] S1:SERIAL_SPEED=57\n[1] S3:NETID=25");
        assert_eq!(p.len(), 2);
        assert_eq!(p[1].name, "NETID");
    }
}
