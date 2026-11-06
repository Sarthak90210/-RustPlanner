//! Button handlers for the designer pages that talk to extra hardware: Script REPL
//! (ConfigREPL / AP_REPL), Bluetooth Setup (ConfigHWBT), ESP8266 Setup (ConfigHWESP8266)
//! and CubeID Update (ConfigCubeID).

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::sync::Arc;
use std::time::Duration;

use mp_core::link::{Link, LinkCommand};
use mp_core::mavlink::dialects::ardupilotmega::*;
use parking_lot::Mutex;

use crate::app::AppCtx;
use crate::forms::FormState;

const COMP_UDP_BRIDGE: u8 = 240;
const COMP_ODID_TXRX_1: u8 = 236;

#[derive(Default)]
pub struct DevicePages {
    repl: Option<mp_core::repl::Repl>,
    repl_input: usize,
    history: Vec<String>,
    /// background job message (Bluetooth / CubeID / ESP8266), shown when it finishes
    job: Arc<Mutex<Option<String>>>,
    busy: Arc<Mutex<Option<(String, f32)>>>,
    esp: Arc<Mutex<BTreeMap<String, [u8; 4]>>>,
    esp_requested: bool,
    esp_filled: bool,
}

fn ip_to_u32(s: &str) -> Option<u32> {
    let ip: std::net::Ipv4Addr = s.trim().parse().ok()?;
    Some(u32::from_le_bytes(ip.octets()))
}

/// stringTobytearray: 4 bytes of `s` from `start`, zero padded.
fn chunk4(s: &str, start: usize) -> u32 {
    let mut b = s.as_bytes().to_vec();
    b.resize(start + 4, 0);
    u32::from_le_bytes([b[start], b[start + 1], b[start + 2], b[start + 3]])
}

impl DevicePages {
    /// Called before the form is drawn: show results and refresh live values.
    pub fn pre(&mut self, ctx: &egui::Context, form: &str, st: &mut FormState, app: &mut AppCtx) {
        if let Some(msg) = self.job.lock().take() {
            app.toasts.error(msg);
        }
        if let Some((text, p)) = self.busy.lock().clone() {
            egui::Window::new(crate::i18n::t("Progress")).collapsible(false).resizable(false).show(ctx, |ui| {
                crate::theme::label(ui, &text);
                crate::theme::progress(ui, p, egui::vec2(300.0, 20.0), "");
            });
            ctx.request_repaint_after(Duration::from_millis(200));
        }
        match form {
            "repl" => self.repl_poll(st),
            "esp8266" => self.esp_poll(st, app),
            "cubeid" => {
                let ok = app.link.map(|l| l.shared.read().cs.compid == COMP_ODID_TXRX_1).unwrap_or(false);
                if ok {
                    st.disabled.remove("but_upfw");
                } else {
                    st.disabled.insert("but_upfw".into());
                }
            }
            _ => {}
        }
    }

    /// Returns true when the button belongs to one of these pages.
    pub fn button(&mut self, form: &str, b: &str, st: &mut FormState, app: &mut AppCtx) -> bool {
        match (form, b) {
            ("repl", "BUT_Connect") => match app.link {
                Some(l) => {
                    st.set("TXT_terminal", "Starting REPL\n");
                    self.repl_input = st.get("TXT_terminal").len();
                    st.disabled.remove("BUT_disconnect");
                    self.repl = Some(mp_core::repl::Repl::start(l));
                }
                None => app.toasts.error("Please Connect First"),
            },
            ("repl", "BUT_disconnect") => {
                if let Some(r) = self.repl.take() {
                    r.stop();
                }
            }
            ("bluetooth", "BUT_btsettings") => self.bluetooth(st, app),
            ("esp8266", "BUT_espsettings") => self.esp_save(st, app),
            ("esp8266", "but_resetdefault") => match app.link {
                Some(l) => {
                    let (sys, _) = ids(l);
                    // PREFLIGHT_STORAGE 2 = reset to defaults, then 1 = save
                    for p1 in [2.0, 1.0] {
                        send_cmd(l, sys, COMP_UDP_BRIDGE, MavCmd::MAV_CMD_PREFLIGHT_STORAGE, [p1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                    }
                    self.esp_filled = false;
                    self.esp_requested = false;
                    self.esp.lock().clear();
                    app.toasts.error("Programmed OK");
                }
                None => app.toasts.error("Please Connect First"),
            },
            ("esp8266", "chk_mode") => {
                let on = st.checked.get("chk_mode").copied().unwrap_or(false);
                for c in ["txt_ip", "txt_gateway", "txt_subnet"] {
                    if on {
                        st.disabled.remove(c);
                    } else {
                        st.disabled.insert(c.into());
                    }
                }
            }
            ("cubeid", "but_upfw") => self.cubeid(None, st, app),
            ("cubeid", "but_customfw") => {
                #[cfg(not(any(target_os = "android", target_os = "ios")))]
                if let Some(p) = rfd::FileDialog::new().add_filter("*.bin", &["bin"]).pick_file() {
                    self.cubeid(Some(p), st, app);
                }
            }
            _ => return false,
        }
        true
    }

    // ---------------------------------------------------------------- REPL

    fn repl_poll(&mut self, st: &mut FormState) {
        if self.repl.is_none() && !st.text.contains_key("TXT_terminal") {
            st.set("TXT_terminal", "*NOTE* Please click connect above");
            st.disabled.insert("BUT_disconnect".into());
        }
        let Some(r) = &self.repl else { return };
        let mut text = st.get("TXT_terminal").to_string();
        if self.repl_input > text.len() {
            self.repl_input = text.len();
        }
        // typed text after the prompt
        let typed = text.get(self.repl_input..).unwrap_or("").to_string();
        if let Some(nl) = typed.find('\n') {
            let cmd = typed[..nl].trim_end_matches('\r').to_string();
            text.truncate(self.repl_input);
            if !cmd.is_empty() && self.history.last() != Some(&cmd) {
                self.history.push(cmd.clone());
            }
            if cmd == "+++" {
                r.write(cmd.as_bytes());
            } else {
                r.write(format!("{cmd}\n").as_bytes());
                // local echo
                text.push_str(&cmd);
                text.push('\n');
            }
            self.repl_input = text.len();
            text.push_str(&typed[nl + 1..]);
        }
        while let Ok(data) = r.output.try_recv() {
            let typed = text.split_off(self.repl_input.min(text.len()));
            let data = data.trim_end_matches('\r').replace('\0', "").replace("\x1b[K", "");
            text.push_str(&data);
            self.repl_input = text.len();
            text.push_str(&typed);
        }
        st.set("TXT_terminal", text);
    }

    // ---------------------------------------------------------------- Bluetooth

    /// BUT_btsettings: find the module's baud rate with "AT", then program name / baud / pin.
    fn bluetooth(&mut self, st: &FormState, app: &mut AppCtx) {
        if self.busy.lock().is_some() {
            return;
        }
        let port = app.settings.port.clone();
        if port.is_empty() || ["AUTO", "TCP", "UDP", "UDPCl", "WS"].contains(&port.as_str()) {
            app.toasts.error("Please select a comport (top right).");
            return;
        }
        let (name, pin, baud) = (st.get("txt_name").to_string(), st.get("txt_pin").to_string(), st.get("cmb_baud").to_string());
        let map: [(u32, u32); 8] = [(57600, 7), (38400, 6), (9600, 4), (19200, 5), (115200, 8), (1200, 1), (2400, 2), (4800, 3)];
        let code = map.iter().find(|(b, _)| b.to_string() == baud).map(|(_, c)| *c).unwrap_or(7);
        let cmds = vec![
            "AT".to_string(),
            "AT+VERSION".into(),
            "AT+ROLE=0\r\n".into(),
            format!("AT+NAME={name}\r\n"),
            format!("AT+NAME{name}"),
            format!("AT+BAUD={baud}\r\n"),
            format!("AT+BAUD{code}"),
            format!("AT+PSWD={pin}\r\n"),
            format!("AT+PIN{pin}"),
            "AT+RESET".into(),
        ];
        let (job, busy) = (self.job.clone(), self.busy.clone());
        *busy.lock() = Some(("Bluetooth: looking for the module".into(), 0.0));
        std::thread::spawn(move || {
            let mut pass = false;
            for (i, (b, _)) in map.iter().enumerate() {
                *busy.lock() = Some((format!("Bluetooth: trying {b} baud"), i as f32 / map.len() as f32));
                let mut p = match mp_core::comms::open_serial_port(&port, *b) {
                    Ok(p) => p,
                    Err(e) => {
                        *job.lock() = Some(format!("Please select a comport {e}"));
                        break;
                    }
                };
                let _ = p.write_all(b"AT");
                std::thread::sleep(Duration::from_millis(1100));
                let _ = p.write_all(b"\r\n");
                std::thread::sleep(Duration::from_millis(200));
                let mut buf = [0u8; 256];
                let n = p.read(&mut buf).unwrap_or(0);
                if String::from_utf8_lossy(&buf[..n]).contains("OK") {
                    for c in &cmds {
                        let _ = p.write_all(c.as_bytes());
                        std::thread::sleep(Duration::from_millis(1000));
                        let _ = p.read(&mut buf);
                    }
                    pass = true;
                    break;
                }
                std::thread::sleep(Duration::from_millis(1100));
            }
            *busy.lock() = None;
            let mut j = job.lock();
            if j.is_none() {
                *j = Some(if pass { "Programmed OK".into() } else { "Error setting parameter".into() });
            }
        });
    }

    // ---------------------------------------------------------------- ESP8266

    fn esp_poll(&mut self, st: &mut FormState, app: &mut AppCtx) {
        let Some(l) = app.link else {
            st.disabled.extend(["BUT_espsettings", "but_resetdefault"].map(String::from));
            return;
        };
        if !self.esp_requested {
            self.esp_requested = true;
            let (tx, rx) = crossbeam_channel::unbounded();
            l.send(LinkCommand::SubscribeRaw { msgid: 22, tx });
            let (sys, _) = ids(l);
            l.send(LinkCommand::Send(Box::new(MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA { target_system: sys, target_component: COMP_UDP_BRIDGE }))));
            let store = self.esp.clone();
            std::thread::spawn(move || {
                while let Ok((_, comp, mut p)) = rx.recv() {
                    if comp != COMP_UDP_BRIDGE {
                        continue;
                    }
                    p.resize(25, 0);
                    let id: String = p[8..24].iter().take_while(|b| **b != 0).map(|b| *b as char).collect();
                    store.lock().insert(id, [p[0], p[1], p[2], p[3]]);
                }
            });
        }
        let m = self.esp.lock().clone();
        if !m.contains_key("WIFI_SSID1") {
            st.disabled.extend(["BUT_espsettings", "but_resetdefault"].map(String::from));
            return;
        }
        st.disabled.remove("BUT_espsettings");
        st.disabled.remove("but_resetdefault");
        if self.esp_filled {
            return;
        }
        self.esp_filled = true;
        let s4 = |pre: &str| -> String {
            let mut b = Vec::new();
            for i in 1..=4 {
                b.extend(m.get(&format!("{pre}{i}")).copied().unwrap_or_default());
            }
            String::from_utf8_lossy(&b).trim_end_matches('\0').to_string()
        };
        let u = |k: &str| m.get(k).map(|b| u32::from_le_bytes(*b)).unwrap_or(0);
        let ip = |k: &str| std::net::Ipv4Addr::from(u(k).to_le_bytes()).to_string();
        st.set("txt_ssid", s4("WIFI_SSID"));
        st.set("txt_password", s4("WIFI_PASSWORD"));
        st.set("cmb_baud", u("UART_BAUDRATE").to_string());
        st.set("cmb_channel", u("WIFI_CHANNEL").to_string());
        st.set("txt_ip", ip("WIFI_IPSTA"));
        st.set("txt_gateway", ip("WIFI_GATEWAYSTA"));
        st.set("txt_subnet", ip("WIFI_SUBNET_STA"));
        st.checked.insert("chk_mode".into(), u("WIFI_MODE") != 0);
        st.set(
            "label5",
            format!(
                "DEBUG_ENABLED {},\nWIFI_MODE {},\nWIFI_IPADDRESS {},\nWIFI_UDP_HPORT {},\nWIFI_UDP_CPORT {},\nWIFI_IPSTA {},\nWIFI_GATEWAYSTA {},\nWIFI_SUBNET_STA {}\n",
                u("DEBUG_ENABLED"),
                u("WIFI_MODE"),
                ip("WIFI_IPADDRESS"),
                u("WIFI_UDP_HPORT"),
                u("WIFI_UDP_CPORT"),
                ip("WIFI_IPSTA"),
                ip("WIFI_GATEWAYSTA"),
                ip("WIFI_SUBNET_STA")
            ),
        );
        let _ = self.button("esp8266", "chk_mode", st, app);
    }

    fn esp_save(&mut self, st: &FormState, app: &mut AppCtx) {
        let Some(l) = app.link else {
            app.toasts.error("Please Connect First");
            return;
        };
        let (sys, _) = ids(l);
        let (ssid, pwd) = (st.get("txt_ssid").to_string(), st.get("txt_password").to_string());
        let mut list: Vec<(String, u32)> = vec![("WIFI_CHANNEL".into(), st.get("cmb_channel").trim().parse().unwrap_or(11)), ("UART_BAUDRATE".into(), st.get("cmb_baud").trim().parse().unwrap_or(921600))];
        for (pre, s) in [("WIFI_SSID", &ssid), ("WIFI_PASSWORD", &pwd), ("WIFI_SSIDSTA", &ssid), ("WIFI_PWDSTA", &pwd)] {
            for i in 0..4 {
                list.push((format!("{pre}{}", i + 1), chunk4(s, i * 4)));
            }
        }
        for (k, c) in [("WIFI_IPSTA", "txt_ip"), ("WIFI_GATEWAYSTA", "txt_gateway"), ("WIFI_SUBNET_STA", "txt_subnet")] {
            match ip_to_u32(st.get(c)) {
                Some(v) => list.push((k.into(), v)),
                None => {
                    app.toasts.error(format!("Invalid IP address {}", st.get(c)));
                    return;
                }
            }
        }
        list.push(("WIFI_MODE".into(), st.checked.get("chk_mode").copied().unwrap_or(false) as u32));
        for (k, v) in list {
            // the ESP stores raw 32 bit values in the float field
            l.send(LinkCommand::Send(Box::new(MavMessage::PARAM_SET(PARAM_SET_DATA { param_value: f32::from_bits(v), target_system: sys, target_component: COMP_UDP_BRIDGE, param_id: k.as_str().into(), param_type: MavParamType::MAV_PARAM_TYPE_UINT32 }))));
        }
        // save to eeprom, reboot
        send_cmd(l, sys, COMP_UDP_BRIDGE, MavCmd::MAV_CMD_PREFLIGHT_STORAGE, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        send_cmd(l, sys, COMP_UDP_BRIDGE, MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        app.toasts.error("Programmed OK");
    }

    // ---------------------------------------------------------------- CubeID

    /// CUBEPILOT_FIRMWARE_UPDATE_START, then one ENCAPSULATED_DATA chunk per RESP offset.
    fn cubeid(&mut self, file: Option<std::path::PathBuf>, st: &FormState, app: &mut AppCtx) {
        let Some(l) = app.link else {
            app.toasts.error("Please Connect First");
            return;
        };
        if self.busy.lock().is_some() {
            return;
        }
        let (sys, comp) = ids(l);
        let force = st.checked.get("CHK_forcebaud").copied().unwrap_or(false);
        if force {
            app.toasts.error("Force 57600 baud: reconnect the link at 57600 for the CubeID update.");
        }
        let (tx, rx) = crossbeam_channel::unbounded();
        l.send(LinkCommand::Subscribe { msgid: 50005, tx });
        let link_tx = l.sender();
        let (job, busy) = (self.job.clone(), self.busy.clone());
        *busy.lock() = Some(("Downloading".into(), 0.0));
        std::thread::spawn(move || {
            let data = match file {
                Some(p) => std::fs::read(p).map_err(|e| e.to_string()),
                None => crate::firmware_ui::get("https://firmware.cubepilot.org/UAVCAN/com.cubepilot.cubeid/1.0/serial_fw_update.bin").map_err(|_| "Bad Download".to_string()),
            };
            let data = match data {
                Ok(d) => d,
                Err(e) => {
                    *busy.lock() = None;
                    *job.lock() = Some(e);
                    return;
                }
            };
            let size = data.len() as u32;
            let crc = cubeid_crc32(&data);
            let chunk = |off: u32| {
                let len = 252.min(size.saturating_sub(off)) as usize;
                let mut d = [0u8; 253];
                d[..len].copy_from_slice(&data[off as usize..off as usize + len]);
                let _ = link_tx.send(LinkCommand::Send(Box::new(MavMessage::ENCAPSULATED_DATA(ENCAPSULATED_DATA_DATA { seqnr: (off / 252) as u16, data: d }))));
                len
            };
            let (mut seen, mut offset) = (false, 0u32);
            let started = std::time::Instant::now();
            loop {
                let _ = link_tx.send(LinkCommand::Send(Box::new(MavMessage::CUBEPILOT_FIRMWARE_UPDATE_START(CUBEPILOT_FIRMWARE_UPDATE_START_DATA { size, crc, target_system: sys, target_component: comp }))));
                let end = std::time::Instant::now() + Duration::from_secs(1);
                let mut done = false;
                while let Some(left) = end.checked_duration_since(std::time::Instant::now()) {
                    if let Ok(MavMessage::CUBEPILOT_FIRMWARE_UPDATE_RESP(r)) = rx.recv_timeout(left) {
                        seen = true;
                        offset = r.offset;
                        if offset > size || chunk(offset) == 0 {
                            done = true;
                            break;
                        }
                    }
                }
                *busy.lock() = Some((format!("Updating {offset} Seen HW: {seen}"), offset as f32 / size.max(1) as f32));
                if done {
                    break;
                }
                if seen && (offset > size || chunk(offset) == 0) {
                    break;
                }
                if !seen && started.elapsed() > Duration::from_secs(30) {
                    *busy.lock() = None;
                    *job.lock() = Some("No CubeID found (no CUBEPILOT_FIRMWARE_UPDATE_RESP)".into());
                    return;
                }
            }
            *busy.lock() = None;
            *job.lock() = Some("CubeID firmware updated".into());
        });
    }
}

fn ids(l: &Link) -> (u8, u8) {
    let s = l.shared.read();
    (s.cs.sysid, s.cs.compid)
}

fn send_cmd(l: &Link, sys: u8, comp: u8, cmd: MavCmd, p: [f32; 7]) {
    l.send(LinkCommand::Send(Box::new(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: p[0],
        param2: p[1],
        param3: p[2],
        param4: p[3],
        param5: p[4],
        param6: p[5],
        param7: p[6],
        command: cmd,
        target_system: sys,
        target_component: comp,
        confirmation: 0,
    }))));
}

/// ConfigCubeID.crc32_update (nibble table).
pub fn cubeid_crc32(data: &[u8]) -> u32 {
    const T: [u32; 16] = [0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c, 0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c];
    let mut crc = 0xffff_ffffu32;
    for &c in data {
        crc = T[((crc ^ c as u32) & 0x0f) as usize] ^ (crc >> 4);
        crc = T[((crc ^ (c as u32 >> 4)) & 0x0f) as usize] ^ (crc >> 4);
    }
    crc ^ 0xffff_ffff
}

#[cfg(test)]
mod tests {
    #[test]
    fn cubeid_crc_is_crc32() {
        // the nibble table computes the standard CRC-32
        assert_eq!(super::cubeid_crc32(b"123456789"), 0xCBF43926);
    }

    #[test]
    fn esp_chunks() {
        assert_eq!(super::chunk4("Ardupilot", 0), u32::from_le_bytes(*b"Ardu"));
        assert_eq!(super::chunk4("Ardupilot", 8), u32::from_le_bytes([b't', 0, 0, 0]));
        assert_eq!(super::ip_to_u32("192.168.4.1"), Some(u32::from_le_bytes([192, 168, 4, 1])));
    }
}
