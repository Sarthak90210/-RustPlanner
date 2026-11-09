//! Port of `GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs`
//!
//! The Accel Calibration setup page. `MainV2.comPort.doCommand` blocks until the vehicle's
//! COMMAND_ACK in C#; here the command is sent and `on_ack` finishes the handler when the ACK
//! arrives (`pending` remembers which button asked). The STATUSTEXT / COMMAND_LONG
//! subscriptions are read from the link's message list and its last ACCELCAL_VEHICLE_POS
//! request; like the C# subscriptions they stay alive after Deactivate until the
//! calibration reports success or failure.

#![allow(non_snake_case)]

use mp_core::extlibs::mavlink::ACCELCAL_VEHICLE_POS;
use mp_core::link::{Link, LinkCommand, LinkEvent};

use crate::app::Toasts;

/// `MAV_CMD.PREFLIGHT_CALIBRATION`
const MAV_CMD_PREFLIGHT_CALIBRATION: u16 = 241;
/// `MAV_CMD.ACCELCAL_VEHICLE_POS`
const MAV_CMD_ACCELCAL_VEHICLE_POS: u16 = 42429;

/// `Strings.CommandFailed` / `Strings.ERROR`
const COMMAND_FAILED: &str = "The Command failed to execute\n";
const ERROR: &str = "Error";

/// Which `doCommand` is waiting for its ACK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    /// BUT_calib_accell_Click: PREFLIGHT_CALIBRATION p5 = 1
    Accel,
    /// BUT_level_Click: p5 = 2
    Level,
    /// BUT_simpleAccelCal_Click: p5 = 4
    SimpleAccel,
}

/// `public partial class ConfigAccelerometerCalibration : MyUserControl, IActivate, IDeactivate`
#[derive(Debug, Clone)]
pub struct ConfigAccelerometerCalibration {
    count: u8,
    _incalibrate: bool,
    pos: ACCELCAL_VEHICLE_POS,
    /// `sub1` / `sub2` are live (STATUSTEXT and COMMAND_LONG)
    subscribed: bool,
    pending: Option<Pending>,
    /// messages already handed to receivedPacket
    seen: usize,
    /// the last COMMAND_LONG ACCELCAL_VEHICLE_POS param1 handled
    last_pos: Option<u32>,
    /// the subscriptions started without the link at hand: they begin at the next poll
    resync: bool,
    pub BUT_calib_accell_Text: String,
    pub BUT_calib_accell_Enabled: bool,
    pub BUT_level_Text: String,
    pub BUT_simpleAccelCal_Text: String,
    pub lbl_Accel_user_Text: String,
}

impl Default for ConfigAccelerometerCalibration {
    fn default() -> Self {
        Self::new()
    }
}

impl ConfigAccelerometerCalibration {
    /// `new ConfigAccelerometerCalibration()`: the designer texts.
    pub fn new() -> Self {
        ConfigAccelerometerCalibration {
            count: 0,
            _incalibrate: false,
            pos: ACCELCAL_VEHICLE_POS(0),
            subscribed: false,
            pending: None,
            seen: 0,
            last_pos: None,
            resync: false,
            BUT_calib_accell_Text: "Calibrate Accel".into(),
            BUT_calib_accell_Enabled: true,
            BUT_level_Text: "Calibrate Level".into(),
            BUT_simpleAccelCal_Text: "Simple Accel Cal".into(),
            lbl_Accel_user_Text: String::new(),
        }
    }

    /// `Activate()`: the button is enabled again and calibration is off (its text is kept).
    pub fn Activate(&mut self) {
        self.BUT_calib_accell_Enabled = true;
        self._incalibrate = false;
    }

    /// `Deactivate()`: `giveComport = false`; calibration off. The subscriptions stay.
    pub fn Deactivate(&mut self) {
        self._incalibrate = false;
    }

    /// `BUT_calib_accell_Click`
    pub fn BUT_calib_accell_Click(&mut self, link: &Link) {
        if self._incalibrate {
            self.count = self.count.wrapping_add(1);
            // sendPacket(COMMAND_LONG { param1 = (float)pos, command = ACCELCAL_VEHICLE_POS })
            link.send(LinkCommand::CommandNoAck { command: MAV_CMD_ACCELCAL_VEHICLE_POS, params: [self.pos.0 as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
            return;
        }
        if self.pending.is_some() {
            // doCommand is still blocking the UI thread
            return;
        }
        self.count = 0;
        log::info!("Sending accel command (mavlink 1.0)");
        self.pending = Some(Pending::Accel);
        link.send(LinkCommand::CommandLong { command: MAV_CMD_PREFLIGHT_CALIBRATION, params: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0] });
    }

    /// `BUT_level_Click`
    pub fn BUT_level_Click(&mut self, link: &Link) {
        if self.pending.is_some() {
            return;
        }
        log::info!("Sending level command (mavlink 1.0)");
        self.pending = Some(Pending::Level);
        link.send(LinkCommand::CommandLong { command: MAV_CMD_PREFLIGHT_CALIBRATION, params: [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0] });
    }

    /// `BUT_simpleAccelCal_Click`
    pub fn BUT_simpleAccelCal_Click(&mut self, link: &Link) {
        if self.pending.is_some() {
            return;
        }
        log::info!("Sending simple accelerometer calibration command (mavlink 1.0)");
        self.pending = Some(Pending::SimpleAccel);
        link.send(LinkCommand::CommandLong { command: MAV_CMD_PREFLIGHT_CALIBRATION, params: [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0] });
    }

    /// The rest of a click handler once `doCommand` returns: true when this page used the
    /// ACK.
    pub fn on_ack(&mut self, event: &LinkEvent, link: Option<&Link>, toasts: &mut Toasts) -> bool {
        let LinkEvent::CommandAck { command: MAV_CMD_PREFLIGHT_CALIBRATION, ok, .. } = event else { return false };
        let Some(kind) = self.pending.take() else { return false };
        if !ok {
            toasts.message(COMMAND_FAILED, ERROR);
            return true;
        }
        match kind {
            Pending::Accel => {
                self._incalibrate = true;
                // SubscribeToPacketType(STATUSTEXT / COMMAND_LONG): from the next message on
                self.subscribed = true;
                match link {
                    Some(l) => {
                        let mut s = l.shared.write();
                        self.seen = s.messages.len();
                        s.accelcal_pos = None;
                    }
                    None => self.resync = true,
                }
                self.last_pos = None;
                self.BUT_calib_accell_Text = "Click when Done".into();
            }
            Pending::Level => self.BUT_level_Text = "Completed".into(),
            Pending::SimpleAccel => self.BUT_simpleAccelCal_Text = "Completed".into(),
        }
        true
    }

    /// Delivers what the subscriptions would have received since the last call.
    pub fn poll(&mut self, link: &Link) {
        if self.resync {
            self.resync = false;
            let mut s = link.shared.write();
            self.seen = s.messages.len();
            s.accelcal_pos = None;
        }
        if !self.subscribed {
            self.seen = link.shared.read().messages.len();
            return;
        }
        let (pos, new): (Option<u32>, Vec<String>) = {
            let s = link.shared.read();
            (s.accelcal_pos, s.messages.iter().skip(self.seen).map(|m| m.text.clone()).collect())
        };
        self.seen += new.len();
        for t in new {
            if !self.subscribed {
                break;
            }
            self.receivedPacket_statustext(&t);
        }
        if self.subscribed && pos.is_some() && pos != self.last_pos {
            self.last_pos = pos;
            self.receivedPacket_command_long(pos.unwrap());
        }
    }

    /// `receivedPacket` for a STATUSTEXT
    pub fn receivedPacket_statustext(&mut self, message: &str) {
        self.UpdateUserMessage(message);
        let lower = message.to_lowercase();
        if lower.contains("calibration successful") || lower.contains("calibration failed") {
            self.BUT_calib_accell_Text = "Done".into();
            self.BUT_calib_accell_Enabled = false;
            self._incalibrate = false;
            // UnSubscribeToPacketType(sub1) / (sub2)
            self.subscribed = false;
        }
    }

    /// `receivedPacket` for a COMMAND_LONG ACCELCAL_VEHICLE_POS: `(ACCELCAL_VEHICLE_POS)param1`
    pub fn receivedPacket_command_long(&mut self, param1: u32) {
        self.pos = ACCELCAL_VEHICLE_POS(param1 as i32);
        let text = format!("Please place vehicle {}", self.pos);
        self.UpdateUserMessage(&text);
    }

    /// `UpdateUserMessage(message)`: only placement and calibration texts are shown.
    pub fn UpdateUserMessage(&mut self, message: &str) {
        let lower = message.to_lowercase();
        if lower.contains("place vehicle") || lower.contains("calibration") {
            self.lbl_Accel_user_Text = message.to_string();
        }
    }

    pub fn in_calibrate(&self) -> bool {
        self._incalibrate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn acks_statustexts_and_positions() {
        let mut c = ConfigAccelerometerCalibration::new();
        let mut toasts = Toasts::default();
        let ack = |ok| LinkEvent::CommandAck { command: 241, result: String::new(), ok };
        c.pending = Some(Pending::Accel);
        assert!(c.on_ack(&ack(false), None, &mut toasts));
        assert_eq!(toasts.0, ["The Command failed to execute\n"]);
        assert!(!c.in_calibrate(), "a refused command does not start calibrating");
        c.pending = Some(Pending::Accel);
        c.on_ack(&ack(true), None, &mut toasts);
        assert!(c.in_calibrate() && c.BUT_calib_accell_Text == "Click when Done");
        c.receivedPacket_statustext("PreArm: something");
        assert_eq!(c.lbl_Accel_user_Text, "", "only place-vehicle / calibration texts are shown");
        c.receivedPacket_command_long(4);
        assert_eq!(c.lbl_Accel_user_Text, "Please place vehicle NOSEDOWN");
        c.receivedPacket_command_long(99);
        assert_eq!(c.lbl_Accel_user_Text, "Please place vehicle 99");
        c.receivedPacket_statustext("Calibration FAILED");
        assert!(!c.in_calibrate() && !c.BUT_calib_accell_Enabled && c.BUT_calib_accell_Text == "Done");
        assert!(!c.on_ack(&ack(true), None, &mut toasts), "nothing awaited");
        // Activate re-enables the button but keeps its text
        c.Activate();
        assert!(c.BUT_calib_accell_Enabled && c.BUT_calib_accell_Text == "Done");
    }
}
