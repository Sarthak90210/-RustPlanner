//! JoystickBase.ProcessButtonEvent and the Joy_* button settings.

use mp_core::{firmware::Firmware, link::LinkCommand};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[allow(non_camel_case_types)]
pub enum Function {
    #[default]
    ChangeMode,
    Do_Set_Relay,
    Do_Repeat_Relay,
    Do_Set_Servo,
    Do_Repeat_Servo,
    Arm,
    Disarm,
    Digicam_Control,
    TakeOff,
    Mount_Mode,
    Toggle_Pan_Stab,
    Mount_Control_0,
    Button_axis0,
    Button_axis1,
}

impl Function {
    pub const ALL: [Self; 14] = [
        Self::ChangeMode,
        Self::Do_Set_Relay,
        Self::Do_Repeat_Relay,
        Self::Do_Set_Servo,
        Self::Do_Repeat_Servo,
        Self::Arm,
        Self::Disarm,
        Self::Digicam_Control,
        Self::TakeOff,
        Self::Mount_Mode,
        Self::Toggle_Pan_Stab,
        Self::Mount_Control_0,
        Self::Button_axis0,
        Self::Button_axis1,
    ];

    pub fn fields(self) -> &'static [&'static str] {
        match self {
            Self::Do_Set_Relay => &["Relay number"],
            Self::Do_Repeat_Relay => &["Relay number", "Repeat count", "Cycle time (s)"],
            Self::Do_Set_Servo => &["Servo number", "PWM"],
            Self::Do_Repeat_Servo => &["Servo number", "PWM", "Repeat count", "Cycle time (s)"],
            Self::Mount_Mode => &["Mount mode"],
            Self::Button_axis0 | Self::Button_axis1 => &["Released PWM", "Pressed PWM"],
            _ => &[],
        }
    }
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ButtonCfg {
    pub buttonno: i16,
    pub function: Function,
    pub mode: String,
    pub params: [f32; 4],
}

impl Default for ButtonCfg {
    fn default() -> Self {
        Self {
            buttonno: -1,
            function: Function::ChangeMode,
            mode: String::new(),
            params: [0.0; 4],
        }
    }
}

pub fn defaults() -> [ButtonCfg; 16] {
    std::array::from_fn(|_| ButtonCfg::default())
}

#[derive(Default)]
pub struct Runtime {
    previous: [bool; 16],
    pub custom: [Option<u16>; 2],
    pending_takeoff: Option<(u32, f32, Instant)>,
}

impl Runtime {
    pub fn release_actions(&mut self, configs: &[ButtonCfg; 16]) -> Vec<LinkCommand> {
        let mut commands = Vec::new();
        for (i, c) in configs.iter().enumerate() {
            if self.previous[i] && c.function == Function::Do_Set_Relay {
                commands.push(LinkCommand::CommandLong {
                    command: 181,
                    params: [c.params[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                });
            }
        }
        self.previous = [false; 16];
        self.pending_takeoff = None;
        self.custom = [None; 2];
        commands
    }

    /// Synchronize while disabled so enabling with a held button does not fire it.
    pub fn sync(&mut self, configs: &[ButtonCfg; 16], states: &[bool]) {
        self.pending_takeoff = None;
        for (i, c) in configs.iter().enumerate() {
            self.previous[i] = pressed(c, states);
        }
    }

    /// A mode change is asynchronous. Take off only after telemetry reports Guided.
    pub fn mode_commands(&mut self, mode: u32, now: Instant) -> Vec<LinkCommand> {
        if let Some((guided, alt, started)) = self.pending_takeoff {
            if now.saturating_duration_since(started) > Duration::from_secs(5) {
                self.pending_takeoff = None;
            } else if mode == guided {
                self.pending_takeoff = None;
                return vec![LinkCommand::CommandLong {
                    command: 22,
                    params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt],
                }];
            }
        }
        Vec::new()
    }

    pub fn poll(
        &mut self,
        configs: &[ButtonCfg; 16],
        states: &[bool],
        firmware: Firmware,
        pan_stab: Option<f32>,
    ) -> Vec<LinkCommand> {
        let mut commands = Vec::new();
        self.custom = [None; 2];
        for (i, c) in configs.iter().enumerate() {
            let down = pressed(c, states);
            let changed = down != self.previous[i];
            self.previous[i] = down;
            // Custom axes must also initialize their released PWM on startup.
            if matches!(c.function, Function::Button_axis0 | Function::Button_axis1)
                && c.buttonno >= 0
            {
                let k = usize::from(c.function == Function::Button_axis1);
                self.custom[k] = Some(c.params[usize::from(down)].clamp(1000.0, 2000.0) as u16);
            }
            if !changed || c.buttonno < 0 {
                continue;
            }
            if !down && c.function != Function::Do_Set_Relay {
                continue;
            }
            let mut p = [0.0; 7];
            let n = c.function.fields().len();
            p[..n].copy_from_slice(&c.params[..n]);
            let command = match c.function {
                Function::ChangeMode => {
                    self.pending_takeoff = None;
                    if let Some((mode, _)) = firmware
                        .modes()
                        .into_iter()
                        .find(|(_, n)| n.eq_ignore_ascii_case(&c.mode))
                    {
                        commands.push(LinkCommand::SetMode(mode));
                    }
                    continue;
                }
                Function::Arm | Function::Disarm => {
                    if c.function == Function::Disarm {
                        self.pending_takeoff = None;
                    }
                    commands.push(LinkCommand::Arm {
                        arm: c.function == Function::Arm,
                        force: false,
                    });
                    continue;
                }
                Function::Mount_Mode => {
                    commands.push(LinkCommand::SetParam {
                        name: "MNT_MODE".into(),
                        value: c.params[0],
                    });
                    continue;
                }
                Function::Toggle_Pan_Stab => {
                    if let Some(value) = pan_stab {
                        commands.push(LinkCommand::SetParam {
                            name: "MNT_STAB_PAN".into(),
                            value: if value > 0.0 { 0.0 } else { 1.0 },
                        });
                    }
                    continue;
                }
                Function::TakeOff => {
                    if let Some((mode, _)) = firmware
                        .modes()
                        .into_iter()
                        .find(|(_, n)| n.eq_ignore_ascii_case("Guided"))
                    {
                        commands.push(LinkCommand::SetMode(mode));
                        let altitude = if firmware == Firmware::ArduCopter2 {
                            2.0
                        } else {
                            20.0
                        };
                        self.pending_takeoff = Some((mode, altitude, Instant::now()));
                    } else {
                        continue;
                    }
                    continue;
                }
                Function::Do_Set_Relay => {
                    p[1] = if down { 1.0 } else { 0.0 };
                    181
                }
                Function::Do_Repeat_Relay => 182,
                Function::Do_Set_Servo => 183,
                Function::Do_Repeat_Servo => 184,
                Function::Digicam_Control => {
                    p = [0.0; 7];
                    p[4] = 1.0;
                    203
                }
                Function::Mount_Control_0 => {
                    p = [0.0; 7];
                    p[6] = 2.0;
                    205
                }
                Function::Button_axis0 | Function::Button_axis1 => continue,
            };
            commands.push(LinkCommand::CommandLong { command, params: p });
        }
        commands
    }
}

fn pressed(config: &ButtonCfg, states: &[bool]) -> bool {
    usize::try_from(config.buttonno)
        .ok()
        .and_then(|n| states.get(n))
        .copied()
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn button_edges_and_relay_release_match_mission_planner() {
        let mut cfg = defaults();
        cfg[0] = ButtonCfg {
            buttonno: 0,
            function: Function::Arm,
            ..Default::default()
        };
        cfg[1] = ButtonCfg {
            buttonno: 1,
            function: Function::Do_Set_Relay,
            params: [3.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        let mut r = Runtime::default();
        let commands = r.poll(&cfg, &[true, true], Firmware::ArduCopter2, None);
        assert!(matches!(
            commands[0],
            LinkCommand::Arm {
                arm: true,
                force: false
            }
        ));
        assert!(
            matches!(commands[1], LinkCommand::CommandLong { command: 181, params } if params[0] == 3.0 && params[1] == 1.0)
        );
        assert!(r
            .poll(&cfg, &[true, true], Firmware::ArduCopter2, None)
            .is_empty());
        let commands = r.poll(&cfg, &[false, false], Firmware::ArduCopter2, None);
        assert_eq!(commands.len(), 1);
        assert!(
            matches!(commands[0], LinkCommand::CommandLong { command: 181, params } if params[1] == 0.0)
        );
    }

    #[test]
    fn disable_releases_relay_and_cancels_pending_takeoff_once() {
        let mut cfg = defaults();
        cfg[0] = ButtonCfg {
            buttonno: 0,
            function: Function::Do_Set_Relay,
            params: [3.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        cfg[1] = ButtonCfg {
            buttonno: 1,
            function: Function::TakeOff,
            ..Default::default()
        };
        let mut r = Runtime::default();
        r.poll(&cfg, &[true, true], Firmware::ArduCopter2, None);
        let commands = r.release_actions(&cfg);
        assert_eq!(commands.len(), 1);
        assert!(
            matches!(commands[0], LinkCommand::CommandLong { command: 181, params } if params[0] == 3.0 && params[1] == 0.0)
        );
        assert!(r.mode_commands(4, Instant::now()).is_empty());
        assert!(r.release_actions(&cfg).is_empty());
    }

    #[test]
    fn held_buttons_do_not_fire_on_enable_and_custom_axes_use_both_edges() {
        let mut cfg = defaults();
        cfg[0] = ButtonCfg {
            buttonno: 0,
            function: Function::Disarm,
            ..Default::default()
        };
        cfg[1] = ButtonCfg {
            buttonno: 1,
            function: Function::Button_axis0,
            params: [1100.0, 1900.0, 0.0, 0.0],
            ..Default::default()
        };
        let mut r = Runtime::default();
        r.sync(&cfg, &[true, false]);
        assert!(r
            .poll(&cfg, &[true, false], Firmware::ArduCopter2, None)
            .is_empty());
        assert_eq!(r.custom[0], Some(1100));
        r.poll(&cfg, &[false, true], Firmware::ArduCopter2, None);
        assert_eq!(r.custom[0], Some(1900));
        let commands = r.poll(&cfg, &[true, false], Firmware::ArduCopter2, None);
        assert!(matches!(
            commands[0],
            LinkCommand::Arm {
                arm: false,
                force: false
            }
        ));
        assert_eq!(r.custom[0], Some(1100));
    }

    #[test]
    fn servo_camera_takeoff_mount_and_mode_parameters_match_reference() {
        for (function, command, expected) in [
            (
                Function::Do_Set_Servo,
                183,
                [6.0, 1700.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            (
                Function::Do_Repeat_Servo,
                184,
                [6.0, 1700.0, 3.0, 0.5, 0.0, 0.0, 0.0],
            ),
            (
                Function::Do_Repeat_Relay,
                182,
                [6.0, 1700.0, 3.0, 0.0, 0.0, 0.0, 0.0],
            ),
            (
                Function::Digicam_Control,
                203,
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            ),
            (
                Function::Mount_Control_0,
                205,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
            ),
        ] {
            let mut cfg = defaults();
            cfg[0] = ButtonCfg {
                buttonno: 0,
                function,
                params: [6.0, 1700.0, 3.0, 0.5],
                ..Default::default()
            };
            let messages = Runtime::default().poll(&cfg, &[true], Firmware::ArduCopter2, None);
            assert!(
                matches!(&messages[0], LinkCommand::CommandLong { command: c, params } if *c == command && *params == expected),
                "{function:?}"
            );
        }
        let mut cfg = defaults();
        cfg[0] = ButtonCfg {
            buttonno: 0,
            function: Function::TakeOff,
            ..Default::default()
        };
        let mut runtime = Runtime::default();
        let messages = runtime.poll(&cfg, &[true], Firmware::ArduCopter2, None);
        assert!(matches!(messages[0], LinkCommand::SetMode(4)));
        assert_eq!(messages.len(), 1);
        assert!(runtime.mode_commands(0, Instant::now()).is_empty());
        let messages = runtime.mode_commands(4, Instant::now());
        assert!(
            matches!(messages[0], LinkCommand::CommandLong { command: 22, params } if params[6] == 2.0)
        );
        assert!(runtime.mode_commands(4, Instant::now()).is_empty());
        cfg[0].function = Function::Mount_Mode;
        cfg[0].params[0] = 3.0;
        assert!(
            matches!(&Runtime::default().poll(&cfg, &[true], Firmware::ArduCopter2, None)[0], LinkCommand::SetParam { name, value } if name == "MNT_MODE" && *value == 3.0)
        );
        cfg[0].function = Function::Toggle_Pan_Stab;
        assert!(
            matches!(&Runtime::default().poll(&cfg, &[true], Firmware::ArduCopter2, Some(1.0))[0], LinkCommand::SetParam { name, value } if name == "MNT_STAB_PAN" && *value == 0.0)
        );
    }
}
