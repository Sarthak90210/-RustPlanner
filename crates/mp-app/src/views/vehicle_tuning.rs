//! Vehicle-specific CONFIG pages from ConfigArduplane, ConfigArdurover and
//! ConfigAntennaTracker. Plane edits are staged; Rover/Tracker controls write directly.
use crate::{
    app::{AppCtx, Toasts},
    forms::{self, FormState},
};
use mp_core::{
    firmware::Firmware,
    link::{LinkCommand, LinkEvent, Param},
    mavlink::dialects::ardupilotmega::*,
};
use std::collections::{BTreeMap, VecDeque};

pub fn form_for(firmware: Firmware, extended: bool) -> Option<&'static str> {
    match (firmware, extended) {
        (Firmware::ArduPlane, false) => Some("plane_tuning"),
        (Firmware::ArduRover, false) => Some("rover_tuning"),
        (Firmware::ArduTracker, true) => Some("tracker_tuning"),
        _ => None,
    }
}

#[derive(Default)]
pub struct VehicleTuning {
    form: Option<&'static str>,
    target: Option<(u8, u8)>,
    st: FormState,
    queue: VecDeque<(String, f32)>,
    waiting: Option<(String, f32)>,
    confirm: Option<(String, f32)>,
}

impl VehicleTuning {
    pub fn leave(&mut self) {
        self.form = None;
        self.queue.clear();
        self.confirm = None;
    }

    fn prepare(&mut self, form: &'static str, target: (u8, u8)) {
        if self.form == Some(form) && self.target == Some(target) {
            return;
        }
        self.form = Some(form);
        self.target = Some(target);
        self.st = FormState::default();
        self.st.parameter_metadata = true;
        self.queue.clear();
        self.waiting = None;
        self.confirm = None;
        if form == "plane_tuning" {
            self.st.staged = Some(BTreeMap::new());
        }
        for name in ["myTrackBar1", "myTrackBar2"] {
            self.st.ranges.insert(name.into(), (1000.0, 2000.0));
            self.st.set(name, "1500");
        }
    }

    fn pending(&self) -> BTreeMap<String, f32> {
        self.st.staged.clone().unwrap_or_default()
    }

    fn start_write(&mut self) {
        if self.waiting.is_none() {
            self.queue = self.pending().into_iter().collect();
        }
    }

    fn send(&mut self, name: String, value: f32, app: &mut AppCtx) {
        if let Some(link) = app.link.filter(|_| app.connected() && !app.readonly) {
            link.send(LinkCommand::SetParam {
                name: name.clone(),
                value,
            });
            self.waiting = Some((name, value));
        }
    }

    fn advance(&mut self, params: &BTreeMap<String, Param>, app: &mut AppCtx) {
        if self.waiting.is_some() || self.confirm.is_some() {
            return;
        }
        if let Some((name, value)) = self.queue.pop_front() {
            if params.get(&name).is_some_and(|p| value > p.value * 2.0) {
                self.confirm = Some((name, value));
            } else {
                self.send(name, value, app);
            }
        }
    }

    pub fn on_event(&mut self, event: &LinkEvent, toasts: &mut Toasts) {
        self.st.on_event(event, toasts);
        if let LinkEvent::ParamSet { name, ok } = event {
            if let Some((pending, value)) = &self.waiting {
                if pending == name {
                    if *ok {
                        if let Some(edits) = &mut self.st.staged {
                            if edits.get(name) == Some(value) {
                                edits.remove(name);
                            }
                        }
                    } else {
                        self.queue.clear();
                        toasts.error(format!("Error setting parameter {name}"));
                    }
                    self.waiting = None;
                }
            }
        }
    }

    fn confirm_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        let Some((name, value)) = self.confirm.clone() else {
            return;
        };
        let mut answer = None;
        egui::Modal::new(egui::Id::new("vehicle_tuning_large_value")).show(ctx, |ui| {
            ui.heading(crate::i18n::t("Large Value"));
            ui.label(format!(
                "{name} has more than doubled the last input. Are you sure?"
            ));
            ui.horizontal(|ui| {
                if ui.button(crate::i18n::t("Yes")).clicked() {
                    answer = Some(true);
                }
                if ui.button(crate::i18n::t("No")).clicked() {
                    answer = Some(false);
                }
            });
        });
        if let Some(yes) = answer {
            self.confirm = None;
            if yes {
                self.send(name, value, app);
            } else {
                // ConfigArduplane restores the declined control and stops this write pass.
                if self.form == Some("plane_tuning") {
                    if let Some(edits) = &mut self.st.staged {
                        edits.remove(&name);
                    }
                }
                self.queue.clear();
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, form: &'static str) {
        let Some(link) = app.link else { return };
        let (params, cs) = {
            let s = link.shared.read();
            (s.params.clone(), s.cs.clone())
        };
        self.prepare(form, (cs.sysid, cs.compid));
        if form == "rover_tuning" {
            self.st.visible.insert(
                "groupBox1".into(),
                params.contains_key("SONAR_TRIGGER_CM") || params.contains_key("RNGFND_TRIGGR_CM"),
            );
        }
        self.st.set("lbl_yawpwm", cs.ch_out[0].to_string());
        self.st.set("lbl_pitchpwm", cs.ch_out[1].to_string());
        let busy = self.waiting.is_some() || !self.queue.is_empty() || self.confirm.is_some();
        let clicked = ui
            .add_enabled_ui(!busy, |ui| forms::show(ui, app, form, &mut self.st))
            .inner;
        let keyboard_write = ui.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::S));
        if !app.readonly {
            if clicked.iter().any(|b| b == "BUT_writePIDS") || (keyboard_write && !busy) {
                self.start_write();
            }
            for button in clicked {
                match button.as_str() {
                    "BUT_rerequestparams" => {
                        if let Some(p) = &mut self.st.staged {
                            p.clear();
                        }
                        link.send(LinkCommand::RequestParams);
                    }
                    "BUT_refreshpart" => {
                        if let Some(p) = &mut self.st.staged {
                            p.clear();
                        }
                        for control in &forms::form(form).controls {
                            let name = control
                                .aliases
                                .iter()
                                .find_map(|n| forms::resolve(&params, n))
                                .or_else(|| {
                                    control
                                        .param
                                        .as_deref()
                                        .and_then(|n| forms::resolve(&params, n))
                                });
                            if let Some(name) = name {
                                link.send(LinkCommand::Send(Box::new(
                                    MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                                        target_system: cs.sysid,
                                        target_component: cs.compid,
                                        param_id: name.as_str().into(),
                                        param_index: -1,
                                    }),
                                )));
                            }
                        }
                    }
                    "BUT_test_yaw" | "BUT_test_pitch" => {
                        let yaw = button == "BUT_test_yaw";
                        let slider = self
                            .st
                            .get(if yaw { "myTrackBar1" } else { "myTrackBar2" })
                            .parse()
                            .unwrap_or(1500.0);
                        let pwm = tracker_servo_pwm(&params, yaw, slider);
                        link.send(LinkCommand::CommandLong {
                            command: 183,
                            params: [if yaw { 1.0 } else { 2.0 }, pwm, 0.0, 0.0, 0.0, 0.0, 0.0],
                        });
                    }
                    _ => {}
                }
            }
            self.advance(&params, app);
        }
        self.confirm_ui(ui.ctx(), app);
        if busy || form == "tracker_tuning" {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(100));
        }
    }
}

fn tracker_servo_pwm(params: &BTreeMap<String, Param>, yaw: bool, slider: f32) -> f32 {
    let channel = if yaw { 1 } else { 2 };
    let value = |suffix: &str, fallback| {
        params
            .get(&format!("RC{channel}_{suffix}"))
            .map(|p| p.value)
            .unwrap_or(fallback)
    };
    let reverse = value("REV", 1.0) != 1.0;
    let fraction = (slider - 1000.0) / 1000.0;
    let fraction = if reverse { 1.0 - fraction } else { fraction };
    value("MIN", 1000.0) + fraction * (value("MAX", 2000.0) - value("MIN", 1000.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn numeric_edit_stays_staged_until_ctrl_s_and_clears_on_simulator_ack() {
        numeric_roundtrip(
            mp_core::sim::SimKind::Plane,
            Firmware::ArduPlane,
            "plane_tuning",
            "TRIM_THROTTLE",
            true,
        );
    }

    #[test]
    fn rover_numeric_edit_writes_directly_without_ctrl_s() {
        numeric_roundtrip(
            mp_core::sim::SimKind::Rover,
            Firmware::ArduRover,
            "rover_tuning",
            "CRUISE_THROTTLE",
            false,
        );
    }

    fn numeric_roundtrip(
        kind: mp_core::sim::SimKind,
        firmware: Firmware,
        form: &'static str,
        parameter: &str,
        staged: bool,
    ) {
        use mp_core::{comms::ConnectionSpec, link::Link};
        let port = std::net::UdpSocket::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let link = Link::connect_with_params(ConnectionSpec::UdpListen { port }, None, false);
        let _sim = mp_core::sim::spawn_with(mp_core::sim::SimOptions {
            gcs_port: port,
            sysid: 1,
            kind,
            offset: (0.0, 0.0),
        })
        .unwrap();
        loop {
            if matches!(
                link.events.recv_timeout(Duration::from_secs(5)).unwrap(),
                LinkEvent::Connected
            ) {
                break;
            }
        }
        let deadline = Instant::now() + Duration::from_secs(3);
        while link.shared.read().cs.firmware != firmware && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(link.shared.read().cs.firmware, firmware);
        link.shared.write().params.insert(
            parameter.into(),
            Param {
                value: 50.0,
                ptype: 9,
                index: 0,
            },
        );
        let (tx, rx) = crossbeam_channel::unbounded();
        link.send(LinkCommand::Subscribe { msgid: 22, tx });
        let ctx = egui::Context::default();
        let assets = crate::assets::Assets::load(&ctx);
        let mut tiles = crate::map::TileCache::offline();
        let mut toasts = Toasts::default();
        let mut settings = crate::app::Settings::default();
        let mut joystick = crate::joystick::Joystick::default();
        let mut page = VehicleTuning::default();
        let mut frame = |events| {
            let mut actions = Vec::new();
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::Vec2::new(1100.0, 750.0),
                    )),
                    events,
                    ..Default::default()
                },
                |ui| {
                    let mut app = AppCtx {
                        link: Some(&link),
                        extra_links: &[],
                        tiles: &mut tiles,
                        toasts: &mut toasts,
                        settings: &mut settings,
                        actions: &mut actions,
                        sim_running: true,
                        assets: &assets,
                        readonly: false,
                        joystick: &mut joystick,
                    };
                    page.ui(ui, &mut app, form);
                },
            );
            out.textures_delta.clear();
        };
        frame(Vec::new());
        let (at, size) = forms::ctl_rect(form, parameter).unwrap();
        let pos = egui::Pos2::ZERO + at + egui::Vec2::new(size.x - 9.0, size.y * 0.25);
        for pressed in [true, false] {
            frame(vec![
                egui::Event::PointerMoved(pos),
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed,
                    modifiers: egui::Modifiers::NONE,
                },
            ]);
        }
        frame(Vec::new());
        if staged {
            assert!(
                rx.recv_timeout(Duration::from_millis(100)).is_err(),
                "Editing wrote before Write Params/Ctrl+S"
            );
            frame(vec![egui::Event::Key {
                key: egui::Key::S,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::CTRL,
            }]);
        } else {
            assert!(
                rx.recv_timeout(Duration::from_millis(100)).is_err(),
                "Direct numeric write skipped MavlinkNumericUpDown's 300 ms timer"
            );
            std::thread::sleep(forms::WRITE_DELAY);
            frame(Vec::new());
        }
        drop(frame);
        let desired = page.pending().get(parameter).copied().unwrap_or(51.0);
        assert!(desired > 50.0 && desired <= 51.0);
        let end = Instant::now() + Duration::from_secs(5);
        loop {
            let event = link
                .events
                .recv_timeout(end.saturating_duration_since(Instant::now()))
                .unwrap();
            let done =
                matches!(&event, LinkEvent::ParamSet { name, ok: true } if name == parameter);
            page.on_event(&event, &mut toasts);
            if done {
                break;
            }
        }
        assert!(page.pending().is_empty());
        assert_eq!(link.shared.read().params[parameter].value, desired);
    }
    #[test]
    fn failed_direct_writes_show_the_original_messages() {
        let mut st = FormState::default();
        let mut toasts = Toasts::default();
        st.awaiting.insert("CRUISE_SPEED".into(), "Set CRUISE_SPEED Failed".into());
        st.awaiting.insert("MOT_PWM_TYPE".into(), "Set MOT_PWM_TYPE Failed!".into());
        st.on_event(&LinkEvent::ParamSet { name: "CRUISE_SPEED".into(), ok: true }, &mut toasts);
        st.on_event(&LinkEvent::ParamSet { name: "MOT_PWM_TYPE".into(), ok: false }, &mut toasts);
        st.on_event(&LinkEvent::ParamSet { name: "OTHER".into(), ok: false }, &mut toasts);
        assert_eq!(toasts.0, vec!["Set MOT_PWM_TYPE Failed!".to_string()]);
        assert!(st.awaiting.is_empty());
    }

    #[test]
    fn firmware_routes_use_distinct_original_forms_and_aliases() {
        assert_eq!(form_for(Firmware::ArduPlane, false), Some("plane_tuning"));
        assert_eq!(form_for(Firmware::ArduRover, false), Some("rover_tuning"));
        assert_eq!(
            form_for(Firmware::ArduTracker, true),
            Some("tracker_tuning")
        );
        assert_eq!(form_for(Firmware::ArduCopter2, false), None);
        let control = forms::form("plane_tuning")
            .controls
            .iter()
            .find(|c| c.name == "ARSPD_FBW_MAX")
            .unwrap();
        assert_eq!(control.aliases, ["AIRSPEED_MAX", "ARSPD_FBW_MAX"]);
        let mut page = VehicleTuning::default();
        page.prepare("plane_tuning", (1, 1));
        page.st
            .staged
            .as_mut()
            .unwrap()
            .insert("RLL_RATE_P".into(), 0.8);
        page.start_write();
        assert_eq!(page.queue.front(), Some(&("RLL_RATE_P".into(), 0.8)));
        page.prepare("tracker_tuning", (1, 1));
        assert!(page.st.staged.is_none() && page.queue.is_empty());
        page.prepare("rover_tuning", (1, 1));
        assert!(page.st.staged.is_none() && page.st.parameter_metadata);
    }

    #[test]
    fn tracker_test_pwm_matches_both_legacy_reverse_conventions() {
        let mut p = BTreeMap::new();
        for ch in [1, 2] {
            for (suffix, value) in [("MIN", 1100.0), ("MAX", 1900.0), ("REV", 1.0)] {
                p.insert(
                    format!("RC{ch}_{suffix}"),
                    Param {
                        value,
                        ptype: 9,
                        index: 0,
                    },
                );
            }
        }
        for yaw in [true, false] {
            assert_eq!(tracker_servo_pwm(&p, yaw, 1000.0), 1100.0);
            assert_eq!(tracker_servo_pwm(&p, yaw, 2000.0), 1900.0);
        }
        p.get_mut("RC1_REV").unwrap().value = -1.0;
        p.get_mut("RC2_REV").unwrap().value = -1.0;
        for yaw in [true, false] {
            assert_eq!(tracker_servo_pwm(&p, yaw, 1000.0), 1900.0);
        }
    }
}
