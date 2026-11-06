//! ConfigMount output routing and dynamic controls on the original designer layout.
use crate::{
    app::AppCtx,
    forms::{self, FormState},
};
use mp_core::link::{LinkCommand, Param};
use std::collections::BTreeMap;

const AXES: [(&str, &str, u8); 3] = [
    ("mavlinkComboBoxTilt", "T", 7),
    ("mavlinkComboBoxRoll", "R", 8),
    ("mavlinkComboBoxPan", "P", 6),
];

#[derive(Default)]
pub struct MountPage {
    st: FormState,
    target: Option<(u8, u8, bool)>,
}

fn outputs(modern: bool) -> Vec<String> {
    std::iter::once("Disable".to_string())
        .chain(
            (if modern { 1 } else { 5 }..=14)
                .map(|n| format!("{}{n}", if modern { "SERVO" } else { "RC" })),
        )
        .collect()
}

impl MountPage {
    pub fn leave(&mut self) {
        self.target = None;
    }

    fn prepare(&mut self, params: &BTreeMap<String, Param>, target: (u8, u8, bool)) {
        if self.target != Some(target) {
            self.target = Some(target);
            self.st = FormState::default();
            self.st.set("label34", "Duration\n(1/10th sec)");
            let channels = outputs(target.2);
            for (ctl, _, function) in AXES {
                self.st.items.insert(ctl.into(), channels.clone());
                let selected = channels
                    .iter()
                    .filter(|c| c.as_str() != "Disable")
                    .filter(|c| {
                        params
                            .get(&format!("{c}_FUNCTION"))
                            .is_some_and(|p| p.value == function as f32)
                    })
                    .last()
                    .cloned()
                    .unwrap_or_else(|| "Disable".into());
                self.st.set(ctl, selected);
            }
            let mut shutter = channels.clone();
            shutter.extend(["Relay".into(), "Transistor".into()]);
            self.st.items.insert("CMB_shuttertype".into(), shutter);
            let selected = channels
                .iter()
                .filter(|c| {
                    params
                        .get(&format!("{c}_FUNCTION"))
                        .is_some_and(|p| p.value == 10.0)
                })
                .last()
                .cloned()
                .unwrap_or_else(|| {
                    match params.get("CAM_TRIGG_TYPE").map(|p| p.value as i32) {
                        Some(1) => "Relay",
                        Some(4) => "Transistor",
                        _ => "Disable",
                    }
                    .into()
                });
            self.st.set("CMB_shuttertype", selected);
            for axis in ["tilt", "roll", "pan"] {
                self.st.options.insert(
                    format!("CMB_inputch_{axis}"),
                    std::iter::once((0, "Disable".into()))
                        .chain((5..=16).map(|n| (n, format!("RC{n}"))))
                        .collect(),
                );
            }
        }
        self.bind(params);
    }

    fn bind(&mut self, params: &BTreeMap<String, Param>) {
        for (ctl, short, _) in AXES {
            let output = self.st.get(ctl).to_string();
            for (control, suffix) in [
                (format!("mavlinkNumericUpDown{short}SM"), "MIN"),
                (format!("mavlinkNumericUpDown{short}SMX"), "MAX"),
            ] {
                self.st
                    .bindings
                    .insert(control, format!("{output}_{suffix}"));
            }
            let check = format!("mavlinkCheckBox{short}R");
            let legacy = format!("{output}_REV");
            let modern = format!("{output}_REVERSED");
            let old = params.contains_key(&legacy);
            self.st
                .bindings
                .insert(check.clone(), if old { legacy } else { modern });
            self.st
                .check_values
                .insert(check, if old { (-1.0, 1.0) } else { (1.0, 0.0) });
        }
        let shutter = self.st.get("CMB_shuttertype").to_string();
        for (ctl, suffix) in [
            ("mavlinkNumericUpDownShutM", "MIN"),
            ("mavlinkNumericUpDownShutMX", "MAX"),
        ] {
            self.st
                .bindings
                .insert(ctl.into(), format!("{shutter}_{suffix}"));
        }
    }

    /// ConfigMount.SelectedIndexChanged: clear previous assignments, set MODE=3,
    /// then update shutter, pitch, roll and yaw in the reference order.
    fn routing_writes(
        &self,
        params: &BTreeMap<String, Param>,
        selection_changed: bool,
    ) -> Vec<(String, f32)> {
        let mut writes = Vec::new();
        let channels = outputs(params.contains_key("SERVO1_MIN"));
        let clear = |writes: &mut Vec<(String, f32)>, function: f32, keep: &str| {
            for output in &channels {
                let name = format!("{output}_FUNCTION");
                if output != keep && params.get(&name).is_some_and(|p| p.value == function) {
                    writes.push((name, 0.0));
                }
            }
        };
        if selection_changed {
            for (ctl, _, function) in AXES {
                clear(&mut writes, function as f32, self.st.get(ctl));
            }
            if let Some(mode) = forms::resolve(params, "MNT_MODE") {
                writes.push((mode, 3.0));
            }
        } else {
            for (ctl, _, function) in AXES {
                if self.st.get(ctl) == "Disable" {
                    clear(&mut writes, function as f32, "");
                }
            }
        }
        let shutter = self.st.get("CMB_shuttertype");
        clear(&mut writes, 10.0, "");
        let trigger = match shutter {
            "Relay" => 1.0,
            "Transistor" => 4.0,
            "Disable" => 0.0,
            output => {
                writes.push((format!("{output}_FUNCTION"), 10.0));
                0.0
            }
        };
        writes.push(("CAM_TRIGG_TYPE".into(), trigger));
        for (ctl, _, function) in AXES {
            let output = self.st.get(ctl);
            if output != "Disable" {
                writes.push((format!("{output}_FUNCTION"), function as f32));
            }
        }
        writes
    }

    fn write(&self, params: &BTreeMap<String, Param>, changed: bool, app: &mut AppCtx) {
        let writes = self.routing_writes(params, changed);
        if let Some((missing, _)) = writes.iter().find(|(name, _)| !params.contains_key(name)) {
            app.toasts
                .error(format!("Failed to set Param\n{missing} is not available"));
        } else if let Some(link) = app.link {
            for (name, value) in writes {
                link.send(LinkCommand::SetParam { name, value });
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        let Some(link) = app.link else { return };
        let (params, target) = {
            let s = link.shared.read();
            (
                s.params.clone(),
                (s.cs.sysid, s.cs.compid, s.params.contains_key("SERVO1_MIN")),
            )
        };
        let activating = self.target != Some(target);
        self.prepare(&params, target);
        if activating && !app.readonly && params.contains_key("CAM_TRIGG_TYPE") {
            self.write(&params, false, app);
        }
        let clicks = ui
            .add_enabled_ui(params.contains_key("CAM_TRIGG_TYPE"), |ui| {
                forms::show(ui, app, "mount", &mut self.st)
            })
            .inner;
        for ctl in clicks {
            if ctl == "LNK_wiki" {
                ui.ctx().open_url(egui::OpenUrl::new_tab("http://copter.ardupilot.com/wiki/common-optional-hardware/common-cameras-and-gimbals/common-camera-gimbal/"));
            } else if !app.readonly
                && (AXES.iter().any(|(c, _, _)| *c == ctl) || ctl == "CMB_shuttertype")
            {
                self.write(&params, true, app);
                self.bind(&params);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};
    fn fixture(modern: bool) -> BTreeMap<String, Param> {
        let mut p = BTreeMap::new();
        let prefix = if modern { "SERVO" } else { "RC" };
        for n in 1..=14 {
            for (suffix, value) in [
                ("MIN", 1000.0),
                ("MAX", 2000.0),
                ("FUNCTION", 0.0),
                (
                    if modern { "REVERSED" } else { "REV" },
                    if modern { 0.0 } else { 1.0 },
                ),
            ] {
                p.insert(
                    format!("{prefix}{n}_{suffix}"),
                    Param {
                        value,
                        ptype: 9,
                        index: 0,
                    },
                );
            }
        }
        for (name, value) in [
            ("MNT_MODE", 0.0),
            ("CAM_TRIGG_TYPE", 0.0),
            ("CAM_SERVO_ON", 1900.0),
        ] {
            p.insert(
                name.into(),
                Param {
                    value,
                    ptype: 9,
                    index: 0,
                },
            );
        }
        p
    }

    #[test]
    fn rendered_selector_click_updates_the_simulator_and_receives_parameter_readback() {
        use mp_core::{
            comms::ConnectionSpec,
            link::{Link, LinkEvent},
            mavlink::dialects::ardupilotmega::MavMessage,
        };
        let port = std::net::UdpSocket::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let link = Link::connect_with_params(ConnectionSpec::UdpListen { port }, None, false);
        let _sim = mp_core::sim::spawn(port).unwrap();
        loop {
            if matches!(
                link.events.recv_timeout(Duration::from_secs(5)).unwrap(),
                LinkEvent::Connected
            ) {
                break;
            }
        }
        for (name, value) in [("SERVO6_FUNCTION", 7.0), ("SERVO9_FUNCTION", 10.0)] {
            link.send(LinkCommand::SetParam {
                name: name.into(),
                value,
            });
            loop {
                if let LinkEvent::ParamSet { name: n, ok } =
                    link.events.recv_timeout(Duration::from_secs(4)).unwrap()
                {
                    if n == name {
                        assert!(ok);
                        break;
                    }
                }
            }
        }
        let mut params = fixture(true);
        params.get_mut("SERVO6_FUNCTION").unwrap().value = 7.0;
        params.get_mut("SERVO9_FUNCTION").unwrap().value = 10.0;
        link.shared.write().params = params;
        let (tx, rx) = crossbeam_channel::unbounded();
        link.send(LinkCommand::Subscribe { msgid: 22, tx });

        let ctx = egui::Context::default();
        let assets = crate::assets::Assets::load(&ctx);
        let mut tiles = crate::map::TileCache::offline();
        let mut toasts = crate::app::Toasts::default();
        let mut settings = crate::app::Settings::default();
        let mut joystick = crate::joystick::Joystick::default();
        let mut page = MountPage::default();
        let mut frame = |events| {
            let mut actions = Vec::new();
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::Vec2::new(900.0, 700.0),
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
                    page.ui(ui, &mut app);
                },
            );
            fn labels(shape: &egui::Shape, out: &mut Vec<(String, egui::Pos2)>) {
                match shape {
                    egui::Shape::Text(t) => {
                        out.push((t.galley.text().to_string(), t.pos + t.galley.size() / 2.0))
                    }
                    egui::Shape::Vec(v) => {
                        for s in v {
                            labels(s, out);
                        }
                    }
                    _ => {}
                }
            }
            out.textures_delta.clear();
            let mut text = Vec::new();
            for s in out.shapes {
                labels(&s.shape, &mut text);
            }
            text
        };
        let readback = |name: &str, value: f32| {
            let end = Instant::now() + Duration::from_secs(5);
            while Instant::now() < end {
                if let Ok(MavMessage::PARAM_VALUE(p)) = rx.recv_timeout(Duration::from_millis(100))
                {
                    let bytes = &p.param_id[..];
                    let n = String::from_utf8_lossy(bytes)
                        .trim_end_matches('\0')
                        .to_string();
                    if n == name && p.param_value == value {
                        return;
                    }
                }
            }
            panic!("No readback for {name}={value}");
        };
        frame(Vec::new()); // Actual page activation emits clear/reassign and trigger writes.
        readback("SERVO9_FUNCTION", 0.0);
        readback("SERVO9_FUNCTION", 10.0);
        frame(Vec::new());
        let click_events = |pos, pressed| {
            vec![
                egui::Event::PointerMoved(pos),
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed,
                    modifiers: egui::Modifiers::NONE,
                },
            ]
        };
        let (at, size) = forms::ctl_rect("mount", "mavlinkComboBoxTilt").unwrap();
        let control = egui::Pos2::ZERO + at + size / 2.0;
        frame(click_events(control, true));
        frame(click_events(control, false));
        let labels = frame(Vec::new());
        let pos = labels
            .iter()
            .find(|(s, _)| s == "SERVO7")
            .unwrap_or_else(|| panic!("Missing output choice: {labels:?}"))
            .1;
        frame(click_events(pos, true));
        frame(click_events(pos, false));
        frame(Vec::new());
        readback("SERVO7_FUNCTION", 7.0);
        drop(frame);
        assert_eq!(page.st.get("mavlinkComboBoxTilt"), "SERVO7");
        assert_eq!(page.st.bindings["mavlinkNumericUpDownTSM"], "SERVO7_MIN");
        // Message subscriptions publish before the worker commits shared state.
        // The operation result is emitted after that state update.
        loop {
            if let LinkEvent::ParamSet { name, ok } = link.events.recv_timeout(Duration::from_secs(3)).unwrap() {
                if name == "SERVO7_FUNCTION" { assert!(ok); break; }
            }
        }
        assert_eq!(link.shared.read().params["SERVO7_FUNCTION"].value, 7.0);
    }

    #[test]
    fn activation_updates_functions_and_trigger_without_forcing_mount_mode() {
        let mut params = fixture(true);
        params.get_mut("SERVO6_FUNCTION").unwrap().value = 7.0;
        params.get_mut("SERVO7_FUNCTION").unwrap().value = 7.0;
        params.get_mut("SERVO9_FUNCTION").unwrap().value = 10.0;
        let mut page = MountPage::default();
        page.prepare(&params, (1, 1, true));
        let writes = page.routing_writes(&params, false);
        assert_eq!(
            writes,
            vec![
                ("SERVO9_FUNCTION".into(), 0.0),
                ("SERVO9_FUNCTION".into(), 10.0),
                ("CAM_TRIGG_TYPE".into(), 0.0),
                ("SERVO7_FUNCTION".into(), 7.0)
            ]
        );
        assert!(!writes
            .iter()
            .any(|(n, _)| n == "MNT_MODE" || n == "SERVO6_FUNCTION"));
    }

    #[test]
    fn mount_reassigns_outputs_without_leaving_old_functions_and_binds_servo_controls() {
        let mut params = fixture(true);
        params.get_mut("SERVO6_FUNCTION").unwrap().value = 7.0;
        params.get_mut("SERVO9_FUNCTION").unwrap().value = 10.0;
        let mut page = MountPage::default();
        page.prepare(&params, (1, 1, true));
        assert_eq!(page.st.get("mavlinkComboBoxTilt"), "SERVO6");
        assert_eq!(page.st.get("CMB_shuttertype"), "SERVO9");
        assert_eq!(page.st.bindings["mavlinkNumericUpDownTSM"], "SERVO6_MIN");
        assert_eq!(page.st.bindings["mavlinkCheckBoxTR"], "SERVO6_REVERSED");
        assert_eq!(page.st.check_values["mavlinkCheckBoxTR"], (1.0, 0.0));
        page.st.set("mavlinkComboBoxTilt", "SERVO7");
        page.st.set("CMB_shuttertype", "Relay");
        let writes = page.routing_writes(&params, true);
        assert_eq!(
            writes,
            vec![
                ("SERVO6_FUNCTION".into(), 0.0),
                ("MNT_MODE".into(), 3.0),
                ("SERVO9_FUNCTION".into(), 0.0),
                ("CAM_TRIGG_TYPE".into(), 1.0),
                ("SERVO7_FUNCTION".into(), 7.0)
            ]
        );
        page.bind(&params);
        assert_eq!(page.st.bindings["mavlinkNumericUpDownTSMX"], "SERVO7_MAX");
        assert!(forms::resolve(&params, &page.st.bindings["mavlinkNumericUpDownShutM"]).is_none());
    }

    #[test]
    fn mount_legacy_reverse_input_channels_and_disabling_match_reference() {
        let mut params = fixture(false);
        params.get_mut("RC5_FUNCTION").unwrap().value = 6.0;
        params.get_mut("CAM_TRIGG_TYPE").unwrap().value = 4.0;
        let mut page = MountPage::default();
        page.prepare(&params, (1, 1, false));
        assert_eq!(page.st.items["mavlinkComboBoxPan"][1], "RC5");
        assert_eq!(page.st.get("CMB_shuttertype"), "Transistor");
        assert_eq!(page.st.bindings["mavlinkCheckBoxPR"], "RC5_REV");
        assert_eq!(page.st.check_values["mavlinkCheckBoxPR"], (-1.0, 1.0));
        assert_eq!(
            page.st.options["CMB_inputch_pan"].last().unwrap(),
            &(16, "RC16".into())
        );
        page.st.set("mavlinkComboBoxPan", "Disable");
        assert!(page
            .routing_writes(&params, true)
            .contains(&("RC5_FUNCTION".into(), 0.0)));
        page.leave();
        params.get_mut("RC5_FUNCTION").unwrap().value = 0.0;
        page.prepare(&params, (1, 1, false));
        assert_eq!(page.st.get("mavlinkComboBoxPan"), "Disable");
    }
}
