//! ConfigHWCompass2's device-ID based priority list and UpdateFirst3 semantics.
use mp_core::link::Param;
use std::collections::BTreeMap;

#[derive(Debug, Clone)]
pub struct Row {
    pub id: u32,
    pub missing: bool,
    pub external: bool,
    pub orientation: Option<i64>,
}

#[derive(Default)]
pub struct PageState {
    pub rows: Vec<Row>,
    target: Option<(u8, u8)>,
    active: bool,
    pub reboot_required: bool,
    pub reboot_prompt: bool,
    pub calibrate_after_reboot: bool,
    awaiting_reboot: bool,
    standalone_reboot: bool,
}

impl PageState {
    /// Preserve the complete row order until deactivation, including rows outside
    /// the three priority parameters that are written back to the autopilot.
    pub fn activate(&mut self, target: (u8, u8), params: &BTreeMap<String, Param>) -> bool {
        if !self.active || self.target != Some(target) {
            if self.target != Some(target) {
                self.reboot_required = false;
                self.reboot_prompt = false;
                self.calibrate_after_reboot = false;
                self.awaiting_reboot = false;
                self.standalone_reboot = false;
            }
            self.target = Some(target);
            self.rows = rows(params);
            self.active = true;
            return self.rows.iter().any(|r| r.missing);
        }
        false
    }

    pub fn leave(&mut self) {
        if self.active && self.reboot_required {
            self.reboot_prompt = true;
            self.standalone_reboot = false;
        }
        self.active = false;
    }

    pub fn move_row(&mut self, a: usize, b: usize) -> Vec<(String, f32)> {
        self.rows.swap(a, b);
        self.reboot_required = true;
        priority_writes(&self.rows)
    }

    pub fn remove_missing(&mut self) -> Vec<(String, f32)> {
        self.rows.retain(|r| !r.missing);
        self.reboot_required = true;
        priority_writes(&self.rows)
    }

    pub fn start_calibration(&mut self) -> bool {
        if self.reboot_required {
            self.reboot_prompt = true;
            self.standalone_reboot = false;
            self.calibrate_after_reboot = true;
            false
        } else {
            true
        }
    }

    pub fn answer_reboot(&mut self, yes: bool) -> bool {
        self.reboot_prompt = false;
        self.awaiting_reboot = yes;
        if !yes {
            self.calibrate_after_reboot = false;
        }
        yes
    }

    pub fn request_reboot(&mut self) {
        self.reboot_prompt = true;
        self.standalone_reboot = true;
        self.calibrate_after_reboot = false;
    }

    fn complete_reboot(&mut self, ok: bool) -> bool {
        self.awaiting_reboot = false;
        let calibrate = ok && self.calibrate_after_reboot;
        self.calibrate_after_reboot = false;
        if ok {
            self.reboot_required = false;
        }
        calibrate
    }

    pub fn on_event(
        &mut self,
        event: &mp_core::link::LinkEvent,
        link: &mp_core::link::Link,
        toasts: &mut crate::app::Toasts,
    ) {
        if !self.awaiting_reboot {
            return;
        }
        match event {
            mp_core::link::LinkEvent::CommandAck {
                command: 246, ok, ..
            } => {
                let calibrate = self.complete_reboot(*ok);
                if !ok {
                    toasts.error("Reboot failed. please manually reboot the hardware.");
                }
                if calibrate {
                    begin_calibration(link);
                }
            }
            mp_core::link::LinkEvent::Disconnected(_) => {
                self.complete_reboot(false);
                toasts.error("Error communicating with the autopilot during reboot");
            }
            _ => {}
        }
    }

    pub fn dialogs(&mut self, ctx: &egui::Context, app: &mut crate::app::AppCtx) {
        if !self.reboot_prompt && !self.awaiting_reboot {
            return;
        }
        if !app.connected() {
            self.reboot_prompt = false;
            self.calibrate_after_reboot = false;
            self.awaiting_reboot = false;
            return;
        }
        if app.link.is_some_and(|link| {
            let s = link.shared.read();
            self.target != Some((s.cs.sysid, s.cs.compid))
        }) {
            self.reboot_prompt = false;
            self.calibrate_after_reboot = false;
            self.reboot_required = false;
            self.awaiting_reboot = false;
            return;
        }
        let mut answer = None;
        egui::Modal::new(egui::Id::new("compass_reboot")).show(ctx, |ui| {
            ui.heading("Reboot");
            if self.awaiting_reboot {
                ui.label("Rebooting...");
                return;
            }
            ui.label(if self.standalone_reboot {
                "Reboot?"
            } else {
                "Reboot required, reboot now?"
            });
            ui.horizontal(|ui| {
                if ui
                    .button(if self.standalone_reboot { "OK" } else { "Yes" })
                    .clicked()
                {
                    answer = Some(true);
                }
                if ui
                    .button(if self.standalone_reboot {
                        "Cancel"
                    } else {
                        "No"
                    })
                    .clicked()
                {
                    answer = Some(false);
                }
            });
        });
        if let Some(yes) = answer {
            self.answer_reboot(yes);
            if let Some(link) = app.link.filter(|_| app.connected() && !app.readonly) {
                if yes {
                    link.send(mp_core::link::LinkCommand::CommandLong {
                        command: 246,
                        params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    });
                }
            }
        }
    }
}

pub fn begin_calibration(link: &mp_core::link::Link) {
    {
        let mut s = link.shared.write();
        s.mag_progress = [0; 3];
        s.mag_report = Default::default();
    }
    link.send(mp_core::link::LinkCommand::CommandLong {
        command: 42424,
        params: [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    });
}

fn value(params: &BTreeMap<String, Param>, names: &[&str]) -> Option<f32> {
    names.iter().find_map(|n| params.get(*n).map(|p| p.value))
}

pub fn rows(params: &BTreeMap<String, Param>) -> Vec<Row> {
    let detected: Vec<u32> = params
        .iter()
        .filter(|(n, p)| n.starts_with("COMPASS") && n.contains("DEV_ID") && p.value != 0.0)
        .map(|(_, p)| p.value as u32)
        .collect();
    let priorities: Vec<u32> = params
        .iter()
        .filter(|(n, p)| n.starts_with("COMPASS_PRIO") && n.ends_with("_ID") && p.value != 0.0)
        .map(|(_, p)| p.value as u32)
        .collect();
    let sensors = [
        (
            ["COMPASS_DEV_ID", "COMPASS1_DEV_ID"],
            ["COMPASS_ORIENT", "COMPASS1_ORIENT"],
            ["COMPASS_EXTERNAL", "COMPASS1_EXTERN"],
        ),
        (
            ["COMPASS_DEV_ID2", "COMPASS2_DEV_ID"],
            ["COMPASS_ORIENT2", "COMPASS2_ORIENT"],
            ["COMPASS_EXTERN2", "COMPASS2_EXTERN"],
        ),
        (
            ["COMPASS_DEV_ID3", "COMPASS3_DEV_ID"],
            ["COMPASS_ORIENT3", "COMPASS3_ORIENT"],
            ["COMPASS_EXTERN3", "COMPASS3_EXTERN"],
        ),
    ];
    priorities
        .iter()
        .chain(detected.iter().filter(|id| !priorities.contains(id)))
        .map(|id| {
            let mut row = Row {
                id: *id,
                missing: !detected.contains(id),
                external: false,
                orientation: None,
            };
            for (dev, orientation, external) in &sensors {
                if value(params, dev).map(|v| v as u32) == Some(*id) {
                    row.orientation = value(params, orientation).map(|v| v as i64);
                    row.external = value(params, external).is_some_and(|v| v > 0.0);
                }
            }
            row
        })
        .collect()
}

pub fn priority_writes(rows: &[Row]) -> Vec<(String, f32)> {
    (0..3)
        .filter(|i| *i != 0 || !rows.is_empty())
        .map(|i| {
            (
                format!("COMPASS_PRIO{}_ID", i + 1),
                rows.get(i).map(|r| r.id as f32).unwrap_or(0.0),
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(values: &[(&str, f32)]) -> BTreeMap<String, Param> {
        values
            .iter()
            .map(|(n, value)| {
                (
                    n.to_string(),
                    Param {
                        value: *value,
                        ptype: 9,
                        index: 0,
                    },
                )
            })
            .collect()
    }

    #[test]
    fn five_device_order_survives_frames_and_moves_into_the_first_three() {
        let p = fixture(&[
            ("COMPASS_PRIO1_ID", 101.0),
            ("COMPASS_PRIO2_ID", 202.0),
            ("COMPASS_PRIO3_ID", 303.0),
            ("COMPASS_DEV_ID", 101.0),
            ("COMPASS_DEV_ID2", 202.0),
            ("COMPASS_DEV_ID3", 303.0),
            ("COMPASS_DEV_ID4", 404.0),
            ("COMPASS_DEV_ID5", 505.0),
        ]);
        let mut page = PageState::default();
        page.activate((1, 1), &p);
        page.move_row(4, 3);
        page.activate((1, 1), &p); // A subsequent frame must retain the local list.
        assert_eq!(page.rows[3].id, 505);
        let writes = page.move_row(3, 2);
        assert_eq!(writes[2], ("COMPASS_PRIO3_ID".into(), 505.0));
        assert!(!page.start_calibration());
        assert!(page.reboot_prompt && page.reboot_required);
        assert!(!page.answer_reboot(false));
        assert!(page.reboot_required);
        assert!(!page.start_calibration());
        assert!(page.answer_reboot(true));
        assert!(page.reboot_required);
        assert!(page.complete_reboot(true));
        assert!(page.start_calibration());
    }

    #[test]
    fn leaving_after_reorder_prompts_and_another_vehicle_resets_pending_state() {
        let p = fixture(&[("COMPASS_DEV_ID", 101.0), ("COMPASS_DEV_ID2", 202.0)]);
        let mut page = PageState::default();
        page.activate((1, 1), &p);
        page.move_row(1, 0);
        page.leave();
        assert!(page.reboot_prompt);
        page.answer_reboot(false);
        page.activate((2, 1), &p);
        assert!(!page.reboot_prompt && !page.reboot_required);
        assert_eq!(page.rows[0].id, 101);
    }

    #[test]
    fn standalone_reboot_confirms_and_failed_reboot_preserves_required_state() {
        let mut page = PageState::default();
        page.reboot_required = true;
        page.request_reboot();
        assert!(page.reboot_prompt && page.standalone_reboot && !page.awaiting_reboot);
        assert!(!page.answer_reboot(false));
        assert!(page.reboot_required && !page.awaiting_reboot);
        page.request_reboot();
        assert!(page.answer_reboot(true));
        assert!(page.reboot_required && page.awaiting_reboot);
        assert!(!page.complete_reboot(false));
        assert!(page.reboot_required && !page.awaiting_reboot);
        page.request_reboot();
        page.answer_reboot(true);
        page.complete_reboot(true);
        assert!(!page.reboot_required);
    }

    #[test]
    fn reordered_priorities_use_device_orientation_and_external_state() {
        let p = fixture(&[
            ("COMPASS_PRIO1_ID", 202.0),
            ("COMPASS_PRIO2_ID", 101.0),
            ("COMPASS_DEV_ID", 101.0),
            ("COMPASS_DEV_ID2", 202.0),
            ("COMPASS_ORIENT", 4.0),
            ("COMPASS_ORIENT2", 8.0),
            ("COMPASS_EXTERNAL", 0.0),
            ("COMPASS_EXTERN2", 1.0),
        ]);
        let r = rows(&p);
        assert_eq!(r.iter().map(|r| r.id).collect::<Vec<_>>(), vec![202, 101]);
        assert!(!r[0].missing && !r[1].missing);
        assert!(r[0].external && !r[1].external);
        assert_eq!((r[0].orientation, r[1].orientation), (Some(8), Some(4)));
    }

    #[test]
    fn missing_priorities_append_detected_devices_and_clear_vacant_priorities() {
        let p = fixture(&[
            ("COMPASS_PRIO1_ID", 999.0),
            ("COMPASS_PRIO2_ID", 0.0),
            ("COMPASS_PRIO3_ID", 0.0),
            ("COMPASS_DEV_ID", 101.0),
            ("COMPASS_DEV_ID2", 202.0),
            ("COMPASS_DEV_ID3", 303.0),
            ("COMPASS_DEV_ID4", 404.0),
        ]);
        let mut r = rows(&p);
        assert_eq!(r.len(), 5);
        assert!(r[0].missing);
        r.swap(0, 1);
        assert_eq!(
            priority_writes(&r),
            vec![
                ("COMPASS_PRIO1_ID".into(), 101.0),
                ("COMPASS_PRIO2_ID".into(), 999.0),
                ("COMPASS_PRIO3_ID".into(), 202.0)
            ]
        );
        r.retain(|r| !r.missing);
        r.truncate(1);
        assert_eq!(
            priority_writes(&r),
            vec![
                ("COMPASS_PRIO1_ID".into(), 101.0),
                ("COMPASS_PRIO2_ID".into(), 0.0),
                ("COMPASS_PRIO3_ID".into(), 0.0)
            ]
        );
    }

    #[test]
    fn numbered_compass_aliases_resolve_by_id() {
        let p = fixture(&[
            ("COMPASS_PRIO1_ID", 202.0),
            ("COMPASS1_DEV_ID", 101.0),
            ("COMPASS2_DEV_ID", 202.0),
            ("COMPASS2_ORIENT", 12.0),
            ("COMPASS2_EXTERN", 1.0),
        ]);
        let r = rows(&p);
        assert_eq!(r[0].id, 202);
        assert_eq!(r[0].orientation, Some(12));
        assert!(r[0].external && !r[0].missing);
    }
}
