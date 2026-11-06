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
