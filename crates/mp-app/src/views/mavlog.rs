//! `Log/MavlinkLog.cs` ("Log" form, opened by Tlog > Kml or Graph): converts telemetry
//! logs to KML/GPX, text, CSV, MATLAB, extracts params / waypoints / CurrentState, and
//! graphs any message field in the ZedGraph pane.

use std::collections::{BTreeMap, VecDeque};
use std::path::{Path, PathBuf};

use egui::{Color32, Rect, Vec2};
use mp_core::mavlink::dialects::ardupilotmega::MavMessage;
use mp_core::mission::Locationwp;
use mp_core::state::CurrentState;
use mp_core::tlog::Record;

use crate::theme;

fn msg_name(m: &MavMessage) -> &'static str {
    <MavMessage as mp_core::mavlink::Message>::message_name(m)
}

#[cfg(test)]
mod file_workflow_tests {
    use super::*;
    use crate::platform::{FilePicker, picker_test_support as picker};
    #[test]
    fn telemetry_export_uses_the_selected_file_and_cancel_has_no_side_effects() {
        let root = picker::root();
        let mut bytes = 1_000_000u64.to_be_bytes().to_vec();
        mp_core::mavlink::write_versioned_msg(&mut bytes, mp_core::mavlink::MavlinkVersion::V2, mp_core::mavlink::MavHeader { system_id: 42, component_id: 1, sequence: 0 },
            &MavMessage::HEARTBEAT(Default::default())).unwrap();
        for name in ["a.tlog", "z.tlog"] { std::fs::write(root.join(name), &bytes).unwrap(); }
        let ctx = egui::Context::default();
        let mut log = MavlinkLog::default();
        log.file_request = Some(("Convert to CSV".into(), FilePicker::in_directory("Tlogs", root.clone(), &["tlog"], None)));
        { let mut draw = |ctx: &egui::Context| log.ui(ctx); picker::click(&ctx, &mut draw, "a.tlog"); picker::click(&ctx, &mut draw, "Open"); }
        assert!(std::fs::read_to_string(root.join("a.csv")).unwrap().contains("HEARTBEAT"));
        assert!(!root.join("z.csv").exists()); assert!(log.file_request.is_none());
        let status = log.status.clone();
        log.file_request = Some(("Convert to CSV".into(), FilePicker::in_directory("Tlogs", root.clone(), &["tlog"], None)));
        { let mut draw = |ctx: &egui::Context| log.ui(ctx); picker::click(&ctx, &mut draw, "Cancel"); }
        assert_eq!(log.status, status); assert!(!root.join("z.csv").exists());
        for name in ["a.tlog", "z.tlog", "a.csv"] { std::fs::remove_file(root.join(name)).unwrap(); } std::fs::remove_dir(root).unwrap();
    }
}

fn num(v: &str) -> Option<f64> {
    v.trim().parse::<f64>().ok()
}

#[derive(Default)]
pub struct MavlinkLog {
    pub open: bool,
    file_request: Option<(String, crate::platform::FilePicker)>,
    progress: f32,
    status: String,
    /// Graph Log: per message, per numeric field, the series (seconds from start, value)
    graph: BTreeMap<String, BTreeMap<String, Vec<(f64, f64)>>>,
    plotted: Vec<(String, String)>,
    graph_name: String,
}

impl MavlinkLog {
    fn each(&mut self, files: &[PathBuf], f: impl Fn(&Path, &[Record]) -> Result<PathBuf, String>) {
        let n = files.len().max(1);
        let mut done = Vec::new();
        for (i, p) in files.iter().enumerate() {
            match mp_core::tlog::read(p).map_err(|e| e.to_string()).and_then(|recs| f(p, &recs)) {
                Ok(out) => done.push(out.display().to_string()),
                Err(e) => done.push(format!("{}: {e}", p.display())),
            }
            self.progress = (i + 1) as f32 / n as f32;
        }
        self.status = done.join("\n");
    }

    pub fn ui(&mut self, ctx: &egui::Context) {
        if let Some(result) = self.file_request.as_mut().and_then(|(_, picker)| picker.show_many(ctx)) {
            let (action, _) = self.file_request.take().unwrap();
            if let Some(paths) = result { self.action_on_files(&action, &paths); }
        }
        if !self.open {
            return;
        }
        let mut open = true;
        egui::Window::new(crate::i18n::t("Log")).open(&mut open).collapsible(false).resizable(false).fixed_size([884.0, 380.0]).show(ctx, |ui| {
            let (full, _) = ui.allocate_exact_size(Vec2::new(884.0, 380.0), egui::Sense::hover());
            let o = full.min;
            let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
            let btns = [
                (129.0, "Create KML + GPX"),
                (203.0, "Convert to Text"),
                (277.0, "Convert to CSV"),
                (351.0, "Graph Log"),
                (425.0, "Extract Params"),
                (499.0, "Extract WPs"),
                (573.0, "Extract CS"),
                (647.0, "Create Matlab file"),
            ];
            let mut clicked = None;
            for (x, t) in btns {
                let rr = r(x, 6.0, 68.0, 30.0);
                if ui.put(rr, |ui: &mut egui::Ui| theme::button_sized(ui, t, rr.size())).clicked() {
                    clicked = Some(t);
                }
            }
            let pr = r(10.0, 42.0, 862.0, 26.0);
            ui.put(pr, |ui: &mut egui::Ui| {
                theme::progress(ui, self.progress, pr.size(), "");
                ui.allocate_response(Vec2::ZERO, egui::Sense::hover())
            });
            // splitContainer1: treeView1 (153) | zg1
            let tree = r(12.0, 74.0, 153.0, 293.0);
            let graph = r(169.0, 74.0, 703.0, 293.0);
            ui.painter().rect_filled(tree, 0.0, theme::control_bg());
            ui.scope_builder(egui::UiBuilder::new().max_rect(tree), |ui| {
                egui::ScrollArea::vertical().id_salt("mavlog_tree").show(ui, |ui| {
                    if self.graph.is_empty() {
                        theme::label(ui, &self.status);
                    }
                    for (m, fields) in &self.graph {
                        egui::CollapsingHeader::new(egui::RichText::new(m).color(Color32::WHITE)).id_salt(("mlt", m)).show(ui, |ui| {
                            for f in fields.keys() {
                                let key = (m.clone(), f.clone());
                                let mut on = self.plotted.contains(&key);
                                if ui.checkbox(&mut on, f).changed() {
                                    if on {
                                        self.plotted.push(key);
                                    } else {
                                        self.plotted.retain(|k| *k != key);
                                    }
                                }
                            }
                        });
                    }
                });
            });
            ui.scope_builder(egui::UiBuilder::new().max_rect(graph), |ui| {
                let series: Vec<(String, VecDeque<(f64, f64)>)> = self
                    .plotted
                    .iter()
                    .filter_map(|(m, f)| self.graph.get(m).and_then(|x| x.get(f)).map(|s| (format!("{m}.{f}"), s.iter().copied().collect())))
                    .collect();
                let t_last = series.iter().filter_map(|(_, s)| s.back().map(|p| p.0)).fold(0.0, f64::max);
                let refs: Vec<(&str, &VecDeque<(f64, f64)>)> = series.iter().map(|(n, s)| (n.as_str(), s)).collect();
                crate::zedgraph::plot_range(ui, graph.size(), &self.graph_name, &refs, 0.0, t_last.max(1.0));
            });
            if let Some(t) = clicked {
                self.action(t);
            }
        });
        if !open {
            self.open = false;
        }
    }

    fn action(&mut self, what: &str) {
        self.file_request = Some((what.into(), crate::platform::FilePicker::open_from(
            "Telemetry Log", crate::platform::data_dir().join("logs"), &["tlog"], what != "Graph Log")));
    }

    fn action_on_files(&mut self, what: &str, files: &[PathBuf]) {
        self.progress = 0.0;
        match what {
            "Create KML + GPX" => self.each(files, |p, recs| {
                let track: Vec<(f64, f64, f64)> = recs
                    .iter()
                    .filter_map(|r| match &r.msg {
                        MavMessage::GLOBAL_POSITION_INT(g) if g.lat != 0 => Some((g.lat as f64 * 1e-7, g.lon as f64 * 1e-7, g.alt as f64 / 1000.0)),
                        _ => None,
                    })
                    .collect();
                let coords: String = track.iter().map(|(la, lo, al)| format!("{lo},{la},{al} ")).collect();
                let kml = format!("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document><name>{}</name><Placemark><name>Flight</name><LineString><extrude>1</extrude><altitudeMode>absolute</altitudeMode><coordinates>{coords}</coordinates></LineString></Placemark></Document></kml>\n", p.display());
                let pts: String = track.iter().map(|(la, lo, al)| format!("<trkpt lat=\"{la}\" lon=\"{lo}\"><ele>{al}</ele></trkpt>")).collect();
                let gpx = format!("<?xml version=\"1.0\"?>\n<gpx version=\"1.1\" creator=\"Mission Planner\"><trk><trkseg>{pts}</trkseg></trk></gpx>\n");
                std::fs::write(p.with_extension("kml"), kml).map_err(|e| e.to_string())?;
                std::fs::write(p.with_extension("gpx"), gpx).map_err(|e| e.to_string())?;
                Ok(p.with_extension("kml"))
            }),
            "Convert to Text" => self.each(files, |p, recs| {
                let mut out = String::new();
                for r in recs {
                    out.push_str(&format!("{} {:?}\n", r.time_us, r.msg));
                }
                let to = p.with_extension("txt");
                std::fs::write(&to, out).map_err(|e| e.to_string())?;
                Ok(to)
            }),
            "Convert to CSV" => self.each(files, |p, recs| {
                let mut out = String::new();
                for r in recs {
                    let f = crate::tools::fields(&r.msg);
                    out.push_str(&format!("{},{},{},{}", r.time_us as f64 / 1e6, r.header.system_id, msg_name(&r.msg), f.iter().map(|(k, v)| format!("{k},{}", v.replace(',', ";"))).collect::<Vec<_>>().join(",")));
                    out.push('\n');
                }
                let to = p.with_extension("csv");
                std::fs::write(&to, out).map_err(|e| e.to_string())?;
                Ok(to)
            }),
            "Extract Params" => self.each(files, |p, recs| {
                let mut params: BTreeMap<String, f32> = BTreeMap::new();
                for r in recs {
                    if let MavMessage::PARAM_VALUE(v) = &r.msg {
                        let id: &[u8] = &v.param_id[..];
                        let end = id.iter().position(|b| *b == 0).unwrap_or(id.len());
                        params.insert(String::from_utf8_lossy(&id[..end]).to_string(), v.param_value);
                    }
                }
                if params.is_empty() {
                    return Err("no PARAM_VALUE messages".into());
                }
                let to = p.with_extension("param");
                std::fs::write(&to, mp_core::paramfile::save(params.iter().map(|(k, v)| (k, *v)))).map_err(|e| e.to_string())?;
                Ok(to)
            }),
            "Extract WPs" => self.each(files, |p, recs| {
                let mut items: BTreeMap<u16, Locationwp> = BTreeMap::new();
                for r in recs {
                    match &r.msg {
                        MavMessage::MISSION_ITEM_INT(m) if m.mission_type as u8 == 0 => {
                            items.insert(m.seq, Locationwp::from_mission_item_int(m));
                        }
                        MavMessage::MISSION_ITEM(m) if m.mission_type as u8 == 0 => {
                            items.insert(m.seq, Locationwp::from_mission_item(m));
                        }
                        _ => {}
                    }
                }
                if items.is_empty() {
                    return Err("no mission items".into());
                }
                let list: Vec<Locationwp> = items.into_values().collect();
                let to = p.with_extension("waypoints");
                std::fs::write(&to, mp_core::mission::write_waypoint_file(&list[0], &list[1..])).map_err(|e| e.to_string())?;
                Ok(to)
            }),
            "Extract CS" => self.each(files, |p, recs| {
                let mut cs = CurrentState::default();
                let names: Vec<&str> = cs.fields().iter().map(|(n, _)| *n).collect();
                let mut out = format!("time,{}\n", names.join(","));
                let mut last = 0u64;
                for r in recs {
                    cs.update(&r.msg);
                    if r.time_us.saturating_sub(last) >= 200_000 {
                        last = r.time_us;
                        out.push_str(&format!("{},{}\n", r.time_us as f64 / 1e6, cs.fields().iter().map(|(_, v)| v.to_string()).collect::<Vec<_>>().join(",")));
                    }
                }
                let to = p.with_file_name(format!("{}-cs.csv", p.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default()));
                std::fs::write(&to, out).map_err(|e| e.to_string())?;
                Ok(to)
            }),
            "Create Matlab file" => self.each(files, |p, recs| {
                let mut cols: BTreeMap<String, Vec<f64>> = BTreeMap::new();
                for r in recs {
                    let name = msg_name(&r.msg);
                    let fields = crate::tools::fields(&r.msg);
                    for (k, v) in fields {
                        if let Some(x) = num(&v) {
                            cols.entry(format!("{name}_{k}")).or_default().push(x);
                        }
                    }
                }
                let mut mat = Vec::new();
                for (name, data) in cols {
                    // MAT v4 header: type (0 = double, little endian), rows, cols, imag, name length
                    for v in [0u32, data.len() as u32, 1, 0, name.len() as u32 + 1] {
                        mat.extend_from_slice(&v.to_le_bytes());
                    }
                    mat.extend_from_slice(name.as_bytes());
                    mat.push(0);
                    for x in data {
                        mat.extend_from_slice(&x.to_le_bytes());
                    }
                }
                let to = p.with_extension("mat");
                std::fs::write(&to, mat).map_err(|e| e.to_string())?;
                Ok(to)
            }),
            "Graph Log" => {
                let Some(p) = files.first() else { return };
                match mp_core::tlog::read(&p) {
                    Ok(recs) => {
                        self.graph.clear();
                        self.plotted.clear();
                        let t0 = recs.first().map(|r| r.time_us).unwrap_or(0);
                        for r in &recs {
                            let t = r.time_us.saturating_sub(t0) as f64 / 1e6;
                            let m = self.graph.entry(msg_name(&r.msg).to_string()).or_default();
                            for (k, v) in crate::tools::fields(&r.msg) {
                                if let Some(x) = num(&v) {
                                    m.entry(k).or_default().push((t, x));
                                }
                            }
                        }
                        self.graph_name = p.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
                        self.progress = 1.0;
                    }
                    Err(e) => self.status = e.to_string(),
                }
            }
            _ => {}
        }
    }
}
