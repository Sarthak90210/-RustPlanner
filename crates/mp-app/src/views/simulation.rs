//! Port of `GCSViews/SITL` (SITL.resx layout): home-location map, Options, Advanced users only,
//! and the Plane / Rover / Multirotor / Helicopter tiles that download and start SITL.

use egui::{Color32, Pos2, Rect, Sense, Vec2};
use mp_core::comms::ConnectionSpec;

use crate::app::{AppAction, AppCtx};
use crate::sitl::{self, SitlRunner, StartArgs, Vehicle};
use crate::views::tuning::group_box;
use crate::{fonts, map, theme};

pub struct SimulationView {
    pub runner: SitlRunner,
    tiles: map::TileCache,
    map_state: map::MapState,
    pending_connect: bool,
    dragging: bool,
    centred: bool,
}

impl Default for SimulationView {
    fn default() -> Self {
        Self {
            runner: SitlRunner::default(),
            tiles: map::TileCache::new(Some(crate::platform::tile_cache_dir())),
            map_state: map::MapState::default(),
            pending_connect: false,
            dragging: false,
            centred: false,
        }
    }
}

const BUILTIN: usize = 3;

impl SimulationView {
    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx) {
        // after SITL starts: MainV2.View.ShowScreen(FlightData) and connect TCP 5760
        {
            let mut st = self.runner.status.lock();
            if st.ready && self.pending_connect {
                st.ready = false;
                self.pending_connect = false;
                app.settings.view = crate::app::View::FlightData;
                app.actions.push(AppAction::Connect(ConnectionSpec::Tcp { host: "127.0.0.1".into(), port: 5760 }));
            }
            if let Some(e) = st.error.take() {
                self.pending_connect = false;
                app.toasts.error(format!("Failed to download and start sitl\n{e}"));
            }
        }

        let full = ui.max_rect();
        let o = full.min;
        let w = full.width().max(879.0) - 25.0;
        let bottom_block = 532.0 - 290.0;
        let map_h = (full.height() - bottom_block - 6.0).max(150.0);
        let p = ui.painter().clone();
        let f = fonts::ui_id(fonts::UI_SIZE);

        // groupBox1: Home Location - Drag Me
        let g1 = Rect::from_min_size(o + Vec2::new(13.0, 3.0), Vec2::new(w - 13.0, map_h));
        group_box(&p, g1, "Home Location - Drag Me");
        let map_rect = Rect::from_min_max(g1.min + Vec2::new(3.0, 16.0), g1.max - Vec2::new(3.0, 3.0));
        let (mut hlat, mut hlng) = app.settings.sitl_home;
        if !self.centred {
            self.centred = true;
            self.map_state.center_on(hlat, hlng);
            self.map_state.provider = app.settings.fd_map.provider;
        }
        let assets = app.assets;
        let mut moved = None;
        let mut dragging = self.dragging;
        ui.scope_builder(egui::UiBuilder::new().max_rect(map_rect), |ui| {
            map::show(ui, &mut self.map_state, &mut self.tiles, |_ui, view, resp, painter| {
                let pt = view.to_screen(hlat, hlng);
                let pin = map::draw_pin(painter, assets.marker_red.id(), assets.marker_shadow.id(), pt);
                if resp.drag_started() {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        dragging = pin.contains(pos);
                    }
                }
                if dragging && resp.dragged() {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        moved = Some(view.to_latlng(pos));
                    }
                }
                if resp.drag_stopped() {
                    dragging = false;
                }
                if resp.double_clicked() {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        moved = Some(view.to_latlng(pos));
                    }
                }
                dragging
            });
        });
        self.dragging = dragging;
        if let Some((a, b)) = moved {
            hlat = a;
            hlng = b;
            app.settings.sitl_home = (a, b);
        }

        // groupBox3: Options
        let base_y = g1.bottom() + 6.0;
        let g3 = Rect::from_min_size(Pos2::new(o.x + 13.0, base_y), Vec2::new(140.0, 72.0));
        group_box(&p, g3, "Options");
        p.text(g3.min + Vec2::new(7.0, 20.0), egui::Align2::LEFT_TOP, "Heading", f.clone(), Color32::WHITE);
        ui.put(Rect::from_min_size(g3.min + Vec2::new(60.0, 18.0), Vec2::new(51.0, 20.0)), egui::DragValue::new(&mut app.settings.sitl_heading).range(0.0..=359.0));
        ui.put(Rect::from_min_size(g3.min + Vec2::new(10.0, 44.0), Vec2::new(123.0, 21.0)), |ui: &mut egui::Ui| {
            let names = [sitl::VERSIONS[0], sitl::VERSIONS[1], sitl::VERSIONS[2], "Built-in (offline)"];
            egui::ComboBox::from_id_salt("cmb_version")
                .width(115.0)
                .selected_text(names[app.settings.sitl_version.min(3)])
                .show_ui(ui, |ui| {
                    for (i, n) in names.iter().enumerate() {
                        ui.selectable_value(&mut app.settings.sitl_version, i, *n);
                    }
                })
                .response
        });

        // groupBox4: Advanced users only
        let g4 = Rect::from_min_size(Pos2::new(o.x + 152.0, base_y), Vec2::new(w - 152.0, 72.0));
        group_box(&p, g4, "Advanced users only");
        p.text(g4.min + Vec2::new(6.0, 20.0), egui::Align2::LEFT_TOP, "Sim Speed", f.clone(), Color32::WHITE);
        ui.put(Rect::from_min_size(g4.min + Vec2::new(70.0, 18.0), Vec2::new(51.0, 20.0)), egui::DragValue::new(&mut app.settings.sitl_speed).range(1..=50));
        p.text(g4.min + Vec2::new(127.0, 20.0), egui::Align2::LEFT_TOP, "Model", f.clone(), Color32::WHITE);
        ui.put(Rect::from_min_size(g4.min + Vec2::new(169.0, 17.0), Vec2::new(121.0, 21.0)), |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("cmb_model")
                .width(113.0)
                .selected_text(app.settings.sitl_model.clone())
                .show_ui(ui, |ui| {
                    for m in ["", "+", "quad", "X", "hexa", "octa", "heli", "plane", "quadplane", "rover", "boat", "balancebot"] {
                        ui.selectable_value(&mut app.settings.sitl_model, m.to_string(), if m.is_empty() { "(default)" } else { m });
                    }
                })
                .response
        });
        p.text(g4.min + Vec2::new(296.0, 20.0), egui::Align2::LEFT_TOP, "Extra command line", f.clone(), Color32::WHITE);
        ui.put(Rect::from_min_size(g4.min + Vec2::new(401.0, 17.0), Vec2::new(230.0, 20.0)), egui::TextEdit::singleline(&mut app.settings.sitl_extra));
        ui.put(Rect::from_min_size(g4.min + Vec2::new(638.0, 17.0), Vec2::new(60.0, 17.0)), egui::Checkbox::new(&mut app.settings.sitl_wipe, "Wipe"));
        for (x, t) in [(9.0, "Copter Swarm - Single link"), (104.0, "Copter Swarm - Multilink"), (199.0, "Plane Swarm - Multilink"), (294.0, "Rover Swarm - Multilink")] {
            let r = Rect::from_min_size(g4.min + Vec2::new(x, 44.0), Vec2::new(89.0, 23.0));
            if ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, t, r.size())).clicked() {
                use mp_core::sim::SimKind;
                let kind = if t.starts_with("Plane") {
                    SimKind::Plane
                } else if t.starts_with("Rover") {
                    SimKind::Rover
                } else {
                    SimKind::Copter
                };
                app.actions.push(crate::app::AppAction::StartSwarm(kind, 3));
            }
        }

        // groupBox2: Please select a firmware to run
        let g2 = Rect::from_min_size(Pos2::new(o.x + 13.0, base_y + 78.0), Vec2::new(w - 13.0, 152.0));
        group_box(&p, g2, "Please select a firmware to run");
        let panel = Pos2::new(g2.center().x - 358.0, g2.top() + 12.0);
        let tiles = [
            (Vehicle::Plane, (3.0, 3.0, 131.0), "Plane", 48.0, (assets.sitl_plane.id(), assets.sitl_plane_over.id())),
            (Vehicle::Rover, (159.0, 3.0, 150.0), "Rover", 217.0, (assets.sitl_rover.id(), assets.sitl_rover_over.id())),
            (Vehicle::Copter, (331.0, 3.0, 125.0), "Multirotor", 370.0, (assets.sitl_quad.id(), assets.sitl_quad_over.id())),
            (Vehicle::Heli, (477.0, 3.0, 235.0), "Helicopter", 569.0, (assets.sitl_heli.id(), assets.sitl_heli_over.id())),
        ];
        let busy = self.runner.status.lock().busy;
        for (veh, (x, y, tw), label, lx, (normal, over)) in tiles {
            let r = Rect::from_min_size(panel + Vec2::new(x, y), Vec2::new(tw, 112.0));
            let resp = ui.interact(r, ui.scope_id().with(("sitl", label)), Sense::click());
            let tex = if resp.hovered() { over } else { normal };
            ui.painter().image(tex, r, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
            ui.painter().text(panel + Vec2::new(lx, 120.0), egui::Align2::LEFT_TOP, label, f.clone(), Color32::WHITE);
            if resp.clicked() && !busy {
                if app.settings.sitl_version == BUILTIN {
                    if veh == Vehicle::Copter {
                        app.actions.push(AppAction::StartSim);
                        app.settings.view = crate::app::View::FlightData;
                    } else {
                        app.toasts.error("The built-in simulator is a multirotor only");
                    }
                } else {
                    self.runner.start(StartArgs {
                        vehicle: veh,
                        version: app.settings.sitl_version,
                        model: app.settings.sitl_model.clone(),
                        home: (hlat, hlng, 584.0, app.settings.sitl_heading as f64),
                        speed: app.settings.sitl_speed,
                        extra: app.settings.sitl_extra.clone(),
                        wipe: app.settings.sitl_wipe,
                    });
                    self.pending_connect = true;
                }
            }
        }

        // Common.LoadingBox("Downloading", "Downloading sitl software")
        let st = self.runner.status.lock().clone();
        if st.busy {
            egui::Window::new("Downloading").collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO).show(ui.ctx(), |ui| {
                ui.horizontal(|ui| {
                    ui.spinner();
                    theme::label(ui, st.message.as_str());
                });
            });
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
        }
        if app.sim_running {
            let r = Rect::from_min_size(Pos2::new(g2.right() - 110.0, g2.top() + 12.0), Vec2::new(100.0, 23.0));
            if ui.put(r, |ui: &mut egui::Ui| theme::button_sized(ui, "Stop simulator", r.size())).clicked() {
                app.actions.push(AppAction::StopSim);
            }
        }
        ui.allocate_rect(Rect::from_min_max(o, Pos2::new(o.x + w, g2.bottom())), Sense::hover());
    }
}
