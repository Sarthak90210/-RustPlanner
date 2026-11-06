//! FlightPlanner's Controls.Icon.Polygon / Zoom and contextMenuStripZoom.

use egui::{Color32, Pos2, Rect, Sense, Stroke, Vec2};

use super::FlightPlanView;
use crate::app::AppCtx;

fn icon_rect(map: Rect, y: f32) -> Rect {
    Rect::from_min_size(map.min + Vec2::new(10.0, y), Vec2::splat(30.0))
}

pub(super) fn over_control(map: Rect, pos: Pos2) -> bool {
    icon_rect(map, 100.0).contains(pos)
        || icon_rect(map, 135.0).contains(pos)
        || (map.contains(pos) && pos.x >= map.right() - 52.0)
        || Rect::from_min_size(
            map.min + Vec2::new(map.width() - 177.0, 3.0),
            Vec2::new(121.0, 21.0),
        )
        .contains(pos)
}

fn icon(ui: &mut egui::Ui, map: Rect, polygon: bool, selected: bool) -> egui::Response {
    let rect = icon_rect(map, if polygon { 100.0 } else { 135.0 });
    let resp = ui.interact(
        rect,
        ui.id()
            .with(if polygon { "polygon_icon" } else { "zoom_icon" }),
        Sense::click(),
    );
    let p = ui.painter().with_clip_rect(map);
    let stroke = Stroke::new(1.0, Color32::from_gray(245));
    p.circle_filled(
        rect.center(),
        15.0,
        if selected {
            Color32::from_rgb(46, 139, 87)
        } else {
            Color32::BLACK
        },
    );
    p.circle_stroke(rect.center(), 15.0, stroke);
    if polygon {
        let pts = [
            (8.0, 8.0),
            (22.0, 5.0),
            (15.0, 27.0),
            (10.0, 25.0),
            (8.0, 8.0),
        ];
        p.add(egui::Shape::line(
            pts.iter()
                .map(|&(x, y)| rect.min + Vec2::new(x, y))
                .collect(),
            stroke,
        ));
    } else {
        let c = rect.min + Vec2::new(18.75, 11.25);
        p.circle_stroke(c, 7.5, stroke);
        p.line_segment(
            [
                rect.min + Vec2::new(7.5, 22.5),
                rect.min + Vec2::splat(15.0),
            ],
            stroke,
        );
        p.line_segment([c - Vec2::new(3.5, 0.0), c + Vec2::new(3.5, 0.0)], stroke);
        p.line_segment([c - Vec2::new(0.0, 3.5), c + Vec2::new(0.0, 3.5)], stroke);
    }
    resp.on_hover_text(if polygon {
        "Polygon (right-click to clear)"
    } else {
        "Zoom"
    })
}

impl FlightPlanView {
    pub(super) fn map_controls(&mut self, ui: &mut egui::Ui, rect: Rect, app: &mut AppCtx) {
        let poly = icon(ui, rect, true, self.drawing_polygon);
        if poly.secondary_clicked() {
            self.polygon.clear();
            self.drawing_polygon = false;
        }
        egui::Popup::menu(&poly).show(|ui| self.polygon_menu(ui, app));
        let zoom = icon(ui, rect, false, false);
        egui::Popup::menu(&zoom).show(|ui| self.zoom_menu(ui, rect.size(), app));
    }

    fn zoom_menu(&mut self, ui: &mut egui::Ui, size: Vec2, app: &mut AppCtx) {
        if ui.button(crate::i18n::t("Zoom to Vehicle")).clicked() {
            let pos = app.link.map(|l| {
                let s = l.shared.read();
                (s.cs.lat, s.cs.lng)
            });
            if let Some((lat, lng)) = pos.filter(|&(a, b)| a != 0.0 || b != 0.0) {
                app.settings.fp_map.center_on(lat, lng);
                app.settings.fp_map.zoom = app.settings.fp_map.zoom.max(17.0);
            } else {
                app.toasts.error("Invalid Location");
            }
            ui.close();
        }
        if ui.button(crate::i18n::t("Zoom to Mission")).clicked() {
            let items = if self.mission_type == 0 {
                &self.items
            } else {
                &self.stash[0]
            };
            let pts: Vec<_> = std::iter::once(&self.home)
                .chain(items.iter())
                .filter(|w| w.has_location() && (w.lat != 0.0 || w.lng != 0.0))
                .map(|w| (w.lat, w.lng))
                .collect();
            app.settings
                .fp_map
                .fit_points(&pts, size - Vec2::new(60.0, 0.0));
            ui.close();
        }
        if ui.button(crate::i18n::t("Zoom to Home")).clicked() {
            let home = app
                .link
                .and_then(|l| l.shared.read().cs.home)
                .filter(|p| !p.is_zero());
            let (lat, lng) = home
                .map(|p| (p.lat, p.lng))
                .unwrap_or((self.home.lat, self.home.lng));
            if lat != 0.0 || lng != 0.0 {
                app.settings.fp_map.center_on(lat, lng);
                app.settings.fp_map.zoom = app.settings.fp_map.zoom.max(17.0);
            } else {
                app.toasts.error("Invalid Location");
            }
            ui.close();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{app::Settings, assets::Assets, joystick::Joystick, map::TileCache};

    struct Planner {
        ctx: egui::Context,
        view: FlightPlanView,
        settings: Settings,
        tiles: TileCache,
        assets: Assets,
        joystick: Joystick,
        toasts: crate::app::Toasts,
        labels: Vec<(String, Pos2)>,
        rect: Rect,
    }

    impl Planner {
        fn new() -> Self {
            let ctx = egui::Context::default();
            let assets = Assets::load(&ctx);
            let mut p = Self {
                ctx,
                assets,
                view: FlightPlanView::default(),
                settings: Settings::default(),
                tiles: TileCache::offline(),
                joystick: Joystick::default(),
                toasts: Default::default(),
                labels: Vec::new(),
                rect: Rect::NOTHING,
            };
            p.frame(Vec::new());
            p.frame(Vec::new());
            p
        }

        fn frame(&mut self, events: Vec<egui::Event>) {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 700.0))),
                events,
                ..Default::default()
            };
            let mut actions = Vec::new();
            let mut output = self.ctx.run_ui(input, |ui| {
                let mut app = AppCtx {
                    link: None,
                    extra_links: &[],
                    tiles: &mut self.tiles,
                    toasts: &mut self.toasts,
                    settings: &mut self.settings,
                    actions: &mut actions,
                    sim_running: false,
                    assets: &self.assets,
                    readonly: false,
                    joystick: &mut self.joystick,
                };
                self.rect = ui.available_rect_before_wrap();
                self.view.map_ui(ui, &mut app);
                self.view.menu_dialogs(ui.ctx(), &mut app);
            });
            output.textures_delta.clear();
            fn labels(shape: &egui::Shape, out: &mut Vec<(String, Pos2)>) {
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
            self.labels.clear();
            for s in &output.shapes {
                labels(&s.shape, &mut self.labels);
            }
        }

        fn click(&mut self, pos: Pos2, button: egui::PointerButton) {
            for pressed in [true, false] {
                self.frame(vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button,
                        pressed,
                        modifiers: egui::Modifiers::NONE,
                    },
                ]);
            }
            self.frame(Vec::new());
        }

        fn menu_click(&mut self, label: &str) {
            let pos = self
                .labels
                .iter()
                .find(|(t, _)| t == label)
                .unwrap_or_else(|| panic!("Missing menu item {label}: {:?}", self.labels))
                .1;
            self.click(pos, egui::PointerButton::Primary);
        }
    }

    #[test]
    fn dropping_home_on_an_icon_ends_the_drag() {
        let mut p = Planner::new();
        p.view.home = mp_core::mission::Locationwp::waypoint(
            p.settings.fp_map.lat,
            p.settings.fp_map.lng,
            10.0,
            3,
        );
        p.frame(Vec::new());
        let pin = p.rect.center() - Vec2::new(0.0, 16.0);
        p.frame(vec![
            egui::Event::PointerMoved(pin),
            egui::Event::PointerButton {
                pos: pin,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        p.frame(vec![egui::Event::PointerMoved(pin + Vec2::new(10.0, 0.0))]);
        assert!(p.view.dragging.is_some());
        let icon = icon_rect(p.rect, 100.0).center();
        p.frame(vec![
            egui::Event::PointerMoved(icon),
            egui::Event::PointerButton {
                pos: icon,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        assert!(p.view.dragging.is_none());
    }

    #[test]
    fn polygon_icon_owns_clicks_and_reuses_polygon_menu() {
        let mut p = Planner::new();
        p.view.polygon.push((12.0, 79.0));
        let poly = icon_rect(p.rect, 100.0).center();
        p.click(poly, egui::PointerButton::Primary);
        assert!(p.view.items.is_empty(), "Icon click added a waypoint");
        assert_eq!((p.view.home.lat, p.view.home.lng), (0.0, 0.0));
        assert!(!p.labels.iter().any(|(s, _)| s == "Fence Inclusion"));
        p.menu_click("Draw a Polygon");
        assert!(p.view.drawing_polygon);
        assert_eq!(
            p.view.polygon.len(),
            1,
            "Resuming polygon drawing discarded existing points"
        );
        p.click(p.rect.center(), egui::PointerButton::Primary);
        assert_eq!(p.view.polygon.len(), 2);
        assert!(p.view.items.is_empty());
        p.click(poly, egui::PointerButton::Secondary);
        assert!(p.view.polygon.is_empty());
        assert!(!p.view.drawing_polygon);
        assert!(
            p.view.ctx_pos.is_none(),
            "Icon right-click opened the waypoint context menu"
        );
    }

    #[test]
    fn zoom_menu_fits_mission_and_centers_home_without_editing_route() {
        let mut p = Planner::new();
        p.view.home = mp_core::mission::Locationwp::waypoint(12.0, 79.0, 10.0, 3);
        p.view
            .items
            .push(mp_core::mission::Locationwp::waypoint(12.1, 79.1, 20.0, 3));
        let zoom = icon_rect(p.rect, 135.0).center();
        p.click(zoom, egui::PointerButton::Primary);
        p.menu_click("Zoom to Mission");
        assert!((p.settings.fp_map.lat - 12.05).abs() < 0.001);
        assert!((p.settings.fp_map.lng - 79.05).abs() < 0.001);
        assert!(p.settings.fp_map.zoom < 16.0);
        p.click(zoom, egui::PointerButton::Primary);
        p.menu_click("Zoom to Home");
        assert_eq!((p.settings.fp_map.lat, p.settings.fp_map.lng), (12.0, 79.0));
        assert_eq!(p.settings.fp_map.zoom, 17.0);
        p.click(zoom, egui::PointerButton::Primary);
        p.menu_click("Zoom to Vehicle");
        assert_eq!((p.settings.fp_map.lat, p.settings.fp_map.lng), (12.0, 79.0));
        assert_eq!(p.view.items.len(), 1);
    }

    #[test]
    fn rotate_prompt_rejects_nan_and_normalizes_heading() {
        let mut p = Planner::new();
        for (input, expected) in [
            ("-90", 270.0),
            ("NaN", 270.0),
            ("inf", 270.0),
            ("450", 90.0),
        ] {
            p.view.menu_prompt = Some(super::super::menu::MenuPrompt::RotateMap(input.into()));
            p.frame(Vec::new());
            p.frame(Vec::new());
            p.menu_click("OK");
            assert_eq!(p.settings.fp_map.bearing, expected);
        }
    }
}
