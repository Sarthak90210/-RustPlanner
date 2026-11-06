//! GimbalVideoControl: GStreamer gimbal video with Full Sized, Mini and Pop Out modes.
//! Replicates `MissionPlanner/Controls/GimbalVideoControl.cs` and `FlightData.cs:6666-6845`.

use egui::{Color32, Pos2, Rect, Sense, Stroke, Vec2};
use mp_core::link::LinkCommand;

use crate::app::AppCtx;
use crate::{fonts, theme};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum GimbalPlacement {
    #[default]
    Closed,
    FullSized,
    Mini,
    PopOut,
}

pub struct GimbalVideo {
    pub placement: GimbalPlacement,
    pub show_mini_map: bool,
    pub yaw_lock: bool,
    pub is_recording: bool,
    pub stream_url: String,
    pub gst: Option<crate::mjpeg::Mjpeg>,
    pub prompt_stream: bool,
    pub stream_input: String,
    pub drag_start: Option<Pos2>,
    pub drag_end: Option<Pos2>,
    pub shortcuts_open: bool,
    pub custom_settings_open: bool,
    pub hfov: f32,
    pub vfov: f32,
}

impl Default for GimbalVideo {
    fn default() -> Self {
        Self {
            placement: GimbalPlacement::Closed,
            show_mini_map: true,
            yaw_lock: false,
            is_recording: false,
            stream_url: String::new(),
            gst: None,
            prompt_stream: false,
            stream_input: String::new(),
            drag_start: None,
            drag_end: None,
            shortcuts_open: false,
            custom_settings_open: false,
            hfov: 60.0,
            vfov: 45.0,
        }
    }
}

impl GimbalVideo {
    pub fn set_placement(&mut self, p: GimbalPlacement, app: &mut AppCtx, ctx: &egui::Context) {
        self.placement = p;
        if p == GimbalPlacement::Closed {
            self.stop();
        } else if self.gst.is_none() {
            let saved = app.settings.planner.get("gimbal_video_stream").cloned();
            let env_pipeline = std::env::var("MP_GSTREAMER").ok();
            if let Some(pipe) = saved.filter(|s| !s.trim().is_empty()).or(env_pipeline) {
                self.start(&pipe, ctx);
            }
        }
    }

    pub fn start(&mut self, pipeline: &str, ctx: &egui::Context) {
        let data = crate::platform::data_dir();
        if let Some(lib) = crate::gstreamer::look_for_gstreamer(&data) {
            self.stream_url = pipeline.to_string();
            self.gst = Some(crate::mjpeg::Mjpeg::gstreamer(lib, pipeline, ctx));
        }
    }

    pub fn stop(&mut self) {
        self.gst = None;
    }

    /// Renders the video box within `rect`, including texture painting and context menu.
    pub fn render_video_box(&mut self, ui: &mut egui::Ui, rect: Rect, app: &mut AppCtx) -> egui::Response {
        let resp = ui.allocate_rect(rect, Sense::click_and_drag());
        let tex = self.gst.as_mut().and_then(|v| v.texture(ui.ctx()));

        if let Some(texture_id) = tex {
            // Zoom/fit into rect
            let (img_w, img_h) = (640.0, 480.0);
            let scale = (rect.width() / img_w).min(rect.height() / img_h);
            let fit_size = Vec2::new(img_w * scale, img_h * scale);
            let fit_rect = Rect::from_center_size(rect.center(), fit_size);
            ui.painter().rect_filled(rect, 0.0, Color32::BLACK);
            ui.painter().image(texture_id, fit_rect, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
        } else {
            ui.painter().rect_filled(rect, 0.0, Color32::from_rgb(20, 20, 20));
            ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, crate::i18n::t("NO VIDEO"), fonts::ui_id(16.0), Color32::from_gray(120));
        }

        // Pointer interactions for object tracking drag rectangle
        if resp.drag_started_by(egui::PointerButton::Primary) {
            self.drag_start = resp.interact_pointer_pos();
            self.drag_end = self.drag_start;
        } else if resp.dragged_by(egui::PointerButton::Primary) {
            self.drag_end = resp.interact_pointer_pos();
        } else if resp.drag_stopped() {
            self.drag_start = None;
            self.drag_end = None;
        }

        if let (Some(s), Some(e)) = (self.drag_start, self.drag_end) {
            let box_rect = Rect::from_two_pos(s, e);
            ui.painter().rect_stroke(box_rect, 0.0, Stroke::new(1.5, Color32::RED), egui::StrokeKind::Inside);
        }

        // Context Menu
        let placement = self.placement;
        let mut change_placement = None;
        let mut prompt_stream = false;
        let mut open_shortcuts = false;
        let mut open_custom = false;

        resp.context_menu(|ui| {
            if ui.button(crate::i18n::t("Video Stream")).clicked() {
                prompt_stream = true;
                ui.close();
            }
            ui.separator();
            if ui.button(crate::i18n::t("Retract")).clicked() {
                if let Some(l) = app.link {
                    l.send(LinkCommand::CommandLong { command: 205, params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Neutral")).clicked() {
                if let Some(l) = app.link {
                    l.send(LinkCommand::CommandLong { command: 205, params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] });
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Point Down")).clicked() {
                if let Some(l) = app.link {
                    l.send(LinkCommand::CommandLong { command: 205, params: [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0] });
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Point Home")).clicked() {
                if let Some(l) = app.link {
                    if let Some(h) = l.shared.read().cs.home {
                        l.send(LinkCommand::CommandLong { command: 195, params: [0.0, 0.0, 0.0, 0.0, h.lat as f32, h.lng as f32, h.alt as f32] });
                    }
                }
                ui.close();
            }
            if ui.checkbox(&mut self.yaw_lock, crate::i18n::t("Yaw Lock")).clicked() {
                ui.close();
            }
            ui.separator();
            if ui.button(crate::i18n::t("Take Picture")).clicked() {
                if let Some(l) = app.link {
                    l.send(LinkCommand::CommandLong { command: 2000, params: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0] });
                }
                ui.close();
            }
            if ui.button(crate::i18n::t("Start Recording")).clicked() {
                if let Some(l) = app.link {
                    l.send(LinkCommand::CommandLong { command: 2500, params: [0.0, f32::NAN, f32::NAN, f32::NAN, 0.0, 0.0, 0.0] });
                }
                self.is_recording = true;
                ui.close();
            }
            if ui.button(crate::i18n::t("Stop Recording")).clicked() {
                if let Some(l) = app.link {
                    l.send(LinkCommand::CommandLong { command: 2501, params: [0.0, f32::NAN, f32::NAN, f32::NAN, 0.0, 0.0, 0.0] });
                }
                self.is_recording = false;
                ui.close();
            }
            ui.separator();
            if ui.button(crate::i18n::t("Shortcuts")).clicked() {
                open_shortcuts = true;
                ui.close();
            }
            if ui.button(crate::i18n::t("Custom Settings")).clicked() {
                open_custom = true;
                ui.close();
            }
            if placement == GimbalPlacement::FullSized {
                ui.separator();
                ui.checkbox(&mut self.show_mini_map, crate::i18n::t("Mini map"));
                if ui.button(crate::i18n::t("Swap with map")).clicked() {
                    change_placement = Some(GimbalPlacement::Mini);
                    ui.close();
                }
                if ui.button(crate::i18n::t("Close")).clicked() {
                    change_placement = Some(GimbalPlacement::Closed);
                    ui.close();
                }
            } else if placement == GimbalPlacement::Mini {
                ui.separator();
                if ui.button(crate::i18n::t("Swap with map")).clicked() {
                    change_placement = Some(GimbalPlacement::FullSized);
                    ui.close();
                }
                if ui.button(crate::i18n::t("Close")).clicked() {
                    change_placement = Some(GimbalPlacement::Closed);
                    ui.close();
                }
            }
        });

        if prompt_stream {
            self.prompt_stream = true;
            self.stream_input = self.stream_url.clone();
        }
        if open_shortcuts {
            self.shortcuts_open = true;
        }
        if open_custom {
            self.custom_settings_open = true;
        }
        if let Some(next) = change_placement {
            self.set_placement(next, app, ui.ctx());
        }

        resp
    }

    /// Prompt modals and extra windows.
    pub fn extra_ui(&mut self, ctx: &egui::Context, app: &mut AppCtx) {
        if self.prompt_stream {
            let mut close = false;
            let mut save = false;
            egui::Window::new(crate::i18n::t("Video Stream"))
                .collapsible(false)
                .resizable(false)
                .fixed_size(Vec2::new(380.0, 110.0))
                .show(ctx, |ui| {
                    theme::label(ui, "GStreamer pipeline or video stream URL:");
                    ui.add(egui::TextEdit::singleline(&mut self.stream_input).desired_width(360.0));
                    ui.horizontal(|ui| {
                        if theme::button(ui, "OK").clicked() {
                            save = true;
                            close = true;
                        }
                        if theme::button(ui, "Cancel").clicked() {
                            close = true;
                        }
                    });
                });
            if save {
                let pipe = self.stream_input.clone();
                app.settings.planner.insert("gimbal_video_stream".into(), pipe.clone());
                self.start(&pipe, ctx);
            }
            if close {
                self.prompt_stream = false;
            }
        }

        if self.shortcuts_open {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Shortcuts"))
                .open(&mut open)
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    egui::Grid::new("gimbal_shortcuts").num_columns(2).striped(true).show(ui, |ui| {
                        ui.label(crate::i18n::t("Slew Gimbal")); ui.label("Arrow Keys"); ui.end_row();
                        ui.label(crate::i18n::t("Fast Modifier")); ui.label("Shift"); ui.end_row();
                        ui.label(crate::i18n::t("Slow Modifier")); ui.label("Ctrl"); ui.end_row();
                        ui.label(crate::i18n::t("Take Picture")); ui.label("Space"); ui.end_row();
                        ui.label(crate::i18n::t("Toggle Recording")); ui.label("R"); ui.end_row();
                        ui.label(crate::i18n::t("Neutral")); ui.label("N"); ui.end_row();
                        ui.label(crate::i18n::t("Point Down")); ui.label("D"); ui.end_row();
                        ui.label(crate::i18n::t("Point Home")); ui.label("H"); ui.end_row();
                        ui.label(crate::i18n::t("Retract")); ui.label("X"); ui.end_row();
                    });
                });
            if !open {
                self.shortcuts_open = false;
            }
        }

        if self.custom_settings_open {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Custom Settings"))
                .open(&mut open)
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        theme::label(ui, "Camera HFOV:");
                        ui.add(egui::DragValue::new(&mut self.hfov).speed(0.5).range(10.0..=120.0));
                    });
                    ui.horizontal(|ui| {
                        theme::label(ui, "Camera VFOV:");
                        ui.add(egui::DragValue::new(&mut self.vfov).speed(0.5).range(10.0..=120.0));
                    });
                });
            if !open {
                self.custom_settings_open = false;
            }
        }

        // Pop Out mode window
        if self.placement == GimbalPlacement::PopOut {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Gimbal Control"))
                .id(egui::Id::new("gimbal_popout_window"))
                .open(&mut open)
                .default_size(Vec2::new(600.0, 400.0))
                .show(ctx, |ui| {
                    let r = ui.available_rect_before_wrap();
                    self.render_video_box(ui, r, app);
                });
            if !open {
                self.set_placement(GimbalPlacement::Closed, app, ctx);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gimbal_placement_transitions_and_defaults() {
        let mut g = GimbalVideo::default();
        assert_eq!(g.placement, GimbalPlacement::Closed);
        assert!(g.show_mini_map);
        assert!(!g.yaw_lock);
        assert!(!g.is_recording);
        assert!(g.gst.is_none());

        g.placement = GimbalPlacement::FullSized;
        assert_eq!(g.placement, GimbalPlacement::FullSized);

        g.placement = GimbalPlacement::Mini;
        assert_eq!(g.placement, GimbalPlacement::Mini);

        g.placement = GimbalPlacement::PopOut;
        assert_eq!(g.placement, GimbalPlacement::PopOut);

        g.stop();
        assert!(g.gst.is_none());
    }
}

