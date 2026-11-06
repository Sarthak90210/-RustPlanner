//! Port of `GCSViews/Help` (Help.resx): changelog box, update buttons, console option.

use egui::{Color32, Rect, RichText, Sense, Vec2};

use crate::{fonts, theme};

const CHANGELOG: &str = include_str!("../../assets/ChangeLog.txt");

pub fn ui(ui: &mut egui::Ui) {
    let o = ui.max_rect().min;
    let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
    // richTextBox1 (106,80 348x161), anchored on all sides
    let full = ui.max_rect();
    let rtb = Rect::from_min_max(o + Vec2::new(106.0, 80.0), egui::pos2(full.right() - 124.0, full.bottom() - 98.0).max(o + Vec2::new(454.0, 241.0)));
    ui.painter().rect_filled(rtb, 0.0, theme::control_bg());
    ui.scope_builder(egui::UiBuilder::new().max_rect(rtb.shrink(3.0)), |ui| {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            ui.label(RichText::new(CHANGELOG.trim_start_matches('\u{feff}')).font(fonts::ui_id(fonts::UI_SIZE)).color(Color32::from_rgb(0xf5, 0xf5, 0xf5)));
        });
    });
    let by = rtb.bottom() + 27.0;
    let cx = rtb.center().x;
    let up = Rect::from_min_size(egui::pos2(cx - 129.0, by), Vec2::new(123.0, 29.0));
    let beta = Rect::from_min_size(egui::pos2(cx + 0.0, by), Vec2::new(123.0, 29.0));
    if ui.put(up, |ui: &mut egui::Ui| theme::button_sized(ui, "Check for Updates", up.size())).clicked() {
        ui.ctx().open_url(egui::OpenUrl::new_tab("https://github.com/Sarthak90210/MissionPlanner/releases"));
    }
    if ui.put(beta, |ui: &mut egui::Ui| theme::button_sized(ui, "Check for BETA Updates", beta.size())).clicked() {
        ui.ctx().open_url(egui::OpenUrl::new_tab("https://github.com/Sarthak90210/MissionPlanner/actions"));
    }
    let mut console = false;
    ui.put(Rect::from_min_size(egui::pos2(cx - 87.0, by + 35.0), Vec2::new(190.0, 17.0)), egui::Checkbox::new(&mut console, crate::i18n::t("Show Console Window (restart)")));
    ui.put(
        Rect::from_min_size(egui::pos2(cx - 32.0, by + 55.0), Vec2::new(70.0, 13.0)),
        egui::Hyperlink::from_label_and_url("Change Log", "https://github.com/ArduPilot/MissionPlanner/blob/master/ChangeLog.txt"),
    );
    let _ = at;
    ui.allocate_rect(Rect::from_min_max(o, egui::pos2(rtb.right(), by + 70.0)), Sense::hover());
}
