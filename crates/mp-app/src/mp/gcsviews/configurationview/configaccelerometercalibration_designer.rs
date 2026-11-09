//! Port of `GCSViews/ConfigurationView/ConfigAccelerometerCalibration.Designer.cs`
//!
//! The page layout from ConfigAccelerometerCalibration.resx (479x275):
//! - label5 "Accelerometer Calibration": Microsoft Sans Serif 12pt, WhiteSmoke, at 3,8
//! - lineSeparator2: 460x2 at 7,30, Black over Gainsboro, opacity 0.6 / 0.7 / 0.1
//! - label4 at 83,45; BUT_calib_accell 102x21 at 188,72
//! - lbl_Accel_user: 460x37 at 7,96, MiddleCenter
//! - label1 at 60,133; BUT_level 102x21 at 188,168
//! - label2 at 60,220; BUT_simpleAccelCal 102x21 at 188,249

#![allow(non_snake_case)]

use egui::{Color32, Pos2, Rect, Sense, Vec2};

use super::configaccelerometercalibration::ConfigAccelerometerCalibration;
use crate::extlibs::controls::LineSeparator;
use crate::{fonts, theme};

/// `$this.Size`
pub const SIZE: Vec2 = Vec2::new(479.0, 275.0);

/// Which button was clicked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Click {
    CalibAccell,
    Level,
    SimpleAccelCal,
}

/// Draws the page at `o`; returns the clicked button.
pub fn show(ui: &mut egui::Ui, o: Pos2, c: &ConfigAccelerometerCalibration) -> Option<Click> {
    let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
    let p = ui.painter().clone();
    let f = fonts::ui_id(fonts::UI_SIZE);
    // label5: Microsoft Sans Serif 12pt (16 px), WhiteSmoke
    p.text(o + Vec2::new(3.0, 8.0), egui::Align2::LEFT_TOP, crate::i18n::t("Accelerometer Calibration"), fonts::ui_id(16.0), Color32::from_rgb(245, 245, 245));
    // lineSeparator2
    let sep = LineSeparator { PrimaryColor: Color32::BLACK, SecondaryColor: Color32::from_rgb(220, 220, 220), Opacity1: 0.6, Opacity2: 0.7, Opacity3: 0.1 };
    sep.paint(&p, r(7.0, 30.0, 460.0, LineSeparator::HEIGHT));
    // label4
    p.text(o + Vec2::new(83.0, 45.0), egui::Align2::LEFT_TOP, crate::i18n::t("Level your Autopilot to set default accelerometer Min/Max (3 axis).\nThis will ask you to place your autopilot on each edge."), f.clone(), theme::text());
    let mut click = None;
    // BUT_calib_accell
    let br = r(188.0, 72.0, 102.0, 21.0);
    let resp = ui.put(br, |ui: &mut egui::Ui| ui.add_enabled_ui(c.BUT_calib_accell_Enabled, |ui| theme::button_sized(ui, &c.BUT_calib_accell_Text, br.size())).inner);
    if resp.clicked() && c.BUT_calib_accell_Enabled {
        click = Some(Click::CalibAccell);
    }
    // lbl_Accel_user: MiddleCenter in 460x37
    let lr = r(7.0, 96.0, 460.0, 37.0);
    p.text(lr.center(), egui::Align2::CENTER_CENTER, &c.lbl_Accel_user_Text, f.clone(), theme::text());
    // label1
    p.text(o + Vec2::new(60.0, 133.0), egui::Align2::LEFT_TOP, crate::i18n::t("Level your Autopilot to set default accelerometer offsets (1 axis/AHRS trims).\nThis requires you to place your autopilot flat and level."), f.clone(), theme::text());
    // BUT_level
    let lv = r(188.0, 168.0, 102.0, 21.0);
    if ui.put(lv, |ui: &mut egui::Ui| theme::button_sized(ui, &c.BUT_level_Text, lv.size())).clicked() {
        click = Some(Click::Level);
    }
    // label2
    p.text(o + Vec2::new(60.0, 220.0), egui::Align2::LEFT_TOP, crate::i18n::t("Level your Autopilot to set default accelerometer scale factors for level flight (1 axis).\nThis requires you to place your autopilot flat and level."), f, theme::text());
    // BUT_simpleAccelCal
    let sr = r(188.0, 249.0, 102.0, 21.0);
    if ui.put(sr, |ui: &mut egui::Ui| theme::button_sized(ui, &c.BUT_simpleAccelCal_Text, sr.size())).clicked() {
        click = Some(Click::SimpleAccelCal);
    }
    ui.allocate_rect(r(0.0, 0.0, SIZE.x, SIZE.y), Sense::hover());
    click
}
