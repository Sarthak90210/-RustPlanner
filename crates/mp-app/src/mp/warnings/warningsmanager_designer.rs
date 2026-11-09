//! Port of `Warnings/WarningsManager.Designer.cs`
//!
//! Form "Warning Manager", client 872x238: the "+" and "Save" buttons, the column labels,
//! and `panel1` (anchored to all four sides, so it grows with the form) holding the rows.
//! The designer's `warningControl1` and `label4` sit in `panel1`, which the constructor's
//! `reload()` clears before the form is shown.

#![allow(non_snake_case)]

use egui::{Pos2, Rect, Vec2};

use crate::{fonts, theme};

/// `ClientSize`
pub const CLIENT_SIZE: Vec2 = Vec2::new(872.0, 238.0);
/// `this.Text`
pub const TEXT: &str = "Warning Manager";
/// `panel1.Location` and its size at the designer size (anchored on every side).
pub const PANEL1: (f32, f32, f32, f32) = (4.0, 53.0, 866.0, 175.0);

/// The labels added to the form: (text, x, y).
const LABELS: [(&str, f32, f32); 7] = [
    ("Field", 129.0, 39.0),       // label1
    ("Comparison", 258.0, 39.0),  // label2
    ("Trigger", 314.0, 39.0),     // label3
    ("Repeat Interval", 748.0, 39.0), // label5
    ("Message", 507.0, 39.0),     // label6
    ("Item type", 3.0, 39.0),     // label7
    ("Color", 398.0, 39.0),       // label8
];

/// What the designer's buttons raised.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DesignerEvent {
    /// `BUT_Add_Click`
    Add,
    /// `BUT_save_Click`
    Save,
}

/// `panel1`'s rectangle for a client size (Top|Bottom|Left|Right anchoring keeps the
/// designer margins: 4 left, 6 right, 53 top, 10 bottom).
pub fn panel1(o: Pos2, client: Vec2) -> Rect {
    let (x, y, w, h) = PANEL1;
    let right = CLIENT_SIZE.x - (x + w);
    let bottom = CLIENT_SIZE.y - (y + h);
    Rect::from_min_max(o + Vec2::new(x, y), o + Vec2::new((client.x - right).max(x), (client.y - bottom).max(y)))
}

/// The buttons and labels.
pub fn show(ui: &mut egui::Ui, o: Pos2) -> Option<DesignerEvent> {
    let p = ui.painter().clone();
    let f = fonts::ui_id(fonts::UI_SIZE);
    for (t, x, y) in LABELS {
        p.text(o + Vec2::new(x, y), egui::Align2::LEFT_TOP, crate::i18n::tr(t), f.clone(), theme::text());
    }
    let mut ev = None;
    // BUT_Add "+" (13,13) 75x23
    let r = ui.put(Rect::from_min_size(o + Vec2::new(13.0, 13.0), Vec2::new(75.0, 23.0)), |ui: &mut egui::Ui| theme::button_sized(ui, "+", Vec2::new(75.0, 23.0)));
    if r.clicked() {
        ev = Some(DesignerEvent::Add);
    }
    // BUT_save "Save" (94,13) 75x23
    let r = ui.put(Rect::from_min_size(o + Vec2::new(94.0, 13.0), Vec2::new(75.0, 23.0)), |ui: &mut egui::Ui| theme::button_sized(ui, "Save", Vec2::new(75.0, 23.0)));
    if r.clicked() {
        ev = Some(DesignerEvent::Save);
    }
    ev
}
