//! Port of `Warnings/WarningControl.cs`
//!
//! One row of the Warning Manager (a 850x27 UserControl) editing one `CustomWarning`. The C#
//! control holds the warning object itself; here it holds its place in
//! `WarningEngine.warnings` (the top-level index and how many `Child` links down), and every
//! edit is applied to that object at once, as the C# event handlers do. Control events fire
//! only when a value really changes (WinForms' Text / Value / Checked setters).

#![allow(non_snake_case)]

use egui::{Pos2, Rect, Vec2};
use mp_core::dotnet::{decimal_from_double, decimal_to_double, Decimal, Exception};
use mp_core::extlibs::utilities::warnings::{Conditional, CustomWarning, WarningColors, WarningEngine, WarningType};

use crate::extlibs::simplegrid::simplegridui_designer::{numeric_updown, NumericUpDown};
use crate::theme;

/// `WarningControl.Size`
pub const SIZE: Vec2 = Vec2::new(850.0, 27.0);

/// Where a control's `custwarning` lives: `WarningEngine.warnings[index]`, then `depth`
/// `Child` links.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WarningPath {
    pub index: usize,
    pub depth: usize,
}

/// The warning at `path` in the engine list.
pub fn node_mut(list: &mut [CustomWarning], path: WarningPath) -> Option<&mut CustomWarning> {
    let mut node = list.get_mut(path.index)?;
    for _ in 0..path.depth {
        node = node.Child.as_deref_mut()?;
    }
    Some(node)
}

/// What a row asks of its form.
pub enum RowEvent {
    /// `ReloadList`
    ReloadList,
    /// an exception out of an event handler
    Error(String),
}

/// `public class WarningControl : UserControl`
pub struct WarningControl {
    pub path: WarningPath,
    /// `Location`
    pub location: Pos2,
    CMB_Source_items: Vec<String>,
    CMB_Source: String,
    CMB_condition: String,
    NUM_warning: NumericUpDown,
    NUM_repeattime: NumericUpDown,
    /// `TXT_warningtext.Text` (public in the C#: the manager hides it on child rows)
    pub TXT_warningtext: String,
    pub TXT_warningtext_Visible: bool,
    TXT_warningtext_Enabled: bool,
    NUM_repeattime_Enabled: bool,
    but_addchild_Enabled: bool,
    CMB_color_Enabled: bool,
    CB_type: bool,
    CMB_color: String,
}

impl WarningControl {
    /// `InitializeComponent()`: the designer values.
    fn initialize_component(path: WarningPath, location: Pos2) -> Self {
        let mut NUM_warning = NumericUpDown::new();
        NUM_warning.decimal_places = 2;
        NUM_warning.maximum = Decimal::from(99999);
        NUM_warning.minimum = Decimal::from(-99999);
        let mut NUM_repeattime = NumericUpDown::new();
        NUM_repeattime.value = Decimal::from(10);
        WarningControl {
            path,
            location,
            CMB_Source_items: Vec::new(),
            CMB_Source: "gpsstat".into(),
            CMB_condition: "EQ".into(),
            NUM_warning,
            NUM_repeattime,
            TXT_warningtext: "WARNING: {name} is {value}".into(),
            TXT_warningtext_Visible: true,
            TXT_warningtext_Enabled: true,
            NUM_repeattime_Enabled: true,
            but_addchild_Enabled: true,
            CMB_color_Enabled: true,
            CB_type: false,
            CMB_color: "NoColor".into(),
        }
    }

    /// `new WarningControl(CustomWarning item)`: binds the lists, takes the item and shows it.
    pub fn new(path: WarningPath, location: Pos2) -> Result<Self, Exception> {
        let mut c = Self::initialize_component(path, location);
        WarningEngine::warnings(|list| -> Result<(), Exception> {
            let item = node_mut(list, path).ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
            // CustomWarning.defaultsrc = MainV2.comPort.MAV.cs (set at startup)
            let name = item.Name().to_string();
            item.SetField(&name)?;
            // DataSource binding selects the first item; custwarning is still null, so the
            // SelectedIndexChanged handlers do nothing
            c.CMB_condition = Conditional::get_names()[0].to_string();
            c.CMB_color = WarningColors::get_names()[0].to_string();
            c.CMB_Source_items = item.GetOptions()?;
            c.CMB_Source = c.CMB_Source_items.first().cloned().unwrap_or_default();
            c.updateDisplay(item)
        })?;
        Ok(c)
    }

    /// `updateDisplay()`
    fn updateDisplay(&mut self, item: &mut CustomWarning) -> Result<(), Exception> {
        let cond = item.ConditionType.to_string();
        if self.CMB_condition != cond {
            self.CMB_condition = cond;
            self.CMB_condition_SelectedIndexChanged(item);
        }
        let name = item.Name().to_string();
        if self.CMB_Source != name {
            self.CMB_Source = name;
            self.CMB_Source_SelectedIndexChanged(item)?;
        }
        if self.NUM_warning.set_value(decimal_from_double(item.Warning)?)? {
            self.NUM_warning_ValueChanged(item);
        }
        if item.type_ == WarningType::SpeakAndText {
            // Fire the checkedchange event to enable/disable fields
            if self.CB_type {
                self.CB_type = false;
                self.CB_type_CheckedChanged(item)?;
            }
            self.CB_type_CheckedChanged(item)?;
            if self.NUM_repeattime.set_value(Decimal::from(item.RepeatTime))? {
                self.NUM_repeattime_ValueChanged(item);
            }
            let text = item.Text().unwrap_or("").to_string();
            self.set_txt(item, text);
            self.set_color_text(item, "NoColor".into());
        } else {
            // Fire the checkedchange event to enable/disable fields
            if !self.CB_type {
                self.CB_type = true;
                self.CB_type_CheckedChanged(item)?;
            }
            self.CB_type_CheckedChanged(item)?;
            // CMB_color.Text = null leaves the text empty
            let color = item.color.clone().unwrap_or_default();
            self.set_color_text(item, color);
        }
        Ok(())
    }

    fn set_txt(&mut self, item: &mut CustomWarning, text: String) {
        if self.TXT_warningtext != text {
            self.TXT_warningtext = text;
            self.TXT_warningtext_TextChanged(item);
        }
    }

    fn set_color_text(&mut self, item: &mut CustomWarning, text: String) {
        if self.CMB_color != text {
            self.CMB_color = text;
            self.cmbColor_SelectedIndexChanged(item);
        }
    }

    fn CMB_Source_SelectedIndexChanged(&mut self, item: &mut CustomWarning) -> Result<(), Exception> {
        item.SetField(&self.CMB_Source)
    }

    fn CMB_condition_SelectedIndexChanged(&mut self, item: &mut CustomWarning) {
        if let Some(c) = Conditional::parse(&self.CMB_condition) {
            item.ConditionType = c;
        }
    }

    fn cmbColor_SelectedIndexChanged(&mut self, item: &mut CustomWarning) {
        item.color = Some(self.CMB_color.clone());
    }

    fn NUM_warning_ValueChanged(&mut self, item: &mut CustomWarning) {
        item.Warning = decimal_to_double(self.NUM_warning.value);
    }

    fn TXT_warningtext_TextChanged(&mut self, item: &mut CustomWarning) {
        item.set_Text(Some(self.TXT_warningtext.clone()));
    }

    fn NUM_repeattime_ValueChanged(&mut self, item: &mut CustomWarning) {
        // (int)NUM_repeattime.Value
        item.RepeatTime = mp_core::dotnet::cast_i32(decimal_to_double(self.NUM_repeattime.value.trunc()));
    }

    /// `but_addchild_Click`: a new child replaces any existing chain.
    fn but_addchild_Click(&mut self, item: &mut CustomWarning) -> RowEvent {
        item.Child = Some(Box::new(CustomWarning::new()));
        RowEvent::ReloadList
    }

    /// `but_remove_Click`: `warnings.Remove(custwarning)`, then unlink it where it is a child.
    fn but_remove_Click(&mut self) -> RowEvent {
        let path = self.path;
        WarningEngine::warnings(|list| {
            if path.depth == 0 {
                if path.index < list.len() {
                    list.remove(path.index);
                }
            } else if let Some(parent) = node_mut(list, WarningPath { index: path.index, depth: path.depth - 1 }) {
                // removewarning: the parent's child becomes the removed one's child
                let grandchild = parent.Child.as_mut().and_then(|c| c.Child.take());
                parent.Child = grandchild;
            }
        });
        RowEvent::ReloadList
    }

    /// `CB_type_CheckedChanged`: a colouring item disables text, repeat and add-child.
    fn CB_type_CheckedChanged(&mut self, item: &mut CustomWarning) -> Result<(), Exception> {
        if self.CB_type {
            // It is a coloring item
            self.but_addchild_Enabled = false;
            self.set_txt(item, String::new());
            self.TXT_warningtext_Enabled = false;
            if self.NUM_repeattime.set_value(Decimal::ZERO)? {
                self.NUM_repeattime_ValueChanged(item);
            }
            self.NUM_repeattime_Enabled = false;
            self.CMB_color_Enabled = true;
            item.type_ = WarningType::Coloring;
        } else {
            // It is regular SpeakAndText item
            self.but_addchild_Enabled = true;
            self.set_txt(item, "WARNING: {name} is {value}".into());
            self.TXT_warningtext_Enabled = true;
            if self.NUM_repeattime.set_value(Decimal::from(10))? {
                self.NUM_repeattime_ValueChanged(item);
            }
            self.NUM_repeattime_Enabled = true;
            self.CMB_color_Enabled = false;
            item.type_ = WarningType::SpeakAndText;
        }
        Ok(())
    }

    /// Draw the row inside the panel at `panel_origin`; edits go straight to the warning.
    pub fn show(&mut self, ui: &mut egui::Ui, panel_origin: Pos2) -> Option<RowEvent> {
        let o = panel_origin + self.location.to_vec2();
        let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
        let path = self.path;
        let mut event = None;

        // cbType "QuickPanel Coloring" (3,5) 122x17
        let mut checked = self.CB_type;
        if ui.put(at(3.0, 5.0, 122.0, 17.0), egui::Checkbox::new(&mut checked, crate::i18n::tr("QuickPanel Coloring").into_owned())).changed() {
            self.CB_type = checked;
            event = self.with_item(path, |c, item| c.CB_type_CheckedChanged(item).err().map(|e| RowEvent::Error(e.to_string())));
        }
        // CMB_Source (131,3) 116x21
        if let Some(v) = combo(ui, at(131.0, 3.0, 116.0, 21.0), ("wc_src", path), &self.CMB_Source, &self.CMB_Source_items, true) {
            self.CMB_Source = v;
            event = self.with_item(path, |c, item| c.CMB_Source_SelectedIndexChanged(item).err().map(|e| RowEvent::Error(e.to_string())));
        }
        // CMB_condition (253,3) 54x21
        let conds: Vec<String> = Conditional::get_names().into_iter().map(String::from).collect();
        if let Some(v) = combo(ui, at(253.0, 3.0, 54.0, 21.0), ("wc_cond", path), &self.CMB_condition, &conds, true) {
            self.CMB_condition = v;
            self.with_item(path, |c, item| {
                c.CMB_condition_SelectedIndexChanged(item);
                None
            });
        }
        // NUM_warning (313,4) 65x20
        if numeric_updown(ui, at(313.0, 4.0, 65.0, 20.0), &mut self.NUM_warning) {
            self.with_item(path, |c, item| {
                c.NUM_warning_ValueChanged(item);
                None
            });
        }
        // cmbColor (384,3) 104x21
        let colors: Vec<String> = WarningColors::get_names().into_iter().map(String::from).collect();
        if let Some(v) = combo(ui, at(384.0, 3.0, 104.0, 21.0), ("wc_color", path), &self.CMB_color, &colors, self.CMB_color_Enabled) {
            self.CMB_color = v;
            self.with_item(path, |c, item| {
                c.cmbColor_SelectedIndexChanged(item);
                None
            });
        }
        // TXT_warningtext (494,3) 236x20
        if self.TXT_warningtext_Visible {
            let mut text = self.TXT_warningtext.clone();
            let r = ui.add_enabled_ui(self.TXT_warningtext_Enabled, |ui| ui.put(at(494.0, 3.0, 236.0, 20.0), egui::TextEdit::singleline(&mut text))).inner;
            if r.changed() && text != self.TXT_warningtext {
                self.TXT_warningtext = text;
                self.with_item(path, |c, item| {
                    c.TXT_warningtext_TextChanged(item);
                    None
                });
            }
        }
        // NUM_repeattime (745,4) 39x20
        let changed = ui.add_enabled_ui(self.NUM_repeattime_Enabled, |ui| numeric_updown(ui, at(745.0, 4.0, 39.0, 20.0), &mut self.NUM_repeattime)).inner;
        if changed {
            self.with_item(path, |c, item| {
                c.NUM_repeattime_ValueChanged(item);
                None
            });
        }
        // but_addchild "+" (791,4) 25x20
        let add = ui.add_enabled_ui(self.but_addchild_Enabled, |ui| ui.put(at(791.0, 4.0, 25.0, 20.0), |ui: &mut egui::Ui| theme::button_sized(ui, "+", Vec2::new(25.0, 20.0)))).inner;
        if add.clicked() {
            event = self.with_item(path, |c, item| Some(c.but_addchild_Click(item)));
        }
        // but_remove "-" (822,4) 25x20
        let rem = ui.put(at(822.0, 4.0, 25.0, 20.0), |ui: &mut egui::Ui| theme::button_sized(ui, "-", Vec2::new(25.0, 20.0)));
        if rem.clicked() {
            event = Some(self.but_remove_Click());
        }
        event
    }

    /// Runs a handler on this control's warning (`custwarning != null` checks included).
    fn with_item(&mut self, path: WarningPath, f: impl FnOnce(&mut Self, &mut CustomWarning) -> Option<RowEvent>) -> Option<RowEvent> {
        WarningEngine::warnings(|list| node_mut(list, path).and_then(|item| f(self, item)))
    }
}

/// A ComboBox (DropDown) at `rect`; the picked item when it changes.
fn combo(ui: &mut egui::Ui, rect: Rect, id: (&str, WarningPath), text: &str, items: &[String], enabled: bool) -> Option<String> {
    let mut pick = None;
    ui.add_enabled_ui(enabled, |ui| {
        ui.put(rect, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt((id.0, id.1.index, id.1.depth))
                .width(rect.width())
                .selected_text(text)
                .show_ui(ui, |ui| {
                    for it in items {
                        if ui.selectable_label(it == text, it).clicked() && it != text {
                            pick = Some(it.clone());
                        }
                    }
                })
                .response
        });
    });
    pick
}
