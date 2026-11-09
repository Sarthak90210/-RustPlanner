//! Port of `ExtLibs/Controls/InputBox.cs`
//!
//! The modal text prompt: a FixedSingle form (ClientSize 396 wide) titled `title`, the prompt
//! label at (9, 20) wrapping at 372, the 372x20 text box (372x400 when multiline) 10 px
//! below it, then OK (228) and Cancel (309) buttons 10 px below that; Enter is OK and Escape
//! is Cancel. The C# offers autocomplete from Settings "InputBox" + title + prompt (letters
//! and digits only), but its filter keeps only empty entries; on OK the history gains the
//! answer (SetList: ';'-joined, URL-encoded, distinct).

#![allow(non_snake_case)]

use std::collections::BTreeMap;

use egui::{Pos2, Rect, Vec2};
use mp_core::extlibs::utilities::custommessagebox::DialogResult;

use crate::{fonts, theme};

/// `string.CleanString()`: letters and digits only.
pub fn CleanString(s: &str) -> String {
    s.chars().filter(|c| c.is_alphanumeric()).collect()
}

/// `Settings.GetList(key)`
fn get_list(settings: &BTreeMap<String, String>, key: &str) -> Vec<String> {
    let Some(v) = settings.get(key) else { return Vec::new() };
    let mut out: Vec<String> = Vec::new();
    for a in v.split(';').map(mp_core::dotnet::url_decode) {
        if !out.contains(&a) {
            out.push(a);
        }
    }
    out
}

/// `Settings.SetList(key, list)`
fn set_list(settings: &mut BTreeMap<String, String>, key: &str, list: &[String]) {
    if list.is_empty() {
        return;
    }
    let mut encoded: Vec<String> = Vec::new();
    for a in list.iter().map(|a| mp_core::dotnet::url_encode(a)) {
        if !encoded.contains(&a) {
            encoded.push(a);
        }
    }
    settings.insert(key.to_string(), encoded.join(";"));
}

/// One open `InputBox.Show(title, promptText, ref value, password, multiline)`.
#[derive(Debug, Clone)]
pub struct InputBox {
    pub title: String,
    pub promptText: String,
    pub password: bool,
    pub multiline: bool,
    /// `textBox.Text`
    pub text: String,
    /// `textBox.AutoCompleteCustomSource`
    autocomplete: Vec<String>,
    id: egui::Id,
}

impl InputBox {
    /// `ShowUI` up to `form.ShowDialog()`: the box with `value`, and the autocomplete list.
    pub fn new(settings: &BTreeMap<String, String>, title: &str, promptText: &str, value: &str, password: bool, multiline: bool) -> Self {
        let mut autocomplete = Vec::new();
        if !password && !title.is_empty() {
            let key = format!("InputBox{}{}", CleanString(title), CleanString(promptText));
            autocomplete = get_list(settings, &key).into_iter().filter(|a| a.is_empty()).collect();
        }
        InputBox {
            title: title.to_string(),
            promptText: promptText.to_string(),
            password,
            multiline,
            text: value.to_string(),
            autocomplete,
            id: egui::Id::new(("InputBox", title.to_string(), promptText.to_string())),
        }
    }

    /// Shows the form; `Some(result)` once it is closed. On OK the answer is in `text` and the
    /// autocomplete history is saved.
    pub fn show(&mut self, ctx: &egui::Context, settings: &mut BTreeMap<String, String>) -> Option<DialogResult> {
        let mut result = None;
        let f = fonts::ui_id(fonts::UI_SIZE);
        let screen = ctx.content_rect();
        // label: AutoSize, MaximumSize 372 wide
        let galley = ctx.fonts_mut(|fo| fo.layout(self.promptText.clone(), f.clone(), theme::text(), 372.0));
        let label_h = galley.size().y.ceil().max(13.0);
        let y_margin = 10.0;
        let text_y = 20.0 + label_h + y_margin;
        let text_h = if self.multiline { 400.0 } else { 20.0 };
        let button_y = text_y + text_h + y_margin;
        let client = Vec2::new(396.0, button_y + 23.0 + y_margin);
        let frame = egui::Frame::window(&ctx.global_style()).fill(theme::bg()).inner_margin(0.0);
        // ShowDialog: nothing behind the form takes input
        egui::Area::new(self.id.with("modal_block")).order(egui::Order::Middle).fixed_pos(screen.min).interactable(true).show(ctx, |ui| {
            ui.allocate_rect(screen, egui::Sense::click_and_drag());
        });
        egui::Window::new(self.title.clone())
            .id(self.id)
            .order(egui::Order::Foreground)
            .collapsible(false)
            .resizable(false)
            .fixed_size(client)
            // StartPosition CenterScreen
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(screen.center())
            .frame(frame)
            .show(ctx, |ui| {
                let (full, _) = ui.allocate_exact_size(client, egui::Sense::hover());
                let o = full.min;
                let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
                ui.painter().galley(o + Vec2::new(9.0, 20.0), galley.clone(), theme::text());
                let mut edit = if self.multiline { egui::TextEdit::multiline(&mut self.text) } else { egui::TextEdit::singleline(&mut self.text) };
                if self.password {
                    edit = edit.password(true);
                }
                let r = ui.put(at(12.0, text_y, 372.0, text_h), edit);
                if !r.has_focus() && ctx.memory(|m| m.focused().is_none()) {
                    r.request_focus();
                }
                let ok = ui.put(at(228.0, button_y, 75.0, 23.0), |ui: &mut egui::Ui| theme::button_sized(ui, "OK", Vec2::new(75.0, 23.0)));
                let cancel = ui.put(at(309.0, button_y, 75.0, 23.0), |ui: &mut egui::Ui| theme::button_sized(ui, "Cancel", Vec2::new(75.0, 23.0)));
                if ok.clicked() {
                    result = Some(DialogResult::OK);
                }
                if cancel.clicked() {
                    result = Some(DialogResult::Cancel);
                }
            });
        // AcceptButton / CancelButton (a multiline box keeps Enter for new lines)
        if result.is_none() {
            if !self.multiline && ctx.input(|i| i.key_pressed(egui::Key::Enter)) {
                result = Some(DialogResult::OK);
            } else if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
                result = Some(DialogResult::Cancel);
            }
        }
        if result == Some(DialogResult::OK) {
            self.autocomplete.push(self.text.clone());
            let key = format!("InputBox{}{}", CleanString(&self.title), CleanString(&self.promptText));
            set_list(settings, &key, &self.autocomplete);
        }
        let _ = Pos2::ZERO;
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn history_keeps_only_empty_entries_then_adds_the_answer() {
        let mut s = BTreeMap::new();
        s.insert("InputBoxRadiusRadiusm".to_string(), "50;;100".to_string());
        let b = InputBox::new(&s, "Radius", "Radius (m)", "50", false, false);
        assert_eq!(b.autocomplete, vec![String::new()], "the C# filter keeps only empty entries");
        let mut l = b.autocomplete.clone();
        l.push("75".into());
        set_list(&mut s, "InputBoxRadiusRadiusm", &l);
        assert_eq!(s["InputBoxRadiusRadiusm"], ";75");
        assert_eq!(CleanString("Radius (m)"), "Radiusm");
    }
}
