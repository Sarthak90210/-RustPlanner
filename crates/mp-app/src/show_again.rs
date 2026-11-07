//! Common.MessageShowAgain: a message with a "Show me again?" box, remembered in Settings as
//! SHOWAGAIN_<tag>, OK (and Cancel when asked for), and an optional [link;url;text] line.

use std::collections::BTreeMap;

use crate::theme;

/// The settings key for a prompt's tag (spaces, '+', '-' and '.' become '_').
pub fn key(tag: &str) -> String {
    format!("SHOWAGAIN_{}", tag.replace([' ', '+', '-', '.'], "_"))
}

/// False once the user unticked "Show me again?" (the prompt then answers OK by itself).
pub fn should_show(planner: &BTreeMap<String, String>, tag: &str) -> bool {
    planner.get(&key(tag)).map(|v| !v.eq_ignore_ascii_case("false")).unwrap_or(true)
}

/// The text without its [link;url;text] token, and that link.
pub fn split_link(text: &str) -> (String, Option<(String, String)>) {
    let Some(start) = text.find("[link;") else { return (text.to_string(), None) };
    let Some(end) = text[start..].find(']').map(|e| start + e) else { return (text.to_string(), None) };
    let inner = &text[start + 6..end];
    match inner.rsplit_once(';') {
        Some((url, label)) => (format!("{}{}", &text[..start], &text[end + 1..]), Some((url.to_string(), label.to_string()))),
        None => (text.to_string(), None),
    }
}

/// One MessageShowAgain prompt waiting for an answer.
#[derive(Clone, Debug, PartialEq)]
pub struct ShowAgain {
    pub title: String,
    pub text: String,
    pub cancel: bool,
    /// the tag (defaults to the title)
    pub tag: String,
}

impl ShowAgain {
    pub fn new(title: &str, text: &str, cancel: bool) -> Self {
        Self { title: title.into(), text: text.into(), cancel, tag: title.into() }
    }

    /// Draws the form; Some(true) for OK, Some(false) for Cancel (or Escape with Cancel shown).
    pub fn ui(&self, ctx: &egui::Context, planner: &mut BTreeMap<String, String>) -> Option<bool> {
        let k = key(&self.tag);
        let mut again = planner.get(&k).map(|v| !v.eq_ignore_ascii_case("false")).unwrap_or(true);
        let (body, link) = split_link(&crate::i18n::t(&self.text));
        let mut answer = None;
        egui::Modal::new(egui::Id::new(("show_again", &self.tag))).show(ctx, |ui| {
            ui.set_min_width(300.0);
            ui.set_max_width(600.0);
            ui.label(egui::RichText::new(crate::i18n::t(&self.title)).strong());
            ui.add_space(8.0);
            theme::label(ui, &body);
            if let Some((url, label)) = &link {
                ui.hyperlink_to(label, url);
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.checkbox(&mut again, crate::i18n::t("Show me again?")).changed() {
                    // chk_CheckStateChanged saves at once
                    planner.insert(k.clone(), if again { "True" } else { "False" }.into());
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if self.cancel && theme::button(ui, "Cancel").clicked() {
                        answer = Some(false);
                    }
                    if theme::button(ui, "OK").clicked() {
                        answer = Some(true);
                    }
                });
            });
        });
        // AcceptButton OK; CancelButton only when Cancel is shown
        if ctx.input(|i| i.key_pressed(egui::Key::Enter)) {
            answer = Some(true);
        } else if self.cancel && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            answer = Some(false);
        }
        answer
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_and_links_follow_messageshowagain() {
        assert_eq!(key("Zero Altitude Warning Plane"), "SHOWAGAIN_Zero_Altitude_Warning_Plane");
        assert_eq!(key("Raw Param Warning"), "SHOWAGAIN_Raw_Param_Warning");
        let mut planner = BTreeMap::new();
        assert!(should_show(&planner, "Refresh Params"));
        planner.insert("SHOWAGAIN_Refresh_Params".to_string(), "False".to_string());
        assert!(!should_show(&planner, "Refresh Params"));
        let (body, link) = split_link("Read this[link;https://x.org/a;Docs] now");
        assert_eq!(body, "Read this now");
        assert_eq!(link, Some(("https://x.org/a".into(), "Docs".into())));
    }
}
