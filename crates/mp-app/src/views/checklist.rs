//! PreFlight tab (`Controls/PreFlight/CheckListControl.cs`): checklist.xml items, each a
//! group box with the description, the live value text and a tick box that sets itself when
//! the condition holds; "Edit" opens CheckListEditor.

use egui::{Color32, Rect, Stroke, StrokeKind, Vec2};
use mp_core::state::CurrentState;

use crate::app::AppCtx;
use crate::theme;

const DEFAULT_XML: &str = include_str!("../../assets/checklistDefault.xml");
const CONDITIONS: [&str; 7] = ["NONE", "EQ", "NEQ", "GT", "GTEQ", "LT", "LTEQ"];
const COLORS: [&str; 8] = ["Green", "Red", "White", "Yellow", "Orange", "Blue", "Black", "Gray"];

#[derive(Clone, Debug)]
pub struct Item {
    pub description: String,
    pub text: String,
    pub name: String,
    pub trigger: f64,
    pub cond: String,
    pub true_color: String,
    pub false_color: String,
}

fn color(n: &str) -> Color32 {
    match n {
        "Green" => Color32::from_rgb(0, 128, 0),
        "Red" => Color32::RED,
        "Yellow" => Color32::YELLOW,
        "Orange" => Color32::from_rgb(255, 165, 0),
        "Blue" => Color32::from_rgb(0, 0, 255),
        "Black" => Color32::BLACK,
        "Gray" => Color32::GRAY,
        _ => Color32::WHITE,
    }
}

fn parse(xml: &str) -> Vec<Item> {
    let Ok(doc) = roxmltree::Document::parse(xml.trim_start_matches('\u{feff}')) else { return Vec::new() };
    doc.descendants()
        .filter(|n| n.has_tag_name("CheckListItem"))
        .map(|n| {
            let t = |tag: &str| n.children().find(|c| c.has_tag_name(tag)).and_then(|c| c.text()).unwrap_or("").to_string();
            Item {
                description: t("Description"),
                text: t("Text"),
                name: t("Name"),
                trigger: t("TriggerValue").parse().unwrap_or(0.0),
                cond: t("ConditionType"),
                true_color: t("TrueColor"),
                false_color: t("FalseColor"),
            }
        })
        .collect()
}

fn esc(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

fn to_xml(items: &[Item]) -> String {
    let mut s = String::from("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<ArrayOfCheckListItem xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n");
    for i in items {
        s.push_str(&format!(
            "  <CheckListItem>\n    <TrueColor>{}</TrueColor>\n    <FalseColor>{}</FalseColor>\n    <Description>{}</Description>\n    <Text>{}</Text>\n    <TriggerValue>{}</TriggerValue>\n    <Name>{}</Name>\n    <ConditionType>{}</ConditionType>\n  </CheckListItem>\n",
            esc(&i.true_color),
            esc(&i.false_color),
            esc(&i.description),
            esc(&i.text),
            i.trigger,
            esc(&i.name),
            esc(&i.cond)
        ));
    }
    s.push_str("</ArrayOfCheckListItem>\n");
    s
}

fn config_path() -> std::path::PathBuf {
    crate::platform::data_dir().join("checklist.xml")
}

pub struct CheckList {
    items: Vec<Item>,
    ticks: Vec<bool>,
    editing: Option<Vec<Item>>,
}

impl Default for CheckList {
    fn default() -> Self {
        let items = std::fs::read_to_string(config_path()).ok().map(|s| parse(&s)).filter(|v| !v.is_empty()).unwrap_or_else(|| parse(DEFAULT_XML));
        let n = items.len();
        Self { items, ticks: vec![false; n], editing: None }
    }
}

impl Item {
    /// CheckListItem.checkCond
    fn check(&self, v: Option<f64>) -> bool {
        let Some(v) = v else { return false };
        match self.cond.as_str() {
            "EQ" => v == self.trigger,
            "NEQ" => v != self.trigger,
            "GT" => v > self.trigger,
            "GTEQ" => v >= self.trigger,
            "LT" => v < self.trigger,
            "LTEQ" => v <= self.trigger,
            _ => false,
        }
    }
}

impl CheckList {
    pub fn ui(&mut self, ui: &mut egui::Ui, app: &mut AppCtx, cs: &CurrentState) {
        if self.ticks.len() != self.items.len() {
            self.ticks = vec![false; self.items.len()];
        }
        let o = ui.available_rect_before_wrap().min;
        let pw = ui.available_width();
        let gw = (pw * 0.94).floor();
        let p = ui.painter().clone();
        let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let mut y = 3.0;
        for (i, it) in self.items.iter().enumerate() {
            let value = if it.name == "mode" { Some(cs.mode.clone()) } else { cs.field(&it.name).map(|v| if v.fract() == 0.0 { format!("{v:.0}") } else { format!("{v:.2}") }) };
            let num = cs.field(&it.name);
            let ok = it.check(num);
            let h = 18.0;
            let gb = Rect::from_min_size(o + Vec2::new(3.0, y), Vec2::new(gw, 17.0 + h));
            p.rect_stroke(Rect::from_min_max(gb.min + Vec2::new(0.0, 4.0), gb.max), 3.0, Stroke::new(1.0, Color32::from_rgb(0xd5, 0xdf, 0xe5)), StrokeKind::Inside);
            let dw = gw * 0.6204;
            let tw = gw * 0.2444;
            p.with_clip_rect(Rect::from_min_size(gb.min + Vec2::new(5.0, 9.0), Vec2::new(dw - 5.0, h))).text(gb.min + Vec2::new(5.0, 9.0 + h / 2.0), egui::Align2::LEFT_CENTER, &it.description, f.clone(), Color32::WHITE);
            let text = it.text.replace("{value}", value.as_deref().unwrap_or(""));
            let col = if it.cond == "NONE" { color(&it.true_color) } else if ok { color(&it.true_color) } else { color(&it.false_color) };
            p.with_clip_rect(Rect::from_min_size(gb.min + Vec2::new(dw, 9.0), Vec2::new(tw, h))).text(gb.min + Vec2::new(dw, 9.0 + h / 2.0), egui::Align2::LEFT_CENTER, text, f.clone(), col);
            let cb = Rect::from_min_size(gb.min + Vec2::new(dw + tw, 7.0), Vec2::new(21.0, 21.0));
            if it.cond != "NONE" {
                self.ticks[i] = ok;
            }
            ui.put(cb, egui::Checkbox::without_text(&mut self.ticks[i]));
            y += 17.0 + h + 4.0;
        }
        let show_edit = crate::display::flag(&app.settings.planner, "displayPreFlightTabEdit");
        if show_edit {
            let br = Rect::from_min_size(o + Vec2::new(3.0, y + 2.0), Vec2::new(75.0, 23.0));
            if ui.put(br, |ui: &mut egui::Ui| theme::button_sized(ui, "Edit", br.size())).clicked() {
                self.editing = Some(self.items.clone());
            }
        }
        ui.allocate_rect(Rect::from_min_size(o, Vec2::new(pw, y + 30.0)), egui::Sense::hover());
        self.editor(ui.ctx(), cs);
    }

    /// CheckListEditor: add / remove / edit items, saved to checklist.xml.
    fn editor(&mut self, ctx: &egui::Context, cs: &CurrentState) {
        let Some(list) = self.editing.as_mut() else { return };
        let fields: Vec<&'static str> = std::iter::once("mode").chain(cs.fields().into_iter().map(|(n, _)| n)).collect();
        let mut done = None;
        egui::Window::new(crate::i18n::t("CheckList Editor")).collapsible(false).default_size([640.0, 420.0]).show(ctx, |ui| {
            let mut remove = None;
            egui::ScrollArea::vertical().max_height(360.0).show(ui, |ui| {
                egui::Grid::new("cl_edit").num_columns(8).striped(true).show(ui, |ui| {
                    for h in ["Description", "Text", "Item", "Condition", "Value", "True", "False", ""] {
                        theme::label(ui, h);
                    }
                    ui.end_row();
                    for (i, it) in list.iter_mut().enumerate() {
                        ui.add(egui::TextEdit::singleline(&mut it.description).desired_width(160.0));
                        ui.add(egui::TextEdit::singleline(&mut it.text).desired_width(90.0));
                        egui::ComboBox::from_id_salt(("cl_name", i)).width(100.0).selected_text(it.name.as_str()).show_ui(ui, |ui| {
                            for f in &fields {
                                ui.selectable_value(&mut it.name, f.to_string(), *f);
                            }
                        });
                        egui::ComboBox::from_id_salt(("cl_cond", i)).width(60.0).selected_text(it.cond.as_str()).show_ui(ui, |ui| {
                            for c in CONDITIONS {
                                ui.selectable_value(&mut it.cond, c.to_string(), c);
                            }
                        });
                        ui.add(egui::DragValue::new(&mut it.trigger).speed(0.1));
                        for (k, c) in [(0, &mut it.true_color), (1, &mut it.false_color)] {
                            egui::ComboBox::from_id_salt(("cl_col", i, k)).width(60.0).selected_text(c.as_str()).show_ui(ui, |ui| {
                                for n in COLORS {
                                    ui.selectable_value(c, n.to_string(), n);
                                }
                            });
                        }
                        if theme::button(ui, "Remove").clicked() {
                            remove = Some(i);
                        }
                        ui.end_row();
                    }
                });
            });
            if let Some(i) = remove {
                list.remove(i);
            }
            ui.horizontal(|ui| {
                if theme::button(ui, "Add").clicked() {
                    list.push(Item { description: "New item".into(), text: "{value}".into(), name: "alt".into(), trigger: 0.0, cond: "NONE".into(), true_color: "Green".into(), false_color: "Red".into() });
                }
                if theme::button(ui, "Save").clicked() {
                    done = Some(true);
                }
                if theme::button(ui, "Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        if let Some(save) = done {
            let list = self.editing.take().unwrap_or_default();
            if save {
                let _ = std::fs::write(config_path(), to_xml(&list));
                self.items = list;
            }
        }
    }
}
