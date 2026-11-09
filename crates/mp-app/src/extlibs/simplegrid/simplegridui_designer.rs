//! Port of `ExtLibs/SimpleGrid/SimpleGridUI.Designer.cs`
//!
//! (with the layout from
//! `SimpleGridUI.resx`).
//!
//! Form "Grid", client 1008x537:
//! - `map` docked Fill: 755x457 at 0,0 (MinZoom 2, MaxZoom 24, Zoom 3)
//! - `groupBox5` "Stats" docked Bottom: 755x80 at 0,457
//! - `tabControl1` docked Right: 253x537 at 755,0, one page `tabSimple` "Simple" (245x511 at 4,22)
//!   holding `groupBox6` "Simple Options" (236x262 at 6,6) and `groupBox4` "Display"
//!   (230x87 at 9,334)
//!
//! Every NumericUpDown's `ValueChanged`, the combo's `SelectedIndexChanged` and every
//! checkbox's `CheckedChanged` are wired to `domainUpDown1_ValueChanged`; `BUT_Accept.Click`
//! to `BUT_Accept_Click`; the form's `Load` to `GridUI_Load` and `Resize` to `GridUI_Resize`.

#![allow(non_snake_case)]

use egui::{Align, Color32, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use mp_core::dotnet::{current_culture, decimal_format_fixed, decimal_parse, Decimal, Exception};

use crate::views::tuning::group_box;
use crate::{fonts, theme};

/// `ClientSize = new Size(1008, 537)`
pub const CLIENT_SIZE: Vec2 = Vec2::new(1008.0, 537.0);
/// `map`: docked Fill between the Stats group and the tab control (755x457 at the designer
/// size).
pub const MAP_RECT: (f32, f32, f32, f32) = (0.0, 0.0, 755.0, 457.0);
/// `tabControl1.Width` (docked Right)
pub const TAB_WIDTH: f32 = 253.0;
/// `groupBox5.Height` (docked Bottom, beside the tab control)
pub const STATS_HEIGHT: f32 = 80.0;

/// The docked layout for a client size: (map, stats, tab control) rectangles.
pub fn layout(o: Pos2, client: Vec2) -> (Rect, Rect, Rect) {
    let left_w = (client.x - TAB_WIDTH).max(0.0);
    let map_h = (client.y - STATS_HEIGHT).max(0.0);
    (
        Rect::from_min_size(o, Vec2::new(left_w, map_h)),
        Rect::from_min_size(o + Vec2::new(0.0, map_h), Vec2::new(left_w, STATS_HEIGHT)),
        Rect::from_min_size(o + Vec2::new(left_w, 0.0), Vec2::new(TAB_WIDTH, client.y)),
    )
}
/// `map.MinZoom` / `map.MaxZoom` / `map.Zoom`
pub const MAP_MIN_ZOOM: i32 = 2;
pub const MAP_MAX_ZOOM: i32 = 24;
pub const MAP_ZOOM: f64 = 3.0;

/// `System.Windows.Forms.NumericUpDown` as the form uses it: the value keeps its decimal
/// scale (`Value.ToString()` of a typed "50.00" is "50.00", of the designer's 50 is "50").
#[derive(Clone, Debug)]
pub struct NumericUpDown {
    pub value: Decimal,
    pub minimum: Decimal,
    pub maximum: Decimal,
    pub increment: Decimal,
    pub decimal_places: u32,
}

impl NumericUpDown {
    /// `new NumericUpDown()`: Minimum 0, Maximum 100, Increment 1, DecimalPlaces 0, Value 0.
    fn new() -> Self {
        NumericUpDown { value: Decimal::ZERO, minimum: Decimal::ZERO, maximum: Decimal::from(100), increment: Decimal::ONE, decimal_places: 0 }
    }

    /// The `Value` setter: an out-of-range value throws; `ValueChanged` fires (returns true)
    /// only when the value differs (`50 == 50.00`, so that keeps the old scale).
    pub fn set_value(&mut self, value: Decimal) -> Result<bool, Exception> {
        if value == self.value {
            return Ok(false);
        }
        if value < self.minimum || value > self.maximum {
            return Err(Exception::ArgumentOutOfRange(format!(
                "Value of '{}' is not valid for 'Value'. 'Value' should be between 'Minimum' and 'Maximum'.\r\nParameter name: Value",
                mp_core::dotnet::decimal_to_string(value)
            )));
        }
        self.value = value;
        Ok(true)
    }

    /// `Constrain(value)`
    fn constrain(&self, value: Decimal) -> Decimal {
        if value < self.minimum {
            self.minimum
        } else if value > self.maximum {
            self.maximum
        } else {
            value
        }
    }

    /// `UpdateEditText`: `Value.ToString("F" + DecimalPlaces, CurrentCulture)`.
    pub fn text(&self) -> String {
        decimal_format_fixed(self.value, 1, self.decimal_places, current_culture())
    }

    /// `UpButton` / `DownButton`: one increment, clamped.
    fn step(&mut self, up: bool) -> bool {
        let v = if up {
            self.value.checked_add(self.increment).map(|v| if v > self.maximum { self.maximum } else { v }).unwrap_or(self.maximum)
        } else {
            self.value.checked_sub(self.increment).map(|v| if v < self.minimum { self.minimum } else { v }).unwrap_or(self.minimum)
        };
        self.set_value(v).unwrap_or(false)
    }

    /// `ParseEditText`: `Value = Constrain(decimal.Parse(Text, CurrentCulture))`; an empty
    /// text, a lone "-" or a parse failure leaves the value as it was.
    fn parse_edit_text(&mut self, text: &str) -> bool {
        if text.is_empty() || text == "-" {
            return false;
        }
        match decimal_parse(text) {
            Ok(v) => {
                let v = self.constrain(v);
                self.set_value(v).unwrap_or(false)
            }
            Err(_) => false,
        }
    }
}

/// The NumericUpDown control drawn at `rect`; true when `ValueChanged` fires. The edit text
/// is committed on Enter or focus loss; the arrows and the mouse wheel (`OnMouseWheel`:
/// `SystemInformation.MouseWheelScrollLines` = 3 steps a notch) step by `Increment`.
pub fn numeric_updown(ui: &mut egui::Ui, rect: Rect, n: &mut NumericUpDown) -> bool {
    let id = ui.id().with(("sg_nud", (rect.left() as i32, rect.top() as i32)));
    let p = ui.painter().clone();
    p.rect_filled(rect, 0.0, theme::control_bg());
    p.rect_stroke(rect, 0.0, Stroke::new(1.0, Color32::from_gray(0x80)), StrokeKind::Inside);
    let arrows = Rect::from_min_max(Pos2::new(rect.right() - 17.0, rect.top() + 1.0), rect.max - Vec2::new(1.0, 1.0));
    let up = Rect::from_min_max(arrows.min, Pos2::new(arrows.right(), arrows.center().y));
    let down = Rect::from_min_max(Pos2::new(arrows.left(), arrows.center().y), arrows.max);
    let mut buf: String = ui.data(|d| d.get_temp::<String>(id)).unwrap_or_else(|| n.text());
    let text_rect = Rect::from_min_max(rect.min + Vec2::new(2.0, 1.0), Pos2::new(arrows.left() - 1.0, rect.bottom() - 1.0));
    let te = ui.put(text_rect, egui::TextEdit::singleline(&mut buf).frame(egui::Frame::NONE).text_color(theme::text()).vertical_align(Align::Center));
    let mut changed = false;
    let mut user_edit = false;
    if te.has_focus() {
        user_edit = buf != n.text();
        ui.data_mut(|d| d.insert_temp(id, buf.clone()));
        let notches: f32 = ui.input(|i| {
            i.raw.events.iter().map(|e| if let egui::Event::MouseWheel { delta, .. } = e { delta.y } else { 0.0 }).sum()
        });
        if notches != 0.0 {
            if user_edit {
                changed |= n.parse_edit_text(buf.trim());
            }
            for _ in 0..3 {
                changed |= n.step(notches > 0.0);
            }
            ui.data_mut(|d| d.insert_temp(id, n.text()));
        }
    } else {
        ui.data_mut(|d| d.remove::<String>(id));
    }
    if te.lost_focus() {
        if buf != n.text() {
            changed |= n.parse_edit_text(buf.trim());
        }
        ui.data_mut(|d| d.remove::<String>(id));
    }
    for (r, dir) in [(up, true), (down, false)] {
        let resp = ui.interact(r, id.with(dir), Sense::click());
        let fill = if resp.is_pointer_button_down_on() { theme::mouse_down() } else if resp.hovered() { theme::mouse_over() } else { theme::bg() };
        p.rect_filled(r, 0.0, fill);
        let c = r.center();
        let tri = if dir { vec![c + Vec2::new(-3.0, 1.5), c + Vec2::new(3.0, 1.5), c + Vec2::new(0.0, -1.5)] } else { vec![c + Vec2::new(-3.0, -1.5), c + Vec2::new(3.0, -1.5), c + Vec2::new(0.0, 1.5)] };
        p.add(egui::Shape::convex_polygon(tri, theme::text(), Stroke::NONE));
        if resp.clicked() {
            // UpButton: `if (UserEdit) ParseEditText();` first
            if user_edit {
                changed |= n.parse_edit_text(buf.trim());
            }
            changed |= n.step(dir);
            ui.data_mut(|d| d.remove::<String>(id));
        }
    }
    changed
}

/// `CMB_startfrom`: a ComboBox bound to a string list.
#[derive(Clone, Debug, Default)]
pub struct ComboBox {
    pub items: Vec<String>,
    pub selected_index: i32,
}

impl ComboBox {
    /// `Text`
    pub fn text(&self) -> String {
        usize::try_from(self.selected_index).ok().and_then(|i| self.items.get(i)).cloned().unwrap_or_default()
    }
}

/// The controls `InitializeComponent` creates, with their designer values.
pub struct Designer {
    /// `this.Text`
    pub text: &'static str,
    pub NUM_altitude: NumericUpDown,
    pub NUM_angle: NumericUpDown,
    pub NUM_Distance: NumericUpDown,
    pub NUM_spacing: NumericUpDown,
    pub NUM_overshoot: NumericUpDown,
    pub NUM_overshoot2: NumericUpDown,
    pub NUM_UpDownFlySpeed: NumericUpDown,
    pub CMB_startfrom: ComboBox,
    pub chk_boundary: bool,
    pub chk_markers: bool,
    pub chk_grid: bool,
    pub CHK_internals: bool,
    pub lbl_area: String,
    pub lbl_distance: String,
    pub lbl_strips: String,
    pub lbl_distbetweenlines: String,
}

/// What a control raised this frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DesignerEvent {
    /// `domainUpDown1_ValueChanged`
    ValueChanged,
    /// `BUT_Accept_Click`
    AcceptClick,
}

/// `InitializeComponent()`
pub fn InitializeComponent() -> Designer {
    let d = Decimal::from;
    let mut NUM_altitude = NumericUpDown::new();
    NUM_altitude.increment = d(10);
    NUM_altitude.maximum = d(9999);
    NUM_altitude.minimum = d(5);
    NUM_altitude.value = d(100);
    let mut NUM_angle = NumericUpDown::new();
    NUM_angle.maximum = d(360);
    let mut NUM_Distance = NumericUpDown::new();
    NUM_Distance.decimal_places = 2;
    NUM_Distance.maximum = d(99999);
    NUM_Distance.minimum = Decimal::new(1, 1);
    NUM_Distance.value = d(50);
    let mut NUM_spacing = NUM_Distance.clone();
    NUM_spacing.value = d(50);
    let mut NUM_overshoot = NumericUpDown::new();
    NUM_overshoot.maximum = d(9999);
    let NUM_overshoot2 = NUM_overshoot.clone();
    let mut NUM_UpDownFlySpeed = NumericUpDown::new();
    NUM_UpDownFlySpeed.maximum = d(360);
    NUM_UpDownFlySpeed.value = d(5);
    Designer {
        text: "Grid",
        NUM_altitude,
        NUM_angle,
        NUM_Distance,
        NUM_spacing,
        NUM_overshoot,
        NUM_overshoot2,
        NUM_UpDownFlySpeed,
        CMB_startfrom: ComboBox { items: Vec::new(), selected_index: -1 },
        chk_boundary: true,
        chk_markers: true,
        chk_grid: true,
        CHK_internals: true,
        lbl_area: "Distance: ".into(),
        lbl_distance: "Distance: ".into(),
        lbl_strips: "Distance: ".into(),
        lbl_distbetweenlines: "Distance: ".into(),
    }
}

impl Designer {
    /// `groupBox5` "Stats": the four caption/value label pairs.
    pub fn paint_stats(&self, p: &egui::Painter, o: Pos2, client: Vec2) {
        let g = layout(o, client).1;
        group_box(p, g, "Stats");
        let f = fonts::ui_id(fonts::UI_SIZE);
        let t = |x: f32, y: f32, s: &str| {
            p.text(g.min + Vec2::new(x, y), egui::Align2::LEFT_TOP, crate::i18n::tr(s), f.clone(), theme::text());
        };
        // label22/lbl_area, label23/lbl_distance, label33/lbl_strips, label25/lbl_distbetweenlines
        t(13.0, 20.0, "Area: ");
        t(154.0, 20.0, &self.lbl_area);
        t(13.0, 33.0, "Distance: ");
        t(154.0, 33.0, &self.lbl_distance);
        t(12.0, 46.0, "No of Strips:");
        t(153.0, 46.0, &self.lbl_strips);
        t(12.0, 63.0, "Dist between lines:");
        t(153.0, 63.0, &self.lbl_distbetweenlines);
    }

    /// `tabControl1` with `tabSimple`: the options, the Accept button and the Display group.
    pub fn show_tabs(&mut self, ui: &mut egui::Ui, o: Pos2, p: &egui::Painter, client: Vec2) -> Vec<DesignerEvent> {
        let mut events = Vec::new();
        let tc = layout(o, client).2;
        let mut tab = 0usize;
        ui.scope_builder(egui::UiBuilder::new().max_rect(Rect::from_min_size(tc.min, Vec2::new(253.0, 23.0))), |ui| {
            theme::tab_strip(ui, "simplegrid_tabs", &["Simple"], &mut tab);
        });
        let page = Rect::from_min_size(tc.min + Vec2::new(4.0, 22.0), Vec2::new(245.0, (tc.height() - 26.0).max(0.0)));
        p.rect_stroke(page.expand(1.0), 0.0, Stroke::new(1.0, Color32::from_gray(0xd9)), StrokeKind::Inside);
        let po = page.min;
        let f = fonts::ui_id(fonts::UI_SIZE);
        let at = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(po + Vec2::new(x, y), Vec2::new(w, h));

        // groupBox6 "Simple Options"
        let g6 = Vec2::new(6.0, 6.0);
        group_box(p, at(g6.x, g6.y, 236.0, 262.0), "Simple Options");
        let txt = |x: f32, y: f32, s: &str| {
            p.text(po + g6 + Vec2::new(x, y), egui::Align2::LEFT_TOP, crate::i18n::tr(s), f.clone(), theme::text());
        };
        let num_rect = |y: f32| at(g6.x + 147.0, g6.y + y, 51.0, 20.0);
        txt(11.0, 21.0, "Altitude");
        txt(11.0, 47.0, "Angle [deg]");
        txt(10.0, 73.0, "Line spacing [m]");
        txt(10.0, 99.0, "Spacing inline [m]");
        txt(11.0, 125.0, "OverShoot [m]");
        txt(11.0, 151.0, "OverShoot [m]");
        txt(11.0, 177.0, "Speed (est)");
        txt(10.0, 204.0, "StartFrom");
        for (y, n) in [
            (19.0, &mut self.NUM_altitude),
            (45.0, &mut self.NUM_angle),
            (71.0, &mut self.NUM_Distance),
            (97.0, &mut self.NUM_spacing),
            (123.0, &mut self.NUM_overshoot),
            (149.0, &mut self.NUM_overshoot2),
            (175.0, &mut self.NUM_UpDownFlySpeed),
        ] {
            if numeric_updown(ui, num_rect(y), n) {
                events.push(DesignerEvent::ValueChanged);
            }
        }
        // CMB_startfrom (106,201) 92x21
        let cmb = at(g6.x + 106.0, g6.y + 201.0, 92.0, 21.0);
        let mut pick = None;
        ui.put(cmb, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt("simplegrid_startfrom")
                .width(92.0)
                .selected_text(self.CMB_startfrom.text())
                .show_ui(ui, |ui| {
                    for (i, n) in self.CMB_startfrom.items.iter().enumerate() {
                        if ui.selectable_label(self.CMB_startfrom.selected_index == i as i32, n).clicked() {
                            pick = Some(i as i32);
                        }
                    }
                })
                .response
        });
        if let Some(i) = pick {
            if i != self.CMB_startfrom.selected_index {
                self.CMB_startfrom.selected_index = i;
                events.push(DesignerEvent::ValueChanged);
            }
        }
        // BUT_Accept (85,228) 75x23
        let r = ui.scope_builder(egui::UiBuilder::new().max_rect(at(g6.x + 85.0, g6.y + 228.0, 75.0, 23.0)), |ui| theme::button_sized(ui, "Accept", Vec2::new(75.0, 23.0))).inner;
        if r.clicked() {
            events.push(DesignerEvent::AcceptClick);
        }

        // groupBox4 "Display"
        let g4 = Vec2::new(9.0, 334.0);
        group_box(p, at(g4.x, g4.y, 230.0, 87.0), "Display");
        for (x, y, w, v, s) in [
            (8.0, 20.0, 71.0, &mut self.chk_boundary, "Boundary"),
            (8.0, 43.0, 64.0, &mut self.chk_markers, "Markers"),
            (8.0, 66.0, 45.0, &mut self.chk_grid, "Grid"),
            (87.0, 19.0, 66.0, &mut self.CHK_internals, "Internals"),
        ] {
            if ui.put(at(g4.x + x, g4.y + y, w + 20.0, 17.0), egui::Checkbox::new(v, crate::i18n::tr(s).into_owned())).changed() {
                events.push(DesignerEvent::ValueChanged);
            }
        }
        events
    }
}
