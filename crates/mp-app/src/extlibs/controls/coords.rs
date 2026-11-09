//! Port of `ExtLibs/Controls/Coords.cs`
//!
//! A coordinate read-out: the CMB_coordsystem combo (GEO / UTM / MGRS, 64x24 at 0,2) and the
//! point in the chosen system beside or under it (`Vertical`), with the altitude ("0.00" +
//! AltUnit) and the altitude source. UTM and MGRS are drawn only between 80S and 84N and inside
//! +-180 longitude; numbers use the current culture.

#![allow(non_snake_case)]

use egui::{Color32, FontId, Pos2, Rect, Vec2};
use mp_core::dotnet::{current_culture, format_fixed};

/// `public enum CoordsSystems`
pub const COORDS_SYSTEMS: [&str; 3] = ["GEO", "UTM", "MGRS"];

/// `CMB_coordsystem`: Location (0, 2), Size (64, 24).
const CMB: (f32, f32, f32, f32) = (0.0, 2.0, 64.0, 24.0);

/// `public partial class Coords : MyUserControl`
#[derive(Debug, Clone, PartialEq)]
pub struct Coords {
    /// `System` (CMB_coordsystem.Text)
    pub System: String,
    pub Lat: f64,
    pub Lng: f64,
    pub Alt: f64,
    pub AltUnit: String,
    pub AltSource: String,
    pub Vertical: bool,
}

impl Default for Coords {
    fn default() -> Self {
        Coords { System: "GEO".into(), Lat: 0.0, Lng: 0.0, Alt: 0.0, AltUnit: "m".into(), AltSource: String::new(), Vertical: false }
    }
}

impl Coords {
    /// Draws the control at `o`; true when `SystemChanged` fires.
    pub fn show(&mut self, ui: &mut egui::Ui, o: Pos2, id: &str, font: FontId, fore: Color32) -> bool {
        let cmb = Rect::from_min_size(o + Vec2::new(CMB.0, CMB.1), Vec2::new(CMB.2, CMB.3));
        let mut changed = false;
        let mut system = self.System.clone();
        ui.put(cmb, |ui: &mut egui::Ui| {
            egui::ComboBox::from_id_salt(id)
                .width(CMB.2 - 8.0)
                .selected_text(system.clone())
                .show_ui(ui, |ui| {
                    for c in COORDS_SYSTEMS {
                        if ui.selectable_label(system == c, c).clicked() {
                            system = c.to_string();
                        }
                    }
                })
                .response
        });
        if system != self.System {
            // CMB_coordsystem_SelectedIndexChanged: Invalidate, then SystemChanged
            self.System = system;
            changed = true;
        }
        self.OnPaint(ui.painter(), o, cmb, font, fore);
        changed
    }

    /// `OnPaint`
    pub fn OnPaint(&self, painter: &egui::Painter, o: Pos2, cmb: Rect, font: FontId, fore: Color32) {
        let c = current_culture();
        let text = Pos2::new(cmb.right() + 3.0, o.y + 3.0);
        let alt = format!("{}{}", format_fixed(self.Alt, 1, 2, c), self.AltUnit);
        let draw = |at: Pos2, s: &str| {
            painter.text(at, egui::Align2::LEFT_TOP, s, font.clone(), fore);
        };
        let in_grid = !(self.Lat > 84.0 || self.Lat < -80.0 || self.Lng >= 180.0 || self.Lng <= -180.0);
        match self.System.as_str() {
            "GEO" => {
                let (la, ln) = (format_fixed(self.Lat, 1, 7, c), format_fixed(self.Lng, 1, 7, c));
                if self.Vertical {
                    draw(text, &format!("{la}\n{ln}\n{alt}"));
                    draw(Pos2::new(cmb.left(), cmb.bottom() + 4.0), &self.AltSource);
                } else {
                    draw(text, &format!("{la} {ln}   {alt}"));
                }
            }
            "UTM" => {
                if !in_grid {
                    return;
                }
                let Some(utm) = mp_core::coords::geo_to_utm(self.Lat, self.Lng) else { return };
                let s = mp_core::coords::utm_string(&utm);
                let parts: Vec<&str> = s.split(' ').filter(|p| !p.is_empty()).collect();
                if self.Vertical {
                    if parts.len() < 3 {
                        return;
                    }
                    draw(text, &format!("{}\n{}\n{}\n{alt}", parts[0], parts[1], parts[2]));
                    draw(Pos2::new(cmb.left(), cmb.bottom() + 4.0), &self.AltSource);
                } else {
                    draw(text, &format!("{s}   {alt}"));
                }
            }
            "MGRS" => {
                if !in_grid {
                    return;
                }
                let Some(mgrs) = mp_core::coords::geo_to_mgrs(self.Lat, self.Lng) else { return };
                if self.Vertical {
                    draw(Pos2::new(o.x + 5.0, cmb.bottom() + 2.0), &format!("{mgrs}\n{alt}"));
                    draw(Pos2::new(cmb.right() + 4.0, cmb.top()), &self.AltSource);
                } else {
                    draw(text, &format!("{mgrs}   {alt}"));
                }
            }
            _ => {}
        }
    }
}
