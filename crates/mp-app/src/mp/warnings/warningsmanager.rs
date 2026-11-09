//! Port of `Warnings/WarningsManager.cs`
//!
//! The Warning Manager form (`new WarningsManager().Show()` from ConfigAdvanced): one
//! `WarningControl` row per warning in `WarningEngine.warnings`, each child row 5 px further
//! right with its message box hidden. "+" adds a warning on the first available field;
//! "Save" writes warnings.xml. The form is modeless, and each click opens another one.

#![allow(non_snake_case)]

use egui::{Pos2, Vec2};
use mp_core::dotnet::Exception;
use mp_core::extlibs::utilities::warnings::{CustomWarning, WarningEngine};

use super::warningcontrol::{RowEvent, WarningControl, WarningPath, SIZE};
use super::warningsmanager_designer::{self as designer, DesignerEvent, CLIENT_SIZE, TEXT};

/// `public partial class WarningsManager : Form`
pub struct WarningsManager {
    id: egui::Id,
    controls: Vec<WarningControl>,
    client: Vec2,
}

impl WarningsManager {
    /// `new WarningsManager()`: `InitializeComponent()` then `reload()`.
    pub fn new(instance: u64) -> Result<Self, Exception> {
        let mut m = WarningsManager { id: egui::Id::new(("warnings_manager", instance)), controls: Vec::new(), client: CLIENT_SIZE };
        m.reload()?;
        Ok(m)
    }

    /// `reload()`: rebuild the rows from the engine list.
    pub fn reload(&mut self) -> Result<(), Exception> {
        self.controls.clear();
        let mut y = 0.0;
        let count = WarningEngine::warnings(|w| w.len());
        for index in 0..count {
            let bottom = self.addwarningcontrol(5.0, y, WarningPath { index, depth: 0 }, false)?;
            y = bottom;
        }
        Ok(())
    }

    /// `addwarningcontrol(x, y, item, hideforchild)`: the row, then its child chain; returns
    /// the last row's `Bottom`.
    fn addwarningcontrol(&mut self, mut x: f32, y: f32, path: WarningPath, hideforchild: bool) -> Result<f32, Exception> {
        let mut wrnctl = WarningControl::new(path, Pos2::new(x, y))?;
        if hideforchild {
            wrnctl.TXT_warningtext_Visible = false;
        }
        let bottom = y + SIZE.y;
        self.controls.push(wrnctl);
        let has_child = WarningEngine::warnings(|list| super::warningcontrol::node_mut(list, path).is_some_and(|n| n.Child.is_some()));
        if has_child {
            x += 5.0;
            return self.addwarningcontrol(x, bottom, WarningPath { index: path.index, depth: path.depth + 1 }, true);
        }
        Ok(bottom)
    }

    /// `BUT_Add_Click`: a warning on `GetOptions()[0]`, added to the engine list.
    fn BUT_Add_Click(&mut self) -> Result<(), Exception> {
        let mut newcw = CustomWarning::new();
        let first = newcw.GetOptions()?.into_iter().next().ok_or_else(|| {
            Exception::ArgumentOutOfRange("Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into())
        })?;
        newcw.SetField(&first)?;
        WarningEngine::warnings(|list| list.push(newcw));
        self.reload()
    }

    /// `BUT_save_Click`
    fn BUT_save_Click(&mut self) -> Result<(), Exception> {
        WarningEngine::SaveConfig()
    }

    /// Draw the form; false once it is closed.
    pub fn show(&mut self, ctx: &egui::Context, toasts: &mut crate::app::Toasts) -> bool {
        let mut open = true;
        let mut errors = Vec::new();
        let frame = egui::Frame::window(&ctx.global_style()).fill(crate::theme::bg()).inner_margin(0.0);
        egui::Window::new(crate::i18n::t(TEXT)).id(self.id).open(&mut open).collapsible(false).resizable(true).default_size(CLIENT_SIZE).frame(frame).show(ctx, |ui| {
            let client = ui.available_size().max(Vec2::new(200.0, 80.0));
            let (full, _) = ui.allocate_exact_size(client, egui::Sense::hover());
            self.client = client;
            let o = full.min;
            ui.painter().rect_filled(full, 0.0, crate::theme::bg());
            match designer::show(ui, o) {
                Some(DesignerEvent::Add) => {
                    if let Err(e) = self.BUT_Add_Click() {
                        errors.push(e.to_string());
                    }
                }
                Some(DesignerEvent::Save) => {
                    if let Err(e) = self.BUT_save_Click() {
                        errors.push(e.to_string());
                    }
                }
                None => {}
            }
            // panel1: rows clipped to the panel (AutoScroll is off)
            let panel = designer::panel1(o, client);
            let mut reload = false;
            ui.scope_builder(egui::UiBuilder::new().max_rect(panel), |ui| {
                ui.set_clip_rect(panel);
                for c in &mut self.controls {
                    match c.show(ui, panel.min) {
                        Some(RowEvent::ReloadList) => reload = true,
                        Some(RowEvent::Error(e)) => errors.push(e),
                        None => {}
                    }
                }
            });
            if reload {
                if let Err(e) = self.reload() {
                    errors.push(e.to_string());
                }
            }
        });
        for e in errors {
            toasts.error(e);
        }
        open
    }
}
