//! Port of `ExtLibs/SimpleGrid/SimpleGridPlugin.cs`
//!
//! (`MissionPlanner.SimpleGrid.GridPlugin`)
//!
//! `Loaded()` adds a "SimpleGrid" item to the end of the Flight Planner map menu's
//! `autoWPToolStripMenuItem` drop-down (or to the menu itself when that item is missing); the
//! planner's context menu draws it from `GridPlugin::menu_text`. Its click (`but_Click`) opens
//! the form when the drawn polygon has more than two points.

#![allow(non_snake_case, non_upper_case_globals)]

use std::collections::BTreeMap;

use mp_core::dotnet::Exception;
use mp_core::extlibs::utilities::PointLatLngAlt;

use super::simplegridui::GridUI;
use crate::map;

/// The parts of `MissionPlanner.Plugin.PluginHost` this plugin reads and writes.
pub struct PluginHost<'a> {
    /// `Host.FPDrawnPolygon.Points`
    pub FPDrawnPolygon: &'a mut Vec<(f64, f64)>,
    /// `Host.cs.PlannedHomeLocation`
    pub PlannedHomeLocation: PointLatLngAlt,
    /// `Host.config` (`Settings.Instance`)
    pub config: &'a mut BTreeMap<String, String>,
    /// `Host.FDMapType`
    pub FDMapType: map::Provider,
}

/// What `but_Click` did.
pub enum ButClick {
    /// `new GridUI(this)` then `ShowDialog()`
    Show(Box<GridUI>),
    /// `CustomMessageBox.Show(text, caption)`: "Please define a polygon." / "Error", or an
    /// exception out of the form's constructor
    Error(String, &'static str),
}

/// `public class GridPlugin : MissionPlanner.Plugin.Plugin`
#[derive(Default)]
pub struct GridPlugin;

impl GridPlugin {
    pub const Name: &'static str = "SimpleGrid";
    pub const Version: &'static str = "0.1";
    pub const Author: &'static str = "Michael Oborne";

    /// `Init()`
    pub fn Init(&mut self) -> bool {
        true
    }

    /// `Loaded()`: the menu item's text (`new ToolStripMenuItem("SimpleGrid")`).
    pub fn Loaded(&mut self) -> bool {
        true
    }

    /// `but.Text`
    pub fn menu_text() -> &'static str {
        "SimpleGrid"
    }

    /// `but_Click`
    pub fn but_Click(&self, host: &mut PluginHost) -> ButClick {
        if host.FPDrawnPolygon.len() > 2 {
            match GridUI::new(host) {
                Ok(gridui) => ButClick::Show(Box::new(gridui)),
                Err(e) => ButClick::Error(exception_text(&e), ""),
            }
        } else {
            ButClick::Error("Please define a polygon.".into(), "Error")
        }
    }

    /// `Exit()`
    pub fn Exit(&mut self) -> bool {
        true
    }
}

fn exception_text(e: &Exception) -> String {
    e.to_string()
}
