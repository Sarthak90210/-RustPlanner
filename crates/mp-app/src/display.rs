//! MainV2.DisplayConfiguration (Planner > Layout): the Basic / Advanced / Custom presets of
//! `DisplayView` that hide screens, tabs and setup pages.

use std::collections::BTreeMap;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Layout {
    Basic,
    Advanced,
    Custom,
}

pub fn layout(planner: &BTreeMap<String, String>) -> Layout {
    match planner.get("displayview").map(|s| s.as_str()) {
        Some("Basic") => Layout::Basic,
        Some("Custom") => Layout::Custom,
        // first run: DisplayConfiguration.Advanced()
        _ => Layout::Advanced,
    }
}

/// DisplayView.isAdvancedMode: the SETUP "Advanced" group (Terminal, Script REPL).
pub fn advanced_mode(planner: &BTreeMap<String, String>) -> bool {
    layout(planner) != Layout::Basic
}

/// displayTerminal (false in Basic).
pub fn terminal(planner: &BTreeMap<String, String>) -> bool {
    layout(planner) != Layout::Basic
}

/// FlightData tab flags. `tab` is the FlightData tab key.
pub fn fd_tab(planner: &BTreeMap<String, String>, tab: &str) -> bool {
    let basic = layout(planner) == Layout::Basic;
    match tab {
        // displayAdvActionsTab / displaySimpleActionsTab
        "Actions" => !basic,
        "ActionsSimple" => basic,
        // displayStatusTab, displayServoTab, displayScriptsTab
        "Status" | "Servo/Relay" | "Scripts" => !basic,
        _ => true,
    }
}

/// displayStandardParams / displayAdvancedParams: MainV2 forces both off whatever view is
/// loaded; the Custom view keeps them reachable in this port.
pub fn friendly_params(planner: &BTreeMap<String, String>) -> bool {
    layout(planner) == Layout::Custom
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets() {
        let mut p = BTreeMap::new();
        assert_eq!(layout(&p), Layout::Advanced);
        assert!(fd_tab(&p, "Actions") && !fd_tab(&p, "ActionsSimple") && advanced_mode(&p));
        p.insert("displayview".to_string(), "Basic".to_string());
        assert!(!fd_tab(&p, "Actions") && fd_tab(&p, "ActionsSimple") && !fd_tab(&p, "Status"));
        assert!(!advanced_mode(&p) && !friendly_params(&p));
    }
}
