//! `DisplayView` (ExtLibs/Utilities/DisplayView.cs): the Basic / Advanced / Custom layouts
//! (Planner > Layout) whose flags hide menu buttons, DATA tabs, PLAN menus and setup pages.
//! The view is stored as JSON in Settings "displayview" as MainV2 does; the older plain
//! name ("Basic") is still read.

use std::cell::RefCell;
use std::collections::BTreeMap;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Layout {
    Basic,
    Advanced,
    Custom,
}

impl Layout {
    pub fn name(self) -> &'static str {
        match self {
            Layout::Basic => "Basic",
            Layout::Advanced => "Advanced",
            Layout::Custom => "Custom",
        }
    }
}

/// Every DisplayView flag with the value `new DisplayView()` gives it (property initialisers,
/// then the constructor). JSON or XML that omits a flag keeps this value.
const FLAGS: &[(&str, bool)] = &[
    ("displayRTKInject", true),
    ("displayGPSOrder", true),
    ("displayHWIDs", true),
    ("displayADSB", true),
    ("displaySimulation", false),
    ("displayTerminal", true),
    ("displayDonate", true),
    ("displayHelp", true),
    ("displayAnenometer", true),
    ("displayQuickTab", true),
    ("displayPreFlightTab", true),
    ("displayAdvActionsTab", false),
    ("displaySimpleActionsTab", true),
    ("displayGaugesTab", true),
    ("displayStatusTab", false),
    ("displayServoTab", false),
    ("displayScriptsTab", false),
    ("displayTelemetryTab", true),
    ("displayDataflashTab", true),
    ("displayMessagesTab", true),
    ("displayTransponderTab", true),
    ("displayAuxFunctionTab", true),
    ("displayPayloadTab", true),
    ("displayRallyPointsMenu", true),
    ("displayGeoFenceMenu", true),
    ("displaySplineCircleAutoWp", true),
    ("displayTextAutoWp", true),
    ("displayCircleSurveyAutoWp", true),
    ("displayPoiMenu", true),
    ("displayTrackerHomeMenu", true),
    ("displayCheckHeightBox", true),
    ("displayPluginAutoWp", true),
    ("displayInstallFirmware", true),
    ("displayWizard", true),
    ("displayFrameType", true),
    ("displayAccelCalibration", true),
    ("displayCompassConfiguration", true),
    ("displayRadioCalibration", true),
    ("displayEscCalibration", true),
    ("displayFlightModes", true),
    ("displayFailSafe", true),
    ("displaySikRadio", true),
    ("displayBattMonitor", true),
    ("displayCAN", true),
    ("displayCompassMotorCalib", true),
    ("displayRangeFinder", true),
    ("displayAirSpeed", true),
    ("displayPx4Flow", true),
    ("displayOpticalFlow", true),
    ("displayOsd", true),
    ("displayCameraGimbal", true),
    ("displayMotorTest", true),
    ("displayBluetooth", true),
    ("displayParachute", true),
    ("displayEsp", true),
    ("displayAntennaTracker", true),
    ("displaySerialPorts", true),
    ("displayGeoFence", true),
    ("displayBasicTuning", true),
    ("displayExtendedTuning", true),
    ("displayStandardParams", false),
    ("displayAdvancedParams", false),
    ("displayMavFTP", true),
    ("displayFullParamList", true),
    ("displayFullParamTree", true),
    ("displayParamCommitButton", false),
    ("displayBaudCMB", true),
    ("displaySerialPortCMB", true),
    ("standardFlightModesOnly", false),
    ("autoHideMenuForce", false),
    ("displayInitialParams", true),
    ("isAdvancedMode", false),
    ("displayREPL", true),
    ("displayServoOutput", true),
    ("displayJoystick", true),
    ("displayOSD", true),
    ("displayUserParam", true),
    ("displayPlannerSettings", true),
    ("displayFFTSetup", true),
    ("displayPreFlightTabEdit", true),
    ("displayPlannerLayout", true),
    ("lockQuickView", false),
];

#[derive(Clone, Debug, PartialEq)]
pub struct DisplayView {
    pub name: Layout,
    pub flags: BTreeMap<String, bool>,
}

impl Default for DisplayView {
    /// `new DisplayView()`: displayName Basic with the constructor's flags.
    fn default() -> Self {
        Self { name: Layout::Basic, flags: FLAGS.iter().map(|(k, v)| (k.to_string(), *v)).collect() }
    }
}

impl DisplayView {
    pub fn get(&self, flag: &str) -> bool {
        self.flags.get(flag).copied().unwrap_or(false)
    }

    fn with(mut self, name: Layout, set: &[(&str, bool)]) -> Self {
        self.name = name;
        for (k, v) in set {
            self.flags.insert(k.to_string(), *v);
        }
        self
    }

    /// DisplayViewExtensions.Basic
    pub fn basic() -> Self {
        Self::default().with(Layout::Basic, &[
            ("displaySimulation", true),
            ("displayAdvActionsTab", false),
            ("displaySimpleActionsTab", true),
            ("displayStatusTab", false),
            ("displayServoTab", false),
            ("displayScriptsTab", false),
            ("displayTerminal", false),
            ("isAdvancedMode", false),
        ])
    }

    /// DisplayViewExtensions.Advanced
    pub fn advanced() -> Self {
        Self::default().with(Layout::Advanced, &[
            ("displaySimulation", true),
            ("displayAdvActionsTab", true),
            ("displaySimpleActionsTab", false),
            ("displayStatusTab", true),
            ("displayServoTab", true),
            ("displayScriptsTab", true),
            ("displayTerminal", true),
            ("isAdvancedMode", true),
        ])
    }

    /// DisplayViewExtensions.Custom: custom.displayview beside the program, else Advanced.
    pub fn custom() -> Self {
        let file = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("custom.displayview")));
        match file.and_then(|f| std::fs::read_to_string(f).ok()).and_then(|t| Self::parse(&t)) {
            Some(v) => Self { name: Layout::Custom, ..v },
            None => Self::advanced(),
        }
    }

    pub fn preset(name: Layout) -> Self {
        match name {
            Layout::Basic => Self::basic(),
            Layout::Advanced => Self::advanced(),
            Layout::Custom => Self::custom(),
        }
    }

    /// DisplayViewExtensions.TryParse: JSON, or the XmlSerializer form for older settings.
    pub fn parse(text: &str) -> Option<Self> {
        let text = text.trim().trim_start_matches('\u{feff}');
        let mut v = Self::default();
        let name = |s: &str| match s {
            "Advanced" | "1" => Layout::Advanced,
            "Custom" | "2" => Layout::Custom,
            _ => Layout::Basic,
        };
        if text.starts_with('{') {
            let map: serde_json::Map<String, serde_json::Value> = serde_json::from_str(text).ok()?;
            for (k, value) in map {
                match (k.as_str(), value) {
                    ("displayName", serde_json::Value::String(s)) => v.name = name(&s),
                    ("displayName", serde_json::Value::Number(n)) => v.name = name(&n.to_string()),
                    (_, serde_json::Value::Bool(b)) => {
                        v.flags.insert(k, b);
                    }
                    _ => {}
                }
            }
            return Some(v);
        }
        let doc = roxmltree::Document::parse(text).ok()?;
        if !doc.root_element().has_tag_name("DisplayView") {
            return None;
        }
        for node in doc.root_element().children().filter(|n| n.is_element()) {
            let key = node.tag_name().name();
            let value = node.text().unwrap_or("").trim();
            if key == "displayName" {
                v.name = name(value);
            } else if let Ok(b) = value.parse::<bool>() {
                v.flags.insert(key.to_string(), b);
            }
        }
        Some(v)
    }

    /// ConvertToString: the JSON MainV2 saves as "displayview".
    pub fn to_json(&self) -> String {
        let mut map = serde_json::Map::new();
        for (k, v) in &self.flags {
            map.insert(k.clone(), (*v).into());
        }
        map.insert("displayName".into(), self.name.name().into());
        serde_json::Value::Object(map).to_string()
    }
}

thread_local! {
    static CACHE: RefCell<Option<(String, DisplayView)>> = const { RefCell::new(None) };
}

/// MainV2.DisplayConfiguration from Settings "displayview" (first run: Advanced).
pub fn view(planner: &BTreeMap<String, String>) -> DisplayView {
    let raw = planner.get("displayview").cloned().unwrap_or_default();
    CACHE.with(|c| {
        if let Some((key, v)) = c.borrow().as_ref() {
            if *key == raw {
                return v.clone();
            }
        }
        let v = match raw.as_str() {
            "" | "Advanced" => DisplayView::advanced(),
            "Basic" => DisplayView::basic(),
            "Custom" => DisplayView::custom(),
            text => DisplayView::parse(text).unwrap_or_else(DisplayView::advanced),
        };
        *c.borrow_mut() = Some((raw, v.clone()));
        v
    })
}

/// One DisplayView flag of the current layout.
pub fn flag(planner: &BTreeMap<String, String>, name: &str) -> bool {
    view(planner).get(name)
}

pub fn layout(planner: &BTreeMap<String, String>) -> Layout {
    view(planner).name
}

/// ConfigPlanner CMB_Layout: apply a preset and save it as MainV2 does.
pub fn select(planner: &mut BTreeMap<String, String>, name: Layout) {
    planner.insert("displayview".into(), DisplayView::preset(name).to_json());
}

/// MainV2 startup: "Force new view in case of saved view in config.xml".
pub fn force_on_load(planner: &mut BTreeMap<String, String>) {
    if let Some(raw) = planner.get("displayview").filter(|r| r.starts_with('{') || r.starts_with('<')) {
        let mut v = DisplayView::parse(raw).unwrap_or_else(DisplayView::advanced);
        v.flags.insert("displayAdvancedParams".into(), false);
        v.flags.insert("displayStandardParams".into(), false);
        v.flags.insert("displayFullParamList".into(), true);
        planner.insert("displayview".into(), v.to_json());
    }
}

/// DisplayView.isAdvancedMode: the SETUP "Advanced" group and BackstageView.Advanced.
pub fn advanced_mode(planner: &BTreeMap<String, String>) -> bool {
    flag(planner, "isAdvancedMode")
}

/// FlightData TabListDisplay: the flag behind each tab.
pub fn fd_tab(planner: &BTreeMap<String, String>, tab: &str) -> bool {
    let name = match tab {
        "Quick" => "displayQuickTab",
        "PreFlight" => "displayPreFlightTab",
        "Actions" => "displayAdvActionsTab",
        "ActionsSimple" => "displaySimpleActionsTab",
        "Gauges" => "displayGaugesTab",
        "Status" => "displayStatusTab",
        "Servo/Relay" => "displayServoTab",
        "Scripts" => "displayScriptsTab",
        "Telemetry Logs" => "displayTelemetryTab",
        "DataFlash Logs" => "displayDataflashTab",
        "Messages" => "displayMessagesTab",
        "Transponder" => "displayTransponderTab",
        "Aux Function" => "displayAuxFunctionTab",
        "Payload Control" => "displayPayloadTab",
        _ => return true,
    };
    flag(planner, name)
}

/// InitialSetup: the DisplayView flag that adds each SETUP page (None: always added).
pub fn setup_page_flag(page: &str) -> Option<&'static str> {
    Some(match page {
        "Install Firmware" | "Install Firmware Legacy" => "displayInstallFirmware",
        "Heli Setup" | "Frame Type" => "displayFrameType",
        "Accel Calibration" => "displayAccelCalibration",
        "Compass" => "displayCompassConfiguration",
        "Radio Calibration" => "displayRadioCalibration",
        "Servo Output" => "displayServoOutput",
        "Serial Ports" => "displaySerialPorts",
        "ESC Calibration" => "displayEscCalibration",
        "Flight Modes" => "displayFlightModes",
        "FailSafe" => "displayFailSafe",
        "Initial Tune Parameter" => "displayInitialParams",
        "HW ID" => "displayHWIDs",
        "ADSB" => "displayADSB",
        "RTK/GPS Inject" => "displayRTKInject",
        "Sik Radio" => "displaySikRadio",
        "CAN GPS Order" => "displayGPSOrder",
        "Battery Monitor" | "Battery Monitor 2" => "displayBattMonitor",
        "DroneCAN/UAVCAN" => "displayCAN",
        "Joystick" => "displayJoystick",
        "Compass/Motor Calib" => "displayCompassMotorCalib",
        "Range Finder" => "displayRangeFinder",
        "Airspeed" => "displayAirSpeed",
        "PX4Flow" => "displayPx4Flow",
        "Optical Flow" => "displayOpticalFlow",
        "OSD" => "displayOsd",
        "Camera Gimbal" => "displayCameraGimbal",
        "Antenna tracker" | "Antenna Tracker" => "displayAntennaTracker",
        "Motor Test" => "displayMotorTest",
        "Bluetooth Setup" => "displayBluetooth",
        "Parachute" => "displayParachute",
        "ESP8266 Setup" => "displayEsp",
        "FFT Setup" => "displayFFTSetup",
        "Advanced" => "isAdvancedMode",
        "Terminal" => "displayTerminal",
        "Script REPL" => "displayREPL",
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_match_display_view_extensions() {
        let mut p = BTreeMap::new();
        assert_eq!(layout(&p), Layout::Advanced);
        assert!(fd_tab(&p, "Actions") && !fd_tab(&p, "ActionsSimple") && advanced_mode(&p));
        select(&mut p, Layout::Basic);
        assert!(p["displayview"].starts_with('{'));
        assert_eq!(layout(&p), Layout::Basic);
        assert!(!fd_tab(&p, "Actions") && fd_tab(&p, "ActionsSimple") && !fd_tab(&p, "Status"));
        assert!(!advanced_mode(&p) && !flag(&p, "displayTerminal") && flag(&p, "displaySimulation"));
        // the older plain name
        p.insert("displayview".into(), "Basic".into());
        assert_eq!(view(&p), DisplayView::basic());
        let basic = DisplayView::basic();
        let advanced = DisplayView::advanced();
        let differ: Vec<_> = basic.flags.iter().filter(|(k, v)| advanced.flags[*k] != **v).map(|(k, _)| k.as_str()).collect();
        assert_eq!(differ, ["displayAdvActionsTab", "displayScriptsTab", "displayServoTab", "displaySimpleActionsTab", "displayStatusTab", "displayTerminal", "isAdvancedMode"]);
    }

    #[test]
    fn custom_json_and_xml_keep_constructor_defaults_for_missing_flags() {
        let v = DisplayView::parse(r#"{"displayName":"Custom","displayFrameType":false,"displayHelp":false}"#).unwrap();
        assert_eq!(v.name, Layout::Custom);
        assert!(!v.get("displayFrameType") && !v.get("displayHelp"));
        // missing flags take new DisplayView() values, not Advanced
        assert!(!v.get("displayStatusTab") && !v.get("displaySimulation") && v.get("displayHWIDs"));
        let x = DisplayView::parse("<?xml version=\"1.0\"?><DisplayView><displayName>Advanced</displayName><displayMotorTest>false</displayMotorTest></DisplayView>").unwrap();
        assert_eq!(x.name, Layout::Advanced);
        assert!(!x.get("displayMotorTest") && x.get("displayOsd"));
        assert!(DisplayView::parse("not a view").is_none());
        assert_eq!(DisplayView::parse(&v.to_json()), Some(v));
    }

    #[test]
    fn startup_forces_parameter_pages() {
        let mut p = BTreeMap::new();
        let mut v = DisplayView::advanced();
        v.flags.insert("displayStandardParams".into(), true);
        v.flags.insert("displayFullParamList".into(), false);
        p.insert("displayview".into(), v.to_json());
        force_on_load(&mut p);
        assert!(!flag(&p, "displayStandardParams") && flag(&p, "displayFullParamList"));
    }
}
