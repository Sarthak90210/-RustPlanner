//! Port of `ExtLibs/Utilities/DisplayView.cs`
//!
//! `DisplayView` is stored in the config as Newtonsoft JSON (`ToJSON`), and older configs hold
//! it as `XmlSerializer` XML; `TryParse` reads both. JSON property names match the C#
//! property names (Newtonsoft matches them case-insensitively on read), `displayName` is
//! written as its number, and properties a value does not mention keep their constructor
//! defaults.

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use super::settings::Settings;
use crate::dotnet::environment::DIRECTORY_SEPARATOR_CHAR;

/// `[Serializable] public enum DisplayNames`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DisplayNames {
    #[default]
    Basic = 0,
    Advanced = 1,
    Custom = 2,
}

impl DisplayNames {
    fn from_i64(v: i64) -> Option<Self> {
        Some(match v {
            0 => DisplayNames::Basic,
            1 => DisplayNames::Advanced,
            2 => DisplayNames::Custom,
            _ => return None,
        })
    }

    fn from_name(s: &str) -> Option<Self> {
        Some(match s.trim() {
            "Basic" => DisplayNames::Basic,
            "Advanced" => DisplayNames::Advanced,
            "Custom" => DisplayNames::Custom,
            _ => return None,
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            DisplayNames::Basic => "Basic",
            DisplayNames::Advanced => "Advanced",
            DisplayNames::Custom => "Custom",
        }
    }
}

impl Serialize for DisplayNames {
    /// Newtonsoft writes enums as numbers.
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_i64(*self as i64)
    }
}

impl<'de> Deserialize<'de> for DisplayNames {
    /// Newtonsoft accepts the number or the member name.
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let v = serde_json::Value::deserialize(d)?;
        let r = match &v {
            serde_json::Value::Number(n) => n.as_i64().and_then(DisplayNames::from_i64),
            serde_json::Value::String(s) => DisplayNames::from_name(s).or_else(|| s.trim().parse::<i64>().ok().and_then(DisplayNames::from_i64)),
            _ => None,
        };
        r.ok_or_else(|| serde::de::Error::custom(format!("Error converting value {v} to type 'MissionPlanner.Utilities.DisplayNames'")))
    }
}

crate::cs_enum! {
    /// `public enum SeverityLevel`
    pub SeverityLevel: i32 {
        Emergency = 0,
        Alert = 1,
        Critical = 2,
        Error = 3,
        Warning = 4,
        Notice = 5,
        Info = 6,
        Debug = 7,
    }
}

macro_rules! display_view {
    ($($name:ident = $default:expr),* $(,)?) => {
        /// `[Serializable] public class DisplayView`
        #[allow(non_snake_case)]
        #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct DisplayView {
            $(pub $name: field_type!($name),)*
        }

        impl Default for DisplayView {
            /// `new DisplayView()`: property initialisers, then the constructor's assignments
            /// (also the template for options a stored value does not have).
            fn default() -> Self {
                DisplayView { $($name: $default,)* }
            }
        }

        impl DisplayView {
            /// The property names, in declaration order.
            pub const PROPERTIES: &'static [&'static str] = &[$(stringify!($name)),*];

            fn set_xml(&mut self, name: &str, text: &str) -> bool {
                match name {
                    $(stringify!($name) => xml_value(&mut self.$name, text),)*
                    _ => true,
                }
            }
        }
    };
}

macro_rules! field_type {
    (displayName) => {
        DisplayNames
    };
    ($other:ident) => {
        bool
    };
}

trait XmlValue {
    fn parse_xml(text: &str) -> Option<Self>
    where
        Self: Sized;
}

impl XmlValue for bool {
    /// `XmlConvert.ToBoolean`: "true"/"1" or "false"/"0" after trimming.
    fn parse_xml(text: &str) -> Option<bool> {
        match text.trim() {
            "true" | "1" => Some(true),
            "false" | "0" => Some(false),
            _ => None,
        }
    }
}

impl XmlValue for DisplayNames {
    fn parse_xml(text: &str) -> Option<DisplayNames> {
        DisplayNames::from_name(text)
    }
}

fn xml_value<T: XmlValue>(slot: &mut T, text: &str) -> bool {
    match T::parse_xml(text) {
        Some(v) => {
            *slot = v;
            true
        }
        None => false,
    }
}

display_view! {
    displayRTKInject = true,
    displayGPSOrder = true,
    displayHWIDs = true,
    displayADSB = true,
    displayName = DisplayNames::Basic,

    //MainV2 buttons
    displaySimulation = false,
    displayTerminal = true,
    displayDonate = true,
    displayHelp = true,

    //flight Data view
    displayAnenometer = true,
    displayQuickTab = true,
    displayPreFlightTab = true,
    displayAdvActionsTab = false,
    displaySimpleActionsTab = true,
    displayGaugesTab = true,
    displayStatusTab = false,
    displayServoTab = false,
    displayScriptsTab = false,
    displayTelemetryTab = true,
    displayDataflashTab = true,
    displayMessagesTab = true,
    displayTransponderTab = true,
    displayAuxFunctionTab = true,
    displayPayloadTab = true,

    //flight plan
    displayRallyPointsMenu = true,
    displayGeoFenceMenu = true,
    displaySplineCircleAutoWp = true,
    displayTextAutoWp = true,
    displayCircleSurveyAutoWp = true,
    displayPoiMenu = true,
    displayTrackerHomeMenu = true,
    displayCheckHeightBox = true,
    displayPluginAutoWp = true,

    //initial setup
    displayInstallFirmware = true,
    displayWizard = true,
    displayFrameType = true,
    displayAccelCalibration = true,
    displayCompassConfiguration = true,
    displayRadioCalibration = true,
    displayEscCalibration = true,
    displayFlightModes = true,
    displayFailSafe = true,
    displaySikRadio = true,
    displayBattMonitor = true,
    displayCAN = true,
    displayCompassMotorCalib = true,
    displayRangeFinder = true,
    displayAirSpeed = true,
    displayPx4Flow = true,
    displayOpticalFlow = true,
    displayOsd = true,
    displayCameraGimbal = true,
    displayMotorTest = true,
    displayBluetooth = true,
    displayParachute = true,
    displayEsp = true,
    displayAntennaTracker = true,
    displaySerialPorts = true,

    //config tuning
    displayGeoFence = true,
    displayBasicTuning = true,
    displayExtendedTuning = true,
    displayStandardParams = false,
    displayAdvancedParams = false,
    displayMavFTP = true,
    displayFullParamList = true,
    displayFullParamTree = true,
    displayParamCommitButton = false,
    displayBaudCMB = true,
    displaySerialPortCMB = true,
    standardFlightModesOnly = false,
    autoHideMenuForce = false,
    displayInitialParams = true,
    isAdvancedMode = false,
    displayREPL = true,
    displayServoOutput = true,
    displayJoystick = true,
    displayOSD = true,
    displayUserParam = true,
    displayPlannerSettings = true,
    displayFFTSetup = true,
    displayPreFlightTabEdit = true,
    displayPlannerLayout = true,

    lockQuickView = false,
}

/// `public static class DisplayViewExtensions`
pub struct DisplayViewExtensions;

impl DisplayViewExtensions {
    /// `TryParse(string value, out DisplayView result)`: XML when the text does not start with
    /// '{', JSON otherwise.
    pub fn try_parse(value: &str) -> Option<DisplayView> {
        if !value.starts_with('{') {
            return Self::from_xml(value);
        }
        Self::from_json(value)
    }

    /// `value.FromJSON<DisplayView>()` with Newtonsoft's case-insensitive property matching.
    fn from_json(value: &str) -> Option<DisplayView> {
        let v: serde_json::Value = serde_json::from_str(value).ok()?;
        let serde_json::Value::Object(o) = v else { return None };
        let mut norm = serde_json::Map::new();
        for (k, val) in o {
            let key = DisplayView::PROPERTIES.iter().find(|p| **p == k).or_else(|| DisplayView::PROPERTIES.iter().find(|p| p.eq_ignore_ascii_case(&k)));
            if let Some(key) = key {
                norm.insert(key.to_string(), val);
            }
        }
        serde_json::from_value(serde_json::Value::Object(norm)).ok()
    }

    /// `XmlSerializer(typeof(DisplayView)).Deserialize`: a `<DisplayView>` root with one child
    /// element per property; unknown elements are ignored and bad values fail the parse.
    fn from_xml(value: &str) -> Option<DisplayView> {
        let doc = roxmltree::Document::parse(value).ok()?;
        let root = doc.root_element();
        if root.tag_name().name() != "DisplayView" {
            return None;
        }
        let mut result = DisplayView::default();
        for c in root.children().filter(|n| n.is_element()) {
            let text: String = c.children().filter(|n| n.is_text()).map(|n| n.text().unwrap_or("")).collect();
            if !result.set_xml(c.tag_name().name(), &text) {
                return None;
            }
        }
        Some(result)
    }

    /// `ConvertToString(this DisplayView v)`: `v.ToJSON()`.
    pub fn convert_to_string(v: &DisplayView) -> String {
        serde_json::to_string(v).unwrap_or_default()
    }

    /// `Basic(this DisplayView v)`
    pub fn basic() -> DisplayView {
        DisplayView {
            displayName: DisplayNames::Basic,
            //MainV2 buttons
            displaySimulation: true,
            displayDonate: true,
            displayHelp: true,

            //flight Data view
            displayAnenometer: true,
            displayQuickTab: true,
            displayPreFlightTab: true,
            displayAdvActionsTab: false,
            displaySimpleActionsTab: true,
            displayGaugesTab: true,
            displayStatusTab: false,
            displayServoTab: false,
            displayScriptsTab: false,
            displayTelemetryTab: true,
            displayDataflashTab: true,
            displayMessagesTab: true,
            displayTransponderTab: true,
            displayAuxFunctionTab: true,
            displayPayloadTab: true,

            //flight plan
            displayRallyPointsMenu: true,
            displayGeoFenceMenu: true,
            displaySplineCircleAutoWp: true,
            displayTextAutoWp: true,
            displayCircleSurveyAutoWp: true,
            displayPoiMenu: true,
            displayTrackerHomeMenu: true,
            displayCheckHeightBox: true,
            displayPluginAutoWp: true,

            //initial setup
            displayInstallFirmware: true,
            displayWizard: true,
            displayFrameType: true,
            displayAccelCalibration: true,
            displayCompassConfiguration: true,
            displayRadioCalibration: true,
            displayServoOutput: true,
            displayEscCalibration: true,
            displayFlightModes: true,
            displayFailSafe: true,
            displaySikRadio: true,
            displayBattMonitor: true,
            displayCAN: true,
            displayCompassMotorCalib: true,
            displayRangeFinder: true,
            displayAirSpeed: true,
            displayPx4Flow: true,
            displayOpticalFlow: true,
            displayOsd: true,
            displayCameraGimbal: true,
            displayMotorTest: true,
            displayBluetooth: true,
            displayParachute: true,
            displayEsp: true,
            displayAntennaTracker: true,
            displayRTKInject: true,
            displayJoystick: true,
            displayREPL: true,
            displayTerminal: false,

            //config tuning
            displayGeoFence: true,
            displayBasicTuning: true,
            displayExtendedTuning: true,
            displayStandardParams: false,
            displayAdvancedParams: false,
            displayMavFTP: true,
            displayFullParamList: true,
            displayFullParamTree: true,
            displayParamCommitButton: false,
            displayBaudCMB: true,
            displaySerialPortCMB: true,
            displayOSD: true,
            standardFlightModesOnly: false,
            autoHideMenuForce: false,
            isAdvancedMode: false,
            ..DisplayView::default()
        }
    }

    /// `Advanced(this DisplayView v)`
    pub fn advanced() -> DisplayView {
        DisplayView {
            displayName: DisplayNames::Advanced,
            //MainV2 buttons
            displaySimulation: true,
            displayDonate: true,
            displayHelp: true,

            //flight Data view
            displayAnenometer: true,
            displayQuickTab: true,
            displayPreFlightTab: true,
            displayAdvActionsTab: true,
            displaySimpleActionsTab: false,
            displayGaugesTab: true,
            displayStatusTab: true,
            displayServoTab: true,
            displayScriptsTab: true,
            displayTelemetryTab: true,
            displayDataflashTab: true,
            displayMessagesTab: true,
            displayTransponderTab: true,
            displayAuxFunctionTab: true,
            displayPayloadTab: true,

            //flight plan
            displayRallyPointsMenu: true,
            displayGeoFenceMenu: true,
            displaySplineCircleAutoWp: true,
            displayTextAutoWp: true,
            displayCircleSurveyAutoWp: true,
            displayPoiMenu: true,
            displayTrackerHomeMenu: true,
            displayCheckHeightBox: true,
            displayPluginAutoWp: true,

            //initial setup
            displayInstallFirmware: true,
            displayWizard: true,
            displayFrameType: true,
            displayAccelCalibration: true,
            displayCompassConfiguration: true,
            displayRadioCalibration: true,
            displayServoOutput: true,
            displayEscCalibration: true,
            displayFlightModes: true,
            displayFailSafe: true,
            displaySikRadio: true,
            displayBattMonitor: true,
            displayCAN: true,
            displayCompassMotorCalib: true,
            displayRangeFinder: true,
            displayAirSpeed: true,
            displayPx4Flow: true,
            displayOpticalFlow: true,
            displayOsd: true,
            displayCameraGimbal: true,
            displayMotorTest: true,
            displayBluetooth: true,
            displayParachute: true,
            displayEsp: true,
            displayAntennaTracker: true,
            displayRTKInject: true,
            displayJoystick: true,
            displayREPL: true,
            displayTerminal: true,

            //config tuning
            displayGeoFence: true,
            displayBasicTuning: true,
            displayExtendedTuning: true,
            displayStandardParams: false,
            displayAdvancedParams: false,
            displayMavFTP: true,
            displayFullParamList: true,
            displayFullParamTree: true,
            displayParamCommitButton: false,
            displayBaudCMB: true,
            displaySerialPortCMB: true,
            standardFlightModesOnly: false,
            displayOSD: true,
            autoHideMenuForce: false,
            isAdvancedMode: true,
            ..DisplayView::default()
        }
    }

    /// `public static string custompath`: `GetRunningDirectory()` already ends in a separator,
    /// so the path carries two.
    pub fn custompath() -> String {
        format!("{}{}custom.displayview", Settings::get_running_directory(), DIRECTORY_SEPARATOR_CHAR)
    }

    /// `Custom(this DisplayView v)`: `custom.displayview` beside the program, else Advanced.
    pub fn custom() -> DisplayView {
        let result = Self::advanced();
        let path = Self::custompath();
        if std::path::Path::new(&path).exists() {
            if let Some(mut r) = std::fs::read_to_string(&path).ok().and_then(|t| Self::try_parse(&t)) {
                r.displayName = DisplayNames::Custom;
                return r;
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_round_trip_and_defaults() {
        let v = DisplayViewExtensions::advanced();
        let s = DisplayViewExtensions::convert_to_string(&v);
        assert!(s.starts_with("{\"displayRTKInject\":true,\"displayGPSOrder\":true,\"displayHWIDs\":true,\"displayADSB\":true,\"displayName\":1,"), "{s}");
        assert_eq!(DisplayViewExtensions::try_parse(&s), Some(v));
        let partial = DisplayViewExtensions::try_parse("{\"DISPLAYNAME\":\"Advanced\",\"displayHelp\":false}").unwrap();
        assert_eq!(partial.displayName, DisplayNames::Advanced);
        assert!(!partial.displayHelp);
        assert!(partial.displayDonate, "unmentioned properties keep the constructor defaults");
        assert!(DisplayViewExtensions::try_parse("{bad").is_none());
    }

    #[test]
    fn xml_values() {
        let x = "<?xml version=\"1.0\"?><DisplayView xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"><displayName>Advanced</displayName><displayTerminal>false</displayTerminal><unknown>1</unknown></DisplayView>";
        let v = DisplayViewExtensions::try_parse(x).unwrap();
        assert_eq!(v.displayName, DisplayNames::Advanced);
        assert!(!v.displayTerminal);
        assert!(DisplayViewExtensions::try_parse("<DisplayView><displayHelp>maybe</displayHelp></DisplayView>").is_none());
    }

    #[test]
    fn constructor_defaults() {
        let d = DisplayView::default();
        assert!(!d.displaySimulation);
        assert!(d.displayTerminal);
        assert!(!d.displayAdvActionsTab);
        assert!(!d.lockQuickView);
        assert_eq!(DisplayViewExtensions::basic().displayTerminal, false);
        assert!(DisplayViewExtensions::basic().displaySimulation);
    }
}
