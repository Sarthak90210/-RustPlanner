//! Port of `ExtLibs/ArduPilot/DeviceInfo.cs`

/// `MissionPlanner.ArduPilot.DeviceInfo`; the C# string fields default to null, here to "".
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Com Port Name
    pub name: String,
    /// Windows device description
    pub description: String,
    /// usb reported device description
    pub board: String,
    /// device vid pid in windows format
    pub hardwareid: String,
}
