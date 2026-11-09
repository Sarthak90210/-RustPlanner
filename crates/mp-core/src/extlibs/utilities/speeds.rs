//! Port of `ExtLibs/Utilities/speeds.cs`

crate::cs_enum! {
    /// `MissionPlanner.speeds`
    pub speeds: i32 {
        meters_per_second = 0,
        fps = 1,
        kph = 2,
        mph = 3,
        knots = 4,
    }
}
