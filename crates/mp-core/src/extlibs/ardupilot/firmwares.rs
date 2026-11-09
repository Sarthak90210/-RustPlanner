//! Port of `ExtLibs/ArduPilot/Firmwares.cs`

crate::cs_enum! {
    /// enum of firmwares
    pub Firmwares: i32 {
        ArduPlane = 0,
        ArduCopter2 = 1,
        ArduRover = 2,
        ArduSub = 3,
        Ateryx = 4,
        ArduTracker = 5,
        Gimbal = 6,
        PX4 = 7,
        Other = 8,
        AP_Periph = 9,
    }
}
