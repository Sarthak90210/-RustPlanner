//! Port of `ExtLibs/ArduPilot/motor_frame_class.cs`

crate::cs_enum! {
    /// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors_Class.h
    pub motor_frame_class: i32 {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_QUAD = 1,
        MOTOR_FRAME_HEXA = 2,
        MOTOR_FRAME_OCTA = 3,
        MOTOR_FRAME_OCTAQUAD = 4,
        MOTOR_FRAME_Y6 = 5,
        MOTOR_FRAME_HELI = 6,
        MOTOR_FRAME_TRI = 7,
        MOTOR_FRAME_SINGLE = 8,
        MOTOR_FRAME_COAX = 9,
        MOTOR_FRAME_TAILSITTER = 10,
        MOTOR_FRAME_HELI_DUAL = 11,
        MOTOR_FRAME_DODECAHEXA = 12,
        MOTOR_FRAME_HELI_QUAD = 13,
    }
}
