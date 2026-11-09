//! Port of `ExtLibs/ArduPilot/Frame.cs`

use super::motor_frame_type::motor_frame_type;

crate::cs_enum! {
    /// `MissionPlanner.ArduPilot.Frame`
    pub Frame: i32 {
        Plus = motor_frame_type::MOTOR_FRAME_TYPE_PLUS.0,
        X = motor_frame_type::MOTOR_FRAME_TYPE_X.0,
        V = motor_frame_type::MOTOR_FRAME_TYPE_V.0,
        H = motor_frame_type::MOTOR_FRAME_TYPE_H.0,
        VTail = motor_frame_type::MOTOR_FRAME_TYPE_VTAIL.0,
        Y = motor_frame_type::MOTOR_FRAME_TYPE_Y6B.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_follow_motor_frame_type() {
        assert_eq!(Frame::Y.0, 10);
        assert_eq!(Frame(4).to_string(), "VTail");
        assert_eq!(Frame::parse("H"), Some(Frame::H));
    }
}
