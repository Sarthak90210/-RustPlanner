//! Port of `ExtLibs/ArduPilot/ap_product.cs`

/// `MissionPlanner.ArduPilot.ap_product` (an `int` enum; any value converts in, as in C#).
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct ap_product(pub i32);

#[allow(non_upper_case_globals)]
impl ap_product {
    /// Hardware in the loop
    pub const AP_PRODUCT_ID_NONE: ap_product = ap_product(0x00);
    /// APM1 with 1280 CPUs
    pub const AP_PRODUCT_ID_APM1_1280: ap_product = ap_product(0x01);
    /// APM1 with 2560 CPUs
    pub const AP_PRODUCT_ID_APM1_2560: ap_product = ap_product(0x02);
    /// Software in the loop
    pub const AP_PRODUCT_ID_SITL: ap_product = ap_product(0x03);
    /// PX4 on NuttX
    pub const AP_PRODUCT_ID_PX4: ap_product = ap_product(0x04);
    /// PX4 FMU2 on NuttX
    pub const AP_PRODUCT_ID_PX4_V2: ap_product = ap_product(0x05);
    /// APM2 with MPU6000ES_REV_C4
    pub const AP_PRODUCT_ID_APM2ES_REV_C4: ap_product = ap_product(0x14);
    /// APM2 with MPU6000ES_REV_C5
    pub const AP_PRODUCT_ID_APM2ES_REV_C5: ap_product = ap_product(0x15);
    /// APM2 with MPU6000ES_REV_D6
    pub const AP_PRODUCT_ID_APM2ES_REV_D6: ap_product = ap_product(0x16);
    /// APM2 with MPU6000ES_REV_D7
    pub const AP_PRODUCT_ID_APM2ES_REV_D7: ap_product = ap_product(0x17);
    /// APM2 with MPU6000ES_REV_D8
    pub const AP_PRODUCT_ID_APM2ES_REV_D8: ap_product = ap_product(0x18);
    /// APM2 with MPU6000_REV_C4
    pub const AP_PRODUCT_ID_APM2_REV_C4: ap_product = ap_product(0x54);
    /// APM2 with MPU6000_REV_C5
    pub const AP_PRODUCT_ID_APM2_REV_C5: ap_product = ap_product(0x55);
    /// APM2 with MPU6000_REV_D6
    pub const AP_PRODUCT_ID_APM2_REV_D6: ap_product = ap_product(0x56);
    /// APM2 with MPU6000_REV_D7
    pub const AP_PRODUCT_ID_APM2_REV_D7: ap_product = ap_product(0x57);
    /// APM2 with MPU6000_REV_D8
    pub const AP_PRODUCT_ID_APM2_REV_D8: ap_product = ap_product(0x58);
    /// APM2 with MPU6000_REV_D9
    pub const AP_PRODUCT_ID_APM2_REV_D9: ap_product = ap_product(0x59);
    /// Flymaple with ITG3205, ADXL345, HMC5883, BMP085
    pub const AP_PRODUCT_ID_FLYMAPLE: ap_product = ap_product(0x100);
    /// Linux with L3G4200D and ADXL345
    pub const AP_PRODUCT_ID_L3G4200D: ap_product = ap_product(0x101);

    /// (name, value, `[DisplayText]`) in declaration order.
    pub const MEMBERS: &'static [(&'static str, i32, &'static str)] = &[
        ("AP_PRODUCT_ID_NONE", 0x00, "HIL"),
        ("AP_PRODUCT_ID_APM1_1280", 0x01, "APM1 1280"),
        ("AP_PRODUCT_ID_APM1_2560", 0x02, "APM1 2560"),
        ("AP_PRODUCT_ID_SITL", 0x03, "SITL"),
        ("AP_PRODUCT_ID_PX4", 0x04, "PX4"),
        ("AP_PRODUCT_ID_PX4_V2", 0x05, "PX4 FMU 2"),
        ("AP_PRODUCT_ID_APM2ES_REV_C4", 0x14, "APM2 ES C4"),
        ("AP_PRODUCT_ID_APM2ES_REV_C5", 0x15, "APM2 ES C5"),
        ("AP_PRODUCT_ID_APM2ES_REV_D6", 0x16, "APM2 ES D6"),
        ("AP_PRODUCT_ID_APM2ES_REV_D7", 0x17, "APM2 ES D7"),
        ("AP_PRODUCT_ID_APM2ES_REV_D8", 0x18, "APM2 ES D8"),
        ("AP_PRODUCT_ID_APM2_REV_C4", 0x54, "APM2 C4"),
        ("AP_PRODUCT_ID_APM2_REV_C5", 0x55, "APM2 C5"),
        ("AP_PRODUCT_ID_APM2_REV_D6", 0x56, "APM2 D6"),
        ("AP_PRODUCT_ID_APM2_REV_D7", 0x57, "APM2 D7"),
        ("AP_PRODUCT_ID_APM2_REV_D8", 0x58, "APM2 D8"),
        ("AP_PRODUCT_ID_APM2_REV_D9", 0x59, "APM2 D9"),
        ("AP_PRODUCT_ID_FLYMAPLE", 0x100, "FlyMaple"),
        ("AP_PRODUCT_ID_L3G4200D", 0x101, "Linux"),
    ];

    /// The member's `[DisplayText]` attribute, if this value is a member.
    pub fn display_text_attribute(self) -> Option<crate::extlibs::utilities::attributes::DisplayTextAttribute> {
        Self::MEMBERS
            .iter()
            .find(|m| m.1 == self.0)
            .and_then(|m| crate::extlibs::utilities::attributes::DisplayTextAttribute::new(m.2).ok())
    }

    /// `DisplayTextAttribute.Text` of the member: the translated display text.
    pub fn display_text(self) -> Option<String> {
        self.display_text_attribute().map(|a| a.text())
    }
}

impl std::fmt::Display for ap_product {
    /// `Enum.ToString()`: the member name, or the number for any other value.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match Self::MEMBERS.iter().find(|m| m.1 == self.0) {
            Some(m) => f.write_str(m.0),
            None => write!(f, "{}", self.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_and_display_text() {
        assert_eq!(ap_product::AP_PRODUCT_ID_SITL.to_string(), "AP_PRODUCT_ID_SITL");
        assert_eq!(ap_product::AP_PRODUCT_ID_PX4_V2.display_text().as_deref(), Some("PX4 FMU 2"));
        assert_eq!(ap_product(7).to_string(), "7");
    }
}
