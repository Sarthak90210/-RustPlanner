//! ArduPilot device IDs (AP_HAL::Device): bus_type:3 | bus:5 | address:8 | devtype:8.
//! Used by the Compass priority table and HW ID page (ConfigHWCompass2 / ConfigHWIDs).

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceId {
    pub raw: u32,
    pub bus_type: &'static str,
    pub bus: u32,
    pub address: u32,
    pub devtype: u32,
}

pub fn decode(raw: u32) -> DeviceId {
    let bus_type = match raw & 0x7 {
        1 => "BUS_TYPE_I2C",
        2 => "BUS_TYPE_SPI",
        3 => "BUS_TYPE_UAVCAN",
        4 => "BUS_TYPE_SITL",
        5 => "BUS_TYPE_MSP",
        6 => "BUS_TYPE_SERIAL",
        7 => "BUS_TYPE_QSPI",
        _ => "UNKNOWN",
    };
    DeviceId { raw, bus_type, bus: (raw >> 3) & 0x1f, address: (raw >> 8) & 0xff, devtype: (raw >> 16) & 0xff }
}

/// Compass DevType names (AP_Compass_Backend::DevTypes).
pub fn compass_devtype(t: u32) -> String {
    match t {
        0x01 => "DEVTYPE_HMC5883_OLD",
        0x07 => "DEVTYPE_HMC5883",
        0x08 => "DEVTYPE_LSM303D",
        0x09 => "DEVTYPE_AK8963",
        0x0A => "DEVTYPE_BMM150",
        0x0B => "DEVTYPE_LSM9DS1",
        0x0C => "DEVTYPE_LIS3MDL",
        0x0D => "DEVTYPE_AK09916",
        0x0E => "DEVTYPE_IST8310",
        0x0F => "DEVTYPE_ICM20948",
        0x10 => "DEVTYPE_MMC3416",
        0x11 => "DEVTYPE_QMC5883L",
        0x12 => "DEVTYPE_MAG3110",
        0x13 => "DEVTYPE_SITL",
        0x14 => "DEVTYPE_IST8308",
        0x15 => "DEVTYPE_RM3100",
        0x16 => "DEVTYPE_RM3100_2",
        0x17 => "DEVTYPE_MMC5883",
        0x18 => "DEVTYPE_AK09918",
        0x19 => "DEVTYPE_AK09915",
        0x1A => "DEVTYPE_QMC5883P",
        0x1B => "DEVTYPE_BMM350",
        0x1C => "DEVTYPE_IIS2MDC",
        _ => return format!("0x{t:02X}"),
    }
    .to_string()
}

#[cfg(test)]
mod tests {
    #[test]
    fn decode_sitl_compass() {
        // 97539 = SITL compass on bus 0 address 0x7d? -> verify field split
        let d = super::decode((0x13 << 16) | (0x1e << 8) | (1 << 3) | 1);
        assert_eq!(d.bus_type, "BUS_TYPE_I2C");
        assert_eq!(d.bus, 1);
        assert_eq!(d.address, 0x1e);
        assert_eq!(super::compass_devtype(d.devtype), "DEVTYPE_SITL");
    }
}
