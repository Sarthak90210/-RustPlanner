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

/// `Device.cs` compass_type names.
pub fn compass_devtype(t: u32) -> String {
    match t {
        0x01 => "DEVTYPE_HMC5883_OLD",
        0x07 => "DEVTYPE_HMC5883",
        0x02 => "DEVTYPE_LSM303D",
        0x04 => "DEVTYPE_AK8963",
        0x05 => "DEVTYPE_BMM150",
        0x06 => "DEVTYPE_LSM9DS1",
        0x08 => "DEVTYPE_LIS3MDL",
        0x09 => "DEVTYPE_AK09916",
        0x0A => "DEVTYPE_IST8310",
        0x0B => "DEVTYPE_ICM20948",
        0x0C => "DEVTYPE_MMC3416",
        0x0D => "DEVTYPE_QMC5883L",
        0x0E => "DEVTYPE_MAG3110",
        0x0F => "DEVTYPE_SITL",
        0x10 => "DEVTYPE_IST8308",
        0x11 => "DEVTYPE_RM3100",
        0x12 => "DEVTYPE_RM3100_2",
        0x13 => "DEVTYPE_MMC5883",
        0x14 => "DEVTYPE_AK09918",
        0x15 => "DEVTYPE_AK09915",
        0x16 => "DEVTYPE_QMC5883P",
        0x17 => "DEVTYPE_BMM350",
        0x18 => "DEVTYPE_IIS2MDC",
        0x19 => "DEVTYPE_LIS2MDL",
        0x1A => "DEVTYPE_AF9838",
        _ => return format!("0x{t:02X}"),
    }
    .to_string()
}

/// `Device.cs` imu_types names.
pub fn imu_devtype(t: u32) -> String {
    match t {
        0x09 => "DEVTYPE_BMI160",
        0x10 => "DEVTYPE_L3G4200D",
        0x11 => "DEVTYPE_ACC_LSM303D",
        0x12 => "DEVTYPE_ACC_BMA180",
        0x13 => "DEVTYPE_ACC_MPU6000",
        0x16 => "DEVTYPE_ACC_MPU9250",
        0x17 => "DEVTYPE_ACC_IIS328DQ",
        0x18 => "DEVTYPE_ACC_LSM9DS1",
        0x21 => "DEVTYPE_GYR_MPU6000",
        0x22 => "DEVTYPE_GYR_L3GD20",
        0x24 => "DEVTYPE_GYR_MPU9250",
        0x25 => "DEVTYPE_GYR_I3G4250D",
        0x26 => "DEVTYPE_GYR_LSM9DS1",
        0x27 => "DEVTYPE_INS_ICM20789",
        0x28 => "DEVTYPE_INS_ICM20689",
        0x29 => "DEVTYPE_INS_BMI055",
        0x2A => "DEVTYPE_SITL",
        0x2B => "DEVTYPE_INS_BMI088",
        0x2C => "DEVTYPE_INS_ICM20948",
        0x2D => "DEVTYPE_INS_ICM20648",
        0x2E => "DEVTYPE_INS_ICM20649",
        0x2F => "DEVTYPE_INS_ICM20602",
        0x30 => "DEVTYPE_INS_ICM20601",
        0x31 => "DEVTYPE_INS_ADIS1647X",
        0x32 => "DEVTYPE_SERIAL",
        0x33 => "DEVTYPE_INS_ICM40609",
        0x34 => "DEVTYPE_INS_ICM42688",
        0x35 => "DEVTYPE_INS_ICM42605",
        0x36 => "DEVTYPE_INS_ICM40605",
        0x37 => "DEVTYPE_INS_IIM42652",
        0x38 => "DEVTYPE_BMI270",
        0x39 => "DEVTYPE_INS_BMI085",
        0x3A => "DEVTYPE_INS_ICM42670",
        0x3B => "DEVTYPE_INS_ICM45686",
        0x3C => "DEVTYPE_INS_SCHA63T",
        0x3D => "DEVTYPE_INS_IIM42653",
        0x3E => "DEVTYPE_INS_LSM6DSV",
        0x3F => "DEVTYPE_INS_ASM330",
        _ => return format!("0x{t:02X}"),
    }
    .to_string()
}

/// `Device.cs` baro_types names.
pub fn baro_devtype(t: u32) -> String {
    match t {
        0x01 => "DEVTYPE_BARO_SITL",
        0x02 => "DEVTYPE_BARO_BMP085",
        0x03 => "DEVTYPE_BARO_BMP280",
        0x04 => "DEVTYPE_BARO_BMP388",
        0x05 => "DEVTYPE_BARO_DPS280",
        0x06 => "DEVTYPE_BARO_DPS310",
        0x07 => "DEVTYPE_BARO_FBM320",
        0x08 => "DEVTYPE_BARO_ICM20789",
        0x09 => "DEVTYPE_BARO_KELLERLD",
        0x0A => "DEVTYPE_BARO_LPS2XH",
        0x0B => "DEVTYPE_BARO_MS5611",
        0x0C => "DEVTYPE_BARO_SPL06",
        0x0D => "DEVTYPE_BARO_DRONECAN",
        0x0E => "DEVTYPE_BARO_MSP",
        0x0F => "DEVTYPE_BARO_ICP101XX",
        0x10 => "DEVTYPE_BARO_ICP201XX",
        0x11 => "DEVTYPE_BARO_MS5607",
        0x12 => "DEVTYPE_BARO_MS5837_30BA",
        0x13 => "DEVTYPE_BARO_MS5637",
        0x14 => "DEVTYPE_BARO_BMP390",
        0x15 => "DEVTYPE_BARO_BMP581",
        0x16 => "DEVTYPE_BARO_SPA06",
        0x17 => "DEVTYPE_BARO_AUAV",
        0x18 => "DEVTYPE_BARO_MS5837_02BA",
        _ => return format!("0x{t:02X}"),
    }
    .to_string()
}

/// `Device.cs` airspeed_types names.
pub fn airspeed_devtype(t: u32) -> String {
    match t {
        0x01 => "DEVTYPE_AIRSPEED_SITL",
        0x02 => "DEVTYPE_AIRSPEED_MS4525",
        0x03 => "DEVTYPE_AIRSPEED_MS5525",
        0x04 => "DEVTYPE_AIRSPEED_DLVR",
        0x05 => "DEVTYPE_AIRSPEED_MSP",
        0x06 => "DEVTYPE_AIRSPEED_SDP3X",
        0x07 => "DEVTYPE_AIRSPEED_DRONECAN",
        0x08 => "DEVTYPE_AIRSPEED_ANALOG",
        0x09 => "DEVTYPE_AIRSPEED_NMEA",
        0x0A => "DEVTYPE_AIRSPEED_ASP5033",
        _ => return format!("0x{t:02X}"),
    }
    .to_string()
}

/// DevType name for a *_ID / *_DEVID parameter, picked by the sensor family in its name.
pub fn devtype_for(param: &str, t: u32) -> String {
    if param.starts_with("COMPASS") {
        compass_devtype(t)
    } else if param.starts_with("INS_") {
        imu_devtype(t)
    } else if param.starts_with("BARO") {
        baro_devtype(t)
    } else if param.starts_with("ARSPD") {
        airspeed_devtype(t)
    } else {
        format!("0x{t:02X}")
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn decode_sitl_compass() {
        // 97539 = SITL compass on bus 0 address 0x7d? -> verify field split
        let d = super::decode((0x0F << 16) | (0x1e << 8) | (1 << 3) | 1);
        assert_eq!(d.bus_type, "BUS_TYPE_I2C");
        assert_eq!(d.bus, 1);
        assert_eq!(d.address, 0x1e);
        assert_eq!(super::compass_devtype(d.devtype), "DEVTYPE_SITL");
    }
}
